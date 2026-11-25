#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

/* ====================================================================
 * CẤU HÌNH CHÂN & GIAO TIẾP (ESP32-S3)
 * ==================================================================== */
#define PMS_UART_PORT       UART_NUM_1
#define PMS_RX_PIN          18      
#define PMS_TX_PIN          17      
#define PMS_BAUD_RATE       9600
#define PMS_BUF_SIZE        256
#define PMS_FRAME_LEN       32

#define I2C_MASTER_PORT     I2C_NUM_0
#define I2C_MASTER_SDA_IO   8       
#define I2C_MASTER_SCL_IO   9       
#define BME688_ADDR_1       0x76
#define BME688_ADDR_2       0x77

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_bme_dev = NULL;

/* ====================================================================
 * CẤU TRÚC DỮ LIỆU & HIỆU CHUẨN
 * ==================================================================== */
typedef struct {
    uint16_t pm1_0;
    uint16_t pm2_5;
    uint16_t pm10;
} pms5003_data_t;

typedef struct {
    float temperature;      // °C
    float humidity;         // %
    float pressure;         // hPa
    float gas_resistance;   // Ohm
    bool gas_valid;
} bme688_data_t;

typedef struct {
    uint16_t par_t1; int16_t par_t2; int8_t par_t3;
    uint16_t par_p1; int16_t par_p2; int8_t par_p3; int16_t par_p4;
    int16_t par_p5;  int8_t par_p6;  int8_t par_p7; int16_t par_p8;
    int16_t par_p9;  uint8_t par_p10;
    uint16_t par_h1; uint16_t par_h2; int8_t par_h3; int8_t par_h4;
    int8_t par_h5;   uint8_t par_h6;  int8_t par_h7;
    int8_t par_gh1;  int16_t par_gh2; int8_t par_gh3;
    uint8_t res_heat_range; int8_t res_heat_val; int8_t range_sw_err;
} bme688_calib_t;

static bme688_calib_t s_calib;
static bool s_bme_ready = false;
static float s_last_temp = 25.0f;
static uint8_t s_gas_variant = 1; // 1: BME688 (GAS_HIGH), 0: BME680 (GAS_LOW)

/* ====================================================================
 * HÀM PHỤ TRỢ I2C & TÍNH TOÁN BME688
 * ==================================================================== */
static esp_err_t bme_i2c_read(uint8_t reg, uint8_t *data, size_t len) {
    if (!s_bme_dev) return ESP_ERR_INVALID_STATE;
    return i2c_master_transmit_receive(s_bme_dev, &reg, 1, data, len, 500);
}

static esp_err_t bme_i2c_write(uint8_t reg, uint8_t val) {
    if (!s_bme_dev) return ESP_ERR_INVALID_STATE;
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_bme_dev, buf, 2, 500);
}

static uint8_t calc_heater_res(uint16_t target_temp, int amb_temp) {
    if (target_temp > 400) target_temp = 400;
    int32_t var1 = (((int32_t)amb_temp * s_calib.par_gh3) / 1000) * 256;
    int32_t var2 = ((int32_t)s_calib.par_gh1 + 784) *
                   (((((int32_t)s_calib.par_gh2 + 154009) * (int32_t)target_temp * 5) / 100 + 3276800) / 10);
    int32_t var3 = var1 + (var2 >> 1);
    int32_t var4 = var3 / ((int32_t)s_calib.res_heat_range + 4);
    int32_t var5 = (131 * (int32_t)s_calib.res_heat_val) + 65536;
    int32_t res_heat_x100 = ((var4 / var5) - 250) * 34;
    return (uint8_t)((res_heat_x100 + 50) / 100);
}

static float calc_gas_resistance(uint16_t gas_adc, uint8_t gas_range) {
    if (s_gas_variant == 1) {
        float var1 = (float)(262144 >> gas_range);
        float var2 = ((float)gas_adc - 512.0f) * 3.0f + 4096.0f;
        return (var2 <= 0.0f) ? 0.0f : (1000000.0f * var1) / var2;
    }
    static const uint32_t lut1[16] = {
        2147483647u, 2147483647u, 2147483647u, 2147483647u, 2147483647u, 2126008810u, 2147483647u, 2130303777u,
        2147483647u, 2147483647u, 2143188679u, 2136746228u, 2147483647u, 2126008810u, 2147483647u, 2147483647u
    };
    static const uint32_t lut2[16] = {
        4096000000u, 2048000000u, 1024000000u, 512000000u, 255744255u, 127110228u, 64000000u, 32258064u,
        16016016u,   8000000u,    4000000u,    2000000u,   1000000u,   500000u,    250000u,   125000u
    };
    uint64_t var1 = (uint64_t)(((int64_t)1340 + (int64_t)s_calib.range_sw_err * 5) * (int64_t)lut1[gas_range]) >> 16;
    int64_t var2 = ((int64_t)gas_adc << 15) - 16777216 + (int64_t)var1;
    uint64_t var3 = (lut2[gas_range] * var1) >> 9;
    return (var2 <= 0) ? 0.0f : (float)((var3 + ((uint64_t)var2 >> 1)) / (uint64_t)var2);
}

static bool bme688_init_sensor(void) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME688_ADDR_2, // Thử 0x77 trước
        .scl_speed_hz = 100000,
    };
    if (s_bme_dev) {
        i2c_master_bus_rm_device(s_bme_dev);
        s_bme_dev = NULL;
    }
    uint8_t chip_id = 0;
    if (i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_bme_dev) != ESP_OK ||
        bme_i2c_read(0xD0, &chip_id, 1) != ESP_OK || chip_id != 0x61) {
        if (s_bme_dev) i2c_master_bus_rm_device(s_bme_dev);
        dev_cfg.device_address = BME688_ADDR_1; // Thử sang 0x76
        if (i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_bme_dev) != ESP_OK ||
            bme_i2c_read(0xD0, &chip_id, 1) != ESP_OK || chip_id != 0x61) {
            return false;
        }
    }

    bme_i2c_write(0xE0, 0xB6); // Soft reset
    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t c1[24], c2[14], c3[5];
    if (bme_i2c_read(0x8A, c1, 24) != ESP_OK ||
        bme_i2c_read(0xE1, c2, 14) != ESP_OK ||
        bme_i2c_read(0x00, c3, 5)  != ESP_OK) return false;

    s_calib.par_t2  = (int16_t)((c1[1] << 8) | c1[0]);
    s_calib.par_t3  = (int8_t)c1[2];
    s_calib.par_p1  = (uint16_t)((c1[5] << 8) | c1[4]);
    s_calib.par_p2  = (int16_t)((c1[7] << 8) | c1[6]);
    s_calib.par_p3  = (int8_t)c1[8];
    s_calib.par_p4  = (int16_t)((c1[11] << 8) | c1[10]);
    s_calib.par_p5  = (int16_t)((c1[13] << 8) | c1[12]);
    s_calib.par_p7  = (int8_t)c1[14];
    s_calib.par_p6  = (int8_t)c1[15];
    s_calib.par_p8  = (int16_t)((c1[19] << 8) | c1[18]);
    s_calib.par_p9  = (int16_t)((c1[21] << 8) | c1[20]);
    s_calib.par_p10 = (uint8_t)c1[22];

    s_calib.par_h2  = (uint16_t)(((uint16_t)c2[0] << 4) | (c2[1] >> 4));
    s_calib.par_h1  = (uint16_t)(((uint16_t)c2[2] << 4) | (c2[1] & 0x0F));
    s_calib.par_h3  = (int8_t)c2[3];
    s_calib.par_h4  = (int8_t)c2[4];
    s_calib.par_h5  = (int8_t)c2[5];
    s_calib.par_h6  = (uint8_t)c2[6];
    s_calib.par_h7  = (int8_t)c2[7];
    s_calib.par_t1  = (uint16_t)((c2[9] << 8) | c2[8]);
    s_calib.par_gh2 = (int16_t)((c2[11] << 8) | c2[10]);
    s_calib.par_gh1 = (int8_t)c2[12];
    s_calib.par_gh3 = (int8_t)c2[13];

    s_calib.res_heat_range = (uint8_t)((c3[2] & 0x30) >> 4);
    s_calib.res_heat_val   = (int8_t)c3[0];
    s_calib.range_sw_err   = (int8_t)((int8_t)(c3[4] & 0xF0) >> 4);

    bme_i2c_write(0x72, 0x01); // Bật đo độ ẩm
    bme_i2c_write(0x75, 0x08); // Bộ lọc IIR
    bme_i2c_read(0xF0, &s_gas_variant, 1);

    bme_i2c_write(0x5A, calc_heater_res(320, 25)); // Đốt ở 320°C
    bme_i2c_write(0x64, 0x59);                     // Thời gian đốt 100ms
    bme_i2c_write(0x70, 0x00);
    bme_i2c_write(0x71, (s_gas_variant == 1) ? 0x20 : 0x10);

    s_bme_ready = true;
    return true;
}

/* ====================================================================
 * HÀM GỘP: ĐỌC PMS5003 + ĐỌC BME688 + XUẤT JSON RA SERIAL PORT
 * ==================================================================== */
void read_and_print_sensors(void) {
    pms5003_data_t pms = {0};
    bme688_data_t bme = {0};
    bool pms_ok = false;
    bool bme_ok = false;

    /* --- 1. ĐỌC CẢM BIẾN BỤI PMS5003 (UART) --- */
    static uint8_t buf[PMS_BUF_SIZE * 2];
    size_t avail = 0;
    uart_get_buffered_data_len(PMS_UART_PORT, &avail);

    int len = 0;
    if (avail >= PMS_FRAME_LEN) {
        if (avail > sizeof(buf)) avail = sizeof(buf);
        len = uart_read_bytes(PMS_UART_PORT, buf, avail, pdMS_TO_TICKS(20));
    } else {
        len = uart_read_bytes(PMS_UART_PORT, buf, 2 * PMS_FRAME_LEN, pdMS_TO_TICKS(1200));
    }

    if (len >= PMS_FRAME_LEN) {
        for (int i = len - PMS_FRAME_LEN; i >= 0; i--) {
            if (buf[i] == 0x42 && buf[i + 1] == 0x4D &&
                (((uint16_t)buf[i + 2] << 8) | buf[i + 3]) == 28) {
                uint16_t sum = 0;
                for (int j = 0; j < 30; j++) sum += buf[i + j];
                uint16_t rec_sum = ((uint16_t)buf[i + 30] << 8) | buf[i + 31];

                if (sum == rec_sum) {
                    pms.pm1_0 = ((uint16_t)buf[i + 10] << 8) | buf[i + 11];
                    pms.pm2_5 = ((uint16_t)buf[i + 12] << 8) | buf[i + 13];
                    pms.pm10  = ((uint16_t)buf[i + 14] << 8) | buf[i + 15];
                    pms_ok = true;
                    break;
                }
            }
        }
    }

    /* --- 2. ĐỌC CẢM BIẾN MÔI TRƯỜNG & GAS BME688 (I2C) --- */
    if (s_bme_ready || bme688_init_sensor()) {
        bme_i2c_write(0x5A, calc_heater_res(320, (int)s_last_temp));
        if (bme_i2c_write(0x74, 0x25) == ESP_OK) {
            uint8_t status = 0;
            bool ready = false;
            for (int i = 0; i < 50; i++) {
                if (bme_i2c_read(0x1D, &status, 1) == ESP_OK && (status & 0x80)) {
                    ready = true;
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            uint8_t d[15];
            if (ready && bme_i2c_read(0x1F, d, sizeof(d)) == ESP_OK) {
                uint32_t pres_r = ((uint32_t)d[0] << 12) | ((uint32_t)d[1] << 4) | (d[2] >> 4);
                uint32_t temp_r = ((uint32_t)d[3] << 12) | ((uint32_t)d[4] << 4) | (d[5] >> 4);
                uint16_t hum_r  = ((uint16_t)d[6] << 8)  | d[7];

                // Nhiệt độ (°C)
                float var1 = (((float)temp_r / 16384.0f) - ((float)s_calib.par_t1 / 1024.0f)) * (float)s_calib.par_t2;
                float var2 = ((((float)temp_r / 131072.0f) - ((float)s_calib.par_t1 / 8192.0f)) *
                              (((float)temp_r / 131072.0f) - ((float)s_calib.par_t1 / 8192.0f))) * ((float)s_calib.par_t3 * 16.0f);
                float t_fine = var1 + var2;
                bme.temperature = t_fine / 5120.0f;
                s_last_temp = bme.temperature;

                // Áp suất (hPa)
                var1 = (t_fine / 2.0f) - 64000.0f;
                var2 = var1 * var1 * ((float)s_calib.par_p6 / 131072.0f);
                var2 = var2 + (var1 * (float)s_calib.par_p5 * 2.0f);
                var2 = (var2 / 4.0f) + ((float)s_calib.par_p4 * 65536.0f);
                var1 = (1.0f + (((((float)s_calib.par_p3 * var1 * var1) / 16384.0f) + ((float)s_calib.par_p2 * var1)) / 524288.0f) / 32768.0f) * (float)s_calib.par_p1;
                if ((int32_t)var1 != 0) {
                    float calc_pres = 1048576.0f - (float)pres_r;
                    calc_pres = ((calc_pres - (var2 / 4096.0f)) * 6250.0f) / var1;
                    float v1 = ((float)s_calib.par_p9 * calc_pres * calc_pres) / 2147483648.0f;
                    float v2 = calc_pres * ((float)s_calib.par_p8 / 32768.0f);
                    float v3 = (calc_pres / 256.0f) * (calc_pres / 256.0f) * (calc_pres / 256.0f) * ((float)s_calib.par_p10 / 131072.0f);
                    float hpa = (calc_pres + (v1 + v2 + v3 + ((float)s_calib.par_p7 * 128.0f)) / 16.0f) / 100.0f;
                    bme.pressure = (hpa < 300.0f || hpa > 1200.0f) ? 1013.25f : hpa;
                } else {
                    bme.pressure = 1013.25f;
                }

                // Độ ẩm (%)
                float tc = bme.temperature;
                var1 = (float)hum_r - (((float)s_calib.par_h1 * 16.0f) + (((float)s_calib.par_h3 / 2.0f) * tc));
                var2 = var1 * (((float)s_calib.par_h2 / 262144.0f) * (1.0f + (((float)s_calib.par_h4 / 16384.0f) * tc) + (((float)s_calib.par_h5 / 1048576.0f) * tc * tc)));
                float h_comp = var2 + (((float)s_calib.par_h6 / 16384.0f + ((float)s_calib.par_h7 / 2097152.0f * tc)) * var2 * var2);
                bme.humidity = (h_comp < 0.0f) ? 0.0f : (h_comp > 100.0f ? 100.0f : h_comp);

                // Điện trở Gas (Ohm)
                const uint8_t gi = (s_gas_variant == 1) ? 13 : 11;
                uint16_t gas_adc = (uint16_t)(((uint16_t)d[gi] << 2) | (d[gi + 1] >> 6));
                uint8_t gas_range = (uint8_t)(d[gi + 1] & 0x0F);
                if ((d[gi + 1] & 0x30) == 0x30) {
                    bme.gas_resistance = calc_gas_resistance(gas_adc, gas_range);
                    bme.gas_valid = (bme.gas_resistance > 0.0f);
                }

                bme_ok = true;
            } else {
                s_bme_ready = false;
            }
        } else {
            s_bme_ready = false;
        }
    }

    /* --- 3. IN CHUỖI JSON 1 DÒNG RA SERIAL PORT --- */
    printf("{\"bme_ok\":%s,\"temp\":%.2f,\"hum\":%.2f,\"pres\":%.2f,"
           "\"gas_ok\":%s,\"gas_ohm\":%.0f,"
           "\"pms_ok\":%s,\"pm1\":%u,\"pm2_5\":%u,\"pm10\":%u}\n",
           bme_ok ? "true" : "false", bme.temperature, bme.humidity, bme.pressure,
           bme.gas_valid ? "true" : "false", bme.gas_resistance,
           pms_ok ? "true" : "false", pms.pm1_0, pms.pm2_5, pms.pm10);
    fflush(stdout);
}

/* ====================================================================
 * KHỞI TẠO PHẦN CỨNG & CHẠY VÒNG LẶP CHÍNH
 * ==================================================================== */
void app_main(void) {
    // Tắt log hệ thống thông thường để Serial chỉ xuất dữ liệu JSON sạch
    esp_log_level_set("*", ESP_LOG_ERROR);

    // Khởi tạo I2C cho BME688
    i2c_master_bus_config_t i2c_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_PORT,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_cfg, &s_i2c_bus));

    // Khởi tạo UART cho PMS5003
    uart_config_t uart_cfg = {
        .baud_rate = PMS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(PMS_UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(PMS_UART_PORT, PMS_TX_PIN, PMS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(gpio_set_pull_mode(PMS_RX_PIN, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(uart_driver_install(PMS_UART_PORT, PMS_BUF_SIZE * 2, 0, 0, NULL, 0));

    bme688_init_sensor();

    // Vòng lặp chu kỳ chuẩn 2 giây
    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        read_and_print_sensors();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2000));
    }
}