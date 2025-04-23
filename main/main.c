#include <stdio.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/i2c_master.h"

#define I2C_PORT 0

#define COMMON_SDA_PIN 0
#define COMMON_SCL_PIN 1

#define SCD4X_I2C_ADDR 0x62     // this
#define BME680_I2C_ADDR_0 0x76
#define BME680_I2C_ADDR_1 0x77  // this

#define I2C_FREQ_HZ 100000 // 100kHz - usual

#define SPIN_ITER   350000  //actual CPU cycles consumed will depend on compiler optimization
#define CORE0       0
// only define xCoreID CORE1 as 1 if this is a multiple core processor target, else define it as tskNO_AFFINITY
#define CORE1       ((CONFIG_FREERTOS_NUMBER_OF_CORES > 1) ? 1 : tskNO_AFFINITY)

static const char *TAG = "i2c_master";

static uint8_t communication_buffer[9] = {0};

#define CMD_WAKE_UP                                (0x36F6)

// Main BUS
i2c_master_bus_handle_t bus_handle;
// Devices handles after they were added to main BUS
i2c_master_dev_handle_t scd41_handle;
i2c_master_dev_handle_t bme680_handle;


// Copied from Sensiniron: https://github.com/Sensirion/embedded-i2c-scd4x/blob/455a41c6b7a7a86a55d6647f5fc22d8574572b7b/sensirion_i2c.c#L180
uint16_t sensirion_i2c_add_command16_to_buffer(uint8_t* buffer, uint16_t offset, uint16_t command) {
    buffer[offset++] = (uint8_t)((command & 0xFF00) >> 8);
    buffer[offset++] = (uint8_t)((command & 0x00FF) >> 0);
    return offset;
}

void sensirion_common_copy_bytes(const uint8_t* source, uint8_t* destination, uint16_t data_length) {
    uint16_t i;
    for (i = 0; i < data_length; i++) {
        destination[i] = source[i];
    }
}

uint16_t sensirion_common_bytes_to_uint16_t(const uint8_t* bytes) {
    return (uint16_t)bytes[0] << 8 | (uint16_t)bytes[1];
}

void co2_sensor_tst(void) {
    esp_err_t ret;
    uint16_t TriesCount = 10;

    // Add CO2 sensor first
    i2c_device_config_t scd41_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SCD4X_I2C_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    // Add CO2 first
    ret = i2c_master_bus_add_device(bus_handle, &scd41_cfg, &scd41_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot add CO2 sensor!");
        while (1);
    } else {
        ESP_LOGI(TAG, "CO2 sensor device added!");
    }

    // Stop sensor!
    TriesCount = 100;
    uint8_t* buff_wr = communication_buffer;
    uint16_t local_offset = 0;
    local_offset = sensirion_i2c_add_command16_to_buffer(buff_wr, local_offset, 0x3f86);
    while (1) {
        ret = i2c_master_transmit(scd41_handle, buff_wr, local_offset, 30);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Cannot stop sensor measurements now. Retry: %d", TriesCount);
            vTaskDelay(pdMS_TO_TICKS(5000));
            TriesCount--;
            if (TriesCount == 0)
                break;
        } else {
            ESP_LOGI(TAG, "CMD Stop Measurements sent at start! Wait 5 sec!");
            vTaskDelay(pdMS_TO_TICKS(5000));
            break;
        }
    }

    // Communicate with CO2
    TriesCount = 100;
    local_offset = 0;
    int sleep_ms = 30;  // Send cmd and wait 30 ms
    local_offset = sensirion_i2c_add_command16_to_buffer(buff_wr, local_offset, 0x36f6);
    while (1) {
        ret = i2c_master_transmit(scd41_handle, buff_wr, local_offset, 30);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Cannot wake up CO2 sensor! Retry: %d", TriesCount);
            vTaskDelay(pdMS_TO_TICKS(5000));
            TriesCount--;
            if (TriesCount == 0)
                break;
        } else {
            ESP_LOGI(TAG, "CMD Wake Up sent!");
            break;
        }
    }
    
    // Read serial number
    uint8_t* buff_wr_serial = communication_buffer;
    local_offset = 0; // Reset offset
    local_offset = sensirion_i2c_add_command16_to_buffer(buff_wr_serial, local_offset, 0x3682);
    TriesCount = 10;
    // Send and receive after a short wait
    uint8_t buff_r_serial[3] = {0};  // Output: serial number
    sleep_ms = 1 * 1000;
    while (1) {
        ret = i2c_master_transmit_receive(scd41_handle, buff_wr_serial, sizeof(buff_wr_serial), buff_r_serial, sizeof(buff_r_serial), sleep_ms);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Cannot get serial number of SO2 sensor! Retry: %d", TriesCount);
            vTaskDelay(pdMS_TO_TICKS(5000));
            TriesCount--;
            if (TriesCount == 0)
                break;
        } else {
            ESP_LOGI(TAG, "CMD Serial sent!");
            break;
        }
    }
    // Transform received:
    uint16_t buff_serial[3] = {0};
    sensirion_common_copy_bytes(&buff_r_serial[0], (uint8_t*)buff_serial, (sizeof(buff_serial) * 2));
    // Sensiniron
    ESP_LOGI(TAG, "Sensor serial number is: 0x%x 0x%x 0x%x", (int)buff_serial[0], (int)buff_serial[1], (int)buff_serial[2]);

    // Start measurement
    TriesCount = 10;
    local_offset = 0; // Reset offset
    local_offset = sensirion_i2c_add_command16_to_buffer(buff_wr, local_offset, 0x21b1);
    while (1) {
        ret = i2c_master_transmit(scd41_handle, buff_wr, local_offset, -1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Cannot start CO2 sensor measurements! Retry: %d", TriesCount);
            vTaskDelay(pdMS_TO_TICKS(5000));
            TriesCount--;
            if (TriesCount == 0)
                break;
        } else {
            ESP_LOGI(TAG, "CMD Start measurements sent! Get measumenets in 5 sec intervals");
            vTaskDelay(pdMS_TO_TICKS(5000));
            break;
        }
    }

    // Consume measurements with 5 sec interval!
    TriesCount = 10;
    local_offset = 0; // Reset offset
    uint8_t buff_r[3] = {0};  // Output: serial number
    sleep_ms = (1 * 1000);  // Send cmd and wait 1 sec
    bool dataReady;
    uint16_t data_ready_status = 0;
    local_offset = sensirion_i2c_add_command16_to_buffer(buff_wr, local_offset, 0xe4b8);
    while (1) {
        ret = i2c_master_transmit_receive(scd41_handle, buff_wr, sizeof(buff_wr), buff_r, sizeof(buff_r), sleep_ms);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Cannot get Data ready status! Retry: %d", TriesCount);
            vTaskDelay(pdMS_TO_TICKS(5000));
            TriesCount--;
            if (TriesCount == 0)
                break;
        } else {
            data_ready_status = sensirion_common_bytes_to_uint16_t(&buff_wr[0]);
            dataReady = (data_ready_status & 2047) != 0;
            ESP_LOGI(TAG, "Data ready %d status: %d", data_ready_status, dataReady);
            vTaskDelay(pdMS_TO_TICKS(1000));
            break;
        }
    }

    // Get measurements
    TriesCount = 30;
    uint16_t MeasuresCount = 10;
    uint16_t co2Raw;         // ppm
    int32_t temperatureRaw;  // millicelsius
    int32_t humidityRaw;     // millipercent
    // 
    uint8_t* buff_wr_measurements = communication_buffer;
    uint8_t* buff_r_measurements = communication_buffer;

    uint16_t co2;
    float temperature, humidity;
    // Dumb loop to collect a few measurements
    while (1) {
        local_offset = 0; // Reset offset
        sleep_ms = (1 * 1000);  // Send cmd and wait 1 sec
        local_offset = sensirion_i2c_add_command16_to_buffer(buff_wr_measurements, local_offset, 0xec05);
        MeasuresCount--;
        if (MeasuresCount == 0) {
            ESP_LOGI(TAG, "Stop measurements.");
            break;
        }
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            ret = i2c_master_transmit_receive(scd41_handle, buff_wr_measurements, sizeof(buff_wr_measurements), buff_r_measurements, sizeof(buff_r_measurements), sleep_ms);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Cannot get measurements! Retry: %d", TriesCount);
                vTaskDelay(pdMS_TO_TICKS(5000));
                TriesCount--;
                if (TriesCount == 0) {
                    break;
                }
            } else {
                co2Raw = sensirion_common_bytes_to_uint16_t(&buff_r_measurements[0]);
                temperatureRaw = sensirion_common_bytes_to_uint16_t(&buff_r_measurements[2]);
                humidityRaw = sensirion_common_bytes_to_uint16_t(&buff_r_measurements[4]);
                ESP_LOGI(TAG, "RAW Measurements ready co2: %d, t: %ld C Humidity: %ld (raw value)", co2Raw, temperatureRaw, humidityRaw);
                
                co2 = co2Raw;
                temperature = (float)temperatureRaw * 175.0f / 65536.0f - 45.0f;
                humidity = (float)humidityRaw * 100.0f / 65536.0f;
                ESP_LOGI(TAG, "Measurements ready co2: %u ppm, t: %.2f °C Humidity: %.2f %%", co2, temperature, humidity);

                // uint16_t temperature;
                // uint16_t humidity;
                // temperatureRaw = ((21875 * (int32_t)temperature) >> 13) - 45000;
                // humidityRaw = ((12500 * (int32_t)humidity) >> 13);
                // const float humidityPercent = humidityRaw / 1000.0f;
                // ESP_LOGI(TAG, "Converted measurements co2: %d, t: %.1f C Humidity: %.1f%%", co2Raw, temperatureRaw, humidityPercent);
                break;
            }
        }
    }
}

void bme650_tst(void) {
    esp_err_t ret;
    uint16_t TriesCount = 0;
    TriesCount = 10;  // Unused is now used
    
    ESP_LOGI(TAG, "Adding BME680...");
    
    // Configure BMD680
    i2c_device_config_t bme680_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME680_I2C_ADDR_1,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    // Add BME680 device second
    ret = i2c_master_bus_add_device(bus_handle, &bme680_cfg, &bme680_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot add temperature sensor!");
        while (1);
    }
    
    ESP_LOGI(TAG, "Temperature sensor device added! Wait 5 seconds.");
    vTaskDelay(pdMS_TO_TICKS(5000));

    // Get chip ID
    uint8_t BME680_REG_ID = 0xd0;
    uint8_t* buff_serial = malloc(1);
    uint8_t buff_r_serial[1] = {0};  // Output: serial number
    buff_serial[0] = BME680_REG_ID;
    ret = i2c_master_transmit_receive(bme680_handle, buff_serial, 1, buff_r_serial, 1, 50);
    if (ret != ESP_OK) {
        // I (586) i2c_master: Sensor serial number is: 0x61
        ESP_LOGI(TAG, "Sensor serial number is: 0x%x", (int)buff_r_serial[0]);
    }
    free(buff_serial);

    // Init
    TriesCount = 3;
    uint8_t* buff_wr = malloc(2);
    uint8_t BME680_REG_RESET = 0xe0;
    uint8_t BME680_RESET_CMD = 0xb6;    // BME680_REG_RESET<7:0>
    int BME680_RESET_PERIOD = 10;      // reset time in ms

    buff_wr[0] = BME680_REG_RESET;
    buff_wr[1] = BME680_RESET_CMD;

    while (1) {
        ret = i2c_master_transmit(bme680_handle, buff_wr, 2, 30);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Cannot stop sensor measurements now. Retry: %d", TriesCount);
            vTaskDelay(pdMS_TO_TICKS(5000));
            TriesCount--;
            if (TriesCount == 0)
                break;
        } else {
            ESP_LOGI(TAG, "CMD Stop Measurements sent at start!");
            vTaskDelay(pdMS_TO_TICKS(BME680_RESET_PERIOD));
            break;
        }
    }
    free(buff_wr);

}

void app_main(void)
{
    esp_err_t ret;
    // New I2C bus setup, new driver used, from IDF 5.4+
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = COMMON_SCL_PIN,
        .sda_io_num = COMMON_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ret = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot init master bus!");
        while (1);
    } else {
        ESP_LOGI(TAG, "Master bus added!");
    }

    ESP_LOGI(TAG, "I2C Bus is ready - now add devices!");

    // CO2 test
    // co2_sensor_tst();

    // Temp and humidity bosh
    bme650_tst();

}