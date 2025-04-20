#include <stdio.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/i2c_master.h"

#define I2C_PORT 0

#define COMMON_SDA_PIN 22
#define COMMON_SCL_PIN 23

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

static void taking_measurements(void * pvParameters) {
    esp_err_t ret;
    uint16_t TriesCount = 10;
    TriesCount = 100;

    uint16_t co2;
    float temperature, humidity;
    vTaskDelay(pdMS_TO_TICKS(30000));

    uint8_t* buff_wr = communication_buffer;
    uint16_t local_offset = 0;
    uint8_t buff_r[3] = {0};  // Output: readiness
    int sleep_ms = (1 * 1000);  // Send cmd and wait 30 ms
    bool dataReady;
    uint16_t data_ready_status = 0;


    local_offset = sensirion_i2c_add_command16_to_buffer(buff_wr, local_offset, 0xe4b8);
    while (1) {
        ret = i2c_master_transmit_receive(scd41_handle, buff_wr, sizeof(buff_wr), buff_r, sizeof(buff_r), sleep_ms);
        data_ready_status = sensirion_common_bytes_to_uint16_t(&buff_wr[0]);
        dataReady = (data_ready_status & 2047) != 0;

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Cannot get Data ready status! Retry: %d", TriesCount);
            vTaskDelay(pdMS_TO_TICKS(5000));
            TriesCount--;
            if (TriesCount == 0)
                break;
        } else {
            ESP_LOGI(TAG, "Data ready %d status: %d", data_ready_status, dataReady);
            break;
        }
    }

}

void app_main(void)
{
    esp_err_t ret;
    uint16_t TriesCount = 10;

    // New I2C bus setup, new driver used, from IDF 5.4+
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = COMMON_SCL_PIN,
        .sda_io_num = COMMON_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ret = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot init master bus!");
        while (1);
    } else {
        ESP_LOGI(TAG, "Master bus added!");
    }

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
    } else {
        ESP_LOGI(TAG, "Temperature sensor device added!");
    }

    ESP_LOGI(TAG, "All devices added! Start communication");

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
    local_offset = 0; // Reset offset
    local_offset = sensirion_i2c_add_command16_to_buffer(buff_wr, local_offset, 0x3682);
    TriesCount = 10;
    // Send and receive after a short wait
    uint8_t buff_r[3] = {0};  // Output: serial number
    sleep_ms = 1 * 1000;
    while (1) {
        ret = i2c_master_transmit_receive(scd41_handle, buff_wr, sizeof(buff_wr), buff_r, sizeof(buff_r), sleep_ms);
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
    sensirion_common_copy_bytes(&buff_r[0], (uint8_t*)buff_serial, (sizeof(buff_serial) * 2));
    // Sensiniron
    ESP_LOGI(TAG, "Sensor serial number is: 0x%x 0x%x 0x%x", (int)buff_serial[0], (int)buff_serial[1], (int)buff_serial[2]);

    // Start measurement
    TriesCount = 10;
    local_offset = 0; // Reset offset
    local_offset = sensirion_i2c_add_command16_to_buffer(buff_wr, local_offset, 0x21b1);
    ret = i2c_master_transmit(scd41_handle, buff_wr, local_offset, -1);
    while (1) {
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Cannot start CO2 sensor measurements! Retry: %d", TriesCount);
            vTaskDelay(pdMS_TO_TICKS(5000));
            TriesCount--;
            if (TriesCount == 0)
                break;
        } else {
            ESP_LOGI(TAG, "CMD Start measurements sent! Get measumenets in 5 sec intervals");
            break;
        }
    }

    // Consume measurements with 5 sec interval!
    TriesCount = 10;
    local_offset = 0; // Reset offset
    sleep_ms = (1 * 1000);  // Send cmd and wait 30 ms
    bool dataReady;
    uint16_t data_ready_status = 0;
    local_offset = sensirion_i2c_add_command16_to_buffer(buff_wr, local_offset, 0xe4b8);
    ret = i2c_master_transmit_receive(scd41_handle, buff_wr, sizeof(buff_wr), buff_r, sizeof(buff_r), sleep_ms);
    while (1) {
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
            break;
        }
    }

    // xTaskCreatePinnedToCore(taking_measurements, "CO2 measure task", 8192, NULL, 9, NULL, tskNO_AFFINITY);

}