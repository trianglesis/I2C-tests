#include <stdio.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/i2c_master.h"

#include <bme680.h>

#define I2C_PORT -1  // Auto select

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

static const char *TAG = "i2c-test-device";

static uint8_t communication_buffer[9] = {0};

#define CMD_WAKE_UP                                (0x36F6)

// Main BUS
i2c_master_bus_handle_t bus_handle;
// Devices handles after they were added to main BUS
i2c_master_dev_handle_t scd41_handle;
i2c_master_dev_handle_t bme680_handle;


// Copied from Sensiniron: https://github.com/Sensirion/embedded-i2c-scd4x/blob/455a41c6b7a7a86a55d6647f5fc22d8574572b7b/sensirion_i2c.c#L180
uint16_t sensirion_i2c_add_command_to_buffer(uint8_t* buffer, uint16_t offset,
                                             uint16_t command) {
    buffer[offset++] = (uint8_t)((command & 0xFF00) >> 8);
    buffer[offset++] = (uint8_t)((command & 0x00FF) >> 0);
    return offset;
}

uint16_t sensirion_i2c_add_command8_to_buffer(uint8_t* buffer, uint16_t offset,
                                              uint8_t command) {
    buffer[offset++] = command;
    return offset;
}

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
    TriesCount = 5;
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


static inline void print_registers(bme680_handle_t handle) {
    /* configuration registers */
    bme680_control_measurement_register_t ctrl_meas_reg;
    bme680_control_humidity_register_t    ctrl_humi_reg;
    bme680_config_register_t              config_reg;
    bme680_control_gas0_register_t        ctrl_gas0_reg;
    bme680_control_gas1_register_t        ctrl_gas1_reg;

    /* attempt to read control humidity register */
    bme680_get_control_humidity_register(handle, &ctrl_humi_reg);

    /* attempt to read control measurement register */
    bme680_get_control_measurement_register(handle, &ctrl_meas_reg);

    /* attempt to read configuration register */
    bme680_get_configuration_register(handle, &config_reg);

    /* attempt to read control gas 0 register */
    bme680_get_control_gas0_register(handle, &ctrl_gas0_reg);

    /* attempt to read control gas 1 register */
    bme680_get_control_gas1_register(handle, &ctrl_gas1_reg);

    ESP_LOGI(TAG, "Variant Id          (0x%02x): %s", handle->variant_id,uint8_to_binary(handle->variant_id));
    ESP_LOGI(TAG, "Configuration       (0x%02x): %s", config_reg.reg,    uint8_to_binary(config_reg.reg));
    ESP_LOGI(TAG, "Control Measurement (0x%02x): %s", ctrl_meas_reg.reg, uint8_to_binary(ctrl_meas_reg.reg));
    ESP_LOGI(TAG, "Control Humidity    (0x%02x): %s", ctrl_humi_reg.reg, uint8_to_binary(ctrl_humi_reg.reg));
    ESP_LOGI(TAG, "Control Gas 0       (0x%02x): %s", ctrl_gas0_reg.reg, uint8_to_binary(ctrl_gas0_reg.reg));
    ESP_LOGI(TAG, "Control Gas 1       (0x%02x): %s", ctrl_gas1_reg.reg, uint8_to_binary(ctrl_gas1_reg.reg));
}

void bme650_alt(void) {
    esp_err_t ret;

    // bus_handle is already init

    // initialize the xLastWakeTime variable with the current time.
    TickType_t          last_wake_time  = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    bme680_config_t dev_cfg         = I2C_BME680_CONFIG_DEFAULT;
    bme680_handle_t dev_hdl;
    //
    // init device
    bme680_init(bus_handle, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(TAG, "bme680 handle init failed");
        assert(dev_hdl);
    }
    
    print_registers(dev_hdl);

    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(TAG, "######################## BME680 - START #########################");
        
        // handle sensor

        esp_err_t result;
        /*
        bme680_data_t data;
        
        result = bme680_get_data(dev_hdl, &data);
        if(result != ESP_OK) {
            ESP_LOGE(TAG, "bme680 device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(TAG, "air temperature:     %.2f °C", data.air_temperature);
            ESP_LOGI(TAG, "dewpoint temperature:%.2f °C", data.dewpoint_temperature);
            ESP_LOGI(TAG, "relative humidity:   %.2f %%", data.relative_humidity);
            ESP_LOGI(TAG, "barometric pressure: %.2f hPa", data.barometric_pressure/100);
            ESP_LOGI(TAG, "gas resistance(%u):   %.2f kΩ", data.gas_index, data.gas_resistance/1000);
            ESP_LOGI(TAG, "iaq score:           %u (%s)", data.iaq_score, bme680_air_quality_to_string(data.iaq_score));
            ESP_LOGI(TAG, "heater is stable:    %s", data.heater_stable ? "yes" : "no");
            ESP_LOGI(TAG, "gas range:           %u", data.gas_range);
            ESP_LOGI(TAG, "gas valid:           %s", data.gas_valid ? "yes" : "no");
        }
        */

        //bme680_data_t data[BME680_HEATER_PROFILE_SIZE];
        //result = bme680_get_data2(dev_hdl, data);
        //if(result != ESP_OK) {
        //    ESP_LOGE(TAG, "bme680 device read failed (%s)", esp_err_to_name(result));
        //}

        ESP_LOGI(TAG, "Index Air(°C) Dew-Point(°C) Humidity(%%) Pressure(hPa) Gas-Resistance(kΩ) Gas-Range Gas-Valid Gas-Index Heater-Stable IAQ-Score");

        for(uint8_t i = 0; i < dev_hdl->dev_config.heater_profile_size; i++) {
            bme680_data_t data;
            result = bme680_get_data_by_heater_profile(dev_hdl, i, &data);
            if(result != ESP_OK) {
                ESP_LOGE(TAG, "bme680 device read failed (%s)", esp_err_to_name(result));
            }
            ESP_LOGI(TAG, "%u    %.2f    %.2f          %.2f         %.2f          %.2f               %u        %s        %u        %s            %u (%s)",
                i,
                data.air_temperature,
                data.dewpoint_temperature,
                data.relative_humidity,
                data.barometric_pressure/100,
                data.gas_resistance/1000,
                data.gas_range,
                data.gas_valid ? "yes" : "no",
                data.gas_index,
                data.heater_stable ? "yes" : "no",
                data.iaq_score, bme680_air_quality_to_string(data.iaq_score));

            /*
            ESP_LOGI(TAG, "(%u) air temperature:     %.2f °C", i, data.air_temperature);
            ESP_LOGI(TAG, "(%u) dewpoint temperature:%.2f °C", i, data.dewpoint_temperature);
            ESP_LOGI(TAG, "(%u) relative humidity:   %.2f %%", i, data.relative_humidity);
            ESP_LOGI(TAG, "(%u) barometric pressure: %.2f hPa", i, data.barometric_pressure/100);
            ESP_LOGI(TAG, "(%u) gas resistance(%u):   %.2f kΩ", i, data.gas_index, data.gas_resistance/1000);
            ESP_LOGI(TAG, "(%u) gas range(%u):        %u", i, data.gas_index, data.gas_range);
            ESP_LOGI(TAG, "(%u) gas valid(%u):        %s", i, data.gas_index, data.gas_valid ? "yes" : "no");
            ESP_LOGI(TAG, "(%u) heater is stable:    %s", i, data.heater_stable ? "yes" : "no");
            ESP_LOGI(TAG, "(%u) iaq score:           %u (%s)", i, data.iaq_score, bme680_air_quality_to_string(data.iaq_score));
            */
            vTaskDelay(pdMS_TO_TICKS(250));
        }
        
        ESP_LOGI(TAG, "######################## BME680 - END ###########################");
        //
        //
        // pause the task per defined wait period
        xTaskDelayUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    bme680_delete( dev_hdl );
    vTaskDelete( NULL );
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
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Added BME680 sensor! Now wait!");
        vTaskDelay(pdMS_TO_TICKS(5000));
    } else if (ret == ESP_ERR_INVALID_ARG) {
        ESP_LOGE(TAG, "ESP_ERR_INVALID_ARG: I2C bus initialization failed because of invalid argument.");
    } else if (ret == ESP_ERR_NO_MEM) {
        ESP_LOGE(TAG, "ESP_ERR_NO_MEM: Create I2C bus failed because of out of memory.");
    } else {
        ESP_LOGE(TAG, "Cannot add temperature sensor!");
    }
    
    // Probe does not work
    ret =  i2c_master_probe(bus_handle, BME680_I2C_ADDR_1, 50);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Probe tested BME680, OK!");
    } else if (ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGE(TAG, "ESP_ERR_NOT_FOUND: I2C probe failed, doesn't find the device with specific address you gave.");
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.");
    } else {
        ESP_LOGE(TAG, "Unexpected BME680, FAIL!");
    }

    // Get chip ID
    // uint8_t BME680_REG_ID = 0xd0;
    // uint8_t read_buffer[2] = {0};  // Output: serial number
    
    // Sensiniron way
    // uint8_t* buff_wr = communication_buffer;
    // uint16_t local_offset = 0;
    // local_offset = sensirion_i2c_add_command_to_buffer(buff_wr, local_offset, BME680_REG_ID);
    // ret = i2c_master_transmit_receive(bme680_handle, buff_wr, sizeof(buff_wr), read_buffer, sizeof(read_buffer), 50);
    
    // uint8_t comm_buffer[1] = {0};
    // comm_buffer[0] = BME680_REG_ID;
    // ret = i2c_master_transmit_receive(bme680_handle, comm_buffer, 1, read_buffer, 1, -1);
    // if (ret != ESP_OK) {
    //     // I (586) i2c_master: Sensor serial number is: 0x61
    //     ESP_LOGI(TAG, "Transmit-receive failed");
    // }

    // Other way
    // i2c_master_transmit(bme680_handle, comm_buffer, 1, -1);
    // ESP_LOGI(TAG, "Sensor serial register sent! Wait and receive back the ID");
    // // vTaskDelay(pdMS_TO_TICKS(5));
    // i2c_master_receive(bme680_handle, read_buffer, 1, -1);

    // 2nd other way
    // i2c_operation_job_t i2c_ops1[] = {
    //     { .command = I2C_MASTER_CMD_START },
    //     { .command = I2C_MASTER_CMD_WRITE, .write = { .ack_check = false, .data = (uint8_t *) &BME680_REG_ID, .total_bytes = 1 } },
    //     { .command = I2C_MASTER_CMD_START },
    //     { .command = I2C_MASTER_CMD_READ, .read = { .ack_value = I2C_ACK_VAL, .data = (uint8_t *)read_buffer, .total_bytes = 1 } },
    //     { .command = I2C_MASTER_CMD_READ, .read = { .ack_value = I2C_NACK_VAL, .data = (uint8_t *)(read_buffer + 1), .total_bytes = 1 } }, // This must be nack.
    //     { .command = I2C_MASTER_CMD_STOP },
    // };
    // i2c_master_execute_defined_operations(bme680_handle, i2c_ops1, sizeof(i2c_ops1) / sizeof(i2c_operation_job_t), -1);

    // Show
    // if (read_buffer[0] != 0x61) {
    //     ESP_LOGE(TAG, "Wrong serial number 0x%x \n\t\t\t(NOT = 0x61)", read_buffer[0]);
    // } else {
    //     ESP_LOGI(TAG, "Sensor serial number is: 0x%x \n\t\t\t(0x61 = OK)", read_buffer[0]);
    // }

    // // Init
    // TriesCount = 1;
    // uint8_t* buff_wr_reset = malloc(2);
    // uint8_t BME680_REG_RESET = 0xe0;
    // uint8_t BME680_RESET_CMD = 0xb6;    // BME680_REG_RESET<7:0>
    // int BME680_RESET_PERIOD = 10;      // reset time in ms

    // buff_wr_reset[0] = BME680_REG_RESET;
    // buff_wr_reset[1] = BME680_RESET_CMD;

    // while (1) {
    //     ret = i2c_master_transmit(bme680_handle, buff_wr_reset, 2, 30);
    //     if (ret != ESP_OK) {
    //         ESP_LOGE(TAG, "Cannot stop sensor measurements now. Retry: %d", TriesCount);
    //         vTaskDelay(pdMS_TO_TICKS(5000));
    //         TriesCount--;
    //         if (TriesCount == 0)
    //             break;
    //     } else {
    //         ESP_LOGI(TAG, "CMD Stop Measurements sent at start!");
    //         vTaskDelay(pdMS_TO_TICKS(BME680_RESET_PERIOD));
    //         break;
    //     }
    // }

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

    i2c_master_bus_handle_t bus_handle;
    ret = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "I2C Bus is ready - now add devices!");
        vTaskDelay(pdMS_TO_TICKS(500));
    } else if (ret == ESP_ERR_INVALID_ARG) {
        ESP_LOGE(TAG, "ESP_ERR_INVALID_ARG: I2C bus initialization failed because of invalid argument!");
    } else if (ret == ESP_ERR_NO_MEM) {
        ESP_LOGE(TAG, "ESP_ERR_NO_MEM: Create I2C bus failed because of out of memory.");
    } else if (ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGE(TAG, "ESP_ERR_NOT_FOUND: No more free bus. ");
    } else {
        ESP_LOGE(TAG, "Cannot init master bus!");
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

    // Probe CO2
    ret =  i2c_master_probe(bus_handle, SCD4X_I2C_ADDR, 500);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Probe tested CO2, OK!");
    } else if (ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGE(TAG, "ESP_ERR_NOT_FOUND: I2C probe failed, doesn't find the device with specific address you gave.");
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.");
    } else {
        ESP_LOGE(TAG, "Unexpected CO2, FAIL!");
    }

    ESP_LOGI(TAG, "Adding BME680...");
    
    // Configure BMD680
    i2c_device_config_t bme680_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME680_I2C_ADDR_1,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    // Add BME680 device second
    ret = i2c_master_bus_add_device(bus_handle, &bme680_cfg, &bme680_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Added BME680 sensor! Now wait!");
        vTaskDelay(pdMS_TO_TICKS(500));
    } else if (ret == ESP_ERR_INVALID_ARG) {
        ESP_LOGE(TAG, "ESP_ERR_INVALID_ARG: I2C bus initialization failed because of invalid argument.");
    } else if (ret == ESP_ERR_NO_MEM) {
        ESP_LOGE(TAG, "ESP_ERR_NO_MEM: Create I2C bus failed because of out of memory.");
    } else {
        ESP_LOGE(TAG, "Cannot add temperature sensor!");
    }
    
    // Probe does not work
    ret =  i2c_master_probe(bus_handle, BME680_I2C_ADDR_1, 500);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Probe tested BME680, OK!");
    } else if (ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGE(TAG, "ESP_ERR_NOT_FOUND: I2C probe failed, doesn't find the device with specific address you gave.");
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash.");
    } else {
        ESP_LOGE(TAG, "Unexpected BME680, FAIL!");
    }


    // CO2 test
    // co2_sensor_tst();

    // Temp and humidity bosh
    // bme650_tst();
    // bme650_alt();

}