# Test I2C

This is a separate "branch" from my project [Air_Quality_station](https://github.com/trianglesis/Air_Quality_station/blob/4802c416501cf49720c3299119afd54bb335f894/README.md)

Where I stuck with `NACK` messages for any interaction with `I2C` device.

Compose driver using official documentation.

Test installed `I2C` device discovery with utility: [i2c_tools](https://github.com/espressif/esp-idf/blob/4c2820d377d1375e787bcef612f0c32c1427d183/examples/peripherals/i2c/i2c_tools/README.md)

Then, create a `I2C` bus and communicate with devices:

- `SCD40` - CO2 Sensor `0x62`
- `BME680` - temperature, humidity, pressure sensor `0x77`

## SDC4x

There are two representations for this sensor, both rely on old `I2C` driver and have partial implementation, something work, something is not:

- [ESP IDF Lib (old)](https://github.com/Sensirion/embedded-i2c-scd4x/blob/455a41c6b7a7a86a55d6647f5fc22d8574572b7b)
- [Sensirion (newer)](https://github.com/Sensirion/embedded-i2c-scd4x/blob/455a41c6b7a7a86a55d6647f5fc22d8574572b7b)


## BME680

There is also a driver for `BME680` from the same repo, but again, it relies on an old `I2C` driver and there are issues with this driver:

- [ESP IDF Lib (old)](https://github.com/UncleRus/esp-idf-lib/blob/a02cd6bb5190cab379125140780adcb8d88f9650/components/bme680)


## Test and discover

Use `i2c_tools` from IDF official repo.

Both sensors are using same pins: `i2cconfig  --port=0 --freq=100000 --sda=22 --scl=23`


![pins](doc/pics/sensor_pins_i2c.png)

Both of them discovered correctly

```log
i2c-tools> i2cconfig  --port=0 --freq=100000 --sda=22 --scl=23
i2c-tools> i2cdetect
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00: 00 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- 62 -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- 77 -- -- -- -- -- -- -- --
```