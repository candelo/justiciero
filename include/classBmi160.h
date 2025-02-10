#ifndef CLASSBMI160_H__
#define CLASSBMI160_H__

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <driver/i2c.h>

#include "bmi160.h"


typedef struct {
    spi_host_device_t spidev;
    gpio_num_t mosi;
    gpio_num_t miso;
    gpio_num_t sclk;
    gpio_num_t cs;
    uint32_t speed;
}Bmi160SpiConfig;

typedef struct {    
    gpio_num_t sda;
    gpio_num_t scl;
    uint8_t addr;
}Bmi160I2cConfig;




class Bmi160{

private:

    bmi160_dev dev;
    spi_bus_config_t spiBus;
    spi_device_interface_config_t spiIf; 
    float accScale;
    float gyrScale;

public:

    typedef struct{
        float x;
        float y;
        float z;
        uint32_t time;
    } Data;

 
    Bmi160();
    uint8_t init(Bmi160SpiConfig spiConfig);
    uint8_t init(Bmi160I2cConfig i2cConfig);

    uint8_t configure();
    
    uint8_t getRawData(bmi160_sensor_data *acc, bmi160_sensor_data *gyr);
    uint8_t getData(Data *acc, Data *gyr);

    static spi_device_handle_t spiHandle;
};


#endif