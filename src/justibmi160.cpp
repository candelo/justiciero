#if 0

#include "justibmi160.h"
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>

const char TAG[] = "BMI_DRIVER";



static spi_device_handle_t SpiHandle = {0};

int8_t bmi_read_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t bmi_write_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len);
void bmi_delay(uint32_t period);


bmi160_dev bmi_init_spi(){

    bmi160_dev dev;
    spi_bus_config_t busSpi = {0};    
    spi_device_interface_config_t spiIf = {0};

    busSpi.mosi_io_num = GPIO_NUM_23;
    busSpi.miso_io_num = GPIO_NUM_19;
    busSpi.sclk_io_num = GPIO_NUM_18;
     
    if(spi_bus_initialize(SPI3_HOST, &busSpi, SPI_DMA_DISABLED) != ESP_OK){
        ESP_LOGE(TAG, "spi cannot be initialized");
    }
    ESP_LOGE(TAG, "spi bud initialized correctly");

    spiIf.spics_io_num = GPIO_NUM_21; //Cambiar
    spiIf.clock_speed_hz = 10 * 1000 * 1000;
    spiIf.mode = 0;
    spiIf.queue_size = 10;
    spiIf.address_bits = 8;

    

    if(spi_bus_add_device(SPI3_HOST, &spiIf, &SpiHandle) != ESP_OK){
        ESP_LOGE(TAG, "spi device cannot be added");
 
    }
    ESP_LOGE(TAG, "spi bus added correctly");

    dev.intf = BMI160_SPI_INTF;
    dev.read = bmi_read_spi;
    dev.write = bmi_write_spi;
    dev.delay_ms = bmi_delay;

    return dev;
}


int8_t bmi_read_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len){
      
    spi_transaction_t Spi_trans = {0};

    Spi_trans.length = 8 * len;
    Spi_trans.rxlength = 8 * len;
    //Spi_trans.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    Spi_trans.addr = reg_addr;
    Spi_trans.rx_buffer = data;
    spi_device_acquire_bus(SpiHandle, portMAX_DELAY);
    spi_device_polling_transmit(SpiHandle, &Spi_trans);
    spi_device_release_bus(SpiHandle);

    return BMI160_OK;
}
int8_t bmi_write_spi(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len){
    
    spi_transaction_t Spi_trans = {0};

    Spi_trans.length = 8 * len;
    Spi_trans.addr = reg_addr;
    Spi_trans.tx_buffer = read_data;
    spi_device_acquire_bus(SpiHandle, portMAX_DELAY);
    spi_device_polling_transmit(SpiHandle, &Spi_trans);
    spi_device_release_bus(SpiHandle);

     return BMI160_OK;

}
void bmi_delay(uint32_t period){
    if(period < 10)period = 10;
    vTaskDelay(pdMS_TO_TICKS(period));
}


void bmi_initSpi(){

    spi_bus_config_t busSpi = {0};
    spi_device_handle_t SpiHandle = {0};
    spi_device_interface_config_t spiIf = {0};

    busSpi.mosi_io_num = GPIO_NUM_23;
    busSpi.miso_io_num = GPIO_NUM_19;
    busSpi.sclk_io_num = GPIO_NUM_18;
     
    if(spi_bus_initialize(SPI3_HOST, &busSpi, SPI_DMA_DISABLED) != ESP_OK){
        ESP_LOGE(TAG, "spi cannot be initialized");
        return;
    }
    ESP_LOGE(TAG, "spi bud initialized correctly");

    spiIf.spics_io_num = GPIO_NUM_21; //Cambiar
    spiIf.clock_speed_hz = 10 * 1000 * 1000;
    spiIf.mode = 0;
    spiIf.queue_size = 10;
    spiIf.address_bits = 8;

    

    if(spi_bus_add_device(SPI3_HOST, &spiIf, &SpiHandle) != ESP_OK){
        ESP_LOGE(TAG, "spi device cannot be added");
        return;
    }
    ESP_LOGE(TAG, "spi bus added correctly");

    uint8_t spiReadAddress = (0x7F | 0x80);
    spi_transaction_t Spi_trans = {0};

    Spi_trans.length = 8;
    //Spi_trans.tx_data[0] = spiReadAddress;
    Spi_trans.rxlength = 8;
    Spi_trans.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;

    //spi_device_acquire_bus(SpiHandle, portMAX_DELAY);
    Spi_trans.addr = spiReadAddress;
    if(spi_device_polling_transmit(SpiHandle, &Spi_trans) != ESP_OK){
        ESP_LOGE(TAG, "SPI Transaction Error");
        return;
    }
    ESP_LOGE(TAG, "SPI Transaction OK");
    ESP_LOGE(TAG, "Received value %d", Spi_trans.rx_data[0]);
    
    //Spi_trans.tx_data[0] = 0x00;
    Spi_trans.addr = (0x00 | 0x80);
    if(spi_device_polling_transmit(SpiHandle, &Spi_trans) != ESP_OK){
        ESP_LOGE(TAG, "SPI Transaction Error");
        return;
    }
    ESP_LOGE(TAG, "SPI Transaction OK");
    ESP_LOGE(TAG, "Received value %d", Spi_trans.rx_data[0]);
}

uint8_t bmi_getId(){
    return 0;
}

#endif