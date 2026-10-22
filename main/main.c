#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

static const char *TAG = "RFID_NATIVO_OK";

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   21

static spi_device_handle_t spi_handle;

// Escreve em um registrador do RC522
void rc522_write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx_data[2] = { (reg << 1) & 0x7E, val };
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_data,
        .rx_buffer = NULL
    };
    spi_device_transmit(spi_handle, &t);
}

// Lê um registrador do RC522
uint8_t rc522_read_reg(uint8_t reg) {
    uint8_t tx_data[2] = { ((reg << 1) & 0x7E) | 0x80, 0x00 };
    uint8_t rx_data[2] = {0};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    spi_device_transmit(spi_handle, &t);
    return rx_data[1];
}

// Limpa e define máscaras de bits nos registradores
void rc522_set_bit_mask(uint8_t reg, uint8_t mask) {
    rc522_write_reg(reg, rc522_read_reg(reg) | mask);
}
void rc522_clear_bit_mask(uint8_t reg, uint8_t mask) {
    rc522_write_reg(reg, rc522_read_reg(reg) & (~mask));
}

// Inicializa o RC522 e liga a antena
void rc522_init(void) {
    rc522_write_reg(0x01, 0x0F); // Soft Reset
    vTaskDelay(pdMS_TO_TICKS(50));
    
    rc522_write_reg(0x2A, 0x8D); // TModeReg
    rc522_write_reg(0x2B, 0x3E); // TPrescalerReg
    rc522_write_reg(0x2D, 30);   // TReloadRegL
    rc522_write_reg(0x2C, 0);    // TReloadRegH
    rc522_write_reg(0x15, 0x40); // TxASKReg
    rc522_write_reg(0x11, 0x3D); // ModeReg
    
    // Liga a antena
    rc522_set_bit_mask(0x14, 0x03); // TxControlReg
}

// Comunica diretamente com a Tag RFID via rádio
esp_err_t rc522_to_card(uint8_t command, uint8_t *send_data, uint8_t send_len, uint8_t *back_data, uint32_t *back_len) {
    uint8_t irqEn = 0x00;
    uint8_t waitIRq = 0x00;
    
    if (command == 0x0E) { // Autenticação
        irqEn = 0x12;
        waitIRq = 0x10;
    } else if (command == 0x0C) { // Transmitir e receber
        irqEn = 0x77;
        waitIRq = 0x30;
    }
    
    rc522_write_reg(0x02, irqEn | 0x80);
    rc522_clear_bit_mask(0x04, 0x80);
    rc522_set_bit_mask(0x0A, 0x80); // Flush FIFO
    rc522_write_reg(0x01, 0x00); // Idle
    
    for (uint8_t i = 0; i < send_len; i++) {
        rc522_write_reg(0x09, send_data[i]);
    }
    
    rc522_write_reg(0x01, command);
    if (command == 0x0C) {
        rc522_set_bit_mask(0x0D, 0x80); // StartSend
    }
    
    uint32_t i = 2000; // Timeout
    uint8_t n;
    do {
        n = rc522_read_reg(0x04);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));
    
    rc522_clear_bit_mask(0x0D, 0x80);
    
    if (i == 0) return ESP_ERR_TIMEOUT;
    if (rc522_read_reg(0x06) & 0x1B) return ESP_FAIL; // BufferErr, CollErr, ParityErr, ProtocolErr
    
    if (back_data && back_len) {
        n = rc522_read_reg(0x04);
        uint8_t status = rc522_read_reg(0x0A);
        if (status > 0) {
            *back_len = status;
            for (i = 0; i < status; i++) {
                back_data[i] = rc522_read_reg(0x09);
            }
            return ESP_OK;
        }
    }
    return ESP_OK;
}

// Solicita a presença de um cartão (Request)
esp_err_t rc522_request(uint8_t req_mode, uint8_t *tag_type) {
    uint32_t len;
    uint8_t buffer[2];
    
    rc522_write_reg(0x0D, 0x07); // BitFramingReg
    buffer[0] = req_mode;
    
    esp_err_t status = rc522_to_card(0x0C, buffer, 1, buffer, &len);
    if ((status == ESP_OK) && (len == 2)) {
        tag_type[0] = buffer[0];
        tag_type[1] = buffer[1];
        return ESP_OK;
    }
    return ESP_FAIL;
}

// Executa a rotina de anticolisão e puxa a UID
esp_err_t rc522_anticoll(uint8_t *uid) {
    uint32_t len;
    uint8_t buffer[5];
    
    rc522_write_reg(0x0D, 0x00); // BitFramingReg
    buffer[0] = 0x93; // PICC_ANTICOLL
    buffer[1] = 0x20;
    
    esp_err_t status = rc522_to_card(0x0C, buffer, 2, buffer, &len);
    if (status == ESP_OK) {
        for (uint8_t i = 0; i < 4; i++) {
            uid[i] = buffer[i];
        }
        return ESP_OK;
    }
    return ESP_FAIL;
}

void app_main(void) {
    ESP_LOGI(TAG, "Inicializando barramento SPI...");

    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000, // 1 MHz para máxima estabilidade
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &spi_handle));

    rc522_init();

    // Valida o hardware lendo o Firmware
    uint8_t versao = rc522_read_reg(0x37);
    if (versao == 0x00 || versao == 0xFF) {
        ESP_LOGE(TAG, "Falha crítica de comunicação com o RC522!");
        return;
    }
    
    ESP_LOGI(TAG, "RC522 pronto! Versão detectada: 0x%02X. Aguardando aproximação de TAGs...", versao);

    uint8_t str[2];
    uint8_t uid[5];

    while (1) {
        // Envia o sinal de busca (0x26 = PICC_REQIDL)
        if (rc522_request(0x26, str) == ESP_OK) {
            // Se encontrar o cartão, extrai o número serial dele (UID)
            if (rc522_anticoll(uid) == ESP_OK) {
                ESP_LOGI(TAG, "-------------------------------------------");
                ESP_LOGI(TAG, "   TAG RFID LIDA DIRETAMENTE COM SUCESSO!  ");
                ESP_LOGI(TAG, "   UID: %02X %02X %02X %02X", uid[0], uid[1], uid[2], uid[3]);
                ESP_LOGI(TAG, "-------------------------------------------");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(300)); // Delay suave entre leituras
    }
}
