#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "nvs_flash.h" // Gerenciamento da memória Flash interna [1]

static const char *TAG = "CONTROLE_ACESSO";

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   21
#define PIN_RELE     GPIO_NUM_2 // Pino do Relé (Equivale ao LED azul da placa para testes)

#define MAX_TAGS 30 // Capacidade máxima de usuários armazenados [1]

static spi_device_handle_t spi_handle;
static uint8_t master_tag[4] = {0};
static uint8_t registered_tags[MAX_TAGS][4] = {0};
static uint8_t total_tags = 0;
static bool has_master = false;

// --- Funções de Comunicação Física SPI (Validadas no seu hardware) ---
void rc522_write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx_data[2] = { (reg << 1) & 0x7E, val };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx_data, .rx_buffer = NULL };
    spi_device_transmit(spi_handle, &t);
}

uint8_t rc522_read_reg(uint8_t reg) {
    uint8_t tx_data[2] = { ((reg << 1) & 0x7E) | 0x80, 0x00 };
    uint8_t rx_data[2] = {0};
    spi_transaction_t t = { .length = 16, .tx_buffer = tx_data, .rx_buffer = rx_data };
    spi_device_transmit(spi_handle, &t);
    return rx_data[1];
}

void rc522_set_bit_mask(uint8_t reg, uint8_t mask) { rc522_write_reg(reg, rc522_read_reg(reg) | mask); }
void rc522_clear_bit_mask(uint8_t reg, uint8_t mask) { rc522_write_reg(reg, rc522_read_reg(reg) & (~mask)); }

void rc522_init(void) {
    rc522_write_reg(0x01, 0x0F); // Soft Reset
    vTaskDelay(pdMS_TO_TICKS(50));
    rc522_write_reg(0x2A, 0x8D); rc522_write_reg(0x2B, 0x3E);
    rc522_write_reg(0x2D, 30); rc522_write_reg(0x2C, 0);
    rc522_write_reg(0x15, 0x40); rc522_write_reg(0x11, 0x3D);
    rc522_set_bit_mask(0x14, 0x03); // Liga Antena
}

esp_err_t rc522_to_card(uint8_t command, uint8_t *send_data, uint8_t send_len, uint8_t *back_data, uint32_t *back_len) {
    uint8_t irqEn = 0x77, waitIRq = 0x30;
    rc522_write_reg(0x02, irqEn | 0x80);
    rc522_clear_bit_mask(0x04, 0x80);
    rc522_set_bit_mask(0x0A, 0x80);
    rc522_write_reg(0x01, 0x00);
    for (uint8_t i = 0; i < send_len; i++) rc522_write_reg(0x09, send_data[i]);
    rc522_write_reg(0x01, command);
    rc522_set_bit_mask(0x0D, 0x80);
    uint32_t i = 2000; uint8_t n;
    do { n = rc522_read_reg(0x04); i--; } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));
    rc522_clear_bit_mask(0x0D, 0x80);
    if (i == 0 || (rc522_read_reg(0x06) & 0x1B)) return ESP_FAIL;
    if (back_data && back_len) {
        uint8_t status = rc522_read_reg(0x0A);
        if (status > 0) {
            *back_len = status;
            for (i = 0; i < status; i++) back_data[i] = rc522_read_reg(0x09);
            return ESP_OK;
        }
    }
    return ESP_OK;
}

esp_err_t rc522_request(uint8_t req_mode, uint8_t *tag_type) {
    uint32_t len; uint8_t buffer[2];
    rc522_write_reg(0x0D, 0x07); buffer[0] = req_mode;
    if (rc522_to_card(0x0C, buffer, 1, buffer, &len) == ESP_OK && len == 2) {
        tag_type[0] = buffer[0]; tag_type[1] = buffer[1];
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t rc522_anticoll(uint8_t *uid) {
    uint32_t len; uint8_t buffer[5];
    rc522_write_reg(0x0D, 0x00); buffer[0] = 0x93; buffer[1] = 0x20;
    if (rc522_to_card(0x0C, buffer, 2, buffer, &len) == ESP_OK) {
        for (uint8_t i = 0; i < 4; i++) uid[i] = buffer[i];
        return ESP_OK;
    }
    return ESP_FAIL;
}

// --- Gerenciamento persistente da Flash (NVS) --- [1]
void carregar_banco_dados(void) {
    nvs_handle_t nvs_h;
    if (nvs_open("rfid_storage", NVS_READWRITE, &nvs_h) == ESP_OK) {
        size_t size = sizeof(master_tag);
        if (nvs_get_blob(nvs_h, "master", master_tag, &size) == ESP_OK) {
            has_master = true;
            ESP_LOGI(TAG, "Tag Mestra Carregada: %02X %02X %02X %02X", master_tag[0], master_tag[1], master_tag[2], master_tag[3]);
        }
        size = sizeof(registered_tags);
        nvs_get_blob(nvs_h, "users", registered_tags, &size);
        uint8_t count = 0;
        nvs_get_u8(nvs_h, "total", &count);
        total_tags = count;
        ESP_LOGI(TAG, "Banco de dados pronto. Usuários cadastrados: %d", total_tags);
        nvs_close(nvs_h);
    }
}

void salvar_banco_dados(void) {
    nvs_handle_t nvs_h;
    if (nvs_open("rfid_storage", NVS_READWRITE, &nvs_h) == ESP_OK) {
        nvs_set_blob(nvs_h, "master", master_tag, sizeof(master_tag));
        nvs_set_blob(nvs_h, "users", registered_tags, sizeof(registered_tags));
        nvs_set_u8(nvs_h, "total", total_tags);
        nvs_commit(nvs_h);
        nvs_close(nvs_h);
        ESP_LOGI(TAG, "Memória persistente atualizada!");
    }
}

bool comparar_uid(uint8_t *a, uint8_t *b) {
    return (a[0] == b[0] && a[1] == b[1] && a[2] == b[2] && a[3] == b[3]);
}

// --- Menu Dinâmico de Cadastro de Usuários --- [1]
void modo_cadastro(uint8_t *str, uint8_t *uid) {
    ESP_LOGW(TAG, "=====================================================");
    ESP_LOGW(TAG, "🔵 MODO DE CADASTRO ATIVO! Aproxime uma TAG de Usuário.");
    ESP_LOGW(TAG, "   (Passe a Tag Mestra novamente para SAIR)");
    ESP_LOGW(TAG, "=====================================================");
    vTaskDelay(pdMS_TO_TICKS(1500)); 

    while (1) {
        if (rc522_request(0x26, str) == ESP_OK) {
            if (rc522_anticoll(uid) == ESP_OK) {
                
                // Se passar a Mestra, fecha o modo de cadastro
                if (comparar_uid(uid, master_tag)) {
                    ESP_LOGI(TAG, "⚪ Modo Cadastro encerrado.");
                    vTaskDelay(pdMS_TO_TICKS(1500));
                    return;
                }

                int index = -1;
                for (int i = 0; i < total_tags; i++) {
                    if (comparar_uid(uid, registered_tags[i])) { index = i; break; }
                }

                if (index >= 0) {
                    // Se a tag já existia, ela será deletada
                    for (int i = index; i < total_tags - 1; i++) {
                        memcpy(registered_tags[i], registered_tags[i+1], 4);
                    }
                    total_tags--;
                    ESP_LOGE(TAG, "🔴 TAG REMOVIDA DO SISTEMA: %02X %02X %02X %02X", uid[0], uid[1], uid[2], uid[3]);
                } else if (total_tags < MAX_TAGS) {
                    // Se a tag é nova, adiciona ao banco
                    memcpy(registered_tags[total_tags], uid, 4);
                    total_tags++;
                    // Altere de ESP_LOGG para ESP_LOGI:
                    ESP_LOGI(TAG, "🟢 NOVA TAG AUTORIZADA COM SUCESSO: %02X %02X %02X %02X", uid[0], uid[1], uid[2], uid[3]);

                } else {
                    ESP_LOGE(TAG, "Banco de dados cheio! Limite de %d atingido.", MAX_TAGS);
                }// Altere de ESP_LOGG para ESP_LOGI:
                ESP_LOGI(TAG, "🟢 NOVA TAG AUTORIZADA COM SUCESSO: %02X %02X %02X %02X", uid[0], uid[1], uid[2], uid[3]);             
                salvar_banco_dados();
                vTaskDelay(pdMS_TO_TICKS(1500));
                ESP_LOGW(TAG, "Aguardando próxima TAG...");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void) {
    // Inicializa a partição da memória Flash NVS [1]
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Inicializa pino do Relé
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE, .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PIN_RELE), .pull_down_en = 0, .pull_up_en = 0
    };
    gpio_config(&io_conf);
    gpio_set_level(PIN_RELE, 0); 

    // Configuração SPI
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO, .mosi_io_num = PIN_NUM_MOSI, .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 32
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000, .mode = 0, .spics_io_num = PIN_NUM_CS, .queue_size = 7
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &spi_handle));

    rc522_init();
    carregar_banco_dados();

    uint8_t str[2];
    uint8_t uid[5];

    while (1) {
        if (rc522_request(0x26, str) == ESP_OK) {
            if (rc522_anticoll(uid) == ESP_OK) {
                
                // Passo 1: Cadastro automático da Tag Mestra no primeiro uso [1]
                if (!has_master) {
                    memcpy(master_tag, uid, 4);
                    has_master = true;
                    salvar_banco_dados();
                    ESP_LOGI(TAG, "🏆 PRIMEIRA LEITURA: Tag Mestra cadastrada: %02X %02X %02X %02X", uid[0], uid[1], uid[2], uid[3]);
                    vTaskDelay(pdMS_TO_TICKS(1500));
                    continue;
                }

                // Passo 2: Se ler a Tag Mestra, entra nas configurações [1]
                if (comparar_uid(uid, master_tag)) {
                    modo_cadastro(str, uid);
                    continue;
                }

                // Passo 3: Verificação de usuários comuns [1]
                bool autorizado = false;
                for (int i = 0; i < total_tags; i++) {
                    if (comparar_uid(uid, registered_tags[i])) { autorizado = true; break; }
                }

                if (autorizado) {
                    ESP_LOGI(TAG, "🟢 ACESSO PERMITIDO! Acionando relé.");
                    gpio_set_level(PIN_RELE, 1);  // Liga o relé
                    vTaskDelay(pdMS_TO_TICKS(3000)); // Mantém ligado por 3 segundos
                    gpio_set_level(PIN_RELE, 0);  // Desliga o relé
                    ESP_LOGI(TAG, "🔒 Relé desativado.");
                } else {
                    ESP_LOGW(TAG, "🔴 ACESSO NEGADO! ID não cadastrado.");
                }
                vTaskDelay(pdMS_TO_TICKS(1500));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
