#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h" // Watchdog Timer para prevenção de travamentos

static const char *TAG = "RFID_RESILIENTE";

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 21
#define PIN_RELE GPIO_NUM_2

#define MAX_TAGS 30

static spi_device_handle_t spi_handle;
static uint8_t master_tag[4] = {0};
static uint8_t registered_tags[MAX_TAGS][4] = {0};
static uint8_t total_tags = 0;
static bool has_master = false;

void rc522_init(void);

// Escreve em um registrador do RC522 com proteção SPI
void rc522_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx_data[2] = {(reg << 1) & 0x7E, val};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_data,
        .rx_buffer = NULL};
    if (spi_device_transmit(spi_handle, &t) != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha de barramento na escrita do reg 0x%02X", reg);
    }
}

// Lê um registrador do RC522 isolando flutuações de pinos
uint8_t rc522_read_reg(uint8_t reg)
{
    uint8_t tx_data[2] = {((reg << 1) & 0x7E) | 0x80, 0x00};
    uint8_t rx_data[2] = {0};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    if (spi_device_transmit(spi_handle, &t) != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha de barramento na leitura do reg 0x%02X", reg);
        return 0xFF;
    }
    return rx_data[1];
}

void rc522_set_bit_mask(uint8_t reg, uint8_t mask) { rc522_write_reg(reg, rc522_read_reg(reg) | mask); }
void rc522_clear_bit_mask(uint8_t reg, uint8_t mask) { rc522_write_reg(reg, rc522_read_reg(reg) & (~mask)); }

// Configuração de rádio do chip
void rc522_init(void)
{
    rc522_write_reg(0x01, 0x0F); // Soft Reset interno do módulo
    vTaskDelay(pdMS_TO_TICKS(50));

    rc522_write_reg(0x2A, 0x8D);
    rc522_write_reg(0x2B, 0x3E);
    rc522_write_reg(0x2D, 30);
    rc522_write_reg(0x2C, 0);
    rc522_write_reg(0x15, 0x40);
    rc522_write_reg(0x11, 0x3D);
    rc522_set_bit_mask(0x14, 0x03); // Ativa as antenas RF
}

// Auto-recuperação ativa se houver surto ou estática no leitor físico
bool verificar_integridade_hardware(void)
{
    uint8_t versao = rc522_read_reg(0x37);
    if (versao == 0x00 || versao == 0xFF)
    {
        ESP_LOGE(TAG, "Hardware instável ou desconectado! Reiniciando rádio...");
        rc522_init();
        return false;
    }
    return true;
}

esp_err_t rc522_to_card(uint8_t command, uint8_t *send_data, uint8_t send_len, uint8_t *back_data, uint32_t *back_len)
{
    uint8_t irqEn = 0x77, waitIRq = 0x30;
    rc522_write_reg(0x02, irqEn | 0x80);
    rc522_clear_bit_mask(0x04, 0x80);
    rc522_set_bit_mask(0x0A, 0x80);
    rc522_write_reg(0x01, 0x00);
    for (uint8_t i = 0; i < send_len; i++)
        rc522_write_reg(0x09, send_data[i]);
    rc522_write_reg(0x01, command);
    rc522_set_bit_mask(0x0D, 0x80);

    uint32_t timeout_counter = 2000;
    uint8_t n;
    do
    {
        n = rc522_read_reg(0x04);
        timeout_counter--;
    } while ((timeout_counter != 0) && !(n & 0x01) && !(n & waitIRq));

    rc522_clear_bit_mask(0x0D, 0x80);

    if (timeout_counter == 0 || (rc522_read_reg(0x06) & 0x1B))
        return ESP_FAIL;

    if (back_data && back_len)
    {
        uint8_t status = rc522_read_reg(0x0A);
        if (status > 0)
        {
            *back_len = status;
            for (timeout_counter = 0; timeout_counter < status; timeout_counter++)
            {
                back_data[timeout_counter] = rc522_read_reg(0x09);
            }
            return ESP_OK;
        }
    }
    return ESP_OK;
}

esp_err_t rc522_request(uint8_t req_mode, uint8_t *tag_type)
{
    uint32_t len;
    uint8_t buffer[2];
    rc522_write_reg(0x0D, 0x07);
    buffer[0] = req_mode;
    if (rc522_to_card(0x0C, buffer, 1, buffer, &len) == ESP_OK && len == 2)
    {
        tag_type[0] = buffer[0];
        tag_type[1] = buffer[1];
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t rc522_anticoll(uint8_t *uid)
{
    uint32_t len;
    uint8_t buffer[5];
    rc522_write_reg(0x0D, 0x00);
    buffer[0] = 0x93;
    buffer[1] = 0x20;
    if (rc522_to_card(0x0C, buffer, 2, buffer, &len) == ESP_OK)
    {
        for (uint8_t i = 0; i < 4; i++)
            uid[i] = buffer[i];
        return ESP_OK;
    }
    return ESP_FAIL;
}

void carregar_banco_dados(void)
{
    nvs_handle_t nvs_h;
    esp_err_t err = nvs_open("rfid_storage", NVS_READWRITE, &nvs_h);
    if (err == ESP_OK)
    {
        size_t size = sizeof(master_tag);
        if (nvs_get_blob(nvs_h, "master", master_tag, &size) == ESP_OK)
        {
            has_master = true;
            ESP_LOGI(TAG, "Tag Mestra carregada com segurança.");
        }
        size = sizeof(registered_tags);
        nvs_get_blob(nvs_h, "users", registered_tags, &size);
        uint8_t count = 0;
        nvs_get_u8(nvs_h, "total", &count);
        total_tags = count;
        nvs_close(nvs_h);
        ESP_LOGI(TAG, "Banco de dados pronto. Usuários cadastrados: %d", total_tags);
    }
    else
    {
        ESP_LOGE(TAG, "Erro ao abrir NVS! Operando temporariamente via memória RAM.");
    }
}

void salvar_banco_dados(void)
{
    nvs_handle_t nvs_h;
    if (nvs_open("rfid_storage", NVS_READWRITE, &nvs_h) == ESP_OK)
    {
        nvs_set_blob(nvs_h, "master", master_tag, sizeof(master_tag));
        nvs_set_blob(nvs_h, "users", registered_tags, sizeof(registered_tags));
        nvs_set_u8(nvs_h, "total", total_tags);
        if (nvs_commit(nvs_h) == ESP_OK)
        {
            ESP_LOGI(TAG, "Mudanças salvas na memória flash!");
        }
        nvs_close(nvs_h);
    }
}

bool comparar_uid(uint8_t *a, uint8_t *b)
{
    return (a[0] == b[0] && a[1] == b[1] && a[2] == b[2] && a[3] == b[3]);
}

void modo_cadastro(uint8_t *str, uint8_t *uid)
{
    ESP_LOGW(TAG, "=====================================================");
    ESP_LOGW(TAG, "🔵 MODO DE CADASTRO ATIVO! Aproxime uma TAG de Usuário.");
    ESP_LOGW(TAG, "=====================================================");
    vTaskDelay(pdMS_TO_TICKS(1500));

    while (1)
    {
        esp_task_wdt_reset(); // Reseta o Watchdog dentro do loop de cadastro

        if (rc522_request(0x26, str) == ESP_OK)
        {
            if (rc522_anticoll(uid) == ESP_OK)
            {
                if (comparar_uid(uid, master_tag))
                {
                    ESP_LOGI(TAG, "⚪ Modo Cadastro encerrado.");
                    vTaskDelay(pdMS_TO_TICKS(1500));
                    return;
                }

                int index = -1;
                for (int i = 0; i < total_tags; i++)
                {
                    if (comparar_uid(uid, registered_tags[i]))
                    {
                        index = i;
                        break;
                    }
                }

                if (index >= 0)
                {
                    for (int i = index; i < total_tags - 1; i++)
                    {
                        memcpy(registered_tags[i], registered_tags[i + 1], 4);
                    }
                    total_tags--;
                    ESP_LOGE(TAG, "🔴 TAG REMOVIDA DO SISTEMA: %02X %02X %02X %02X", uid[0], uid[1], uid[2], uid[3]);
                }
                else if (total_tags < MAX_TAGS)
                {
                    memcpy(registered_tags[total_tags], uid, 4);
                    total_tags++;
                    ESP_LOGI(TAG, "🟢 NOVA TAG AUTORIZADA COM SUCESSO: %02X %02X %02X %02X", uid[0], uid[1], uid[2], uid[3]);
                }
                else
                {
                    ESP_LOGE(TAG, "Banco de dados cheio!");
                }

                salvar_banco_dados();
                vTaskDelay(pdMS_TO_TICKS(1500));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Configuração do Watchdog com limite de 5 segundos para destravar o processador
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 5000,
        .idle_core_mask = (1 << 0),
        .trigger_panic = true};
    esp_task_wdt_init(&twdt_config);
    esp_task_wdt_add(NULL);

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE, .mode = GPIO_MODE_OUTPUT, .pin_bit_mask = (1ULL << PIN_RELE), .pull_down_en = 0, .pull_up_en = 0};
    gpio_config(&io_conf);
    gpio_set_level(PIN_RELE, 0);

    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO, .mosi_io_num = PIN_NUM_MOSI, .sclk_io_num = PIN_NUM_CLK, .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 32};
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000, .mode = 0, .spics_io_num = PIN_NUM_CS, .queue_size = 7};
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &spi_handle));

    rc522_init();
    carregar_banco_dados();

    uint8_t str[2];
    uint8_t uid[5];
    uint32_t checagem_hardware_timer = 0;

    while (1)
    {
        esp_task_wdt_reset(); // Sinaliza ao Watchdog que o código principal está saudável

        // Verifica a conexão e o rádio do sensor RC522 a cada ~1.4 segundos
        if (checagem_hardware_timer++ >= 7)
        {
            verificar_integridade_hardware();
            checagem_hardware_timer = 0;
        }

        if (rc522_request(0x26, str) == ESP_OK)
        {
            if (rc522_anticoll(uid) == ESP_OK)
            {

                if (!has_master)
                {
                    memcpy(master_tag, uid, 4);
                    has_master = true;
                    salvar_banco_dados();
                    ESP_LOGI(TAG, "🏆 Tag Mestra cadastrada: %02X %02X %02X %02X", uid[0], uid[1], uid[2], uid[3]);
                    vTaskDelay(pdMS_TO_TICKS(1500));
                    continue;
                }

                if (comparar_uid(uid, master_tag))
                {
                    modo_cadastro(str, uid);
                    continue;
                }
                bool autorizado = false;
                for (int i = 0; i < total_tags; i++)
                {
                    if (comparar_uid(uid, registered_tags[i]))
                    {
                        autorizado = true;
                        break;
                    }
                }
                if (autorizado)
                {
                    ESP_LOGI(TAG, "🟢 ACESSO PERMITIDO! Acionando relé.");
                    gpio_set_level(PIN_RELE, 1);
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    gpio_set_level(PIN_RELE, 0);
                    ESP_LOGI(TAG, "🔒 Relé desativado.");
                }
                else
                {
                    ESP_LOGW(TAG, "🔴 ACESSO NEGADO! ID não cadastrado.");
                }
                vTaskDelay(pdMS_TO_TICKS(1500));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}