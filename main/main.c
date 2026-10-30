#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h" 
#include "esp_timer.h"
#include "led_strip.h"
#include "esp_err.h"

static const char *TAG = "RFID_RESILIENTE";

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 21
#define PIN_RELE GPIO_NUM_13

#define MAX_TAGS 30

#define LED_STRIP_LED_COUNT 12
#define LED_STRIP_MEMORY_BLOCK_WORDS 0

#define LED_STRIP_GPIO_PIN GPIO_NUM_22
#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)

#define BUTTON_ESTADO GPIO_NUM_14
#define BUTTON_MASTER GPIO_NUM_27

static int64_t ultimo_tempo_rele = 0;
bool has_master = false;

led_strip_handle_t configure_led(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN,
        .max_leds = LED_STRIP_LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = { .invert_out = false }
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_STRIP_RMT_RES_HZ,
        .mem_block_symbols = LED_STRIP_MEMORY_BLOCK_WORDS,
        .flags = { .with_dma = 0 }
    };

    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    return led_strip;
}

static spi_device_handle_t spi_handle;
static uint8_t master_tag[4] = {0};
static uint8_t registered_tags[MAX_TAGS][4] = {0};
static uint8_t total_tags = 0;

void rc522_init(void);

void rc522_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx_data[2] = {(reg << 1) & 0x7E, val};
    spi_transaction_t t = { .length = 16, .tx_buffer = tx_data, .rx_buffer = NULL };
    if (spi_device_transmit(spi_handle, &t) != ESP_OK)
        ESP_LOGE(TAG, "Falha na escrita SPI");
}

uint8_t rc522_read_reg(uint8_t reg)
{
    uint8_t tx_data[2] = {((reg << 1) & 0x7E) | 0x80, 0x00};
    uint8_t rx_data[2] = {0};
    spi_transaction_t t = { .length = 16, .tx_buffer = tx_data, .rx_buffer = rx_data };
    if (spi_device_transmit(spi_handle, &t) != ESP_OK) return 0xFF;
    return rx_data[1];
}

void rc522_set_bit_mask(uint8_t reg, uint8_t mask) { rc522_write_reg(reg, rc522_read_reg(reg) | mask); }
void rc522_clear_bit_mask(uint8_t reg, uint8_t mask) { rc522_write_reg(reg, rc522_read_reg(reg) & (~mask)); }

void rc522_init(void)
{
    rc522_write_reg(0x01, 0x0F);
    vTaskDelay(pdMS_TO_TICKS(50));
    rc522_write_reg(0x2A, 0x8D);
    rc522_write_reg(0x2B, 0x3E);
    rc522_write_reg(0x2D, 30);
    rc522_write_reg(0x2C, 0);
    rc522_write_reg(0x15, 0x40);
    rc522_write_reg(0x11, 0x3D);
    rc522_set_bit_mask(0x14, 0x03);
}

bool verificar_integridade_hardware(void)
{
    uint8_t versao = rc522_read_reg(0x37);
    if (versao == 0x00 || versao == 0xFF)
    {
        ESP_LOGE(TAG, "Hardware instável! Reiniciando rádio...");
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
    for (uint8_t i = 0; i < send_len; i++) rc522_write_reg(0x09, send_data[i]);
    rc522_write_reg(0x01, command);
    rc522_set_bit_mask(0x0D, 0x80);

    uint32_t timeout_counter = 2000;
    uint8_t n;
    do {
        n = rc522_read_reg(0x04);
        timeout_counter--;
    } while ((timeout_counter != 0) && !(n & 0x01) && !(n & waitIRq));

    rc522_clear_bit_mask(0x0D, 0x80);
    if (timeout_counter == 0 || (rc522_read_reg(0x06) & 0x1B)) return ESP_FAIL;

    if (back_data && back_len)
    {
        uint8_t status = rc522_read_reg(0x0A);
        if (status > 0)
        {
            *back_len = status;
            for (timeout_counter = 0; timeout_counter < status; timeout_counter++)
                back_data[timeout_counter] = rc522_read_reg(0x09);
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
        for (uint8_t i = 0; i < 4; i++) uid[i] = buffer[i];
        return ESP_OK;
    }
    return ESP_FAIL;
}

void carregar_banco_dados(void)
{
    nvs_handle_t nvs_h;
    if (nvs_open("rfid_storage", NVS_READWRITE, &nvs_h) == ESP_OK)
    {
        size_t size = sizeof(master_tag);
        if (nvs_get_blob(nvs_h, "master", master_tag, &size) == ESP_OK)
        {
            has_master = true;
            ESP_LOGI(TAG, "Tag Mestra carregada.");
        }
        size = sizeof(registered_tags);
        nvs_get_blob(nvs_h, "users", registered_tags, &size);
        nvs_get_u8(nvs_h, "total", &total_tags);
        nvs_close(nvs_h);
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
        nvs_commit(nvs_h);
        nvs_close(nvs_h);
    }
}

bool comparar_uid(uint8_t *a, uint8_t *b) { return (a[0] == b[0] && a[1] == b[1] && a[2] == b[2] && a[3] == b[3]); }

void modo_cadastro(uint8_t *str, uint8_t *uid, led_strip_handle_t led_strip)
{
    ESP_LOGW(TAG, "🔵 MODO DE CADASTRO ATIVO!");
    for (int i = 0; i < LED_STRIP_LED_COUNT; i++) led_strip_set_pixel(led_strip, i, 0, 0, 24);
    led_strip_refresh(led_strip);
    vTaskDelay(pdMS_TO_TICKS(1e3));

    while (1)
    {
        esp_task_wdt_reset();
        if (rc522_request(0x26, str) == ESP_OK && rc522_anticoll(uid) == ESP_OK)
        {
            if (comparar_uid(uid, master_tag))
            {
                ESP_LOGI(TAG, "⚪ Cadastro encerrado.");
                led_strip_clear(led_strip);
                vTaskDelay(pdMS_TO_TICKS(1e3));
                return;
            }

            int index = -1;
            for (int i = 0; i < total_tags; i++)
            {
                if (comparar_uid(uid, registered_tags[i])) { index = i; break; }
            }

            if (index >= 0)
            {
                for (int i = index; i < total_tags - 1; i++) memcpy(registered_tags[i], registered_tags[i + 1], 4);
                total_tags--;
                ESP_LOGE(TAG, "🔴 TAG REMOVIDA");
                for (int i = 0; i < LED_STRIP_LED_COUNT; i++) led_strip_set_pixel(led_strip, i, 24, 0, 0);
            }
            else if (total_tags < MAX_TAGS)
            {
                memcpy(registered_tags[total_tags], uid, 4);
                total_tags++;
                ESP_LOGI(TAG, "🟢 TAG AUTORIZADA");
                for (int i = 0; i < LED_STRIP_LED_COUNT; i++) led_strip_set_pixel(led_strip, i, 0, 24, 0);
            }
            
            led_strip_refresh(led_strip);
            salvar_banco_dados();
            vTaskDelay(pdMS_TO_TICKS(1e3));
            for (int i = 0; i < LED_STRIP_LED_COUNT; i++) led_strip_set_pixel(led_strip, i, 0, 0, 24);
            led_strip_refresh(led_strip);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void)
{
    bool active_rele = false;
    led_strip_handle_t led_strip = configure_led();
    int64_t ultimo_tempo_led = esp_timer_get_time();
    int64_t intervalo = 5e6;
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 5000, .idle_core_mask = (1 << 0), .trigger_panic = true};
    esp_task_wdt_init(&twdt_config);
    esp_task_wdt_add(NULL);

    gpio_config_t conf_rele = {
        .intr_type = GPIO_INTR_DISABLE, .mode = GPIO_MODE_OUTPUT, .pin_bit_mask = (1ULL << PIN_RELE) };
    gpio_config(&conf_rele);
    gpio_set_level(PIN_RELE, !active_rele);

    // Ambos os botões configurados sem interrupção (Software Polling)
    gpio_reset_pin(BUTTON_ESTADO);
    gpio_set_direction(BUTTON_ESTADO, GPIO_MODE_INPUT);
    gpio_pullup_en(BUTTON_ESTADO);

    gpio_reset_pin(BUTTON_MASTER);
    gpio_set_direction(BUTTON_MASTER, GPIO_MODE_INPUT);
    gpio_pullup_en(BUTTON_MASTER);

    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO, .mosi_io_num = PIN_NUM_MOSI, .sclk_io_num = PIN_NUM_CLK, .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 32};
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000, .mode = 0, .spics_io_num = PIN_NUM_CS, .queue_size = 7};
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &spi_handle));

    rc522_init();
    carregar_banco_dados();

    uint8_t str[2], uid[5];
    uint32_t checagem_hardware_timer = 0;
    uint8_t led_count = 0;
    bool tag_rf = false;
    
    int64_t tempo_pressionado_master = 0;
    bool master_resetado = false;

    ESP_LOGI(TAG, "Sistema iniciado (Segure Master por 2s para limpar ID).");

    while (1)
    {
        esp_task_wdt_reset();

        // -------------------------------------------------------------
        // PROTEÇÃO: Botão Master precisa ser segurado por 2 SEGUNDOS
        // Isso bloqueia ruídos de choque, EMI ou Ground Bounce
        // -------------------------------------------------------------
        if (gpio_get_level(BUTTON_MASTER) == 0) 
        {
            if (tempo_pressionado_master == 0)
            {
                tempo_pressionado_master = esp_timer_get_time();
                master_resetado = false;
            }
            else if (!master_resetado && (esp_timer_get_time() - tempo_pressionado_master >= 2e6))
            {
                has_master = false;
                master_resetado = true;
                ESP_LOGW(TAG, "⚠️ TAG MESTRA RESETADA VIA BOTÃO MASTER!");
                
                // Pisca Laranja alertando o Reset
                for (int i = 0; i < LED_STRIP_LED_COUNT; i++) led_strip_set_pixel(led_strip, i, 24, 12, 0);
                led_strip_refresh(led_strip);
                vTaskDelay(pdMS_TO_TICKS(1000));
                led_strip_clear(led_strip);
            }
        }
        else 
        {
            tempo_pressionado_master = 0;
        }

        if (checagem_hardware_timer++ >= 7)
        {
            verificar_integridade_hardware();
            checagem_hardware_timer = 0;
        }

        if (rc522_request(0x26, str) == ESP_OK && rc522_anticoll(uid) == ESP_OK)
        {
            if (!has_master)
            {
                memcpy(master_tag, uid, 4);
                has_master = true;
                salvar_banco_dados();
                ESP_LOGI(TAG, "🏆 Tag Mestra cadastrada!");
                for (int i = 0; i < LED_STRIP_LED_COUNT; i++) led_strip_set_pixel(led_strip, i, 0, 0, 24);
                led_strip_refresh(led_strip);
                
                tag_rf = true;
                ultimo_tempo_rele = esp_timer_get_time();
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue; 
            }

            if (comparar_uid(uid, master_tag))
            {
                modo_cadastro(str, uid, led_strip);
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
            
            tag_rf = true;
            ultimo_tempo_rele = esp_timer_get_time(); 

            if (autorizado)
            {
                ESP_LOGI(TAG, "🟢 ACESSO PERMITIDO!");
                gpio_set_level(PIN_RELE, active_rele);
                for (int i = 0; i < LED_STRIP_LED_COUNT; i++) led_strip_set_pixel(led_strip, i, 0, 24, 0);
            }
            else
            {
                ESP_LOGW(TAG, "🔴 ACESSO NEGADO!");
                for (int i = 0; i < LED_STRIP_LED_COUNT; i++) led_strip_set_pixel(led_strip, i, 24, 0, 0);
            }
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        if (esp_timer_get_time() - ultimo_tempo_rele >= intervalo && tag_rf)
        {
            tag_rf = false;
            led_strip_clear(led_strip);
            ultimo_tempo_rele = esp_timer_get_time();
            gpio_set_level(PIN_RELE, !active_rele);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
        
        if (esp_timer_get_time() - ultimo_tempo_led >= 1e5 && !gpio_get_level(BUTTON_ESTADO))
        {
            led_strip_clear(led_strip);
            ultimo_tempo_led = esp_timer_get_time();
            
            led_strip_set_pixel(led_strip, led_count % 12, 6, 6, 6);
            led_strip_set_pixel(led_strip, (led_count + 1) % 12, 50, 50, 50);
            led_strip_set_pixel(led_strip, (led_count + 2) % 12, 100, 100, 100);
            led_strip_refresh(led_strip);
            
            if (++led_count >= 120) led_count = 0;
        }
    }
}