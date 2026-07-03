#include "uart_handler.h"
#include "espnow_helper.h"
#include "lcd_processor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "debug_control.h"
#include <string.h>
#include "lcd_commands.h"
#include "RCSwitch.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_timer.h"
#include "system_uptime.h"
#include "nvs_flash.h"
#include "nvs.h"

#define TAG "MAIN_APP"
#define RS433_TAG "RS433_TEST"
#define RECEIVER_GPIO 14
// #define REED_STATE_GPIO 33

#define UART_NUM UART_NUM_2
#define UART2_TX_PIN 17
#define UART2_RX_PIN 16

#define UART1_NUM UART_NUM_1
#define UART1_TX_PIN 1
#define UART1_RX_PIN 3

#define GPIO_INPUT_PIN 27
#define GPIO_M_PIN 33
#define CONFIG_LED_GPIO 25

#define CONFIG_BUTTON_PIN 26
#define CONFIG_MODE_TIMEOUT 6000000 // 10 seconds timeout for config mode
#define MAX_VALUES 8

RCSWITCH_t myRCSwitch;
static bool reed_state = false;
static volatile char dst = 'L';
static volatile screen_type_t my_lcd = SCREEN_L;

static int intensity_dome1 = 1;
static int intensity_dome2 = 1;
static int color_dome1 = 1;
static int color_dome2 = 1;

static bool endo_state_dome1 = false;
static bool endo_state_dome2 = false;
static bool lamp_state_dome1 = false;
static bool lamp_state_dome2 = false;
static bool boost_state_dome1 = false;
static bool boost_state_dome2 = false;

bool is_dome1_command_allowed(void);
bool is_dome2_command_allowed(void);
void send_endo_command(bool state, bool dome1);
void send_lamp_command(bool state, bool dome1);
void send_boost_command(bool state, bool dome1);
void send_intensity_command(int intensity, bool dome1);
void send_color_command(int color, bool dome1);

//--------------------------------------------------------------------------------------------------------
#define SUCCESS 0

#define UART_RX_BUF_SIZE 64

typedef struct
{
    unsigned long dec_intensity;
    unsigned long inc_intensity;
    unsigned long dec_color;
    unsigned long inc_color;
    unsigned long endo_toggle;
    unsigned long boost_toggle;
    unsigned long lamp_toggle;
    unsigned long dome_toggle;
} rf_mappings_t;

static rf_mappings_t current_mappings = {
    .dec_intensity = 16170421,
    .inc_intensity = 16170420,
    .dec_color = 16170423,
    .inc_color = 16170422,
    .endo_toggle = 16170417,
    .boost_toggle = 16170419,
    .lamp_toggle = 16170418,
    .dome_toggle = 16170424};

static rf_mappings_t new_mappings;
static bool config_mode = false;
static TickType_t config_start_tick = 0;
static int mapping_to_update = 0;
static const char *mapping_names[MAX_VALUES] = {
    "Decrease Intensity",
    "Increase Intensity",
    "Decrease Color",
    "Increase Color",
    "Endo Toggle",
    "Boost Toggle",
    "Lamp Toggle",
    "Dome Toggle"};


static esp_timer_handle_t led_timer = NULL;
static bool led_blink_state = false;

void led_on()
{
    if (led_timer)
        esp_timer_stop(led_timer);
    gpio_set_level(CONFIG_LED_GPIO, 1);
}

void led_off()
{
    if (led_timer)
        esp_timer_stop(led_timer);
    gpio_set_level(CONFIG_LED_GPIO, 0);
}

void led_blink_pattern(uint8_t count, uint32_t on_ms, uint32_t off_ms)
{
    for (uint8_t i = 0; i < count; i++)
    {
        gpio_set_level(CONFIG_LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        gpio_set_level(CONFIG_LED_GPIO, 0);
        if (i < count - 1)
            vTaskDelay(pdMS_TO_TICKS(off_ms));
    }
    // Return to steady ON state after blinking (for config mode)
    if (config_mode)
    {
        gpio_set_level(CONFIG_LED_GPIO, 1);
    }
}

void enter_config_mode()
{
    config_mode = true;
    config_start_tick = xTaskGetTickCount();
    gpio_set_level(CONFIG_LED_GPIO, 1);
    ESP_LOGI(TAG, "=== CONFIG MODE STARTED ===");
    mapping_to_update = 0;
}

static void blink_led_fast(uint8_t times)
{
    for (uint8_t i = 0; i < times; i++)
    {
        gpio_set_level(CONFIG_LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(CONFIG_LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void blink_led_success(void)
{
    for (uint8_t i = 0; i < 5; i++)
    {
        gpio_set_level(CONFIG_LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(CONFIG_LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

#define RF_MAPPINGS_VERSION 1
void initialize_default_mappings()
{
    // Set default values for current_mappings
    current_mappings = (rf_mappings_t){
        .endo_toggle = 16170417,
        .lamp_toggle = 16170418,
        .boost_toggle = 16170419,
        .inc_intensity = 16170420,
        .dec_intensity = 16170421,
        .inc_color = 16170422,
        .dec_color = 16170423,
        .dome_toggle = 16170424};
    ESP_LOGI(RS433_TAG, "Initialized default mappings");
}
void save_new_mappings()
{
    memcpy(&current_mappings, &new_mappings, sizeof(rf_mappings_t));
    ESP_LOGI(RS433_TAG, "Saving mappings to NVS...");

    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK)
    {
        err = nvs_set_u32(my_handle, "version", RF_MAPPINGS_VERSION);
        if (err != ESP_OK)
        {
            ESP_LOGE(RS433_TAG, "Failed to save version to NVS: %s", esp_err_to_name(err));
        }

        err = nvs_set_blob(my_handle, "rf_mappings", &current_mappings, sizeof(rf_mappings_t));
        if (err != ESP_OK)
        {
            ESP_LOGE(RS433_TAG, "Failed to save mappings to NVS: %s", esp_err_to_name(err));
        }
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
    else
    {
        ESP_LOGE(RS433_TAG, "Failed to open NVS handle: %s", esp_err_to_name(err));
    }
}

static void end_config_mode(bool saved)
{
    if (saved)
    {
        ESP_LOGI(RS433_TAG, "Config saved. Exiting config mode");
        led_blink_pattern(4, 100, 60);
    }
    led_off();
    config_mode = false;
    save_new_mappings();
}
void load_mappings_from_nvs()
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err == ESP_OK)
    {
        size_t required_size = sizeof(rf_mappings_t);
        uint32_t version = 0;

        err = nvs_get_u32(my_handle, "version", &version);
        if (err == ESP_OK && version == RF_MAPPINGS_VERSION)
        {
            err = nvs_get_blob(my_handle, "rf_mappings", &current_mappings, &required_size);
            if (err == ESP_OK && required_size == sizeof(rf_mappings_t))
            {
                ESP_LOGI(RS433_TAG, "Loaded mappings from NVS");
            }
            else
            {
                ESP_LOGE(RS433_TAG, "Failed to load mappings from NVS: %s", esp_err_to_name(err));
                initialize_default_mappings();
            }
        }
        else
        {
            ESP_LOGW(RS433_TAG, "No valid mappings found in NVS, using default values");
            initialize_default_mappings();
        }
        nvs_close(my_handle);
    }
    else
    {
        ESP_LOGE(RS433_TAG, "Failed to open NVS handle: %s", esp_err_to_name(err));
    }
}

static SemaphoreHandle_t mappings_mutex = NULL;

static void handle_config_mode(void)
{
    if (!config_mode)
        return;

    // timeout
    if ((xTaskGetTickCount() - config_start_tick) >= pdMS_TO_TICKS(1500000))
    {
        ESP_LOGI(RS433_TAG, "Config mode timeout");
        end_config_mode(false);
        return;
    }

    if (!available(&myRCSwitch))
        return;

    unsigned long value = getReceivedValue(&myRCSwitch);
    resetAvailable(&myRCSwitch);
    if (value == 0)
        return;

    if (mapping_to_update < 0 || mapping_to_update >= MAX_VALUES)
    {
        ESP_LOGE(RS433_TAG, "mapping_to_update out of range (%d)", mapping_to_update);
        end_config_mode(false);
        return;
    }

    // duplicate check
    bool is_duplicate = false;
    if (mappings_mutex)
        xSemaphoreTake(mappings_mutex, pdMS_TO_TICKS(200));
    unsigned long *vals = (unsigned long *)&new_mappings;
    for (int i = 0; i < MAX_VALUES; i++)
    {
        if (vals[i] == value)
        {
            is_duplicate = true;
            break;
        }
    }
    if (mappings_mutex)
        xSemaphoreGive(mappings_mutex);

    if (is_duplicate)
    {
        ESP_LOGW(RS433_TAG, "Duplicate code %lu detected", value);
        led_blink_pattern(3, 120, 80);
        led_on();
        return;
    }

    // store learned value
    if (mappings_mutex)
        xSemaphoreTake(mappings_mutex, pdMS_TO_TICKS(200));
    vals[mapping_to_update] = value;
    if (mappings_mutex)
        xSemaphoreGive(mappings_mutex);

    ESP_LOGI(RS433_TAG, "Learned %s -> %lu (index %d)",
             mapping_names[mapping_to_update], value, mapping_to_update);

    // short success
    led_blink_pattern(2, 100, 80);
    led_off();
    vTaskDelay(pdMS_TO_TICKS(250));

    mapping_to_update++;
    if (mapping_to_update < MAX_VALUES)
    {
        ESP_LOGI(RS433_TAG, "Waiting for: %s", mapping_names[mapping_to_update]);
        led_on();
        return;
    }

    // Final uniqueness pass
    bool unique = true;
    unsigned long snapshot[MAX_VALUES];
    if (mappings_mutex)
        xSemaphoreTake(mappings_mutex, pdMS_TO_TICKS(200));
    for (int i = 0; i < MAX_VALUES; ++i)
        snapshot[i] = vals[i];
    if (mappings_mutex)
        xSemaphoreGive(mappings_mutex);

    for (int i = 0; i < MAX_VALUES && unique; ++i)
    {
        for (int j = i + 1; j < MAX_VALUES; ++j)
        {
            if (snapshot[i] == snapshot[j])
            {
                unique = false;
                ESP_LOGE(RS433_TAG, "Duplicate learned mapping: %s and %s",
                         mapping_names[i], mapping_names[j]);
            }
        }
    }

    if (unique)
    {
        save_new_mappings();
        ESP_LOGI(RS433_TAG, "All mappings learned and saved");
        led_blink_pattern(4, 100, 60);
        end_config_mode(true);
    }
    else
    {
        ESP_LOGW(RS433_TAG, "Duplicates found after learning - not saved");
        led_blink_pattern(3, 150, 100);
        end_config_mode(false);
    }
}

bool is_command_allowed_for(char dome)
{
    return (dst == dome);
}

static uint8_t reset_cmd_count = 0; // Counter for consecutive reset commands

void uart_data_received(uint8_t *data, size_t len)
{
    if (!data || len == 0)
        return;

    if (DEBUG_INFO)
    {
        ESP_LOGI(TAG, "UART data received: %.*s", (int)len, (char *)data);
    }

    // --- Check for uptime reset commands ---
    uint8_t reset_packet[9] = {0x5A, 0xA5, 0x06, 0x83, 0x00, 0x99, 0x01, 0x00, 0x00};
    bool is_reset_cmd = false;

    if (len >= 9 && memcmp(data, reset_packet, 9) == 0)
    {
        is_reset_cmd = true;
    }
    else if (len < UART_RX_BUF_SIZE)
    {
        char tmp[UART_RX_BUF_SIZE];
        memcpy(tmp, data, len);
        tmp[len] = '\0';
        if (strncmp(tmp, "@RA0#", 5) == 0)
        {
            is_reset_cmd = true;
        }
    }

    // --- Reset counter logic ---
    if (is_reset_cmd)
    {
        reset_cmd_count++;
        if (reset_cmd_count >= 4)
        { // ✅ Trigger reset only after 4 consecutive commands
            ESP_LOGI(TAG, "Reset command received 4 times consecutively. Performing reset.");
            reset_system_uptime();
            reset_cmd_count = 0; // Reset counter after successful reset
        }
    }
    else
    {
        reset_cmd_count = 0; // ❌ Break sequence if any other data comes
    }

    // --- Forward to LCD processor ---
    process_lcd_data(data, len);
}

void uart1_data_received(uint8_t *data, size_t len)
{
    if (DEBUG_INFO)
    {
        ESP_LOGI(TAG, "UART data received: %.*s", (int)len, (char *)data);
    }
    if (data && len > 0)
    {
        process_lcd_data(data, len);
    }
}

void espnow_data_received(const uint8_t *data, int data_len)
{
    char received_data[20] = {0};
    strncpy(received_data, (const char *)data, data_len > (sizeof(received_data) - 1) ? (sizeof(received_data) - 1) : data_len);
    if (DEBUG_INFO)
    {
        ESP_LOGI(TAG, "Received Data (as string): %s", received_data);
    }
    process_espnow_received_data(received_data);
}

void lcd_reset()
{
    char *data_arrays[] = {
        (char[]){0x5A, 0xA5, 0x05, 0x82, 0x00, 0x02, 0x00, 0x01},
        (char[]){0x5A, 0xA5, 0x05, 0x82, 0x00, 0x0A, 0x00, 0x01},
        (char[]){0x5A, 0xA5, 0x05, 0x82, 0x00, 0x42, 0x00, 0x01},
        (char[]){0x5A, 0xA5, 0x05, 0x82, 0x00, 0x4A, 0x00, 0x01},
        (char[]){0x5A, 0xA5, 0x05, 0x82, 0x00, 0x06, 0x00, 0x01}, // red l
        (char[]){0x5A, 0xA5, 0x05, 0x82, 0x00, 0x1C, 0x00, 0x01}, // green l
        (char[]){0x5A, 0xA5, 0x05, 0x82, 0x00, 0x44, 0x00, 0x01}, // green r
        (char[]){0x5A, 0xA5, 0x05, 0x82, 0x00, 0x46, 0x00, 0x01}  // red r
    };
    for (int i = 0; i < sizeof(data_arrays) / sizeof(data_arrays[0]); i++)
    {
        uart_write_bytes(UART_NUM_2, data_arrays[i], 8);
    }
}

// Send ENDO command helper
void send_endo_command(bool state, bool dome1)
{
    char lcd_data[8] = {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x00, 0x00, 0x00};
    char command[8];
    lcd_data[5] = dome1 ? 0x07 : 0x47;
    lcd_data[7] = state ? 0x01 : 0x00;
    if (state)
        strcpy(command, "@E_1#");
    else
        strcpy(command, "@E_0#");
    uart_write_bytes(UART_NUM_2, command, strlen(command));
    ESP_LOGI(RS433_TAG, "Sent command: %s", command);
    vTaskDelay(pdMS_TO_TICKS(10));
    uart_write_bytes(UART_NUM_2, (const char *)lcd_data, sizeof(lcd_data));
}

// Send lamp command only if dome and gpio dome match
void send_lamp_command(bool state, bool dome1)
{
    char data[8];
    char command[8];
    if (dome1)
    {
        data[0] = 0x5A;
        data[1] = 0xA5;
        data[2] = 0x05;
        data[3] = 0x82;
        data[4] = 0x00;
        data[5] = 0x0A;
        data[6] = 0x00;
        data[7] = state ? 0x01 : 0x00;
    }
    else
    {
        data[0] = 0x5A;
        data[1] = 0xA5;
        data[2] = 0x05;
        data[3] = 0x82;
        data[4] = 0x00;
        data[5] = 0x4A;
        data[6] = 0x00;
        data[7] = state ? 0x01 : 0x00;
    }
    if (state)
        strcpy(command, "@L_1#");
    else
        strcpy(command, "@L_0#");
    uart_write_bytes(UART_NUM_2, command, strlen(command));
    ESP_LOGI(RS433_TAG, "Sent command: %s", command);
    vTaskDelay(pdMS_TO_TICKS(10));
    uart_write_bytes(UART_NUM_2, (const char *)data, sizeof(data));
}

// Send boost command if dome and gpio dome match
void send_boost_command(bool state, bool dome1)
{
    char data[8];
    char command[8];
    if (dome1)
    {
        data[0] = 0x5A;
        data[1] = 0xA5;
        data[2] = 0x05;
        data[3] = 0x82;
        data[4] = 0x00;
        data[5] = 0x1A;
        data[6] = 0x00;
        data[7] = state ? 0x01 : 0x00;
    }
    else
    {
        data[0] = 0x5A;
        data[1] = 0xA5;
        data[2] = 0x05;
        data[3] = 0x82;
        data[4] = 0x00;
        data[5] = 0x50;
        data[6] = 0x00;
        data[7] = state ? 0x01 : 0x00;
    }
    if (state)
        strcpy(command, "@D_1#");
    else
        strcpy(command, "@D_0#");
    uart_write_bytes(UART_NUM_2, command, strlen(command));
    ESP_LOGI(RS433_TAG, "Sent command: %s", command);
    vTaskDelay(pdMS_TO_TICKS(10));
    uart_write_bytes(UART_NUM_2, (const char *)data, sizeof(data));
}

// Send intensity command if dome and gpio dome match
void send_intensity_command(int intensity, bool dome1)
{
    if (intensity < 1)
        intensity = 1;
    if (intensity > 10)
        intensity = 10;

    char command[8];
    // Intensity command string
    if (intensity == 10)
    {
        strcpy(command, "@I0:#");
    }
    else
    {
        snprintf(command, sizeof(command), "@I%02d#", intensity);
    }
    uart_write_bytes(UART_NUM_2, command, strlen(command));
    ESP_LOGI(RS433_TAG, "Sent command: %s", command);

    // Data packet for intensity
    char data[8] = {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x00, 0x00, (char)(intensity - 1)};
    data[5] = dome1 ? 0x1E : 0x41;
    vTaskDelay(pdMS_TO_TICKS(10));
    uart_write_bytes(UART_NUM_2, (const char *)data, sizeof(data));
}

// Send color command if dome and gpio dome match
void send_color_command(int color, bool dome1)
{
    char command[6];
    char data[8] = {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x00, 0x00, 0x00};

    if (color < 1)
        color = 1;
    if (color > 3)
        color = 3;

    switch (color)
    {
    case 1:
        strcpy(command, "@C-5#");
        data[5] = dome1 ? 0x02 : 0x42;
        data[7] = 0x00;
        break;
    case 2:
        strcpy(command, "@C05#");
        data[5] = dome1 ? 0x02 : 0x42;
        data[7] = 0x01;
        break;
    case 3:
        strcpy(command, "@C+5#");
        data[5] = dome1 ? 0x02 : 0x42;
        data[7] = 0x02;
        break;
    }
    uart_write_bytes(UART_NUM_2, command, strlen(command));
    ESP_LOGI(RS433_TAG, "Sent command: %s", command);
    vTaskDelay(pdMS_TO_TICKS(10));
    uart_write_bytes(UART_NUM_2, (const char *)data, sizeof(data));
}

static void gpio_dome_select_task(void *arg)
{
    gpio_set_direction(GPIO_INPUT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(GPIO_INPUT_PIN);

    gpio_set_direction(GPIO_M_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(GPIO_M_PIN);

    gpio_set_direction(CONFIG_BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(CONFIG_BUTTON_PIN);

    bool last_button = 1;
    TickType_t last_change = 0;

    int last_r = gpio_get_level(GPIO_INPUT_PIN);
    int last_m = gpio_get_level(GPIO_M_PIN);

    // Initial dome detection
    if (last_m == 0)
    {
        my_lcd = SCREEN_M;
        dst = 'M';
    }
    else if (last_r == 0)
    {
        my_lcd = SCREEN_R;
        dst = 'R';
    }
    else
    {
        my_lcd = SCREEN_L;
        dst = 'L';
    }

    ESP_LOGI(TAG, "Initial Dome Selected: %c", dst);

    while (1)
    {
        int pin_r = gpio_get_level(GPIO_INPUT_PIN);
        int pin_m = gpio_get_level(GPIO_M_PIN);

        int button_state = gpio_get_level(CONFIG_BUTTON_PIN);

        if ((pin_r != last_r || pin_m != last_m) &&
            (xTaskGetTickCount() - last_change) > pdMS_TO_TICKS(200))
        {
            last_r = pin_r;
            last_m = pin_m;
            last_change = xTaskGetTickCount();

            if (pin_m == 0)
            {
                my_lcd = SCREEN_M;
                dst = 'M';
            }
            else if (pin_r == 0)
            {
                my_lcd = SCREEN_R;
                dst = 'R';
            }
            else
            {
                my_lcd = SCREEN_L;
                dst = 'L';
            }

            ESP_LOGI(TAG, "Dome changed via GPIO → %c", dst);
        }

        if (button_state == 0 && last_button == 1 && !config_mode)
        {
            enter_config_mode();
        }

        last_button = button_state;

        if (config_mode)
            handle_config_mode();

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
void rs433_receiver_test_task(void *arg)
{
    ESP_LOGI(RS433_TAG, "Starting RS433 receiver test task");

    static int intensity = 1;
    static int color = 2;

    static unsigned long last_value = 0;
    static int64_t last_handled_time = 0;
    const int64_t DEBOUNCE_DELAY_US = 700000; // 700ms

    initSwich(&myRCSwitch);
    enableReceive(&myRCSwitch, RECEIVER_GPIO);

    while (1)
    {
        if (available(&myRCSwitch))
        {
            unsigned long value = getReceivedValue(&myRCSwitch);
            int64_t now = esp_timer_get_time();

            if (value == 0)
            {
                resetAvailable(&myRCSwitch);
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            if (value == last_value && (now - last_handled_time) < DEBOUNCE_DELAY_US)
            {
                resetAvailable(&myRCSwitch);
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            last_value = value;
            last_handled_time = now;

            ESP_LOGI(RS433_TAG, "Received RF value: %lu", value);
            ESP_LOGI(RS433_TAG, "Current Dome Selected: %c", dst);

            bool allow_L = is_command_allowed_for('L');
            bool allow_M = is_command_allowed_for('M');
            bool allow_R = is_command_allowed_for('R');

            // Ignore commands while in config mode
            if (config_mode)
            {
                handle_config_mode();
                resetAvailable(&myRCSwitch);
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            // ----- DOME TOGGLE -----
            if (value == current_mappings.dome_toggle)
            {
                ESP_LOGI(RS433_TAG, "Dome toggle command received");

                if (dst == 'L')
                    dst = 'M';
                else if (dst == 'M')
                    dst = 'R';
                else
                    dst = 'L';

                ESP_LOGI(RS433_TAG, "Now active dome: %c", dst);
            }

            // ----- ENDO -----
            else if (value == current_mappings.endo_toggle &&
                     (allow_L || allow_M || allow_R))
            {
                bool is_dome1 = (dst == 'L');

                if (is_dome1)
                {
                    endo_state_dome1 = !endo_state_dome1;
                    send_endo_command(endo_state_dome1, true);
                }
                else
                {
                    endo_state_dome2 = !endo_state_dome2;
                    send_endo_command(endo_state_dome2, false);
                }
            }

            // ----- LAMP -----
            else if (value == current_mappings.lamp_toggle &&
                     (allow_L || allow_M || allow_R))
            {
                bool is_dome1 = (dst == 'L');

                if (is_dome1)
                {
                    lamp_state_dome1 = !lamp_state_dome1;
                    send_lamp_command(lamp_state_dome1, true);
                }
                else
                {
                    lamp_state_dome2 = !lamp_state_dome2;
                    send_lamp_command(lamp_state_dome2, false);
                }
            }

            // ----- BOOST -----
            else if (value == current_mappings.boost_toggle &&
                     (allow_L || allow_M || allow_R))
            {
                bool is_dome1 = (dst == 'L');

                if (is_dome1)
                {
                    boost_state_dome1 = !boost_state_dome1;
                    send_boost_command(boost_state_dome1, true);
                }
                else
                {
                    boost_state_dome2 = !boost_state_dome2;
                    send_boost_command(boost_state_dome2, false);
                }
            }

            // ----- INTENSITY INC -----
            else if (value == current_mappings.inc_intensity &&
                     (allow_L || allow_M || allow_R))
            {
                if (intensity < 10)
                {
                    intensity++;
                    send_intensity_command(intensity, (dst == 'L'));
                }
            }

            // ----- INTENSITY DEC -----
            else if (value == current_mappings.dec_intensity &&
                     (allow_L || allow_M || allow_R))
            {
                if (intensity > 1)
                {
                    intensity--;
                    send_intensity_command(intensity, (dst == 'L'));
                }
            }

            // ----- COLOR INC -----
            else if (value == current_mappings.inc_color &&
                     (allow_L || allow_M || allow_R))
            {
                if (color < 3)
                {
                    color++;
                    send_color_command(color, (dst == 'L'));
                }
            }

            // ----- COLOR DEC -----
            else if (value == current_mappings.dec_color &&
                     (allow_L || allow_M || allow_R))
            {
                if (color > 1)
                {
                    color--;
                    send_color_command(color, (dst == 'L'));
                }
            }

            else
            {
                ESP_LOGW(RS433_TAG, "Unhandled RF code: %lu", value);
            }

            resetAvailable(&myRCSwitch);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(5000));

    esp_reset_reason_t reason = esp_reset_reason();
    if (DEBUG_INFO)
    {
        ESP_LOGI(TAG, "Initializing application...");
    }

    // --- Initialize NVS (needed for ESP-NOW & configs) ---
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        if (DEBUG_INFO)
        {
            ESP_LOGW(TAG, "NVS flash issue detected, erasing...");
        }
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (DEBUG_INFO)
    {
        ESP_LOGI(TAG, "NVS initialized successfully");
    }

    // --- Initialize UART ---
    if (DEBUG_INFO)
    {
        ESP_LOGI(TAG, "Initializing UART...");
    }
    ESP_ERROR_CHECK(uart_init(UART_NUM, UART2_TX_PIN, UART2_RX_PIN, uart_data_received));

    // --- Initialize ESP-NOW ---
    if (DEBUG_INFO)
    {
        ESP_LOGI(TAG, "Initializing ESP-NOW...");
    }
    ESP_ERROR_CHECK(espnow_init(espnow_data_received));

    // --- Start UART task ---
    if (DEBUG_INFO)
    {
        ESP_LOGI(TAG, "Starting UART task...");
    }
    ESP_ERROR_CHECK(uart_start_task(UART_NUM, "uart_task", 4096, 5));

    // --- LCD setup ---
    set_lcd_type();
    lcd_reset();

    // --- LED setup ---
    gpio_reset_pin(CONFIG_LED_GPIO);
    gpio_set_direction(CONFIG_LED_GPIO, GPIO_MODE_OUTPUT);
    led_off();

    // --- Create mutex for RF mappings ---
    mappings_mutex = xSemaphoreCreateMutex();
    if (mappings_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mappings mutex!");
    }

    // --- Load mappings from NVS ---
    load_mappings_from_nvs();

    // --- Initialize RF receiver on correct GPIO ---
    enableReceive(&myRCSwitch, RECEIVER_GPIO);

    // --- Initialize REED GPIO (Pin 33) ---

    uptime_init(); // uptime_seconds = 0 at boot

    ESP_LOGI(TAG, "Starting tasks...");

    // Create only ONE instance of each task
    BaseType_t domeTask = xTaskCreate(
        gpio_dome_select_task,
        "gpio_dome_select_task",
        4096,
        NULL,
        5,
        NULL);

    BaseType_t rfTask = xTaskCreate(
        rs433_receiver_test_task,
        "rs433_receiver_task",
        8192,
        NULL,
        5,
        NULL);

    if (domeTask == pdPASS && rfTask == pdPASS)
    {
        ESP_LOGI(TAG, "All tasks created successfully");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to create one or more tasks!");
    }

    // --- Main loop ---
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
