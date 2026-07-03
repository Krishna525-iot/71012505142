#include "lcd_commands.h"
#include "lcd_processor.h"
#include "esp_log.h"
#include "espnow_helper.h"
#include <string.h>
#include "debug_control.h"
#include "uart_handler.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "system_uptime.h"

extern void reset_system_uptime(void);

#define TAG "LCD_PROCESSOR"
#define DATA_SIZE 9

screen_type_t my_lcd = SCREEN_L;
uint8_t dst = 'L';
uint8_t isync = '0';

static bool is_remote_command = false;

static const uint8_t laser_on_data[] = {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x5A, 0x00, 0x01};
static const uint8_t laser_off_data[] = {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x5A, 0x00, 0x00};

void process_laser_command(const char *received_data)
{
    if (!received_data)
        return;

    if (strncmp(received_data, "@PA1#", 5) == 0)
    {
        ESP_LOGI(TAG, "[LASER] Turning ON");
        uart_write_bytes(UART_NUM_2, (const char *)laser_on_data, sizeof(laser_on_data));
    }
    else if (strncmp(received_data, "@PA0#", 5) == 0)
    {
        ESP_LOGI(TAG, "[LASER] Turning OFF");
        uart_write_bytes(UART_NUM_2, (const char *)laser_off_data, sizeof(laser_off_data));
    }
}

void set_lcd_type(void)
{
    gpio_set_direction(27, GPIO_MODE_INPUT);
    gpio_pullup_en(27);

    gpio_set_direction(33, GPIO_MODE_INPUT);
    gpio_pullup_en(33);

    if (gpio_get_level(33) == 0)
    {
        my_lcd = SCREEN_M;
        dst = 'M';
    }
    else if (gpio_get_level(27) == 0)
    {
        my_lcd = SCREEN_R;
        dst = 'R';
    }
    else
    {
        my_lcd = SCREEN_L;
        dst = 'L';
    }

    ESP_LOGI(TAG, "[INIT] This dome detected as: SCREEN_%c", dst);
}

void send_to_uart(int uart_num, const char *data)
{
    if (!data)
        return;

    ssize_t w = uart_write_bytes((uart_port_t)uart_num, data, strlen(data));
    if (w < 0)
    {
        ESP_LOGE(TAG, "UART write failed (uart %d)", uart_num);
    }
}

static void update_lcd_sync_ui(bool sync_on)
{
    const char *cmd_str = sync_on ? "@N_1#" : "@N_0#";
    char screen_char = (my_lcd == SCREEN_L) ? 'L' : (my_lcd == SCREEN_M) ? 'M'
                                                                         : 'R';

    for (int i = 0; i < reverse_command_map_size; i++)
    {
        if (strncmp(reverse_command_map[i].command, cmd_str, 5) == 0 &&
            reverse_command_map[i].screen == my_lcd)
        {
            // Write 1: primary write
            uart_write_bytes(UART_NUM_2,
                             (const char *)reverse_command_map[i].data, 8);
            vTaskDelay(pdMS_TO_TICKS(20));

            // Write 2: confirmation write to ensure LCD rendered it
            uart_write_bytes(UART_NUM_2,
                             (const char *)reverse_command_map[i].data, 8);

            // Flush the 2 ACKs (2 × 6 bytes = 12 bytes) that DWIN sent
            // back for the two writes above. Without this they sit in the
            // RX ring buffer and misalign every subsequent UART read,
            // producing "[WARN] Short binary packet ignored" spam.
            vTaskDelay(pdMS_TO_TICKS(15)); // let DWIN finish sending ACK #2
            uart_flush_input(UART_NUM_2);

            ESP_LOGI(TAG, "[SYNC UI] Screen %c updated → SYNC %s (sent x2, ACKs flushed)",
                     screen_char, sync_on ? "ON" : "OFF");
            return;
        }
    }

    ESP_LOGW(TAG, "[SYNC UI] WARN: No map entry for screen %c sync %s",
             screen_char, sync_on ? "ON" : "OFF");
}

void lcd_reset_synk()
{
    ESP_LOGI(TAG, "[SYNC RESET] Resetting LCD states...");

    uint8_t data_arrays[][8] = {

        // ---------- INTENSITY ----------
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x1E, 0x00, 0x00}, // Intensity L
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x41, 0x00, 0x00}, // Intensity R
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x70, 0x00, 0x00}, // Intensity M

        // ---------- COLOR ----------
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x02, 0x00, 0x01}, // Color L
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x42, 0x00, 0x01}, // Color R
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x71, 0x00, 0x01}, // Color M

        // ---------- LAMP OFF ----------
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x0A, 0x00, 0x01}, // Lamp L OFF
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x4A, 0x00, 0x01}, // Lamp R OFF
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x79, 0x00, 0x01}, // Lamp M OFF

        // ---------- ENDO OFF ----------
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x07, 0x00, 0x00}, // Endo L OFF
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x47, 0x00, 0x00}, // Endo R OFF
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x76, 0x00, 0x00}, // Endo M OFF

        // ---------- DEPTH / BOOST OFF ----------
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x1A, 0x00, 0x00}, // Depth L OFF
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x50, 0x00, 0x00}, // Depth R OFF
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x7E, 0x00, 0x00}, // Depth M OFF
        // ---------- Focus OFF ----------
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x48, 0x00, 0x00}, // focus R off
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x08, 0x00, 0x00}, // focus L off
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x77, 0x00, 0x00}, // focus M off
        // ---------- LASER OFF ----------
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x5A, 0x00, 0x00}, // Laser L OFF
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x5B, 0x00, 0x00}, // Laser R/M OFF


        // ---------- RED INTENSITY RESET ----------
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x05, 0x00, 0x00}, // Red intensity L → 0
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x45, 0x00, 0x00}, // Red intensity R → 0
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x74, 0x00, 0x00}, // Red intensity M → 0

        // ---------- GREEN INTENSITY RESET ----------
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x03, 0x00, 0x00}, // Green intensity L → 0
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x43, 0x00, 0x00}, // Green intensity R → 0
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x72, 0x00, 0x00}, // Green intensity M → 0

        // ---------- OVERHEAD OFF ----------
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x3C, 0x00, 0x00}, // Overhead L OFF
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x4C, 0x00, 0x00}, // Overhead R OFF
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x7A, 0x00, 0x00}, // Overhead M OFF
        
        // ---------- RGB RESET ----------
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x06, 0x00, 0x01}, // Red L OFF
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x1C, 0x00, 0x01}, // Green L OFF
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x44, 0x00, 0x01}, // Green R OFF
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x46, 0x00, 0x01}, // Red R OFF
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x73, 0x00, 0x01}, // Green M OFF
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x75, 0x00, 0x01}, // Red M OFF

        
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x6A, 0x00, 0x00}, // Sync L clear → 0
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x6B, 0x00, 0x00}, // Sync R clear → 0
        {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x6C, 0x00, 0x00}, // Sync M clear → 0
    };

    int num_entries = sizeof(data_arrays) / sizeof(data_arrays[0]);

    for (int i = 0; i < num_entries; i++)
    {
        uart_write_bytes(UART_NUM_2, (const char *)data_arrays[i], 8);
        vTaskDelay(pdMS_TO_TICKS(40));
    }
    uart_flush_input(UART_NUM_2);

    ESP_LOGI(TAG, "[SYNC RESET] Done (%d packets sent, RX flushed)", num_entries);
}

// =====================================================
// MAIN LCD PROCESSOR
// =====================================================
void process_lcd_data(uint8_t *data, size_t len)
{
    if (!data || len == 0)
        return;

    // =========================
    // ASCII PACKET
    // =========================
    if (len >= 5 && data[0] == '@')
    {
        char rx[32] = {0};
        size_t copy_len = (len < 31) ? len : 31;
        memcpy(rx, data, copy_len);
        rx[copy_len] = '\0';

        ESP_LOGI(TAG, "[LOCAL ASCII] %s", rx);
        process_espnow_received_data(rx);
        return;
    }

    if (len < 9)
    {
        // ESP_LOGW(TAG, "[WARN] Short binary packet ignored (len=%d)", (int)len);
        return;
    }

    ESP_LOG_BUFFER_HEX(TAG, data, len);

    for (int i = 0; i < lcd_command_map_size; i++)
    {
        if (data[5] == lcd_command_map[i].data[5] &&
            data[8] == lcd_command_map[i].data[8] &&
            lcd_command_map[i].screen == my_lcd)
        {
            char screen_char =
                (lcd_command_map[i].screen == SCREEN_L) ? 'L' : (lcd_command_map[i].screen == SCREEN_M) ? 'M'
                                                                                                        : 'R';

            bool is_sync_cmd = false;
            if (strncmp(lcd_command_map[i].command, "@N_1#", 5) == 0)
            {
                isync = '1';
                is_sync_cmd = true;
                ESP_LOGI(TAG, "[SYNC] Local press: SYNC ON  (screen %c)", screen_char);
            }
            else if (strncmp(lcd_command_map[i].command, "@N_0#", 5) == 0)
            {
                isync = '0';
                is_sync_cmd = true;
                ESP_LOGI(TAG, "[SYNC] Local press: SYNC OFF (screen %c)", screen_char);
            }

            char formatted[32];
            snprintf(formatted, sizeof(formatted),
                     "%sW%c%c",
                     lcd_command_map[i].command,
                     screen_char,
                     isync);

            ESP_LOGI(TAG, "[LOCAL CMD] %s  (isync=%c  screen=%c)",
                     formatted, isync, screen_char);

            if (is_sync_cmd)
            {
                lcd_reset_synk();
                update_lcd_sync_ui(isync == '1');
            }
            else if (isync == '1')
            {
                // In sync mode, apply the command to ALL local screens
                // to match what remote domes do via ESP-NOW.
                ESP_LOGI(TAG, "[SYNC MODE LOCAL] Applying '%.*s' to all local screens", 5, formatted);
                process_espnow_received_data(formatted);
            }

            if (!is_remote_command)
            {
                if (isync == '1' && !is_sync_cmd)
                {
                    // Sync mode: broadcast for all three domes so each
                    // remote dome receives its own targeted packet.
                    const char domes[] = {'L', 'M', 'R'};
                    for (int d = 0; d < 3; d++)
                    {
                        char pkt[32];
                        snprintf(pkt, sizeof(pkt), "%sW%c1",
                                 lcd_command_map[i].command, domes[d]);
                        espnow_send((uint8_t *)pkt, strlen(pkt));
                        ESP_LOGI(TAG, "[ESPNOW TX] Broadcasted: %s", pkt);
                    }
                }
                else
                {
                    espnow_send((uint8_t *)formatted, strlen(formatted));
                    ESP_LOGI(TAG, "[ESPNOW TX] Broadcasted: %s", formatted);
                }
            }

            return;
        }
    }

    ESP_LOGW(TAG, "[WARN] No command match  data[5]=0x%02X  data[8]=0x%02X",
             data[5], data[8]);
}

void process_espnow_received_data(const char *rx)
{
    if (!rx)
        return;

    size_t len = strlen(rx);
    if (len < 5)
        return;

    char screen_char = (my_lcd == SCREEN_L) ? 'L' : (my_lcd == SCREEN_M) ? 'M'
                                                                         : 'R';

    // ---- SYNC ON ----
    if (strncmp(rx, "@N_1#", 5) == 0)
    {
        isync = '1';
        ESP_LOGI(TAG, "@N_1#");
        lcd_reset_synk();
        update_lcd_sync_ui(true);
        return;
    }

    // ---- SYNC OFF ----
    if (strncmp(rx, "@N_0#", 5) == 0)
    {
        isync = '0';
        ESP_LOGI(TAG, "@N_0#");
        lcd_reset_synk();
        update_lcd_sync_ui(false);
        return;
    }

    // ---- NORMAL MODE: ignore all remote commands ----
    if (isync == '0')
    {
        char dome = (len > 6) ? rx[6] : '?';
        // Only allow if the command originated from our own dome (local call)
        if (dome != screen_char)
        {
            // ESP_LOGI(TAG, "[NORMAL MODE] Ignoring remote cmd '%.*s' from dome %c (we are %c, sync OFF)",
            //          5, rx, dome, screen_char);
            return;
        }

        // Local-origin command in normal mode — apply to our screen only
        screen_type_t target = my_lcd;
        for (int i = 0; i < reverse_command_map_size; i++)
        {
            if (strncmp(rx, reverse_command_map[i].command, 5) == 0 &&
                reverse_command_map[i].screen == target)
            {
                uart_write_bytes(UART_NUM_2,
                                 (const char *)reverse_command_map[i].data, 8);
                vTaskDelay(pdMS_TO_TICKS(15));
                uart_flush_input(UART_NUM_2);
                ESP_LOGI(TAG, "[NORMAL MODE] → Screen %c applied (ACKs flushed)", screen_char);
                return;
            }
        }
        ESP_LOGW(TAG, "[WARN] No reverse map match for '%.*s' on screen %c", 5, rx, screen_char);
        return;
    }

    // ---- SYNC MODE: apply to all three screens ----
    ESP_LOGI(TAG, "[SYNC MODE] '%.*s'  →  applying to L + M + R", 5, rx);

    for (int s = 0; s < 3; s++)
    {
        screen_type_t scr = (s == 0) ? SCREEN_L : (s == 1) ? SCREEN_M
                                                           : SCREEN_R;
        char sc = (s == 0) ? 'L' : (s == 1) ? 'M'
                                            : 'R';
        bool applied = false;

        for (int i = 0; i < reverse_command_map_size; i++)
        {
            if (strncmp(rx, reverse_command_map[i].command, 5) == 0 &&
                reverse_command_map[i].screen == scr)
            {
                uart_write_bytes(UART_NUM_2,
                                 (const char *)reverse_command_map[i].data, 8);
                applied = true;
            }
        }

        if (!applied)
        {
            ESP_LOGW(TAG, "[SYNC MODE] No reverse map match for '%.*s' on screen %c",
                     5, rx, sc);
        }
        else
        {
            ESP_LOGI(TAG, "[SYNC MODE]  → Screen %c applied", sc);
        }
    }

    // Flush all ACKs after the batch write
    vTaskDelay(pdMS_TO_TICKS(20));
    uart_flush_input(UART_NUM_2);
}