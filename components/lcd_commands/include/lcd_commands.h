#ifndef LCD_COMMANDS_H
#define LCD_COMMANDS_H

#include <stdint.h>
#include <stddef.h> // For size_t

// ------------------------------------------------------------
// Updated screen types to support THREE domes: L / M / R
// ------------------------------------------------------------
typedef enum {
    SCREEN_L = 0,   // Left screen
    SCREEN_M = 1,   // Middle screen  (NEW)
    SCREEN_R = 2    // Right screen
} screen_type_t;

// ------------------------------------------------------------
// LCD command structure
// ------------------------------------------------------------
typedef struct {
    uint8_t data[9];        // Raw data pattern
    const char *command;    // Corresponding command string
    screen_type_t screen;   // Screen type
} lcd_command_t;

// ------------------------------------------------------------
// External declarations
// ------------------------------------------------------------
extern lcd_command_t lcd_command_map[];
extern lcd_command_t reverse_command_map[];

extern const size_t lcd_command_map_size;
extern const size_t reverse_command_map_size;

#endif // LCD_COMMANDS_H
