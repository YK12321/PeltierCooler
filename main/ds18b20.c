#include "ds18b20.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

// DS18B20 ROM commands
#define DS18B20_CMD_SKIP_ROM        0xCC
#define DS18B20_CMD_CONVERT_T       0x44
#define DS18B20_CMD_READ_SCRATCHPAD 0xBE
#define DS18B20_CMD_SEARCH_ROM      0xF0

// OneWire timing (in microseconds)
#define OW_RESET_PULSE      480
#define OW_PRESENCE_WAIT    70
#define OW_PRESENCE_PULSE   410
#define OW_WRITE_0          60
#define OW_WRITE_1          10
#define OW_READ_INIT        3
#define OW_READ_WAIT        10
#define OW_RECOVERY         1

static gpio_num_t ow_pin;

// OneWire low-level functions
static inline void ow_set_output(void) {
    gpio_set_direction(ow_pin, GPIO_MODE_OUTPUT);
}

static inline void ow_set_input(void) {
    gpio_set_direction(ow_pin, GPIO_MODE_INPUT);
}

static inline void ow_write_low(void) {
    gpio_set_level(ow_pin, 0);
}

static inline void ow_write_high(void) {
    gpio_set_level(ow_pin, 1);
}

static inline int ow_read(void) {
    return gpio_get_level(ow_pin);
}

static void ow_delay_us(uint32_t us) {
    ets_delay_us(us);
}

// OneWire reset
static bool ow_reset(void) {
    ow_set_output();
    ow_write_low();
    ow_delay_us(OW_RESET_PULSE);
    
    ow_set_input();
    ow_delay_us(OW_PRESENCE_WAIT);
    
    int presence = !ow_read();
    ow_delay_us(OW_PRESENCE_PULSE);
    
    return presence;
}

// Write a bit
static void ow_write_bit(int bit) {
    ow_set_output();
    ow_write_low();
    
    if (bit) {
        ow_delay_us(OW_WRITE_1);
        ow_set_input();
        ow_delay_us(OW_WRITE_0 - OW_WRITE_1);
    } else {
        ow_delay_us(OW_WRITE_0);
        ow_set_input();
    }
    
    ow_delay_us(OW_RECOVERY);
}

// Write a byte
static void ow_write_byte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        ow_write_bit(byte & 0x01);
        byte >>= 1;
    }
}

// Read a bit
static int ow_read_bit(void) {
    ow_set_output();
    ow_write_low();
    ow_delay_us(OW_READ_INIT);
    
    ow_set_input();
    ow_delay_us(OW_READ_WAIT);
    
    int bit = ow_read();
    ow_delay_us(OW_WRITE_0 - OW_READ_INIT - OW_READ_WAIT);
    
    return bit;
}

// Read a byte
static uint8_t ow_read_byte(void) {
    uint8_t byte = 0;
    
    for (int i = 0; i < 8; i++) {
        byte >>= 1;
        if (ow_read_bit()) {
            byte |= 0x80;
        }
    }
    
    return byte;
}

// DS18B20 functions
void ds18b20_init(gpio_num_t pin) {
    ow_pin = pin;
    gpio_reset_pin(pin);
    gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
    ow_set_input();
}

int ds18b20_get_device_count(gpio_num_t pin) {
    ow_pin = pin;
    if (!ow_reset()) {
        return 0;
    }
    // Simple implementation: assume 1 device if presence detected
    // Full implementation would use search ROM algorithm
    return 1;
}

void ds18b20_request_temperatures(gpio_num_t pin) {
    ow_pin = pin;
    if (!ow_reset()) {
        return;
    }
    
    ow_write_byte(DS18B20_CMD_SKIP_ROM);
    ow_write_byte(DS18B20_CMD_CONVERT_T);
}

float ds18b20_get_temp(gpio_num_t pin) {
    ow_pin = pin;
    
    if (!ow_reset()) {
        return DS18B20_ERROR;
    }
    
    ow_write_byte(DS18B20_CMD_SKIP_ROM);
    ow_write_byte(DS18B20_CMD_READ_SCRATCHPAD);
    
    uint8_t scratchpad[9];
    for (int i = 0; i < 9; i++) {
        scratchpad[i] = ow_read_byte();
    }
    
    // Check CRC (optional, simplified version)
    // For now, just read the temperature
    
    int16_t raw_temp = (scratchpad[1] << 8) | scratchpad[0];
    
    // Check for error condition
    if (raw_temp == 0x0550) {
        return DS18B20_ERROR;
    }
    
    // Convert to Celsius
    float temperature = (float)raw_temp / 16.0f;
    
    return temperature;
}
