/**
 * @file temp_sensor.cpp
 * @brief Main application entry point for directed air cooling system
 */

#include "TemperatureControlSystem.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "Main";

// Default control parameters
static constexpr float DEFAULT_TEMPERATURE_C = 22.0f;
static constexpr float DEFAULT_ERROR_MARGIN_C = 0.5f;  // ±0.5°C margin
static constexpr uint32_t CONTROL_PERIOD_MS = 1000;

/**
 * @brief Main control task
 */
static void control_task(void* param)
{
    auto* system = static_cast<TemperatureControlSystem*>(param);
    
    ESP_LOGI(TAG, "Control task started");
    
    while (true) {
        system->update();
        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== Peltier Cooler Control System ===");
    ESP_LOGI(TAG, "Target Temperature: %.2f °C ± %.2f °C", DEFAULT_TEMPERATURE_C, DEFAULT_ERROR_MARGIN_C);
    
    // Create the temperature control system with default settings
    static TemperatureControlSystem controlSystem(DEFAULT_TEMPERATURE_C, CONTROL_PERIOD_MS, DEFAULT_ERROR_MARGIN_C);
    
    // Initialize all hardware
    if (!controlSystem.begin()) {
        ESP_LOGE(TAG, "Failed to initialize control system");
        return;
    }
    
    ESP_LOGI(TAG, "System initialized successfully");
    
    // Create control task
    BaseType_t result = xTaskCreate(
        control_task,
        "control_task",
        4096,
        &controlSystem,
        5,
        nullptr
    );
    
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create control task");
        return;
    }
    
    ESP_LOGI(TAG, "System running...");
}
