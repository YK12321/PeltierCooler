/**
 * @file temp_sensor.cpp
 * @brief Main application entry point for directed air cooling system
 */

#include "TemperatureControlSystem.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "Main";

// Control parameters
static constexpr float TARGET_TEMPERATURE_C = 22.0f;
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
    
    // Create the temperature control system
    static TemperatureControlSystem controlSystem(TARGET_TEMPERATURE_C, CONTROL_PERIOD_MS);
    
    // Initialize all hardware
    if (!controlSystem.begin()) {
        ESP_LOGE(TAG, "Failed to initialize control system");
        return;
    }
    
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
