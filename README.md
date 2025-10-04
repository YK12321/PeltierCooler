# 🌡️ Peltier Cooler Control System

[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.0-blue.svg)](https://docs.espressif.com/projects/esp-idf/en/v5.0/)
[![Platform](https://img.shields.io/badge/Platform-ESP32-green.svg)](https://www.espressif.com/en/products/socs/esp32)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

An embedded temperature control system for precision cooling applications using a Peltier module, dual-fan airflow management, and servo-controlled valve. Built with modern C++ design patterns on ESP-IDF.

## 🎯 Overview

This project implements a **closed-loop PID temperature controller** for a directed air cooling system. Warm air is drawn through a Peltier cooler, directed via a servo-controlled valve, and exhausted to maintain precise temperature regulation on the target load.

### System Architecture

```
   Warm Air → [Intake Fan] → [Peltier Module] → [Servo Valve] → Load
                                                      ↓
                                                [Exhaust Fan]
                                                      ↓
                                              [DS18B20 Sensor]
```

## ✨ Key Features

- **Precision Control**: PID algorithm with configurable gains (Kp=0.9, Ki=0.08, Kd=0.1)
- **Object-Oriented Design**: Modular C++ classes for sensors, actuators, and controllers
- **Robust Operation**: Exponential moving average filtering and automatic fail-safe mode
- **Hardware PWM**: 25 kHz fan control and 50 Hz servo positioning using ESP32 LEDC
- **Real-time Monitoring**: Serial logging at 115200 baud with temperature, control output, and actuator states

## 🔧 Hardware Requirements

| Component | Specification | Notes |
|-----------|--------------|-------|
| **Microcontroller** | ESP32 DevKit | Any ESP32 module supported by ESP-IDF v5.0+ |
| **Temperature Sensor** | DS18B20 | Digital one-wire sensor with 4.7 kΩ pull-up resistor |
| **Fans** | 2× 12V PWM-capable | Intake and exhaust (PWM control via MOSFET/driver) |
| **Servo** | Standard RC servo | 5V, 50 Hz PWM (e.g., SG90, MG996R) |
| **Peltier Module** | TEC1-12706 or similar | Requires heatsink and thermal compound |
| **Power Supply** | 12V 5A+ | For Peltier and fans (separate 5V for servo) |

### Wiring Diagram

| Function | GPIO Pin | Connection Details |
|----------|----------|-------------------|
| **DS18B20 Data** | GPIO4 | One-wire bus with 4.7 kΩ pull-up to 3.3V |
| **Intake Fan PWM** | GPIO16 | Via N-channel MOSFET (e.g., IRLZ44N) |
| **Exhaust Fan PWM** | GPIO17 | Via N-channel MOSFET (e.g., IRLZ44N) |
| **Valve Servo PWM** | GPIO18 | Direct connection (5V servo) |

> ⚠️ **Important**: Never connect fans or Peltier modules directly to ESP32 GPIO pins. Use appropriate drivers/relays rated for your components.

## � Software Architecture

The firmware uses a **layered, object-oriented architecture** for maintainability and testability:

```
┌─────────────────────────────────────┐
│   temp_sensor.cpp (Main Entry)     │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│  TemperatureControlSystem           │  ◄── Orchestrator
│  - Manages control loop             │
│  - Coordinates all components       │
└──┬────────┬────────┬────────┬───────┘
   │        │        │        │
   ▼        ▼        ▼        ▼
┌────────┬────────┬────────┬───────────┐
│DS18B20 │Fan     │Servo   │PID        │  ◄── Component Classes
│Sensor  │Ctrl    │Ctrl    │Controller │
└────────┴────────┴────────┴───────────┘
          │        │
          ▼        ▼
    ┌──────────────────┐
    │ ESP-IDF HAL      │  ◄── Hardware Abstraction
    │ (LEDC, GPIO)     │
    └──────────────────┘
```

### Class Responsibilities

- **`DS18B20Sensor`**: Temperature measurement and sensor health monitoring
- **`FanController`**: PWM-based fan speed control with duty cycle management
- **`ServoController`**: Servo angle positioning with microsecond-level pulse control
- **`PIDController`**: Proportional-Integral-Derivative feedback computation
- **`TemperatureControlSystem`**: High-level orchestration and control loop execution

## 🚀 Getting Started

### Prerequisites

1. **ESP-IDF v5.0+** installed ([Installation Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/))
2. **Python 3.7+** (included with ESP-IDF)
3. **USB-to-Serial driver** for your ESP32 board

### Quick Start

1. **Clone the repository**
   ```bash
   git clone https://github.com/YK12321/PeltierCooler.git
   cd PeltierCooler/PeltierCoolerProject
   ```

2. **Configure ESP-IDF environment**
   ```bash
   # Windows PowerShell
   . $HOME/esp/esp-idf/export.ps1
   
   # Linux/macOS
   . $HOME/esp/esp-idf/export.sh
   ```

3. **Build the project**
   ```bash
   idf.py build
   ```

4. **Flash to ESP32**
   ```bash
   idf.py -p COM5 flash monitor  # Replace COM5 with your port
   ```

5. **Monitor output**
   - Press `Ctrl+]` to exit monitor
   - Expected log format:
     ```
     I (1234) TempControl: T=22.15°C (raw 22.20°C), err=0.15, intake=25%, exhaust=25%, valve=42.3°
     ```

## ⚙️ Configuration

### Adjusting Target Temperature

Edit `main/temp_sensor.cpp`:

```cpp
static constexpr float TARGET_TEMPERATURE_C = 22.0f;  // Change to desired setpoint
```

### Tuning PID Gains

Modify constants in `main/TemperatureControlSystem.cpp`:

```cpp
static constexpr float PID_KP = 0.9f;   // Proportional gain
static constexpr float PID_KI = 0.08f;  // Integral gain
static constexpr float PID_KD = 0.1f;   // Derivative gain
```

**Tuning Tips:**
- Increase Kp for faster response (may cause oscillation)
- Increase Ki to eliminate steady-state error
- Increase Kd to reduce overshoot

### Changing GPIO Pins

Update pin definitions in `main/TemperatureControlSystem.cpp`:

```cpp
static constexpr gpio_num_t PIN_TEMP_SENSOR   = GPIO_NUM_4;
static constexpr gpio_num_t PIN_FAN_INTAKE    = GPIO_NUM_16;
static constexpr gpio_num_t PIN_FAN_EXHAUST   = GPIO_NUM_17;
static constexpr gpio_num_t PIN_SERVO_VALVE   = GPIO_NUM_18;
```

## 📂 Project Structure

```
PeltierCoolerProject/
├── CMakeLists.txt                      # Top-level build configuration
├── sdkconfig                           # ESP-IDF configuration
├── README.md                           # This file
└── main/
    ├── CMakeLists.txt                  # Component build file
    ├── temp_sensor.cpp                 # Application entry point
    ├── TemperatureControlSystem.h/cpp  # Main control orchestrator
    ├── DS18B20Sensor.h/cpp             # Temperature sensor interface
    ├── FanController.h/cpp             # Fan PWM controller
    ├── ServoController.h/cpp           # Servo positioning
    ├── PIDController.h/cpp             # PID algorithm
    └── ds18b20.h/c                     # Low-level OneWire driver
```

## 🛡️ Safety Features

- **Fail-Safe Mode**: Activates maximum cooling if sensor fails
- **Anti-Windup**: PID integral clamping prevents runaway
- **Input Validation**: All actuator commands are range-checked
- **Sensor Monitoring**: Continuous health checks on DS18B20

## 🔍 Troubleshooting

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| "Sensor disconnected" warnings | Wiring issue, missing pull-up | Check 4.7 kΩ resistor on GPIO4 |
| Fans not responding | MOSFET not switching | Verify gate voltage, check PWM signal |
| Servo jittering | Insufficient power | Use separate 5V supply for servo |
| Temperature oscillating | PID tuning | Reduce Kp, increase Kd |
| Build errors | ESP-IDF version mismatch | Use ESP-IDF v5.0 or later |

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request. For major changes, please open an issue first to discuss what you would like to change.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- Built with [ESP-IDF](https://github.com/espressif/esp-idf) by Espressif Systems
- DS18B20 driver inspired by community OneWire implementations
- PID control theory references from [Control Systems Engineering](https://www.wiley.com/en-us/Control+Systems+Engineering%2C+8th+Edition-p-9781119474227)

## 📧 Contact

For questions or support, please open an issue on GitHub.

---

**⭐ If this project helped you, consider giving it a star!**