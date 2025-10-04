| Supported Targets | ESP32 |
| ----------------- | ----- |

# Directed Air Peltier Cooler Control

Embedded control firmware for a directed-air cooling system. Warm intake air is cooled by a Peltier stage, directed through a funnel, and onto the load. A DS18B20 temperature sensor mounted on the load provides feedback. The firmware modulates intake and exhaust fan speeds plus a valve servo to stabilise the load temperature at a configurable setpoint.

## Features

- Closed-loop temperature control using a PID compensator (1 Hz refresh)
- Pulse-width modulated (PWM) control for intake and exhaust fans (25 kHz LEDC)
- Servo positioning for the Peltier valve using 50 Hz PWM
- Exponential moving average smoothing on sensor data and fail-safe behaviour on sensor faults
- Structured logging over serial via ESP-IDF monitor

## Hardware Mapping

| Function            | Default GPIO | Notes                                                 |
|---------------------|--------------|-------------------------------------------------------|
| DS18B20 temperature | GPIO4        | One-wire bus with 4.7 kΩ pull-up to 3.3 V             |
| Intake fan PWM      | GPIO16       | Logic-level PWM, 25 kHz, active-high                  |
| Exhaust fan PWM     | GPIO17       | Logic-level PWM, 25 kHz, active-high                  |
| Valve servo PWM     | GPIO18       | 50 Hz PWM, 600–2400 µs pulse (adjust for your servo)  |

> 💡 Adjust the pin assignments in `main/temp_sensor.cpp` to match your wiring if necessary.

## Control Behaviour

- **Target temperature:** `22 °C` (edit `TARGET_TEMP_C` in `main/temp_sensor.cpp` to change)
- **Fans:** maintain a baseline 20% duty cycle, ramping to 100% as load temperature rises above the target
- **Servo valve:** transitions between 15° (nearly closed) and 120° (fully open) proportional to the cooling demand
- **Fail-safe:** if the DS18B20 is absent or reports an error, both fans and the valve drive to their maximum cooling state

## Build & Flash

From the project root:

```powershell
idf.py build
idf.py flash
idf.py monitor
```

Press `Ctrl+]` to exit the monitor. The console logs display the filtered temperature, raw sensor readings, PID error, fan duty cycles, and servo position each second.

## Project Structure

```
├── CMakeLists.txt                # Project entry-point
├── README.md                     # This document
└── main
	├── CMakeLists.txt            # Component build definition
	├── ds18b20.c / ds18b20.h     # Minimal one-wire driver for DS18B20
	└── temp_sensor.cpp           # Control loop, fan and servo management
```

Legacy template artifacts (`main.c`, Makefile) are kept for reference but excluded from the build.
