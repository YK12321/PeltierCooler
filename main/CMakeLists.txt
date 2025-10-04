idf_component_register(
    SRCS 
        "temp_sensor.cpp"
        "ds18b20.c"
        "DS18B20Sensor.cpp"
        "FanController.cpp"
        "ServoController.cpp"
        "PIDController.cpp"
        "TemperatureControlSystem.cpp"
    INCLUDE_DIRS "."
)
