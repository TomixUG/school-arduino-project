#include <Arduino.h>
#include <BleGamepad.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

BleGamepad bleGamepad;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Starting BLE work!");

    if (!accel.begin())
    {
        Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
        while (1)
            ;
    }
    accel.setRange(ADXL345_RANGE_16_G);

    BleGamepadConfiguration bleGamepadConfig;
    bleGamepadConfig.setAutoReport(false);
    bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD);
    bleGamepadConfig.setButtonCount(64);
    bleGamepadConfig.setHatSwitchCount(4);
    bleGamepadConfig.setVid(0xe502);
    bleGamepadConfig.setPid(0xabcd);
    // Some non-Windows operating systems and web based gamepad testers don't like min axis set below 0, so 0 is set by default
    // bleGamepadConfig.setAxesMin(0x8001); // -32767 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
    bleGamepadConfig.setAxesMin(0x0000); // 0 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
    bleGamepadConfig.setAxesMax(0x7FFF); // 32767 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
    bleGamepad.begin(&bleGamepadConfig); // Simulation controls, special buttons and hats 2/3/4 are disabled by default
}

void loop()
{
    sensors_event_t event;
    accel.getEvent(&event);
    if (bleGamepad.isConnected())
    {
        bleGamepad.setLeftThumb(mapf(event.acceleration.x, -10, 10, 32767, 0), 16383);
        bleGamepad.setRightThumb(16383, mapf(event.acceleration.y, -10, 10, 0, 32767));

        bleGamepad.sendReport();
    }

    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("X: ");
    Serial.print(event.acceleration.x);
    Serial.print(" ");
    Serial.print("Y: ");
    Serial.print(event.acceleration.y);
    Serial.print(" ");
    Serial.print("Z: ");
    Serial.print(event.acceleration.z);
    Serial.print(" ");
    Serial.println("m/s^2 ");
    delay(100);
}