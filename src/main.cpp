#include <Arduino.h>
#include <BleGamepad.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

BleGamepad bleGamepad;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

#define RIGHT_UP 0
#define RIGHT_DOWN 15
#define LEFT_UP 32
#define LEFT_DOWN 25

#define SWITCH 2
#define LED 33

bool mode = false;
// A (false) - normal
// B (true) - gaming

bool keyStates[4] = {false, false, false, false};            // temporary storage for the key states
int keyPins[4] = {RIGHT_UP, RIGHT_DOWN, LEFT_UP, LEFT_DOWN}; // the key pins on the microcontroller
uint8_t keyCodesA[4] = {32, 33, 27, 14};                     // keyCodes for mode A
uint8_t keyCodesB[4] = {18, 5, 17, 16};                      // keyCodes for mode B

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void handleButton(int keyIndex)
{
    // handle the button press
    if (!digitalRead(keyPins[keyIndex]))
    {
        // button pressed
        if (!keyStates[keyIndex])
        {
            // key not currently pressed
            keyStates[keyIndex] = true;
            bleGamepad.press(mode == false ? keyCodesA[keyIndex] : keyCodesB[keyIndex]);
            bleGamepad.sendReport();
        }
    }
    else
    {
        // button not pressed
        if (keyStates[keyIndex])
        {
            // key currently pressed
            keyStates[keyIndex] = false;
            bleGamepad.release(mode == false ? keyCodesA[keyIndex] : keyCodesB[keyIndex]);
            bleGamepad.sendReport();
        }
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(RIGHT_UP, INPUT_PULLUP);
    pinMode(RIGHT_DOWN, INPUT_PULLUP);
    pinMode(LEFT_UP, INPUT_PULLUP);
    pinMode(LEFT_DOWN, INPUT_PULLUP);

    pinMode(SWITCH, INPUT_PULLUP);

    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);

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
        // switching the modes
        if (digitalRead(SWITCH) == HIGH)
            mode = false;
        else
            mode = true;

        // the joystick
        if (mode == false)
            bleGamepad.setRightThumb(mapf(event.acceleration.x, -10.3, 10.8, 32767, 0), mapf(event.acceleration.y, -11, 9.6, 0, 32767));
        else
            bleGamepad.setLeftThumb(mapf(event.acceleration.x, -10.3, 10.8, 32767, 0), mapf(event.acceleration.y, -11, 9.6, 0, 32767));

        bleGamepad.sendReport();

        // the buttons
        for (int counter = 0; counter < 4; counter++)
        {
            handleButton(counter);
        }
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
}