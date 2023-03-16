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

unsigned long ledPreviousMillis = 0; // will store last time LED was updated
const long ledInterval = 500;        // interval at which to blink (milliseconds)
int ledState = LOW;                  // stores wheter the led is on or off

enum Mode
{
    CONTROL,
    GAMING
};
Mode mode = CONTROL;

bool keyStates[4] = {false, false, false, false};                // temporary storage for the key states
int keyPins[4] = {RIGHT_UP, RIGHT_DOWN, LEFT_UP, LEFT_DOWN};     // the key pins on the microcontroller
uint8_t keyCodesA[4] = {BUTTON_2, BUTTON_1, BUTTON_4, BUTTON_5}; // keyCodes for mode A
uint8_t keyCodesB[4] = {18, 5, 17, 16};                          // keyCodes for mode B

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
            bleGamepad.press(mode == CONTROL ? keyCodesA[keyIndex] : keyCodesB[keyIndex]);
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
            bleGamepad.release(mode == CONTROL ? keyCodesA[keyIndex] : keyCodesB[keyIndex]);
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
    bleGamepadConfig.setAxesMin(0x0000);
    bleGamepadConfig.setAxesMax(0x7FFF);
    bleGamepad.begin(&bleGamepadConfig);
}

void loop()
{
    sensors_event_t event;
    accel.getEvent(&event);

    if (bleGamepad.isConnected())
    {
        // if device is connected turn the led on
        digitalWrite(LED, HIGH);

        // the joystick
        if (digitalRead(SWITCH) == LOW)
        {
            mode = CONTROL;
            bleGamepad.setLeftThumb(mapf(event.acceleration.x, -10.3, 10.8, 32767, 0), mapf(event.acceleration.y, -11, 9.6, 0, 32767));
            bleGamepad.setRightThumb(16383, 16383); // reset the other joystick to be in  the midle
        }
        else
        {
            mode = GAMING;
            bleGamepad.setRightThumb(mapf(event.acceleration.x, -10.3, 10.8, 32767, 0), mapf(event.acceleration.y, -11, 9.6, 0, 32767));
            bleGamepad.setLeftThumb(16383, 16383); // reset the other joystick to be in the midle
        }

        bleGamepad.sendReport();

        // the buttons
        for (int counter = 0; counter < 4; counter++)
        {
            handleButton(counter);
        }
    }
    else
    {
        // if device is not connected, blink the LED
        unsigned long currentMillis = millis();
        if (currentMillis - ledPreviousMillis >= ledInterval)
        {
            ledPreviousMillis = currentMillis; // save the last time the led blinked

            // if the LED is off turn it on and vice-versa:
            if (ledState == LOW)
                ledState = HIGH;
            else
                ledState = LOW;

            // set the LED with the ledState of the variable:
            digitalWrite(LED, ledState);
        }
    }

    // show accelerometer data
    Serial.println("X: " + String(event.acceleration.x) + "   Y: " + String(event.acceleration.y) + "   Z: " + String(event.acceleration.z));
}