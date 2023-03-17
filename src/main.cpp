#include <Arduino.h>
#include <BleGamepad.h> // knihovna na komunikaci s telefonem přes bluetooth
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h> // knihovna na získávání dat z akcelerometru

// definování pinů periferií
#define RIGHT_UP 0
#define RIGHT_DOWN 15
#define LEFT_UP 32
#define LEFT_DOWN 25
#define SWITCH 2
#define LED 33

/* iniciliace knihovnen */
BleGamepad bleGamepad;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

/* proměnné pro blikání LED */
unsigned long ledPreviousMillis = 0; // před jakou dobou LEDka naposledy blikla
const long ledInterval = 500;        // interval blikání LED
int ledState = LOW;                  // ukládá stav LEDky (zaplá/vypnutá)

/* enum indikující aktuální mód */
enum Mode
{
    CONTROL,
    GAMING
};
Mode mode = CONTROL;

bool keyStates[4] = {false, false, false, false};                // jaká tlačítka momentálně jsou stisknuta
int keyPins[4] = {RIGHT_UP, RIGHT_DOWN, LEFT_UP, LEFT_DOWN};     // fyzické piny tlačítek
uint8_t keyCodesA[4] = {BUTTON_2, BUTTON_1, BUTTON_4, BUTTON_5}; // kódy tlačítek pro mód A
uint8_t keyCodesB[4] = {18, 5, 17, 16};                          // kódy tlačítek pro mód B

/* map funkce s float proměnnými, ať můžu používat desetinná místa */
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* funkce pro zpracování stisku tlačítek */
void handleButton(int keyIndex)
{
    if (digitalRead(keyPins[keyIndex]) == HIGH)
    {
        // tlačítko není stisknuto
        // zkontrolujeme jestli změnilo stav, pokud ano, odešleme zprávu o puštění tlačítka
        if (keyStates[keyIndex])
        {
            keyStates[keyIndex] = false; // zapíšeme nový stav
            // podle módu zvolíme kód tlačítka
            bleGamepad.release(mode == CONTROL ? keyCodesA[keyIndex] : keyCodesB[keyIndex]);
            bleGamepad.sendReport(); // odešleme zprávu
        }
    }
    else
    {
        // tlačítko je stisknuto
        // zkontrolujeme jestli změnilo stav, pokud ano, odešleme zprávu o stisknutí tlačítka
        if (!keyStates[keyIndex])
        {
            keyStates[keyIndex] = true; // zapíšeme nový stav
            // podle módu zvolíme kód tlačítka
            bleGamepad.press(mode == CONTROL ? keyCodesA[keyIndex] : keyCodesB[keyIndex]);
            bleGamepad.sendReport(); // odešleme zprávu
        }
    }
}

void setup()
{
    Serial.begin(115200); // inicializace seriového portu

    /* nastavení módu pinů */
    pinMode(RIGHT_UP, INPUT_PULLUP);
    pinMode(RIGHT_DOWN, INPUT_PULLUP);
    pinMode(LEFT_UP, INPUT_PULLUP);
    pinMode(LEFT_DOWN, INPUT_PULLUP);
    pinMode(SWITCH, INPUT_PULLUP);
    pinMode(LED, OUTPUT);

    /* inicializace akcelerometru */
    if (!accel.begin())
    {
        Serial.println("Akcelerometr nenalezen! Zkontrolujte připojení.");
        while (1)
            ;
    }
    accel.setRange(ADXL345_RANGE_16_G);

    /* inicializace Bluetooth gamepad knihovny */
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
    // získání dat z akcelerometru do proměnné event
    sensors_event_t event;
    accel.getEvent(&event);

    if (bleGamepad.isConnected())
    {
        // pokud je zařízení připojeno, zaponeme LEDku
        digitalWrite(LED, HIGH);

        // zkontrolujeme hodnotu přepínače a podle ní se rozhodeme jaký mód je aktivní
        // podle módu odešleme data z akcelerometru buď do levého nebo pravého joysticku
        if (digitalRead(SWITCH) == LOW)
        {
            mode = CONTROL; // nastavíme mód
            // mapujeme hodnoty z akcelerometru osy X do horizontální osy joysticku
            // mapujeme hodnoty z akcelerometru osy Y do vertikální osy joysticku
            bleGamepad.setLeftThumb(mapf(event.acceleration.x, -10.3, 10.8, 32767, 0), mapf(event.acceleration.y, -11, 9.6, 0, 32767));
            bleGamepad.setRightThumb(16383, 16383); // druhý joystick resetujeme na střed
        }
        else
        {
            mode = GAMING; // nastavíme mód
            // mapujeme hodnoty z akcelerometru osy X do horizontální osy joysticku
            // mapujeme hodnoty z akcelerometru osy Y do vertikální osy joysticku
            bleGamepad.setRightThumb(mapf(event.acceleration.x, -10.3, 10.8, 32767, 0), mapf(event.acceleration.y, -11, 9.6, 0, 32767));
            bleGamepad.setLeftThumb(16383, 16383); // druhý joystick resetujeme na střed
        }

        bleGamepad.sendReport(); // odešleme zprávu

        // zpracujeme stisknutí tlačítek
        for (int counter = 0; counter < 4; counter++)
        {
            handleButton(counter);
        }
    }
    else
    {
        // pokud není zařízení připojeno, začneme blikat LEDkou
        // abychom nemuseli používat delay, používám millis() funkci na zjištění uplynulého času mezi bliky
        unsigned long currentMillis = millis();
        // pokud uplynul čas
        if (currentMillis - ledPreviousMillis >= ledInterval)
        {
            ledPreviousMillis = currentMillis; // uložíme si aktuální čas

            // zapneme nebo vypneme LEDku
            if (ledState == LOW)
                ledState = HIGH;
            else
                ledState = LOW;

            // zapíšeme nový stav LEDky
            digitalWrite(LED, ledState);
        }
    }

    // Vypíšeme hodnoty z akcelerometru do sériové console
    Serial.println("X: " + String(event.acceleration.x) +
                   "   Y: " + String(event.acceleration.y) +
                   "   Z: " + String(event.acceleration.z));
}