#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>

#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <RTClib.h>

#include <stdlib.h>

#include "display.h"
#include "secrets.h"


// === Configurable constants ===
// analog button pin
static const uint8_t ANALOG_BUTTON_PIN = A0;

// PWM pins mapping
static const uint8_t PWM_1_PIN = TX;
static const uint8_t PWM_2_PIN = RX;
static const uint8_t PWM_3_PIN = D7;
static const uint8_t PWM_4_PIN = D8;
static const uint8_t PWM_5_PIN = D3;
static const uint8_t PWM_6_PIN = D4;

// PWM settings
static const uint16_t PWM_FREQ_HZ = 250; // default 250 hz
static const uint16_t PWM_RES_BITS = 10;  // 4-16 bits
static const uint16_t MIN_BRIGHTNESS = 100;
static const uint16_t MAX_BRIGHTNESS = 1023; // max = 2^PWM_RES_BITS
static const uint16_t HIGH_BRIGHTNESS_THRESHOLD = uint16_t((double)MAX_BRIGHTNESS * 1.0 - 0.1);
static const uint16_t LOW_BRIGHTNESS_THRESHOLD = uint16_t((double)MIN_BRIGHTNESS * 1.0 + 0.1);

// digits change fade in/out effect time
static const uint16_t FADE_IN_MILLIS = 380;
static const uint16_t FADE_OUT_MILLIS = 220;

// delay between PWM duty update (digits change fade in/out effect)
static const uint16_t PWM_CHANGE_DELAY_MILLIS = 10;

// update time fron NTP server every N days
static const uint8_t NTP_UPDATE_PERIOD_DAYS = 3;

// dits status blink delay
static const uint16_t STATUS_BLINK_DELAY = 150;

// Timezone, default UTC+3
static const long TIMEZONE_OFFSET_SECONDS = 3 * 60 * 60;

// NTP server address
static const char NTP_SERVER_ADDR[] = "time.google.com";

// WiFi credentials
// Define constants in secrets.h file or change to ssid and password string
const char *ssid     = WIFI_SSID;
const char *password = WIFI_PASSWORD;

const char *CLOCK_HOSTNAME = "nixie-clock-in14";

static const uint16_t WIFI_CHECK_TIMEOUT = 300; // do not set values greater than ~500, this will affect the clock animation
static const uint16_t WIFI_CHECK_DELAY = 3000;

// Input
static const uint16_t UP_BUTTON_VALUE = 389;
static const uint16_t DOWN_BUTTON_VALUE = 105;
static const uint16_t SET_BUTTON_VALUE = 10;

static const uint16_t BUTTON_MEASURE_HALF_WINDOW = 25;
static const uint16_t BUTTON_MEASURE_DELAY_MILLIS = 5;
static const uint16_t BUTTON_MEASURE_COUNTS = 5; // only odd values
static const uint16_t BUTTON_MEASURE_PRESSED_THRESHOLD = 800;

static const uint16_t BUTTON_LONG_PRESS_TIME = 800;


// === Constants ===

static const uint8_t pwmDutyPinsSequence[6] {PWM_1_PIN, PWM_2_PIN, PWM_3_PIN, PWM_4_PIN, PWM_5_PIN, PWM_6_PIN};



// === Variables ===

// 0 = disabled, 1 = fade out, 2 = fade in, 3 = in/out
uint8_t dimmPinsFadeState[6] { 0, 0, 0, 0, 0, 3 };

RTC_DS3231 rtc;

// WiFi
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_SERVER_ADDR, TIMEZONE_OFFSET_SECONDS);
wl_status_t lastWifiStatus = wl_status_t::WL_IDLE_STATUS;
uint32_t lastWifiStatusCheck = 0;

// Global state
enum class State { CLOCK_DISPLAY, CLOCK_SETUP, BRIGHTNESS_SETUP, DIGITS_DIAGNOSTIC, WIFI_STATUS, ANTI_POISONING,
     STATE_ITEMS };
State state = State::CLOCK_DISPLAY;

// last obtained time from RTC chip
uint8_t lastHour = 0;
uint8_t lastMinute = 0;
uint8_t lastSecond = 0;

// timers
uint32_t lastClockChangeMillis = 0;
uint32_t lastPWMChangeMillis = 0;
uint32_t lastNTPUpdateMillis = 0;
uint32_t lastBlinkMillis = 0;
uint32_t diagnosticMillis = 0;

uint32_t diagnosticCounter = 0;

// display status
bool statusBlinkEnabled = false;
uint8_t statusBlinkState = 1;

// input
bool readButton = false;
uint32_t lastInputReadMillis = 0;
uint32_t buttonsPressedStartMillis = 0;
uint16_t buttonsInputAcc[BUTTON_MEASURE_COUNTS];
uint8_t buttonsInputMeasureCount = 0;

enum class ButtonType { NONE = 0, UP = 1, DOWN = 2, SET = 3 };
enum class PressType { NONE = 0, SHORT = 1, LONG = 2 };
ButtonType buttonType = ButtonType::NONE;
PressType pressType = PressType::NONE;



void handleState();
void displayClock(DateTime now);
void display();
void initDisplay();
void initRTC();
void initWiFi();
void initPWM();
void setMaxPWM();
void setZeroPWM();
void setPWM(uint16_t value);
void setRelativeBrightness(uint8_t pin, uint32_t value, uint32_t range);
void rightDotBlinkSync();
void rightDotBlinkAsync();
void displayInitStatus(uint8_t status);
void performNetworkTasks();
void inputHandler();
int comparator(const void * a, const void * b);


void setup() {
    delay(500);

    // init display
    initPWM();
    setZeroPWM();
    initDisplay();
    setMaxPWM();

    displayInitStatus(1);
    rightDotBlinkSync();

    // initRTC
    displayInitStatus(2);
    initRTC();

    // initWiFi
    displayInitStatus(3);
    initWiFi();
}

void loop() {
    // === Input ===
    inputHandler();

    // === processing section ===
    rightDotBlinkAsync(); // display status
    performNetworkTasks();

    // === display section ===
    clearBufferWithoutDots();
    handleState();
    displayBuffer();
}

void handleState() {
    DateTime now = rtc.now();
    uint32_t currentMillis = millis();
    switch (state) {
        case State::CLOCK_DISPLAY:
            displayClock(now);
            break;
        case State::CLOCK_SETUP:
            setMaxPWM();
            setDigits((int) State::CLOCK_SETUP, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            // TODO change clock
            break;
        case State::BRIGHTNESS_SETUP:
            setMaxPWM();
            setDigits((int) State::BRIGHTNESS_SETUP, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            // TODO change brightness
            break;
        case State::DIGITS_DIAGNOSTIC:
            setMaxPWM();
            if (currentMillis - diagnosticMillis >= 1000) {
                diagnosticCounter = (diagnosticCounter + 1) % 10;
                diagnosticMillis = currentMillis;
            }

            setDigits(diagnosticCounter, diagnosticCounter, diagnosticCounter, diagnosticCounter, diagnosticCounter, diagnosticCounter);
            setDots(diagnosticCounter % 2, diagnosticCounter % 2);
            break;
        case State::WIFI_STATUS:
            setMaxPWM();
            setDigits((int) State::WIFI_STATUS, 1, 0, 0, 0, 0, 0, 0, 0, 0, lastWifiStatus, 1);
            break;
        case State::ANTI_POISONING:
            setMaxPWM();
            // TODO вынести в отдельную функцию статуса - номер слева, цифра данных справа
            setDigits((int) State::ANTI_POISONING, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            // TODO ANTI_POISONING
            break;
        default:
            delay(1000);
            break;
    }
}


void displayClock(DateTime now) {
    uint8_t second = now.second();
    uint8_t minute = now.minute();
    uint8_t hour = now.hour();
    uint32_t currentMillis = millis();

    if (lastSecond != second || lastMinute != minute || lastHour != hour) {
        // At the end of the loop there are 980-1007 ms if there are no other calculations in the loop
        lastClockChangeMillis = currentMillis;
        
        // group digits for dimming by rule: fade out -> fade in
        dimmPinsFadeState[0] =
            ((hour % 10 == 9 || hour == 23) && minute == 59 && second == 59) + // (_9)|(23)-59-59 -> 1
             (hour % 10 == 0 && minute == 0 && second == 0) * 2; // _0-00-00 -> 2
        dimmPinsFadeState[1] = (minute == 59 && second == 59) + (minute == 0 && second == 0) * 2;  // __-59-59 -> 1 ; __-00-00 -> 2
        dimmPinsFadeState[2] = (minute % 10 == 9 && second == 59) + (minute % 10 == 0 && second == 0) * 2;  // __-_9-59 -> 1 ; __-_0-00 -> 2
        dimmPinsFadeState[3] = (second == 59) + (second == 0) * 2; // __-__-59 -> 1 ; __-__-00 -> 2
        dimmPinsFadeState[4] = (second % 10 == 9) + (second % 10 == 0) * 2; // __-__-_9 -> 1 ; __-__-_0 -> 2
    }

    uint32_t secondTimePoint = currentMillis - lastClockChangeMillis;
    // set dimming value
    if (currentMillis - lastPWMChangeMillis >= PWM_CHANGE_DELAY_MILLIS) {
        if (secondTimePoint <= FADE_IN_MILLIS) {
            // fade in
            for (uint8_t i = 0; i < 6; i++) {
                if (dimmPinsFadeState[i] == 2 || dimmPinsFadeState[i] == 3) {
                    setRelativeBrightness(pwmDutyPinsSequence[i], secondTimePoint, FADE_IN_MILLIS);
                }
            }
            lastPWMChangeMillis = currentMillis;
        } else if (secondTimePoint <= 1000 && (1000 - secondTimePoint) <= FADE_OUT_MILLIS) {
            // fade out
            for (uint8_t i = 0; i < 6; i++) {
                if (dimmPinsFadeState[i] == 1 || dimmPinsFadeState[i] == 3) {
                    setRelativeBrightness(pwmDutyPinsSequence[i], max(1000 - secondTimePoint, uint32_t(0)), FADE_OUT_MILLIS);
                }
            }
            lastPWMChangeMillis = currentMillis;
        }

        // fallback restore brightness for other digits
        for (uint8_t i = 0; i < 6; i++) {
            if (dimmPinsFadeState[i] == 0) {
                analogWrite(pwmDutyPinsSequence[i], MAX_BRIGHTNESS);
            }
        }
    }

    // display current time
    setDigits(hour, minute, second);

    if (!statusBlinkEnabled) {
        setDots(1, 1);
    }

    lastHour = hour;
    lastMinute = minute;
    lastSecond = second;
}

void initDisplay() {
    initPorts();
    digitalWrite(HC595_LATCH_PIN, HIGH);
    clearBufferWithoutDots();
    displayBuffer();
}

void initRTC() {
    // TODO if possible - add RTC chip reset pin
    while (!rtc.begin()) {
        delay(500);
    }
}

void initWiFi() {
    WiFi.hostname(CLOCK_HOSTNAME);
    WiFi.begin(ssid, password);
    WiFi.setAutoReconnect(true);
}

void initPWM() {
    // init PWM pins
    for (uint8_t i = 0; i < 6; i++)
        pinMode(pwmDutyPinsSequence[i], OUTPUT);

    analogWriteFreq(PWM_FREQ_HZ);
    analogWriteResolution(PWM_RES_BITS);
}

void setMaxPWM() {
    setPWM(MAX_BRIGHTNESS);
}

void setZeroPWM() {
    setPWM(0);
}

void setPWM(uint16_t value) {
    for (uint8_t i = 0; i < 6; i++)
        analogWrite(pwmDutyPinsSequence[i], value);
}

// value = [0...range]
void setRelativeBrightness(uint8_t pin, uint32_t value, uint32_t range) {
    uint32_t calculatedValue = (uint32_t)
        min(
            max(
                (uint32_t) ((((double_t) value / (double_t) range) * ((double_t) MAX_BRIGHTNESS - (double_t) MIN_BRIGHTNESS)) + MIN_BRIGHTNESS),
                (uint32_t) MIN_BRIGHTNESS),
            (uint32_t) MAX_BRIGHTNESS);
    calculatedValue = calculatedValue > HIGH_BRIGHTNESS_THRESHOLD ? MAX_BRIGHTNESS : calculatedValue;
    calculatedValue = calculatedValue < LOW_BRIGHTNESS_THRESHOLD ? MIN_BRIGHTNESS : calculatedValue;
    analogWrite(pin, calculatedValue);
}

void rightDotBlinkSync()
{
    int blinks = 0;
    while (blinks < 4)
    {
        setDots(0, 1);
        displayBuffer();
        delay(STATUS_BLINK_DELAY);

        setDots(0, 0);
        displayBuffer();
        delay(STATUS_BLINK_DELAY);

        blinks++;
    }
    setDots(0, 1);
}

void rightDotBlinkAsync() {
    if (statusBlinkEnabled) {
        uint32_t currentMillis = millis();
        if (currentMillis - lastBlinkMillis > STATUS_BLINK_DELAY) {
            setDots(0, statusBlinkState);
            displayBuffer();
            statusBlinkState = !statusBlinkState;
            lastBlinkMillis = currentMillis;
        }
    }
}

void displayInitStatus(uint8_t status) {
    clearBufferWithoutDots();
    setDots(0, 1);
    setDigits(status);
    displayBuffer();
}

void performNetworkTasks() {
    uint32_t currentMillis = millis();
    uint32_t lastClockChange = currentMillis - lastClockChangeMillis;
    bool noAnimationPeriod = lastClockChange > (FADE_IN_MILLIS + 5) && lastClockChange < (FADE_OUT_MILLIS - 5);

    // check connection status
    if (currentMillis - lastWifiStatusCheck >= WIFI_CHECK_DELAY && noAnimationPeriod) {
        int8_t status = WiFi.waitForConnectResult(WIFI_CHECK_TIMEOUT);
        lastWifiStatus = status == -1 ? wl_status_t::WL_IDLE_STATUS : (wl_status_t) status;
        lastWifiStatusCheck = currentMillis;
    }

    statusBlinkEnabled = lastWifiStatus != WL_CONNECTED;

    // update time from NTP server every NTP_UPDATE_PERIOD_DAYS days from start
    if ((lastNTPUpdateMillis == 0 || (currentMillis - lastNTPUpdateMillis > NTP_UPDATE_PERIOD_DAYS * 24 * 60 * 60 * 1000)) && noAnimationPeriod) {
        if (lastWifiStatus == WL_CONNECTED) {
            WiFi.persistent(true);

            timeClient.begin();
            if (timeClient.update()) {
                rtc.adjust(DateTime(timeClient.getEpochTime()));
                lastNTPUpdateMillis = currentMillis;
                statusBlinkEnabled = false;
            } else {
                statusBlinkEnabled = true;
            }
        }
    }
}

// Raw ADC values
void inputHandler() {
    uint32_t currentMillis = millis();
    uint32_t currentMeasure = analogRead(ANALOG_BUTTON_PIN);

    if (!readButton) {
        if (currentMeasure < BUTTON_MEASURE_PRESSED_THRESHOLD) {
            buttonsPressedStartMillis = currentMillis;
            readButton = true;
        }
    } else {
        if (buttonsInputMeasureCount < BUTTON_MEASURE_COUNTS) {
            // measure
            if (currentMillis - lastInputReadMillis >= BUTTON_MEASURE_DELAY_MILLIS) {
                buttonsInputAcc[buttonsInputMeasureCount] = currentMeasure;
                buttonsInputMeasureCount++;
                lastInputReadMillis = currentMillis;
            }
        } else {
            // calculate press and button type
            if (currentMeasure > BUTTON_MEASURE_PRESSED_THRESHOLD) {
                // TODO add very long press
                pressType = (currentMillis - buttonsPressedStartMillis) >= BUTTON_LONG_PRESS_TIME ?
                    PressType::LONG : PressType::SHORT;

                std::qsort(buttonsInputAcc, BUTTON_MEASURE_COUNTS, sizeof(uint16_t), comparator);
                uint16_t measureMedian = buttonsInputAcc[BUTTON_MEASURE_COUNTS % 2];

                if (measureMedian > (UP_BUTTON_VALUE - BUTTON_MEASURE_HALF_WINDOW) && measureMedian < (UP_BUTTON_VALUE + BUTTON_MEASURE_HALF_WINDOW)) {
                    buttonType = ButtonType::UP;
                }
                if (measureMedian > (DOWN_BUTTON_VALUE - BUTTON_MEASURE_HALF_WINDOW) && measureMedian < (DOWN_BUTTON_VALUE + BUTTON_MEASURE_HALF_WINDOW)) {
                    buttonType = ButtonType::DOWN;
                }
                if (measureMedian > std::max((SET_BUTTON_VALUE - BUTTON_MEASURE_HALF_WINDOW), 0) && measureMedian < (SET_BUTTON_VALUE + BUTTON_MEASURE_HALF_WINDOW)) {
                    buttonType = ButtonType::SET;
                }

                // reset state
                readButton = false;
                buttonsInputMeasureCount = 0;
                buttonsPressedStartMillis = 0;
                lastInputReadMillis = 0;
            }
        }
    }

    // handle result
    if (pressType != PressType::NONE && buttonType != ButtonType::NONE) {
        if (buttonType == ButtonType::SET && pressType == PressType::SHORT) {
            // change current state
            state = (State)(((int) state + 1) % ((int) State::STATE_ITEMS));
        }

        // TODO handle input

        buttonType = ButtonType::NONE;
        pressType = PressType::NONE;
    }
}

int comparator(const void * a, const void * b) {
    return ( *(uint16_t*)a - *(uint16_t*)b );
}