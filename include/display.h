#include <Arduino.h>

// 74HC pins
static const uint8_t HC595_LATCH_PIN = D0; // ST_CP
static const uint8_t HC595_CLOCK_PIN = D5; // SH_CP
static const uint8_t HC595_DATA_PIN = D6;  // DS

// clock dots
static const uint8_t L_DOT_INDEX = 1;
static const uint8_t R_DOT_INDEX = 0;

void initPorts();
void clearBufferWithoutDots();
void displayBuffer();
void printBuffer();
void setDots(uint8_t ldot, uint8_t rdot);
void setDigits(uint8_t h, uint8_t m, uint8_t s);
void setDigits(uint8_t h1, uint8_t h2, uint8_t m1, uint8_t m2,uint8_t s1, uint8_t s2);
void setDigits(
    uint8_t h1, uint8_t v1,
    uint8_t h2, uint8_t v2,
    uint8_t m1, uint8_t v3,
    uint8_t m2, uint8_t v4,
    uint8_t s1, uint8_t v5,
    uint8_t s2, uint8_t v6
);
void setDigits(uint32_t value);
