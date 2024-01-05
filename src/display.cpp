#include "display.h"

uint8_t viewBuffer[64]; // register bits (8x8)
uint8_t digitMapping[6][10] { // [rank][digit] -> buffer bit
    { 13,12,11,10, 9, 8, 7, 6, 5, 4 }, // H1 0, 1, 2,...,9 digits 
    { 23,22,21,20,19,18,17,16,15,14 },  // H2

    { 33,32,31,30,29,28,27,26,25,24 },  // M1
    { 43,42,41,40,39,38,37,36,35,34 },  // M2

    { 53,52,51,50,49,48,47,46,45,44 },  // S1
    { 63,62,61,60,59,58,57,56,55,54 }  // S2
};

uint8_t valueDigitBuffer[6];

void initPorts() {
    pinMode(HC595_LATCH_PIN, OUTPUT);
    pinMode(HC595_CLOCK_PIN, OUTPUT);
    pinMode(HC595_DATA_PIN, OUTPUT);
}

void clearBufferWithoutDots() {
    for (uint8_t i = 0; i < 64; i++) {
        if (i != L_DOT_INDEX && i != R_DOT_INDEX) {
            viewBuffer[i] = 0;
        }
    }
}

// start from 0b00001000 (shift 4 bits) and H1, sequence H1:9,8,7,6...0 | H2: 9,...,0
void displayBuffer() {
    digitalWrite(HC595_LATCH_PIN, LOW); // start transmission
    for (uint8_t blockIndex = 0; blockIndex < 8; blockIndex++) {
        uint8_t octet = 0;
        for (uint8_t blockBit = 0; blockBit < 8; blockBit++) {
            octet = (octet | viewBuffer[blockIndex * 8 + blockBit]) << (blockBit < 7 ? 1 : 0);
        }
        shiftOut(HC595_DATA_PIN, HC595_CLOCK_PIN, MSBFIRST, octet);
    }
    digitalWrite(HC595_LATCH_PIN, HIGH); // end transmission
}

void setDots(uint8_t ldot, uint8_t rdot) {
    viewBuffer[L_DOT_INDEX] = ldot;
    viewBuffer[R_DOT_INDEX] = rdot;
}

void setDigits(uint8_t h, uint8_t m, uint8_t s) {
    setDigits(h / 10, h % 10, m / 10, m % 10, s / 10, s % 10);
}

void setDigits(uint32_t value) {
    for (int8_t i = 5; i >= 0; i--) {
        valueDigitBuffer[i] = (value == 0 && i < 5) ? -1 : (value % 10);
        value /= 10;
    }
    
    setDigits(
        valueDigitBuffer[0], valueDigitBuffer[0] >= 0 ? 1 : 0,
        valueDigitBuffer[1], valueDigitBuffer[1] >= 0 ? 1 : 0,
        valueDigitBuffer[2], valueDigitBuffer[2] >= 0 ? 1 : 0,
        valueDigitBuffer[3], valueDigitBuffer[3] >= 0 ? 1 : 0,
        valueDigitBuffer[4], valueDigitBuffer[4] >= 0 ? 1 : 0,
        valueDigitBuffer[5], valueDigitBuffer[5] >= 0 ? 1 : 0
    );
}

void setDigits(uint8_t h1, uint8_t h2, uint8_t m1, uint8_t m2, uint8_t s1, uint8_t s2) {
    setDigits(h1, 1, h2, 1, m1, 1, m2, 1, s1, 1, s2, 1);
}

void setDigits(
    uint8_t h1, uint8_t v1,
    uint8_t h2, uint8_t v2,
    uint8_t m1, uint8_t v3,
    uint8_t m2, uint8_t v4,
    uint8_t s1, uint8_t v5,
    uint8_t s2, uint8_t v6
) {
    viewBuffer[digitMapping[0][h1]] = v1;
    viewBuffer[digitMapping[1][h2]] = v2;
    viewBuffer[digitMapping[2][m1]] = v3;
    viewBuffer[digitMapping[3][m2]] = v4;
    viewBuffer[digitMapping[4][s1]] = v5;
    viewBuffer[digitMapping[5][s2]] = v6;
}
