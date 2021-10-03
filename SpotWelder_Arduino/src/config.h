#include <Wire.h> 
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <RotaryEncoder.h>
#include <OneButton.h>
#include <Ticker.h>

#define VERSION "0.1"
#define AUTHOR "LunaX-0921"

#define PIN_IN1 10
#define PIN_IN2 11
#define PIN_SW 12       // encoder switch
#define PIN_PWR 9       // power-welding switch
#define PIN_PULSE 8     // pint for SSR / welding

RotaryEncoder *encoder = nullptr;
OneButton *button = nullptr;
OneButton *powerBtn = nullptr;

Ticker *tickerHB = nullptr;
Ticker *tickerClearMsg = nullptr;


#define DEFAULT_PULSE_MS 200
#define DEFAULT_DELAY_MS 1500
#define MAX_PULSE_MS 750
#define MIN_PULSE_MS 0
#define MAX_DELAY_MS 3000
#define MIN_DELAY_MS 500
#define MAX_COUNT 9999

#define LCD_BLINK_MS 500
#define LCD_PULSE_MS 250
#define LCD_CLEAR_MS 1000
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;

LiquidCrystal lcd(rs,en,d4,d5,d6,d7); // LCD Shield

uint8_t bell[8]  = {0x4,0xe,0xe,0xe,0x1f,0x0,0x4};
uint8_t note[8]  = {0x2,0x3,0x2,0xe,0x1e,0xc,0x0};
uint8_t clock[8] = {0x0,0xe,0x15,0x17,0x11,0xe,0x0};
uint8_t heart[8] = {0x0,0xa,0x1f,0x1f,0xe,0x4,0x0};
uint8_t duck[8]  = {0x0,0xc,0x1d,0xf,0xf,0x6,0x0};
uint8_t check[8] = {0x0,0x1,0x3,0x16,0x1c,0x8,0x0};
uint8_t cross[8] = {0x0,0x1b,0xe,0x4,0xe,0x1b,0x0};
uint8_t retarrow[8] = {	0x1,0x1,0x5,0x9,0x1f,0x8,0x4};
uint8_t danger[8] = {	0x4,0x8,0x1f,0x3,0x4,0x18,0x1c};


const byte backLightpin = 5; 
//const byte backLightpin = 10; 
const byte contrast_pin = 6;

char printString[16];

typedef struct  {
   uint16_t pulse_ms;            // time for welding in range of 25ms - 100ms, default 50
   uint16_t time_delay_ms;       // time between two spot weldings, to avoid overheating, range 200-1000ms
   uint16_t welding_count;       // number of weldings points
} Data;

uint16_t savedDelayMS = 0;
uint16_t savedPulseMS = 0;

#define MAX_STATE 2
static uint8_t state = 0;  // 0=idle, 1=change timeout, 2 = change delay

Data eeprom_values ;
uint16_t eeprom_adr = 0;

bool refresh = false;