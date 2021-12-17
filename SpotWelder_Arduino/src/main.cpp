#include <Arduino.h>
#include "config.h"

/*********************************************************************************************
 * Interruptroutine for RotaryEncoder
 * 
 ********************************************************************************************/
void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}

/*********************************************************************************************
 * Interruptroutinte for RotaryEncoder
 * 
 ********************************************************************************************/
void checkTicks()
{
  // include all buttons here to be checked
  button->tick(); // just call tick() to check the state.
  powerBtn->tick();
}

/*********************************************************************************************
 * OneButton interrupt routine, called if user made a double click
 * increase state
 * if max state than go to 0 (go back without saving)
 *********************************************************************************************/
void click() {
   state++;
   if (state > MAX_STATE) {
     state = 0;
     refresh = true;
   }
}

/*********************************************************************************************
 * clickPwrBtn()
 * interrupt callback routine for power (welding) button
 * 
 *********************************************************************************************/
void clickPwrBtn() {
  state = 10; // welding
}

/*********************************************************************************************
 * doubleClick()
 * interrupt routine for rotary knob switch. Is used to change values
 * Doubleclick is only useable if state is > 0
 *********************************************************************************************/
void doubleClick() {
  if (state > 0) {
    state = 99;     // save state 
  }
}

/*********************************************************************************************
 * Initiize LCD and create some new characters
 * 
 ********************************************************************************************/
void LCDInit() {
  lcd.begin(16,4);

  lcd.createChar(0, bell);
  lcd.createChar(1, note);
  lcd.createChar(2, clock);
  lcd.createChar(3, heart);
  lcd.createChar(4, duck);
  lcd.createChar(5, check);
  lcd.createChar(6, cross);
  lcd.createChar(7, retarrow);
  lcd.createChar(8, danger);

  lcd.home (); 
  lcd.clear (); 
  lcd.setCursor(0,0);
  lcd.print("Spot Welder");
  lcd.setCursor(0,1);
  lcd.print("(c) "); lcd.print(AUTHOR);
  lcd.setCursor(0,3);
  lcd.print("Version:"); lcd.print(VERSION);
}

/*********************************************************************************************
 * LCD write function only for a formatted number
 * 
 ********************************************************************************************/
void writeNumber(int n, const char *fmt, uint8_t x, uint8_t y) {
  char tmp[64];
  sprintf(tmp, fmt, n);
  lcd.setCursor(x,y);
  lcd.print(tmp);
}

/*********************************************************************************************
 * setFootage()
 * print a string in the last row (status row), due to state as parameter
 * 
 *********************************************************************************************/
void setFootage(uint8_t id) {
  char tmp[20];
  switch(id) {
    case 0: 
      lcd.setCursor(0,3);
      sprintf(tmp,"%-16s", "- change value? ");
      lcd.print(tmp);
      break;
    case 1: 
    case 2:
      lcd.setCursor(0,3);
      sprintf(tmp,"%-16s", "--- changing ---");
      lcd.print(tmp);
      break;
    case 97:
      lcd.setCursor(0,3);
      sprintf(tmp,"%-16s", "---cool down ---");
      lcd.print(tmp);
      break;
    case 98:
      sprintf(tmp,"%16s", " ");
      lcd.setCursor(0,3); lcd.print(tmp);
      break;      
    case 99: 
      lcd.setCursor(0,3);
      sprintf(tmp,"%16s", "--- saving ---");
      lcd.print(tmp);
      break;

  }
}

/*********************************************************************************************
 * mainLCD()
 * 
 * update LCD screen with "normal" data, called by void loop()
 *********************************************************************************************/
void mainLCD(bool clear=false) {
  if (clear) lcd.clear();
  writeNumber(eeprom_values.pulse_ms,     "Welding:%4dms",1,0);
  writeNumber(eeprom_values.time_delay_ms,"Cooling:%4dms",1,1);
  writeNumber(eeprom_values.welding_count,"Count  :%6d",1,2);

}

/*********************************************************************************************
 * selectRow()
 * normaly used if user try to change numbers (pulse time, pause time).
 * Indicator on postion 0 at the row is an > sign for selected row
 * 
 *********************************************************************************************/
void selectRow(uint8_t row, bool remove=false) {
  lcd.setCursor(0,row);
  if (remove) {
    lcd.print(" ");
  }
  else {
    lcd.print(">");
  }
}

/*********************************************************************************************
 * changeValues()
 * called by void loop() 
 * 
 * depends on state, user can change values
 * state = 1 : changing welding time
 * state = 2 : changing delay time between welding
 * 
 * change footage on screen 
 * 
 * Method is called by void loop()
 *********************************************************************************************/
void changeValues(uint8_t state, uint16_t value) {
  int newValue = 0;
  switch(state) {
    case 1:
      selectRow(0);
      lcd.setCursor(9,0);
      newValue = savedPulseMS + value;
      sprintf(printString,"%4dms", newValue);
      lcd.print(printString);
      eeprom_values.pulse_ms = newValue;
      break;
    case 2:
      selectRow(1);
      lcd.setCursor(9,1);
      newValue = savedDelayMS + value;
      sprintf(printString,"%4dms", newValue);
      lcd.print(printString);
      eeprom_values.time_delay_ms = newValue;
      break;
  }
}

/*********************************************************************************************
 * save2Eeprom()
 * 
 * save struct into eeprom
 * 
 *********************************************************************************************/
void save2Eeprom() {
  #ifndef DBG_NO_EEPROM_SAVE
    EEPROM.put(0,eeprom_values);
    savedDelayMS = eeprom_values.time_delay_ms;
    savedPulseMS = eeprom_values.pulse_ms;
  #endif
}

void pressLongStart() {
  state = 98;
}

void pressLongStop() {
  state = 98;
}



/*********************************************************************************************
 * reset()
 * a reset impulse was recognizes. Set eeprom values to default and save them again into eeprom
 * This is usefull if user made an mistake with values
 *********************************************************************************************/
void reset() {
  lcd.clear();
  lcd.print("RESET");
  lcd.print("wait ....");
}
/*********************************************************************************************
 * clearMessage()
 * LCD routine to delete last status row from LCD
 * 
 *********************************************************************************************/
void clearMessage() {
  setFootage(98);
  //tickerClearMsg->pause();
}

/*********************************************************************************************
 * heartBeat()
 * If system is up and running a heartbeat blinks at last column in the first row
 * 
 *********************************************************************************************/
void heartBeat() {
  static bool hb = true;
  lcd.setCursor(15,0);
  if (hb) lcd.write(3); else {lcd.write(' ');  clearMessage(); }
  hb = !hb;
}

/*********************************************************************************************
 * startWelding()
 * if user push the power(welding) button, this routine is called. Welding (pulse) time is based on eeprom value
 * 
 *********************************************************************************************/
void startWelding() {
  lcd.setCursor(0,3);
  for (int x=0; x < 16; x++) {
    lcd.write(8);
  }
  unsigned long start = millis();
  while ( (millis()-start) < eeprom_values.pulse_ms) {
    // enable SSR
    digitalWrite(PIN_PULSE, HIGH);
  }
  // disable SSR
  digitalWrite(PIN_PULSE, LOW);

  // delay short time between two weldings
  start = millis();
  bool done = false;
  while ((millis() - start) < eeprom_values.time_delay_ms) {
    if ((millis() - start) > LCD_PULSE_MS && !done) {
      clearMessage();
      setFootage(97);
      done=true;
    }
  }
  // update welding count
  eeprom_values.welding_count++;
  #ifndef DBG_WELDING_TIMING
    save2Eeprom();
  #endif
}


/*********************************************************************************************
 * Arudiono routines setup() and loop()
 *********************************************************************************************/
void setup() {
    Serial.begin(115200);
    Serial.println("Hello");
  // set eeprom defaults

  pinMode(PIN_PULSE, OUTPUT);
  digitalWrite(PIN_PULSE, LOW);

  EEPROM.get(0, eeprom_values);

  // are values from eeprom in range?
  eeprom_values.pulse_ms = constrain(eeprom_values.pulse_ms, MIN_PULSE_MS, MAX_PULSE_MS);
  eeprom_values.time_delay_ms = constrain(eeprom_values.time_delay_ms, MIN_DELAY_MS, MAX_DELAY_MS);
  eeprom_values.welding_count = constrain(eeprom_values.welding_count, 0, MAX_COUNT);
  
  // set defaults if max is reached
  if (eeprom_values.pulse_ms == MAX_PULSE_MS) eeprom_values.pulse_ms = DEFAULT_PULSE_MS;
  if (eeprom_values.time_delay_ms == MAX_DELAY_MS) eeprom_values.time_delay_ms = DEFAULT_DELAY_MS;
  if (eeprom_values.welding_count == MAX_COUNT) eeprom_values.welding_count = 0;

  LCDInit();
  
  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);
  button = new OneButton(PIN_SW, true, true);
  powerBtn = new OneButton(PIN_PWR, true, true);

  button->setPressTicks(2000); // that is the time when LongPressStart is called
  button->attachLongPressStart(pressLongStart);
  button->attachLongPressStop(pressLongStop);

  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_SW), checkTicks, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_SW), checkTicks, CHANGE);


  button->attachClick(click);
  button->attachDoubleClick(doubleClick);
  powerBtn->attachClick(clickPwrBtn);

  tickerHB = new Ticker(heartBeat, LCD_BLINK_MS, 0, MILLIS);             // heartbeat every 500ms, endles
  tickerClearMsg = new Ticker(clearMessage, LCD_CLEAR_MS,0, MILLIS);    // clear message after 1000ms

  EEPROM.put(0, eeprom_values);

  tickerHB->start();
  tickerClearMsg->start();
  delay (2000);
  lcd.clear();

  state = 0;
  savedDelayMS = eeprom_values.time_delay_ms;
  savedPulseMS = eeprom_values.pulse_ms;

  tickerClearMsg->pause();
}

void loop() {
  static int pos = 0;
  tickerHB->update();
  tickerClearMsg->update();
  button->tick();
  encoder->tick();
  powerBtn->tick();
  int newPos = encoder->getPosition();
  #ifdef DBG_WELDING_TIMING
    state=10;
  #endif
  switch(state) {
    case 0 :                          // system is in idle state
      lcd.noCursor();
      selectRow(0,true);
      selectRow(1,true);
      if (pos != newPos) {
        setFootage(98);
        //tickerClearMsg->resume();
      }
      mainLCD(refresh);               // if refresh is true, LCD is cleared before printing
      refresh=false;
      break;
    case 1:                           // change state: here changing pulse timing
      tickerHB->pause();
      selectRow(1,true);
      selectRow(0);
      setFootage(state);
      if (pos != newPos) {
        changeValues(state, pos);
        pos = newPos;
      }
      tickerHB->resume();
      break;
    case 2:                           // change state: here changing pause between two weldings
      tickerHB->pause();
      selectRow(0,true);
      selectRow(1);
      setFootage(state);
      if (pos != newPos) {
        changeValues(state, pos);
        pos = newPos;
      }
      tickerHB->resume();
      break;
    case 10:                          // start a welding pulse
      startWelding();
      state = 0;
      break;
    case 98:                          // reset
      reset();
      save2Eeprom();
      state = 0;
      break;
    case 99:                          // save new values to eeprom
      lcd.clear();
      lcd.print("saving");
      save2Eeprom();
      delay(1000);
      state=0;
      tickerHB->resume();
      break;
  }
} // loop ()
