/*
  Pure_Sine_Wave_Inverter_V.2.1 [NOT compatible with version 1.0 boards]
  Change log: Charging process has been modified. Charging relay will disengage at bat. voltage 14.2 V. Starting modulation index changed to 80.
  The if condition has been removed from tick2ms initialization. Output voltage calculation has been converted into integer domain.
  Sayantan Sinha: 19/10/2020
  sPWM on the atMega328P for the arduino NANO. H-bridge output with deadtime.
  Half-bridge #1: Operating freq 16 kHz, High Side driver = pin D9 (OC1A), Low Side driver = pin D10 (OC1B)
  Half-bridge #2: Operating freq 50 Hz, High Side driver = pin D11, Low Side driver = pin D12.
*/
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define SD0 0            // Shut down MOSFET driver ICs           (D8 i.e. bit-0 in PORTB)  (HIGH: shutdown, LOW: run)
#define HS1 1            // High side drive pin of Half Bridge #1 (D9 i.e. bit-1 in PORTB)
#define LS1 2            // Low side drive pin of Half Bridge #1  (D10 i.e. bit-2 in PORTB)
#define HS2 3            // High side drive pin of Half Bridge #2 (D11 i.e. bit-3 in PORTB)
#define LS2 4            // Low side drive pin of Half Bridge #2  (D12 i.e. bit-4 in PORTB)
#define LED 5            // LED                                   (D13 i.e. bit-5 in PORTB)
#define FAN_PIN 1        // Cooling fan
#define VI_SENS_PIN 2    // Output of the optocoupler
#define PH_SENS_PIN 3    // To sense +ve or -Ve half
#define BUZZ_PIN 4
#define TAPPING_RLY_PIN 5
#define CHG_RLY_PIN 6
#define CHNG_OVR_RLY_PIN 7         // Change over relay
#define LED_PIN 13
#define VO_SENS_PIN A0
#define T_SENS_PIN A1              // Temperature sensing pin (output from LM35)
#define LED_GRID A2
//#define LED_CHG A2
#define LED_SD A3
// A4 & A5 are used in I2C communication
#define VB_SENS_PIN A6  // A6 in Arduino Nano
#define I_SENS_PIN A7   // A7 in Arduino Nano
#define SW_FREQ 16000
#define MAX_COUNT 500       // (16 MHz / SW_FREQ) / 2
#define NO_OF_PULSES 160    // 16 kHz / 50 Hz = 320; 320 / 2 = 160 divisions in one half-cycle
#define Q_CYCLE 80
#define K_P 0.01
#define VO_FB_FACTOR 100    // Connect 220 V to 6 V step-down transformer in feedback
#define VO_DESIRED 220      // Set AC voltage at Output
#define SC_VOLTAGE 20       // Output voltage at short circuit
#define VB_SENS_FACTOR  4.047      // Battery sensor factor
#define I_SENS_FACTOR 5.0          // Current sensor factor
#define ADC_RES 0.004888           // Resolution of the ADC = 4.888 mV
//#define VI_CUTOFF 40.0    // 40 A cutoff current
#define VB_CUTOFF 10.80     // Corelate with battery data sheet
#define VB_LOW 11.00        // Battery warning starts beeping here
#define VB_CHG_START 13.7   // Battery will start charging below this voltage
#define VB_SET 14.0         // Battery voltage during charging
#define VB_FLOAT 13.8       // Battery voltage where floating charge starts
#define VB_CHG_STOP 14.2    // If by any chance battery voltage reaches this value, charging will be shutdown
#define I_SET 5.0           // Maximum charging current
#define I_FLOAT 0.01        // Current goes below this value indicates floating charge
#define I_FAST 0.3          // Current beyond this value indicates fast charge
#define T_HIGH 42.0         // Above this temperature the fan will turn on
#define T_C 60.0            // Above this temperature the unit will shut down
#define T_LOW 35.0          // Below this temperature the fan will turn off
#define GRID_MODE 0
#define INV_MODE 1

//Look up tables with 160 entries each, normalised to have max value of 500 which is the period of the PWM loaded into register ICR1.(D:\Simulations\MPLAB PROJECTS\SPWM_PhaseFreqCorrect_LookUp_Table_Gen.m)
unsigned int lookUp_100[NO_OF_PULSES] = {0, 10, 20, 29, 39, 49, 59, 69, 78, 88, 98, 107, 117, 126, 136, 145, 155, 164, 173, 182, 191, 200, 209, 218, 227, 236, 244, 253, 261, 270, 278, 286, 294, 302, 310, 317, 325, 332, 339, 347, 354, 360, 367, 374, 380, 387, 393, 399, 405, 410, 416, 421, 426, 431, 436, 441, 446, 450, 454, 458, 462, 466, 469, 472, 476, 478, 481, 484, 486, 488, 490, 492, 494, 495, 497, 498, 498, 499, 500, 500, 500, 500, 500, 499, 498, 498, 497, 495, 494, 492, 490, 488, 486, 484, 481, 478, 476, 472, 469, 466, 462, 458, 454, 450, 446, 441, 436, 431, 426, 421, 416, 410, 405, 399, 393, 387, 380, 374, 367, 360, 354, 347, 339, 332, 325, 317, 310, 302, 294, 286, 278, 270, 261, 253, 244, 236, 227, 218, 209, 200, 191, 182, 173, 164, 155, 145, 136, 126, 117, 107, 98, 88, 78, 69, 59, 49, 39, 29, 20, 10};
unsigned int lookUp[NO_OF_PULSES] = {500, 490, 480, 471, 461, 451, 441, 431, 422, 412, 402, 393, 383, 374, 364, 355, 345, 336, 327, 318, 309, 300, 291, 282, 273, 264, 256, 247, 239, 230, 222, 214, 206, 198, 190, 183, 175, 168, 161, 153, 146, 140, 133, 126, 120, 113, 107, 101, 95, 90, 84, 79, 74, 69, 64, 59, 54, 50, 46, 42, 38, 34, 31, 28, 24, 22, 19, 16, 14, 12, 10, 8, 6, 5, 3, 2, 2, 1, 0, 0, 0, 0, 0, 1, 2, 2, 3, 5, 6, 8, 10, 12, 14, 16, 19, 22, 24, 28, 31, 34, 38, 42, 46, 50, 54, 59, 64, 69, 74, 79, 84, 90, 95, 101, 107, 113, 120, 126, 133, 140, 146, 153, 161, 168, 175, 183, 190, 198, 206, 214, 222, 230, 239, 247, 256, 264, 273, 282, 291, 300, 309, 318, 327, 336, 345, 355, 364, 374, 383, 393, 402, 412, 422, 431, 441, 451, 461, 471, 480, 490};

volatile bool toggleHalfBridge2 = false;
volatile bool positiveHalf = true;
volatile bool zeroCrossing = false;
volatile bool sensVoltage = false;
volatile bool knockKnock = false;
volatile bool rampUp = true;
volatile int tick2msCounter = 0;
bool flagChangeDuty = false, lowBat = false, criticalBat = false, overCurrent = false, sc = false, criticalTem = false, flagBeep = false, flagSwitchToGrid = false;
bool flagWait = false, flag2000ms = false, flagCharging = false, flag125vTapping = false, flagChgSw = false, flagOverVoltage = false, flagShutDown = false, flagFan = false;
bool flagFloatChg = false;
byte mode = GRID_MODE;
float vBat, current, tem;  // Output voltage from feedback transformer, battery voltage, pd across current sense resistor
unsigned int vOut;
float iSens;  // Input current from sens resistance
unsigned int modIndx = 80; // Start the inverter with this modulation index
enum beepType {shortBeep = 100, longBeep = 300};
unsigned long eventTime = 0;
unsigned long dispTime = 0;
unsigned int cc = 300;       // Counter to keep delay of 3 s between lcd updates during charging

void changeDuty(void);
void correctVo(void);
void error(void);
void tick2ms(void);
void tick2000ms(void);
void dispRefresh(void);
void atZero (void);
void atPositive (void);
void switchToInv(void);
void switchToGrid(void);
void isGridOn(void);
void shutDown(void);
void startChg(void);
void stopChg(void);
void actuateCurrent(void);
void beep(byte t = shortBeep, byte n = 1, unsigned int p = 500);

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup()
{
  DDRB = DDRB | 0b00111111;  // Set digital pin13 (PB5) to digital pin 8 (PB0) as output. D13 = PB5 (PB7 & PB6 are unusable). PORTB = [XTAL2 XTAL1 D13 D12 D11 D10 D9 D8]
  PORTB =  _BV(SD0);         // Shut down the gate driver ICs
  pinMode (CHNG_OVR_RLY_PIN, OUTPUT);
  pinMode (VI_SENS_PIN, INPUT_PULLUP);
  pinMode (PH_SENS_PIN, INPUT_PULLUP);
  pinMode (CHG_RLY_PIN, OUTPUT);
  pinMode (TAPPING_RLY_PIN, OUTPUT);
  pinMode (BUZZ_PIN, OUTPUT);
  pinMode (LED_GRID, OUTPUT);
  //  pinMode (LED_CHG, OUTPUT);
  pinMode (LED_SD, OUTPUT);
  pinMode (FAN_PIN, OUTPUT);
  digitalWrite(CHNG_OVR_RLY_PIN, LOW);
  digitalWrite(CHG_RLY_PIN, LOW);
  digitalWrite(TAPPING_RLY_PIN, LOW);
  digitalWrite(LED_GRID, LOW);
  digitalWrite(LED_SD, LOW);
  //  digitalWrite(LED_CHG, LOW);
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(BUZZ_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZ_PIN, LOW);
  lcd.begin();
  if (digitalRead (VI_SENS_PIN) == LOW)
    mode = GRID_MODE;
  else
    mode = INV_MODE;
  changeDuty();
  // Timer2 Initialization for interruputing every 2 ms
  TCCR2A = 0b00000010;       // CTC Mode
  TCCR2B = 0b00000001;       // No prescaling
  OCR2A = 32;                // For interrrupt after 2 ms
  if (mode == INV_MODE) {
    switchToInv();
  }
  else {
    switchToGrid();
  }
  sei();                     // Enable global interrupts.
}

void loop()
{
  /*#################################################################################################################*/
  /*                                       ~~~~~~~~INVERTER MODE~~~~~~~~                                             */
  /*#################################################################################################################*/
  if (mode == INV_MODE) {
    if (zeroCrossing) {
      zeroCrossing = false;
      PORTB = PORTB & ~(_BV(HS2) | _BV(LS2));  // Reset HS2 & LS2
      if (flagSwitchToGrid) {
        PORTB = _BV(SD0);
        switchToGrid();
      }
    }
    if (toggleHalfBridge2) {
      toggleHalfBridge2 = false;
      if (positiveHalf)
        PORTB = PORTB | _BV(LS2);  // HS2 = LOW, LS2 = HIGH. (Change the polarity)
      else
        PORTB = PORTB | _BV(HS2);  // HS2 = HIGH, LS2 = LOW. (Change the polarity)
      vBat = (float)analogRead(VB_SENS_PIN) * ADC_RES * VB_SENS_FACTOR;
    }
    if (knockKnock) {
      knockKnock = false;
      isGridOn();
    }
    if (sensVoltage) {
      sensVoltage = false;
      vOut = analogRead(VO_SENS_PIN) * 48 / VO_FB_FACTOR;
      tem = (float)analogRead(T_SENS_PIN) * 0.4888;    // LM35: V_out = temperature (°C) * 10 mV
      if ((VO_DESIRED - vOut) > 5 || (VO_DESIRED - vOut) < 5)
        correctVo();
      else
        rampUp = false;
      if (vBat < VB_CUTOFF)
        if (!rampUp)
          criticalBat = true;
      if (vBat < VB_LOW)
        lowBat = true;
      if (vOut < SC_VOLTAGE && !rampUp)
        sc = true;
      if (tem > T_HIGH) {
        if (tem > T_C)
          criticalTem = true;
        else
          criticalTem = false;
        if (!flagFan) {
          digitalWrite(FAN_PIN, HIGH);
          flagFan = true;
        }
      }
      else if (tem < T_LOW && flagFan) {
        digitalWrite(FAN_PIN, LOW);
        flagFan = false;
      }
      if (criticalBat || sc || criticalTem) {  // Low battery or short circuit occured or critically high temperature
        shutDown();
      }
      if (lowBat && !flagBeep) {
        lowBat = false;
        beep(longBeep, 1, 2000);
      }
    }
    if (flagChangeDuty)
      changeDuty();
    if (flagShutDown) {
      if (knockKnock) {
        knockKnock = false;
        isGridOn();
      }
      if (flagSwitchToGrid)
        switchToGrid();
      unsigned long timeNow = millis();
      unsigned long timeElapsed = eventTime > timeNow ? (4294967295 - eventTime) + timeNow : timeNow - eventTime;
      if (timeElapsed > 5000)
        switchToInv();  // Reattempt to switch to inverter 5 s after shutdown
    }
    if (digitalRead(VI_SENS_PIN) == LOW && !flagWait) {
      tick2000ms();      // If Grid voltage is sensed, wait 2 s before switching to the Grid Mode
      flagWait = true;
    }
  }
  /*#################################################################################################################*/
  /*                                       ~~~~~~~~GRID MODE~~~~~~~~                                                 */
  /*#################################################################################################################*/
  else if (mode == GRID_MODE) {
    if (digitalRead(VI_SENS_PIN) == HIGH && !flagWait) {
      tick2ms();      // If Grid voltage down is sensed, wait 2 ms before switching to the Inverter Mode
      flagWait = true;
    }
    if (digitalRead(PH_SENS_PIN) == LOW) {   // Mains is at positive half.
      positiveHalf = false;                  // If grid shuts down, inverter will start negative half
    }
    else {
      positiveHalf = true;
    }
    if (knockKnock) {
      knockKnock = false;
      if (digitalRead(VI_SENS_PIN) == HIGH) {
        TIMSK1 = 0b00000000;         // TOIE1 = 0: All Timer1 Interrupts Disabled    // Shutdown charging first
        TCCR1A = 0b00000000;         // OC1A (pin D9) Disconnected;  OC1B (pin D10) Disconnected;
        PORTB =  _BV(SD0);           // Shut down gate drivers
        switchToInv();
      }
      else {
        flagWait = false;
      }
    }
    else {                            // Charging Control...
      unsigned long timeNow = millis();
      unsigned long timeElapsed = eventTime > timeNow ? (4294967295 - eventTime) + timeNow : timeNow - eventTime;
      if (!flagCharging) {
        if (timeElapsed > 3000) {     // Start charging after 3 s
          startChg();
        }
      }
      else {
        if (timeElapsed > 10) {      // Actuate current at every 10 ms interval
          vBat = (float)analogRead(VB_SENS_PIN) * ADC_RES * VB_SENS_FACTOR;
          current = (float)analogRead(I_SENS_PIN) * ADC_RES * I_SENS_FACTOR;
          tem = (float)analogRead(T_SENS_PIN) * 0.4888;    // LM35: V_out = temperature (°C) * 10 mV
          if (!flagFloatChg && !flagFan) {
            digitalWrite(FAN_PIN, HIGH);
            flagFan = true;
          }
          if (flagFloatChg && flagFan) {
            digitalWrite(FAN_PIN, LOW);
            flagFan = false;
          }
          actuateCurrent();
          eventTime = millis();
        }
      }
    }
  }
  if (flagBeep)
    beep();
}

ISR(TIMER1_OVF_vect)
{
  static byte i = 1;  // Static means it will NOT be reinitialized
  if (i >= NO_OF_PULSES) {
    i = 0;
  }
  OCR1A = lookUp[i];
  OCR1B = lookUp[i];
  if (i == 1) {
    positiveHalf = !positiveHalf;
    zeroCrossing = true;
    TCCR1A = positiveHalf ? 0b11000000 : 0b00110000;       // +Ve half: Set OC1A (pin D9) on Compare Match; Disconnect OC1B (pin D10); -Ve half: Opposite
  }
  else if (i == 2) {
    toggleHalfBridge2 = true;
  }
  else if (i == Q_CYCLE) {
    sensVoltage = true;
  }
  i++;
}

ISR(TIMER1_COMPA_vect)
{
  PORTB = 0b00010100;
}

ISR(TIMER1_COMPB_vect)
{
  PORTB = 0;
}

ISR(TIMER2_COMPA_vect)
{
  if (flag2000ms) {
    if (++tick2msCounter >= 125) {  // 125 * 16 ms = 2 s
      knockKnock = true;
      TIMSK2 = 0b00000000;         // No more interrupt
    }
  }
  else {
    knockKnock = true;
    TIMSK2 = 0b00000000;         // No more interrupt
  }
}

void tick2ms(void)             // knockKnock after 2 ms
{
  TCCR2A = 0b00000010;         // CTC Mode
  TCCR2B = 0b00000110;         // f_cpu / 256
  OCR2A = 125;                 // For interrrupt after 2 ms (2 / (128 * 0.0000625))
  flag2000ms = false;
  knockKnock = false;
  TCNT2 = 0;                   // Reset counter
  TIMSK2 = 0b00000010;         // Interrupt on compare match A
}

void tick2000ms(void)        // Timer2 Initialization for interruputing after 2000 ms
{
  TCCR2A = 0b00000010;       // CTC Mode
  TCCR2B = 0b00000111;       // f_cpu / 1024
  OCR2A = 250;               // For interrrupt after 16 ms (16 / (1024 * 0.0000625))
  tick2msCounter = 0;
  flag2000ms = true;
  knockKnock = false;
  TCNT2 = 0;                 // Reset counter
  TIMSK2 = 0b00000010;       // Interrupt on compare match A
}

void atZero(void)
{
  zeroCrossing = true;
  tick2ms();  // Knock after 2 ms
}

void atPositive (void)
{
  if (mode == GRID_MODE)
    positiveHalf = true;
}

void changeDuty(void)
{
  //  PORTB |= _BV(LED);
  modIndx = modIndx > 100 ? 100 : modIndx < 20 ? 20 : modIndx;
  lookUp[0] = MAX_COUNT;
  for (int i = 1; i <= Q_CYCLE; i++) {
    lookUp[i] = MAX_COUNT - ((lookUp_100[i] * modIndx) / 100);  // Calculate new duty cycle for quarter cycle
  }
  int j = Q_CYCLE - 1;
  for (int i = Q_CYCLE + 1; i < NO_OF_PULSES; i++) {              // Copy the duty cycle for next quarter cycle
    lookUp[i] = lookUp[j];
    j--;
  }
  flagChangeDuty = false;
  //  PORTB &= ~_BV(LED);
}

void correctVo(void)
{
  flagChangeDuty = true;
  int e = VO_DESIRED - vOut;
  if ((e > 0 && modIndx >= 100) || (e < 0 && modIndx <= 20))
    flagChangeDuty = false;
  if (flagChangeDuty) {
    if (e > 0) {
      if (rampUp)
        modIndx = modIndx + 5;  // Step-up the voltage linerarly
      else
        modIndx = modIndx + 1;  // Step-up the voltage linerarly
    }
    else if (e < 0) {
      if (rampUp)
        modIndx = modIndx - 5;  // Step-up the voltage linerarly
      else
        modIndx = modIndx - 1;  // Step-down the voltage linerarly
    }
    else
      flagChangeDuty = false;
  }
  else
    rampUp = false;  // Ramping up at start up is finished
}

void isGridOn(void)
{
  if (digitalRead(VI_SENS_PIN) == HIGH) { // Grid off
    if (mode == GRID_MODE) {
      zeroCrossing = false;
      digitalWrite(SD0, LOW);
      switchToInv();
    }
    else {
      flagSwitchToGrid = false;
      flagWait = false;
    }
  }
  else {
    if (mode == INV_MODE) {
      flagSwitchToGrid = true;
    }
    else {
      zeroCrossing = false;
    }
  }
}

void switchToGrid(void)
{
  TIMSK1 = 0b00000000;        // TOIE1 = 0: Overflow Interrupt Disabled
  TCCR1A = 0b00000000;        // OC1A (pin D9) Disconnected;  OC1B (pin D10) Disconnected, Phase and Frequency Correct (Mode 8);
  PORTB =  _BV(SD0);          // Shut down
  digitalWrite(CHNG_OVR_RLY_PIN, LOW);  // Switch power output to grid
  digitalWrite(CHG_RLY_PIN, LOW);      // Charging relay off
  mode = GRID_MODE;
  modIndx = 80;
  changeDuty();
  flagSwitchToGrid = false;
  flagCharging = false;
  flagWait = false;
  criticalTem = false;
  eventTime = millis();
  lcd.clear();
  lcd.print("Grid On B:     V");
  lcd.setCursor(10, 0);
  lcd.print(vBat);
  digitalWrite(LED_GRID, HIGH);
  digitalWrite(LED_SD, LOW);
  //  digitalWrite(LED_CHG, LOW);
}


void switchToInv(void)
{
  TIMSK1 = 0b00000000;             // TOIE1 = 0: Overflow Interrupt Disabled
  TCCR1A = 0b00000000;             // OC1A (pin D9) Disconnected;  OC1B (pin D10) Disconnected, Phase and Frequency Correct (Mode 8);
  PORTB =  _BV(SD0);               // Shut down gate drivers
  digitalWrite(CHNG_OVR_RLY_PIN, HIGH);  // Switch power output to inverter
  digitalWrite(CHG_RLY_PIN, LOW);  // Charging relay off
  digitalWrite(LED_GRID, LOW);
  digitalWrite(LED_SD, LOW);
  //  digitalWrite(LED_CHG, LOW);
  lcd.clear();
  lcd.print("Inverter On");
  criticalBat = false;
  criticalTem = false;
  vBat = (float)analogRead(VB_SENS_PIN) * ADC_RES * VB_SENS_FACTOR;
  tem = (float)analogRead(T_SENS_PIN) * 0.4888;    // LM35: V_out = temperature (°C) * 10 mV
  if (vBat > VB_LOW && tem < T_C) {
    PORTB = 0;                                            // Enable gate drivers
    PORTB |= positiveHalf ? _BV(LS1) : _BV(LS2);          // To charge the bootstrap capacitors
    delay(1);
    //PORTB = _BV(POW_RLY);                               // Switch power output to inverter
    PORTB =  _BV(SD0);                                    // Shut down gate drivers
    flagWait = false;
    lowBat = false;                  // Reset all the protection flags
    overCurrent = false;
    flagShutDown = false;
    zeroCrossing = false;
    sc = false;
    rampUp = true;
    beep(shortBeep, 3);
    TCCR1A = positiveHalf ? 0b11000000 : 0b00110000;       // To start with: +Ve half-> Set OC1A (pin D9) on Compare Match; Disconnect OC1B (pin D10); -Ve half-> Opposite
    TCCR1B = 0b00010001;       // PWM, Phase and Frequency Correct (Mode 8); Clock Select = System clock (No Prescaling) [Ref. Data Sheet, p. 132]
    ICR1 = MAX_COUNT;          // Switching frequency = 16 kHz (1 / (MAX_COUNT * 62.5e-9))
    OCR1A = lookUp[0];
    OCR1B = lookUp[0];
    TIMSK1 = 0b00000001;       // TOIE1 = 1: Overflow Interrupt Enable
    PORTB &=  ~_BV(SD0);       // Pull shutdown low (Enable gate drivers)
    PORTB |= positiveHalf ? _BV(LS2) : _BV(HS2);          // If the last grid cycle was +Ve half then start with -Ve half and vice versa
  }
  else {
    if (vBat < VB_LOW)
      criticalBat = true;
    else if (tem >= T_C)
      criticalTem = true;
    shutDown();
  }
  mode = INV_MODE;
}

void shutDown(void)
{
  PORTB |=  _BV(SD0);              // shutdown the gate driver ICs
  TIMSK1 = 0b00000000;             // TOIE1 = 0: Overflow Interrupt Disabled
  TCCR1A = 0b00000000;             // OC1A (pin D9) Disconnected;  OC1B (pin D10) Disconnected, Phase and Frequency Correct (Mode 8);
  PORTB =  _BV(SD0);
  digitalWrite(CHNG_OVR_RLY_PIN, HIGH);           // Switch power output to inverter
  delay(3);
  digitalWrite(CHG_RLY_PIN, LOW);  // Charging relay off
  zeroCrossing = false;
  toggleHalfBridge2 = false;
  flagShutDown = true;
  flagWait = false;
  modIndx = 80;
  changeDuty();
  lcd.setCursor(0, 1);
  lcd.print(criticalBat ? "BAT LOW  " : criticalTem ? "THERMAL SHUTDOWN" : overCurrent ? "OVER LOAD       " : "OVER LOAD       ");
  if (criticalBat) {
    lcd.print(vBat);
    lcd.print("V");
  }
  if (tem > T_HIGH && !flagFan) {
    digitalWrite(FAN_PIN, HIGH);
    flagFan = true;
  }
  else if (tem < T_LOW && flagFan) {
    digitalWrite(FAN_PIN, LOW);
    flagFan = false;
  }
  digitalWrite(LED_SD, HIGH);
  eventTime = millis();
}

void startChg(void)
{
  vBat = (float)analogRead(VB_SENS_PIN) * ADC_RES * VB_SENS_FACTOR;
  tem = (float)analogRead(T_SENS_PIN) * 0.4888;    // LM35: V_out = temperature (°C) * 10 mV
  if (vBat < VB_CHG_START && tem < T_C) {
    digitalWrite(CHG_RLY_PIN, HIGH);
    digitalWrite(TAPPING_RLY_PIN, LOW);
    delay(3);
    PORTB =  _BV(SD0);    // Shutdown the gate driver ICs.
    TCCR1A = 0b00000010;  // OC1A disconnected & OC1B Clear on comp match;
    TCCR1B = 0b00011001;  // Fast PWM Mode (Mode 14); Clock Select = System clock (No Prescaling) [Ref. Data Sheet, p. 132]
    ICR1 = 3200;          // switching frequency = 5 kHz.
    OCR1A = 0;            // pins LS1 & LS2 are set at interrupt on compare match A
    OCR1B = 0;            // pins LS1 & LS2 are reset at interrupt on compare match B. Min. duty cycle is 0%
    TIMSK1 = 0;           // No Interrupts
    flagCharging = true;
    flagOverVoltage = false;
    flagFloatChg = false;
    criticalTem = false;
    //    digitalWrite(LED_CHG, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("Charging       A");
  }
  else {
    digitalWrite(CHG_RLY_PIN, LOW);
    PORTB =  _BV(SD0);     // Shutdown the gate driver ICs.
    flagCharging = false;
    lcd.setCursor(0, 1);
    lcd.print("                ");
  }
  flag125vTapping = false;
  flagChgSw = false;     // Switching has not been started
  cc = 300;
  if (tem > T_HIGH) {
    digitalWrite(FAN_PIN, HIGH);
    flagFan = true;
  }
  else if (tem < T_LOW) {
    digitalWrite(FAN_PIN, LOW);
    flagFan = false;
  }
}

void stopChg(void)
{
  TIMSK1 = 0b00000000;         // TOIE1 = 0: All timer1 Interrupts Disabled
  TCCR1A = 0b00000000;         // OC1A (pin D9) Disconnected;  OC1B (pin D10) Disconnected;
  PORTB =  _BV(SD0);           // Shutdown switching
  flagChgSw = false;                  // Status flag to indicate that the charging switching is shut down
  digitalWrite(CHG_RLY_PIN, LOW);     // Disengage the charging relay
  flagCharging = false;
  //  digitalWrite(LED_CHG, LOW);
  lcd.setCursor(0, 1);
  lcd.print("                ");
}

void actuateCurrent(void)
{
  if (!flagChgSw) {
    if (current < I_SET && vBat < VB_CHG_START) {
      lcd.setCursor(0, 1);
      lcd.print("Charging       A");
      lcd.setCursor(9, 1);  // Update current and voltage
      lcd.print(current);
      lcd.setCursor(10, 0);
      lcd.print(vBat);
      TCCR1A = 0b00100010;  // OC1A disconnected & OC1B Clear on comp match;
      OCR1B = 0;  // Start with min duty cycle
      PORTB &=  ~_BV(SD0);  // Enable gate drivers
      TIMSK1 = 0b00000110;  // Interrupts on compare match A & B
      flagChgSw = true;
      flagFloatChg = false;
    }
  }

  /* ====================== Fast Charging ====================== */
  if (!flagFloatChg) {
    if (current < I_SET && vBat < VB_SET) {
      if (OCR1B < 1280) {    // Maximum duty cycle 40% (3200 * 40%);
        OCR1B = OCR1B + 1;   // Increase current slowly
      }
    }
    else if (vBat > VB_SET || current > I_SET) {
      if (OCR1B > 0) {       // Minimum possible duty cycle 0%
        OCR1B = OCR1B - 1;   // Decrease current slowly
      }
    }
  }

  /* ====================== Floating Charge ====================== */
  else {
    if (current < I_SET && vBat < VB_FLOAT) {
      if (OCR1B < 640) {    // Maximum duty cycle 20% (3200 * 20%);
        OCR1B = OCR1B + 1;   // Increase current slowly
      }
    }
    else if (vBat > VB_FLOAT || current > I_SET) {
      if (OCR1B > 0) {       // Minimum possible duty cycle 0%
        OCR1B = OCR1B - 1;   // Decrease current slowly
      }
    }
  }
  if (vBat > VB_CHG_STOP) {  // For protection against over charging
    if (OCR1B == 0)
      stopChg();
  }

  cc--;
  if (cc <= 0) {
    cc = 300;                 // Update current in the lcd every 3 s
    if (vBat > VB_FLOAT && current < I_FLOAT) {
      if (OCR1B < 640) {      // (If duty cycle is less than 5%) Fast charging ended, swithed to float charge
        if (!flagFloatChg) {  // Not in float charge mode
          //      digitalWrite(LED_CHG, LOW);
          flagFloatChg = true;
          lcd.setCursor(0, 1);
          lcd.print("                ");
        }
      }
    }
    else if (current > I_FAST) {
      if (flagFloatChg) {
        //      digitalWrite(LED_CHG, HIGH);
        flagFloatChg = false;
        lcd.setCursor(0, 1);
        lcd.print("Charging       A");
      }
    }
    if (!flagFloatChg) {
      lcd.setCursor(9, 1);
      lcd.print(current);
    }
  }
  if (cc == 150) {
    lcd.setCursor(10, 0);
    lcd.print(vBat);
  }
}

void beep(byte t = shortBeep, byte n = 1, unsigned int p = 500)  // beep(<type of beep>, <number of beeps>, <pause time in ms between beeps>)
{
  static bool beepState = false;
  static unsigned int beepTime;
  static unsigned int beepPause;
  static unsigned long beepToggled = 0;
  static int noOfBeep = 0;
  if (!flagBeep) {
    beepTime = t;
    beepPause = p;
    noOfBeep = n;
    digitalWrite(BUZZ_PIN, HIGH);
    beepToggled = millis();
    beepState = true;
    flagBeep = true;
  }
  if (flagBeep) {
    unsigned long timeNow = millis();
    unsigned long beepRun = beepToggled > timeNow ? (4294967295 - beepToggled) + timeNow : timeNow - beepToggled;
    if (beepState) {
      if (beepRun > beepTime) {
        digitalWrite(BUZZ_PIN, LOW);
        beepToggled = millis();
        beepState = false;
        noOfBeep--;
      }
    }
    else {
      if (beepRun > beepPause) {  // Blanking time 500 ms between two beeps
        if (noOfBeep) {
          digitalWrite(BUZZ_PIN, HIGH);
          beepToggled = millis();
          beepState = true;
        }
        else {
          flagBeep = false;
        }
      }
    }
  }
}


void error(void)
{
  while (1) {
    digitalWrite(LED_PIN, HIGH);
    delay(300);
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }
}
