#include <Arduino.h>


/**
 *                               ###########
 *   Amp Channel Footswitch --< |           |  <===> Equilizer    
 *   Amp FX Send -------------> |    CRL    |  <===> Delay 
 *   amp FX Return -----------< |           |  <===> Noise Gate
 *   Amp Input ---------------< |           |  <---- Over Drive <---- Guitar Output
 *                               ###########
 * ----------------------------------------------------------------------------------
 * 
 *   Model K 
 *   - Mute relay: DPDT
 *   - Over Drive relay: DPDT
 *   - Foot Switch: Latching
 * 
 *   Model A
 *   - Mute relay: DPDT
 *   - Foot Switch: type = Push button, LED indication; High = 7.5 V, Low = 5.5V;
 *        Use voltage divider 3/2 to scale max. voltage to 5V; use Arduino analog inputs
 */

#define MODEL_K

#ifdef MODEL_K
  #define ODRIVE_RELAY_SET_PIN 2

  #define EQ_RELAY_SET_PIN    7
  #define DELAY_RELAY_SET_PIN 10
  #define NGATE_RELAY_SET_PIN 8

  #define MUTE_RELAY_SET_PIN  6

  #define CLEAN_DIRTY_BUTTON_PIN 12
  #define RYTHM_LEAD_BUTTON_PIN  13

#else // Model A
  #define ODRIVE_RELAY_SET_PIN 2

  #define EQ_RELAY_SET_PIN    7
  #define DELAY_RELAY_SET_PIN 10
  #define NGATE_RELAY_SET_PIN 8

  #define MUTE_RELAY_SET_PIN  6

  #define CLEAN_DIRTY_BUTTON_PIN 12
  #define RYTHM_LEAD_BUTTON_PIN  13

  #define FOOT_SWITCH_ANALOG_THRESHOLD 818
#endif 


#define BUTTON_UNDEFINED 0
#define BUTTON_NO_CHANGE 0
#define BUTTON_PRESSED   1
#define BUTTON_RELEASED  2


#define CLEAN_DIRTY_CHANNEL_MASK 0b01
#define LEAD_RYTHM_CHANNEL_MASK 0b10

#define DIRTY_CHANNEL_ON_FLAG 0b01
#define LEAD_CHANNEL_ON_FLAG 0b10


static int pedalState = 0;
static uint8_t histA = 0;
static uint8_t histB = 0;


/**
 * 
 */
int readFootSwitch(uint8_t pin) {
#ifdef MODEL_K
  return digitalRead(pin);
#else
  return analogRead(pin) > FOOT_SWITCH_ANALOG_THRESHOLD ? 1 : 0;
#endif
}


/**
 * 
 */
int pollButton(uint8_t* pHist, uint8_t pin) {
  *pHist = (*pHist << 1) | !readFootSwitch(pin);

  if ((*pHist & 0b11000111) == 0b00000111) {	
    *pHist = 0xFF;
    return BUTTON_PRESSED;
  }

  if ((*pHist & 0b11000111) == 0b11000000) {
    *pHist = 0x00;
    return BUTTON_RELEASED;
  }

  return BUTTON_NO_CHANGE;
}


/**
 *
 */
bool isButtonDown(uint8_t* pHist) {
  return (*pHist == 0b11111111);
}


/**
 * 
 */
void initSwitches() {
  for (int i=0; i < 8; i++) {
    pollButton(&histA, CLEAN_DIRTY_BUTTON_PIN);
    pollButton(&histB, RYTHM_LEAD_BUTTON_PIN);
    delay(15);
  }
}


/**
 *
 */
void applyState() {
  digitalWrite(MUTE_RELAY_SET_PIN, 1);
  delay(50);

  if ((pedalState & CLEAN_DIRTY_CHANNEL_MASK) == DIRTY_CHANNEL_ON_FLAG) {
    if ((pedalState & LEAD_RYTHM_CHANNEL_MASK) == LEAD_CHANNEL_ON_FLAG) {
      // Lead settings
      digitalWrite(EQ_RELAY_SET_PIN, 1);
      digitalWrite(DELAY_RELAY_SET_PIN, 1);
      digitalWrite(NGATE_RELAY_SET_PIN, 1);
      digitalWrite(ODRIVE_RELAY_SET_PIN, 1);
    } else {
      // Rythm settings
      digitalWrite(EQ_RELAY_SET_PIN, 0);
      digitalWrite(DELAY_RELAY_SET_PIN, 0);
      digitalWrite(NGATE_RELAY_SET_PIN, 1);
#ifdef MODEL_K
      digitalWrite(ODRIVE_RELAY_SET_PIN, 0);
#else
      digitalWrite(ODRIVE_RELAY_SET_PIN, 1);
#endif
    }
  } else {
    // clean settings
    digitalWrite(EQ_RELAY_SET_PIN, 0);
    digitalWrite(DELAY_RELAY_SET_PIN, 1);
    digitalWrite(NGATE_RELAY_SET_PIN, 0);
    digitalWrite(ODRIVE_RELAY_SET_PIN, 0);
  }

  delay(50);
  digitalWrite(MUTE_RELAY_SET_PIN, 0);
}


/**
 * 
 */
void setup() {
  // put your setup code here, to run once:
  pinMode(ODRIVE_RELAY_SET_PIN, OUTPUT);

  pinMode(EQ_RELAY_SET_PIN, OUTPUT);
  pinMode(DELAY_RELAY_SET_PIN, OUTPUT);
  pinMode(NGATE_RELAY_SET_PIN, OUTPUT);
  pinMode(MUTE_RELAY_SET_PIN, OUTPUT);

#ifdef MODEL_K
  pinMode(CLEAN_DIRTY_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RYTHM_LEAD_BUTTON_PIN, INPUT_PULLUP);
#endif

  initSwitches();

  if (isButtonDown(&histA)) {
    pedalState |= DIRTY_CHANNEL_ON_FLAG;
  }

  if (isButtonDown(&histB)) {
    pedalState |= LEAD_CHANNEL_ON_FLAG;
  }

  applyState();
}


/**
 * 
 */
void loop() {
  int eventA = pollButton(&histA, CLEAN_DIRTY_BUTTON_PIN);
  int eventB = pollButton(&histB, RYTHM_LEAD_BUTTON_PIN);
  int oldState = pedalState;
  
  if (BUTTON_PRESSED == eventA) {
    pedalState &= ~CLEAN_DIRTY_CHANNEL_MASK; 
    pedalState |= DIRTY_CHANNEL_ON_FLAG; 
  } else if (BUTTON_RELEASED == eventA) {
    pedalState &= ~CLEAN_DIRTY_CHANNEL_MASK; 
  }
  
  if (BUTTON_PRESSED == eventB) {
    pedalState &= ~LEAD_RYTHM_CHANNEL_MASK;
    pedalState |= LEAD_CHANNEL_ON_FLAG; 
  } else if (BUTTON_RELEASED == eventB) {
    pedalState &= ~LEAD_RYTHM_CHANNEL_MASK; 
  }

  if (oldState != pedalState) {
    if (oldState & CLEAN_DIRTY_CHANNEL_MASK != pedalState & CLEAN_DIRTY_CHANNEL_MASK) {
      applyState();
    } else if (pedalState & CLEAN_DIRTY_CHANNEL_MASK == DIRTY_CHANNEL_ON_FLAG) {
      applyState();
    }
  }

  delay(15);
}
