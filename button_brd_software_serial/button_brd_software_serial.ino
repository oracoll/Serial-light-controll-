/*
 Button Board - ATmega168 (16MHz)
 TX on D2 (SoftwareSerial), RX disabled
 Buttons:
  D5 = Btn1
  D6 = Btn2
  D7 = Btn3
  D8 = Btn4
 Buzzer: D11 (passive)
 Analogs:
  A0 = headlights
  A1 = door open
  A2 = reverse light
  A3 = board power
  A4 = spare
  A5 = spare

 Protocol:
  'H'                 - heartbeat (single byte)
  'F'                 - toggle flasher (single byte)
  'S' <state_byte>    - set relays exactly: bit0..bit5 -> R1..R6
*/

#include <SoftwareSerial.h>
#include <EEPROM.h>

#define TX_PIN 2
SoftwareSerial link(-1, TX_PIN); // TX enabled, RX disabled

// Pins
const uint8_t BTN1 = 5;
const uint8_t BTN2 = 6;
const uint8_t BTN3 = 7;
const uint8_t BTN4 = 8;
const uint8_t BUZZ = 11;
const uint8_t LED_STAT = 13;

// Analogs
const uint8_t A_HEAD = A0;
const uint8_t A_DOOR = A1;
const uint8_t A_REV = A2;
const uint8_t A_PWR = A3;

// EEPROM addresses
const int EEPROM_BYPASS = 0;
const int EEPROM_FORCE_R5 = 1;
const int EEPROM_FORCE_R6 = 2;
const int EEPROM_DESIRED_STATE = 3;
const int EEPROM_FIRST_PRESS = 4;
const int EEPROM_REVERSE_LIGHT_MODE = 5; // 0=auto, 1=manual off
const int EEPROM_CHECKBYTE = 10;

// timings
const unsigned long HEARTBEAT_MS = 500;
unsigned long lastHeartbeat = 0;

// debounce
const unsigned long DEBOUNCE_MS = 40;
bool lastRaw[4] = {HIGH, HIGH, HIGH, HIGH};
bool stable[4]  = {HIGH, HIGH, HIGH, HIGH};
unsigned long lastDebounceTime[4] = {0,0,0,0};

// long press
const unsigned long LONG_MS = 3000;
const unsigned long SIMULTANEOUS_PRESS_DELAY = 200; // Delay to detect if btn3+btn4 pressed together
unsigned long pressStart[4] = {0,0,0,0};
bool held[4] = {false,false,false,false};

// buzzer non-blocking
bool beeping = false;
unsigned long beepEnd = 0;
void startBeep(int freq, int dur) {
  tone(BUZZ, freq);
  beeping = true;
  beepEnd = millis() + dur;
}
void stopBeepIfDue() {
  if (beeping && millis() >= beepEnd) {
    noTone(BUZZ);
    beeping = false;
  }
}

// relay desired state
uint8_t desiredState = 0x00;

// force flags
bool forceR5 = false; // forced roof ON (via long press on btn3)
bool forceR6 = false; // forced reverse ON (via long press on btn4)
uint8_t reverseLightMode = 0; // 0=auto, 1=manual off

// bypass
bool bypassMode = false;
bool firstPressInBypass = true;

// Delayed button processing for bypass mode
unsigned long btn1PressTime = 0;
unsigned long btn2PressTime = 0;
bool btn1Pending = false;
bool btn2Pending = false;
const unsigned long BYPASS_BUTTON_DELAY = 500;

// alerts
const unsigned long ALERT_UNIT_MS = 200;

// Alert tracking for Headlights (A0) + Door (A1)
bool prevHeadlightDoorCombo = false;
int headlightDoorAlertCount = 0;
unsigned long headlightDoorAlertTimer = 0;
int headlightDoorAlertStep = 0;

// Alert tracking for Door (A1) + Power (A3)
bool prevDoorPowerCombo = false;
int doorPowerAlertCount = 0;
unsigned long doorPowerAlertTimer = 0;
int doorPowerAlertStep = 0;

// scheduled second beep for blocked events
unsigned long schedSecondBeep = 0;
unsigned long schedSecondBeep2 = 0;

// button press tracking with simultaneous press detection
bool btn3PressWaiting = false;
bool btn4PressWaiting = false;
unsigned long btn3PressTime = 0;
unsigned long btn4PressTime = 0;

// simultaneous press latches
bool bypassLatched = false;
bool flasherLatched = false;

// LED TX indicator
unsigned long ledTxUntil = 0;

// Save states to EEPROM
void saveStates() {
  EEPROM.update(EEPROM_BYPASS, bypassMode);
  EEPROM.update(EEPROM_FORCE_R5, forceR5);
  EEPROM.update(EEPROM_FORCE_R6, forceR6);
  EEPROM.update(EEPROM_DESIRED_STATE, desiredState);
  EEPROM.update(EEPROM_FIRST_PRESS, firstPressInBypass);
  EEPROM.update(EEPROM_REVERSE_LIGHT_MODE, reverseLightMode);
}

// Load states from EEPROM
void loadStates() {
  if (EEPROM.read(EEPROM_CHECKBYTE) != 0x55) {
    EEPROM.update(EEPROM_BYPASS, 0);
    EEPROM.update(EEPROM_FORCE_R5, 0);
    EEPROM.update(EEPROM_FORCE_R6, 0);
    EEPROM.update(EEPROM_DESIRED_STATE, 0);
    EEPROM.update(EEPROM_FIRST_PRESS, 1);
    EEPROM.update(EEPROM_REVERSE_LIGHT_MODE, 0);
    EEPROM.update(EEPROM_CHECKBYTE, 0x55);
    return;
  }
  
  bypassMode = EEPROM.read(EEPROM_BYPASS);
  forceR5 = EEPROM.read(EEPROM_FORCE_R5);
  forceR6 = EEPROM.read(EEPROM_FORCE_R6);
  desiredState = EEPROM.read(EEPROM_DESIRED_STATE);
  firstPressInBypass = EEPROM.read(EEPROM_FIRST_PRESS);
  reverseLightMode = EEPROM.read(EEPROM_REVERSE_LIGHT_MODE);
}

// helper: read & debounce button, return true when a new press (edge LOW) is detected
bool sampleButton(uint8_t pin, int idx) {
  bool raw = digitalRead(pin);
  if (raw != lastRaw[idx]) {
    lastDebounceTime[idx] = millis();
    lastRaw[idx] = raw;
  }
  if (millis() - lastDebounceTime[idx] > DEBOUNCE_MS) {
    if (stable[idx] != raw) {
      stable[idx] = raw;
      if (stable[idx] == LOW) {
        pressStart[idx] = millis();
        held[idx] = true;
        return true;
      } else {
        held[idx] = false;
        return false;
      }
    }
  }
  return false;
}

// send heartbeat
void sendHeartbeat() {
  if (millis() - lastHeartbeat >= HEARTBEAT_MS) {
    link.write('H');
    lastHeartbeat = millis();
  }
}

void sendSetState() {
  link.write('S');
  link.write(desiredState & 0x3F);
  flashTxLed(6);
  saveStates();
}

void flashTxLed(int ms) {
  digitalWrite(LED_STAT, HIGH);
  ledTxUntil = millis() + ms;
}

void sendFlasherToggle() {
  link.write('F');
}

// compute ADC thresholds
const int HEAD_TH = 600;
const int DOOR_TH = 200;
const int REV_TH  = 200;
const int PWR_TH  = 200;

void setup() {
  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);
  pinMode(BTN3, INPUT_PULLUP);
  pinMode(BTN4, INPUT_PULLUP);

  pinMode(BUZZ, OUTPUT);
  pinMode(LED_STAT, OUTPUT);
  digitalWrite(LED_STAT, LOW);

  link.begin(9600);
  
  loadStates();
  
  desiredState = 0x00;
  sendSetState();
  
  startBeep(2000, 40);
  lastHeartbeat = millis();
}

void loop() {
  unsigned long currentMillis = millis();
  
  stopBeepIfDue();
  
  if (ledTxUntil && currentMillis >= ledTxUntil) {
    digitalWrite(LED_STAT, LOW);
    ledTxUntil = 0;
  }

  // Read analog inputs
  int a0 = analogRead(A_HEAD);
  int a1 = analogRead(A_DOOR);
  int a2 = analogRead(A_REV);
  int a3 = analogRead(A_PWR);
  bool headOn = (a0 >= HEAD_TH);
  bool doorOn = (a1 >= DOOR_TH);
  bool revOn  = (a2 >= REV_TH);
  bool pwrOn  = (a3 >= PWR_TH);
  bool voltageOk = (a3 >= 204); // Approximately 10V on 12V system (10/12 * 255 ≈ 212)

  // Apply R6 mode based on settings
  if (forceR6) {
    desiredState |= 0x20; // R6 ON (forced)
  } else if (reverseLightMode == 0) { // Auto mode
    if (revOn) {
      desiredState |= 0x20; // R6 ON when reverse light detected
    } else {
      desiredState &= ~0x20; // R6 OFF when no reverse light
    }
  } else { // Manual off mode (1)
    desiredState &= ~0x20; // R6 OFF regardless
  }

  // Send state update if R6 changed due to auto mode
  {
    static uint8_t lastDesiredState = 0;
    if ((desiredState & 0x20) != (lastDesiredState & 0x20)) {
      sendSetState();
      lastDesiredState = desiredState;
    }
  }

  // ========== ALERT: Headlights (A0) + Door (A1) - 5 beeps, medium tone ==========
  bool currentHeadlightDoorCombo = (headOn && doorOn && voltageOk);
  if (currentHeadlightDoorCombo && !prevHeadlightDoorCombo) {
    // Start alert on the rising edge of the condition
    headlightDoorAlertCount = 0;
    headlightDoorAlertStep = 0;
    headlightDoorAlertTimer = 0;
  }
  if (!doorOn) {
    // Stop alert immediately if door is closed
    headlightDoorAlertCount = 5;
  }
  prevHeadlightDoorCombo = currentHeadlightDoorCombo;

  if (currentHeadlightDoorCombo && headlightDoorAlertCount < 5) {
    if (headlightDoorAlertTimer == 0) headlightDoorAlertTimer = currentMillis;
    if (currentMillis - headlightDoorAlertTimer >= ALERT_UNIT_MS) {
      headlightDoorAlertTimer = currentMillis;
      if (headlightDoorAlertStep < 10) {
        if ((headlightDoorAlertStep % 2) == 0) startBeep(1100, 110);
        headlightDoorAlertStep++;
      } else {
        headlightDoorAlertStep = 0;
        headlightDoorAlertCount++;
        headlightDoorAlertTimer += ALERT_UNIT_MS * 1;
      }
    }
  }

  // ========== ALERT: Door (A1) + Power (A3) - 6 beeps, low tone ==========
  bool currentDoorPowerCombo = (doorOn && pwrOn && voltageOk);
  if (currentDoorPowerCombo && !prevDoorPowerCombo) {
    // Start alert on the rising edge of the condition
    doorPowerAlertCount = 0;
    doorPowerAlertStep = 0;
    doorPowerAlertTimer = 0;
  }
  if (!doorOn) {
    // Stop alert immediately if door is closed
    doorPowerAlertCount = 6;
  }
  prevDoorPowerCombo = currentDoorPowerCombo;

  if (currentDoorPowerCombo && doorPowerAlertCount < 6) {
    if (doorPowerAlertTimer == 0) doorPowerAlertTimer = currentMillis;
    if (currentMillis - doorPowerAlertTimer >= ALERT_UNIT_MS) {
      doorPowerAlertTimer = currentMillis;
      if (doorPowerAlertStep < 12) {
        if ((doorPowerAlertStep % 2) == 0) startBeep(800, 100);
        doorPowerAlertStep++;
      } else {
        doorPowerAlertStep = 0;
        doorPowerAlertCount++;
        doorPowerAlertTimer += ALERT_UNIT_MS * 1;
      }
    }
  }

  // read buttons (debounced)
  bool edge1 = sampleButton(BTN1, 0);
  bool edge2 = sampleButton(BTN2, 1);
  bool edge3 = sampleButton(BTN3, 2);
  bool edge4 = sampleButton(BTN4, 3);

  // detect simultaneous press for bypass (1+2)
  if (stable[0] == LOW && stable[1] == LOW) {
    if (!bypassLatched) {
      bypassMode = !bypassMode;
      bypassLatched = true;
      
      if (bypassMode) {
        firstPressInBypass = true;
        startBeep(2200, 120);
        btn1Pending = false;
        btn2Pending = false;
      } else {
        desiredState &= ~(0x01 | 0x02 | 0x04 | 0x08);
        sendSetState();
        startBeep(1800, 120);
        btn1Pending = false;
        btn2Pending = false;
      }
      saveStates();
    }
  } else {
    bypassLatched = false;
  }

  // Handle delayed button processing in bypass mode
  if (bypassMode) {
    if (edge1 && !btn1Pending && !btn2Pending) {
      btn1PressTime = currentMillis;
      btn1Pending = true;
    }
    
    if (edge2 && !btn2Pending && !btn1Pending) {
      btn2PressTime = currentMillis;
      btn2Pending = true;
    }
    
    if (btn1Pending && (currentMillis - btn1PressTime >= BYPASS_BUTTON_DELAY)) {
      btn1Pending = false;
      
      if (stable[1] == LOW) {
        // Both buttons pressed - bypass exit handled above
      } else {
        if (firstPressInBypass) {
          startBeep(1000, 60);
          firstPressInBypass = false;
          saveStates();
        } else {
          bool newOn = !((desiredState & 0x01) || (desiredState & 0x02));
          if (newOn) {
            desiredState |= (0x01 | 0x02);
          } else {
            desiredState &= ~(0x01 | 0x02);
          }
          sendSetState();
          startBeep(1200, 90);
        }
      }
    }
    
    if (btn2Pending && (currentMillis - btn2PressTime >= BYPASS_BUTTON_DELAY)) {
      btn2Pending = false;
      
      if (stable[0] == LOW) {
        // Both buttons pressed - bypass exit handled above
      } else {
        if (firstPressInBypass) {
          startBeep(1000, 60);
          firstPressInBypass = false;
          saveStates();
        } else {
          bool newOn = !((desiredState & 0x04) || (desiredState & 0x08));
          if (newOn) desiredState |= (0x04 | 0x08);
          else desiredState &= ~(0x04 | 0x08);
          sendSetState();
          startBeep(1300, 90);
        }
      }
    }
  } else {
    // Normal mode - process buttons immediately
    
    if (edge1) {
      if (headOn) {
        bool newOn = !((desiredState & 0x01) || (desiredState & 0x02));
        if (newOn) {
          desiredState |= (0x01 | 0x02);
        } else {
          desiredState &= ~(0x01 | 0x02);
        }
        sendSetState();
        startBeep(800, 90);
      } else {
        startBeep(700, 80);
        schedSecondBeep = currentMillis + 120;
      }
    }

    if (edge2) {
      if (headOn) {
        bool newOn = !((desiredState & 0x04) || (desiredState & 0x08));
        if (newOn) desiredState |= (0x04 | 0x08);
        else desiredState &= ~(0x04 | 0x08);
        sendSetState();
        startBeep(900, 90);
      } else {
        startBeep(700, 80);
        schedSecondBeep = currentMillis + 120;
      }
    }
  }

  // ========== BUTTON 3 LOGIC ==========
  // On press, start tracking with delay to detect simultaneous btn3+btn4
  if (edge3) {
    btn3PressTime = currentMillis;
    btn3PressWaiting = true;
  }

  // Check for simultaneous press btn3+btn4 with delay
  if (btn3PressWaiting && currentMillis - btn3PressTime >= SIMULTANEOUS_PRESS_DELAY) {
    if (stable[3] == LOW) {
      // Both btn3 and btn4 are pressed - this is a flasher toggle, NOT relay action
      btn3PressWaiting = false;
      // Flasher toggle is handled in the btn3+btn4 section below
    } else if (currentMillis - btn3PressTime >= LONG_MS) {
      // Long press detected - force R5 ON
      if (!forceR5) {
        forceR5 = true;
        desiredState |= 0x10;
        sendSetState();
        startBeep(1500, 110);
        saveStates();
      }
      btn3PressWaiting = false;
    }
  }

  // Release detection for button 3
  if (btn3PressWaiting && stable[2] == HIGH) {
    btn3PressWaiting = false;
    unsigned long pressDuration = currentMillis - btn3PressTime;
    
    // Only act if press was BEFORE long press threshold
    if (pressDuration < LONG_MS) {
      if (forceR5) {
        // In forced mode - short press turns it off
        forceR5 = false;
        desiredState &= ~0x10;
        sendSetState();
        startBeep(1000, 90);
        saveStates();
      } else {
        // Not in forced mode - check if headlights are on (A0 power)
        if (headOn) {
          // Toggle relay 5
          desiredState ^= 0x10;
          sendSetState();
          startBeep(1000, 90);
        } else {
          // No power on A0 - blocked, beep only
          startBeep(700, 80);
          schedSecondBeep2 = currentMillis + 120;
        }
      }
    }
  }

  // ========== BUTTON 4 LOGIC ==========
  // On press, start tracking with delay to detect simultaneous btn3+btn4
  if (edge4) {
    btn4PressTime = currentMillis;
    btn4PressWaiting = true;
  }

  // Check for simultaneous press btn3+btn4 with delay
  if (btn4PressWaiting && currentMillis - btn4PressTime >= SIMULTANEOUS_PRESS_DELAY) {
    if (stable[2] == LOW) {
      // Both btn3 and btn4 are pressed - this is a flasher toggle, NOT relay action
      btn4PressWaiting = false;
      // Flasher toggle is handled in the btn3+btn4 section below
    } else if (currentMillis - btn4PressTime >= LONG_MS) {
      // Long press detected - force R6 ON
      if (!forceR6) {
        forceR6 = true;
        sendSetState();
        startBeep(1700, 110);
        saveStates();
      }
      btn4PressWaiting = false;
    }
  }

  // Release detection for button 4
  if (btn4PressWaiting && stable[3] == HIGH) {
    btn4PressWaiting = false;
    unsigned long pressDuration = currentMillis - btn4PressTime;
    
    // Only act if press was BEFORE long press threshold
    if (pressDuration < LONG_MS) {
      if (forceR6) {
        // In forced mode - short press turns it off
        forceR6 = false;
        sendSetState();
        startBeep(1200, 90);
        saveStates();
      } else {
        // Not in forced mode - toggle between auto and manual off modes
        reverseLightMode = (reverseLightMode == 0) ? 1 : 0;
        sendSetState();

        if (reverseLightMode == 0) {
          startBeep(1200, 90); // Auto mode beep
        } else {
          startBeep(1400, 90); // Manual off mode beep
        }
        saveStates();
      }
    }
  }

  // Buttons 3+4 together -> flasher toggle (only after SIMULTANEOUS_PRESS_DELAY)
  if (stable[2] == LOW && stable[3] == LOW &&
      currentMillis - pressStart[2] >= SIMULTANEOUS_PRESS_DELAY &&
      currentMillis - pressStart[3] >= SIMULTANEOUS_PRESS_DELAY) {
    if (!flasherLatched) {
      sendFlasherToggle();
      flashTxLed(6);
      startBeep(1800, 110);
      flasherLatched = true;
      btn3PressWaiting = false;
      btn4PressWaiting = false;
    }
  } else {
    flasherLatched = false;
  }

  // scheduled secondary beeps for blocked events
  if (schedSecondBeep && currentMillis >= schedSecondBeep) {
    startBeep(700, 80);
    schedSecondBeep = 0;
  }

  if (schedSecondBeep2 && currentMillis >= schedSecondBeep2) {
    startBeep(700, 80);
    schedSecondBeep2 = 0;
  }

  // send heartbeat regularly
  sendHeartbeat();

  // resend state periodically to keep relay board synced
  static unsigned long lastStateResend = 0;
  if (currentMillis - lastStateResend > 2000) {
    sendSetState();
    lastStateResend = currentMillis;
  }

  delay(1);
}