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
const int EEPROM_R6_MODE = 5; // 0=auto, 1=forced off, 2=forced on
const int EEPROM_CHECKBYTE = 10; // To check if EEPROM was initialized

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

// relay desired state (we maintain local desired states so S packet is exact)
uint8_t desiredState = 0x00; // bits 0..5 => R1..R6

// force flags
bool forceR5 = false; // forced roof ON (via long press on btn3)
bool forceR6 = false; // forced reverse ON (via long press on btn4)
uint8_t r6Mode = 0; // 0=auto (follow A2), 1=forced off, 2=forced on

// bypass
bool bypassMode = false;
bool firstPressInBypass = true; // Track if it's the first press after entering bypass

// Delayed button processing for bypass mode
unsigned long btn1PressTime = 0;
unsigned long btn2PressTime = 0;
bool btn1Pending = false;
bool btn2Pending = false;
const unsigned long BYPASS_BUTTON_DELAY = 500; // 500ms delay before processing individual buttons in bypass

// alerts
// alertMode: 0 none, 1 headlight+door (3 beeps x5), 2 door+power (2 beeps x5)
int alertMode = 0;
int alertCount = 0;
unsigned long alertTimer = 0;
int alertStep = 0;
const unsigned long ALERT_UNIT_MS = 200;

// scheduled second beep for blocked events
unsigned long schedSecondBeep = 0;

// button press waiting flags
bool btn3PressedWaiting = false;
bool btn4PressedWaiting = false;

// simultaneous press latches
bool bypassLatched = false;
bool flasherLatched = false;

// LED TX indicator
unsigned long ledTxUntil = 0;

// Button 4 press counter and timing
unsigned long btn4LastPress = 0;
int btn4PressCount = 0;
const unsigned long BTN4_PRESS_WINDOW = 1000; // 1 second window for double press

// Save states to EEPROM
void saveStates() {
  EEPROM.update(EEPROM_BYPASS, bypassMode);
  EEPROM.update(EEPROM_FORCE_R5, forceR5);
  EEPROM.update(EEPROM_FORCE_R6, forceR6);
  EEPROM.update(EEPROM_DESIRED_STATE, desiredState);
  EEPROM.update(EEPROM_FIRST_PRESS, firstPressInBypass);
  EEPROM.update(EEPROM_R6_MODE, r6Mode);
}

// Load states from EEPROM
void loadStates() {
  // Check if EEPROM was initialized (first run)
  if (EEPROM.read(EEPROM_CHECKBYTE) != 0x55) {
    // Initialize EEPROM
    EEPROM.update(EEPROM_BYPASS, 0);
    EEPROM.update(EEPROM_FORCE_R5, 0);
    EEPROM.update(EEPROM_FORCE_R6, 0);
    EEPROM.update(EEPROM_DESIRED_STATE, 0);
    EEPROM.update(EEPROM_FIRST_PRESS, 1);
    EEPROM.update(EEPROM_R6_MODE, 0);
    EEPROM.update(EEPROM_CHECKBYTE, 0x55);
    return;
  }
  
  bypassMode = EEPROM.read(EEPROM_BYPASS);
  forceR5 = EEPROM.read(EEPROM_FORCE_R5);
  forceR6 = EEPROM.read(EEPROM_FORCE_R6);
  desiredState = EEPROM.read(EEPROM_DESIRED_STATE);
  firstPressInBypass = EEPROM.read(EEPROM_FIRST_PRESS);
  r6Mode = EEPROM.read(EEPROM_R6_MODE);
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
        // pressed edge
        pressStart[idx] = millis();
        held[idx] = true;
        return true;
      } else {
        // released
        held[idx] = false;
        return false;
      }
    }
  }
  return false;
}

// send heartbeat or commands
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
  saveStates(); // Save state after any change
}

void flashTxLed(int ms) {
  digitalWrite(LED_STAT, HIGH);
  ledTxUntil = millis() + ms;
}

// helper to send flasher toggle
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
  
  // Load saved states from EEPROM
  loadStates();
  
  // Ensure relays are OFF on startup regardless of saved state
  desiredState = 0x00;
  sendSetState();
  
  // small startup beep
  startBeep(2000, 40);
  lastHeartbeat = millis();
}

void loop() {
  unsigned long currentMillis = millis();
  
  // update non-blocking tasks
  stopBeepIfDue();
  
  // Clear TX LED if time expired
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

  // Apply R6 mode based on settings
  if (forceR6) {
    // Forced ON (long press)
    desiredState |= 0x20; // R6 ON
  } else if (r6Mode == 1) {
    // Forced OFF (double press)
    desiredState &= ~0x20; // R6 OFF
  } else {
    // Auto mode (follow A2)
    if (revOn) {
      desiredState |= 0x20; // R6 ON
    } else {
      desiredState &= ~0x20; // R6 OFF
    }
  }

  // alerts: set mode when conditions start
  if (headOn && doorOn) {
    if (alertMode != 1) { 
      alertMode = 1; 
      alertCount = 0; 
      alertStep = 0; 
      alertTimer = 0; 
    }
  } else if (!headOn && doorOn && pwrOn) {
    if (alertMode != 2) { 
      alertMode = 2; 
      alertCount = 0; 
      alertStep = 0; 
      alertTimer = 0; 
    }
  } else {
    // clear
    alertMode = 0;
    alertCount = 0;
    alertStep = 0;
    alertTimer = 0;
  }

  // run alert patterns non-blocking
  if (alertMode != 0 && alertCount < 5) {
    if (alertTimer == 0) alertTimer = currentMillis;
    if (currentMillis - alertTimer >= ALERT_UNIT_MS) {
      alertTimer = currentMillis;
      if (alertMode == 1) {
        // 3 short beeps then pause
        if (alertStep < 6) {
          // even steps -> beep
          if ((alertStep % 2) == 0) startBeep(1600, 120);
          alertStep++;
        } else {
          alertStep = 0;
          alertCount++;
          // long pause equal to 3 units -> skip by moving timer forward
          alertTimer += ALERT_UNIT_MS * 2;
        }
      } else if (alertMode == 2) {
        // 2 beeps then pause
        if (alertStep < 4) {
          if ((alertStep % 2) == 0) startBeep(1200, 140);
          alertStep++;
        } else {
          alertStep = 0;
          alertCount++;
          alertTimer += ALERT_UNIT_MS * 2;
        }
      }
    }
  }

  // read buttons (debounced)
  bool edge1 = sampleButton(BTN1, 0);
  bool edge2 = sampleButton(BTN2, 1);
  bool edge3 = sampleButton(BTN3, 2);
  bool edge4 = sampleButton(BTN4, 3);

  // Track previous bypass mode to detect changes
  static bool prevBypassMode = false;
  
  // detect simultaneous press for bypass (1+2)
  if (stable[0] == LOW && stable[1] == LOW) {
    if (!bypassLatched) {
      bool oldBypassMode = bypassMode; // Store old state
      bypassMode = !bypassMode;
      bypassLatched = true;
      
      // Handle bypass mode changes
      if (bypassMode) {
        // Entering bypass mode
        firstPressInBypass = true;
        startBeep(2200, 120); // High pitch beep for entering bypass
        
        // Clear any pending button actions when entering bypass
        btn1Pending = false;
        btn2Pending = false;
      } else {
        // Exiting bypass mode - turn off relays 1&2 and 3&4
        desiredState &= ~(0x01 | 0x02 | 0x04 | 0x08); // Clear bits for R1,R2,R3,R4
        sendSetState();
        startBeep(1800, 120); // Different pitch for exiting bypass
        
        // Clear any pending button actions when exiting bypass
        btn1Pending = false;
        btn2Pending = false;
      }
      saveStates();
    }
  } else {
    bypassLatched = false;
  }
  
  prevBypassMode = bypassMode;

  // Handle delayed button processing in bypass mode
  if (bypassMode) {
    // If button 1 is pressed and we're not already processing it
    if (edge1 && !btn1Pending && !btn2Pending) {
      btn1PressTime = currentMillis;
      btn1Pending = true;
    }
    
    // If button 2 is pressed and we're not already processing it  
    if (edge2 && !btn2Pending && !btn1Pending) {
      btn2PressTime = currentMillis;
      btn2Pending = true;
    }
    
    // Check if button 1 delay has expired
    if (btn1Pending && (currentMillis - btn1PressTime >= BYPASS_BUTTON_DELAY)) {
      btn1Pending = false;
      
      // Check if button 2 was also pressed during the delay period (simultaneous press)
      if (stable[1] == LOW) {
        // Both buttons are pressed - this should trigger bypass exit, so ignore individual action
        // The bypass mode handler above will take care of it
      } else {
        // Only button 1 was pressed - process it
        if (firstPressInBypass) {
          // First press in bypass mode - just beep but don't toggle
          startBeep(1000, 60);
          firstPressInBypass = false;
          saveStates();
        } else {
          // Normal operation in bypass mode
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
    
    // Check if button 2 delay has expired
    if (btn2Pending && (currentMillis - btn2PressTime >= BYPASS_BUTTON_DELAY)) {
      btn2Pending = false;
      
      // Check if button 1 was also pressed during the delay period (simultaneous press)
      if (stable[0] == LOW) {
        // Both buttons are pressed - ignore individual action
      } else {
        // Only button 2 was pressed - process it
        if (firstPressInBypass) {
          // First press in bypass mode - just beep but don't toggle
          startBeep(1000, 60);
          firstPressInBypass = false;
          saveStates();
        } else {
          // Normal operation in bypass mode
          bool newOn = !((desiredState & 0x04) || (desiredState & 0x08));
          if (newOn) desiredState |= (0x04 | 0x08);
          else desiredState &= ~(0x04 | 0x08);
          sendSetState();
          startBeep(1300, 90);
        }
      }
    }
  } else {
    // Not in bypass mode - process buttons immediately (normal operation)
    
    // BUTTON 1 pressed (edge) - normal mode
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
        // blocked -> schedule double low beep
        startBeep(700, 80);
        schedSecondBeep = currentMillis + 120;
      }
    }

    // BUTTON 2 pressed (edge) - normal mode
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

  // NEW: Fixed Button 3 logic with proper interlock
  // BUTTON 3 pressed (edge)
  if (edge3) {
    btn3PressedWaiting = true;
  }

  // BUTTON 3 long press handling (while button is held)
  if (stable[2] == LOW) {
    // Check for long press
    if (held[2] && (currentMillis - pressStart[2] >= LONG_MS) && !forceR5) {
      forceR5 = true;
      desiredState |= 0x10; // set R5 ON
      sendSetState();
      startBeep(1500, 110);
      // ensure we only trigger once per hold:
      held[2] = false; // suppress further long triggers until release
      btn3PressedWaiting = false; // Prevent short press action after long press
      saveStates();
    }
  }
  
  // BUTTON 3 short press release (only if not handled as long press)
  if (btn3PressedWaiting && stable[2] == HIGH) {
    btn3PressedWaiting = false;
    
    // If forceR5 is active, short press turns it off and returns to interlock
    if (forceR5) {
      forceR5 = false;
      desiredState &= ~0x10; // turn R5 off
      sendSetState();
      startBeep(1000, 90);
      saveStates();
    } else {
      // Normal short press - check interlock conditions
      if (headOn || bypassMode) {
        // Headlights on or in bypass - toggle R5
        desiredState ^= 0x10; // toggle R5
        sendSetState();
        startBeep(1000, 90);
      } else {
        // Headlights off and not in bypass - just beep to indicate no action
        startBeep(700, 80);
      }
    }
  }

  // BUTTON 4 pressed (edge)
  if (edge4) {
    btn4PressedWaiting = true;
    
    // Track button 4 press count for double press detection
    if (currentMillis - btn4LastPress > BTN4_PRESS_WINDOW) {
      btn4PressCount = 0;
    }
    btn4PressCount++;
    btn4LastPress = currentMillis;
  }

  // BUTTON 4 long press and release handling
  if (stable[3] == LOW) {
    if (held[3] && (currentMillis - pressStart[3] >= LONG_MS) && !forceR6) {
      // Long press - force R6 ON regardless of A2 state
      forceR6 = true;
      r6Mode = 2; // forced on
      sendSetState();
      startBeep(1700, 110);
      held[3] = false; // suppress further triggers until release
      btn4PressedWaiting = false; // Prevent short press action after long press
      saveStates();
    }
  }

  // BUTTON 4 short press release (only if not handled as long press)
  if (btn4PressedWaiting && stable[3] == HIGH) {
    btn4PressedWaiting = false;
    
    // If forceR6 is active, short press returns to previous state
    if (forceR6) {
      forceR6 = false;
      r6Mode = 0; // back to auto mode
      sendSetState();
      startBeep(1200, 90);
      saveStates();
    } else {
      // Handle single/double press when not in forced mode
      if (btn4PressCount == 1) {
        // Single press - turn on R6 if A2 has 12V (handled automatically in main loop)
        // Just beep to acknowledge
        startBeep(1200, 60);
      } else if (btn4PressCount == 2) {
        // Double press - force R6 OFF even when A2 has 12V
        r6Mode = 1; // forced off
        sendSetState();
        startBeep(1400, 90); // Different beep for forced off
        saveStates();
      }
      // Reset press count after processing
      btn4PressCount = 0;
    }
  }

  // Buttons 3+4 together -> flasher toggle
  if (stable[2] == LOW && stable[3] == LOW) {
    if (!flasherLatched) {
      sendFlasherToggle();
      flashTxLed(6);
      startBeep(1800, 110);
      flasherLatched = true;
    }
  } else {
    flasherLatched = false;
  }

  // scheduled secondary beeps for blocked events
  if (schedSecondBeep && currentMillis >= schedSecondBeep) {
    startBeep(700, 80);
    schedSecondBeep = 0;
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