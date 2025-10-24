/*
 Relay Board - ATmega168 (16MHz)
 RX on D2 (SoftwareSerial), TX disabled
 Relays:
  D3 = R1 (white left)
  D4 = R2 (white right)
  D5 = R3 (amber left)
  D6 = R4 (amber right)
  D7 = R5 (roof)
  D8 = R6 (reverse)
 D13 = status LED (comm blink / timeout)

 Protocol:
  'H'                 - heartbeat (single byte)
  'F'                 - toggle Flasser on/off (single byte)
  'S' <state_byte>    - set relays exactly: bit0..bit5 -> R1..R6
*/

#include <SoftwareSerial.h>

#define RX_PIN 2
SoftwareSerial link(RX_PIN, -1); // RX enabled, TX disabled

// pins
const uint8_t REL1 = 3;
const uint8_t REL2 = 4;
const uint8_t REL3 = 5;
const uint8_t REL4 = 6;
const uint8_t REL5 = 7;
const uint8_t REL6 = 8;
const uint8_t LED_PIN = 13;

// comm / timeout
unsigned long lastPacketMillis = 0;
const unsigned long TIMEOUT_WARN_MS = 2000; // LED steady after this
const unsigned long TIMEOUT_LOSS_MS = 5000; // loss -> relays off
bool commActive = false;

// comm blink
unsigned long blinkUntil = 0;
const unsigned long BLINK_MS = 40;

// current state (applied)
uint8_t relayState = 0; // bit0..bit5 => R1..R6

// Flasher
bool flasherActive = false;
unsigned long flasherTimer = 0;
const unsigned long FL_STEP_MS = 100; // step interval
int flStep = 0; // step in pattern machine
int flSubCount = 0; // counts flashes within subpattern

// helper to apply relayState
void applyRelays(uint8_t state) {
  digitalWrite(REL1, (state & 0x01) ? HIGH : LOW);
  digitalWrite(REL2, (state & 0x02) ? HIGH : LOW);
  digitalWrite(REL3, (state & 0x04) ? HIGH : LOW);
  digitalWrite(REL4, (state & 0x08) ? HIGH : LOW);
  digitalWrite(REL5, (state & 0x10) ? HIGH : LOW);
  digitalWrite(REL6, (state & 0x20) ? HIGH : LOW);
  relayState = state;
}

void turnAllRelaysOff() {
  applyRelays(0);
}

// Flasher pattern: 4 flashes on R3, 4 flashes on R4, then 2 on R3, 2 on R4, repeat.
// Non-blocking state machine.
void runFlasher() {
  if (!flasherActive) return;
  unsigned long now = millis();
  if (now - flasherTimer < FL_STEP_MS) return;
  flasherTimer = now;

  // state sequence:
  // flStep 0 -> do 4 flashes on R3 (on/off)
  // flStep 1 -> do 4 flashes on R4
  // flStep 2 -> do 2 flashes on R3
  // flStep 3 -> do 2 flashes on R4
  // each flash consists of ON step then OFF step => need 8 steps for 4 flashes, 4 steps for 2 flashes
  int stepsNeeded = 8;
  if (flStep == 2 || flStep == 3) stepsNeeded = 4;

  // compute if this is ON or OFF substep (even index = ON)
  bool onPhase = (flSubCount % 2 == 0);

  uint8_t s = relayState;
  // Ensure R5 & others remain as in relayState; only R3/R4 are toggled for flasher.
  if (flStep == 0) {
    // R3 flashes
    if (onPhase) s |= 0x04; else s &= ~0x04;
    s &= ~0x08; // ensure R4 off during R3 flashes
  } else if (flStep == 1) {
    if (onPhase) s |= 0x08; else s &= ~0x08;
    s &= ~0x04;
  } else if (flStep == 2) {
    if (onPhase) s |= 0x04; else s &= ~0x04;
    s &= ~0x08;
  } else if (flStep == 3) {
    if (onPhase) s |= 0x08; else s &= ~0x08;
    s &= ~0x04;
  }

  // Apply only R3/R4 changes (preserve other relays)
  // Build apply state: keep R1,R2,R5,R6 from relayState and set R3/R4 from s
  uint8_t applyState = (relayState & (0x01 | 0x02 | 0x10 | 0x20)) | (s & (0x04 | 0x08));
  applyRelays(applyState);

  flSubCount++;
  if (flSubCount >= stepsNeeded) {
    flSubCount = 0;
    flStep++;
    if (flStep > 3) flStep = 0;
  }
}

void setup() {
  pinMode(REL1, OUTPUT);
  pinMode(REL2, OUTPUT);
  pinMode(REL3, OUTPUT);
  pinMode(REL4, OUTPUT);
  pinMode(REL5, OUTPUT);
  pinMode(REL6, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  turnAllRelaysOff();
  digitalWrite(LED_PIN, LOW);

  link.begin(9600);
  lastPacketMillis = millis();
}

void loop() {
  // process incoming packets
  if (link.available()) {
    char c = link.read();
    lastPacketMillis = millis();
    commActive = true;

    // short blink on LED
    digitalWrite(LED_PIN, HIGH);
    blinkUntil = millis() + BLINK_MS;

    if (c == 'H') {
      // heartbeat - nothing to do
    } else if (c == 'F') {
      // toggle flasher
      flasherActive = !flasherActive;
      if (!flasherActive) {
        // restore relayState R3/R4 as per stored relayState (off or on as in state byte)
        applyRelays(relayState);
      } else {
        // init flasher counters
        flStep = 0;
        flSubCount = 0;
        flasherTimer = millis();
      }
    } else if (c == 'S') {
      // expect one more byte for state
      // wait until available - but non-blocking: read if available else ignore
      while (link.available() == 0) {
        // short timeout wait (avoid blocking) - break if too long
        if (millis() - lastPacketMillis > 50) break;
      }
      if (link.available()) {
        int st = link.read();
        if (st >= 0) {
          uint8_t newState = (uint8_t)st & 0x3F;
          // apply state exactly
          applyRelays(newState);
        }
      }
    } else {
      // unknown command - ignore
    }
  }

  // end short blink
  if (blinkUntil && millis() > blinkUntil) {
    digitalWrite(LED_PIN, LOW);
    blinkUntil = 0;
  }

  // timeout handling: warn and loss
  unsigned long since = millis() - lastPacketMillis;
  if (since > TIMEOUT_LOSS_MS) {
    // comm lost -> LED OFF and relays off
    digitalWrite(LED_PIN, LOW);
    turnAllRelaysOff();
    commActive = false;
    flasherActive = false;
  } else if (since > TIMEOUT_WARN_MS) {
    // warn: steady LED on
    digitalWrite(LED_PIN, HIGH);
  }

  // run flasher if active
  runFlasher();
}
