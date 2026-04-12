// https://funduino.de/nr-32-der-rotary-encoder-ky-040
// zwischen grau (sleeve) und rot (tip) messe ich 15kOhm (oder 16kOhm) in eine Richtung und 24kOhm in die andere Richung beim Drehen des Encoders 
// 3.5kOhm für Mute
// https://forum.arduino.cc/t/steering-wheel-remote-audio-control/223878
// https://github.com/bigevtaylor/arduino-swc
// MCP 4151-104E (100kOhm) wird verwendet (Pin1->CS, Pin2->CLK, Pin3->SDO, Pin4->GND, Pin5->Pin 7(B)->GND, Pin 8->+5V)
// https://github.com/PaulusElektrus/Arduino_and_MCP4151/blob/master/MCP4151-U-Messung_Steckplatine.png (aber stimmt nicht ganz mit der Messung)

// ============================================================
// Pioneer SPH-DA77DAB - Drehencoder (Vol+/Vol-) + Taster (Mute)
// MCP4151-104 (100kΩ, SPI) + KY-040 Encoder
// Arduino Nano — PlatformIO
// ============================================================
//
// VERDRAHTUNG:
//   MCP4151:
//     Pin 1 (CS)   -> D9
//     Pin 2 (SCK)  -> D13
//     Pin 3 (SDI)  -> D11
//     Pin 4 (VSS)  -> GND
//     Pin 5 (PA0)  -> Pioneer SWC REM+  (weiss/braun)
//     Pin 6 (PW0)  -> Pioneer SWC REM+  (PA0 + PW0 brücken!)
//     Pin 7 (PB0)  -> GND
//     Pin 8 (VDD)  -> 5V
//
//   KY-040:
//     CLK -> D2   (Interrupt)
//     DT  -> D3
//     SW  -> D4   (Mute-Taster)
//     +   -> 5V
//     GND -> GND
//
//   Pioneer SWC-Kabel:
//     REM+  (weiss/braun) -> MCP4151 Pin 5+6
//     REM-  (schwarz)     -> GND
//
//   diesen MCP4151 unterstützt nicht open source output 04.04.2026
// ============================================================

#include <Arduino.h>
#include <SPI.h>
//#include <MCP4151.h>

// Default built-in LED is on digital pin 13
//int ledPin = 13;  // LED pin
//int ledPin = 5;  // LED pin /nur zum testen da 13 von SPI clock verwendet wird

constexpr uint8_t LED_PIN   =  5;   // LED pin
constexpr uint8_t ENCODER_A =  2;   // Interrupt-fähig
constexpr uint8_t ENCODER_B =  3;
constexpr uint8_t SW_PIN    =  4;   // SW = Mute
constexpr uint8_t CS_PIN    =  9;
//const int clkPin = 2;    // Rotary encoder CLK (A)
//const int dtPin  = 3;    // Rotary encoder DT (B)
//const int swPin  = 4;    // Rotary encoder button (SW) (10kOhm <-> VCC, 100nF <-> GND)
//const int csPin  = 9;    // Chip select for MCP4151

// Variables
volatile int blinkDelay = 500;          // ms
const int defaultDelay = 500;
const int minDelay = 100;
const int maxDelay = 2000;
const int stepMs     = 50;              // change per detent

// ---- Timing --------------------------------------------------
constexpr uint16_t PULSE_MS      = 50; //150;  // Befehlsdauer ms
constexpr uint8_t  ENC_DEBOUNCE  =   5;  // Encoder ms
constexpr uint8_t  BTN_DEBOUNCE  =  50;  // Taster ms

// ---- Globale Variablen ---------------------------------------
volatile int8_t   encoderDelta   = 0;
volatile uint8_t  lastA          = HIGH;
volatile uint32_t lastEncTime    = 0;

bool     lastBtnState = HIGH;
bool     stableBtnState = HIGH;
uint32_t lastBtnTime  = 0;
bool     btnHandled    = false;  // verhindert Mehrfachauslösung

// --- MCP4151 Commands ---
constexpr uint8_t MCP4151_WRITE = 0x00;
constexpr uint8_t MCP4151_SHUTDOWN = 0x08;  // Wiper trennen (Hi-Z)
#define CMD_WRITE 0x00    // Write Data
#define ADDR_W0   0x00    // Volatile Wiper 0
#define ADDR_TCON 0x04    // TCON Register
const int wiper0writeAddr = B00000000;
const int wiper1writeAddr = B00010000;
const int tconwriteAddr = B01000000;
          
//const int tconwriteAddr = B00000100;
const int tcon_0off_1off = B00000000;
const int tcon_0on_1on = B11111111;

// Set the wiper addresses for the digipot
int wiperTip = B00000000;
int wiperRing = B00010000;
// Set the digitpot resistor value for no button
int ground = 0;

const byte ADDRESS_WIPER0 = 0x0;
        const byte ADDRESS_WIPER1 = 0x1;
        const byte COMMAND_MASK = 0x00;
    
        const byte COMMAND_WRITE = 0x0;
        const byte COMMAND_READ = 0x3;
        const byte COMMAND_INCREMENT = 0x1;
        const byte COMMAND_DECREMENT = 0x2;

const byte MUTE = 9;      // 3.5kOhm
const byte VOL_UP = 41;   // 16kOhm
const byte VOL_DOWN = 61; // 24kOhm

constexpr uint8_t POS_IDLE   = 255;  //181;  // ~71 kΩ  Ruhezustand
constexpr uint8_t POS_VOL_UP =  41;  // ~16 kΩ  Vol+
constexpr uint8_t POS_VOL_DOWN =  61;  // ~24 kΩ  Vol-
constexpr uint8_t POS_MUTE   =   9;  // ~ 3.5kΩ Mute


// non-blocking blink
unsigned long tPrev = 0;
bool ledState = LOW;

// for ISR debounce
volatile unsigned long lastEdgeMs = 0;

// Button debounce
unsigned long lastButtonPress = 0;
const unsigned long debounceTime = 200;  // ms

bool released, pressed = false;

//MCP4151 pot(CS, MOSI, MISO, SCK);
//MCP4151 pot(CS, MOSI, MISO, SCK, 4000000, 250000, SPI_MODE0);
//MCP4151 pot(10, MOSI, MISO, 9, 4000000, 250000, SPI_MODE0);

// ---- Prototypen ----------------------------------------------
void sendCommand(uint8_t pos);
void setIdle();
void setWiper(uint8_t pos);
void encoderISR();

void setup() {
  delay(1000); // some microcontrollers reboot twice

  Serial.begin(115200);
  Serial.println(F("setup(): begin"));

  // Pull-ups keep lines HIGH; encoder pulls them LOW when moving
  //pinMode(ledPin, OUTPUT);
  //pinMode(clkPin, INPUT_PULLUP);
  //pinMode(dtPin,  INPUT_PULLUP);
  //pinMode(swPin,  INPUT_PULLUP);  // button to GND

  // MCP4151 via SPI
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16); // ~1 MHz

  // Encoder
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

  // Mute-Taster
  pinMode(SW_PIN, INPUT_PULLUP);  // button to GND

  // Ruhezustand setzen
  //setWiper(POS_IDLE);
  setIdle();   // statt setWiper(POS_IDLE)

  delay(10);

  Serial.println("Pioneer SWC bereit. Vol+/Vol-/Mute aktiv.");
    
  Serial.println(F("setup(): ready"));
  delay(2000);

  //Serial.println("Rotary Encoder LED Blink with Button Reset");

  /* Serial.println("Widerstand auf 24k setzen -> VOL-");
  sendCommand(POS_VOL_DOWN); // 24kOhm
  Serial.println("Widerstand auf 16k setzen -> VOL+");
  sendCommand(POS_VOL_UP); // 16kOhm
  Serial.println("Widerstand auf 16k setzen -> VOL+");
  sendCommand(POS_VOL_UP); // 16kOhm */
 
}


void loop() {

  /*   // ---- TEMPORÄR: Encoder-Rohwerte direkt ausgeben ------------
  static uint8_t lastRawA = HIGH;
  static uint8_t lastRawB = HIGH;
  
  uint8_t rawA = digitalRead(ENCODER_A);
  uint8_t rawB = digitalRead(ENCODER_B);
  
  if (rawA != lastRawA || rawB != lastRawB) {
    Serial.print(F("A="));
    Serial.print(rawA);
    Serial.print(F(" B="));
    Serial.println(rawB);
    lastRawA = rawA;
    lastRawB = rawB;
  } */

  // ---- Encoder -----------------------------------------------
  int8_t delta = 0;
  noInterrupts();
    delta = encoderDelta;
    encoderDelta = 0;
  interrupts();

  if (delta > 0) {
    Serial.println(F("VOL+"));
    sendCommand(POS_VOL_UP);
  } else if (delta < 0) {
    Serial.println(F("VOL-"));
    sendCommand(POS_VOL_DOWN);
  }

  // ---- Mute-Taster (saubere Flankenauswertung) ---------------
  bool rawBtn = digitalRead(SW_PIN);

   // Änderung erkannt → Timer neu starten
  if (rawBtn != lastBtnState) {
    lastBtnTime = millis();
    btnHandled  = false;
  }
  lastBtnState = rawBtn;

  // Erst nach Debounce-Zeit auswerten
  if ((millis() - lastBtnTime) > BTN_DEBOUNCE) {
    // Taster gedrückt (LOW) und noch nicht ausgeführt
    if (rawBtn == LOW && !btnHandled) {
      Serial.println(F("MUTE"));
      sendCommand(POS_MUTE);
      btnHandled = true;  // nicht nochmals auslösen bis losgelassen
    }
  }

  /* unsigned long now = millis();
  
  // show switch bounce:
  if (digitalRead(swPin) == HIGH) {
      released = true;
      pressed = false;
      //Serial.print(".");
  }
  if ((digitalRead(swPin) == LOW) && (released == true)) {
      released = false;
      Serial.print(".");
  }
  
  // Non-blocking blink
  if (now - tPrev >= (unsigned long)blinkDelay) {
    tPrev = now;
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
  }

  // (Optional) print status occasionally
  static unsigned long tPrint = 0;
  if (now - tPrint > 300) {
    tPrint = now;
    Serial.print("Delay: ");
    Serial.print(blinkDelay);
    Serial.print(" ms  |  Freq: ");
    Serial.print(1000.0 / (2.0 * blinkDelay), 2); // on+off = 2*delay
    Serial.println(" Hz");
  } */
}


// ==============================================================
// Encoder-ISR (auf ENCODER_A, entprellt)
void encoderISR() {
  uint32_t now = millis();
  if ((now - lastEncTime) < ENC_DEBOUNCE) return;
  lastEncTime = now;

  uint8_t a = digitalRead(ENCODER_A);
  uint8_t b = digitalRead(ENCODER_B);

  if (a != lastA) {
    encoderDelta += (b != a) ? 1 : -1;
    lastA = a;
  }
}

// ==============================================================
// Befehl senden: kurzer Impuls, dann Ausgang offen
void sendCommand(uint8_t pos) {
  setWiper(pos);
  delay(PULSE_MS);
  setWiper(POS_IDLE);
  //setIdle();   // statt setWiper(POS_IDLE)
  delay(10);
}

// ==============================================================
// MCP4151 in Shutdown versetzen → Ausgang hochohmig (offen)
void setIdle() {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(MCP4151_SHUTDOWN);
  SPI.transfer(0x00);  // Datenbyte wird ignoriert, trotzdem senden
  digitalWrite(CS_PIN, HIGH);
}

// ============================================================
// MCP4151: Wiper-Position setzen (0–255)
void setWiper(uint8_t pos) {
  digitalWrite(CS_PIN, LOW); // active
  SPI.transfer(MCP4151_WRITE);
  SPI.transfer(pos);
  digitalWrite(CS_PIN, HIGH); // inactive
}
