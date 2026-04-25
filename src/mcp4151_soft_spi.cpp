#include "mcp4151_soft_spi.h"

// MCP4151 Kommandos laut Datenblatt
#define MCP_CMD_WRITE    0x00  // Write Command (C1=0, C0=0)
#define MCP_ADDR_WIPER   0x00  // Wiper 0 Adresse (A3=0, A2=0, A1=0, A0=0)
#define MCP_MAX_VALUE    255   // Maximaler Widerstandswert (8-Bit)

// Konstruktor
MCP4151_SoftSPI::MCP4151_SoftSPI(uint8_t csPin, uint8_t sckPin, uint8_t dataPin) {
    _csPin = csPin;
    _sckPin = sckPin;
    _dataPin = dataPin;
}

// Initialisierung
void MCP4151_SoftSPI::begin() {
    pinMode(_csPin, OUTPUT);
    pinMode(_sckPin, OUTPUT);
    pinMode(_dataPin, OUTPUT);
    
    // Inaktive Zustände
    digitalWrite(_csPin, HIGH);
    digitalWrite(_sckPin, LOW);
    digitalWrite(_dataPin, LOW);
    
    // Kleine Verzögerung zur Stabilisierung
    delay(10);
}

// Software-SPI Byte-Transfer (MSB first)
void MCP4151_SoftSPI::_transferByte(uint8_t data) {
    for (int i = 7; i >= 0; i--) {
        digitalWrite(_dataPin, (data >> i) & 1);
        digitalWrite(_sckPin, HIGH);
        delayMicroseconds(1);  // Kurze Pause für Stabilität
        digitalWrite(_sckPin, LOW);
        delayMicroseconds(1);
    }
}

// Sendet ein Kommando mit Daten an den MCP4151
void MCP4151_SoftSPI::_sendCommand(uint8_t command, uint8_t data) {
    digitalWrite(_csPin, LOW);
    _transferByte(command);
    _transferByte(data);
    digitalWrite(_csPin, HIGH);
    delayMicroseconds(5);  // Kurze Pause nach dem Schreiben
}

// Schreibt einen Wert (0-257)
void MCP4151_SoftSPI::writeValue(uint16_t value) {
    // Wertebereich begrenzen
    if (value > MCP_MAX_VALUE) {
        value = MCP_MAX_VALUE;
    }
    
    // Command Byte: [A3 A2 A1 A0 C1 C0 0 0]
    uint8_t commandByte = (MCP_ADDR_WIPER << 4) | (MCP_CMD_WRITE << 2);
    
    // Data Byte: Wert auf 8 Bit begrenzen (für 257 müsste man 9 Bit senden,
    // aber der MCP4151 akzeptiert 0-257 mit 8-Bit-Wert + 1 Spezialfall)
    uint8_t dataByte = (uint8_t)value;
    
    _sendCommand(commandByte, dataByte);
}

// Schreibt einen Byte-Wert (0-255)
void MCP4151_SoftSPI::writeByte(uint8_t data) {
    uint8_t commandByte = (MCP_ADDR_WIPER << 4) | (MCP_CMD_WRITE << 2);
    _sendCommand(commandByte, data);
}

// Setzt den Wiper auf Minimum
void MCP4151_SoftSPI::setMin() {
    writeValue(0);
}

// Setzt den Wiper auf Maximum
void MCP4151_SoftSPI::setMax() {
    writeValue(MCP_MAX_VALUE);
}

// Setzt den Wiper auf Mittelwert
void MCP4151_SoftSPI::setMid() {
    writeValue(128);
}