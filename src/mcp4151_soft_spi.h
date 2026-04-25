#ifndef MCP4151_SOFT_SPI_H
#define MCP4151_SOFT_SPI_H

#include <Arduino.h>

// ============================================================
// MCP4151 Software-SPI Klasse
// Keine Hardware-SPI, daher bleibt Pin 13 (LED_BUILTIN) frei
// ============================================================

class MCP4151_SoftSPI {
public:
    /**
     * Konstruktor
     * @param csPin   Chip Select Pin
     * @param sckPin  SPI Clock Pin
     * @param dataPin Serial Data In/Out
     */
    MCP4151_SoftSPI(uint8_t csPin, uint8_t sckPin, uint8_t dataPin);
    
    /**
     * Initialisiert die Pins für Software-SPI
     * Muss in setup() aufgerufen werden
     */
    void begin();
    
    /**
     * Schreibt einen Wert in den MCP4151
     * @param value Wert zwischen 0 und 257
     */
    void writeValue(uint16_t value);
    
    /**
     * Schreibt einen Byte-Wert direkt (0-255)
     * @param data Byte-Wert
     */
    void writeByte(uint8_t data);
    
    /**
     * Setzt den Wiper auf den Minimalwert (0)
     */
    void setMin();
    
    /**
     * Setzt den Wiper auf den Maximalwert (257)
     */
    void setMax();
    
    /**
     * Setzt den Wiper auf die Mitte (128)
     */
    void setMid();

private:
    uint8_t _csPin;
    uint8_t _sckPin;
    uint8_t _dataPin;
    
    // Interne Hilfsfunktionen
    void _transferByte(uint8_t data);
    void _sendCommand(uint8_t command, uint8_t data);
};

#endif // MCP4151_SOFT_SPI_H