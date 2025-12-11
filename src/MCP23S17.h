/*
 * MCP23S17.h - Bibliothek für MCP23S17 SPI Port Expander
 * 
 * Diese Bibliothek ermöglicht die einfache Steuerung des MCP23S17
 * 16-Bit I/O Expanders über SPI.
 * 
 * Features:
 * - pinMode, digitalWrite, digitalRead Funktionen
 * - Interrupt-Unterstützung
 * - Pull-up Widerstände konfigurierbar
 * - Portweise Operationen (8 Pins gleichzeitig)
 * - Unterstützung für bis zu 8 Chips pro CS-Pin
 * - Optimiert für ESP32
 * 
 * Autor: Erstellt für Arduino IDE
 * Datum: 2025
 */

#ifndef MCP23S17_H
#define MCP23S17_H

#include <Arduino.h>
#include <SPI.h>

// MCP23S17 Register-Adressen (IOCON.BANK = 0)
#define MCP23S17_IODIRA   0x00  // I/O Richtung Port A (1=Eingang, 0=Ausgang)
#define MCP23S17_IODIRB   0x01  // I/O Richtung Port B
#define MCP23S17_IPOLA    0x02  // Eingangs-Polarität Port A (1=invertiert)
#define MCP23S17_IPOLB    0x03  // Eingangs-Polarität Port B
#define MCP23S17_GPINTENA 0x04  // Interrupt-on-Change Port A (1=aktiviert)
#define MCP23S17_GPINTENB 0x05  // Interrupt-on-Change Port B
#define MCP23S17_DEFVALA  0x06  // Standard-Vergleichswert Port A für Interrupt
#define MCP23S17_DEFVALB  0x07  // Standard-Vergleichswert Port B für Interrupt
#define MCP23S17_INTCONA  0x08  // Interrupt-Steuerung Port A (1=mit DEFVAL vergleichen)
#define MCP23S17_INTCONB  0x09  // Interrupt-Steuerung Port B
#define MCP23S17_IOCON    0x0A  // Konfigurationsregister
#define MCP23S17_GPPUA    0x0C  // Pull-up Widerstände Port A (1=aktiviert)
#define MCP23S17_GPPUB    0x0D  // Pull-up Widerstände Port B
#define MCP23S17_INTFA    0x0E  // Interrupt-Flag Port A (1=Interrupt ausgelöst)
#define MCP23S17_INTFB    0x0F  // Interrupt-Flag Port B
#define MCP23S17_INTCAPA  0x10  // Interrupt-Erfassung Port A (Wert beim Interrupt)
#define MCP23S17_INTCAPB  0x11  // Interrupt-Erfassung Port B
#define MCP23S17_GPIOA    0x12  // GPIO Port A (Lesen/Schreiben)
#define MCP23S17_GPIOB    0x13  // GPIO Port B
#define MCP23S17_OLATA    0x14  // Ausgangs-Latch Port A
#define MCP23S17_OLATB    0x15  // Ausgangs-Latch Port B

// IOCON Register Bits
#define IOCON_BANK   0x80  // Register-Bank-Auswahl (0=sequenziell)
#define IOCON_MIRROR 0x40  // Interrupt-Mirror (1=INTA und INTB verbunden)
#define IOCON_SEQOP  0x20  // Sequenzielle Operation (1=deaktiviert)
#define IOCON_DISSLW 0x10  // Slew Rate Control (1=deaktiviert)
#define IOCON_HAEN   0x08  // Hardware-Adressierung (1=aktiviert)
#define IOCON_ODR    0x04  // Interrupt als Open-Drain (1=aktiviert)
#define IOCON_INTPOL 0x02  // Interrupt-Polarität (1=Active-High)

// SPI Opcodes
#define MCP23S17_WRITE_CMD 0x40  // Schreib-Opcode
#define MCP23S17_READ_CMD  0x41  // Lese-Opcode

// Port-Definitionen
#define MCP23S17_PORTA 0  // Port A (GPIO 0-7)
#define MCP23S17_PORTB 1  // Port B (GPIO 8-15)

// Pin-Modi
#define MCP23S17_INPUT         0x00  // Eingang ohne Pull-up
#define MCP23S17_OUTPUT        0x01  // Ausgang
#define MCP23S17_INPUT_PULLUP  0x02  // Eingang mit Pull-up

// Interrupt-Modi
#define MCP23S17_INT_CHANGE    0x00  // Interrupt bei Änderung
#define MCP23S17_INT_COMPARE   0x01  // Interrupt bei Vergleich mit DEFVAL

class MCP23S17 {
public:
    /*
     * Konstruktor
     * csPin: Chip Select Pin
     * address: Hardware-Adresse (0-7) über A0, A1, A2 Pins
     * spiFrequency: SPI-Frequenz in Hz (Standard: 10 MHz)
     */
    MCP23S17(uint8_t csPin, uint8_t address = 0, uint32_t spiFrequency = 10000000);
    
    /*
     * Initialisierung
     * Muss in setup() aufgerufen werden
     * Gibt true zurück bei Erfolg
     */
    bool begin();
    
    // ========== GPIO Grundfunktionen ==========
    
    /*
     * Pin-Modus setzen
     * pin: 0-15 (0-7 = Port A, 8-15 = Port B)
     * mode: MCP23S17_INPUT, MCP23S17_OUTPUT, MCP23S17_INPUT_PULLUP
     */
    void pinMode(uint8_t pin, uint8_t mode);
    
    /*
     * Digitalen Wert schreiben
     * pin: 0-15
     * value: HIGH oder LOW
     */
    void digitalWrite(uint8_t pin, uint8_t value);
    
    /*
     * Digitalen Wert lesen
     * pin: 0-15
     * Gibt HIGH oder LOW zurück
     */
    uint8_t digitalRead(uint8_t pin);
    
    // ========== Portweise Operationen ==========
    
    /*
     * Port-Modus setzen (alle 8 Pins gleichzeitig)
     * port: MCP23S17_PORTA oder MCP23S17_PORTB
     * modes: 8-Bit Wert (1=Ausgang, 0=Eingang)
     */
    void portMode(uint8_t port, uint8_t modes);
    
    /*
     * Port schreiben (alle 8 Pins gleichzeitig)
     * port: MCP23S17_PORTA oder MCP23S17_PORTB
     * value: 8-Bit Wert
     */
    void writePort(uint8_t port, uint8_t value);
    
    /*
     * Port lesen (alle 8 Pins gleichzeitig)
     * port: MCP23S17_PORTA oder MCP23S17_PORTB
     * Gibt 8-Bit Wert zurück
     */
    uint8_t readPort(uint8_t port);
    
    /*
     * Alle 16 GPIOs schreiben
     * value: 16-Bit Wert (LOW-Byte = Port A, HIGH-Byte = Port B)
     */
    void writeGPIO(uint16_t value);
    
    /*
     * Alle 16 GPIOs lesen
     * Gibt 16-Bit Wert zurück
     */
    uint16_t readGPIO();
    
    // ========== Pull-up Konfiguration ==========
    
    /*
     * Pull-up Widerstand für einzelnen Pin
     * pin: 0-15
     * enable: true=aktiviert, false=deaktiviert
     */
    void setPullup(uint8_t pin, bool enable);
    
    /*
     * Pull-up Widerstände für ganzen Port
     * port: MCP23S17_PORTA oder MCP23S17_PORTB
     * value: 8-Bit Wert (1=aktiviert)
     */
    void setPortPullups(uint8_t port, uint8_t value);
    
    // ========== Interrupt-Funktionen ==========
    
    /*
     * Interrupt für einzelnen Pin aktivieren
     * pin: 0-15
     * mode: MCP23S17_INT_CHANGE oder MCP23S17_INT_COMPARE
     * defaultValue: Vergleichswert bei MCP23S17_INT_COMPARE
     */
    void enableInterrupt(uint8_t pin, uint8_t mode = MCP23S17_INT_CHANGE, uint8_t defaultValue = 0);
    
    /*
     * Interrupt für einzelnen Pin deaktivieren
     * pin: 0-15
     */
    void disableInterrupt(uint8_t pin);
    
    /*
     * Interrupt für ganzen Port aktivieren
     * port: MCP23S17_PORTA oder MCP23S17_PORTB
     * mask: 8-Bit Wert (1=Interrupt aktiviert)
     * mode: MCP23S17_INT_CHANGE oder MCP23S17_INT_COMPARE
     */
    void setPortInterrupts(uint8_t port, uint8_t mask, uint8_t mode = MCP23S17_INT_CHANGE);
    
    /*
     * Interrupt-Polarität setzen
     * activeHigh: true=Active-High, false=Active-Low
     */
    void setInterruptPolarity(bool activeHigh);
    
    /*
     * Interrupt-Mirror aktivieren (INTA und INTB verbunden)
     * enable: true=aktiviert, false=deaktiviert
     */
    void setInterruptMirror(bool enable);
    
    /*
     * Interrupt-Flags lesen
     * port: MCP23S17_PORTA oder MCP23S17_PORTB
     * Gibt 8-Bit Wert zurück (1=Interrupt aufgetreten)
     */
    uint8_t getInterruptFlags(uint8_t port);
    
    /*
     * Interrupt-Capture-Werte lesen (GPIO-Zustand beim Interrupt)
     * port: MCP23S17_PORTA oder MCP23S17_PORTB
     * Gibt 8-Bit Wert zurück
     */
    uint8_t getInterruptCapture(uint8_t port);
    
    /*
     * Alle Interrupts löschen (durch Lesen der Capture-Register)
     */
    void clearInterrupts();
    
    // ========== Erweiterte Funktionen ==========
    
    /*
     * Eingangs-Polarität setzen (invertiert Eingangswert)
     * pin: 0-15
     * invert: true=invertiert, false=normal
     */
    void setInputPolarity(uint8_t pin, bool invert);
    
    /*
     * Register direkt lesen (für fortgeschrittene Anwendungen)
     * reg: Register-Adresse
     */
    uint8_t readRegister(uint8_t reg);
    
    /*
     * Register direkt schreiben (für fortgeschrittene Anwendungen)
     * reg: Register-Adresse
     * value: Zu schreibender Wert
     */
    void writeRegister(uint8_t reg, uint8_t value);
    
    /*
     * Chip komplett zurücksetzen (alle Register auf Standard)
     */
    void reset();

private:
    uint8_t _csPin;           // Chip Select Pin
    uint8_t _address;         // Hardware-Adresse (0-7)
    uint32_t _spiFrequency;   // SPI-Frequenz
    SPIClass* _spi;           // SPI-Instanz
    
    // Lokale Register-Kopien für schnellere Bit-Operationen
    uint8_t _directionA;      // IODIRA Cache
    uint8_t _directionB;      // IODIRB Cache
    uint8_t _pullupA;         // GPPUA Cache
    uint8_t _pullupB;         // GPPUB Cache
    uint8_t _outputA;         // OLATA Cache
    uint8_t _outputB;         // OLATB Cache
    
    /*
     * SPI-Transaktion starten
     */
    void beginTransaction();
    
    /*
     * SPI-Transaktion beenden
     */
    void endTransaction();
    
    /*
     * Einzelnes Bit in Register setzen oder löschen
     * reg: Register-Adresse
     * bit: Bit-Position (0-7)
     * value: true=setzen, false=löschen
     */
    void updateRegisterBit(uint8_t reg, uint8_t bit, bool value);
};

#endif // MCP23S17_H
