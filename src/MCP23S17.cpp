/*
 * MCP23S17.cpp - Implementierung für MCP23S17 SPI Port Expander
 */

#include "MCP23S17.h"

/*
 * Konstruktor - Initialisiert die Bibliothek
 */
MCP23S17::MCP23S17(uint8_t csPin, uint8_t address, uint32_t spiFrequency) {
    _csPin = csPin;
    _address = address & 0x07;  // Nur 3 Bits für Adresse (0-7)
    _spiFrequency = spiFrequency;
    _spi = &SPI;
    
    // Register-Caches initialisieren
    _directionA = 0xFF;  // Alle Pins als Eingang (Standard)
    _directionB = 0xFF;
    _pullupA = 0x00;     // Keine Pull-ups (Standard)
    _pullupB = 0x00;
    _outputA = 0x00;     // Alle Ausgänge LOW
    _outputB = 0x00;
}

/*
 * Initialisierung mit Standard-SPI-Pins
 */
bool MCP23S17::begin() {
    // CS-Pin als Ausgang konfigurieren
    ::pinMode(_csPin, OUTPUT);
    ::digitalWrite(_csPin, HIGH);  // CS inaktiv (HIGH)
    
    // SPI mit Standard-Pins initialisieren
    _spi->begin();
    
    // Warte kurz nach SPI-Init
    delay(1);
    
    // IOCON Register konfigurieren und Chip initialisieren
    return initializeChip();
}

/*
 * Initialisierung mit benutzerdefinierten SPI-Pins
 */
bool MCP23S17::begin(int8_t sck, int8_t miso, int8_t mosi) {
    // CS-Pin als Ausgang konfigurieren
    ::pinMode(_csPin, OUTPUT);
    ::digitalWrite(_csPin, HIGH);  // CS inaktiv (HIGH)
    
    // SPI mit benutzerdefinierten Pins initialisieren
    _spi->begin(sck, miso, mosi, _csPin);
    
    // Warte kurz nach SPI-Init
    delay(1);
    
    // IOCON Register konfigurieren und Chip initialisieren
    return initializeChip();
}

/*
 * Initialisierung mit bestehendem SPI-Bus
 */
bool MCP23S17::begin(SPIClass* spi) {
    // Übergebenen SPI-Bus verwenden
    _spi = spi;
    
    // CS-Pin als Ausgang konfigurieren
    ::pinMode(_csPin, OUTPUT);
    ::digitalWrite(_csPin, HIGH);  // CS inaktiv (HIGH)
    
    // SPI wird NICHT initialisiert - muss bereits initialisiert sein!
    // Der Benutzer hat die volle Kontrolle über den SPI-Bus
    
    // Warte kurz
    delay(1);
    
    // IOCON Register konfigurieren und Chip initialisieren
    return initializeChip();
}

// ========== GPIO Grundfunktionen ==========

/*
 * Pin-Modus setzen
 */
void MCP23S17::pinMode(uint8_t pin, uint8_t mode) {
    if (pin > 15) return;  // Ungültiger Pin
    
    // Port und Bit bestimmen
    uint8_t port = (pin < 8) ? MCP23S17_PORTA : MCP23S17_PORTB;
    uint8_t bit = pin % 8;
    
    // Je nach Modus Register aktualisieren
    if (mode == MCP23S17_OUTPUT) {
        // Ausgang: Direction-Bit löschen
        updateRegisterBit((port == MCP23S17_PORTA) ? MCP23S17_IODIRA : MCP23S17_IODIRB, 
                         bit, false);
        
        // Cache aktualisieren
        if (port == MCP23S17_PORTA) {
            _directionA &= ~(1 << bit);
        } else {
            _directionB &= ~(1 << bit);
        }
        
    } else {
        // Eingang: Direction-Bit setzen
        updateRegisterBit((port == MCP23S17_PORTA) ? MCP23S17_IODIRA : MCP23S17_IODIRB, 
                         bit, true);
        
        // Cache aktualisieren
        if (port == MCP23S17_PORTA) {
            _directionA |= (1 << bit);
        } else {
            _directionB |= (1 << bit);
        }
        
        // Pull-up aktivieren wenn INPUT_PULLUP
        if (mode == MCP23S17_INPUT_PULLUP) {
            setPullup(pin, true);
        } else {
            setPullup(pin, false);
        }
    }
}

/*
 * Digitalen Wert schreiben
 */
void MCP23S17::digitalWrite(uint8_t pin, uint8_t value) {
    if (pin > 15) return;  // Ungültiger Pin
    
    // Port und Bit bestimmen
    uint8_t port = (pin < 8) ? MCP23S17_PORTA : MCP23S17_PORTB;
    uint8_t bit = pin % 8;
    uint8_t reg = (port == MCP23S17_PORTA) ? MCP23S17_OLATA : MCP23S17_OLATB;
    
    // Output-Cache aktualisieren
    if (port == MCP23S17_PORTA) {
        if (value) {
            _outputA |= (1 << bit);
        } else {
            _outputA &= ~(1 << bit);
        }
        writeRegister(reg, _outputA);
    } else {
        if (value) {
            _outputB |= (1 << bit);
        } else {
            _outputB &= ~(1 << bit);
        }
        writeRegister(reg, _outputB);
    }
}

/*
 * Digitalen Wert lesen
 */
uint8_t MCP23S17::digitalRead(uint8_t pin) {
    if (pin > 15) return LOW;  // Ungültiger Pin
    
    // Port und Bit bestimmen
    uint8_t port = (pin < 8) ? MCP23S17_PORTA : MCP23S17_PORTB;
    uint8_t bit = pin % 8;
    uint8_t reg = (port == MCP23S17_PORTA) ? MCP23S17_GPIOA : MCP23S17_GPIOB;
    
    // Port lesen
    uint8_t value = readRegister(reg);
    
    // Bit extrahieren
    return (value & (1 << bit)) ? HIGH : LOW;
}

// ========== Portweise Operationen ==========

/*
 * Port-Modus setzen (alle 8 Pins gleichzeitig)
 */
void MCP23S17::portMode(uint8_t port, uint8_t modes) {
    if (port > 1) return;  // Nur Port A oder B
    
    uint8_t reg = (port == MCP23S17_PORTA) ? MCP23S17_IODIRA : MCP23S17_IODIRB;
    
    // Invertieren: modes hat 1=Ausgang, Register braucht 1=Eingang
    uint8_t direction = ~modes;
    
    writeRegister(reg, direction);
    
    // Cache aktualisieren
    if (port == MCP23S17_PORTA) {
        _directionA = direction;
    } else {
        _directionB = direction;
    }
}

/*
 * Port schreiben (alle 8 Pins gleichzeitig)
 */
void MCP23S17::writePort(uint8_t port, uint8_t value) {
    if (port > 1) return;  // Nur Port A oder B
    
    uint8_t reg = (port == MCP23S17_PORTA) ? MCP23S17_OLATA : MCP23S17_OLATB;
    
    writeRegister(reg, value);
    
    // Cache aktualisieren
    if (port == MCP23S17_PORTA) {
        _outputA = value;
    } else {
        _outputB = value;
    }
}

/*
 * Port lesen (alle 8 Pins gleichzeitig)
 */
uint8_t MCP23S17::readPort(uint8_t port) {
    if (port > 1) return 0;  // Nur Port A oder B
    
    uint8_t reg = (port == MCP23S17_PORTA) ? MCP23S17_GPIOA : MCP23S17_GPIOB;
    
    return readRegister(reg);
}

/*
 * Alle 16 GPIOs schreiben
 */
void MCP23S17::writeGPIO(uint16_t value) {
    // Low-Byte = Port A, High-Byte = Port B
    writePort(MCP23S17_PORTA, value & 0xFF);
    writePort(MCP23S17_PORTB, (value >> 8) & 0xFF);
}

/*
 * Alle 16 GPIOs lesen
 */
uint16_t MCP23S17::readGPIO() {
    uint8_t portA = readPort(MCP23S17_PORTA);
    uint8_t portB = readPort(MCP23S17_PORTB);
    
    return (uint16_t)portA | ((uint16_t)portB << 8);
}

// ========== Pull-up Konfiguration ==========

/*
 * Pull-up Widerstand für einzelnen Pin
 */
void MCP23S17::setPullup(uint8_t pin, bool enable) {
    if (pin > 15) return;  // Ungültiger Pin
    
    // Port und Bit bestimmen
    uint8_t port = (pin < 8) ? MCP23S17_PORTA : MCP23S17_PORTB;
    uint8_t bit = pin % 8;
    uint8_t reg = (port == MCP23S17_PORTA) ? MCP23S17_GPPUA : MCP23S17_GPPUB;
    
    // Bit setzen oder löschen
    updateRegisterBit(reg, bit, enable);
    
    // Cache aktualisieren
    if (port == MCP23S17_PORTA) {
        if (enable) {
            _pullupA |= (1 << bit);
        } else {
            _pullupA &= ~(1 << bit);
        }
    } else {
        if (enable) {
            _pullupB |= (1 << bit);
        } else {
            _pullupB &= ~(1 << bit);
        }
    }
}

/*
 * Pull-up Widerstände für ganzen Port
 */
void MCP23S17::setPortPullups(uint8_t port, uint8_t value) {
    if (port > 1) return;  // Nur Port A oder B
    
    uint8_t reg = (port == MCP23S17_PORTA) ? MCP23S17_GPPUA : MCP23S17_GPPUB;
    
    writeRegister(reg, value);
    
    // Cache aktualisieren
    if (port == MCP23S17_PORTA) {
        _pullupA = value;
    } else {
        _pullupB = value;
    }
}

// ========== Interrupt-Funktionen ==========

/*
 * Interrupt für einzelnen Pin aktivieren
 */
void MCP23S17::enableInterrupt(uint8_t pin, uint8_t mode, uint8_t defaultValue) {
    if (pin > 15) return;  // Ungültiger Pin
    
    // Port und Bit bestimmen
    uint8_t port = (pin < 8) ? MCP23S17_PORTA : MCP23S17_PORTB;
    uint8_t bit = pin % 8;
    
    // Interrupt aktivieren
    uint8_t intEnReg = (port == MCP23S17_PORTA) ? MCP23S17_GPINTENA : MCP23S17_GPINTENB;
    updateRegisterBit(intEnReg, bit, true);
    
    // Interrupt-Modus setzen
    uint8_t intConReg = (port == MCP23S17_PORTA) ? MCP23S17_INTCONA : MCP23S17_INTCONB;
    updateRegisterBit(intConReg, bit, mode == MCP23S17_INT_COMPARE);
    
    // Default-Wert setzen (nur bei COMPARE-Modus relevant)
    if (mode == MCP23S17_INT_COMPARE) {
        uint8_t defValReg = (port == MCP23S17_PORTA) ? MCP23S17_DEFVALA : MCP23S17_DEFVALB;
        updateRegisterBit(defValReg, bit, defaultValue != 0);
    }
}

/*
 * Interrupt für einzelnen Pin deaktivieren
 */
void MCP23S17::disableInterrupt(uint8_t pin) {
    if (pin > 15) return;  // Ungültiger Pin
    
    // Port und Bit bestimmen
    uint8_t port = (pin < 8) ? MCP23S17_PORTA : MCP23S17_PORTB;
    uint8_t bit = pin % 8;
    
    // Interrupt deaktivieren
    uint8_t intEnReg = (port == MCP23S17_PORTA) ? MCP23S17_GPINTENA : MCP23S17_GPINTENB;
    updateRegisterBit(intEnReg, bit, false);
}

/*
 * Interrupt für ganzen Port aktivieren
 */
void MCP23S17::setPortInterrupts(uint8_t port, uint8_t mask, uint8_t mode) {
    if (port > 1) return;  // Nur Port A oder B
    
    // Interrupt-Enable setzen
    uint8_t intEnReg = (port == MCP23S17_PORTA) ? MCP23S17_GPINTENA : MCP23S17_GPINTENB;
    writeRegister(intEnReg, mask);
    
    // Interrupt-Modus setzen (0=Change, 1=Compare)
    uint8_t intConReg = (port == MCP23S17_PORTA) ? MCP23S17_INTCONA : MCP23S17_INTCONB;
    if (mode == MCP23S17_INT_COMPARE) {
        writeRegister(intConReg, mask);
    } else {
        writeRegister(intConReg, 0x00);
    }
}

/*
 * Interrupt-Polarität setzen
 */
void MCP23S17::setInterruptPolarity(bool activeHigh) {
    uint8_t iocon = readRegister(MCP23S17_IOCON);
    
    if (activeHigh) {
        iocon |= IOCON_INTPOL;
    } else {
        iocon &= ~IOCON_INTPOL;
    }
    
    writeRegister(MCP23S17_IOCON, iocon);
}

/*
 * Interrupt-Mirror aktivieren
 */
void MCP23S17::setInterruptMirror(bool enable) {
    uint8_t iocon = readRegister(MCP23S17_IOCON);
    
    if (enable) {
        iocon |= IOCON_MIRROR;
    } else {
        iocon &= ~IOCON_MIRROR;
    }
    
    writeRegister(MCP23S17_IOCON, iocon);
}

/*
 * Interrupt-Flags lesen
 */
uint8_t MCP23S17::getInterruptFlags(uint8_t port) {
    if (port > 1) return 0;  // Nur Port A oder B
    
    uint8_t reg = (port == MCP23S17_PORTA) ? MCP23S17_INTFA : MCP23S17_INTFB;
    
    return readRegister(reg);
}

/*
 * Interrupt-Capture-Werte lesen
 */
uint8_t MCP23S17::getInterruptCapture(uint8_t port) {
    if (port > 1) return 0;  // Nur Port A oder B
    
    uint8_t reg = (port == MCP23S17_PORTA) ? MCP23S17_INTCAPA : MCP23S17_INTCAPB;
    
    return readRegister(reg);
}

/*
 * Alle Interrupts löschen
 */
void MCP23S17::clearInterrupts() {
    // Durch Lesen der Capture-Register werden Interrupts gelöscht
    readRegister(MCP23S17_INTCAPA);
    readRegister(MCP23S17_INTCAPB);
}

// ========== Erweiterte Funktionen ==========

/*
 * Eingangs-Polarität setzen
 */
void MCP23S17::setInputPolarity(uint8_t pin, bool invert) {
    if (pin > 15) return;  // Ungültiger Pin
    
    // Port und Bit bestimmen
    uint8_t port = (pin < 8) ? MCP23S17_PORTA : MCP23S17_PORTB;
    uint8_t bit = pin % 8;
    uint8_t reg = (port == MCP23S17_PORTA) ? MCP23S17_IPOLA : MCP23S17_IPOLB;
    
    updateRegisterBit(reg, bit, invert);
}

/*
 * Register direkt lesen
 */
uint8_t MCP23S17::readRegister(uint8_t reg) {
    beginTransaction();
    
    // Opcode senden: Lese-Befehl + Adresse
    _spi->transfer(MCP23S17_READ_CMD | (_address << 1));
    
    // Register-Adresse senden
    _spi->transfer(reg);
    
    // Daten lesen
    uint8_t value = _spi->transfer(0x00);
    
    endTransaction();
    
    return value;
}

/*
 * Register direkt schreiben
 */
void MCP23S17::writeRegister(uint8_t reg, uint8_t value) {
    beginTransaction();
    
    // Opcode senden: Schreib-Befehl + Adresse
    _spi->transfer(MCP23S17_WRITE_CMD | (_address << 1));
    
    // Register-Adresse senden
    _spi->transfer(reg);
    
    // Daten schreiben
    _spi->transfer(value);
    
    endTransaction();
}

/*
 * Chip komplett zurücksetzen
 */
void MCP23S17::reset() {
    // Alle Register auf Standardwerte setzen
    writeRegister(MCP23S17_IODIRA, 0xFF);
    writeRegister(MCP23S17_IODIRB, 0xFF);
    writeRegister(MCP23S17_IPOLA, 0x00);
    writeRegister(MCP23S17_IPOLB, 0x00);
    writeRegister(MCP23S17_GPINTENA, 0x00);
    writeRegister(MCP23S17_GPINTENB, 0x00);
    writeRegister(MCP23S17_DEFVALA, 0x00);
    writeRegister(MCP23S17_DEFVALB, 0x00);
    writeRegister(MCP23S17_INTCONA, 0x00);
    writeRegister(MCP23S17_INTCONB, 0x00);
    writeRegister(MCP23S17_IOCON, IOCON_HAEN);
    writeRegister(MCP23S17_GPPUA, 0x00);
    writeRegister(MCP23S17_GPPUB, 0x00);
    writeRegister(MCP23S17_OLATA, 0x00);
    writeRegister(MCP23S17_OLATB, 0x00);
    
    // Caches zurücksetzen
    _directionA = 0xFF;
    _directionB = 0xFF;
    _pullupA = 0x00;
    _pullupB = 0x00;
    _outputA = 0x00;
    _outputB = 0x00;
    
    // Interrupts löschen
    clearInterrupts();
}

// ========== Private Hilfsfunktionen ==========

/*
 * Chip initialisieren (wird von beiden begin()-Varianten aufgerufen)
 */
bool MCP23S17::initializeChip() {
    // IOCON Register konfigurieren:
    // - HAEN aktivieren (Hardware-Adressierung)
    // - SEQOP deaktivieren (sequenzieller Zugriff aktiviert)
    // - Alle anderen Bits auf 0
    writeRegister(MCP23S17_IOCON, IOCON_HAEN);
    
    // Alle Pins als Eingang konfigurieren (Standardzustand)
    writeRegister(MCP23S17_IODIRA, 0xFF);
    writeRegister(MCP23S17_IODIRB, 0xFF);
    
    // Alle Pull-ups deaktivieren
    writeRegister(MCP23S17_GPPUA, 0x00);
    writeRegister(MCP23S17_GPPUB, 0x00);
    
    // Alle Ausgänge auf LOW
    writeRegister(MCP23S17_OLATA, 0x00);
    writeRegister(MCP23S17_OLATB, 0x00);
    
    // Polarität normal (nicht invertiert)
    writeRegister(MCP23S17_IPOLA, 0x00);
    writeRegister(MCP23S17_IPOLB, 0x00);
    
    // Alle Interrupts deaktivieren
    writeRegister(MCP23S17_GPINTENA, 0x00);
    writeRegister(MCP23S17_GPINTENB, 0x00);
    
    // Kommunikation testen: IOCON lesen
    uint8_t testValue = readRegister(MCP23S17_IOCON);
    return (testValue & IOCON_HAEN) != 0;  // Prüfe ob HAEN gesetzt ist
}

/*
 * SPI-Transaktion starten
 */
void MCP23S17::beginTransaction() {
    _spi->beginTransaction(SPISettings(_spiFrequency, MSBFIRST, SPI_MODE0));
    ::digitalWrite(_csPin, LOW);  // CS aktivieren
}

/*
 * SPI-Transaktion beenden
 */
void MCP23S17::endTransaction() {
    ::digitalWrite(_csPin, HIGH);  // CS deaktivieren
    _spi->endTransaction();
}

/*
 * Einzelnes Bit in Register setzen oder löschen
 */
void MCP23S17::updateRegisterBit(uint8_t reg, uint8_t bit, bool value) {
    if (bit > 7) return;  // Ungültiges Bit
    
    // Register lesen
    uint8_t regValue = readRegister(reg);
    
    // Bit setzen oder löschen
    if (value) {
        regValue |= (1 << bit);
    } else {
        regValue &= ~(1 << bit);
    }
    
    // Register zurückschreiben
    writeRegister(reg, regValue);
}
