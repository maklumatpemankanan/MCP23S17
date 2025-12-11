# MCP23S17 Arduino Library

Complete Arduino library for the **MCP23S17** 16-bit SPI I/O Expander from Microchip.

## Features

- ✅ **GPIO Basic Functions**: pinMode, digitalWrite, digitalRead
- ✅ **Interrupt Support**: Complete interrupt configuration and handling
- ✅ **Pull-up Resistors**: Configurable for each pin
- ✅ **Port-wise Operations**: Control 8 pins simultaneously
- ✅ **16-bit GPIO**: Read/write all 16 pins at once
- ✅ **Multiple Chips**: Up to 8 MCP23S17 per CS pin
- ✅ **ESP32 Optimized**: Tested on ESP32
- ✅ **Clean Code**: Extensively commented

## Hardware Requirements

- ESP32 (or other Arduino-compatible boards)
- MCP23S17 Port Expander
- SPI connection

## Installation

### Method 1: Arduino IDE

1. Download the library as ZIP
2. Arduino IDE → Sketch → Include Library → Add .ZIP Library
3. Select ZIP file
4. Done!

### Method 2: Manual

1. Copy library to Arduino library folder:
   - Windows: `Documents\Arduino\libraries\MCP23S17`
   - Linux: `~/Arduino/libraries/MCP23S17`
   - Mac: `~/Documents/Arduino/libraries/MCP23S17`
2. Restart Arduino IDE

## Wiring

### Standard ESP32 to MCP23S17

| ESP32 Pin | MCP23S17 Pin | Description |
|-----------|--------------|-------------|
| GPIO 18   | SCK (Pin 12) | SPI Clock   |
| GPIO 23   | SI (Pin 13)  | SPI MOSI    |
| GPIO 19   | SO (Pin 14)  | SPI MISO    |
| GPIO 5    | CS (Pin 11)  | Chip Select |
| 3.3V      | VDD (Pin 9)  | Power       |
| 3.3V      | RESET (Pin 18)| Reset (high)|
| GND       | VSS (Pin 10) | Ground      |

### Hardware Addressing (A0, A1, A2)

Pins A0, A1, A2 (Pins 15, 16, 17) determine the chip address:

| A2 | A1 | A0 | Address |
|----|----|----|---------|
| 0  | 0  | 0  | 0       |
| 0  | 0  | 1  | 1       |
| 0  | 1  | 0  | 2       |
| 0  | 1  | 1  | 3       |
| 1  | 0  | 0  | 4       |
| 1  | 0  | 1  | 5       |
| 1  | 1  | 0  | 6       |
| 1  | 1  | 1  | 7       |

**Note**: For address 0, connect all three pins to GND.

### Interrupt Pins (Optional)

- **INTA** (Pin 20): Interrupt A
- **INTB** (Pin 19): Interrupt B

These can be connected to an ESP32 GPIO pin.

## Quick Start

### Simple Blink Example

```cpp
#include <MCP23S17.h>

#define CS_PIN 5

MCP23S17 mcp(CS_PIN, 0);  // CS pin 5, address 0

void setup() {
  mcp.begin();
  mcp.pinMode(0, MCP23S17_OUTPUT);  // Pin 0 as output
}

void loop() {
  mcp.digitalWrite(0, HIGH);
  delay(1000);
  mcp.digitalWrite(0, LOW);
  delay(1000);
}
```

## API Reference

### Constructor

```cpp
MCP23S17(uint8_t csPin, uint8_t address = 0, uint32_t spiFrequency = 10000000)
```

- `csPin`: GPIO pin for Chip Select
- `address`: Hardware address 0-7 (via A0, A1, A2)
- `spiFrequency`: SPI frequency in Hz (default: 10 MHz)

### Initialization

```cpp
bool begin()
```

Initializes the MCP23S17. Must be called in `setup()`.

**Returns**: `true` on success, `false` on error

### GPIO Basic Functions

#### pinMode

```cpp
void pinMode(uint8_t pin, uint8_t mode)
```

Configures a pin as input or output.

- `pin`: 0-15 (0-7 = Port A, 8-15 = Port B)
- `mode`: 
  - `MCP23S17_INPUT` - Input without pull-up
  - `MCP23S17_OUTPUT` - Output
  - `MCP23S17_INPUT_PULLUP` - Input with pull-up

**Example**:
```cpp
mcp.pinMode(0, MCP23S17_OUTPUT);         // Pin 0 as output
mcp.pinMode(8, MCP23S17_INPUT);          // Pin 8 as input
mcp.pinMode(9, MCP23S17_INPUT_PULLUP);   // Pin 9 as input with pull-up
```

#### digitalWrite

```cpp
void digitalWrite(uint8_t pin, uint8_t value)
```

Writes a digital value to a pin.

- `pin`: 0-15
- `value`: `HIGH` or `LOW`

**Example**:
```cpp
mcp.digitalWrite(0, HIGH);  // Pin 0 to HIGH
mcp.digitalWrite(0, LOW);   // Pin 0 to LOW
```

#### digitalRead

```cpp
uint8_t digitalRead(uint8_t pin)
```

Reads the digital state of a pin.

- `pin`: 0-15

**Returns**: `HIGH` or `LOW`

**Example**:
```cpp
if (mcp.digitalRead(8) == HIGH) {
  // Pin 8 is HIGH
}
```

### Port-wise Operations

#### portMode

```cpp
void portMode(uint8_t port, uint8_t modes)
```

Configures all 8 pins of a port simultaneously.

- `port`: `MCP23S17_PORTA` or `MCP23S17_PORTB`
- `modes`: 8-bit value (1 = output, 0 = input)

**Example**:
```cpp
// Port A: Pins 0-3 as output, 4-7 as input
mcp.portMode(MCP23S17_PORTA, 0b00001111);

// Port B: All as output
mcp.portMode(MCP23S17_PORTB, 0xFF);
```

#### writePort

```cpp
void writePort(uint8_t port, uint8_t value)
```

Writes 8-bit value to a port.

- `port`: `MCP23S17_PORTA` or `MCP23S17_PORTB`
- `value`: 8-bit value

**Example**:
```cpp
mcp.writePort(MCP23S17_PORTA, 0b10101010);  // Alternating pattern
mcp.writePort(MCP23S17_PORTB, 255);         // All HIGH
```

#### readPort

```cpp
uint8_t readPort(uint8_t port)
```

Reads 8-bit value from a port.

**Example**:
```cpp
uint8_t portValue = mcp.readPort(MCP23S17_PORTA);
```

#### writeGPIO

```cpp
void writeGPIO(uint16_t value)
```

Writes all 16 pins simultaneously.

- `value`: 16-bit value (low byte = Port A, high byte = Port B)

**Example**:
```cpp
mcp.writeGPIO(0x00FF);  // Port A = 0xFF, Port B = 0x00
```

#### readGPIO

```cpp
uint16_t readGPIO()
```

Reads all 16 pins simultaneously.

**Example**:
```cpp
uint16_t allPins = mcp.readGPIO();
```

### Pull-up Resistors

#### setPullup

```cpp
void setPullup(uint8_t pin, bool enable)
```

Enables/disables pull-up for a pin.

**Example**:
```cpp
mcp.setPullup(8, true);   // Enable pull-up
mcp.setPullup(9, false);  // Disable pull-up
```

#### setPortPullups

```cpp
void setPortPullups(uint8_t port, uint8_t value)
```

Configures pull-ups for entire port.

**Example**:
```cpp
mcp.setPortPullups(MCP23S17_PORTB, 0xFF);  // All pull-ups on
```

### Interrupt Functions

#### enableInterrupt

```cpp
void enableInterrupt(uint8_t pin, uint8_t mode = MCP23S17_INT_CHANGE, uint8_t defaultValue = 0)
```

Enables interrupt for a pin.

- `pin`: 0-15
- `mode`:
  - `MCP23S17_INT_CHANGE` - Interrupt on change
  - `MCP23S17_INT_COMPARE` - Interrupt on difference from defaultValue
- `defaultValue`: Compare value (only for COMPARE mode)

**Example**:
```cpp
// Interrupt on any change
mcp.enableInterrupt(0, MCP23S17_INT_CHANGE);

// Interrupt only when pin goes HIGH
mcp.enableInterrupt(1, MCP23S17_INT_COMPARE, LOW);
```

#### disableInterrupt

```cpp
void disableInterrupt(uint8_t pin)
```

Disables interrupt for a pin.

#### setPortInterrupts

```cpp
void setPortInterrupts(uint8_t port, uint8_t mask, uint8_t mode = MCP23S17_INT_CHANGE)
```

Configures interrupts for entire port.

**Example**:
```cpp
// Enable interrupts for pins 0, 2, 4
mcp.setPortInterrupts(MCP23S17_PORTA, 0b00010101);
```

#### setInterruptPolarity

```cpp
void setInterruptPolarity(bool activeHigh)
```

Sets interrupt polarity.

- `true`: Active-high
- `false`: Active-low (default)

#### setInterruptMirror

```cpp
void setInterruptMirror(bool enable)
```

Connects INTA and INTB pins.

#### getInterruptFlags

```cpp
uint8_t getInterruptFlags(uint8_t port)
```

Reads which pins triggered an interrupt.

**Example**:
```cpp
uint8_t flags = mcp.getInterruptFlags(MCP23S17_PORTA);
if (flags & 0x01) {
  // Pin 0 triggered interrupt
}
```

#### getInterruptCapture

```cpp
uint8_t getInterruptCapture(uint8_t port)
```

Reads GPIO state at time of interrupt.

#### clearInterrupts

```cpp
void clearInterrupts()
```

Clears all interrupts.

### Advanced Functions

#### reset

```cpp
void reset()
```

Resets all registers to default values.

#### readRegister / writeRegister

```cpp
uint8_t readRegister(uint8_t reg)
void writeRegister(uint8_t reg, uint8_t value)
```

Direct register access for advanced applications.

## Examples

The library includes the following examples:

1. **MCP23S17_Blink** - Simple LED blinking
2. **MCP23S17_Interrupt** - Button with interrupt handling
3. **MCP23S17_PortOperations** - Port-wise operations
4. **MCP23S17_MultipleChips** - Multiple chips simultaneously
5. **MCP23S17_FullTest** - Complete function test

## Using Multiple Chips

Up to 8 MCP23S17 can operate on the same CS pin:

```cpp
MCP23S17 mcp1(CS_PIN, 0);  // Address 0
MCP23S17 mcp2(CS_PIN, 1);  // Address 1
MCP23S17 mcp3(CS_PIN, 2);  // Address 2

void setup() {
  mcp1.begin();
  mcp2.begin();
  mcp3.begin();
  
  // Now all 3 chips can be controlled independently
  mcp1.digitalWrite(0, HIGH);
  mcp2.digitalWrite(0, HIGH);
  mcp3.digitalWrite(0, HIGH);
}
```

## Tips & Tricks

### Performance

- For best performance, use port-wise operations instead of individual pins
- Register cache reduces unnecessary SPI transfers

### Interrupt Handling

```cpp
#define INT_PIN 4
volatile bool interruptFlag = false;

void IRAM_ATTR handleInterrupt() {
  interruptFlag = true;
}

void setup() {
  // ... Initialize MCP ...
  
  pinMode(INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), handleInterrupt, FALLING);
  
  mcp.enableInterrupt(0);
}

void loop() {
  if (interruptFlag) {
    interruptFlag = false;
    
    uint8_t flags = mcp.getInterruptFlags(MCP23S17_PORTA);
    uint8_t capture = mcp.getInterruptCapture(MCP23S17_PORTA);
    
    // Process interrupt ...
    
    mcp.clearInterrupts();  // Important!
  }
}
```

### Debugging

```cpp
// Output register values
uint8_t iocon = mcp.readRegister(MCP23S17_IOCON);
Serial.print("IOCON: 0x");
Serial.println(iocon, HEX);
```

## Troubleshooting

### Chip Not Detected

- Check wiring (especially VDD and GND)
- Verify SPI connection
- Ensure RESET pin is HIGH
- Check addressing (A0, A1, A2)

### Pins Not Responding

- Check if `pinMode()` was called
- For inputs: Pull-up enabled?
- For outputs: Correct port configuration?

### Interrupts Not Working

- Call `clearInterrupts()` after processing
- Check interrupt polarity
- ESP32 interrupt pin configured correctly?

## Technical Details

- **Communication**: SPI Mode 0 (CPOL=0, CPHA=0)
- **Max. SPI Frequency**: 10 MHz (default)
- **Supply Voltage**: 1.8V - 5.5V (ESP32: 3.3V)
- **Max. Output Current**: 25 mA per pin

## License

This library is freely available for private and commercial use.

## Support

For questions or issues, please create an issue on GitHub.

## Changelog

### Version 1.0.0
- Initial release
- All basic functions implemented
- 5 examples included
- Complete documentation
