# MCP23S17 Arduino Library

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Version](https://img.shields.io/badge/version-1.2.0-green.svg)
![Arduino](https://img.shields.io/badge/Arduino-Compatible-brightgreen.svg)
![ESP32](https://img.shields.io/badge/ESP32-Optimized-orange.svg)

A comprehensive Arduino library for the **MCP23S17** 16-bit SPI I/O Expander from Microchip.

## ✨ Features

- 🎯 **Arduino-style API** - pinMode, digitalWrite, digitalRead
- ⚡ **Interrupt Support** - Full interrupt configuration and handling
- 🔌 **Pull-up Resistors** - Configurable for each pin
- 📦 **Port Operations** - Control 8 pins simultaneously
- 🔢 **16-bit GPIO** - Read/write all 16 pins at once
- 🔗 **Multiple Chips** - Up to 8 MCP23S17 per CS pin
- 🎛️ **Flexible SPI** - Three initialization methods
- 🚀 **ESP32 Optimized** - Tested and optimized for ESP32
- 📝 **Well Documented** - Extensive comments and examples

## 📋 Table of Contents

- [Hardware Requirements](#hardware-requirements)
- [Installation](#installation)
- [Wiring](#wiring)
- [Quick Start](#quick-start)
- [SPI Initialization Methods](#spi-initialization-methods)
- [API Reference](#api-reference)
- [Examples](#examples)
- [Advanced Usage](#advanced-usage)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## 🔧 Hardware Requirements

- Arduino-compatible board (ESP32 recommended)
- MCP23S17 SPI I/O Expander
- SPI connection (4 wires minimum)

### Tested Platforms

- ✅ ESP32
- ✅ Arduino Uno
- ✅ Arduino Mega
- ✅ Arduino Nano

## 📥 Installation

### Method 1: Arduino Library Manager (Recommended)

1. Open Arduino IDE
2. Go to `Sketch` → `Include Library` → `Manage Libraries...`
3. Search for "MCP23S17"
4. Click "Install"

### Method 2: Manual Installation

1. Download the latest release as ZIP
2. In Arduino IDE: `Sketch` → `Include Library` → `Add .ZIP Library...`
3. Select the downloaded ZIP file
4. Restart Arduino IDE

### Method 3: Git Clone

```bash
cd ~/Arduino/libraries
git clone https://github.com/yourusername/MCP23S17.git
```

## 🔌 Wiring

### Standard ESP32 to MCP23S17

| ESP32 Pin | MCP23S17 Pin | Function     |
| --------- | ------------ | ------------ |
| GPIO 18   | SCK (12)     | SPI Clock    |
| GPIO 23   | SI (13)      | SPI MOSI     |
| GPIO 19   | SO (14)      | SPI MISO     |
| GPIO 5    | CS (11)      | Chip Select  |
| 3.3V      | VDD (9)      | Power        |
| 3.3V      | RESET (18)   | Reset (High) |
| GND       | VSS (10)     | Ground       |

### Hardware Addressing (A0, A1, A2)

Connect pins A0, A1, A2 (pins 15, 16, 17) to set chip address:

| A2  | A1  | A0  | Address |
| --- | --- | --- | ------- |
| 0   | 0   | 0   | 0       |
| 0   | 0   | 1   | 1       |
| 0   | 1   | 0   | 2       |
| 0   | 1   | 1   | 3       |
| 1   | 0   | 0   | 4       |
| 1   | 0   | 1   | 5       |
| 1   | 1   | 0   | 6       |
| 1   | 1   | 1   | 7       |

For address 0: Connect all three to GND.

### Optional Interrupt Pins

- **INTA** (Pin 20): Interrupt A
- **INTB** (Pin 19): Interrupt B

Connect to ESP32 GPIO for interrupt handling.

## 🚀 Quick Start

```cpp
#include <MCP23S17.h>

#define CS_PIN 5

MCP23S17 mcp(CS_PIN, 0);  // CS pin 5, address 0

void setup() {
  Serial.begin(115200);

  if (!mcp.begin()) {
    Serial.println("MCP23S17 not found!");
    while(1);
  }

  mcp.pinMode(0, MCP23S17_OUTPUT);  // Pin 0 as output
}

void loop() {
  mcp.digitalWrite(0, HIGH);
  delay(1000);
  mcp.digitalWrite(0, LOW);
  delay(1000);
}
```

## 🎛️ SPI Initialization Methods

The library offers **three flexible ways** to initialize the MCP23S17:

### Method 1: Standard SPI Pins (Simplest)

Uses the board's default SPI pins.

```cpp
MCP23S17 mcp(CS_PIN, 0);

void setup() {
  mcp.begin();  // Uses default pins
}
```

**ESP32 Default Pins (VSPI):**

- SCK = GPIO 18
- MISO = GPIO 19
- MOSI = GPIO 23

---

### Method 2: Custom SPI Pins (Flexible)

Specify your own SPI pins.

```cpp
MCP23S17 mcp(CS_PIN, 0);

void setup() {
  mcp.begin(14, 12, 13);  // SCK=14, MISO=12, MOSI=13
}
```

**Use when:**

- Default pins are already in use
- Special PCB routing requirements
- Different pin layout needed

---

### Method 3: Existing SPI Bus (Most Powerful)

Share an SPI bus with multiple devices.

```cpp
SPIClass mySPI(HSPI);
MCP23S17 mcp(CS_PIN, 0);

void setup() {
  // Initialize bus first
  mySPI.begin(14, 12, 13, 15);

  // Connect MCP to existing bus
  mcp.begin(&mySPI);
}
```

**Use when:**

- Multiple SPI devices on same bus
- Need full control over SPI settings
- Separate buses for different device groups
- Working with other SPI libraries

---

### Comparison Table

| Feature       | Standard | Custom Pins | Existing Bus |
| ------------- | -------- | ----------- | ------------ |
| Simplicity    | ⭐⭐⭐      | ⭐⭐          | ⭐            |
| Flexibility   | ⭐        | ⭐⭐          | ⭐⭐⭐          |
| Bus Sharing   | ❌        | ❌           | ✅            |
| Pin Selection | ❌        | ✅           | ✅            |
| SPI Control   | ❌        | ❌           | ✅            |

## 📚 API Reference

### Constructor

```cpp
MCP23S17(uint8_t csPin, uint8_t address = 0, uint32_t spiFrequency = 10000000)
```

- `csPin`: GPIO pin for Chip Select
- `address`: Hardware address 0-7 (via A0, A1, A2)
- `spiFrequency`: SPI frequency in Hz (default: 10 MHz)

### Initialization

#### begin()

```cpp
bool begin()
```

Initialize with default SPI pins.

**Returns:** `true` on success, `false` on failure

#### begin(sck, miso, mosi)

```cpp
bool begin(int8_t sck, int8_t miso, int8_t mosi)
```

Initialize with custom SPI pins.

**Parameters:**

- `sck`: Serial Clock pin
- `miso`: Master In Slave Out pin
- `mosi`: Master Out Slave In pin

**Returns:** `true` on success, `false` on failure

#### begin(spi)

```cpp
bool begin(SPIClass* spi)
```

Initialize with existing SPI bus.

**Parameters:**

- `spi`: Pointer to initialized SPIClass object

**Returns:** `true` on success, `false` on failure

**Important:** SPI bus must be initialized before calling this!

### Basic GPIO Functions

#### pinMode

```cpp
void pinMode(uint8_t pin, uint8_t mode)
```

Configure pin mode.

**Parameters:**

- `pin`: 0-15 (0-7 = Port A, 8-15 = Port B)
- `mode`: 
  - `MCP23S17_INPUT` - Input without pull-up
  - `MCP23S17_OUTPUT` - Output
  - `MCP23S17_INPUT_PULLUP` - Input with pull-up

**Example:**

```cpp
mcp.pinMode(0, MCP23S17_OUTPUT);
mcp.pinMode(8, MCP23S17_INPUT_PULLUP);
```

#### digitalWrite

```cpp
void digitalWrite(uint8_t pin, uint8_t value)
```

Write digital value to pin.

**Parameters:**

- `pin`: 0-15
- `value`: `HIGH` or `LOW`

**Example:**

```cpp
mcp.digitalWrite(0, HIGH);
```

#### digitalRead

```cpp
uint8_t digitalRead(uint8_t pin)
```

Read digital value from pin.

**Parameters:**

- `pin`: 0-15

**Returns:** `HIGH` or `LOW`

**Example:**

```cpp
if (mcp.digitalRead(8) == HIGH) {
  // Pin 8 is HIGH
}
```

### Port Operations

#### portMode

```cpp
void portMode(uint8_t port, uint8_t modes)
```

Configure all 8 pins of a port.

**Parameters:**

- `port`: `MCP23S17_PORTA` or `MCP23S17_PORTB`
- `modes`: 8-bit value (1 = output, 0 = input)

**Example:**

```cpp
mcp.portMode(MCP23S17_PORTA, 0xFF);  // All outputs
```

#### writePort

```cpp
void writePort(uint8_t port, uint8_t value)
```

Write 8-bit value to port.

**Example:**

```cpp
mcp.writePort(MCP23S17_PORTA, 0b10101010);
```

#### readPort

```cpp
uint8_t readPort(uint8_t port)
```

Read 8-bit value from port.

**Example:**

```cpp
uint8_t value = mcp.readPort(MCP23S17_PORTA);
```

#### writeGPIO

```cpp
void writeGPIO(uint16_t value)
```

Write all 16 pins simultaneously.

**Parameters:**

- `value`: 16-bit value (low byte = Port A, high byte = Port B)

**Example:**

```cpp
mcp.writeGPIO(0x00FF);  // Port A = 0xFF, Port B = 0x00
```

#### readGPIO

```cpp
uint16_t readGPIO()
```

Read all 16 pins simultaneously.

**Example:**

```cpp
uint16_t allPins = mcp.readGPIO();
```

### Pull-up Configuration

#### setPullup

```cpp
void setPullup(uint8_t pin, bool enable)
```

Enable/disable pull-up for single pin.

**Example:**

```cpp
mcp.setPullup(8, true);   // Enable pull-up
```

#### setPortPullups

```cpp
void setPortPullups(uint8_t port, uint8_t value)
```

Configure pull-ups for entire port.

**Example:**

```cpp
mcp.setPortPullups(MCP23S17_PORTB, 0xFF);  // All pull-ups on
```

### Interrupt Functions

#### enableInterrupt

```cpp
void enableInterrupt(uint8_t pin, uint8_t mode = MCP23S17_INT_CHANGE, uint8_t defaultValue = 0)
```

Enable interrupt for pin.

**Parameters:**

- `pin`: 0-15
- `mode`:
  - `MCP23S17_INT_CHANGE` - Interrupt on change
  - `MCP23S17_INT_COMPARE` - Interrupt on difference from defaultValue
- `defaultValue`: Comparison value (only for COMPARE mode)

**Example:**

```cpp
mcp.enableInterrupt(0, MCP23S17_INT_CHANGE);
```

#### disableInterrupt

```cpp
void disableInterrupt(uint8_t pin)
```

Disable interrupt for pin.

#### setPortInterrupts

```cpp
void setPortInterrupts(uint8_t port, uint8_t mask, uint8_t mode = MCP23S17_INT_CHANGE)
```

Configure interrupts for entire port.

#### setInterruptPolarity

```cpp
void setInterruptPolarity(bool activeHigh)
```

Set interrupt polarity.

**Parameters:**

- `true`: Active-High
- `false`: Active-Low (default)

#### setInterruptMirror

```cpp
void setInterruptMirror(bool enable)
```

Connect INTA and INTB pins.

#### getInterruptFlags

```cpp
uint8_t getInterruptFlags(uint8_t port)
```

Read which pins triggered interrupt.

**Example:**

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

Read GPIO state at time of interrupt.

#### clearInterrupts

```cpp
void clearInterrupts()
```

Clear all interrupts.

### Advanced Functions

#### reset

```cpp
void reset()
```

Reset all registers to default values.

#### readRegister / writeRegister

```cpp
uint8_t readRegister(uint8_t reg)
void writeRegister(uint8_t reg, uint8_t value)
```

Direct register access for advanced applications.

## 📖 Examples

The library includes 7 comprehensive examples:

### 1. MCP23S17_Blink

Simple LED blink example - perfect for getting started.

### 2. MCP23S17_CustomSPI

Using custom SPI pins instead of defaults.

### 3. MCP23S17_ExistingSPI

Connecting to an existing SPI bus - ideal for multiple SPI devices.

### 4. MCP23S17_Interrupt

Button with interrupt handling and ISR processing.

### 5. MCP23S17_PortOperations

Control 8 pins simultaneously using port operations.

### 6. MCP23S17_MultipleChips

Using multiple MCP23S17 chips (up to 8 per CS pin).

### 7. MCP23S17_FullTest

Complete functionality test with diagnostics.

## 🔥 Advanced Usage

### Multiple Chips on Same Bus

```cpp
MCP23S17 mcp1(CS_PIN, 0);  // Address 0
MCP23S17 mcp2(CS_PIN, 1);  // Address 1
MCP23S17 mcp3(CS_PIN, 2);  // Address 2

void setup() {
  mcp1.begin();
  mcp2.begin();
  mcp3.begin();

  // Control independently
  mcp1.digitalWrite(0, HIGH);
  mcp2.digitalWrite(0, HIGH);
  mcp3.digitalWrite(0, HIGH);
}
```

### Sharing SPI Bus with Other Devices

```cpp
SPIClass sharedBus(HSPI);

MCP23S17 mcp(15, 0);
// + SD card, display, etc.

void setup() {
  sharedBus.begin(14, 12, 13, 15);

  mcp.begin(&sharedBus);
  SD.begin(4, sharedBus);
  display.begin(&sharedBus);
}
```

### Separate Buses for Different Speeds

```cpp
SPIClass fastBus(VSPI);
SPIClass slowBus(HSPI);

MCP23S17 mcpFast(5, 0);
MCP23S17 mcpSlow(15, 0);

void setup() {
  fastBus.begin();
  fastBus.setFrequency(40000000);  // 40 MHz

  slowBus.begin(14, 12, 13, 15);
  slowBus.setFrequency(1000000);   // 1 MHz

  mcpFast.begin(&fastBus);
  mcpSlow.begin(&slowBus);
}
```

### Interrupt Handling

```cpp
#define INT_PIN 4
volatile bool interruptFlag = false;

void IRAM_ATTR handleInterrupt() {
  interruptFlag = true;
}

void setup() {
  mcp.begin();

  pinMode(INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), handleInterrupt, FALLING);

  mcp.pinMode(0, MCP23S17_INPUT_PULLUP);
  mcp.enableInterrupt(0);
  mcp.clearInterrupts();
}

void loop() {
  if (interruptFlag) {
    interruptFlag = false;

    uint8_t flags = mcp.getInterruptFlags(MCP23S17_PORTA);
    uint8_t capture = mcp.getInterruptCapture(MCP23S17_PORTA);

    // Process interrupt

    mcp.clearInterrupts();  // Important!
  }
}
```

## 🐛 Troubleshooting

### Chip Not Found

**Problem:** `begin()` returns false

**Solutions:**

- Check wiring (especially VDD and GND)
- Verify SPI connections
- Ensure RESET pin is HIGH (3.3V)
- Check address pins (A0, A1, A2)
- Try slower SPI frequency: `MCP23S17 mcp(5, 0, 1000000);`

### Pins Not Responding

**Problem:** digitalWrite/digitalRead not working

**Solutions:**

- Verify `pinMode()` was called
- For inputs: check if pull-up is needed
- For outputs: verify port configuration
- Test with register dump: `Serial.println(mcp.readRegister(MCP23S17_IOCON), HEX);`

### Interrupts Not Working

**Problem:** No interrupt triggered

**Solutions:**

- Call `clearInterrupts()` after processing
- Check interrupt polarity setting
- Verify ESP32 interrupt pin configuration
- Ensure interrupt is enabled: `mcp.enableInterrupt(pin)`
- Check INTA/INTB connection

### Multiple Chips Conflict

**Problem:** Chips interfere with each other

**Solutions:**

- Ensure different addresses (A0, A1, A2)
- If same address: use different CS pins
- Check HAEN bit is enabled (done automatically)
- Verify each chip initializes: check `begin()` return value

## 💡 Tips & Best Practices

### Performance

- Use port operations instead of individual pins for better speed
- Register cache reduces unnecessary SPI transfers
- For maximum speed: use `writeGPIO()` for all 16 pins at once

### Power Management

- MCP23S17 supports 1.8V - 5.5V operation
- ESP32: use 3.3V
- Max current per pin: 25 mA
- Use external drivers for high-current loads

### Debugging

```cpp
// Print configuration
Serial.print("IOCON: 0x");
Serial.println(mcp.readRegister(MCP23S17_IOCON), HEX);

// Dump all GPIO states
uint16_t state = mcp.readGPIO();
Serial.print("GPIO: 0b");
Serial.println(state, BIN);
```

### SPI Bus Sharing

When sharing SPI bus with other devices:

- MCP handles transactions automatically
- No manual transaction management needed
- Different devices can use different CS pins
- Or use hardware addressing for multiple MCPs

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

### Development Setup

```bash
git clone https://github.com/yourusername/MCP23S17.git
cd MCP23S17
```

### Coding Standards

- Follow existing code style
- Add comments for complex logic
- Include examples for new features
- Update documentation

## 📄 License

This library is released under the MIT License. See LICENSE file for details.

## 🙏 Acknowledgments

- Microchip for the MCP23S17 datasheet
- Arduino community for inspiration
- All contributors and testers

## 📞 Support

- **Issues:** [GitHub Issues](https://github.com/yourusername/MCP23S17/issues)
- **Discussions:** [GitHub Discussions](https://github.com/yourusername/MCP23S17/discussions)
  
  

## 🔗 Links

- [MCP23S17 Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/20001952C.pdf)
- [Arduino Reference](https://www.arduino.cc/reference/en/)
- [ESP32 Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)

## ⭐ Star History

If you find this library useful, please consider giving it a star! ⭐
