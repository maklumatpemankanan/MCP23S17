/*
 * MCP23S17_MultipleChips.ino
 * 
 * Beispiel: Mehrere MCP23S17 Chips am selben SPI-Bus
 * 
 * Hardware-Verbindungen (ESP32 <-> MCP23S17):
 * Gemeinsame Verbindungen:
 * - GPIO 18 (SCK)  <-> SCK (alle Chips)
 * - GPIO 23 (MOSI) <-> SI  (alle Chips)
 * - GPIO 19 (MISO) <-> SO  (alle Chips)
 * - GPIO 5  (CS)   <-> CS  (alle Chips)
 * - 3.3V           <-> VDD, RESET (alle Chips)
 * - GND            <-> GND (alle Chips)
 * 
 * Chip-Adressierung:
 * Chip 1: A0=GND, A1=GND, A2=GND -> Adresse 0
 * Chip 2: A0=VDD, A1=GND, A2=GND -> Adresse 1
 * Chip 3: A0=GND, A1=VDD, A2=GND -> Adresse 2
 * 
 * LEDs:
 * - Chip 1, Pin 0-7: LEDs 0-7
 * - Chip 2, Pin 0-7: LEDs 8-15
 * - Chip 3, Pin 0-7: LEDs 16-23
 */

#include <MCP23S17.h>

// CS-Pin für ESP32
#define CS_PIN 5

// Drei MCP23S17 Objekte mit unterschiedlichen Adressen
MCP23S17 mcp1(CS_PIN, 0);  // Adresse 0 (A2=0, A1=0, A0=0)
MCP23S17 mcp2(CS_PIN, 1);  // Adresse 1 (A2=0, A1=0, A0=1)
MCP23S17 mcp3(CS_PIN, 2);  // Adresse 2 (A2=0, A1=1, A0=0)

void setup() {
  Serial.begin(115200);
  Serial.println("\nMCP23S17 Multiple Chips Beispiel");
  
  // Chip 1 initialisieren
  if (!mcp1.begin()) {
    Serial.println("Fehler: MCP23S17 #1 (Adresse 0) nicht gefunden!");
  } else {
    Serial.println("MCP23S17 #1 initialisiert");
  }
  
  // Chip 2 initialisieren
  if (!mcp2.begin()) {
    Serial.println("Fehler: MCP23S17 #2 (Adresse 1) nicht gefunden!");
  } else {
    Serial.println("MCP23S17 #2 initialisiert");
  }
  
  // Chip 3 initialisieren
  if (!mcp3.begin()) {
    Serial.println("Fehler: MCP23S17 #3 (Adresse 2) nicht gefunden!");
  } else {
    Serial.println("MCP23S17 #3 initialisiert");
  }
  
  // Alle Pins 0-7 auf allen Chips als Ausgang
  mcp1.portMode(MCP23S17_PORTA, 0xFF);
  mcp2.portMode(MCP23S17_PORTA, 0xFF);
  mcp3.portMode(MCP23S17_PORTA, 0xFF);
  
  Serial.println("\nLauflichter über alle 24 LEDs...\n");
}

void loop() {
  // Lauflicht über alle 3 Chips (24 LEDs)
  
  // Chip 1 (LEDs 0-7)
  for (int pin = 0; pin < 8; pin++) {
    mcp1.writePort(MCP23S17_PORTA, 1 << pin);
    Serial.print("LED ");
    Serial.println(pin);
    delay(100);
  }
  
  // Chip 2 (LEDs 8-15)
  for (int pin = 0; pin < 8; pin++) {
    mcp2.writePort(MCP23S17_PORTA, 1 << pin);
    Serial.print("LED ");
    Serial.println(pin + 8);
    delay(100);
  }
  
  // Chip 3 (LEDs 16-23)
  for (int pin = 0; pin < 8; pin++) {
    mcp3.writePort(MCP23S17_PORTA, 1 << pin);
    Serial.print("LED ");
    Serial.println(pin + 16);
    delay(100);
  }
  
  // Alle LEDs aus
  mcp1.writePort(MCP23S17_PORTA, 0x00);
  mcp2.writePort(MCP23S17_PORTA, 0x00);
  mcp3.writePort(MCP23S17_PORTA, 0x00);
  
  delay(500);
  
  // Binärzähler über alle 24 LEDs
  Serial.println("\nBinär-Zähler...\n");
  
  for (uint32_t count = 0; count < 256; count++) {
    // Untere 8 Bits -> Chip 1
    mcp1.writePort(MCP23S17_PORTA, count & 0xFF);
    
    // Mittlere 8 Bits -> Chip 2
    mcp2.writePort(MCP23S17_PORTA, (count >> 8) & 0xFF);
    
    // Obere 8 Bits -> Chip 3
    mcp3.writePort(MCP23S17_PORTA, (count >> 16) & 0xFF);
    
    Serial.print("Zähler: ");
    Serial.println(count);
    
    delay(50);
  }
  
  Serial.println("\nNeustart...\n");
  delay(1000);
}
