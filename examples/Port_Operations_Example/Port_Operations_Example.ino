/*
 * Port_Operations_Example.ino
 * 
 * Beispiel für port-weise Operationen mit dem MCP23S17
 * Demonstriert das gleichzeitige Setzen mehrerer Pins
 * 
 * Hardware Setup:
 * - MCP23S17 verbunden über SPI
 * - CS-Pin an GPIO 5
 * - Hardware-Adresse: 0
 * - 8 LEDs an Port A (Pins 0-7)
 * - 8 Taster an Port B (Pins 8-15) mit Pull-ups
 * 
 * Funktionen:
 * - Lauflicht auf Port A
 * - Port B Eingänge lesen und auf Serial Monitor ausgeben
 * - Demonstriert schnelle Port-Operationen
 */

#include <MCP23S17.h>

// MCP23S17 Instanz
MCP23S17 mcp(5, 0);

void setup() {
  Serial.begin(115200);
  Serial.println("MCP23S17 Port Operations Example");
  Serial.println("=================================");
  
  // MCP23S17 initialisieren
  if (!mcp.begin()) {
    Serial.println("Fehler bei Initialisierung!");
    while(1);
  }
  Serial.println("MCP23S17 initialisiert!");
  
  // Port A komplett als Ausgang (0x00 = alle Pins Ausgang)
  mcp.portMode(MCP23S17_PORTA, 0x00);
  Serial.println("Port A: Alle Pins als Ausgang");
  
  // Port B komplett als Eingang (0xFF = alle Pins Eingang)
  mcp.portMode(MCP23S17_PORTB, 0xFF);
  Serial.println("Port B: Alle Pins als Eingang");
  
  // Pull-ups für Port B aktivieren (0xFF = alle Pull-ups aktiv)
  mcp.setPortPullups(MCP23S17_PORTB, 0xFF);
  Serial.println("Port B: Pull-ups aktiviert");
  
  Serial.println("\nStarte Lauflicht auf Port A...");
  Serial.println();
}

void loop() {
  // === Lauflicht auf Port A ===
  
  // Bit-Muster für Lauflicht
  uint8_t patterns[] = {
    0b00000001,  // Pin 0
    0b00000010,  // Pin 1
    0b00000100,  // Pin 2
    0b00001000,  // Pin 3
    0b00010000,  // Pin 4
    0b00100000,  // Pin 5
    0b01000000,  // Pin 6
    0b10000000,  // Pin 7
  };
  
  // Durch alle Muster durchgehen
  for (int i = 0; i < 8; i++) {
    // Kompletten Port A mit einem Schreibvorgang setzen
    mcp.writePort(MCP23S17_PORTA, patterns[i]);
    
    // Port B lesen (alle 8 Pins auf einmal)
    uint8_t portB_value = mcp.readPort(MCP23S17_PORTB);
    
    // Binärdarstellung ausgeben
    Serial.print("Port A (LEDs): ");
    Serial.print(patterns[i], BIN);
    Serial.print(" | Port B (Taster): ");
    Serial.print(portB_value, BIN);
    
    // Anzahl gedrückter Taster zählen (LOW = gedrückt bei Pull-up)
    uint8_t pressed = 0;
    for (int j = 0; j < 8; j++) {
      if (!(portB_value & (1 << j))) {
        pressed++;
      }
    }
    Serial.print(" | Gedrückte Taster: ");
    Serial.println(pressed);
    
    delay(200);
  }
  
  // === Rückwärts ===
  for (int i = 6; i >= 1; i--) {
    mcp.writePort(MCP23S17_PORTA, patterns[i]);
    
    uint8_t portB_value = mcp.readPort(MCP23S17_PORTB);
    
    Serial.print("Port A (LEDs): ");
    Serial.print(patterns[i], BIN);
    Serial.print(" | Port B (Taster): ");
    Serial.println(portB_value, BIN);
    
    delay(200);
  }
  
  // === Alle LEDs gleichzeitig blinken ===
  Serial.println("\nAlle LEDs blinken...");
  
  // Alle an (0xFF = alle Bits gesetzt)
  mcp.writePort(MCP23S17_PORTA, 0xFF);
  delay(500);
  
  // Alle aus (0x00 = alle Bits gelöscht)
  mcp.writePort(MCP23S17_PORTA, 0x00);
  delay(500);
  
  Serial.println();
}
