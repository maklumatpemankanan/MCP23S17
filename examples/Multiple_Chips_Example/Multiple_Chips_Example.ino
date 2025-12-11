/*
 * Multiple_Chips_Example.ino
 * 
 * Demonstriert die Verwendung von mehreren MCP23S17 Chips
 * am gleichen SPI-Bus mit unterschiedlichen Hardware-Adressen
 * 
 * Hardware Setup:
 * - Mehrere MCP23S17 am gleichen SPI-Bus
 * - Alle teilen sich: MOSI, MISO, SCK, CS
 * - Unterschiedliche Hardware-Adressen durch A0, A1, A2 Pins:
 *   - Chip 1: A0=0, A1=0, A2=0 → Adresse 0
 *   - Chip 2: A0=1, A1=0, A2=0 → Adresse 1
 *   - Chip 3: A0=0, A1=1, A2=0 → Adresse 2
 * - CS-Pin an GPIO 5 (gemeinsam für alle Chips)
 * 
 * Funktionen:
 * - Verschiedene LEDs an verschiedenen Chips steuern
 * - Zeigt wie bis zu 8 Chips (48 zusätzliche I/Os!) verwendet werden können
 */

#include <MCP23S17.h>

// Drei MCP23S17 Instanzen mit unterschiedlichen Adressen
// Alle verwenden den gleichen CS-Pin (5), aber verschiedene Adressen
MCP23S17 mcp1(5, 0);  // Adresse 0: A0=0, A1=0, A2=0
MCP23S17 mcp2(5, 1);  // Adresse 1: A0=1, A1=0, A2=0
MCP23S17 mcp3(5, 2);  // Adresse 2: A0=0, A1=1, A2=0

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("MCP23S17 Multiple Chips Example");
  Serial.println("================================");
  Serial.println();
  
  // === Chip 1 initialisieren (Adresse 0) ===
  Serial.print("Initialisiere Chip 1 (Adresse 0)... ");
  if (mcp1.begin()) {
    Serial.println("OK!");
  } else {
    Serial.println("FEHLER!");
  }
  
  // === Chip 2 initialisieren (Adresse 1) ===
  Serial.print("Initialisiere Chip 2 (Adresse 1)... ");
  if (mcp2.begin()) {
    Serial.println("OK!");
  } else {
    Serial.println("FEHLER!");
  }
  
  // === Chip 3 initialisieren (Adresse 2) ===
  Serial.print("Initialisiere Chip 3 (Adresse 2)... ");
  if (mcp3.begin()) {
    Serial.println("OK!");
  } else {
    Serial.println("FEHLER!");
  }
  
  Serial.println();
  
  // === Pins konfigurieren ===
  
  // Chip 1: Port A als Ausgang (LEDs 0-7)
  mcp1.portMode(MCP23S17_PORTA, 0x00);  // Alle Ausgänge
  Serial.println("Chip 1 Port A: Ausgänge (LEDs)");
  
  // Chip 2: Port A und B als Ausgang (LEDs 0-15)
  mcp2.portMode(MCP23S17_PORTA, 0x00);
  mcp2.portMode(MCP23S17_PORTB, 0x00);
  Serial.println("Chip 2 Port A+B: Ausgänge (LEDs)");
  
  // Chip 3: Port B als Eingang mit Pull-ups (Taster)
  mcp3.portMode(MCP23S17_PORTB, 0xFF);  // Alle Eingänge
  mcp3.setPortPullups(MCP23S17_PORTB, 0xFF);  // Pull-ups aktivieren
  Serial.println("Chip 3 Port B: Eingänge mit Pull-ups (Taster)");
  
  Serial.println();
  Serial.println("=== Gesamt: 48 I/O Pins verfügbar! ===");
  Serial.println("Chip 1: 16 Pins");
  Serial.println("Chip 2: 16 Pins");
  Serial.println("Chip 3: 16 Pins");
  Serial.println();
  Serial.println("Starte Demo...");
  Serial.println();
}

void loop() {
  // === Demo 1: Lauflicht über alle 3 Chips ===
  Serial.println("Demo 1: Lauflicht über alle Chips");
  
  for (int chip = 1; chip <= 3; chip++) {
    for (int pin = 0; pin < 8; pin++) {
      // LED einschalten
      switch(chip) {
        case 1:
          mcp1.digitalWrite(pin, HIGH);
          Serial.print("Chip 1, Pin ");
          break;
        case 2:
          mcp2.digitalWrite(pin, HIGH);
          Serial.print("Chip 2, Pin ");
          break;
        case 3:
          mcp3.digitalWrite(pin, HIGH);
          Serial.print("Chip 3, Pin ");
          break;
      }
      Serial.print(pin);
      Serial.println(": EIN");
      
      delay(100);
      
      // LED ausschalten
      switch(chip) {
        case 1:
          mcp1.digitalWrite(pin, LOW);
          break;
        case 2:
          mcp2.digitalWrite(pin, LOW);
          break;
        case 3:
          mcp3.digitalWrite(pin, LOW);
          break;
      }
    }
  }
  
  Serial.println();
  
  // === Demo 2: Alle LEDs blinken gleichzeitig ===
  Serial.println("Demo 2: Alle LEDs blinken gleichzeitig");
  
  for (int i = 0; i < 3; i++) {
    // Alle an
    mcp1.writePort(MCP23S17_PORTA, 0xFF);
    mcp2.writePort(MCP23S17_PORTA, 0xFF);
    mcp2.writePort(MCP23S17_PORTB, 0xFF);
    Serial.println("Alle LEDs: EIN");
    delay(500);
    
    // Alle aus
    mcp1.writePort(MCP23S17_PORTA, 0x00);
    mcp2.writePort(MCP23S17_PORTA, 0x00);
    mcp2.writePort(MCP23S17_PORTB, 0x00);
    Serial.println("Alle LEDs: AUS");
    delay(500);
  }
  
  Serial.println();
  
  // === Demo 3: Taster von Chip 3 lesen ===
  Serial.println("Demo 3: Taster-Status von Chip 3 lesen");
  
  uint8_t buttons = mcp3.readPort(MCP23S17_PORTB);
  Serial.print("Taster Port B (binär): ");
  for (int i = 7; i >= 0; i--) {
    Serial.print((buttons & (1 << i)) ? "1" : "0");
  }
  Serial.println();
  
  // Einzelne Taster anzeigen
  for (int i = 0; i < 8; i++) {
    bool pressed = !(buttons & (1 << i));  // LOW = gedrückt
    Serial.print("Taster ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(pressed ? "GEDRÜCKT" : "nicht gedrückt");
  }
  
  Serial.println();
  
  // === Demo 4: 16-Bit Operation auf Chip 2 ===
  Serial.println("Demo 4: 16-Bit Muster auf Chip 2");
  
  uint16_t patterns[] = {
    0x0001, 0x0002, 0x0004, 0x0008,
    0x0010, 0x0020, 0x0040, 0x0080,
    0x0100, 0x0200, 0x0400, 0x0800,
    0x1000, 0x2000, 0x4000, 0x8000
  };
  
  for (int i = 0; i < 16; i++) {
    mcp2.writeGPIO(patterns[i]);
    Serial.print("Muster: 0x");
    Serial.println(patterns[i], HEX);
    delay(100);
  }
  
  mcp2.writeGPIO(0x0000);  // Alle aus
  
  Serial.println();
  Serial.println("=== Demo beendet, starte von vorne ===");
  Serial.println();
  delay(2000);
}
