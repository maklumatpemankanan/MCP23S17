/*
 * MCP23S17_CustomSPI.ino
 * 
 * Beispiel: Benutzerdefinierte SPI-Pins verwenden
 * 
 * Dieses Beispiel zeigt wie man den MCP23S17 mit eigenen
 * SPI-Pins initialisiert statt der Standard-Pins zu verwenden.
 * 
 * Hardware-Verbindungen (ESP32 <-> MCP23S17):
 * CUSTOM SPI-PINS:
 * - GPIO 14 (SCK)  <-> SCK (Pin 12)
 * - GPIO 12 (MISO) <-> SO  (Pin 14)
 * - GPIO 13 (MOSI) <-> SI  (Pin 13)
 * - GPIO 15 (CS)   <-> CS  (Pin 11)
 * 
 * STANDARD-VERBINDUNGEN:
 * - 3.3V           <-> VDD (Pin 9), RESET (Pin 18)
 * - GND            <-> VSS (Pin 10), A0, A1, A2 (Adresse = 0)
 * 
 * LED an Pin GPA0 (Pin 21) mit Vorwiderstand nach GND
 */

#include <MCP23S17.h>

// Benutzerdefinierte SPI-Pins
#define CUSTOM_SCK  14
#define CUSTOM_MISO 12
#define CUSTOM_MOSI 13
#define CUSTOM_CS   15

// MCP23S17 Objekt erstellen (CS-Pin, Adresse)
MCP23S17 mcp(CUSTOM_CS, 0);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========================================");
  Serial.println("MCP23S17 Custom SPI-Pins Beispiel");
  Serial.println("========================================\n");
  
  // Zeige verwendete Pins
  Serial.println("Verwendete SPI-Pins:");
  Serial.print("  SCK  (Clock):     GPIO ");
  Serial.println(CUSTOM_SCK);
  Serial.print("  MISO (Data In):   GPIO ");
  Serial.println(CUSTOM_MISO);
  Serial.print("  MOSI (Data Out):  GPIO ");
  Serial.println(CUSTOM_MOSI);
  Serial.print("  CS   (Chip Sel):  GPIO ");
  Serial.println(CUSTOM_CS);
  Serial.println();
  
  // MCP23S17 mit benutzerdefinierten SPI-Pins initialisieren
  // Syntax: mcp.begin(SCK, MISO, MOSI)
  if (!mcp.begin(CUSTOM_SCK, CUSTOM_MISO, CUSTOM_MOSI)) {
    Serial.println("FEHLER: MCP23S17 nicht gefunden!");
    Serial.println("Überprüfe die Verdrahtung:");
    Serial.println("  - VDD und RESET auf 3.3V?");
    Serial.println("  - GND angeschlossen?");
    Serial.println("  - SPI-Pins korrekt?");
    while(1) {
      delay(1000);
    }
  }
  
  Serial.println("✓ MCP23S17 erfolgreich initialisiert");
  Serial.println();
  
  // Pin 0 als Ausgang für LED konfigurieren
  mcp.pinMode(0, MCP23S17_OUTPUT);
  
  Serial.println("LED blinkt an Pin 0...");
  Serial.println("========================================\n");
}

void loop() {
  // LED einschalten
  mcp.digitalWrite(0, HIGH);
  Serial.println("LED AN");
  delay(1000);
  
  // LED ausschalten
  mcp.digitalWrite(0, LOW);
  Serial.println("LED AUS");
  delay(1000);
}

/*
 * HINWEISE:
 * 
 * 1. Standard vs. Custom SPI-Pins:
 *    - Standard: mcp.begin()
 *    - Custom:   mcp.begin(SCK, MISO, MOSI)
 * 
 * 2. ESP32 Standard-SPI-Pins (VSPI):
 *    - SCK  = GPIO 18
 *    - MISO = GPIO 19
 *    - MOSI = GPIO 23
 *    - CS   = Frei wählbar (hier GPIO 5 oft verwendet)
 * 
 * 3. Warum Custom-Pins?
 *    - Standard-Pins bereits belegt
 *    - Spezielle PCB-Anforderungen
 *    - Mehrere SPI-Geräte mit verschiedenen Bussen
 * 
 * 4. Mehrere MCP23S17 mit Custom-Pins:
 *    Alle können die gleichen SPI-Pins verwenden,
 *    aber benötigen unterschiedliche CS-Pins oder Adressen!
 *    
 *    Beispiel:
 *    MCP23S17 mcp1(15, 0);  // CS=15, Adresse=0
 *    MCP23S17 mcp2(15, 1);  // CS=15, Adresse=1
 *    
 *    mcp1.begin(14, 12, 13);  // Beide mit gleichen SPI-Pins
 *    mcp2.begin(14, 12, 13);
 */
