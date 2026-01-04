/*
 * MCP23S17_ExistingSPI.ino
 * 
 * Beispiel: MCP23S17 an bestehendem SPI-Bus verwenden
 * 
 * Dieses Beispiel zeigt wie man den MCP23S17 an einen bereits
 * initialisierten SPI-Bus anschließt. Dies ist nützlich wenn:
 * - Mehrere verschiedene SPI-Geräte am gleichen Bus hängen
 * - Man die volle Kontrolle über den SPI-Bus behalten möchte
 * - Spezielle SPI-Einstellungen erforderlich sind
 * 
 * Hardware-Verbindungen (ESP32 <-> MCP23S17):
 * HSPI-BUS (Beispiel):
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

// Pin-Definitionen
#define SCK_PIN  14
#define MISO_PIN 12
#define MOSI_PIN 13
#define CS_PIN   15

// Eigenen SPI-Bus erstellen (HSPI beim ESP32)
SPIClass customSPI(HSPI);

// MCP23S17 Objekt erstellen (CS-Pin, Adresse)
MCP23S17 mcp(CS_PIN, 0);

// Optional: Weitere SPI-Geräte am gleichen Bus
// Beispiel: SD-Karte, Display, etc.

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========================================");
  Serial.println("MCP23S17 mit bestehendem SPI-Bus");
  Serial.println("========================================\n");
  
  // SPI-Bus MANUELL initialisieren
  // Hier haben wir die volle Kontrolle über den Bus
  Serial.println("Initialisiere SPI-Bus (HSPI)...");
  customSPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  
  // Optionale SPI-Einstellungen (wenn nötig)
  // customSPI.setFrequency(10000000);  // 10 MHz
  // customSPI.setDataMode(SPI_MODE0);
  // customSPI.setBitOrder(MSBFIRST);
  
  Serial.println("✓ SPI-Bus initialisiert");
  Serial.println();
  
  // Zeige Bus-Informationen
  Serial.println("SPI-Bus Details:");
  Serial.print("  SCK  (Clock):     GPIO ");
  Serial.println(SCK_PIN);
  Serial.print("  MISO (Data In):   GPIO ");
  Serial.println(MISO_PIN);
  Serial.print("  MOSI (Data Out):  GPIO ");
  Serial.println(MOSI_PIN);
  Serial.print("  CS   (Chip Sel):  GPIO ");
  Serial.println(CS_PIN);
  Serial.println();
  
  // MCP23S17 am bestehenden SPI-Bus initialisieren
  // WICHTIG: Den SPI-Bus als Pointer übergeben!
  Serial.println("Verbinde MCP23S17 mit SPI-Bus...");
  if (!mcp.begin(&customSPI)) {
    Serial.println("FEHLER: MCP23S17 nicht gefunden!");
    Serial.println("Überprüfe die Verdrahtung!");
    while(1) {
      delay(1000);
    }
  }
  
  Serial.println("✓ MCP23S17 erfolgreich verbunden");
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
 * ERWEITERTE ANWENDUNGSFÄLLE:
 * 
 * ═══════════════════════════════════════════════════════════
 * 1. MEHRERE GERÄTE AM GLEICHEN SPI-BUS
 * ═══════════════════════════════════════════════════════════
 * 
 * SPIClass mySPI(HSPI);
 * mySPI.begin(14, 12, 13, 15);
 * 
 * // MCP23S17 Port-Expander
 * MCP23S17 mcp1(15, 0);
 * mcp1.begin(&mySPI);
 * 
 * MCP23S17 mcp2(15, 1);  // Andere Adresse
 * mcp2.begin(&mySPI);
 * 
 * // SD-Karte am gleichen Bus
 * SD.begin(4, mySPI);  // CS=4
 * 
 * // Display am gleichen Bus
 * tft.begin(&mySPI);
 * 
 * ═══════════════════════════════════════════════════════════
 * 2. ZWEI GETRENNTE SPI-BUSSE
 * ═══════════════════════════════════════════════════════════
 * 
 * // VSPI für schnelle Geräte
 * SPIClass fastSPI(VSPI);
 * fastSPI.begin(18, 19, 23, 5);
 * fastSPI.setFrequency(40000000);  // 40 MHz
 * 
 * // HSPI für langsamere Geräte
 * SPIClass slowSPI(HSPI);
 * slowSPI.begin(14, 12, 13, 15);
 * slowSPI.setFrequency(1000000);   // 1 MHz
 * 
 * MCP23S17 mcpFast(5, 0);
 * mcpFast.begin(&fastSPI);
 * 
 * MCP23S17 mcpSlow(15, 0);
 * mcpSlow.begin(&slowSPI);
 * 
 * ═══════════════════════════════════════════════════════════
 * 3. SHARED SPI-BUS MIT TRANSAKTIONEN
 * ═══════════════════════════════════════════════════════════
 * 
 * SPIClass sharedSPI(HSPI);
 * sharedSPI.begin(14, 12, 13, 15);
 * 
 * MCP23S17 mcp(15, 0);
 * mcp.begin(&sharedSPI);
 * 
 * // Andere SPI-Operationen
 * sharedSPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
 * digitalWrite(otherCS, LOW);
 * sharedSPI.transfer(0x42);
 * digitalWrite(otherCS, HIGH);
 * sharedSPI.endTransaction();
 * 
 * // MCP funktioniert parallel (verwaltet eigene Transaktionen)
 * mcp.digitalWrite(0, HIGH);
 * 
 * ═══════════════════════════════════════════════════════════
 * 4. DYNAMISCHER BUS-WECHSEL (Fortgeschritten)
 * ═══════════════════════════════════════════════════════════
 * 
 * SPIClass bus1(VSPI);
 * SPIClass bus2(HSPI);
 * 
 * bus1.begin();
 * bus2.begin(14, 12, 13, 15);
 * 
 * MCP23S17 mcp(5, 0);
 * 
 * // Zuerst an Bus1
 * mcp.begin(&bus1);
 * mcp.digitalWrite(0, HIGH);
 * 
 * // Später zu Bus2 wechseln (wenn nötig)
 * mcp.begin(&bus2);
 * mcp.digitalWrite(0, LOW);
 * 
 * ═══════════════════════════════════════════════════════════
 * VORTEILE DER begin(SPIClass*) METHODE:
 * ═══════════════════════════════════════════════════════════
 * 
 * ✅ Volle Kontrolle über SPI-Bus
 * ✅ Mehrere Geräte am gleichen Bus
 * ✅ Getrennte Busse für verschiedene Geschwindigkeiten
 * ✅ Kompatibel mit anderen SPI-Bibliotheken
 * ✅ Flexibler bei komplexen Projekten
 * 
 * ═══════════════════════════════════════════════════════════
 * WICHTIGE HINWEISE:
 * ═══════════════════════════════════════════════════════════
 * 
 * ⚠️  SPI-Bus MUSS vor begin() initialisiert sein!
 * ⚠️  Bus darf während Betrieb nicht beendet werden!
 * ⚠️  Bei mehreren Geräten: Unterschiedliche CS-Pins ODER Adressen
 * ⚠️  MCP verwaltet eigene SPI-Transaktionen automatisch
 * 
 */
