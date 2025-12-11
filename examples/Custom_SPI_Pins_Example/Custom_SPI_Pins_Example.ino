/*
 * Custom_SPI_Pins_Example.ino
 * 
 * Demonstriert die Verwendung von benutzerdefinierten SPI-Pins
 * Dies ist besonders nützlich beim ESP32, da die SPI-Pins frei
 * konfigurierbar sind.
 * 
 * Hardware Setup:
 * - MCP23S17 mit benutzerdefinierten SPI-Pins
 * - SCK: GPIO 14
 * - MISO: GPIO 12
 * - MOSI: GPIO 13
 * - CS: GPIO 15
 * - Hardware-Adresse: 0
 * 
 * Funktionen:
 * - Zeigt wie benutzerdefinierte SPI-Pins verwendet werden
 * - Nützlich wenn Standard-SPI-Pins bereits belegt sind
 * - Einfaches Blink-Beispiel
 */

#include <MCP23S17.h>

// Benutzerdefinierte SPI-Pin-Definitionen
#define CUSTOM_SCK   14   // SPI Clock
#define CUSTOM_MISO  12   // SPI Master In Slave Out
#define CUSTOM_MOSI  13   // SPI Master Out Slave In
#define CUSTOM_CS    15   // Chip Select

// MCP23S17 Instanz mit benutzerdefiniertem CS-Pin
MCP23S17 mcp(CUSTOM_CS, 0);

// LED-Pin am MCP23S17
const uint8_t LED_PIN = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("MCP23S17 Custom SPI Pins Example");
  Serial.println("=================================");
  Serial.println();
  
  Serial.println("Benutzerdefinierte SPI-Pin-Konfiguration:");
  Serial.print("SCK (Clock):  GPIO ");
  Serial.println(CUSTOM_SCK);
  Serial.print("MISO (Input): GPIO ");
  Serial.println(CUSTOM_MISO);
  Serial.print("MOSI (Output): GPIO ");
  Serial.println(CUSTOM_MOSI);
  Serial.print("CS (Select):  GPIO ");
  Serial.println(CUSTOM_CS);
  Serial.println();
  
  // MCP23S17 mit benutzerdefinierten Pins initialisieren
  Serial.print("Initialisiere MCP23S17... ");
  if (mcp.begin(CUSTOM_SCK, CUSTOM_MISO, CUSTOM_MOSI)) {
    Serial.println("OK!");
  } else {
    Serial.println("FEHLER!");
    while(1);
  }
  
  // LED-Pin als Ausgang konfigurieren
  mcp.pinMode(LED_PIN, OUTPUT);
  Serial.println("Pin 0 als Ausgang konfiguriert (LED)");
  
  Serial.println();
  Serial.println("Starte Blink-Programm...");
  Serial.println();
}

void loop() {
  // LED einschalten
  mcp.digitalWrite(LED_PIN, HIGH);
  Serial.println("LED: EIN");
  delay(1000);
  
  // LED ausschalten
  mcp.digitalWrite(LED_PIN, LOW);
  Serial.println("LED: AUS");
  delay(1000);
}

/*
 * HINWEISE für ESP32 SPI-Pin-Auswahl:
 * ====================================
 * 
 * Der ESP32 unterstützt mehrere SPI-Busse:
 * - HSPI (Hardware SPI 2)
 * - VSPI (Hardware SPI 3) - Standard
 * 
 * Standard VSPI-Pins:
 * - SCK:  GPIO 18
 * - MISO: GPIO 19
 * - MOSI: GPIO 23
 * - CS:   GPIO 5
 * 
 * Standard HSPI-Pins:
 * - SCK:  GPIO 14
 * - MISO: GPIO 12
 * - MOSI: GPIO 13
 * - CS:   GPIO 15
 * 
 * Die SPI-Pins können auf fast jeden GPIO umgelegt werden,
 * aber es ist empfehlenswert, die Hardware-SPI-Pins zu verwenden
 * für bessere Performance.
 * 
 * Pins die vermieden werden sollten:
 * - GPIO 0: Boot-Mode-Pin
 * - GPIO 1: TX (Serial)
 * - GPIO 2: Boot-Mode-Pin, onboard LED
 * - GPIO 3: RX (Serial)
 * - GPIO 6-11: Flash-Pins (NICHT verwenden!)
 * - GPIO 15: Boot-Mode-Pin (mit Pull-up)
 * - GPIO 34-39: Nur Eingänge, keine internen Pull-ups
 * 
 * Empfohlene GPIO für SPI:
 * - GPIO 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33
 */
