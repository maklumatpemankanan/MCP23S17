/*
 * Basic_GPIO_Example.ino
 * 
 * Einfaches Beispiel für die MCP23S17 Bibliothek
 * Demonstriert grundlegende GPIO-Funktionen
 * 
 * Hardware Setup:
 * - MCP23S17 verbunden über SPI
 * - ESP32 Standard-SPI-Pins: MOSI=23, MISO=19, SCK=18
 * - CS-Pin an GPIO 5
 * - Hardware-Adresse: A0=A1=A2=0 (Adresse 0)
 * 
 * Funktionen:
 * - LED an Pin 0 blinken lassen
 * - Taster an Pin 8 mit Pull-up lesen
 * - Status auf Serial Monitor ausgeben
 */

#include <MCP23S17.h>

// MCP23S17 Instanz erstellen
// Parameter: CS-Pin, Hardware-Adresse (0-7)
MCP23S17 mcp(5, 0);

// Pin-Definitionen
const uint8_t LED_PIN = 0;      // LED an Pin 0 (Port A)
const uint8_t BUTTON_PIN = 8;   // Taster an Pin 8 (Port B)

void setup() {
  // Serial Monitor starten
  Serial.begin(115200);
  Serial.println("MCP23S17 Basic GPIO Example");
  Serial.println("============================");
  
  // MCP23S17 initialisieren
  if (mcp.begin()) {
    Serial.println("MCP23S17 erfolgreich initialisiert!");
  } else {
    Serial.println("Fehler bei MCP23S17 Initialisierung!");
    while(1);  // Anhalten bei Fehler
  }
  
  // LED-Pin als Ausgang konfigurieren
  mcp.pinMode(LED_PIN, OUTPUT);
  Serial.println("Pin 0 als Ausgang konfiguriert (LED)");
  
  // Button-Pin als Eingang mit Pull-up konfigurieren
  mcp.pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println("Pin 8 als Eingang mit Pull-up konfiguriert (Taster)");
  
  Serial.println("\nBereit! LED blinkt, Taster-Status wird angezeigt.");
  Serial.println();
}

void loop() {
  // LED einschalten
  mcp.digitalWrite(LED_PIN, HIGH);
  Serial.print("LED: EIN  - ");
  
  // Taster-Status lesen und anzeigen
  uint8_t buttonState = mcp.digitalRead(BUTTON_PIN);
  if (buttonState == LOW) {  // Pull-up: LOW = gedrückt
    Serial.println("Taster: GEDRÜCKT");
  } else {
    Serial.println("Taster: NICHT gedrückt");
  }
  
  delay(500);
  
  // LED ausschalten
  mcp.digitalWrite(LED_PIN, LOW);
  Serial.print("LED: AUS - ");
  
  // Taster-Status lesen und anzeigen
  buttonState = mcp.digitalRead(BUTTON_PIN);
  if (buttonState == LOW) {
    Serial.println("Taster: GEDRÜCKT");
  } else {
    Serial.println("Taster: NICHT gedrückt");
  }
  
  delay(500);
}
