/*
 * MCP23S17_Blink.ino
 * 
 * Einfaches Beispiel: LED an Pin 0 blinken lassen
 * 
 * Hardware-Verbindungen (ESP32 <-> MCP23S17):
 * - GPIO 18 (SCK)  <-> SCK
 * - GPIO 23 (MOSI) <-> SI
 * - GPIO 19 (MISO) <-> SO
 * - GPIO 5  (CS)   <-> CS
 * - GND            <-> GND, A0, A1, A2 (Adresse = 0)
 * - 3.3V           <-> VDD, RESET
 * 
 * LED an Pin GPA0 (Pin 0) mit Vorwiderstand nach GND
 */

#include <MCP23S17.h>

// CS-Pin für ESP32
#define CS_PIN 5

// MCP23S17 Objekt erstellen (CS-Pin 5, Adresse 0)
MCP23S17 mcp(CS_PIN, 0);

void setup() {
  Serial.begin(115200);
  Serial.println("MCP23S17 Blink Beispiel");
  
  // MCP23S17 initialisieren
  if (!mcp.begin()) {
    Serial.println("Fehler: MCP23S17 nicht gefunden!");
    while(1);  // Anhalten bei Fehler
  }
  
  Serial.println("MCP23S17 erfolgreich initialisiert");
  
  // Pin 0 als Ausgang konfigurieren
  mcp.pinMode(0, MCP23S17_OUTPUT);
  
  Serial.println("LED blinkt an Pin 0...");
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
