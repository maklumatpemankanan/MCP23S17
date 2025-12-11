/*
 * MCP23S17_PortOperations.ino
 * 
 * Beispiel: Portweise Operationen mit 8 Pins gleichzeitig
 * 
 * Hardware-Verbindungen (ESP32 <-> MCP23S17):
 * - GPIO 18 (SCK)  <-> SCK
 * - GPIO 23 (MOSI) <-> SI
 * - GPIO 19 (MISO) <-> SO
 * - GPIO 5  (CS)   <-> CS
 * - GND            <-> GND, A0, A1, A2
 * - 3.3V           <-> VDD, RESET
 * 
 * 8 LEDs an Port A (GPA0-GPA7) mit Vorwiderständen
 * 8 DIP-Schalter an Port B (GPB0-GPB7) nach GND
 */

#include <MCP23S17.h>

// CS-Pin für ESP32
#define CS_PIN 5

// MCP23S17 Objekt erstellen
MCP23S17 mcp(CS_PIN, 0);

void setup() {
  Serial.begin(115200);
  Serial.println("\nMCP23S17 Port-Operationen Beispiel");
  
  // MCP23S17 initialisieren
  if (!mcp.begin()) {
    Serial.println("Fehler: MCP23S17 nicht gefunden!");
    while(1);
  }
  
  Serial.println("MCP23S17 erfolgreich initialisiert");
  
  // Port A: Alle Pins als Ausgang (0xFF = alle Ausgänge)
  mcp.portMode(MCP23S17_PORTA, 0xFF);
  
  // Port B: Alle Pins als Eingang (0x00 = alle Eingänge)
  mcp.portMode(MCP23S17_PORTB, 0x00);
  
  // Port B: Pull-ups aktivieren (alle 8 Pins)
  mcp.setPortPullups(MCP23S17_PORTB, 0xFF);
  
  Serial.println("Port A = Ausgang (LEDs)");
  Serial.println("Port B = Eingang mit Pull-ups (Schalter)");
  Serial.println("\nSchalter-Zustand wird auf LEDs angezeigt...\n");
}

void loop() {
  // Port B lesen (alle 8 Schalter)
  uint8_t switches = mcp.readPort(MCP23S17_PORTB);
  
  // Wert invertieren (wegen Pull-up: 0=gedrückt, 1=offen)
  switches = ~switches;
  
  // Auf Port A ausgeben (LEDs)
  mcp.writePort(MCP23S17_PORTA, switches);
  
  // Binär-Anzeige im Serial Monitor
  Serial.print("Schalter: ");
  for (int i = 7; i >= 0; i--) {
    Serial.print((switches >> i) & 1);
  }
  Serial.print(" (0x");
  Serial.print(switches, HEX);
  Serial.println(")");
  
  delay(100);
}
