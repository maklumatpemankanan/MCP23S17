/*
 * MCP23S17_Interrupt.ino
 * 
 * Beispiel: Button-Interrupts verarbeiten
 * 
 * Hardware-Verbindungen (ESP32 <-> MCP23S17):
 * - GPIO 18 (SCK)  <-> SCK
 * - GPIO 23 (MOSI) <-> SI
 * - GPIO 19 (MISO) <-> SO
 * - GPIO 5  (CS)   <-> CS
 * - GPIO 4  (INT)  <-> INTA
 * - GND            <-> GND, A0, A1, A2
 * - 3.3V           <-> VDD, RESET
 * 
 * Taster an Pin GPA0 (Pin 0) nach GND
 * LED an Pin GPB0 (Pin 8) mit Vorwiderstand nach GND
 */

#include <MCP23S17.h>

// Pin-Definitionen für ESP32
#define CS_PIN  5
#define INT_PIN 4

// MCP23S17 Objekt erstellen
MCP23S17 mcp(CS_PIN, 0);

// Interrupt-Flag (volatile weil es in ISR geändert wird)
volatile bool interruptOccurred = false;

// Interrupt Service Routine
void IRAM_ATTR handleInterrupt() {
  interruptOccurred = true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("\nMCP23S17 Interrupt Beispiel");
  
  // MCP23S17 initialisieren
  if (!mcp.begin()) {
    Serial.println("Fehler: MCP23S17 nicht gefunden!");
    while(1);
  }
  
  Serial.println("MCP23S17 erfolgreich initialisiert");
  
  // Pin 0 als Eingang mit Pull-up für Button
  mcp.pinMode(0, MCP23S17_INPUT_PULLUP);
  
  // Pin 8 als Ausgang für LED
  mcp.pinMode(8, MCP23S17_OUTPUT);
  mcp.digitalWrite(8, LOW);
  
  // Interrupt für Pin 0 aktivieren (bei Änderung)
  mcp.enableInterrupt(0, MCP23S17_INT_CHANGE);
  
  // Interrupt-Polarität auf Active-Low setzen (Standard)
  mcp.setInterruptPolarity(false);
  
  // ESP32 Interrupt-Pin konfigurieren
  pinMode(INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), handleInterrupt, FALLING);
  
  // Vorhandene Interrupts löschen
  mcp.clearInterrupts();
  
  Serial.println("Bereit! Drücke den Button an Pin 0");
}

void loop() {
  // Prüfe ob Interrupt aufgetreten ist
  if (interruptOccurred) {
    interruptOccurred = false;
    
    // Interrupt-Flags lesen (Port A)
    uint8_t flags = mcp.getInterruptFlags(MCP23S17_PORTA);
    
    // Capture-Register lesen (GPIO-Zustand beim Interrupt)
    uint8_t capture = mcp.getInterruptCapture(MCP23S17_PORTA);
    
    Serial.print("Interrupt! Flags: 0x");
    Serial.print(flags, HEX);
    Serial.print(", Capture: 0x");
    Serial.println(capture, HEX);
    
    // Pin 0 Zustand prüfen (0 = gedrückt wegen Pull-up)
    if ((capture & 0x01) == 0) {
      Serial.println("Button gedrückt - LED AN");
      mcp.digitalWrite(8, HIGH);
    } else {
      Serial.println("Button losgelassen - LED AUS");
      mcp.digitalWrite(8, LOW);
    }
    
    // Interrupts löschen (wichtig!)
    mcp.clearInterrupts();
  }
  
  // Kleine Verzögerung um Entprellung zu unterstützen
  delay(50);
}
