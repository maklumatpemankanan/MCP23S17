/*
 * Interrupt_Example.ino
 * 
 * Demonstriert die Interrupt-Funktionalität des MCP23S17
 * 
 * Hardware Setup:
 * - MCP23S17 verbunden über SPI
 * - CS-Pin an GPIO 5
 * - INT-Pin des MCP23S17 an ESP32 GPIO 4
 * - Taster an Pin 0 des MCP23S17
 * - LED an Pin 8 des MCP23S17
 * - Hardware-Adresse: 0
 * 
 * Funktionen:
 * - Interrupt bei Tastendruck
 * - LED wird bei Interrupt umgeschaltet
 * - Interrupt-Flags und gecaptureten Wert anzeigen
 */

#include <MCP23S17.h>

// MCP23S17 Instanz
MCP23S17 mcp(5, 0);

// Pin-Definitionen
const uint8_t MCP_BUTTON_PIN = 0;   // Taster am MCP23S17
const uint8_t MCP_LED_PIN = 8;      // LED am MCP23S17
const uint8_t ESP32_INT_PIN = 4;    // INT-Pin vom MCP23S17 zum ESP32

// Volatile Variable für Interrupt-Flag
volatile bool interruptOccurred = false;

// LED-Status
bool ledState = false;

/*
 * Interrupt Service Routine (ISR)
 * Wird aufgerufen, wenn der MCP23S17 einen Interrupt auslöst
 */
void IRAM_ATTR handleInterrupt() {
  interruptOccurred = true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("MCP23S17 Interrupt Example");
  Serial.println("===========================");
  
  // MCP23S17 initialisieren
  if (!mcp.begin()) {
    Serial.println("Fehler bei Initialisierung!");
    while(1);
  }
  Serial.println("MCP23S17 initialisiert!");
  
  // Taster-Pin als Eingang mit Pull-up
  mcp.pinMode(MCP_BUTTON_PIN, INPUT_PULLUP);
  Serial.println("Pin 0: Eingang mit Pull-up (Taster)");
  
  // LED-Pin als Ausgang
  mcp.pinMode(MCP_LED_PIN, OUTPUT);
  mcp.digitalWrite(MCP_LED_PIN, LOW);
  Serial.println("Pin 8: Ausgang (LED)");
  
  // Interrupt-Konfiguration für MCP23S17
  // Interrupt bei Änderung, Active-Low, Push-Pull
  mcp.configureInterrupts(
    false,  // mirrored: INTA und INTB getrennt
    false,  // open_drain: false = Push-Pull
    false   // polarity: false = Active-Low
  );
  Serial.println("Interrupt-Konfiguration: Active-Low, Push-Pull");
  
  // Interrupt für Taster-Pin aktivieren
  // MCP23S17_INT_CHANGE: Interrupt bei jeder Änderung (gedrückt und losgelassen)
  mcp.enableInterrupt(MCP_BUTTON_PIN, MCP23S17_INT_CHANGE);
  Serial.println("Interrupt für Pin 0 aktiviert (INT_CHANGE)");
  
  // ESP32 Interrupt-Pin konfigurieren
  pinMode(ESP32_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ESP32_INT_PIN), handleInterrupt, FALLING);
  Serial.println("ESP32 Interrupt-Pin konfiguriert (GPIO 4, FALLING)");
  
  Serial.println("\nBereit! Drücke den Taster an Pin 0...");
  Serial.println();
  
  // Interrupt-Flags einmal lesen, um sie zu löschen
  mcp.getInterruptCapture();
}

void loop() {
  // Prüfen ob ein Interrupt aufgetreten ist
  if (interruptOccurred) {
    // Flag zurücksetzen
    interruptOccurred = false;
    
    Serial.println("=== INTERRUPT AUSGELÖST ===");
    
    // Interrupt-Flags lesen (welche Pins haben Interrupt ausgelöst)
    uint16_t flags = mcp.getInterruptFlags();
    Serial.print("Interrupt Flags: 0x");
    Serial.println(flags, HEX);
    
    // Gecaptureten Wert lesen (Zustand der Pins beim Interrupt)
    uint16_t capture = mcp.getInterruptCapture();
    Serial.print("Captured Value: 0x");
    Serial.println(capture, HEX);
    
    // Prüfen ob Pin 0 den Interrupt ausgelöst hat
    if (flags & (1 << MCP_BUTTON_PIN)) {
      Serial.println("Taster-Pin hat Interrupt ausgelöst!");
      
      // Pin-Status beim Interrupt
      bool buttonPressed = !(capture & (1 << MCP_BUTTON_PIN));  // LOW = gedrückt
      
      if (buttonPressed) {
        Serial.println("Taster wurde GEDRÜCKT");
        
        // LED umschalten
        ledState = !ledState;
        mcp.digitalWrite(MCP_LED_PIN, ledState);
        
        Serial.print("LED Status: ");
        Serial.println(ledState ? "EIN" : "AUS");
      } else {
        Serial.println("Taster wurde LOSGELASSEN");
      }
    }
    
    Serial.println();
    
    // Kleine Verzögerung für Entprellung
    delay(50);
  }
  
  // Hauptprogramm kann hier andere Aufgaben erledigen
  // Der Interrupt arbeitet asynchron im Hintergrund
  
  delay(10);
}
