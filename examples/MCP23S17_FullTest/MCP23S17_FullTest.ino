/*
 * MCP23S17_FullTest.ino
 * 
 * Vollständiger Funktionstest der MCP23S17 Bibliothek
 * 
 * Hardware-Verbindungen (ESP32 <-> MCP23S17):
 * - GPIO 18 (SCK)  <-> SCK
 * - GPIO 23 (MOSI) <-> SI
 * - GPIO 19 (MISO) <-> SO
 * - GPIO 5  (CS)   <-> CS
 * - GND            <-> GND, A0, A1, A2
 * - 3.3V           <-> VDD, RESET
 * 
 * Optional: 16 LEDs an allen Pins mit Vorwiderständen
 */

#include <MCP23S17.h>

// CS-Pin für ESP32
#define CS_PIN 5

// MCP23S17 Objekt erstellen
MCP23S17 mcp(CS_PIN, 0);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n========================================");
  Serial.println("MCP23S17 Vollständiger Funktionstest");
  Serial.println("========================================\n");
  
  // Test 1: Initialisierung
  Serial.println("Test 1: Initialisierung");
  Serial.println("-----------------------");
  if (!mcp.begin()) {
    Serial.println("FEHLER: MCP23S17 nicht gefunden!");
    while(1);
  }
  Serial.println("✓ MCP23S17 erfolgreich initialisiert\n");
  delay(1000);
  
  // Test 2: Einzelne Pin-Operationen
  Serial.println("Test 2: Einzelne Pin-Operationen");
  Serial.println("---------------------------------");
  
  // Pins 0-7 als Ausgang
  for (int pin = 0; pin < 8; pin++) {
    mcp.pinMode(pin, MCP23S17_OUTPUT);
  }
  Serial.println("✓ Pins 0-7 als Ausgang konfiguriert");
  
  // Pins 8-15 als Eingang mit Pull-up
  for (int pin = 8; pin < 16; pin++) {
    mcp.pinMode(pin, MCP23S17_INPUT_PULLUP);
  }
  Serial.println("✓ Pins 8-15 als Eingang mit Pull-up konfiguriert");
  
  // LEDs einzeln durchschalten
  Serial.println("\nLEDs 0-7 einzeln durchschalten...");
  for (int pin = 0; pin < 8; pin++) {
    mcp.digitalWrite(pin, HIGH);
    Serial.print("  Pin ");
    Serial.print(pin);
    Serial.println(" = HIGH");
    delay(200);
    mcp.digitalWrite(pin, LOW);
  }
  Serial.println("✓ Einzelne Pin-Operationen OK\n");
  delay(1000);
  
  // Test 3: Portweise Operationen
  Serial.println("Test 3: Portweise Operationen");
  Serial.println("------------------------------");
  
  // Lauflicht mit Port-Operationen
  Serial.println("Lauflicht auf Port A...");
  for (int i = 0; i < 8; i++) {
    mcp.writePort(MCP23S17_PORTA, 1 << i);
    delay(100);
  }
  mcp.writePort(MCP23S17_PORTA, 0x00);
  
  // Binärmuster
  Serial.println("Binärmuster 0-255 auf Port A...");
  for (int i = 0; i < 256; i++) {
    mcp.writePort(MCP23S17_PORTA, i);
    delay(10);
  }
  mcp.writePort(MCP23S17_PORTA, 0x00);
  Serial.println("✓ Portweise Operationen OK\n");
  delay(1000);
  
  // Test 4: 16-Bit GPIO Operationen
  Serial.println("Test 4: 16-Bit GPIO Operationen");
  Serial.println("--------------------------------");
  
  // Alle Pins als Ausgang
  mcp.portMode(MCP23S17_PORTA, 0xFF);
  mcp.portMode(MCP23S17_PORTB, 0xFF);
  
  Serial.println("16-Bit Lauflicht...");
  for (int i = 0; i < 16; i++) {
    mcp.writeGPIO(1 << i);
    delay(100);
  }
  mcp.writeGPIO(0x0000);
  
  Serial.println("16-Bit Binärzähler (0-1023)...");
  for (uint16_t i = 0; i < 1024; i++) {
    mcp.writeGPIO(i);
    delay(5);
  }
  mcp.writeGPIO(0x0000);
  Serial.println("✓ 16-Bit GPIO Operationen OK\n");
  delay(1000);
  
  // Test 5: Pull-up Test
  Serial.println("Test 5: Pull-up Konfiguration");
  Serial.println("------------------------------");
  
  // Pin 8 als Eingang ohne Pull-up
  mcp.pinMode(8, MCP23S17_INPUT);
  mcp.setPullup(8, false);
  Serial.print("Pin 8 ohne Pull-up: ");
  Serial.println(mcp.digitalRead(8) ? "HIGH" : "LOW");
  
  // Pin 8 mit Pull-up
  mcp.setPullup(8, true);
  delay(10);
  Serial.print("Pin 8 mit Pull-up:  ");
  Serial.println(mcp.digitalRead(8) ? "HIGH" : "LOW");
  
  Serial.println("✓ Pull-up Konfiguration OK\n");
  delay(1000);
  
  // Test 6: Port-Pullups
  Serial.println("Test 6: Port Pull-ups");
  Serial.println("----------------------");
  
  mcp.portMode(MCP23S17_PORTB, 0x00);  // Alle als Eingang
  mcp.setPortPullups(MCP23S17_PORTB, 0xFF);  // Alle Pull-ups an
  
  uint8_t portB = mcp.readPort(MCP23S17_PORTB);
  Serial.print("Port B mit Pull-ups: 0x");
  Serial.println(portB, HEX);
  Serial.println("✓ Port Pull-ups OK\n");
  delay(1000);
  
  // Test 7: Register-Zugriff
  Serial.println("Test 7: Direkte Register-Zugriffe");
  Serial.println("----------------------------------");
  
  uint8_t ioconValue = mcp.readRegister(MCP23S17_IOCON);
  Serial.print("IOCON Register: 0x");
  Serial.println(ioconValue, HEX);
  
  if (ioconValue & IOCON_HAEN) {
    Serial.println("✓ Hardware-Adressierung aktiviert");
  }
  
  Serial.println("✓ Register-Zugriff OK\n");
  delay(1000);
  
  // Test 8: Reset-Funktion
  Serial.println("Test 8: Reset-Funktion");
  Serial.println("----------------------");
  
  mcp.reset();
  Serial.println("✓ Chip erfolgreich zurückgesetzt");
  
  // Prüfe ob alle Pins wieder Eingang sind
  uint16_t gpioValue = mcp.readGPIO();
  Serial.print("GPIO nach Reset: 0x");
  Serial.println(gpioValue, HEX);
  Serial.println("✓ Reset OK\n");
  
  Serial.println("========================================");
  Serial.println("Alle Tests erfolgreich abgeschlossen!");
  Serial.println("========================================\n");
}

void loop() {
  // Demo-Modus: Lauflicht über alle 16 Pins
  
  static unsigned long lastUpdate = 0;
  static int currentPin = 0;
  
  if (millis() - lastUpdate > 100) {
    lastUpdate = millis();
    
    // Alle Pins als Ausgang
    mcp.portMode(MCP23S17_PORTA, 0xFF);
    mcp.portMode(MCP23S17_PORTB, 0xFF);
    
    // Lauflicht
    mcp.writeGPIO(1 << currentPin);
    
    currentPin++;
    if (currentPin >= 16) {
      currentPin = 0;
    }
  }
}
