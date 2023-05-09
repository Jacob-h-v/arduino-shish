#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>

//  Variables
const int PulseWire = 0;       // PulseSensor PURPLE WIRE connected to ANALOG PIN 0
const int LED = LED_BUILTIN;          // The on-board Arduino LED, close to PIN 13.
int Threshold = 510;           // Determine which Signal to "count as a beat" and which to ignore.   
int rawSignal;                       

PulseSensorPlayground pulseSensor;  // Creates an instance of the PulseSensorPlayground object called "pulseSensor"

void setup() {        
  Serial.begin(9600);     
   
  // Configure the PulseSensor object, by assigning our variables to it. 
  pulseSensor.analogInput(PulseWire);
  pulseSensor.blinkOnPulse(LED);       //auto-magically blink Arduino's LED with heartbeat.
  pulseSensor.setThreshold(Threshold);

  // Double-check the "pulseSensor" object was created and "began" seeing a signal. 
   if (pulseSensor.begin()) {
    //Serial.println("We created a pulseSensor Object !");  //This prints one time at Arduino power-up,  or on Arduino reset.   
  }
}

void loop() {
  // Read the raw signal from pulse sensor
  rawSignal = analogRead(0);
  //if (pulseSensor.sawStartOfBeat()) {            // Constantly test to see if "a beat happened".
    int myBPM = pulseSensor.getBeatsPerMinute();  // Calls function on our pulseSensor object that returns BPM as an "int".
    Serial.print("<START>");
    Serial.print(myBPM);
    Serial.print(",");
    Serial.print(rawSignal - Threshold);
    Serial.println("<END>");
  
   
  delay(100);
}
