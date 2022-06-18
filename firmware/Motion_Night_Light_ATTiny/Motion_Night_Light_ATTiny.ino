#include <avr/sleep.h>
#include <avr/interrupt.h>

// Constants
const int ldrSensorPin = 1; // PB2 / ADC1
const int ldrDividerPowerPin = 1; // PB1 - To enable power to be turned on / off to the LDR voltage divider. 1M >= when dark, <=10k when bright
const int pirSensorPin = 3; // PB3
const int ledControlPin = 4; // PB4
const int lightOnTimeInMilliseconds = (1000 * 30);
const int errorLedPin = 0; // PB0 - Red LED (direct drive)

const int lowLightThreshold = 150;

// Global Variables


// Interrupt vectors
ISR(PCINT0_vect) {
  
}

ISR(WDT_vect) {

}

// End of interrupt vectors

// Function definitions
void enter_sleep() {
  // Enable interrupts before entering sleep mode
  GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
  PCMSK |= _BV(PCINT3);                   // Use PB3 as interrupt pin
  
  sleep_enable();
  // ADC off ## IMPORTANT ## Saves ~280uA          
  disableADC();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  sei();                                  // Enable interrupts
  sleep_cpu();
  
  // Wake up from sleep here
  delay(100); // Give the MCU some time to wakeup
  GIMSK &= ~_BV(PCIE);                     // Disable Pin Change Interrupts
}

void turnErrorLedOn(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}

void turnErrorLedOff(int pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);
}

void turnLightOn(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}

void turnLightOff(int pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);
}

void ldrDividerPowerOn(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}

void ldrDividerPowerOff(int pin) {
  digitalWrite(pin, LOW);
}

void enableADC() {
  ADCSRA |= _BV(ADEN);                   // ADC on
}

void disableADC() {
    ADCSRA &= ~_BV(ADEN);               // ADC off    
}

int getAverageLightLevel(int pin, int nSamplesToAverage, int delayBetweenReadsInMs) {
  enableADC();
  int average = 0;
  
  for (int i = 0; i < nSamplesToAverage; i++) {
    int adcValue = analogRead(pin);
    delay(delayBetweenReadsInMs);
    average += adcValue;
  }
  
  if (average == 0) {
    return 0;
  }
  
  // Prevent divide by zero and negative number of samples to average
  if (nSamplesToAverage < 1) {
    return -1;
  }
  
  average = average / nSamplesToAverage;
  
  return average;
}

void setup() {
  // Flash the LEDs on startup to check hardware
  turnErrorLedOn(errorLedPin);
  delay(100);
  turnErrorLedOff(errorLedPin);
  
  turnLightOn(ledControlPin);
  delay(1000);
  turnLightOff(ledControlPin);
  
  // Disable the watchdog timer
  WDTCR = 0x00;
}

void loop() {
  /*
   * 1. Sleep to save power
   * 2. Wakeup by PIR motion sensor
   * 3. Check if it's dark or not
   * 4. IF it's dark THEN turn the light on for a period of time
   * 5. Turn the light off
   */
  // 1. Sleep
  turnLightOff(ledControlPin);
  turnErrorLedOff(errorLedPin);
  enter_sleep();
  // 2. Wakeup by PIR motion sensor
  
  // 3. Check if it's dark or not
  // Initialize light level to a value greater than the low light level threshold to prevent turning the light on if the ADC reading fails or returns zero
  int lightLevel = lowLightThreshold * 2; 
  ldrDividerPowerOn(ldrDividerPowerPin);
  enableADC();
  delay(50); // Allow voltage divider voltage to rise and stabilize

  lightLevel = getAverageLightLevel(ldrSensorPin, 5, 10);

  disableADC();
  ldrDividerPowerOff(ldrDividerPowerPin);

  // Debugging
  if (lightLevel >= lowLightThreshold) {
    turnErrorLedOn(errorLedPin);
  } else {
    turnErrorLedOff(errorLedPin);
  }
  
  // 4. Turn the light on for a period of time
  if ((lightLevel != -1) && (lightLevel < lowLightThreshold)) {
    turnLightOn(ledControlPin);
    delay(lightOnTimeInMilliseconds);
    
    // TODO: Turn the light off only if motion is no longer detected
    // 5. Turn the light off
    turnLightOff(ledControlPin);
  }
}
