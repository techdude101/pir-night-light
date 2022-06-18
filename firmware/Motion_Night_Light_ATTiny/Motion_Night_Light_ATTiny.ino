/**
 * @file Motion_Night_Light_ATTiny.ino
 * @brief Main application
 * 
 * @author techdude101
 * @bug No known bugs
 */

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

/* 
 * Vref = 1.1V 
 * R1 = 10k, R2 = 1k
 * Voltage divider Vout = (Vbatt / (R1 + R2)) * R2
 * Voltage divider Vout = (3.2 V / (R1 + R2)) * R2 = 0.290 V
 * ADC Value @ 3.2 V = 0.290 V / (1.1 V / 1024) = 269.9 = 270
 */
const int lowBatteryAdcValue = 270; // Approx. 3.2 V

// Global Variables


// Interrupt vectors
ISR(PCINT0_vect) {
  
}

ISR(WDT_vect) {

}

// End of interrupt vectors

// Function definitions
/**
 * @brief Enables pin change interrupt, turns the ADC off to save power and enters sleep mode
 * @return Void.
 */
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

/**
 * @brief Sets the pin as an output and sets the pin HIGH to turn the error LED on
 * @return Void.
 */
void turnErrorLedOn(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}

/**
 * @brief Sets the pin LOW to turn the error LED off and sets the pin as an input
 * @return Void.
 */
void turnErrorLedOff(int pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);
}

/**
 * @brief Sets the pin as an output and sets the pin HIGH to turn the LED on
 * @return Void.
 */
void turnLightOn(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}

/**
 * @brief Sets the pin LOW to turn the LED off and sets the pin as an input
 * @return Void.
 */
void turnLightOff(int pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);
}

/**
 * @brief Sets the pin HIGH to supply power to the voltage divider
 * @return Void.
 */
void ldrDividerPowerOn(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}

/**
 * @brief Sets the pin LOW to disable power to the voltage divider
 * @return Void.
 */
void ldrDividerPowerOff(int pin) {
  digitalWrite(pin, LOW);
}

/**
 * @brief Enables the ADC
 * @return Void.
 */
void enableADC() {
  ADCSRA |= _BV(ADEN);                   // ADC on
}

/**
 * @brief Disables the ADC
 * @return Void.
 */
void disableADC() {
    ADCSRA &= ~_BV(ADEN);               // ADC off    
}

/**
 * @brief Reads the ADC <nSamplesToAverage> times with a delay of <delayBetweenReadsInMs> milliseconds inbetween reads
 * @return Average light level or -1 if nSamplesToAverage is invalid
 */
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
