/**
 * temperatureRelay_wdt.ino
 * Bill Case 30Jun14
  *
 * Sketch for activating a solid state relay based on the temperature limit
 * measured by a LM34, or for a timeframe starting with a pushbutton interrupt.
 * A status led will blink each time the WDT interrupt wakes the MCU.
 *
 * Binary sketch size: 1,994 bytes (of a 8,192 byte maximum)
 * MCU = ATTiny85
 * Solid State Relay = Crouzet M-OAC5AH
 * Temp sensor = LM34 (provides 10mV per degree Fahrenheit)
 * Vcc = 5v
 * Total sleep current:
 *    without LM34  = 337.0 uA
 *    with    LM34  = 496.0 uA
 *    driving relay = 14.8mA
 */
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#define AD3INPUT    3  // Pin 2 (PB3) - use A/D #3 to read LM34 output
#define RELAYPIN    4  // Pin 3 (PB4) - activate relay
#define ACTIVITYLED 0  // Pin 5 (PB0) - activity led
#define PUSHBTN     2  // Pin 7 (PB2) - pushbutton on INT0

#define LED_DELAY          200      // msec to flash on
#define TIMER_MAX_COUNT8 (30*60)/8  // 30min*60sec/WDT_Period_8sec
#define TIMER_MAX_COUNT4 (30*60)/4  // 30min*60sec/WDT_Period_4sec

volatile boolean timerActive  = false; // timer to run relayOn
volatile boolean temperActive = false; // temperature mark hit to run relayOn
volatile long timerCount      = 0;     // number of 4sec ticks

/**
 * entry point for all Arduino sketches
 * Define I/O pins, use internal pullup resistor on pushbutton
 * pin so we can activate this interrupt when grounded.
 * Setup the WatchDog Timer for interrupt only every 4 sec.
 */
void setup()
{
  // is this reset caused by WDT
  if(MCUSR & _BV(WDRF)) {
    MCUSR &= ~_BV(WDRF);               // clear WDT's reset flag
    WDTCR |= ( _BV(WDCE) | _BV(WDE) ); // enable Watchdog change bit
    WDTCR = 0x00;                      // disable WDT
  }
  
  cli();
  delay(2000);                  //Allow voltage to settle on power-up
  analogReference(DEFAULT);     // Vcc (5v) A/D reference
  pinMode(RELAYPIN,    OUTPUT); // define pin directon
  pinMode(ACTIVITYLED, OUTPUT); 
  pinMode(PUSHBTN,     INPUT); // interrupt pushbutton

  digitalWrite(RELAYPIN, LOW); // ensure relay is off
  digitalWrite(PUSHBTN, HIGH); // set internal pullup
  timerActive  = false;        //reset
  temperActive = false;        //reset
  timerCount   = 0;            //reset

  // WDT setup
  MCUSR &= ~(_BV(WDRF)); // clear reset flag

  /* In order to change WDE or the prescaler, we need to
   * set WDCE (This will allow updates for 4 clock cycles).
   */ 
  WDTCR |= _BV(WDCE) | _BV(WDE); 
//  WDTCR  = _BV(WDP0) | _BV(WDP3); // WDT prescaler for 8.0 seconds 
  WDTCR  = _BV(WDP3);             // WDT prescaler for 4.0 seconds 
  WDTCR |= _BV(WDIE);             // enable only the WDT interrupt w/o reset

} // end of setup

/**
 * loop - Arduino endless loop
 * If active, the timer overrides the temperature check.
 * Temperature check is performed when timer period is over
 * or if timer is not active.
 * Go back to sleep until the next interrupt. 
 */
void loop()
{
  //  flashLed(ACTIVITYLED);  // debug use
  delay(200);

    if(timerActive) {
      if(timerCount > TIMER_MAX_COUNT4) {
        timerCount = 0; //reset
        timerActive = false;
        checkTemp();
      }
      else
        timerCount++;
    }
    else
      checkTemp();

  enterSleep();
} // end of loop

/**
 * Enable the pushbutton interrupt (INT0) and go to sleep.
 * WDT or Pushbutton will wake MCU up
 */
void enterSleep(void)
{
  //  set_sleep_mode(SLEEP_MODE_PWR_SAVE); // still allows loop() to run
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);     // lowest power consumption

  sleep_enable();

  MCUCR &= ~(_BV(ISC01) | _BV(ISC00)); //INT0 on low level
  GIMSK |= _BV(INT0);                  //enable INT0

  sei();        // enable any interrupts
  sleep_mode(); // going to sleep now

                   // program continues after either WDT timeout or int0 interrupt
  sleep_disable(); // First thing to do is disable sleep. */
  cli();  

  power_all_enable();     // active all chip functions
  flashLed(ACTIVITYLED);  // let world know i'm awake
}

/**
 * Flash the LED 
 */
void flashLed(int ledNum) {
  digitalWrite(ledNum, HIGH);
  delay(LED_DELAY);
  digitalWrite(ledNum, LOW);
}

/**
 * Activate the relay if temperature is above 100 degrees F.
 * Deactivate the relay when temperature is 98.5. This ensures
 * some temperature hystessis so relay doesn't cycle on/off every
 * time the WDT wakes up
 */
void checkTemp() { 
  float outsideTemp = 0.0;

  outsideTemp = readLM34Temperature(AD3INPUT);
  if( outsideTemp > 100.0 ) {
    temperActive = true;
    digitalWrite(RELAYPIN, HIGH); // turn on relay
  }
  else {
    if ((temperActive) && outsideTemp <= 98.5) {
      digitalWrite(RELAYPIN, LOW);  // turn off relay
      temperActive = false;
    }
    else
      digitalWrite(RELAYPIN, LOW);  // turn off relay
  }
}

/**
 * Read the output of the LM34 and return the current temerature.
 * LM34 returns 10mV for each degree F. 
 * Thermal response time for LM34 is roughly 3 minutes, so we'll
 * get to the correct temp eventually. 
 */
float readLM34Temperature( int inputPin ) {
  float finalTemp  = 0;
  int intermVal    = 0;
  
  intermVal = analogRead(inputPin);
  finalTemp = (5.0 * intermVal * 100.0)/1024; 

  return finalTemp;
}

/**
 * WatchDog Timer Interrupt Service Routine.
 * Not required to do much, but MCU is now awake and will
 * continue the program counter from next instruction following
 * sleep_mode() in the enterSleep() routine.
 */
ISR(WDT_vect)
{
  MCUSR &= ~( _BV(WDRF)); // clear so next interrupt will not reset the MCU
  // nothing else to do
  // flashLed(RELAYPIN); // debug indicator
}

/**
 * Interrupt 0 Service Routine
 * Pushbutton has been pushed and a low value was detected on
 * the interrupt pin. User wants to turn the relay on for a time
 * period (~30mins). MCU is now awake and will continue the
 * program counter from next instruction following sleep_mode()
 * in the enterSleep() routine.
 */
ISR(INT0_vect)
{
  timerActive = true;
  digitalWrite(RELAYPIN, HIGH); // turn on relay
  GIMSK = 0; //disable external interrupts (only need one to wake up)
}

