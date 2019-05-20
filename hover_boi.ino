//
// Hover Boi
//
// Note: this code for Arduino UNO, ATMEGA328P-PU
//

// Receiver (RX) data structure:
typedef struct {
    unsigned long timer;
    volatile int value;
    byte last_state;
} RX_t;

// ESC data structure:
typedef struct {
    unsigned long timer;
    int value;
    int done;
} ESC_t;

// Usual "normal" RX channel order is: Aileron, Elevator, Throttle, Rudder
// Use pins 8, 9, 10, and 11, for channels 1 through 4 respectively
//
//NOTE: RX Channels used are:
//  RX Channel  Description
//  ----------  -----------------------------------------------------------------------------
//      CH1      this is the HoverBoi left/right turn servo. Uses Aileron (right-side) stick. (not used in this code)
//      CH2      this is the HoverBoi hover fan throttle. Uses P2 potentiometer (HOV. CH3)
//      CH3      this is the HoverBoi forward throttle. Uses throttle stick.
//      CH4      unused
//
typedef enum {CH1, CH2, CH3, CH4, CH5, CH6, CH7, CH8} rx_channels_t;

// Global vars
unsigned long rx_interrupt_time; // variable to hold time that an RX interrupt occurred.
unsigned long rx_loop_diff; // time diff between rx_interrupt time and now. Use to tell if we lost RX signal
unsigned long loop_timer; // main loop timer value. We'll time the loop to be approximately 4ms (250Hz).
unsigned long esc_loop_timer; // ESC's PWM loop timer
int esc_loop_done;
RX_t rx[3]; // we're only using 3 RX channels
int rx_lost; // flag sets if we loose RX signal (will be set to 0 or 1)
ESC_t esc[4]; // We will have 4 ESCs, two for hover motors, two for thrust motors.
int start;
int f_thr; // forward throttle
int h_thr; // hover throttle


//
// Arduino "setup" method.
// This is called once after reset.
//
void setup()
{
    // Note: Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs

    //pinMode(LED_BUILTIN, OUTPUT);
    // Note: not using "pinMode" method above, because setting ATMega registers directly uses less code after compile
    DDRD |= B11110000;  //Configure digital pins 4, 5, 6 and 7 as output. One for each ESC and motor.
    DDRB |= B00110000;  //Configure digital pins 12 and 13 as output.


    //Use the led on the Arduino for startup indication.
    digitalWrite(LED_BUILTIN, HIGH);  //Turn on the warning led.

    //Wait 5 seconds before and PWM ESCs before continuing.
    for(int i = 0; i < 1250 ; i++)
    {
        PORTD |= B11110000;       //Set digital pins 4, 5, 6 and 7 high.
        delayMicroseconds(1000);  //Wait 1000us.
        PORTD &= B00000000;       //Set digital pins 4, 5, 6 and 7 low.
        delayMicroseconds(3000);  //Wait 3000us.
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));                 //Change the led status.
    }
    digitalWrite(LED_BUILTIN, HIGH);

    // Make RX pins generate interrupts:
    // Enable interrupts on ATMEGA328P-PU pints 8, 9, 10, and 11
    // PCICR, PCIE0, PCMSK0, and PCINT[0..3] are explained in the ATMEGA documentation:
    // ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061A.pdf
    PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
    PCMSK0 |= (1 << PCINT0);  // set PCINT0 (digital input 8) to trigger an interrupt on state change
    PCMSK0 |= (1 << PCINT1);  // set PCINT1 (digital input 9)to trigger an interrupt on state change
    PCMSK0 |= (1 << PCINT2);  // set PCINT2 (digital input 10)to trigger an interrupt on state change
    // We're not using RX CH4    PCMSK0 |= (1 << PCINT3);  // set PCINT3 (digital input 11)to trigger an interrupt on state change

    // Wait until the receiver is active and the throttle is set to the lowest position.
    // Flash the LED if and while we're waiting for throttle safe position.
    while(rx[CH3].value < 990 || rx[CH3].value > 1020)
    {
      //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
      start ++;                                                               //While waiting increment start whith every loop.
      PORTD |= B11110000;                                                     //Set digital pins 4, 5, 6 and 7 high.
      delayMicroseconds(1000);                                                //Wait 1000us.
      PORTD &= B00001111;                                                     //Set digital pins 4, 5, 6 and 7 low.
      delay(3);                                                               //Wait 3 milliseconds before the next loop.
      if(start == 125){                                                       //Every 125 loops (500ms).
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));                 //Change the led status.
        start = 0;                                                            //Start again at 0.
      }
    }
    start = 0;                                                                //Set start back to 0.

    //
    // NOTE: Start serial port for debugging to serial terminal.
    //       Once debugging is done, or not needed, comment out Serial stuffs...
    Serial.begin(115200);      //Start the serial connetion @ 57600bps

    // get time for first loop
    loop_timer = micros();  // get microseconds

    // When setup is done, turn off the led.
    digitalWrite(LED_BUILTIN, LOW);  //Turn off the warning led.
} // end of setup()


//
// Arduino "loop" method.
// This is called after setup() is done
// and continuously loops.
//
void loop()
{
    // Serial debug prints
//     Serial.print(F("RX: "));
//     Serial.print(rx[CH1].value);
//     Serial.print(F("; "));
//     Serial.print(rx[CH2].value);
//     Serial.print(F("; "));
//     Serial.print(rx[CH3].value);
//     Serial.print(F("; "));
// //    Serial.print(rx[CH4].value);
// //    Serial.print(F("; "));
//     Serial.println(F(""));

    //
    // Set ESC output values to the RX input value
    // Eventually, the throttle channel will get adjusted by an outside
    // signal, such as a sonic distance measurement, or some such.
    //
    // ESC Descriptions:
    // ----------------
    //     Use ESC Channel 1 and ESC Channel 2 for thrust motors
    //     Use ESC Channel 3 and ESC Channel 4 for hover motors
    //


    // If RX signal is not updated after X seconds, consider it "lost".
    rx_loop_diff = micros() - rx_interrupt_time;
    if(rx_loop_diff > 10000000 && rx_loop_diff < 20000000 ) {
        digitalWrite(13, HIGH);
        rx_lost = 1;
        Serial.print(F("rx_lost: "));
        Serial.println(rx_loop_diff);
    } else {
        if(rx_lost) {
            if(!( rx[CH3].value < 980 || rx[CH3].value > 1020 ||
                  rx[CH2].value < 980 || rx[CH2].value > 1020)  ) {
                   digitalWrite(13, LOW);
                   rx_lost = 0;
            }
        }
    }



    // Store received throttle values. If RX is lost, then set to lowest (1000).
    f_thr = rx_lost ? 1000 : rx[CH3].value;  // forward throttle
    h_thr = rx_lost ? 1000 : rx[CH2].value;  // hover throttle

    // Set ESC values. If RX value is less than 1000, then set to 1000.
    esc[CH1].value = (f_thr < 1000) ? 1000 : f_thr; // ESC CH1 and CH2 get same value
    esc[CH2].value = (f_thr < 1000) ? 1000 : f_thr;
    esc[CH3].value = (h_thr < 1000) ? 1000 : h_thr; // ESC CH3 and CH4 get same value
    esc[CH4].value = (h_thr < 1000) ? 1000 : h_thr;


    //
    // All the information for controlling the motor's is available.
    // The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
    //
    if(micros() - loop_timer > 4050) digitalWrite(13, HIGH);  //Turn on the LED if the loop time exceeds 4050us.
    while(micros() - loop_timer < 4000);  // We wait until 4000us are passed.

    loop_timer = micros();  // Get time for next loop

    //
    // Last thing: pulse ESCs
    //
    // First, set digital output pins 4, 5, 6 and 7 high.
    PORTD |= B11110000;  // <- PORTD = PORTD | B11110000;

    esc[CH1].timer = esc[CH1].value + loop_timer;  // Calculate the time to digital port 4 is set low.
    esc[CH2].timer = esc[CH2].value + loop_timer;  // Calculate the time to digital port 5 is set low.
    esc[CH3].timer = esc[CH3].value + loop_timer;  // Calculate the time to digital port 6 is set low.
    esc[CH4].timer = esc[CH4].value + loop_timer;  // Calculate the time to digital port 7 is set low.

    delayMicroseconds(1000); // dummy delay here. remove and replace with code to check for distance sensor.

    // esc CHx done flags
    esc[CH1].done = 0;
    esc[CH2].done = 0;
    esc[CH3].done = 0;
    esc[CH4].done = 0;

    // ESC loop:
    esc_loop_done = 0;  // esc loop done flag
    while(!esc_loop_done) // stay in this loop until "done"
    {
        esc_loop_timer = micros();  // Get the current time.

        if(esc[CH1].timer <= esc_loop_timer) {
            PORTD &= B11101111;  // When the delay time is expired, digital port 4 is set low.
            esc[CH1].done = 1;
        }

        if(esc[CH2].timer <= esc_loop_timer) {
            PORTD &= B11011111;  // When the delay time is expired, digital port 5 is set low.
            esc[CH2].done = 1;
        }

        if(esc[CH3].timer <= esc_loop_timer) {
            PORTD &= B10111111;  // When the delay time is expired, digital port 6 is set low.
            esc[CH3].done = 1;
        }

        if(esc[CH4].timer <= esc_loop_timer) {
            PORTD &= B01111111;  // When the delay time is expired, digital port 7 is set low.
            esc[CH4].done = 1;
        }

        // while loop is "done" when all PWM channels are done.
        esc_loop_done = esc[CH1].done && esc[CH2].done && esc[CH3].done && esc[CH4].done;
    } // end of while(!esc_loop_done)
} // end of loop()



//
// Interrupt Service Routine (ISR)
// This routine is called every time a bit in the PCINT0_vect changes
// For this code, input pins 8, 9, 10 or 11 are tied to this ISR vector in setup()
ISR(PCINT0_vect)
{
    // get the current time in microseconds
    rx_interrupt_time = micros();

    // then check each pin (RX channel) for change

    // RX Channel 1 ============================================================
    if( PINB & B00000001)  // Is input 8 high?
    { 
        if(rx[CH1].last_state == 0)  // Input changed from 0 to 1
        {
          rx[CH1].last_state = 1;  //Remember current input state
          rx[CH1].timer = rx_interrupt_time;  //Set timer to rx_interrupt_time
        }
    }
    else if(rx[CH1].last_state == 1)  // Input is not high and changed from 1 to 0
    {
        rx[CH1].last_state = 0;  // Remember current input state
        rx[CH1].value = rx_interrupt_time - rx[CH1].timer;  // Channel value is rx_interrupt_time - timer
    }

    // RX Channel 2 ============================================================
    if( PINB & B00000010 )  // Is input 9 high?
    {
        if(rx[CH2].last_state == 0)  // Input changed from 0 to 1
        {
          rx[CH2].last_state = 1;  // Remember current input state
          rx[CH2].timer = rx_interrupt_time;  // Set timer to rx_interrupt_time
        }
    }
    else if(rx[CH2].last_state == 1)  // Input is not high and changed from 1 to 0
    {
        rx[CH2].last_state = 0;  // Remember current input state
        rx[CH2].value = rx_interrupt_time - rx[CH2].timer;  // Channel value is rx_interrupt_time - timer
    }

    // RX Channel 3 ============================================================
    if( PINB & B00000100 )  // Is input 10 high?
    {
        if(rx[CH3].last_state == 0)  // Input changed from 0 to 1
        {
          rx[CH3].last_state = 1;  // Remember current input state
          rx[CH3].timer = rx_interrupt_time;  // Set timer to rx_interrupt_time
        }
    }
    else if(rx[CH3].last_state == 1)  // Input is not high and changed from 1 to 0
    {
        rx[CH3].last_state = 0;  // Remember current input state
        rx[CH3].value = rx_interrupt_time - rx[CH3].timer;  // Channel value is rx_interrupt_time - timer
    }

// HoverBoi not using Channel 4
//    // RX Channel 4 ============================================================
//    if( PINB & B00001000 )  // Is input 11 high?
//    {
//        if(rx[CH4].last_state == 0)  // Input changed from 0 to 1
//        {
//          rx[CH4].last_state = 1;  // Remember current input state
//          rx[CH4].timer = rx_interrupt_time;  // Set timer to rx_interrupt_time
//        }
//    }
//    else if(rx[CH4].last_state == 1)  // Input is not high and changed from 1 to 0
//    {
//        rx[CH4].last_state = 0;  // Remember current input state
//        rx[CH4].value = rx_interrupt_time - rx[CH4].timer;  // Channel value is rx_interrupt_time - timer
//    }

} // end of ISR(PCINT0_vect)
