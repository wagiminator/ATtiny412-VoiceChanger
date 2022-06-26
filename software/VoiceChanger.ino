// ===================================================================================
// Project:   Voice Changer
// Version:   v1.0
// Year:      2022
// Author:    Stefan Wagner
// Github:    https://github.com/wagiminator
// EasyEDA:   https://easyeda.com/wagiminator
// License:   http://creativecommons.org/licenses/by-sa/3.0/
// ===================================================================================
//
// Description:
// ------------
// The amplified signal from the microphone is continuously sampled by the ADC and
// the corresponding values are written into a ring buffer. These values are in turn
// read in at variable speed and sent to the amplifier via the DAC. The voice is
// changed by the different sampling rates of the ADC and DAC.
//
// Wiring:
// -------
//                         +-\/-+
//                   Vdd  1|°   |8  GND
// AUDIO OUT --- TXD PA6  2|    |7  PA3 AIN3 -------- SLCT BUTTON
//  AUDIO IN --- RXD PA7  3|    |6  PA0 AIN0 UPDI --- UPDI
// UP BUTTON --- SDA PA1  4|    |5  PA2 AIN2 SCL ---- DOWN BUTTON
//                         +----+
//
// Compilation Settings:
// ---------------------
// Core:    megaTinyCore (https://github.com/SpenceKonde/megaTinyCore)
// Board:   ATtiny412/402/212/202
// Chip:    ATtiny412
// Clock:   4 MHz internal
//
// Leave the rest on default settings. Don't forget to "Burn bootloader"! Compile
// and upload the code.
//
// No Arduino core functions or libraries are used. To compile and upload without
// Arduino IDE download AVR 8-bit toolchain at:
// https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers
// and extract to tools/avr-gcc. Use the makefile to compile and upload.
//
// Fuse Settings: 0:0x00 1:0x00 2:0x01 4:0x00 5:0xC5 6:0x04 7:0x00 8:0x00
//
// Operating Instructions:
// -----------------------
// Connect a 5V power supply and a speaker (max 3W/4Ω). Turn on the device and
// speak into the microphone. Use the buttons to change the voice.

// ===================================================================================
// Libraries, Definitions and Macros
// ===================================================================================

// Libraries
#include <avr/io.h>               // for GPIO
#include <avr/interrupt.h>        // for interrupts
#include <util/delay.h>           // for delays

// Pin definitions
#define PIN_UP        PA1         // pin connected to UP button
#define PIN_DOWN      PA2         // pin connected to DOWN button
#define PIN_SLCT      PA3         // pin connected to SELECT button
#define PIN_DAC       PA6         // audio output pin
#define PIN_ADC       PA7         // audio input pin

// Audio pitch values
#define PITCH_START   103         // start value for audio pitch
#define PITCH_MIN     63          // minimum value for audio pitch
#define PITCH_MAX     183         // maximum value for audio pitch

// Pin manipulation macros
enum {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7};      // enumerate pin designators
#define pinInput(x)       VPORTA.DIR &= ~(1<<(x))   // set pin to INPUT
#define pinOutput(x)      VPORTA.DIR |=  (1<<(x))   // set pin to OUTPUT
#define pinLow(x)         VPORTA.OUT &= ~(1<<(x))   // set pin to LOW
#define pinHigh(x)        VPORTA.OUT |=  (1<<(x))   // set pin to HIGH
#define pinToggle(x)      VPORTA.IN  |=  (1<<(x))   // TOGGLE pin
#define pinRead(x)        (VPORTA.IN &   (1<<(x)))  // READ pin
#define pinPullup(x)      (&PORTA.PIN0CTRL)[x] |= PORT_PULLUPEN_bm  // enable pullup
#define pinDisable(x)     (&PORTA.PIN0CTRL)[x] |= PORT_ISC_INPUT_DISABLE_gc     

// ===================================================================================
// Ring Buffer Implementation
// ===================================================================================

#define BUF_LEN       192                       // buffer length
volatile uint8_t BUF_buffer[BUF_LEN];           // this is the buffer
volatile uint8_t BUF_head, BUF_tail;            // head and tail pointer

// Push byte to ring buffer
void BUF_push(uint8_t data) {
  if(++BUF_head >= BUF_LEN) BUF_head = 0;       // increase head pointer
  BUF_buffer[BUF_head] = data;                  // push data byte to buffer
}

// Pop data byte from buffer
uint8_t BUF_pop(void) {
  if(++BUF_tail >= BUF_LEN) BUF_tail = 0;       // increase tail pointer
  return BUF_buffer[BUF_tail];                  // pop data byte from buffer
}

// ===================================================================================
// ADC Implementation for Audio Input
// ===================================================================================

// Init analog to digital converter (ADC)
void ADC_init(void) {
  pinDisable(PIN_ADC);                          // disable digital input buffer
  ADC0.CTRLA   = ADC_RESSEL_8BIT_gc             // 8-bit resolution
               | ADC_FREERUN_bm                 // free running mode
               | ADC_ENABLE_bm;                 // enable ADC
  ADC0.CTRLC   = ADC_SAMPCAP_bm                 // select sample capacitance
               | ADC_REFSEL_VDDREF_gc           // set Vdd as reference
               | ADC_PRESC_DIV8_gc;             // set prescaler -> 500kHz ADC clock
  ADC0.MUXPOS  = PIN_ADC;                       // set the input pin
  ADC0.INTCTRL = ADC_RESRDY_bm;                 // enable result ready interrupt
  ADC0.COMMAND = ADC_STCONV_bm;                 // start first conversion
}

// ADC result ready interrupt service routine
ISR(ADC0_RESRDY_vect) {
  BUF_push(ADC0.RESL);                          // push result to buffer
}

// ===================================================================================
// DAC Implementation for Audio Output
// ===================================================================================

// Setup the digital to analog converter (DAC)
void DAC_init(void) {
  pinDisable(PIN_DAC);                          // disable digital input buffer
  VREF_CTRLA |= VREF_DAC0REFSEL_4V34_gc;        // set DAC reference to 4.3V
  DAC0.CTRLA  = DAC_ENABLE_bm                   // enable DAC
              | DAC_OUTEN_bm;                   // enable output buffer
}

// Set DAC value
void DAC_set(uint8_t value) {
  DAC0.DATA = value;                            // set DAC
}

// ===================================================================================
// Periodic Interrupt Implementation using TCB for Audio Output
// ===================================================================================

// Init timer/counter B (TCB)
void TCB_init(void) {
  TCB0.CTRLA   = TCB_CLKSEL_CLKDIV1_gc          // set prescaler
               | TCB_ENABLE_bm;                 // enable timer
  TCB0.CTRLB   = TCB_CNTMODE_INT_gc;            // set timer to periodic interrupt mode
  TCB0.CCMP    = PITCH_START;                   // set TOP value
  TCB0.INTCTRL = TCB_CAPT_bm;                   // enable interrupt
}

// Set pitch value
void TCB_setPitch(uint8_t value) {
  TCB0.CCMP = value;                            // set pitch value
  TCB0.CNT  = 0;                                // reset counter
}

// Timer interrupt service routine
ISR(TCB0_INT_vect) {
  TCB0.INTFLAGS = TCB_CAPT_bm;                  // clear interrupt flag
  DAC_set(BUF_pop());                           // set DAC value from buffer
}

// ===================================================================================
// Button Implementation
// ===================================================================================

// Button variables
uint8_t butlast_up = 0;                         // remember last button state
uint8_t butlast_down = 0;                       // remember last button state
uint8_t butlast_slct = 0;                       // remember last button state

// Init buttons
void BUT_init(void) {
  pinPullup(PIN_UP);                            // enable internal pullup resistor
  pinPullup(PIN_DOWN);                          // enable internal pullup resistor
  pinPullup(PIN_SLCT);                          // enable internal pullup resistor
}

// ===================================================================================
// Main Function
// ===================================================================================

int main(void) {
  // Setup MCU
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 3);       // set clock frequency to 4 MHz

  // Setup modules
  BUT_init();                                   // init buttons
  DAC_init();                                   // init DAC
  ADC_init();                                   // init ADC
  TCB_init();                                   // Init TCB
  sei();                                        // enable interrupts
  uint8_t pitch = PITCH_START;                  // set initial pitch value

  // Loop
  while(1) {
    // Check buttons
    uint8_t butread = pinRead(PIN_UP);          // check UP button
    if(butread != butlast_up) {
      if(!butread && (pitch >= PITCH_MIN)) {
        pitch -= 20;
        TCB_setPitch(pitch);
      }
      butlast_up = butread;
      _delay_ms(10);
    }

    butread = pinRead(PIN_DOWN);                // check DOWN button
    if(butread != butlast_down) {
      if(!butread && (pitch <= PITCH_MAX)) {
        pitch += 20;
        TCB_setPitch(pitch);
      }
      butlast_down = butread;
      _delay_ms(10);
    }   
  }
}
