/*
 * DIYINHK.c
 *
 *     Created: 8/5/2017
 *      Author: Eric Williams <wd6cmu@gmail.com>
 * Description: Audio sample rate decoder/display for DIYINHK XMOS USB/I2S DAC interface
 *              for atTiny2313A or atTiny4313 and CPS02841A 4-digit LED display
 */ 

#define F_CPU 8000000UL     // Internal RC oscillator, no div8
#define nop asm("nop")

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>

/*
** Bit map:
**
**   Port A:
**      0: DP anode (pin 3)
**   Port B:
**      0: Digit 4 cathode (pin 6)
**      1: Digit 3 cathode (pin 8)
**      2: Digit 2 cathode (pin 9)
**      3: Digit 1 cathode (pin 12)
**      4: code bit 0 from XMOS card
**      5: code bit 1 (ISP MOSI)
**      6: code bit 2 (ISP MISO)
**      7: code bit 3 (ISP SCK)
**   Port D:
**      0: Eseg anode (pin 1)
**      1: Dseg anode (pin 2)
**      2: Cseg anode (pin 4)
**      3: Gseg anode (pin 5)
**      4: Aseg anode (pin 11)
**      5: Fseg anode (pin 10)
**      6: Bseg anode (pin 7)
**
**   Current limiting resistor in series with all anode lines
**   Pull-up on RESET (pin 1)
*/

//segment definitions
#define SEG_a 		0x10
#define SEG_b 		0x40
#define SEG_c 		0x04
#define SEG_d 		0x02
#define SEG_e 		0x01
#define SEG_f 		0x20
#define SEG_g 		0x08
#define SEG_dot		0x80

//7-segments character definitions
const unsigned char font[] PROGMEM = {
    (SEG_a|SEG_b|SEG_c|SEG_d|SEG_e|SEG_f),          // 0
    (SEG_b|SEG_c),                                  // 1
    (SEG_a|SEG_b|SEG_d|SEG_e|SEG_g),                // 2
    (SEG_a|SEG_b|SEG_c|SEG_d|SEG_g),                // 3
    (SEG_b|SEG_c|SEG_f|SEG_g),                      // 4
    (SEG_a|SEG_c|SEG_d|SEG_f|SEG_g),                // 5
    (SEG_a|SEG_c|SEG_d|SEG_e|SEG_f|SEG_g),          // 6
    (SEG_a|SEG_b|SEG_c),                            // 7
    (SEG_a|SEG_b|SEG_c|SEG_d|SEG_e|SEG_f|SEG_g),    // 8
    (SEG_a|SEG_b|SEG_c|SEG_d|SEG_f|SEG_g),          // 9
    (SEG_a|SEG_b|SEG_c|SEG_e|SEG_f|SEG_g),          // THE SYMBOL "A"
    (SEG_c|SEG_d|SEG_e|SEG_f|SEG_g),                // THE SYMBOL "b"
    (SEG_a|SEG_d|SEG_e|SEG_f),                      // THE SYMBOL "C"
    (SEG_b|SEG_c|SEG_d|SEG_e|SEG_g),                // THE SYMBOL "d"
    (SEG_a|SEG_d|SEG_e|SEG_f|SEG_g),                // THE SYMBOL "E"
    (SEG_a|SEG_e|SEG_f|SEG_g),                      // THE SYMBOL "F"
    (SEG_c|SEG_e|SEG_g),                            // THE SYMBOL "n"
    (SEG_c|SEG_d|SEG_e|SEG_g),                      // THE SYMBOL "o"
    (SEG_d|SEG_e|SEG_f|SEG_g),                      // THE SYMBOL "t"
    (SEG_d|SEG_e|SEG_f),                            // THE SYMBOL "L"
    (SEG_c|SEG_e|SEG_f|SEG_g),                      // THE SYMBOL "h"
    (SEG_b|SEG_c|SEG_e|SEG_f|SEG_g),                // THE SYMBOL "H"
    (SEG_a|SEG_b|SEG_e|SEG_f|SEG_g),                // THE SYMBOL "P"
    (SEG_b|SEG_c|SEG_d|SEG_f),                      // THE SYMBOL "J"
    (SEG_e|SEG_g),                                  // THE SYMBOL "r"
    (SEG_c|SEG_d|SEG_e),                            // THE SYMBOL "u"
    (SEG_b|SEG_c|SEG_d|SEG_f|SEG_g),                // THE SYMBOL "y"
};

uint8_t digit_disp;
char fb_current[4];     // display frame buffer, right to left digit positions

// Timer 0 interrupt routine for multiplexing character display.
ISR(TIMER0_OVF_vect)
{
    PORTB |= 0x0F;
    PORTD = 0;
    PORTA &= ~1;
    digit_disp = (digit_disp + 1) & 3;
    PORTD = fb_current[digit_disp] & 0x7F;
    if (fb_current[digit_disp] & 0x80) PORTA |= 1;  // decimal point segment
    PORTB = (PORTB & 0xF0) | (~(1 << digit_disp) & 0x0F);
}

ISR(TIMER0_COMPA_vect)
{
    PORTB |= 0x0F;	// Turn off segments at compare True
}

const char d0[] PROGMEM = " 44.1";
const char d1[] PROGMEM = " 48.0";
const char d2[] PROGMEM = " 88.2";
const char d3[] PROGMEM = " 96.0";
const char d4[] PROGMEM = "176.4";
const char d5[] PROGMEM = "192.0";
const char d6[] PROGMEM = "352.8";
const char d7[] PROGMEM = "384.0";
const char d8[] PROGMEM = "768.0";
const char dc[] PROGMEM = "D64 ";
const char dd[] PROGMEM = "D128";
const char de[] PROGMEM = "D256";
const char df[] PROGMEM = "D512";

PGM_P const decode[] PROGMEM = {
    d0, d1, d2, d3,
    d4, d5, d6, d7,
    d8,  0,  0,  0,
    dc, dd, de, df
};


int main(void)
{
    int8_t j;
    int8_t code, old_code = -1;
    PGM_P P;
    char *p, str[6];

    DDRA = 0x01;
    DDRB = 0x0F;
    DDRD = 0x7F;
    
    PORTB = 0x0F;
    //PORTB = 0xFF;   // pull-ups for testing

	// 8-bit Timer0 enabled for display multiplexing
    TCCR0A = 0;
	TCCR0B = _BV(CS01|CS00);    // clk/64 (122Hz refresh rate)
	TIMSK |= _BV(TOIE0) | _BV(OCIE0A);        // enable OVF interrupt
	OCR0A = 0;	// can be adjusted to reduce duty cycle/brightness
    sei();

    while(1) {
        code = PINB >> 4;
        if (code != old_code) {     // updating fb_current continuously disturbs display
            old_code = code;
            memcpy_P(&P, &decode[code], sizeof(PGM_P));    // PGM_P to display string for this code
            if (P == 0) {
                continue;   // unknown code
            } else {
                memset(fb_current, 0, sizeof(fb_current));
                strcpy_P(str, P);
                for (j = 3, p = str; *p && j >= 0; p++) {
                    if (*p == '.') switch(j) {
                        case 1:  fb_current[3] |= 0x80; break;
                        case 0:  fb_current[1] |= 0x80; break;
                        case -1: fb_current[0] |= 0x80; break;  // For safety's sake, never gets here
                        default: break;
                    }
                    else if (*p == ':' && j == 1)
                        fb_current[2] |= 0x80;
                    else if (*p >= '0' && *p <= '9')
                        fb_current[j--] = pgm_read_byte(font + *p - '0');
                    else if (*p >= 'A' && *p <= 'F')
                        fb_current[j--] = pgm_read_byte(font + *p - 'A' + 10);
                    else
                        fb_current[j--] = 0;
                }
            }
        }            
    }
}