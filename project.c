/********************************************
 *
 *  Name: Adam Raslan
 *  Email: araslan@usc.edu
 *  Section: 12:30pm Friday Weber
 *  Assignment: Rangefinder Project
 *
 ********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "lcd.h"

unsigned int threshold;
volatile uint8_t a, b, x;
volatile uint8_t new_state, old_state;

volatile uint8_t changed = 0; 
volatile uint32_t test_distance = 0; 
volatile int pulse_count = 0; 
volatile int counting = 0;
volatile int error = 0; 


enum STATES {LOCAL, REMOTE};
volatile int localthreshold = 0;
volatile int remotethreshold = 0;
volatile int thresholdState = LOCAL;



void timer2_init(void);
void debounce(uint8_t bit);
void timer1_init();

int main(void){
    lcd_init(); // call to initialize the lcd

    timer0_init(); //call to initialize timer 0 

    timer2_init(); //call to initialize timer 2

    timer1_init(); //call to initialize timer 1

    DDRB &= ~(1 << PB4); // set acquire button to input
    PORTB |= (1 << PB4); // enable pull up resistor for acquire button

    DDRB |= (1 << PB3); // set PB3 as output

    DDRC |= (1 << PC4); //LED
    DDRB |= (1 << PB5); //LED
    PORTB |= (1 << PB5); // LED
    PORTC |= (1 << PC4); //LED


    DDRC &= ~(1 << PC0); // make right button as input 

    //rotary encoder
    DDRD &= ~(1 << PD2 | 1 << PD3);
    PORTD |= (1 << PD2 | 1 << PD3);


    DDRC |= (1 << PC1); //set trigger as output
    PORTC &= ~(1 << PC1); //set to 0
    PCICR |= (1 << PCIE1); // enable pin change interrupt on PC2
    PCMSK1 |= (1 << PCINT10); // enable pin change interrupt on PC2

    //set PC3 as output and put 0 in

    DDRC |= (1 << PC3); 
    PORTC &= ~(1<< PC3); 


    PCICR |= (1<<PCIE2);
    PCMSK2 |= (1<<PCINT18 | 1<<PCINT19);
    sei(); //enable interrupts

    char buf[10];

    //display splash screen
    lcd_writecommand(1); 
l   cd_moveto(0,0);
    lcd_stringout("EE109 Project");
    lcd_moveto(1,0);
    lcd_stringout("Adam Raslan");
    _delay_ms(1000);
    lcd_writecommand(1);


    x = PIND;

    a = (PIND >> PD2) & 1;
    b = (PIND >> PD3) & 1;

    if (!b && !a)
	    old_state = 0;
    else if (!b && a)
	    old_state = 1;
    else if (b && !a)
	    old_state = 2;
    else
	    old_state = 3;

    new_state = old_state;
    int16_t tempremotethreshold;
    int16_t templocalthreshold; 
    templocalthreshold = eeprom_read_word((void *) 100);
    tempremotethreshold = eeprom_read_word((void *) 200);
    if(tempremotethreshold != NULL && tempremotethreshold < 400 && tempremotethreshold > 0 ){
        remotethreshold = tempremotethreshold;
    }else{
        remotethreshold = 2;
    }

    if(templocalthreshold != NULL && templocalthreshold < 400 && templocalthreshold > 0 ){
        localthreshold = templocalthreshold;
    }
    else{
        localthreshold = 2;
    }


    threshold = localthreshold;
    char thresh_string[20];

    while (1){

         if (changed) {
            if (thresholdState == REMOTE ) {
                lcd_moveto(0,1);
                lcd_stringout("Remote    "); 
            }
            else {
                lcd_moveto(0,1);
                lcd_stringout("Local    "); 
            }

            if (thresholdState == LOCAL ){
                threshold = localthreshold; 
            }else{
                threshold = remotethreshold; 
            }   

            eeprom_update_word((void *) 100, localthreshold);
            eeprom_update_word((void *) 200, remotethreshold);
        }

        if (!(PINC & (1 << PC0))) {
            _delay_ms(5);
            while(!(PINC & (1<<PC0))) {} 
            _delay_ms(5);

            if (thresholdState == LOCAL) thresholdState = REMOTE;
            else thresholdState = LOCAL;

        }

        
        if (!(PINB & (1<<PB4))){
            debounce(PB4); 
            PORTC |= (1 << PC1); 
            _delay_us(10); 
            PORTC &= ~(1 << PC1); 
            //if accquire button has been pressed
            lcd_writecommand(1);
            lcd_moveto(0,0); 
            if (error != 1){
                lcd_writecommand(1);
                snprintf(buf, 10,"ds=%3d.%d", test_distance); 
                lcd_stringout(buf);
                OCR2A = (-23 * test_distance) / 400 + 35;
            }else if (error == 1){
                lcd_writecommand(1);
                lcd_stringout("error"); 
                error = 0; 
                PORTB |= (1 << PB5);
                PORTC |= (1 << PC4);
            }
        }
           
        snprintf(thresh_string, 20, "%3d", threshold);
        lcd_moveto(1,1);



        if (threshold >= test_distance) { 
            //turn PINB5 and PINC4 off, this turns on red LED
            PORTB &= ~(1 << PB5);
            PORTC &= ~(1 << PC4);
        }
        else if (threshold < test_distance) { 
            //turn PINB5 and PINC4 off, this turns on green LED
            PORTB |= (1 << PB5);
            PORTC &= ~(1 << PC4);
        }   lcd_stringout(lcd_str);
    
    }
}

void debounce(uint8_t bit)
{
    // Add code to debounce input "bit" of PINB
    // assuming we have sensed the start of a press.
        _delay_ms(5);
         while( (PINB & (1<<bit)) == 0 )
           {}
        _delay_ms(5);
}



void timer0_init(){
    TIMSK0 |= (1 << OCIE0A);
    TCCR0A |= (1 << WGM01);
}

ISR(TIMER0_COMPA_vect){
    
}

void timer1_init()
{
    TCCR1B |= (1 << WGM12);
    TIMSK1 |= (1 << OCIE1A);
    OCR1A = 46400; 
}


ISR(PCINT1_vect)
{
    if ((PINC & (1 << PC2)) && counting == 0){
        //if echo signal is 1
        TCNT1 = 0;
        TCCR1B |= (1 << CS11);
        counting = 1; 
        //start counting
    }else if ((!(PINC & (1<<PC2)))&& counting == 1){
        // if echo signal is 0
        pulse_count = TCNT1;
        test_distance = pulse_count/(2*58);
        if (test_distance >= 400){
            char buf[10] = "errorflag"; 
        }
        TCCR1B &= ~(1 << CS11);
        TCNT1 = 0;
        counting = 0; 
    }
}


ISR(TIMER1_COMPA_vect){
    //error flag if count goes over and reset appropriate values
    error= 1;
    TCCR1B &= ~(1<< CS11);
    TCNT1 = 0;
}

ISR (PCINT2_vect) {

    //obtain values for a and b from pin 2 and 3 on D

    x = PIND;
    a = (PIND >> PD2) & 1;
    b = (PIND >> PD3) & 1;


    if (old_state == 0){ 
         // Handle A and B inputs for state 1
        if (b == 1){
            new_state = 2;
            if (thresholdState == LOCAL ){
                localthreshold--;
            }else{
                remotethreshold--;
            }
        }else if (a == 1){
            new_state = 1;
            if (thresholdState == LOCAL ){
                localthreshold++;
            }else{
                remotethreshold++;
            }   
        }
        

    }
    else if (old_state == 1) {
        // Handle A and B inputs for state 1
        if (a == 0){
            new_state = 0;
            if (thresholdState == LOCAL ){
                localthreshold--;
            }else{
                remotethreshold--;
            }
        }else if (b == 1){
            new_state = 3;
            if (thresholdState == LOCAL ){
                localthreshold++;
            }else{
                remotethreshold++;
            }        
        }

    }
    else if (old_state == 2) {
        // Handle A and B inputs for state 2
        if (a == 1){
            new_state = 3;
            if (thresholdState == LOCAL ){
                localthreshold--;
            }else{
                remotethreshold--;
            }
        }else if (b == 0){
            new_state = 0;
            if (thresholdState == LOCAL ){
                localthreshold++;
            }else{
                remotethreshold++;
            }    
        }

    }
    else {   // old_state = 3
        // Handle A and B inputs for state 3
        if (b == 0){
            new_state = 1;
            if (thresholdState == LOCAL ){
                localthreshold--;
            }else{
                remotethreshold--;
            }
        }else if (a == 0){
            new_state = 2;
            if (thresholdState == LOCAL ){
                localthreshold++;
            }else{
                remotethreshold++;
            }  
        }
    }

    if (localthreshold < 0){
        localthreshold = 400; 
    }

    if (localthreshold > 400){
        localthreshold = 0; 
    }

    if (remotethreshold < 0){
        remotethreshold = 400; 
    }

    if (remotethreshold > 400){
        remotethreshold = 0; 
    }

    if (new_state != old_state) {
        changed = 1;
        old_state = new_state;
    }
}

void timer2_init(void)
{
    TCCR2A |= (0b11 << WGM20);  // Fast PWM mode, modulus = 256
    TCCR2A |= (0b10 << COM2A0); // Turn D11 on at 0x00 and off at OCR2A
    OCR2A = 22;                // Initial pulse duty cycle of 50%
    TCCR2B |= (0b111 << CS20);  // Prescaler = 1024 for 16ms period
}

