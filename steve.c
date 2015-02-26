//Steve's soul, author: Jordan Dykstra, Robert Prosser, Anibal Hernandez
#include <avr/io.h>
#include <util/delay.h>
#include "serial.c"

#define NUM_ADC 7

#define LEFT_MOTOR_RED_WIRE 0x20 //PORT B
#define LEFT_MOTOR_BLACK_WIRE 0x10 //PORT B
#define RIGHT_MOTOR_RED_WIRE 0x08 //PORT B
#define RIGHT_MOTOR_BLACK_WIRE 0x04 //PORT B
#define BEAM_BREAK_INPUT 0x01 //PORTB
#define PWM_PIN 0x02 //PORTB

#define LINE_SENSOR_ONE 0x10 //Analog port 
#define LINE_SENSOR_TWO 0x20 //Analog port

#define RACK_MOTOR_RED 0x80 //PORTD
#define RACK_MOTOR_BLACK 0x40 //PORTD
#define DIG_LINE_SENSOR_ONE 0x20 //PORTD
#define DIG_LINE_SENSOR_TWO 0x10 //PORTD 

typedef struct adcData { 
   uint16_t len;
   int16_t readings[NUM_ADC]; 
} adcData; 

void SetupClaw();
void OpenClaw();
void CloseClaw();
void LeftWheelForward();
void LeftWheelReverse();
void RightWheelForward();
void RightWheelReverse(); 
void StopLeftWheel();
void StopRightWheel(); 
void ReadAllADCValues(adcData *data);
void MoveClawUp();
void MoveClawDown(); 
uint32_t ReadDigLineSensor(uint8_t port);

int main(void)
{
   serial_init(); 
   DDRB = PWM_PIN | LEFT_MOTOR_RED_WIRE | LEFT_MOTOR_BLACK_WIRE | RIGHT_MOTOR_RED_WIRE | RIGHT_MOTOR_BLACK_WIRE; 
   DDRD = RACK_MOTOR_RED | RACK_MOTOR_BLACK | DIG_LINE_SENSOR_ONE; 
   SetupClaw(); 

   //ADC setup. Lookup settings in the manual to tweak. 
   ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //set the ADC prescaler to 128
   ADMUX = (1 << REFS0); //Set reference voltage to AVCC
   ADMUX |= (1 << ADLAR); //Put highest 8 bits into the ADCH register
   ADCSRA |= (1 << ADEN) | (1 << ADIE); //Enable ADC   

   adcData data; 
   data.len = 3; 

   while (1)
   {
      print_int32(ReadDigLineSensor(DIG_LINE_SENSOR_ONE)); 
      print_string("****"); 
      /*ReadAllADCValues(&data); 
         LeftWheelForward();
         RightWheelForward(); 

      if (data.readings[0] > 200)
      {
         StopLeftWheel(); 
         _delay_ms(55); 
      }
      else if (data.readings[2] > 200)
      {
         StopRightWheel(); 
         _delay_ms(55); 
      } */
   }
} 

uint32_t ReadDigLineSensor(uint8_t port)
{  
   uint32_t count = 0;

   DDRD |= port; //Drive pin high to charge cap (see schematic)
   PORTD |= port; 
   _delay_us(50); //50 us to charge cap (should probably verify this mathematically)
   
   //Switch the port to an input
   DDRD &= ~(port); 
   PORTD &= ~(port); 

   while (PIND & port) //count the time it takes for the cap to drop below 2.5 V
      ++count; 

   return count; 
}  

void MoveClawDown()
{
   PORTD |= RACK_MOTOR_RED;
   PORTD &= ~(RACK_MOTOR_BLACK); 
   _delay_ms(500); 
   while (PINB & BEAM_BREAK_INPUT)
      ;
   PORTD &= ~(RACK_MOTOR_RED);
   PORTD &= ~(RACK_MOTOR_BLACK); 
}

void MoveClawUp()
{
   PORTD &= ~(RACK_MOTOR_RED); 
   PORTD |= RACK_MOTOR_BLACK; 
   _delay_ms(500);    
   while (PINB & BEAM_BREAK_INPUT)
      ;
   PORTD &= ~(RACK_MOTOR_RED);
   PORTD &= ~(RACK_MOTOR_BLACK); 
}  

//ADC runs at 125 kHz, 13 cycles per read
//8 us/cycle, meaning that the total length of 
//time for the ADC to process the data is roughly
//104 us * data->len. I'm not taking into account
//CPU cycles or memory accesses, that's purely the ADC 
void ReadAllADCValues(adcData *data)
{
   uint16_t ind = 0;

   //read all desired adc values into the adcData struct
   while (ind < data->len) 
   { 
      //Select the current adc mux value
      ADMUX &= ~(0xF); //clear lower 4 bits, which are the mux values
      ADMUX |= ind; //set the correct mux value 
      ADCSRA |= (1 << ADSC); //Start measuring! 
      
      while (!(ADCSRA & (1 << ADIF)))  //wait for measurement to complete
         ;

      data->readings[ind] = ADCH; //add the reading to the data
      ADCSRA &= ~(1 << ADIF); //clear interrupt flag 
      ++ind;
   } 
} 

//Setup timer for claw opening / closing
//The specific WGM values determine the mode of operation, in this case PWM
//The COM bits invert the PWM
//The CS bits set the prescaler for the timer
//The ICR1 register is the top bit
//The OCR1A bit is where the first toggle occurs, then the line goes low
//again when ICR1 is hit. 
void SetupClaw()
{
   TCCR1A |= 1 << WGM11 | 1 << COM1A1 | 1 << COM1A0 ;
   TCCR1B |= 1 << WGM13 | 1 << WGM12 | 1 << CS10 | 1 << CS12; //clock prescaler of 1024
   ICR1 = 313; //This will prevent anything from happening for now
}

void OpenClaw()
{
   OCR1A = ICR1 - 31; //Set the pulse width to about 2 ms
   _delay_ms(1000); 
   ICR1 = 313; 

}

void CloseClaw()
{
   OCR1A = ICR1 - 15; //Set the pulse width to about 1 ms
   _delay_ms(1000); 
   ICR1 = 313; 
}

void StopClawOutput()
{

} 

void LeftWheelForward()
{
    PORTB |= LEFT_MOTOR_RED_WIRE; //drive red wire high
    PORTB &= ~(LEFT_MOTOR_BLACK_WIRE); //drive black wire low
}

void LeftWheelReverse()
{
    PORTB |= LEFT_MOTOR_BLACK_WIRE; 
    PORTB &= ~(LEFT_MOTOR_RED_WIRE); 
}

void RightWheelForward()
{
    PORTB |= RIGHT_MOTOR_RED_WIRE;
    PORTB &= ~(RIGHT_MOTOR_BLACK_WIRE); 
}

void RightWheelReverse()
{
   PORTB |= RIGHT_MOTOR_BLACK_WIRE;
   PORTB &= ~(RIGHT_MOTOR_RED_WIRE); 
}

void StopLeftWheel()
{
   PORTB &= ~(LEFT_MOTOR_BLACK_WIRE); 
   PORTB &= ~(LEFT_MOTOR_RED_WIRE); 
}

void StopRightWheel()
{
   PORTB &= ~(RIGHT_MOTOR_BLACK_WIRE); 
   PORTB &= ~(RIGHT_MOTOR_RED_WIRE); 
}
