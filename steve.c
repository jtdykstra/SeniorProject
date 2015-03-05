//Steve's soul, author: Jordan Dykstra, Robert Prosser, Anibal Hernandez
#include <avr/io.h>
#include <util/delay.h>
#include "serial.c"

#define NUM_ADC 7

//PORTB
#define LEFT_MOTOR_RED_WIRE 0x20 
#define LEFT_MOTOR_BLACK_WIRE 0x10
#define RIGHT_MOTOR_RED_WIRE 0x08 
#define RIGHT_MOTOR_BLACK_WIRE 0x04
#define BEAM_BREAK_INPUT 0x01
#define PWM_PIN 0x02 

//Analog port
#define INNER_LEFT_LINE_SENSOR 0 
#define MIDDLE_LINE_SENSOR 1
#define INNER_RIGHT_LINE_SENSOR 2
#define OUTER_LEFT_LINE_SENSOR 3
#define OUTER_RIGHT_LINE_SENSOR 4

//PORT D
#define RACK_MOTOR_RED 0x80 
#define RACK_MOTOR_BLACK 0x40 
#define DIG_LINE_SENSOR_RIGHT 0x20
#define DIG_LINE_SENSOR_LEFT 0x10 

#define THRESHOLD 200 //analog line senosr

#define BLACK_THRESHOLD 400 //digital sensors
#define WHITE_THRESHOLD 100


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
void MoveLeftWheelTicks(int ticks);
void MoveRightWheelTicks(int ticks);
void MoveForwardTicks(int ticks);
void MoveForwardTick();

int main(void)
{
   serial_init(); 
   DDRB = PWM_PIN | LEFT_MOTOR_RED_WIRE | LEFT_MOTOR_BLACK_WIRE | RIGHT_MOTOR_RED_WIRE | RIGHT_MOTOR_BLACK_WIRE; 
   DDRD = RACK_MOTOR_RED | RACK_MOTOR_BLACK | DIG_LINE_SENSOR_LEFT | DIG_LINE_SENSOR_RIGHT; 
   SetupClaw(); 

   //ADC setup. Lookup settings in the manual to tweak. 
   ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //set the ADC prescaler to 128
   ADMUX = (1 << REFS0); //Set reference voltage to AVCC
   ADMUX |= (1 << ADLAR); //Put highest 8 bits into the ADCH register
   ADCSRA |= (1 << ADEN) | (1 << ADIE); //Enable ADC   

   adcData data; 
   data.len = 7; 

   int state = 0; 

   while (1)
   {  
      ReadAllADCValues(&data);
      /*print_int(data.readings[MIDDLE_LINE_SENSOR]);
      print_string("   "); 
      print_int(data.readings[OUTER_RIGHT_LINE_SENSOR]);
      print_string("   ");
      print_string("\r\n\r\n");
      _delay_ms(1000); 
*/
      if (data.readings[OUTER_LEFT_LINE_SENSOR] > THRESHOLD && 
            data.readings[OUTER_RIGHT_LINE_SENSOR] > THRESHOLD &&
              state == 0)
      {
        state = 1; 
        
      } 


      switch(state) 
      {
         case 0: //follow the line
            if (data.readings[INNER_LEFT_LINE_SENSOR] > THRESHOLD)
            {  
               StopLeftWheel();
               _delay_ms(30); 
            }

            if (data.readings[INNER_RIGHT_LINE_SENSOR] > THRESHOLD)
            {
               StopRightWheel();
               _delay_ms(30);
            }

            LeftWheelForward();
            RightWheelForward();
            break;
         case 1: //move claw to rings
            LeftWheelReverse();
            RightWheelReverse();
            _delay_ms(30);
            StopLeftWheel();
            StopRightWheel();
            _delay_ms(1000);
            MoveForwardTicks(3); 
            state = 2; 
            break;
         case 2:
            RightWheelForward(); 
            LeftWheelReverse(); 
            while (data.readings[INNER_LEFT_LINE_SENSOR] < THRESHOLD)
               ReadAllADCValues(&data);
                
            
            RightWheelReverse(); 
            LeftWheelForward();
            _delay_ms(50);
            StopRightWheel();
            StopLeftWheel();
            state = 3;
            break;
         case 3:
            MoveForwardTicks(8); 
            RightWheelReverse();
            LeftWheelForward();
            while (data.readings[INNER_RIGHT_LINE_SENSOR] < THRESHOLD)
               ReadAllADCValues(&data); 
            RightWheelForward();
            LeftWheelReverse();
            _delay_ms(50);
            StopRightWheel();
            StopLeftWheel();
            state = 4; 
            break;
         default:
             
            break;
      } 

   }
} 

void MoveLeftWheelTicks(int ticks)
{
   int count = 0;
   uint32_t reading = 0; 

   LeftWheelForward();
   _delay_ms(50); 
 
   for (; count <= ticks; ++count)
   {  
      while (reading < BLACK_THRESHOLD) //go to black line
      {
         reading = ReadDigLineSensor(DIG_LINE_SENSOR_LEFT); 
      } 
   } 
   
   LeftWheelReverse();
   _delay_ms(10); 
   StopLeftWheel(); 
}

void MoveRightWheelTicks(int ticks)
{
   int count = 0;
   uint32_t reading = 0; 

   RightWheelForward();
   _delay_ms(50); 
 
   for (; count <= ticks; ++count)
   {  
      while (reading < BLACK_THRESHOLD) //go to black line
      {
         reading = ReadDigLineSensor(DIG_LINE_SENSOR_RIGHT); 
      } 
   } 
   
   RightWheelReverse();
   _delay_ms(10); 
   StopRightWheel(); 
}


void MoveForwardTicks(int ticks)
{
   int count = 0;
   for (count = 0; count <= ticks; ++count)
      MoveForwardTick(); 
}

void MoveForwardTick()
{
   int count = 0;
   uint32_t reading = 0; 
  
         LeftWheelForward();
         RightWheelForward(); 
         _delay_ms(50); 

         while (reading < BLACK_THRESHOLD) //go to black line
         {
            reading = ReadDigLineSensor(DIG_LINE_SENSOR_RIGHT); 
         }  
           
         RightWheelReverse(); 
         LeftWheelReverse();
         _delay_ms(10); 
         StopLeftWheel(); 
         StopRightWheel(); 
   
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
