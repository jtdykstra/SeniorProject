//Steve's soul, author: Jordan Dykstra, Robert Prosser, Anibal Hernandez
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "serial.c"

#define NUM_ADC 7

//PORTB
#define LEFT_MOTOR_RED_WIRE 0x20      //Pin 13 
#define LEFT_MOTOR_BLACK_WIRE 0x10    //Pin 12
#define RIGHT_MOTOR_RED_WIRE 0x08     //Pin 11
#define RIGHT_MOTOR_BLACK_WIRE 0x04   //Pin 10
#define BEAM_BREAK_INPUT 0x01         //Pin 8
#define PWM_PIN 0x02                  //Pin 9

//Analog port
#define INNER_LEFT_LINE_SENSOR 0      //Analog pin 0
#define MIDDLE_LINE_SENSOR 1          //Analog pin 1
#define INNER_RIGHT_LINE_SENSOR 2     //Analog pin 2
#define OUTER_LEFT_LINE_SENSOR 3      //Analog pin 3
#define OUTER_RIGHT_LINE_SENSOR 4     //Analog pin 4

//PORT D
#define RACK_MOTOR_RED 0x80           //Pin 7 
#define RACK_MOTOR_BLACK 0x40         //Pin 6
#define DIG_LINE_SENSOR_RIGHT 0x20    //Pin 4
#define DIG_LINE_SENSOR_LEFT 0x10    //Pin 5

//Analog sensors
#define THRESHOLD 200                 
#define OUTER_LEFT_THRESH 150
#define INNER_LEFT_THRESH 150
#define MIDDLE_THRESH 150
#define INNER_RIGHT_THRESH 150
#define OUTER_RIGHT_THRESH 150

//Digital sensors
#define BLACK_THRESHOLD 400  
#define WHITE_THRESHOLD 100

//Interrupt counts for specific motor speeds
#define MOTOR_SPEED_1 16 //500 HZ  

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
void StopClawOutput();
uint32_t ReadDigLineSensor(uint8_t port);
void MoveLeftWheelTicks(int ticks);
void MoveRightWheelTicks(int ticks);
void MoveForwardTicks(int ticks);
void MoveForwardTick();
uint32_t AvgReadDigLineSensor(uint8_t port);
void AvgReadAllADCValues(adcData *data);
void FullyOpenClaw();
void FullyCloseClaw();

static volatile uint16_t isrCount = 0;
static volatile uint16_t speed = 1; 
static uint8_t leftWheelToggle = 0; 
static uint8_t rightWheelToggle = 0; 
static void (*timer1Behavior)(); 
static volatile int state = 0; 
static volatile int openClawState = 0;
static volatile int closeClawState = 0;
static volatile int lineFollowCount = 0;
static volatile int turning = 0;
static int lowMotorCount = 5;
static int highMotorCount = 20;
static int curMotorCount = 0;

ISR(TIMER2_COMPA_vect) {
   
   if (turning)
   {
      ++curMotorCount;
      if (curMotorCount == lowMotorCount)
         PORTB |= (leftWheelToggle | rightWheelToggle);  
      else if (curMotorCount == highMotorCount)
      {
         PORTB &= ~(leftWheelToggle | rightWheelToggle);
         curMotorCount = 0;
      }
   }

   if (!turning)
      PORTB |= leftWheelToggle | rightWheelToggle; 
}

ISR(TIMER1_COMPA_vect) {
   timer1Behavior(); 
}

ISR(BADISR_vect) {
   print_string("bad isr!\r\n");
}

void FollowOne()
{
      state = 3;
      TIMSK1 &= ~(1 << OCIE1A);
      TCNT1 = 0;
}

void FollowTwo()
{
   //update this
   state = 13;
   TIMSK1 &= ~(1 << OCIE1A);
   TCNT1 = 0;
}

void FollowThree()
{
   state = 9;
   TIMSK1 &= ~(1 << OCIE1A);
   TCNT1 = 0;
}

void OpenClawISRHandler()
{
   if (openClawState == 0)
   {
      PORTB |= PWM_PIN;
      OCR1A = 31;
      TCNT1 = 0;
      openClawState = 1;
   }
   else if (openClawState == 1)
   {
      PORTB &= ~(PWM_PIN);
      TCNT1 = 0;
      OCR1A = 281;
      openClawState = 0;
   }
}

void CloseClawISRHandler()
{

   if (closeClawState == 0)
   {
      PORTB |= PWM_PIN;
      TCNT1 = 0;
      OCR1A = 15;
      closeClawState = 1;
   }
   else if (closeClawState == 1)
   {
      PORTB &= ~(PWM_PIN);
      TCNT1 = 0;
      OCR1A = 297;
      closeClawState = 0;
   }
}

int main(void)
{
   serial_init(); 
   DDRB = PWM_PIN | LEFT_MOTOR_RED_WIRE | LEFT_MOTOR_BLACK_WIRE | RIGHT_MOTOR_RED_WIRE | RIGHT_MOTOR_BLACK_WIRE; 
   DDRD = RACK_MOTOR_RED | RACK_MOTOR_BLACK | DIG_LINE_SENSOR_LEFT | DIG_LINE_SENSOR_RIGHT; 

   //ADC setup. Lookup settings in the manual to tweak. 
   ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //set the ADC prescaler to 128
   ADMUX = (1 << REFS0); //Set reference voltage to AVCC
   ADMUX |= (1 << ADLAR); //Put highest 8 bits into the ADCH register
   ADCSRA |= (1 << ADEN); //Enable ADC   

   //Setup the interrupt timer
   TIMSK2 = (1 << OCIE2A); //enable timer 2 interrupts
   TCNT2 = 0; //initialize count to 0
   TCCR2A = (1 << WGM21); //CTC mode
   TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); //Prescaler
   OCR2A = 255; 

   //Line following / claw timer setup 
   TCNT1 = 0;
   TCCR1A = 0;
   TCCR1B = (1 << WGM12) | (1 << CS10) | (1 << CS12); //clock divider of 1024 

   //ADC data container
   adcData data; 
   data.len = 6; 

   uint32_t leftDig = 0;
   uint32_t rightDig = 0;
   sei();
   int flag = 0;
   int crossCount = 0;
   int blackOn = 0;
   StopLeftWheel();
   StopRightWheel();
   _delay_ms(1000);
   MoveClawDown();
   
   while (1)
   {  


      /*AvgReadAllADCValues(&data);
      //rightDig = ReadDigLineSensor(DIG_LINE_SENSOR_RIGHT); 
      //leftDig = ReadDigLineSensor(DIG_LINE_SENSOR_LEFT); 
      set_cursor(0,0);
      print_int(data.readings[OUTER_LEFT_LINE_SENSOR]);
      print_string("***");
      print_int(data.readings[INNER_LEFT_LINE_SENSOR]);
      print_string("***");
      print_int(data.readings[MIDDLE_LINE_SENSOR]);
      print_string("***");
      print_int(data.readings[INNER_RIGHT_LINE_SENSOR]);
      print_string("***");
      print_int(data.readings[OUTER_RIGHT_LINE_SENSOR]);
      print_string("\r\n\r\n");
      //print_int32(leftDig);
      //print_string("----");
      //print_int32(rightDig); 
      _delay_ms(500);
      clear_screen();*/
   /*if (flag)
   {
      FullyOpenClaw();
      MoveClawUp();
      flag = 0;
   }
   else
   {
      FullyCloseClaw();
      MoveClawDown();
      flag = 1;
   }
   _delay_ms(2000);*/

      
      AvgReadAllADCValues(&data);
      if ((data.readings[OUTER_LEFT_LINE_SENSOR] > THRESHOLD || 
            data.readings[OUTER_RIGHT_LINE_SENSOR] > THRESHOLD) &&
              (state == 0 || state == 6) && blackOn == 0)
      {
         StopLeftWheel();
         StopRightWheel();
         //_delay_ms(3000);
         ++crossCount;   
         blackOn = 1;
      }  
      else if ((data.readings[OUTER_LEFT_LINE_SENSOR] < THRESHOLD && data.readings[OUTER_RIGHT_LINE_SENSOR] < THRESHOLD)
                 && blackOn == 1)
         blackOn = 0;

      if (crossCount == 2)
      {
         ++state;
         ++crossCount;
      }
 
      switch(state) 
      {
         case 0: //follow the line
            if (data.readings[MIDDLE_LINE_SENSOR] > THRESHOLD)
            {
               LeftWheelForward();
               RightWheelForward();
            }
            else if (data.readings[INNER_LEFT_LINE_SENSOR] > THRESHOLD)
            {  
               StopLeftWheel();
               RightWheelForward();
               _delay_ms(75); 
            }
            else if (data.readings[INNER_RIGHT_LINE_SENSOR] > THRESHOLD)
            {
               StopRightWheel();
               LeftWheelForward();
               _delay_ms(75);
            }

            break;
         case 1: 
            StopLeftWheel();
            StopRightWheel(); 
            //MoveClawDown();
            FullyOpenClaw();

            state = 2; 
            cli();
            TIFR1 |= (1 << OCF1A);
            TCNT1 = 0;
            timer1Behavior = FollowOne; 
            OCR1A = 60000; 
            TIMSK1 = (1 << OCIE1A);
            sei();

            break;
         case 2: //follow the line for x seconds
            if (data.readings[MIDDLE_LINE_SENSOR] > THRESHOLD)
            {
               LeftWheelForward();
               RightWheelForward();
            }
            else if (data.readings[INNER_LEFT_LINE_SENSOR] > THRESHOLD)
            {  
               StopLeftWheel();
               RightWheelForward();
               _delay_ms(75); 
            }
            else if (data.readings[INNER_RIGHT_LINE_SENSOR] > THRESHOLD)
            {
               StopRightWheel();
               LeftWheelForward();
               _delay_ms(75);
            }

            break;
         case 3:
            StopLeftWheel();
            StopRightWheel();
            FullyCloseClaw();
            state = 4;
            break;
         case 4:
            
            if (data.readings[MIDDLE_LINE_SENSOR] > THRESHOLD)
            {
               LeftWheelReverse();
               RightWheelReverse(); 
            }
            else if (data.readings[INNER_LEFT_LINE_SENSOR] > THRESHOLD)
            {  
               LeftWheelReverse();
               StopRightWheel();
               _delay_ms(75); 
            }
            else if (data.readings[INNER_RIGHT_LINE_SENSOR] > THRESHOLD)
            {
               StopLeftWheel();
               RightWheelReverse();;
               _delay_ms(75);
            }

            if  (data.readings[INNER_LEFT_LINE_SENSOR] > THRESHOLD && 
                  data.readings[INNER_RIGHT_LINE_SENSOR] > THRESHOLD)
            {
               state = 5; 
               StopLeftWheel();
               StopRightWheel();
            }

            break;
         case 5:
            //turn right until middle line sensor hits middle line
            turning = 1;
            LeftWheelReverse();
            RightWheelForward(); 
            _delay_ms(500);  

            AvgReadAllADCValues(&data);
            while (data.readings[MIDDLE_LINE_SENSOR] < THRESHOLD)
               AvgReadAllADCValues(&data);
            StopLeftWheel();
            StopRightWheel(); 
            turning = 0;
            //_delay_ms(1000);

            state = 6; 
            crossCount = 0;
            blackOn = 0;
            break;
         case 6:
            if (data.readings[MIDDLE_LINE_SENSOR] > THRESHOLD)
            {
               LeftWheelForward();
               RightWheelForward();
            }
            else if (data.readings[INNER_LEFT_LINE_SENSOR] > THRESHOLD)
            {  
               StopLeftWheel();
               RightWheelForward();
               _delay_ms(75); 
            }
            else if (data.readings[INNER_RIGHT_LINE_SENSOR] > THRESHOLD)
            {
               StopRightWheel();
               LeftWheelForward();
               _delay_ms(75);
            }
            
            break;
         case 7: //reverse direction for a bit 
            state = 8; 
            cli();
            TIFR1 |= (1 << OCF1A);
            TCNT1 = 0;
            timer1Behavior = FollowThree; 
            OCR1A = 15000; 
            TIMSK1 = (1 << OCIE1A);
            sei(); 
            break;
         case 8: //reverse direction for a bit
            if (data.readings[MIDDLE_LINE_SENSOR] > THRESHOLD)
            {
               LeftWheelReverse();
               RightWheelReverse(); 
            }
            else if (data.readings[INNER_LEFT_LINE_SENSOR] > THRESHOLD)
            {  
               LeftWheelReverse();
               StopRightWheel();
               _delay_ms(75); 
            }
            else if (data.readings[INNER_RIGHT_LINE_SENSOR] > THRESHOLD)
            {
               StopLeftWheel();
               RightWheelReverse();;
               _delay_ms(75);
            }
            break;
         case 9: //Move the claw up
            StopLeftWheel();
            StopRightWheel();
            MoveClawUp();
            state = 10;
            break;
         case 10: //Go forward to line
            if (data.readings[MIDDLE_LINE_SENSOR] > THRESHOLD)
            {
               LeftWheelForward();
               RightWheelForward();
            }
            else if (data.readings[INNER_LEFT_LINE_SENSOR] > THRESHOLD)
            {  
               StopLeftWheel();
               RightWheelForward();
               _delay_ms(75); 
            }
            else if (data.readings[INNER_RIGHT_LINE_SENSOR] > THRESHOLD)
            {
               StopRightWheel();
               LeftWheelForward();
               _delay_ms(75);
            }

            if (data.readings[INNER_RIGHT_LINE_SENSOR] > THRESHOLD && data.readings[INNER_LEFT_LINE_SENSOR] > THRESHOLD)
            {
               state = 11;
            }

            break;
         case 11:
            StopLeftWheel();
            StopRightWheel();
            //_delay_ms(1000);
            state = 12; 
            cli();
            TIFR1 |= (1 << OCF1A);
            TCNT1 = 0;
            timer1Behavior = FollowTwo; 
            OCR1A = 30000; 
            TIMSK1 = (1 << OCIE1A);
            sei(); 
            break;
         case 12:
            if (data.readings[MIDDLE_LINE_SENSOR] > THRESHOLD)
            {
               LeftWheelForward();
               RightWheelForward();
            }
            else if (data.readings[INNER_LEFT_LINE_SENSOR] > THRESHOLD)
            {  
               StopLeftWheel();
               RightWheelForward();
               _delay_ms(75); 
            }
            else if (data.readings[INNER_RIGHT_LINE_SENSOR] > THRESHOLD)
            {
               StopRightWheel();
               LeftWheelForward();
               _delay_ms(75);
            }
            break;
         case 13:
            
            FullyOpenClaw(); 
            state = 14;
            break;
         case 14:

            if (data.readings[MIDDLE_LINE_SENSOR] > THRESHOLD)
            {
               LeftWheelReverse();
               RightWheelReverse(); 
            }
            else if (data.readings[INNER_LEFT_LINE_SENSOR] > THRESHOLD)
            {  
               LeftWheelReverse();
               StopRightWheel();
               _delay_ms(75); 
            }
            else if (data.readings[INNER_RIGHT_LINE_SENSOR] > THRESHOLD)
            {
               StopLeftWheel();
               RightWheelReverse();;
               _delay_ms(75);
            }

            if  (data.readings[INNER_LEFT_LINE_SENSOR] > THRESHOLD && 
                  data.readings[INNER_RIGHT_LINE_SENSOR] > THRESHOLD)
            {
               state = 15; 
               StopLeftWheel();
               StopRightWheel();
            }
            break;
         case 15:
            turning = 1;
            LeftWheelReverse();
            RightWheelForward(); 
            _delay_ms(500);  

            AvgReadAllADCValues(&data);
            while (data.readings[MIDDLE_LINE_SENSOR] < THRESHOLD)
               AvgReadAllADCValues(&data);
            StopLeftWheel();
            StopRightWheel(); 
            MoveClawDown();
            turning = 0;
            //_delay_ms(1000);
            crossCount = 0;
            blackOn = 0;
            state = 0; 
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
   {
      MoveForwardTick(); 
      _delay_ms(500);
   }
}

void MoveForwardTick()
{
   int count = 0;
   int state = 0;
   uint32_t reading = 0; 

   LeftWheelForward();
   RightWheelForward(); 

   reading = AvgReadDigLineSensor(DIG_LINE_SENSOR_RIGHT); 

   if (reading < WHITE_THRESHOLD)
      state = 0;
   else if (reading > WHITE_THRESHOLD)
      state = 1; 
   
   //on black
   if (state)
   {
      //Get to white
      while (reading > WHITE_THRESHOLD)
         reading = AvgReadDigLineSensor(DIG_LINE_SENSOR_RIGHT); 

   } 
   
   //Get to black
   while (reading < BLACK_THRESHOLD) //go to next black line on wheel
   {
      reading = AvgReadDigLineSensor(DIG_LINE_SENSOR_RIGHT); 
   }  
     
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

uint32_t AvgReadDigLineSensor(uint8_t port)
{
   uint32_t sum = 0;
   uint16_t count = 0;
   uint16_t samples = 10; 

   for (; count < samples; ++count)
      sum += ReadDigLineSensor(port); 

   sum /= samples; 

   return sum; 
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
      while (ADCSRA & (1 << ADSC))  //wait for measurement to complete
         ;
      data->readings[ind] = ADCH; //add the reading to the data
      //ADCSRA &= ~(1 << ADIF); //clear interrupt flag 
      ++ind;
   } 
} 

void AvgReadAllADCValues(adcData *data)
{
   uint16_t ind = 0;
   uint16_t samples = 0;
   uint16_t max_sample = 2;
   
   for (ind = 0; ind < data->len; ++ind)
      data->readings[ind] = 0;

   while (samples < max_sample)
   {
      ind = 0;

      //read all desired adc values into the adcData struct
      while (ind < data->len) 
      { 
         //Select the current adc mux value
         ADMUX &= ~(0xF); //clear lower 4 bits, which are the mux values
         ADMUX |= ind; //set the correct mux value 
         ADCSRA |= (1 << ADSC); //Start measuring! 
         while (ADCSRA & (1 << ADSC))  //wait for measurement to complete
            ;
         data->readings[ind] += ADCH; //add the reading to the data
         //ADCSRA &= ~(1 << ADIF); //clear interrupt flag 
         ++ind;
      } 

      ++samples; 
   }

   for (ind = 0; ind < data->len; ++ind)
      data->readings[ind] /= samples; 
}

//Deprecated
//Setup timer for claw opening / closing
//The specific WGM values determine the mode of operation, in this case PWM
//The COM bits invert the PWM
//The CS bits set the prescaler for the timer
//The ICR1 register is the top bit
//The OCR1A bit is where the first toggle occurs, then the line goes low
//again when ICR1 is hit. 
/*void SetupClaw()
{
   TCCR1A |= 1 << WGM11 | 1 << COM1A1 | 1 << COM1A0 ;
   TCCR1B |= 1 << WGM13 | 1 << WGM12 | 1 << CS10 | 1 << CS12; //clock prescaler of 1024
   ICR1 = 313; //This will prevent anything from happening for now
}*/

void OpenClaw()
{
   timer1Behavior = OpenClawISRHandler;
   TCNT1 = 0;
   OCR1A = 31;
   TIMSK1 = (1 << OCIE1A);
}

void FullyOpenClaw()
{
   OpenClaw();
   _delay_ms(4000);
   StopClawOutput();
}

void CloseClaw()
{
   timer1Behavior = CloseClawISRHandler;
   TCNT1 = 0;
   OCR1A = 15; 
   TIMSK1 = (1 << OCIE1A);
}

void FullyCloseClaw()
{
   CloseClaw();
   _delay_ms(4000);
   StopClawOutput();
}


void StopClawOutput()
{
      TIMSK1 &= ~(1 << OCIE1A);
} 

void LeftWheelForward()
{
    leftWheelToggle = LEFT_MOTOR_RED_WIRE; //drive red wire high
    PORTB &= ~(LEFT_MOTOR_BLACK_WIRE); //drive black wire low
}

void LeftWheelReverse()
{
    leftWheelToggle = LEFT_MOTOR_BLACK_WIRE; 
    PORTB &= ~(LEFT_MOTOR_RED_WIRE); 
}

void RightWheelForward()
{
    rightWheelToggle = RIGHT_MOTOR_RED_WIRE;
    PORTB &= ~(RIGHT_MOTOR_BLACK_WIRE); 
}

void RightWheelReverse()
{
   rightWheelToggle = RIGHT_MOTOR_BLACK_WIRE; 
   PORTB &= ~(RIGHT_MOTOR_RED_WIRE); 
}

void StopLeftWheel()
{
   leftWheelToggle = 0; 
   PORTB &= ~(LEFT_MOTOR_BLACK_WIRE); 
   PORTB &= ~(LEFT_MOTOR_RED_WIRE); 
}

void StopRightWheel()
{
   rightWheelToggle = 0; 
   PORTB &= ~(RIGHT_MOTOR_BLACK_WIRE); 
   PORTB &= ~(RIGHT_MOTOR_RED_WIRE); 
}
