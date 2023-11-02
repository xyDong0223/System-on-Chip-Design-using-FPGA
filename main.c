/*--------------------------------------------------------------------------------------------------
    Demonstration program for Cortex-M0 SoC design - basic version, no CMSIS

    Enable the interrupt for UART - interrupt when a character is received.
    Repeat:
      Set the LEDs to match the switches, then flash the 8 rightmost LEDs.
      Go to sleep mode, waiting for an interrupt
          On UART interrupt, the ISR sends the received character back to UART and stores it
              main() also shows the character code on the 8 rightmost LEDs
      When a whole message has been received, the stored characters are copied to another array
          with their case inverted, then printed.

    Version 6 - March 2023
  ------------------------------------------------------------------------------------------------*/

#include <stdio.h>                    // needed for printf
#include "DES_M0_SoC.h"            // defines registers in the hardware blocks used
#include <math.h>

#define BUF_SIZE                        100                // size of the array to hold received characters
#define ASCII_CR                        '\r'            // character to mark the end of input
#define CASE_BIT                        ('A' ^ 'a')        // bit pattern used to change the case of a letter
#define FLASH_DELAY                    1000000        // delay for flashing LEDs, ~220 ms
#define Write               0x50000000
#define Read                0x40000000
#define INVERT_LEDS                    (GPIO_LED ^= 0xff)        // inverts the 8 rightmost LEDs
#define RAD_TO_DEG          (180 / 3.14)
#define ARRAY_SIZE(__x__)   (sizeof(__x__)/sizeof(__x__[0]))  // macro to find array size

// Global variables - shared between main and UART_ISR
volatile uint8  RxBuf[BUF_SIZE];    // array to hold received characters
volatile uint8  counter  = 0;         // current number of characters in RxBuf[]
volatile uint8  BufReady = 0;         // flag indicates data in RxBuf is ready for processing
volatile int    changed = 1;
uint32  a,b,c;
uint32 a2,b2,c2;
uint32 SPIdata;
uint32 VGAdata;
uint32 Voicedata;


//////////////////////////////////////////////////////////////////
// Interrupt service routine, runs when UART interrupt occurs - see cm0dsasm.s
//////////////////////////////////////////////////////////////////
void UART_ISR()
{
    char c;
    c = UART_RXD;                     // read character from UART (there must be one waiting)
    RxBuf[counter]  = c;  // store in buffer
    counter++;            // increment counter, number of characters in buffer
    UART_TXD = c;                  // write character to UART (assuming transmit queue not full)

    /* Counter is now the position in the buffer that the next character should go into.
        If this is the end of the buffer, i.e. if counter == BUF_SIZE-1, then null terminate
        and indicate that a complete sentence has been received.
        If the character just put in was a carriage return, do the same.  */
    if (counter == BUF_SIZE-1 || c == ASCII_CR)
    {
        counter--;                            // decrement counter (CR will be over-written)
        RxBuf[counter] = NULL;  // null terminate to make the array a valid string
        BufReady       = 1;        // indicate that data is ready for processing
    }
}


//////////////////////////////////////////////////////////////////
// Interrupt service routine for System Tick interrupt
//////////////////////////////////////////////////////////////////
void SysTick_ISR()
{
    // Do nothing - this interrupt is not used here
}

//////////////////////////////////////////////////////////////////
// Delay
//////////////////////////////////////////////////////////////////
void delay (uint32 n)
{
    volatile uint32 i;
        for(i=0; i<n; i++);        // do nothing n times
}
void dealy(uint32 n) {
    volatile uint32 i;
        for(i=0;i<n;i++);
}


//////////////////////////////////////////////////////////////////
// Converting
//////////////////////////////////////////////////////////////////
uint8 radians_to_degrees(uint8 radians) {
    return radians * RAD_TO_DEG;
}
uint16 angletransfer(uint16 x,uint16 y,uint16 z){
       uint16 angle;
       angle = (atan(x/sqrt(y*y+z*z))/6.28)*360; //Convert acceleration data into angles
       return angle;
}



//////////////////////////////////////////////////////////////////
// SPI
//////////////////////////////////////////////////////////////////
typedef struct {
    volatile int read;
    volatile int write;
}SPI_block;

#define pt2SPI ((SPI_block *)0x52000000)

typedef struct{
        int16 x;
        int16 y;
        int16 z;
} Vector_int;

uint32 readSPI(uint8 command, uint8 address) {
    uint32 val ;
    
            SPIdata = (Read)+((changed)<<24)+(command)+(address<<8); //control command
            changed=~changed; // change Signal
            pt2SPI->write = SPIdata; //write data
            delay(1000);
            val = pt2SPI->read; //read data

        return val;
}

void writeSPI(uint8 command, uint8 address,uint8 data) {
    
         SPIdata = (Write)+((changed)<<24)+(command)+(address<<8)+(data<<16);//control command
         changed=~changed; // change Signal
         pt2SPI->write = SPIdata;//wirte data

}

void calculateAcceleration(Vector_int* accelerationt){
      
      writeSPI(0x0A,0x2C, 0x13 ); //set to 100hz ODR and +-2g RANGE
      writeSPI(0x0A,0x2D, 0x22 ); //set to measurement mode and Ultralow noise mode
    
      accelerationt->x = (readSPI(0x0B,0x0F)<<8); //read four most significant bits of x
      printf("high %d\n\n",readSPI(0x0B,0x0F));
      delay(100);
      accelerationt->x=accelerationt->x+readSPI(0x0B,0x0E);//read eight least significant bits of x
      printf("low %d\n",readSPI(0x0B,0x0E));
      printf("X %f\n",accelerationt->x);
    
      accelerationt->y =(readSPI(0x0B,0x11)<<8);//read four most significant bits of y
      printf("high %d\n\n",readSPI(0x0B,0x11));
      delay(100);
    
      accelerationt->y =accelerationt->y+readSPI(0x0B,0x10);//read eight least significant bits of y
      printf("low %d\n",readSPI(0x0B,0x10));
      printf("Y %f\n",accelerationt->y);
    
      accelerationt->z = (readSPI(0x0B,0x13)<<8);//read four most significant bits of z
      printf("high %d\n\n",readSPI(0x0B,0x13));
      delay(100);
      accelerationt->z=accelerationt->z+readSPI(0x0B,0x12);//read eight least significant bits of z
      printf("low %d\n",readSPI(0x0B,0x12));
      printf("Z %f\n",accelerationt->z);
}

//////////////////////////////////////////////////////////////////
//Display
//////////////////////////////////////////////////////////////////
#define DISPLAY_BASE 0x53000000
uint8* displayReg = (uint8*) DISPLAY_BASE;

void wirteDisplay(uint16 x,uint16 y,uint16 z) {
          uint32 value;
          value = angletransfer(x,y,z); // transfer data from acceleration to angle
          displayReg[9] =  0xFF; //enbale 8digitals
          displayReg[8]  = 0xFF; //Hex mode
          displayReg[0]  = value%10; //Display data in separate digits
          displayReg[1]  = (value/10)%10;
          displayReg[2]  = (value/100)%10;
          displayReg[3]  = (value/1000)%10;
          value = angletransfer(y,x,z);
          displayReg[4]  = value%10;
          displayReg[5]  = (value/10)%10;
          displayReg[6]  = (value/100)%10;
          displayReg[7]  = (value/1000)%10;
}


//////////////////////////////////////////////////////////////////
//light
//////////////////////////////////////////////////////////////////

int light(int32  angle,Vector_int* accelerationt){
        uint32 LED_INI;
    if(accelerationt->y <0){
        if (angle > 1050){
            LED_INI = 0X0001; //Turn on the corresponding light
            
        }else if (angle > 900){
            LED_INI = 0X0002;
            
        }else if  (angle > 750){
            LED_INI = 0X0004;
            
        }else if  (angle > 600){
            LED_INI = 0X0008;
            
        }else if  (angle > 450){
            LED_INI = 0X0010;
            
        }else if  (angle > 300){
            LED_INI = 0X0020;
            
        }else if  (angle > 150){
            LED_INI = 0X0040;
            
        }else if  (angle >= 0){
            LED_INI = 0X0080;
            
        
        }else{
            LED_INI = 0X0000;
        }
    }else{
        if (angle > 1050){
            LED_INI = 0X8000;
            
        }else if (angle > 900){
            LED_INI = 0X4000;
            
        }else if  (angle > 750){
            LED_INI = 0X2000;
            
        }else if  (angle > 600){
            LED_INI = 0X1000;
            
        }else if  (angle > 450){
            LED_INI = 0X0800;
            
        }else if  (angle > 300){
            LED_INI = 0X0400;
            
        }else if  (angle > 150){
            LED_INI = 0X0200;
            
        }else if  (angle > 0){
            LED_INI = 0X0100;
            
        
        }else{
            LED_INI = 0X0000;
        }
    }
            
        GPIO_LED =  LED_INI;  //Write data to the Running light block
        return GPIO_LED;
    }

//////////////////////////////////////////////////////////////////
//VGA
//////////////////////////////////////////////////////////////////
typedef struct {
    volatile int write_VGA;
}VGA_block;

#define pt2VGA ((VGA_block *)0x54000000)

void writeVGA(uint16 x, uint16 y,uint16 z,Vector_int* accelerationt) {

    VGAdata = 0;
    if(accelerationt->x < 0){
        
          VGAdata = 0x01<<31; //Graph left and right movement control command
    
    }
    if(accelerationt->y < 0){
        
        VGAdata = VGAdata + (0x01<<30); //Graph Up and Down Movement Control Command
    }
    if(accelerationt->z < 0){
        
        VGAdata = VGAdata + ((z & 0xFFF)/100); //Picture Color Change Command
    }else if(accelerationt->z >= 0){
        VGAdata = VGAdata + (((z & 0xFFF)/100)<<4);//Picture Color Change Command
    }
    VGAdata = VGAdata + (((x & 0xFFF)/7)<<16) + (((y & 0xFFF)/6)<<8);//Left and right movement and up and down movement amplitude
        pt2VGA->write_VGA = VGAdata; //write data
}

//////////////////////////////////////////////////////////////////
//Voice
//////////////////////////////////////////////////////////////////
typedef struct {
    volatile int write_Voice;
}Voice_block;
#define pt2Voice ((Voice_block *)0x55000000)

void writeVoice(uint16 x) {
       if(x>500){ // bigger than 0.5g
             Voicedata = 0x02;//control command(turn on the voice)
             pt2Voice->write_Voice=Voicedata;//write data
             delay(100000);
         }else{
             Voicedata=0; //turn off voice
             pt2Voice->write_Voice=Voicedata;//write data
             
             delay(100000);
         }
         
    
}

//////////
    ////////////////////////////////////////////////////////
// Main Function
//////////////////////////////////////////////////////////////////
int main(void)
{
    
    Vector_int accelerationt;
    int16 y_new; //Absolute value of the x-axis
    int16 x_new; //Absolute value of the y-axis
    int16 z_new; //Absolute value of the z-axis
    


// ========================  Working Loop ==========================================
    
    while(1)        // loop forever
    {
        
          calculateAcceleration(&accelerationt); //get the x,y,z-axis value from accelerometer

        if(GPIO_SW ==0x0001){ // run all blocks when switches==1
            if(accelerationt.x <0){
                x_new=-accelerationt.x; // update the absolute value of the x-axis
            }else {
                x_new=accelerationt.x;
            }
            if(accelerationt.y <0){
                y_new=-accelerationt.y;// update the absolute value of the y-axis
            }else {
                y_new=accelerationt.y;
            }
            if(accelerationt.z <0){
                z_new=-accelerationt.z;// update the absolute value of the z-axis
            }else {
                z_new=accelerationt.z;
            }
            
            writeVGA(x_new, y_new,z_new, &accelerationt); //run VGA
            light(y_new,&accelerationt); // run light
            writeVoice(x_new); // run voice
            
            wirteDisplay(x_new,y_new,z_new); //run
            
            printf("X: %f g   ",accelerationt.x);
            printf("Y: %f g  ",accelerationt.y);
            printf("Z: %f g  ",accelerationt.z);
           }
             
         delay(400000);


    } // end of infinite loop
    

    
}  // end of main

