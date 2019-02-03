//*************************************************************************
// FindTargetLine
// 
// Finds the line seen by the Pixy2 with an endpoint closest to the  
// center point of the screen. Based on that we 
//
//
// https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:hooking_up_pixy_to_a_microcontroller_-28like_an_arduino-29
//
// *************************************************************************

// Uncomment one of these to enable another type of serial interface
#define I2C
//#define UART
//#define SPI_SS
   
#ifdef I2C

#include <Pixy2I2C.h>
Pixy2I2C pixy;

#else 
#ifdef UART

#include <Pixy2UART.h>
Pixy2UART pixy;

#else 
#ifdef SPI_SS

#include <Pixy2SPI_SS.h>
Pixy2SPI_SS pixy;

#else

#include <Pixy2.h>
Pixy2 pixy;

#endif
#endif
#endif

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
  
  pixy.init();

  Serial.println(pixy.changeProg("line_tracking"));
}

void loop()
{
  int8_t i;
  char buf[128];
 
  pixy.line.getMainFeatures();

  if (pixy.line.numVectors)
    Serial.print("Detected ");
    pixy.line.vectors->print();
 
//      Serial.print("  block ");
//      Serial.print(i);
//      Serial.print(": ");
//      pixy.ccc.blocks[i].print();

}
