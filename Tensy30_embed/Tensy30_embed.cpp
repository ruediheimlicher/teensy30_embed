///
/// @mainpage	Tensy30_embed
///
/// @details	Description of the project
/// @n
/// @n
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Ruedi Heimlicher
/// @author		Ruedi Heimlicher
/// @date		15.11.2017 13:33
/// @version	<#version#>
///
/// @copyright	(c) Ruedi Heimlicher, 2017
/// @copyright	Licence
///
/// @see		ReadMe.txt for references
///


///
/// @file		Tensy30_embed.cpp
/// @brief		Main sketch
///
/// @details	<#details#>
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Ruedi Heimlicher
/// @author		Ruedi Heimlicher
/// @date		15.11.2017 13:33
/// @version	<#version#>
///
/// @copyright	(c) Ruedi Heimlicher, 2017
/// @copyright	Licence
///
/// @see		See Tensy30_embed.h and ReadMe.txt for references
/// @n
///


// Core library for code-sense - IDE-based
// Help: http://bit.ly/2x8HivE
#if defined(WIRING) // Wiring specific
#include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#include "WProgram.h"
#elif defined(ROBOTIS) // Robotis specific
#include "libpandora_types.h"
#include "pandora.h"
#elif defined(MPIDE) // chipKIT specific
#include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
#include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
#include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
#include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
#include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
#include "Arduino.h"
#elif defined(RFDUINO) // RFduino specific
#include "Arduino.h"
#elif defined(SPARK) || defined(PARTICLE) // Particle / Spark specific
#include "application.h"
#elif defined(ESP8266) // ESP8266 specific
#include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#include "Arduino.h"
#else // error
#error Platform not defined
#endif // end IDE

// Include application, user and local libraries
#include "lcd_teensy.h"

// Define structures and classes
#define USB_DATENBREITE 64
uint8_t loopcounter = 0;
int val;
int u;
uint8_t sendbuffer[USB_DATENBREITE] = {};

uint8_t recvbuffer[USB_DATENBREITE] = {};

#define  LED_ON      GPIOC_PSOR=(1<<5)
#define  LED_OFF     GPIOC_PCOR=(1<<5)

uint32_t         n = 0;
uint32_t               v = 0;
uint8_t                  mask;


// Define variables and constants


// Prototypes
// Help: http://bit.ly/2l0ZhTa


// Utilities


// Functions


// Add setup code
void setup()
{
   pinMode(LCD_CLOCK_PIN, OUTPUT);
   pinMode(LCD_ENABLE_PIN, OUTPUT);
   pinMode(LCD_RSDS_PIN, OUTPUT);
   
   pinMode(13, OUTPUT);
   sendbuffer[0] = 1;
   recvbuffer[USB_DATENBREITE-1] = '\0';
   sendbuffer[USB_DATENBREITE-1] = '\0';
   Serial.begin(9600); // USB is always 12 Mbit/sec
}

// Add loop code
void loop()
{
   val = analogRead(0);
   int e = usb_rawhid_recv(recvbuffer, 50);
   Serial.print("usb: e: ");
   Serial.println(e);
   for (int i=0;i<64;i++)
   {
      Serial.print(recvbuffer[i]);
      Serial.print(' ');
   }
   Serial.println();
   
   // USB
   u = usb_rawhid_available();
   Serial.print("usb: ");
   Serial.println(u);
   
   int pos = loopcounter & 0x3F;
   Serial.print("pos: ");
   Serial.println(pos);
   if (pos<63)
   {
      sendbuffer[pos] = loopcounter & 0xFF;
   }
   
   int s = usb_rawhid_send(sendbuffer, 50);
   Serial.print("send usb: ");
   Serial.println(s);
   
   for (int i=0;i<64;i++)
   {
      Serial.print(sendbuffer[i]);
      Serial.print(' ');
   }
   Serial.println();
   
   int u = usb_rawhid_recv(recvbuffer, 50);
   
   Serial.print("recv usb: ");
   Serial.println(s);
   Serial.print(' ');
   int diff = memcmp((const char*)recvbuffer,(const char*)sendbuffer,sizeof(recvbuffer));
   Serial.println(diff);
   Serial.println();
   for (int i=0;i<64;i++)
   {
      Serial.print(recvbuffer[i]);
      Serial.print(' ');
   }
   Serial.println();

   Serial.print("loopcounter: ");
   Serial.println(loopcounter);
   loopcounter++;

  
      //digitalWriteFast(13, HIGH);
   LED_ON;
      delay(100);
   LED_OFF;
   //   digitalWriteFast(13, LOW);
      delay(2500);
   
   

}
