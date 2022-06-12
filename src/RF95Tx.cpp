// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX

#include <SPI.h>
#include <RH_RF95.h>

/* for feather32u4 
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
*/

/* for feather m0  
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
*/

/* for shield 
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 7
*/

/* Feather 32u4 w/wing
#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     2    // "SDA" (only SDA/SCL/RX/TX have IRQ!)
*/

#if defined(TEENSYDUINO)
  /* Teensy 3.x w/wing */
  #define RFM95_RST     9    // "D" yellow
  #define RFM95_CS      10   // "E" white
  #define RFM95_INT     4    // "B" blue

#else
//Kit's Feather m0 express bread board wing 
#define RFM95_RST     11   // "D" yellow
#define RFM95_CS      10   // "E" white
#define RFM95_INT     6    // "B" blue
#endif


// #if defined(ESP8266)
//   /* for ESP w/featherwing */ 
//   #define RFM95_CS  2    // "E"
//   #define RFM95_RST 16   // "D"
//   #define RFM95_INT 15   // "B"

// #elif defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
//   // Feather M0 w/Radio
//   #define RFM95_CS      8
//   #define RFM95_INT     3
//   #define RFM95_RST     4

// #elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_NRF52840_FEATHER) || defined(ARDUINO_NRF52840_FEATHER_SENSE)
//   #define RFM95_INT     9  // "A"
//   #define RFM95_CS      10  // "B"
//   #define RFM95_RST     11  // "C"
  
// #elif defined(ESP32)  
//   /* ESP32 feather w/wing */
//   #define RFM95_RST     27   // "A"
//   #define RFM95_CS      33   // "B"
//   #define RFM95_INT     12   //  next to A

// #elif defined(ARDUINO_NRF52832_FEATHER)
//   /* nRF52832 feather w/wing */
//   #define RFM95_RST     7   // "A"
//   #define RFM95_CS      11   // "B"
//   #define RFM95_INT     31   // "C"
  
// #elif defined(TEENSYDUINO)
//   /* Teensy 3.x w/wing */
//   #define RFM95_RST     9   // "A"
//   #define RFM95_CS      10   // "B"
//   #define RFM95_INT     4    // "C"
// #endif

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

volatile int gLastFrameNumber = 0;
volatile int gLastRawFrameNumber = -1;
volatile int32_t gPrevFrameTick = -1;

extern "C"
{
extern void usb_isr(void);
extern void usb_start_sof_interrupts(int interface);

void USBHandlerHook(void)
{
int frameNumber = USB1_FRINDEX;
//USB1_USBSTS = USB_USBSTS_SRI; // clear prior SOF
if(!(frameNumber & 0x07))
   {
   //1 ms SOF frame   
      //Measure phase using Cortex cpu timer. Convert to 0.25 us ticks using a runtime multiply and compile time divides for speed.
   //int32_t frameTick = ((SysTick->LOAD  - SysTick->VAL)*(kHighSpeedTimerTicksPerus*1024*1024/(VARIANT_MCK/1000000)))>>20;
   int32_t frameTick = micros();
   gLastFrameNumber = frameNumber >> 3;
   gPrevFrameTick = frameTick;
   }
gLastRawFrameNumber = frameNumber;
usb_isr();
}

void USB1_SetHandler(void (*pf_isr)(void))
{
//__disable_irq();
NVIC_DISABLE_IRQ(IRQ_USB1);
attachInterruptVector(IRQ_USB1, &USBHandlerHook);
usb_start_sof_interrupts(NUM_INTERFACE);
NVIC_ENABLE_IRQ(IRQ_USB1);

//usb_start_sof_interrupts(NUM_INTERFACE);
///__enable_irq();
}

}


void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  pinMode(RFM95_CS, OUTPUT);          // Chip Select is an output

   USB1_SetHandler(&USBHandlerHook);

  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }

  delay(100);

#if defined(TEENSYDUINO)
  Serial.println("Teensy LoRa TX Test!");
#elif defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
   Serial.println("Teensy LoRa TX Test!");
#endif


  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop()
{
   //if(gLastFrameNumber == 2000)
      {
      Serial.println("USB Frame:"+String(gLastFrameNumber)+" at: "+String(gPrevFrameTick)+" raw frame:"+String(gLastRawFrameNumber));   
      }
   delay(500);

//   delay(1000); // Wait 1 second between transmits, could also 'sleep' here!
//   Serial.println("Transmitting..."); // Send a message to rf95_server
  
//   char radiopacket[20] = "Hello World #      ";
//   itoa(packetnum++, radiopacket+13, 10);
//   Serial.print("Sending "); Serial.println(radiopacket);
//   radiopacket[19] = 0;
  
//   Serial.println("Sending...");
//   delay(10);
//   rf95.send((uint8_t *)radiopacket, 20);

//   Serial.println("Waiting for packet to complete..."); 
//   delay(10);
//   rf95.waitPacketSent();
//   // Now wait for a reply
//   uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
//   uint8_t len = sizeof(buf);

//   Serial.println("Waiting for reply...");
//   if (rf95.waitAvailableTimeout(1000))
//   { 
//     // Should be a reply message for us now   
//     if (rf95.recv(buf, &len))
//    {
//       Serial.print("Got reply: ");
//       Serial.println((char*)buf);
//       Serial.print("RSSI: ");
//       Serial.println(rf95.lastRssi(), DEC);    
//     }
//     else
//     {
//       Serial.println("Receive failed");
//     }
//   }
//   else
//   {
//     Serial.println("No reply, is there a listener around?");
//   }

}