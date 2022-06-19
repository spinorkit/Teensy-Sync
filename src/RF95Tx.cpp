// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX

#include <SPI.h>
#include <RH_RF95.h>

#define ADCTIMER_OUT 2

#define PIT_ADCTIMER_IDX 1

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

/*
Teensy Pin, The IMXRT pin name, and the GPIO Port.pin
Pin	Name	GPIO
 1	      AD_B0_02	1.02
 0	      AD_B0_03	1.03
24/A10	AD_B0_12	1.12
25/A11	AD_B0_13	1.13
19/A5	   AD_B1_00	1.16
18/A4	   AD_B1_01	1.17
14/A0	   AD_B1_02	1.18
15/A1	   AD_B1_03	1.19
40/A16	AD_B1_04 	1.20
41/A17	AD_B1_05 	1.21
17/A3	   AD_B1_06	1.22
16/A2	   AD_B1_07	1.23
22/A8	   AD_B1_08	1.24
23/A9	   AD_B1_09	1.25
20/A6	   AD_B1_10	1.26
21/A7	   AD_B1_11	1.27
38/A14	AD_B1_12 	1.28
39/A5	   AD_B1_13 	1.29
26/A12	AD_B1_14	1.30
27/A13	AD_B1_15	1.31
10	      B0_00	2.00
12	      B0_01	2.01
11	      B0_02	2.02
13	      B0_03	2.03
6	      B0_10	2.10
9	      B0_11	2.11
32	      B0_12	2.12
8	      B1_00	2.16
7	      B1_01	2.17
36	      B1_02    	2.18
37	      B1_03    	2.19
35	      B1_12    	2.28
34	      B1_13    	2.29
45	      SD_B0_00	3.12
44	      SD_B0_01	3.13
43	      SD_B0_02	3.14
42	      SD_B0_03	3.15
47	      SD_B0_04	3.16
46	      SD_B0_05	3.17
28	      EMC_32	3.18
31	      EMC_36	3.22
30	      EMC_37	3.23
 2	      EMC_04	4.04
 3	      EMC_05	4.05
 4	      EMC_06	4.06
33	      EMC_07	4.07
5	      EMC_08	4.08
51	      EMC_22	4.22
48	      EMC_24	4.24
53	      EMC_25	4.25
52	      EMC_26	4.26
49	      EMC_27	4.27
50	      MC_28	4.28
54	      EMC_29	4.29
29	      EMC_31	4.31
---	----	----
*/


// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//Bug in imxrt.h
#define CCM_ANALOG_PLL_SYS_DIV_SELECT_FIXED		((uint32_t)(1))

extern volatile uint32_t F_BUS_ACTUAL;

volatile int gLastFrameNumber = 0;
volatile int gLastRawFrameNumber = -1;
volatile int32_t gPrevFrameTick = -1;

extern "C"
{
//extern void sys_pll_start();

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


/*
F_CPU: 528000000
F_CPU_ACTUAL: 528000000     
F_BUS_ACTUAL: 132000000     
CCM_ANALOG_PLL_ARM: 80002058
CCM_ANALOG_PLL_SYS_SS: 0    
CCM_CACRR: 1
CCM_CBCMR: B5AE8304
CCM_CSCMR1: 66130040        
CCM_CBCDR: 180A8300
CCM_ANALOG_PLL_SYS: 80002001
CCM_ANALOG_PLL_SYS_NUM: 0   
CCM_ANALOG_PLL_SYS_DENOM: 18


Teensy LoRa TX Test!        
F_CPU: 600000000
F_CPU_ACTUAL: 600000000     
F_BUS_ACTUAL: 150000000     
CCM_ANALOG_PLL_ARM: 80002064
CCM_ANALOG_PLL_SYS_SS: 0    
CCM_CACRR: 1
CCM_CBCMR: B5AE8304
CCM_CSCMR1: 66130040
CCM_CBCDR: 180A8300
CCM_ANALOG_PLL_SYS: 80002001
CCM_ANALOG_PLL_SYS_NUM: 0
CCM_ANALOG_PLL_SYS_DENOM: 18
LoRa radio init OK!
CCM_CACRR: 1
CCM_CBCMR: B5AE8304    
CCM_CSCMR1: 66130040
CCM_CBCDR: 180A8300
CCM_ANALOG_PLL_SYS: 80002001
CCM_ANALOG_PLL_SYS_NUM: 0
CCM_ANALOG_PLL_SYS_DENOM: 18

F_BUS_ACTUAL: 150000000     
CCM_CBCMR: B5AE8304   E=>PLL1
CCM_CBCDR: 180A8300
CCM_ANALOG_PLL_SYS: 80002001    0x80000000=>isLocked, 0x2000 =>enable, 0x0001 => Fout=Fref*22.
CCM_ANALOG_PLL_SYS_NUM: 0   
CCM_ANALOG_PLL_SYS_DENOM: 18
LoRa radio init OK!

CCM_CBCMR: 3048112900 (0xB5AE 8304)  E=>PLL1
CCM_ANALOG_PLL_SYS: 2147491841
CCM_ANALOG_PLL_SYS_NUM: 0     
CCM_ANALOG_PLL_SYS_DENOM: 18  
LoRa radio init OK!

CCM_ANALOG_PLL_SYS: 2147491841 (0x8000 2001), 0x80000000=>isLocked, 0x2000 =>enable, 0x0001 => Fout=Fref*22.
CCM_ANALOG_PLL_SYS_NUM: 0     
CCM_ANALOG_PLL_SYS_DENOM: 18  
*/


/*

need to switch to alternate clock during reconfigure of ARM PLL
USB PLL is running, so we can use 120 MHz
CCM_CBCDR &= ~CCM_CBCDR_PERIPH_CLK_SEL
New Frequency: ARM=528000000, IPG=132000000
F_CPU: 528000000
F_CPU_ACTUAL: 528000000
F_BUS_ACTUAL: 132000000
CCM_ANALOG_PLL_ARM: 80002058
CCM_ANALOG_PLL_SYS_SS: 0
CCM_CACRR: 1
CCM_CBCMR: B5AE8304
CCM_CSCMR1: 66130040  //24 MHz from OSC
CCM_CBCDR: 180A8300
CCM_ANALOG_PLL_SYS: 80002001
CCM_ANALOG_PLL_SYS_NUM: 0
CCM_ANALOG_PLL_SYS_DENOM: 18


Teensy LoRa TX Test!
F_CPU: 528000000
F_CPU_ACTUAL: 528000000
F_BUS_ACTUAL: 132000000
CCM_ANALOG_PLL_ARM: 80002042
CCM_ANALOG_PLL_SYS_SS: 0
CCM_CACRR: 1
CCM_CBCMR: B5AE8304
CCM_CSCMR1: 6613000A
CCM_CBCDR: 180A8300
CCM_ANALOG_PLL_SYS: 80002001
CCM_ANALOG_PLL_SYS_NUM: 0
CCM_ANALOG_PLL_SYS_DENOM: 18
*/

/*
Change to 12 MHz from IPG_CLK_ROOT
CCM_CSCMR1: 6613000A
*/

//ADC_ETC_CTRL

void xbar_connect(unsigned int input, unsigned int output)
{
  if (input >= 88) return;
  if (output >= 132) return;
  volatile uint16_t *xbar = &XBARA1_SEL0 + (output / 2);
  uint16_t val = *xbar;
  if (!(output & 1)) {
    val = (val & 0xFF00) | input;
  } else {
    val = (val & 0x00FF) | (input << 8);
  }
  *xbar = val;
}

void xbar_init() 
{
CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);   //turn clock on for xbara1

//EMC_04 (Teensy4.1 pin 2, ADCTIMER_OUT)
//IOMUXC XBAR_INOUT06 function direction select: output
IOMUXC_GPR_GPR6 |= IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_6;

//ALT3 — Select mux mode: ALT3 mux port: XBAR1_INOUT06 of instance: xbar1 (page 430)
IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_04 = (IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_04 & ~7) | 3;

xbar_connect(57, 6); //PIT_TRIGGER1, IOMUX_XBAR_INOUT06 (XBAR1_OUT06)


//PIT_TRIGGER1, ADC_ETC_TRIG00 see pages 63,71 : Chapter 4 Interrupts, DMA Events, and XBAR Assignments
//xbar_connect(57, 103);   // pit to adc_etc

}


void pit_init(uint32_t cycles)
{
  CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);
  PIT_MCR = 0;

  IMXRT_PIT_CHANNELS[PIT_ADCTIMER_IDX].LDVAL = cycles-1;
  IMXRT_PIT_CHANNELS[PIT_ADCTIMER_IDX].TCTRL = PIT_TCTRL_TEN;
}


void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  pinMode(RFM95_CS, OUTPUT);          // Chip Select is an output

  pinMode(ADCTIMER_OUT, OUTPUT);


   USB1_SetHandler(&USBHandlerHook);

  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }

  delay(100);

  xbar_init();

  //adc_init();
  //adc_etc_init();

  pit_init(12 * 10);

#if defined(TEENSYDUINO)
  Serial.println("Teensy LoRa TX Test!");
#elif defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
   Serial.println("Teensy LoRa TX Test!");
#endif

//set_arm_clock_pll2_528();

Serial.println("F_CPU: "+String(F_CPU));
Serial.println("F_CPU_ACTUAL: "+String(F_CPU_ACTUAL));
Serial.println("F_BUS_ACTUAL: "+String(F_BUS_ACTUAL));




Serial.println("CCM_ANALOG_PLL_ARM: "+String(CCM_ANALOG_PLL_ARM, HEX));
Serial.println("CCM_ANALOG_PLL_SYS_SS: "+String(CCM_ANALOG_PLL_SYS_SS, HEX));

Serial.println("CCM_CACRR: "+String(CCM_CACRR, HEX));
Serial.println("CCM_CBCMR: "+String(CCM_CBCMR, HEX));
Serial.println("CCM_CSCMR1: "+String(CCM_CSCMR1, HEX));
Serial.println("CCM_CBCDR: "+String(CCM_CBCDR, HEX));
Serial.println("CCM_ANALOG_PLL_SYS: "+String(CCM_ANALOG_PLL_SYS, HEX));
Serial.println("CCM_ANALOG_PLL_SYS_NUM: "+String(CCM_ANALOG_PLL_SYS_NUM));
Serial.println("CCM_ANALOG_PLL_SYS_DENOM: "+String(CCM_ANALOG_PLL_SYS_DENOM));







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