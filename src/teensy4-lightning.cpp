/*
This Teensy 4.1 firmware contains two different timing examples.

INTERRUPT_TIMER uses interrupts to call a mock data sampling function.
See https://www.pjrc.com/teensy/td_timing_IntervalTimer.html

An alternative is to define ADC_TIMER. We use this library to directly 
connect to the ADC for improved timing. This example takes input data on pin A0.

Documentation for the ADC library is at:
https://forum.pjrc.com/threads/25532-ADC-library-update-now-with-support-for-Teensy-3-1
http://pedvide.github.io/ADC/docs/Teensy_4_html/class_a_d_c___module.html#ab65bd1bb76ab7fbf4743c0e1bf456cb7
See also src/ADCExample.io for a concise example.

For a pin layout see https://www.pjrc.com/teensy-4-1-released/.
*/

//Default USB
//USB\VID_16C0&PID_0483&MI_00\7&11FE0FF2&0&0000


//DUAL_USB:
//USB\VID_16C0&PID_048B&MI_02\7&253AF44&0&0002
//USB\VID_16C0&PID_048B&MI_00\7&253AF44&0&0000



// One of these two timing options should be commented out.
// #define INTERRUPT_TIMER 
#define ADC_TIMER

#define OUTPUT_USB_SOF_PLL_SIGNALS 1

#define __get_PRIMASK __get_primask
#define __set_PRIMASK __set_primask


#include "Arduino.h"
#include <cmath>
#include <vector>
#include <cstdint>
#include <util/atomic.h>

#include "RingBufferSized.h"
#include "ADIPackets.h"

#ifdef ADC_TIMER

#include <ADC.h>
#include <ADC_util.h>

#include <SPI.h>
#include <RH_RF95.h>


#include "usb-locking.h"

#define SERIALDEBUG SerialUSB1

bool gIsTx = true;

int LEDpin = 5;
int PWMpin = 19;
const int readPin1 = A0;
const int readPin2 = A2;

#endif // ADC_TIMER

// const int kADCChannels = 2;
const char *kSerialNumber = "00001";
const char *kFWVersion = "0.0.1";
const int ledPin = 13; // the pin with a LED
int ledState = LOW;
volatile double gPhase = 0;
std::vector<double> gGains(kADCChannels, 1.0); // no gain set for now.
const int kDefaultADCPointsPerSec = 100;
int gADCPointsPerSec = kDefaultADCPointsPerSec; //~5000 max with 2 samples (1 point) per packet
const int kSampleRates[] = {10000, 4000, 2000, 1000, 400, 200, 100};
const int kNSampleRates = sizeof(kSampleRates) / sizeof(int);
const int kSamplePeriodus = 1.0 / kDefaultADCPointsPerSec * 1e6 ;
const int kADCStartChan = 2; //A1
const int kADCEndChan = kADCStartChan + kADCChannels;
const int kBytesPerSample = sizeof(int16_t);

//Statically allocating individual buffers larger than this causes the firmware to crash for some reason
const int kTotalBufferSpaceBytes = kADCChannels < 2 ? 32000 : 64000;
const int kBufferPoints = kTotalBufferSpaceBytes / kBytesPerSample / kADCChannels;
typedef RingBufferSized<int16_t, kBufferPoints> TRingBuf;
TRingBuf gSampleBuffers[kADCChannels];

#ifdef INTERRUPT_TIMER
IntervalTimer interruptTimer;
#endif // INTERRUPT_TIMER

#ifdef ADC_TIMER
ADC *adc = new ADC(); // adc object;
#endif                // ADC_TIMER

enum ADIDeviceSynchModes
{
   kDeviceSynchNone = 0 | 0,
   kDeviceSyncRoundTrip = 1 | 0,
   kDeviceSyncUSBFrameTimes = 2 | 0,
   kDeviceSynchUSBLocked = 4 | 0,
   kDeviceSynchUSBStartOnSpecifiedFrame = 8 | 0,
   kDeviceSynchUSBFullSuppport = kDeviceSyncRoundTrip|kDeviceSyncUSBFrameTimes|kDeviceSynchUSBLocked|kDeviceSynchUSBStartOnSpecifiedFrame|0,
   kDeviceSyncUSBFrameTimesRT = kDeviceSyncRoundTrip | kDeviceSyncUSBFrameTimes
};



void doStart(int freq);
void doStop();

void debugNewLine()
{
  //Serial.write('\n'); //Readability while testing only!
}

inline uint32_t saveIRQState(void)
{
  uint32_t pmask = __get_PRIMASK() & 1;
  __set_PRIMASK(1);
  return pmask;
}

inline void restoreIRQState(uint32_t pmask)
{
  __set_PRIMASK(pmask);
}

//volatile bool gUSBBPinState = false;
const int outputTestPin = 2;

void mockSampleData()
{
  const double kPi = 3.1415926535897932384626433832795;
  const double kSinewaveFreqHz = 1;
  const double kDefaultSampleRate = 100;
  const double kSamplerClockRateOffset = 0.0;
  const double radsPerSamplePerHz = kSinewaveFreqHz * 2 * kPi / kDefaultSampleRate * (1.0 - kSamplerClockRateOffset);
  const int kFullScaleADCUnits = 0x7fff; //16 bits
  const double kAmp = kFullScaleADCUnits; //

  for (int i(0); i < kADCChannels; ++i) // One sample per input per timer tick. Driven by timer
  {
    const double measuredAmp = kAmp * gGains[i];
    double result = measuredAmp * sin(gPhase);

    if ((i & 1)) //Square wave on odd channels
      result = std::signbit(result) ? -measuredAmp : measuredAmp;

    int16_t iResult = result;
    gSampleBuffers[i].Push(iResult);
  }
  gPhase += radsPerSamplePerHz;
  gPhase = fmod(gPhase, 2 * kPi);
}

const int kMaxCommandLenBytes = 64;

volatile int32_t gFirstADCPointus = 0;

volatile int32_t gLastTxTick = 0;

enum State
{
  kIdle,
  kWaitingForUSBSOF,
  kStartingSampling,
  kHadFirstSample,
  kSampling,
};

volatile State gState = kIdle;
volatile bool gFirstSampleTimeRequested = false;

volatile bool gADCstate = false;

volatile bool gTXPrepared = false;

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

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT, hardware_spi);

int16_t packetnum = 0;  // packet counter, we increment per xmission


void SetupRadio()
{
 // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  rf95.setSPIFrequency(RHGenericSPI::Frequency8MHz);

  while (!rf95.init()) {
    SERIALDEBUG.println("LoRa radio init failed");
    SERIALDEBUG.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  SERIALDEBUG.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    SERIALDEBUG.println("setFrequency failed");
    while (1);
  }
  SERIALDEBUG.print("Set Freq to: "); SERIALDEBUG.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);
  rf95.setPreambleLength(6);

}

void gOnUSBSOF(int16_t frameNumber)
{
if(gTXPrepared && (frameNumber & 0x3ff) == 2)
   {
   gLastTxTick = USBTimerTick();
   rf95.transmit();      
   gTXPrepared = false;
   }
}

void loopRadio()
   {
   if(gIsTx)
      {
      static uint16_t sLastFrameNumber = gLastFrameNumber;

      if(gLastFrameNumber & 0x3ff)
         return;

      if(gLastFrameNumber == sLastFrameNumber)
         return;

      sLastFrameNumber = gLastFrameNumber;

   //delay(1000); // Wait 1 second between transmits, could also 'sleep' here!
   //Serial.println("Transmitting..."); // Send a message to rf95_server
   
   //char radiopacket[20] = "Hello World #      ";
   //itoa(packetnum++, radiopacket+13, 10);
   ////Serial.print("Sending "); Serial.println(radiopacket);
   //radiopacket[19] = 0;
   
   //Serial.println("Sending...");
   //delay(10);
   //rf95.send((uint8_t *)radiopacket, 20);
   //rf95.prepareToTx((uint8_t *)radiopacket, 20);
   rf95.prepareToTx((uint8_t *)&sLastFrameNumber, 2);
   gTXPrepared = true;
   return;
   }
else
   {
   //SERIALDEBUG.println("Waiting for packet to complete..."); 
   //delay(10);
   //rf95.waitPacketSent();
   // Now wait for a reply
   uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
   uint8_t len = sizeof(buf);

   //SERIALDEBUG.println("Waiting for reply...");
   //if (rf95.waitAvailableTimeout(1000))
      { 
      // Should be a reply message for us now   
      if (rf95.recv(buf, &len))
         {
         SERIALDEBUG.print("Rx packet at [us]: ");
         SERIALDEBUG.println(rf95.GetLastRxPacketTick()/4);
         //SERIALDEBUG.println((char*)buf);
         //SERIALDEBUG.print("RSSI: ");
         //SERIALDEBUG.println(rf95.lastRssi(), DEC);    
         }
      else
         {
         //SERIALDEBUG.println("Receive failed");
         }
      }
   // else
   //    {
   //    SERIALDEBUG.println("No reply, is there a listener around?");
   //    }

   }

}


void setup()
{
  //auto irqState = saveIRQState();

//https://forum.pjrc.com/threads/59828-Teensy-4-Set-interrupt-priority-on-given-pins
//they all default to 128 (except the ones that have different defaults, 64 for serial ports, 32 for systick, etc...)

   SetupUSBHook();

  //restoreIRQState(irqState);

pinMode(RFM95_RST, OUTPUT);
digitalWrite(RFM95_RST, HIGH);
  
pinMode(RFM95_CS, OUTPUT);          // Chip Select is an output

SerialUSB1.begin(0);

  Serial.begin(0); //baud rate is ignored
  while (!Serial)
    ;

  Serial.setTimeout(50);

//SERIALDEBUG.println("IRQ_USB1 priority: "+String(NVIC_GET_PRIORITY(IRQ_USB1)));
//SERIALDEBUG.println("IRQ_USBPHY0 priority: "+String(NVIC_GET_PRIORITY(IRQ_USBPHY0)));


  SetupRadio();

SERIALDEBUG.println("IRQ_GPIO6789 priority: "+String(NVIC_GET_PRIORITY(IRQ_GPIO6789)));
SERIALDEBUG.println("IRQ_PIT priority: "+String(NVIC_GET_PRIORITY(IRQ_PIT)));


#ifdef ADC_TIMER

  ///// ADC0 ////
  adc->adc0->setAveraging(1);                                           // set number of averages
  adc->adc0->setResolution(16);                                         // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);     // change the sampling speed

  ///// ADC1 ////
  adc->adc1->setAveraging(1);                                           // set number of averages
  adc->adc1->setResolution(16);                                         // set bits of resolution
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);     // change the sampling speed

#endif // ADC_TIMER
}

void StartSampling()
{
   SERIALDEBUG.println("Start");
  for (int chan(0); chan < kADCChannels; ++chan)
  {
    auto &buffer = gSampleBuffers[chan];
    buffer.Clear();
  }

#ifdef INTERRUPT_TIMER
  interruptTimer.begin(mockSampleData, kSamplePeriodus);
#endif // INTERRUPT_TIMER

doStart(gADCPointsPerSec);
gState = kWaitingForUSBSOF;

digitalWrite(ledPin, HIGH);
}

void StopSampling()
{
doStop();

  gState = kIdle;
  gFirstSampleTimeRequested = false;

#ifdef INTERRUPT_TIMER
  interruptTimer.end();
#endif // INTERRUPT_TIMER

  for (int chan(0); chan < kADCChannels; ++chan)
  {
    auto buffer = gSampleBuffers[chan];
    buffer.Clear();
  }
  digitalWrite(ledPin, LOW);
}

void sendFirstSampleTimeIfNeeded()
{
  if (!gFirstSampleTimeRequested)
    return;

  gFirstSampleTimeRequested = false;
  debugNewLine(); //Readability while testing only!

  FirstSampleTimePacket ftPacket(gFirstADCPointus);
  ftPacket.write(Serial);

  debugNewLine(); //Readability while testing only!
}

#ifdef ADC_TIMER

//volatile uint16_t adc_val;

void adc0_isr()
{
  int16_t adc_val = (int16_t)ADC1_R0;
  int16_t val = (adc_val << 4) - 0x8000;

#ifdef OUTPUT_USB_SOF_PLL_SIGNALS
// if(chan - kADCStartChan == 0)
//    {
//    //val = gLastBit;
//    //gLastBit = 1-gLastBit;
//    val = gPrevFrameTick;
//    if(val >= kHighSpeedTimerTicksPerUSBFrame/2)
//       val -= kHighSpeedTimerTicksPerUSBFrame;
//    }
// else if(chan - kADCStartChan == 1)
   {
   val = gLastUSBPLLErrorValue;//OSCCTRL->DFLLVAL.bit.FINE;
   }
#endif

  gSampleBuffers[0].Push(val);
  //digitalWrite(outputTestPin, gUSBBPinState = !gUSBBPinState);
  asm("DSB");
}

void adc1_isr()
{
  int16_t adc_val = (int16_t)ADC2_R0;
  int16_t val = (adc_val << 4) - 0x8000;
  #ifdef OUTPUT_USB_SOF_PLL_SIGNALS
  if(gIsTx)
     val = gLastTxTick/2;
  else
     val = rf95.GetLastRxPacketTick()/2;
  #endif
  gSampleBuffers[1].Push(val);
  //digitalWrite(outputTestPin, gUSBBPinState = !gUSBBPinState);
  asm("DSB");
}

// TODO: provide a multiplex example for when using more than 2 channels.
void doStart(int freq)
{
  adc->adc0->stopTimer();
  adc->adc0->startSingleRead(readPin1); // call this to setup everything before the Timer starts, differential is also possible
  adc->adc0->enableInterrupts(adc0_isr);

  adc->adc1->stopTimer();
  adc->adc1->startSingleRead(readPin2);
  adc->adc1->enableInterrupts(adc1_isr);

  adc->adc0->startTimer(freq); //frequency in Hz
  adc->adc1->startTimer(freq);
}

void doStop()
{
adc->adc0->stopTimer();
adc->adc1->stopTimer();
}

#endif // ADC_TIMER


void loop()
{
   loopRadio();

  int hasRx = Serial.peek();

  if (hasRx >= 0)
  {
    char cmdBuf[kMaxCommandLenBytes];
    Serial.readBytesUntil('\n', cmdBuf, kMaxCommandLenBytes);

    auto cmd = cmdBuf[0];

    // for examples of other device commands see SAMDLightning.ino
    switch (cmd)
    {
    case 'b': //begin sampling
      StartSampling();
      break;
    case 'f': //first sample time
      gFirstSampleTimeRequested = true;
      if (gState == kSampling)
        sendFirstSampleTimeIfNeeded();
      break;
    case 's': //stop sampling
      StopSampling();
      break;
   case 'n':   //return micro second time now
      {
      int32_t now = micros();
      //uint64_t now64 = micros64();
      //digitalWrite(5, HIGH);

      auto timeRequestNumber = cmdBuf[1];
      TimePacket timePacket(now, timeRequestNumber);
      timePacket.write(Serial);

      //digitalWrite(5, LOW);

      break;
      }
   case 'u':   //time of last USB SOF
      {
      auto irqState = saveIRQState(); //disable interrupts
      auto lastUSBSOFTimeus = gLastUSBSOFTimeus;
      auto lastFrameNumber = gLastFrameNumber;
      int32_t now = micros();
      restoreIRQState(irqState);
      
      auto timeRequestNumber = cmdBuf[1];
      LatestUSBFrameTimePacket packet(now, timeRequestNumber, lastFrameNumber, lastUSBSOFTimeus);
      packet.write(Serial);
      break;
      }

    case 'v': //version info as JSON
      Serial.print("{");
      Serial.print("\"deviceClass\": \"Arduino_Example\",");
      Serial.print("\"deviceName\": \"Teensy 4.1\",");
      Serial.print("\"version\": \"" + String(kFWVersion) + "\",");
      Serial.print("\"numberOfChannels\": " + String(kADCChannels) + ",");
      Serial.print("\"deviceSynchModes\": " + String(kDeviceSynchNone) + ",");
      Serial.print("\"serialNumber\": \"" + String(kSerialNumber) + "\"");
      Serial.print("}$$$");

      Packet::ResetPacketCount(); //new session

      break;
    case '~': //sample rate
    {
      auto rateChar = cmdBuf[1]; //'0123456'
      unsigned int index = rateChar - '0';
      if (index < sizeof(kSampleRates) / sizeof(int))
        gADCPointsPerSec = kSampleRates[index];
      if (gADCPointsPerSec > 100)
        gADCPointsPerPacket = kPointsPerMediumSizePacket;
      else
        gADCPointsPerPacket = kPointsPerPacket;

      break;
    }
    default:
      break;
    }
  }

  if (gState == kIdle)
    return;

  if (gState == kHadFirstSample)
  {
    gState = kSampling;
    sendFirstSampleTimeIfNeeded();
  }

  //Find the number of samples in the ringbuffer with the least samples
  int points = gSampleBuffers[0].GetCount();
  for (int chan(1); chan < kADCChannels; ++chan)
  {
    auto &buffer = gSampleBuffers[chan];
    points = min(buffer.GetCount(), points); // on each loop run see if there is enough data in the buffer to create a packet
  }

  while (points >= gADCPointsPerPacket)
  {
    Packet packet;

    for (int pt(0); pt < gADCPointsPerPacket; ++pt)
    {
      for (int chan(0); chan < kADCChannels; ++chan)
      {
        auto &buffer = gSampleBuffers[chan];
        packet.addSample(chan, buffer.GetNext()); // getting the data out of the buffer into a packet. getNext bumps the mOut.
      }
      packet.nextPoint();
    }
    packet.write(Serial);
    --points;
  }


} //loop
