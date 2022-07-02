#include "Arduino.h"
//#include <cmath>
//#include <vector>
#include <cstdint>

#include "usb-locking.h"

//Bug in imxrt.h
#define CCM_ANALOG_PLL_SYS_DIV_SELECT_FIXED		((uint32_t)(1))

#define PIT_USBLOCKTIMER_CLK_HZ (12000000)

extern volatile uint32_t F_BUS_ACTUAL;

//volatile int gLastFrameNumber = 0;
volatile int gLastRawFrameNumber = -1;  //8kHz micro frames
volatile int32_t gPrevFrameTick = -1;

const int32_t kUSBFramePeriodus = 1000;
const int kUSBFrameBits = 11; //USB frame count is 11 bits
const int32_t kUSBFrameCountMask = (1 << kUSBFrameBits) - 1;
const int32_t kUSBFrameMSBitMask = 1 << (kUSBFrameBits - 1);


volatile int16_t gLastFrameNumber = 0;

volatile int gLastPLLControlVal = 0;

volatile bool gUSBBPinState = false;

const int kHighSpeedTimerTicksPerus = 4;
const int kHighSpeedTimerTicksPerUSBFrame = 1000*kHighSpeedTimerTicksPerus;

const int kOneOverLeadGainus = 1;   // 1/proportional gain

#if 1   //defined(__SAMD51__) || defined(__TEESNSY41__)
   const int kOneOverLagGainus = 4096; // 1/integral gain
   const int kOneOverClippedLeadGainus = 4; 
#else
   const int kOneOverLagGainus = 2048; // 1/integral gain
   const int kOneOverClippedLeadGainus = 1; 
#endif
const int kFixedPointScaling = kOneOverLagGainus*kHighSpeedTimerTicksPerus;

const int kPLLOffsetMax = 127;
const int kPLLOffsetMin = -128;


//Integrator for integral feedback to remove DC error
volatile int32_t sPSDPhaseAccum = 0;

//First order LPF for lead (proportional) feedback
volatile int32_t gLeadPhaseAccum = 0;
const int kLeadPhaseTC = 16;

volatile int32_t gLastUSBSOFTimeus = 0;

inline void SetPllSysFreqOffset(int32_t offsetPartsPer2p16)
{
CCM_ANALOG_PLL_SYS_NUM = offsetPartsPer2p16 & 0x3fffffff; //units of 1 part in 64*1024
}


extern "C"
{
//extern void sys_pll_start();

extern void usb_isr(void);
extern void usb_start_sof_interrupts(int interface);

void USBHandlerHook(void)
{
int rawFrameNumber = USB1_FRINDEX;
//USB1_USBSTS = USB_USBSTS_SRI; // clear prior SOF
if(!(rawFrameNumber & 0x07))
   {
   //1 ms SOF frame   
   //Measure phase using PIT timer. Convert to 0.25 us ticks using a runtime multiply and compile time divides for speed.
   //int32_t frameTick = ((SysTick->LOAD  - SysTick->VAL)*(kHighSpeedTimerTicksPerus*1024*1024/(VARIANT_MCK/1000000)))>>20;
   int32_t frameTick = ((IMXRT_PIT_CHANNELS[PIT_USBLOCKTIMER_IDX].LDVAL - IMXRT_PIT_CHANNELS[PIT_USBLOCKTIMER_IDX].CVAL)*(kHighSpeedTimerTicksPerus*1024*1024/(PIT_USBLOCKTIMER_CLK_HZ/1000000)))>>20;

   int32_t newUSBSOFTimeus = micros();

   int16_t frameNumber = rawFrameNumber >> 3;

      int phase = frameTick;
      //phase needs to be bipolar, so wrap values above kHighSpeedTimerTicksPerUSBFrame/2 to be -ve. We want to lock with frameHSTick near 0.
      if(phase >= kHighSpeedTimerTicksPerUSBFrame/2)
         phase -= kHighSpeedTimerTicksPerUSBFrame;

      //First order LPF for lead (proportional) feedback (LPF to reduce the effects of phase detector noise)
      gLeadPhaseAccum += phase;
      int leadPhase = gLeadPhaseAccum/kLeadPhaseTC;
      gLeadPhaseAccum -= leadPhase;

      //Unfiltered lead feedback clipped to +/- 1 to reduce the effects of phase detector noise without adding delay
      int signOfPhase = 0;
      if(phase > 0)
         signOfPhase = 1;
      else if(phase < 0)
        signOfPhase = -1;

      //Calculate the filtered error signal
      int32_t filterOut = (signOfPhase*kFixedPointScaling/kOneOverClippedLeadGainus + 
         leadPhase*kFixedPointScaling/(kOneOverLeadGainus*kHighSpeedTimerTicksPerus) + 
         sPSDPhaseAccum)/kFixedPointScaling;
      sPSDPhaseAccum += phase; //integrate the phase to get lag (integral, 2nd order) feedback

      //Clip to limits of DCO
      if(filterOut > kPLLOffsetMax)
         filterOut = kPLLOffsetMax;
      else if(filterOut < kPLLOffsetMin)
         filterOut = kPLLOffsetMin;

      //int32_t newDCOControlVal = kDFLLFineMax - filterOut;

      int32_t newPLLControlVal = -filterOut;
      gLastPLLControlVal = newPLLControlVal;

      SetPllSysFreqOffset(newPLLControlVal);

      //Set DCO control value
      #ifdef PHASE_LOCK_TO_USB_SOF
      #if defined(__SAMD51__)
      OSCCTRL->DFLLVAL.bit.FINE = newDCOControlVal & 0xff;
      #else
      //SAMD21 has 10 bit fine DCO control
      SYSCTRL->DFLLVAL.bit.FINE = newDCOControlVal & 0x3ff;
      #endif
      #endif

   gLastFrameNumber = frameNumber;
   gLastUSBSOFTimeus = newUSBSOFTimeus;

   gPrevFrameTick = frameTick;


   //int32_t frameTick = micros();
   gLastFrameNumber = frameNumber;
   gPrevFrameTick = frameTick;
   }
gLastRawFrameNumber = rawFrameNumber;
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

} //extern "C"


void PitInit(int timerIdx, uint32_t cycles)
{
  CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);
  PIT_MCR = 0;

  IMXRT_PIT_CHANNELS[timerIdx].LDVAL = cycles-1;
  IMXRT_PIT_CHANNELS[timerIdx].TCTRL = PIT_TCTRL_TEN;
}


void SetupUSBHook()
{
//we want 1 kHz
PitInit(PIT_USBLOCKTIMER_IDX, 12*1000);

USB1_SetHandler(&USBHandlerHook);

SetPllSysFreqOffset(0);

CCM_ANALOG_PLL_SYS_DENOM = (64*1024)/22;
}

