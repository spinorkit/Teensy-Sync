
#define PIT_USBLOCKTIMER_IDX 2

extern int32_t USBTimerTick();
extern void SetupUSBHook();
extern void PitInit(int timerIdx, uint32_t cycles);

extern volatile int32_t gPrevFrameTick;
extern volatile int gLastUSBPLLErrorValue;
extern volatile int16_t gLastFrameNumber;
extern volatile int32_t gLastUSBSOFTimeus;

extern void gOnUSBSOF(int16_t frameNumber);

const int kHighSpeedTimerTicksPerus = 4;
const int kHighSpeedTimerTicksPerUSBFrame = 1000*kHighSpeedTimerTicksPerus;
