
#define PIT_USBLOCKTIMER_IDX 2

extern void SetupUSBHook();
extern void PitInit(int timerIdx, uint32_t cycles);

extern volatile int32_t gPrevFrameTick;
extern volatile int gLastPLLControlVal;

const int kHighSpeedTimerTicksPerus = 4;
const int kHighSpeedTimerTicksPerUSBFrame = 1000*kHighSpeedTimerTicksPerus;
