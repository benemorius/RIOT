#include <stdint.h>

#define FCFG1_BASE (0x50001000)
#define FCFG1_O_FCFG1_REVISION (0x31c)
#define AUX_DDI0_OSC_BASE (0x400ca000)
// #define DDI_0_OSC_O_AMPCOMPTH2 (0x14)
// #define DDI_0_OSC_O_AMPCOMPTH1 (0x10)
// #define DDI_0_OSC_O_AMPCOMPCTL (0xc)

#ifdef __cplusplus
extern "C" {
#endif

uint32_t GetTrimForAmpcompTh2(void);
uint32_t GetTrimForAmpcompTh1(void);
uint32_t GetTrimForAmpcompCtrl(uint32_t ui32Fcfg1Revision);

#ifdef __cplusplus
}
#endif
