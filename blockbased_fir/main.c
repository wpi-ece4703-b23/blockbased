#include "xlaudio.h"
#include "xlaudio_armdsp.h"

#define BLOCKSIZE 8
#define NUMTAPS 58

q15_t taps[NUMTAPS + BLOCKSIZE - 1];
arm_fir_instance_q15 F;

typedef q15_t int16_T;

#include "fdacoefs.h"

void processBuffer(uint16_t x[BLOCKSIZE], uint16_t y[BLOCKSIZE]) {
    q15_t xq[BLOCKSIZE], yq[BLOCKSIZE];
    xlaudio_adc14_to_q15_vec(x, xq, BLOCKSIZE);
    arm_fir_q15(&F, xq, yq, BLOCKSIZE);
    xlaudio_q15_to_dac14_vec(yq, y, BLOCKSIZE);
}

// This is the interrupt-based (per-sample) filter operation for comparison
// Note that you cannot simultaneously run the DMA-based scheme and interrupt-based scheme
uint16_t processSample(uint16_t x) {
      q15_t v = xlaudio_adc14_to_q15(x);
      taps[0] = v;
      int i;
      q15_t q = 0;
      for (i=0; i<NUMTAPS; i++)
          q = q + (taps[i] * B[i] >> 15);
      for (i=NUMTAPS-1; i>0; i--)
          taps[i] = taps[i-1];
      return xlaudio_q15_to_dac14(q);
}


#include <stdio.h>

int main(void) {
    WDT_A_hold(WDT_A_BASE);

    arm_fir_init_q15(&F, NUMTAPS, (int16_t *) B, taps, BLOCKSIZE);

    xlaudio_init_dma(FS_8000_HZ, XLAUDIO_J1_2_IN, BUFLEN_8, processBuffer);
    uint32_t c = xlaudio_measurePerfBuffer(processBuffer);
    printf("DMA Cycles: %d\n", c);

    // This is what you'd write in an interrupt-driven IO scheme:
    //
    //        xlaudio_init_intr(FS_8000_HZ, XLAUDIO_J1_2_IN, processSample);
    //        uint32_t c = xlaudio_measurePerfSample(processSample);
    //        printf("INT Cycles: %d\n", c);

    xlaudio_run();

    return 1;
}

