/*
 * Filter Coefficients (C Source) generated by the Filter Design and Analysis Tool
 * Generated by MATLAB(R) 9.8 and Signal Processing Toolbox 8.4.
 * Generated on: 17-Nov-2023 13:28:07
 */

/*
 * Discrete-Time IIR Filter (real)
 * -------------------------------
 * Filter Structure    : Direct-Form II, Second-Order Sections
 * Number of Sections  : 4
 * Stable              : Yes
 * Linear Phase        : No
 */

/* General type conversion for MATLAB generated C-code  */
// #include "tmwtypes.h"
/* 
 * Expected path to tmwtypes.h 
 * C:\Program Files\MATLAB\R2020a\extern\include\tmwtypes.h 
 */
/*
 * Warning - Filter coefficients were truncated to fit specified data type.  
 *   The resulting response may not match generated theoretical response.
 *   Use the Filter Design & Analysis Tool to design accurate
 *   single-precision filter coefficients.
 */
#define MWSPT_NSEC 9
const int NL[MWSPT_NSEC] = { 1,3,1,3,1,3,1,3,1 };
const real32_T NUM[MWSPT_NSEC][3] = {
  {
      0.583255887,              0,              0 
  },
  {
                1,   -1.192962766,              1 
  },
  {
      0.583255887,              0,              0 
  },
  {
                1,   -1.584364414,              1 
  },
  {
     0.1701243967,              0,              0 
  },
  {
                1,   -1.320302367,              1 
  },
  {
     0.1701243967,              0,              0 
  },
  {
                1,   -1.499031901,              1 
  },
  {
                1,              0,              0 
  }
};
const int DL[MWSPT_NSEC] = { 1,3,1,3,1,3,1,3,1 };
const real32_T DEN[MWSPT_NSEC][3] = {
  {
                1,              0,              0 
  },
  {
                1,   -1.368279934,   0.9712641239 
  },
  {
                1,              0,              0 
  },
  {
                1,   -1.421427727,   0.9723116755 
  },
  {
                1,              0,              0 
  },
  {
                1,   -1.352174163,   0.9914537072 
  },
  {
                1,              0,              0 
  },
  {
                1,   -1.462519288,   0.9920934439 
  },
  {
                1,              0,              0 
  }
};
