/*
 * phaseLeadingedge.h
 *
 *  Created on: May 16, 2022
 *      Author: barlo
 */

#ifndef PHASELEADINGEDGE_H_
#define PHASELEADINGEDGE_H_


/*
 * EncoderHeader.h
 *
 *  Created on: Nov 16, 2021
 *      Author: henri
 */



//
// Included Files
//
#include "IQMathLib.h"         // Include header for IQmath library
//
// Typedef for the structure of the POSSPEED Object
//
typedef struct
{
    int theta_elec;         // Output: Motor Electrical angle (Q15)
    int theta_mech;         // Output: Motor Mechanical Angle (Q15)
    int DirectionQep;       // Output: Motor rotation direction (Q0)
    int QEP_cnt_idx;        // Variable: Encoder counter index (Q0)
    int theta_raw;          // Variable: Raw angle from Timer 2 (Q0)

    //
    // Parameter: 0.9999/total count, total count = 4000 (Q26)
    //
    int mech_scaler;

    int pole_pairs;         // Parameter: Number of pole pairs (Q0)

    //
    // Parameter: Raw angular offset between encoder and phase a (Q0)
    //
    int cal_angle;
    int index_sync_flag;    // Output: Index sync status (Q0)

    //
    // Parameter :  Scaler converting 1/N cycles to a GLOBAL_Q speed (Q0)
    // independently with global Q
    //
    Uint32 SpeedScaler;
    _iq Speed_pr;           // Output :  speed in per-unit

    //
    // Parameter : Scaler converting GLOBAL_Q speed to rpm (Q0) speed
    // independently with global Q
    //
    Uint32 BaseRpm;

    //
    // Output : speed in r.p.m. (Q0) - independently with global Q
    //
    int32 SpeedRpm_pr;

    _iq  oldpos;            // Input: Electrical angle (pu)
    _iq Speed_fr;           // Output :  speed in per-unit

    //
    // Output : Speed in rpm  (Q0) - independently with global Q
    //
    int32 SpeedRpm_fr;
    int32 AccelRPM_fr;
    void (*init)();         // Pointer to the init function
    void (*calc)();         // Pointer to the calc function
} POSSPEED;

//
// Typedef for the POSSPEED_handle
//
typedef POSSPEED *POSSPEED_handle;

//
// Defines for the default initializer for the POSSPEED Object.
//

//Look at speedscaler value first val in second line
//Look at BaseRPM value, 6000 now
#if (CPU_FRQ_150MHZ)
    #define POSSPEED_DEFAULTS {0x0, 0x0,0x0,0x0,0x0,4096,3,0,0x0,\
        94,0,6000,0,\
        0,0,0,0,\
        (void (*)(long))POSSPEED_Init,\
        (void (*)(long))POSSPEED_Calc }
#endif
#if (CPU_FRQ_100MHZ)
    #define POSSPEED_DEFAULTS {0x0, 0x0,0x0,0x0,0x0,65536,3,0,0x0,\
        63,0,6000,0,\
        0,0,0,0,\
        (void (*)(long))POSSPEED_Init,\
        (void (*)(long))POSSPEED_Calc }
#endif /* PHASELEADINGEDGE_H_ */

//
// Function Prototypes
//
void POSSPEED_Init(void);
void POSSPEED_Calc(POSSPEED_handle);


//
// End of File
//







#endif /* PHASELEADINGEDGE_H_ */

