/*
 * phaseLeadingEdge.c
 *
 *  Created on: May 16, 2022
 *      Author: barlo
 */


/*
 * EncoderFunctions.c
 *
 *  Created on: Nov 16, 2021
 *      Author: henri
 */



//
// Included Files
//
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "phaseLeadingEdge.h"   // Example specific Include file
#include "IQmathLib.h"

//
// POSSPEED_Init -
//
void
POSSPEED_Init(void)
{
    #if (CPU_FRQ_150MHZ)
        EQep1Regs.QUPRD=1500000;  // Unit Timer for 100Hz at 150 MHz SYSCLKOUT
    #endif
    #if (CPU_FRQ_100MHZ)
        EQep1Regs.QUPRD=1000000;  // Unit Timer for 100Hz at 100 MHz SYSCLKOUT
    #endif

    EQep1Regs.QDECCTL.bit.QSRC=00;      // QEP quadrature count mode

    EQep1Regs.QEPCTL.bit.FREE_SOFT=2;

    //
    // PCRM=00 mode - QPOSCNT reset on index event
    //
    EQep1Regs.QEPCTL.bit.PCRM=1;

    EQep1Regs.QEPCTL.bit.UTE=1;         // Unit Timeout Enable
    EQep1Regs.QEPCTL.bit.QCLM=1;        // Latch on unit time out
    EQep1Regs.QPOSMAX=0xffffffff;
    EQep1Regs.QEPCTL.bit.QPEN=1;        // QEP enable

    EQep1Regs.QCAPCTL.bit.UPPS=5;       // 1/32 for unit position
    EQep1Regs.QCAPCTL.bit.CCPS=7;       // 1/128 for CAP clock
    EQep1Regs.QCAPCTL.bit.CEN=1;        // QEP Capture Enable

}

//
// POSSPEED_Calc -
//
void
POSSPEED_Calc(POSSPEED *p)
{
    long tmp;
    unsigned int pos16bval,temp1;
    _iq Tmp1,newp,oldp;

    //
    // Position calculation - mechanical and electrical motor angle
    //

    //
    // Motor direction: 0=CCW/reverse, 1=CW/forward
    //
    p->DirectionQep = EQep1Regs.QEPSTS.bit.QDF;

    //
    // capture position once per QA/QB period
    //
    pos16bval=(unsigned int)EQep1Regs.QPOSCNT;

    //
    // raw theta = current pos. + ang. offset from QA
    //
    p->theta_raw = pos16bval+ p->cal_angle;

    //
    // The following lines calculate p->theta_mech ~=
    // QPOSCNT/mech_scaler [current cnt/(total cnt in 1 rev.)]
    // where mech_scaler = 4000 cnts/revolution
    //
    tmp = (long)((long)p->theta_raw*(long)p->mech_scaler);  // Q0*Q26 = Q26
    tmp &= 0x03FFF000;
    p->theta_mech = (int)(tmp>>11);                 // Q26 -> Q15
    p->theta_mech &= 0x7FFF;

    //
    // The following lines calculate p->elec_mech
    //
    p->theta_elec = p->pole_pairs*p->theta_mech;    // Q0*Q15 = Q15
    p->theta_elec &= 0x7FFF;

    //
    // Check an index occurrence
    //
    if (EQep1Regs.QFLG.bit.IEL == 1)
    {
        p->index_sync_flag = 0x00F0;
        EQep1Regs.QCLR.bit.IEL=1;                   // Clear interrupt flag
    }

    //
    // High Speed Calculation using QEP Position counter
    // Check unit Time out-event for speed calculation:
    // Unit Timer is configured for 100Hz in INIT function
    //
    if(EQep1Regs.QFLG.bit.UTO==1)      // If unit timeout (one 100Hz period)
    {
        //
        // Differentiator
        // The following lines calculate position =
        // (x2-x1)/4000 (position in 1 revolution)
        //
        pos16bval=(unsigned int)EQep1Regs.QPOSLAT;      // Latched POSCNT value
        tmp = (long)((long)pos16bval*(long)p->mech_scaler); // Q0*Q26 = Q26
        tmp &= 0x03FFF000;
        tmp = (int)(tmp>>11);           // Q26 -> Q15
        tmp &= 0x7FFF;
        newp=_IQ15toIQ(tmp);
        oldp=p->oldpos;

        if (p->DirectionQep==0)         // POSCNT is counting down
        {
            if (newp>oldp)
            {
                Tmp1 = - (_IQ(1) - newp + oldp);    // x2-x1 should be negative
            }
            else
            {
                Tmp1 = newp -oldp;
            }
        }
        else if (p->DirectionQep==1)    // POSCNT is counting up
        {
            if (newp<oldp)
            {
                Tmp1 = _IQ(1) + newp - oldp;
            }
            else
            {
                Tmp1 = newp - oldp;     // x2-x1 should be positive
            }
        }

        if (Tmp1>_IQ(1))
        {
            p->Speed_fr = _IQ(1);
        }
        else if (Tmp1<_IQ(-1))
        {
            p->Speed_fr = _IQ(-1);
        }
        else
        {
            p->Speed_fr = Tmp1;
        }

        //
        // Update the electrical angle
        //
        p->oldpos = newp;

        //
        // Change motor speed from pu value to rpm value (Q15 -> Q0)
        // Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q
        //
        p->SpeedRpm_fr = _IQmpy(p->BaseRpm,p->Speed_fr);

        EQep1Regs.QCLR.bit.UTO=1;                   // Clear interrupt flag
    }

    //
    // Low-speed computation using QEP capture counter
    //
    if(EQep1Regs.QEPSTS.bit.UPEVNT==1)     // Unit position event
    {
        if(EQep1Regs.QEPSTS.bit.COEF==0)   // No Capture overflow
        {
            temp1=(unsigned long)EQep1Regs.QCPRDLAT;   // temp1 = t2-t1
        }
        else   // Capture overflow, saturate the result
        {
            temp1=0xFFFF;
        }

        //
        // p->Speed_pr = p->SpeedScaler/temp1
        //
        p->Speed_pr = _IQdiv(p->SpeedScaler,temp1);
        Tmp1=p->Speed_pr;

        if (Tmp1>_IQ(1))
        {
            p->Speed_pr = _IQ(1);
        }
        else
        {
            p->Speed_pr = Tmp1;
        }

        //
        // Convert p->Speed_pr to RPM
        //
        if (p->DirectionQep==0)     // Reverse direction = negative
        {
            //
            // Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q
            //
            p->SpeedRpm_pr = -_IQmpy(p->BaseRpm,p->Speed_pr);
        }
        else                        // Forward direction = positive
        {
            //
            // Q0 = Q0*GLOBAL_Q => _IQXmpy(), X = GLOBAL_Q
            //
            p->SpeedRpm_pr = _IQmpy(p->BaseRpm,p->Speed_pr);
        }

        EQep1Regs.QEPSTS.all=0x88;  // Clear Unit position event flag
                                    // Clear overflow error flag
    }
}

//
// End of File
//

