//###########################################################################
//
// FILE:   Example_2833xAdcSoc.c
//
// TITLE:  ADC Start of Conversion Example
//
//! \addtogroup f2833x_example_list
//! <h1> ADC Start of Conversion (adc_soc)</h1>
//!
//! This ADC example uses ePWM1 to generate a periodic ADC SOC on SEQ1.
//! Two channels are converted, ADCINA3 and ADCINA2.
//!
//
// Included Files
//
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "ME6_globals.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "phaseLeadingEdge.h"
//
// Function Prototypes
void Gpio_setup(void);
void gpioPinSet(Uint32 loopCount);
void InitEncoderEPWM(void);
void InitEQepGpio(void);

__interrupt void adc_isr(void);
__interrupt void prdTick(void);

//
// Globals
//

#define PHASE_ARRAY_LENGTH 20

Uint32 LoopCount;
Uint16 ConversionCount;
Uint16 phaseVoltage[PHASE_ARRAY_LENGTH];
Uint16 phaseVoltage1[PHASE_ARRAY_LENGTH];
Uint32 currentTime;
Uint32 timeChange;

// Clock Initialization  Settingss
#define CPU_CLK   150e6
#define PWM_CLK   5e3
#define SP        CPU_CLK/(2*PWM_CLK)
#define TBCTLVAL  0x200E      // up-down count, timebase=SYSCLKOUT
#define INVERTER_PERIOD 1000
#define INVERTER_START_DUTY 50

#define LSPCLK_FREQ CPU_CLK/4
#define SCI_FREQ 115200
#define SCI_PRD (LSPCLK_FREQ/(SCI_FREQ*8))-1

#define ADC_SHCLK  0x1
#define ADC_CKPS   0x0

// Functioon
__interrupt void control_loop_isr(void); // This  should triggle when the gpio changes

/// Global Variables

float ref = 0;
float T = 1 / 100; // WE're running at 100 hz, therefore
float Ki, Kp, meas, meas_old, out_old, out, phaseAngle, resonanceOffset,
        gpioPinSetValue, phaseReference;
float control_structure, freqLimit;
int tmpPhaseAngle;
int systemInitialized;
POSSPEED qep_posspeed = POSSPEED_DEFAULTS;

//
// Main
//
void main(void)
{

    // Global Declarations
    meas = 0;
    meas_old = 0;
    out_old = 0;
    out = 0;
    // Controller initialization
    Kp = -0.1;
    Ki = -1.5;
    T = 0.01; // 100 hertz
    resonanceOffset = 0; // This has to be updated when running program
    phaseReference = 0;
    control_structure = 1;  // 0 means P controller, 1 mean PI
    out = 0;
    freqLimit = 2000;
    systemInitialized = 0;
    phaseAngle = 0;


    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the DSP2833x_SysCtrl.c file.
    //
    InitSysCtrl();

    EALLOW;
#if (CPU_FRQ_150MHZ)     // Default - 150 MHz SYSCLKOUT
    //
    // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3)   = 25.0 MHz
    //
#define ADC_MODCLK 0x3
#endif

    EDIS;

    //
    // Define ADCCLK clock frequency ( less than or equal to 25 MHz )
    // Assuming InitSysCtrl() has set SYSCLKOUT to 150 MHz
    //
    EALLOW;
    SysCtrlRegs.HISPCP.all = ADC_MODCLK;
    EDIS;

    //
    // Step 2. Initialize GPIO:
    // This example function is found in the DSP2833x_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    //
    // InitGpio();  // Skipped for this example
    EALLOW;
    Gpio_setup();
    InitEQepGpio(); // This is the gpio for the pjhase detection

    EDIS;

    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts

    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the DSP2833x_PieCtrl.c file.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();
    // Initialize all the gpio pins that will be used for the data transfer

    EALLOW;
    // This is needed to write to EALLOW protected register
    PieVectTable.XINT13 = &control_loop_isr;
    PieVectTable.ADCINT = &adc_isr;
    PieVectTable.EPWM1_INT = &prdTick;
    EDIS;
    // This is needed to disable write to EALLOW protected registers

    //
    // Step 4. Initialize all the Device Peripherals:
    // This function is found in DSP2833x_InitPeripherals.c
    //

    // InitPeripherals(); // Not requi½red for this example
    InitAdc();  // For this example, init the ADC
    InitEncoderEPWM();

    // TIMERS
    // Timers
    InitCpuTimers();   // For this example, only initialize the Cpu Timers
    ConfigCpuTimer(&CpuTimer1, 150, 10000); // 10 milliseconds
    CpuTimer1Regs.TCR.all = 0x4000; //write-only instruction to set TSS bit = 0

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Here we synchronize pwm clocks

    //EALLOW;
    // SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    //   EDIS;
    //
    // Step 5. User specific code, enable interrupts:
    //

    //

    IER |= M_INT1;      // Enable CPU Interrupt 1
    IER |= M_INT3;

    IER |= M_INT13;

    EINT;
    // Enable Global interrupt INTM
    ERTM;
    // Enable Global realtime interrupt DBGM

    // Enable ADCINT in PIE
    //
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1; /// EQEP
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1; // inverter PWM
    PieCtrlRegs.PIEIER3.bit.INTx3 = 1; // PIE for  inverter control (MAIN CONTROL LOOP)

    EINT;
    // Enable Global interrupt INTM
    ERTM;
    // Enable Global realtime interrupt DBGM

    LoopCount = 0;
    ConversionCount = 0;

    // Configure GPIO

    //
    // Configure ADC
    //
    AdcRegs.ADCMAXCONV.all = 0x0001;       // Setup 2 conv's on SEQ1
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x3; // Setup ADCINA3 as 1st SEQ1 conv.
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x2; // Setup ADCINA2 as 2nd SEQ1 conv.

    //
    // Enable SOCA from ePWM to start SEQ1
    //
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;  // Enable SEQ1 interrupt (every EOS)

    //
    // Assumes ePWM1 clock is already enabled in InitSysCtrl();
    //
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;     // Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 4;    // Select SOC from from CPMA on upcount
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;     // Generate pulse on 1st event
    EPwm1Regs.CMPA.half.CMPA = 0x0080;  // Set compare A value
    EPwm1Regs.TBPRD = 0xFFFF;           // Set period for ePWM1
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;    // count up and start

    // Initalize eqep registers
    qep_posspeed.init(&qep_posspeed);

    Uint32 loopMax = 65000;
    int a = 5;
    while (a < 10)
    {
        //gpioPinSet(LoopCount);

        LoopCount++;
        if (LoopCount > loopMax)
        {
            LoopCount = 0;
        }
    }
}

//
// adc_isr -
//
__interrupt void adc_isr(void)
{
    phaseVoltage[ConversionCount] = AdcRegs.ADCRESULT0 >> 4;
    phaseVoltage1[ConversionCount] = AdcRegs.ADCRESULT1 >> 4;

    //
    // If 40 conversions have been logged, start over
    //
    if (ConversionCount == PHASE_ARRAY_LENGTH - 1)
    {
        ConversionCount = 0;
    }
    else
    {
        ConversionCount++;
    }

    //
    // Reinitialize for next ADC sequence
    //
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;         // Reset SEQ1
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;       // Clear INT SEQ1 bit
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

    return;
}

__interrupt void control_loop_isr(void)
{
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    // The idea here is that we want to control the system to zero phase shift
    // Initial freuqency is 28 kHz

    // Start by definition of phase
    // The direction of the phase will be used to help define the reference value.

    // inclusion of

    // Pulle measure from the adc
    // Then we have to give a value for what this difference in value is.
    int iterator = 0;
    tmpPhaseAngle = 0;
    for (iterator = 0; iterator < 10; iterator++)
    {
        tmpPhaseAngle = tmpPhaseAngle + phaseVoltage[iterator];
    }

    tmpPhaseAngle = tmpPhaseAngle / 10;
    // Now we have a normalized phaseAngle
    //Convert phaseAngle from 0-4096 to 0 to 180

    phaseAngle = ((float) tmpPhaseAngle - 100) * 360 / 4096;
    if (qep_posspeed.DirectionQep)
    {
        phaseAngle = -phaseAngle;
    }
    meas = phaseReference + phaseAngle;

    // The measure value

    /////// change this to adc value

    if (control_structure == 0)
    {
        // P CONTROLLER
        out = -meas * Kp;
        if (out < -freqLimit)
        {
            out = 0;
            out_old = 0;
        }
        else if (out > freqLimit)
        {
            out = 0;
            out_old = 0;
        }
        else
        {

        }
    }
    else if (control_structure == 1)
    {
        // i.e PI CONTROLLER

        // The measured value is the angle
        out = out_old + meas * (Kp + Ki * T) - meas_old * Kp;
        // P CONTROLLER

        // Make sure system is in bounds
        if (out < -freqLimit)
        {
            out = 0;
        }
        else if (out > freqLimit)
        {
            out = 0;
        }
        else
        {

        }
    }
    phaseAngle = ((float) tmpPhaseAngle - 100) / 4096 * 360;

    if (control_structure < 2)
    {
        gpioPinSetValue = out;
        gpioPinSet(gpioPinSetValue + freqLimit);
    }
    else
    {
        gpioPinSetValue = tmpPhaseAngle;
        if (qep_posspeed.DirectionQep)
        {
            gpioPinSetValue = gpioPinSetValue + 10000;
        }
        gpioPinSet( freqLimit+ gpioPinSetValue);

    }

    out_old = out;
    meas_old = meas;

    Uint32 timeChange = currentTime - CpuTimer2Regs.TIM.all;
    CpuTimer2Regs.TCR.bit.TRB = 1;        // Reset CPU timer to period value
    CpuTimer2Regs.TCR.bit.TSS = 0;        // Start or reset CPU timer 2

    Uint32 currentTime = CpuTimer2Regs.TIM.all;

    // The control frequency is currently at 10 ms. dvs 100 hz

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;

    EALLOW;
    EDIS;
}

void gpioPinSet(Uint32 freqInt)
{
    //GpioDataRegs.GPACLEAR.all  =loopCount;
    //GpioDataRegs.GPASET.all    =0xFFFFFFFF;
    // GpioDataRegs.GPASET.all    =0x00000000;//FFFFFFFF;

    GpioDataRegs.GPACLEAR.all = 0xFFFFFFFF;
    GpioDataRegs.GPASET.all = freqInt;
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0; //Turn on pull-up

    GpioDataRegs.GPASET.bit.GPIO1 = 1;
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
    //GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;

    //GpioCtrlRegs.GPACTRL.all = loopCount;
    //GpioDataRegs.GPASET.all = loopCount;
    return;
}

void Gpio_setup(void)
{

    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;   // Enable pullup on GPIO8
    // GpioDataRegs.GPASET.bit.GPIO0 = 1;   // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;  // GPIO8 = GPIO8
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;   // GPIO8 = output

    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;   // Enable pullup on GPIO9
    //GpioDataRegs.GPASET.bit.GPIO1 = 1;   // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;  // GPIO9 = GPIO9
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;   // GPIO9 = output

    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;  // Enable pullup on GPIO10
    //GpioDataRegs.GPASET.bit.GPIO2 = 1;  // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0; // GPIO10 = GPIO10
    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;   // GPIO10 = output

    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;  // Enable pullup on GPIO11
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0; // GPIO11 = GPIO11
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;  // GPIO11 = output
    //

    //
    // Enable GPIO outputs on GPIO8 - GPIO11, set it high
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;   // Enable pullup on GPIO8
    //GpioDataRegs.GPASET.bit.GPIO4 = 1;   // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;  // GPIO8 = GPIO8
    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;   // GPIO8 = output

    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;   // Enable pullup on GPIO9
    //GpioDataRegs.GPASET.bit.GPIO5 = 1;   // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;  // GPIO9 = GPIO9
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;   // GPIO9 = output

    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;  // Enable pullup on GPIO10
    //GpioDataRegs.GPASET.bit.GPIO6 = 1;  // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0; // GPIO10 = GPIO10
    GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;   // GPIO10 = output

    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;  // Enable pullup on GPIO11
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0; // GPIO11 = GPIO11
    GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;  // GPIO11 = output

    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;   // Enable pullup on GPIO8
    //GpioDataRegs.GPASET.bit.GPIO8 = 1;   // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;  // GPIO8 = GPIO8
    GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;   // GPIO8 = output

    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;   // Enable pullup on GPIO9
    // GpioDataRegs.GPASET.bit.GPIO9 = 1;   // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;  // GPIO9 = GPIO9
    GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;   // GPIO9 = output

    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;  // Enable pullup on GPIO10
    //GpioDataRegs.GPASET.bit.GPIO10 = 1;  // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0; // GPIO10 = GPIO10
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;   // GPIO10 = output

    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;  // Enable pullup on GPIO11
    //GpioDataRegs.GPASET.bit.GPIO11 = 1;  // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0; // GPIO11 = GPIO11
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;  // GPIO11 = output

    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;  // Enable pullup on GPIO11
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0; // GPIO11 = GPIO11
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;  // GPIO11 = output
    //
    // Enable GPIO outputs on GPIO8 - GPIO11, set it high
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;   // Enable pullup on GPIO8
    //GpioDataRegs.GPASET.bit.GPIO13 = 1;   // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;  // GPIO8 = GPIO8
    GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;   // GPIO8 = output

    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;   // Enable pullup on GPIO9
    //GpioDataRegs.GPASET.bit.GPIO14 = 1;   // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;  // GPIO9 = GPIO9
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;   // GPIO9 = output

    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;  // Enable pullup on GPIO10
    //GpioDataRegs.GPASET.bit.GPIO15 = 1;  // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0; // GPIO10 = GPIO10
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;   // GPIO10 = output

    GpioCtrlRegs.GPAMUX1.all = 0x00000000;  // All GPIO
    GpioCtrlRegs.GPAMUX1.all = 0x00000000;  // All GPIO
    GpioCtrlRegs.GPADIR.all = 0xFFFFFFFF;   // All outputs

    EDIS;

    return;
}

void setOutputPins(int number)
{
    // Here we have to convert an integer to a 16 bit binary
    // Number range is 0 to 65536

}

__interrupt void prdTick(void)
{
    //
    // Position and Speed measurement
    //

    qep_posspeed.calc(&qep_posspeed);

    //qep_posspeed.SpeedRpm_fr;
    //
    // Acknowledge this interrupt to receive more interrupts from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    EPwm1Regs.ETCLR.bit.INT = 1;
}

void InitEncoderEPWM()
{
    EPwm1Regs.TBSTS.all = 0;
    EPwm1Regs.TBPHS.half.TBPHS = 0;
    EPwm1Regs.TBCTR = 0;

    EPwm1Regs.CMPCTL.all = 0x50;     // immediate mode for CMPA and CMPB
    EPwm1Regs.CMPA.half.CMPA = SP / 2;
    EPwm1Regs.CMPB = 0;

    //
    // CTR=CMPA when inc->EPWM1A=1, when dec->EPWM1A=0
    //
    EPwm1Regs.AQCTLA.all = 0x60;

    EPwm1Regs.AQCTLB.all = 0x09;     // CTR=PRD ->EPWM1B=1, CTR=0 ->EPWM1B=0
    EPwm1Regs.AQSFRC.all = 0;
    EPwm1Regs.AQCSFRC.all = 0;

    EPwm1Regs.TZSEL.all = 0;
    EPwm1Regs.TZCTL.all = 0;
    EPwm1Regs.TZEINT.all = 0;
    EPwm1Regs.TZFLG.all = 0;
    EPwm1Regs.TZCLR.all = 0;
    EPwm1Regs.TZFRC.all = 0;

    EPwm1Regs.ETSEL.all = 0x0A;      // Interrupt on PRD
    EPwm1Regs.ETPS.all = 1;
    EPwm1Regs.ETFLG.all = 0;
    EPwm1Regs.ETCLR.all = 0;
    EPwm1Regs.ETFRC.all = 0;

    EPwm1Regs.PCCTL.all = 0;

    EPwm1Regs.TBCTL.all = 0x0010 + TBCTLVAL; // Enable Timer
    EPwm1Regs.TBPRD = SP;

}

void InitEQepGpio(void)
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;    // Enable pull-up on GPIO24 (EQEP2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;    // Enable pull-up on GPIO25 (EQEP2B)
    GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;    // Enable pull-up on GPIO26 (EQEP2I)
    GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;    // Enable pull-up on GPIO27 (EQEP2S)

    //
    // Inputs are synchronized to SYSCLKOUT by default.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 0;  // Sync to SYSCLKOUT GPIO24 (EQEP2A)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 0;  // Sync to SYSCLKOUT GPIO25 (EQEP2B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 0;  // Sync to SYSCLKOUT GPIO26 (EQEP2I)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 0;  // Sync to SYSCLKOUT GPIO27 (EQEP2S)

    //
    // Configure eQEP-2 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be eQEP2 functional
    // pins. Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 1;   // Configure GPIO24 as EQEP2A
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 1;   // Configure GPIO25 as EQEP2B
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 1;   // Configure GPIO26 as EQEP2I
    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 1;   // Configure GPIO27 as EQEP2S

    EDIS;
}

//
// End of File
//

