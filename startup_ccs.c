/******************************************************************************
*  Filename:       startup_ccs.c
*  Revised:        $Date: 2014-03-13 14:58:46 +0100 (to, 13 mar 2014) $
*  Revision:       $Revision: 12379 $
*
*  Description:    Startup code for CC26xx PG2 device family for use with CCS.
*
*  Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/


//*****************************************************************************
//
// Check if compiler is CCS
//
//*****************************************************************************
#if !(defined(__TI_COMPILER_VERSION__))
#error "startup_ccs.c: Unsupported compiler!"
#endif

//#include <inc/hw_types.h>
//#include <inc/hw_memmap.h>
//#include <inc/hw_cpu_scs.h>

//*****************************************************************************
//
//! Forward declaration of the reset ISR and the default fault handlers.
//
//*****************************************************************************
void        ResetISR( void );
static void NmiSR( void );
static void FaultISR( void );
static void IntDefaultHandler( void );
extern int  main(void);

extern void AONRTCIntHandler(void);
extern void Timer0AIntHandler(void);
extern void Timer0BIntHandler(void);
extern void Timer1AIntHandler(void);
extern void Timer1BIntHandler(void);
extern void Timer2AIntHandler(void);
extern void Timer2BIntHandler(void);
extern void Timer3AIntHandler(void);
extern void Timer3BIntHandler(void);
extern void UART0IntHandler(void);
extern void GPIOIntHandler(void);
//extern void SysTickIntHandler(void);
extern void RFCCPE0IntHandler(void);
extern void RFCHardwareIntHandler(void);
extern void RFCCPE1IntHandler(void);

static void WatchdogHandler(void);

//*****************************************************************************
//
//! The entry point for the application startup code and device trim fxn.
//
//*****************************************************************************
extern void _c_int00(void);
extern void trimDevice(void);


//*****************************************************************************
//
// CCS: Linker variable that marks the top of the stack.
//
//*****************************************************************************
extern unsigned long __STACK_END;


//! The vector table. Note that the proper constructs must be placed on this to
//! ensure that it ends up at physical address 0x0000.0000 or at the start of
//! the program if located at a start address other than 0.
//
//*****************************************************************************
#pragma DATA_SECTION(g_pfnVectors, ".intvecs")
void (* const g_pfnVectors[])(void) =
{
    (void (*)(void))((unsigned long)&__STACK_END),
                                            // The initial stack pointer
    ResetISR,                               // The reset handler
    NmiSR,                                  // The NMI handler
    FaultISR,                               // The hard fault handler
    FaultISR,                               // The MPU fault handler  modified from IntDefaultHandler, 2018/04/15
    FaultISR,                               // The bus fault handler  modified from IntDefaultHandler, 2018/04/15
    FaultISR,                               // The usage fault handler  modified from IntDefaultHandler, 2018/04/15
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    IntDefaultHandler,                      // SVCall handler
    IntDefaultHandler,                      // Debug monitor handler
    0,                                      // Reserved
    IntDefaultHandler,                      // The PendSV handler
	IntDefaultHandler,                      // The SysTick handler
	GPIOIntHandler,                         // AON edge detect
    IntDefaultHandler,                      // I2C
	RFCCPE1IntHandler,                      // RF Core Command & Packet Engine 1
    IntDefaultHandler,                      // AON SpiSplave Rx, Tx and CS
	AONRTCIntHandler,                       // AON RTC
	UART0IntHandler,                        // UART0 Rx and Tx
    IntDefaultHandler,                      // AUX software event 0
    IntDefaultHandler,                      // SSI0 Rx and Tx
    IntDefaultHandler,                      // SSI1 Rx and Tx
	RFCCPE0IntHandler,                      // RF Core Command & Packet Engine 0
	RFCHardwareIntHandler,                  // RF Core Hardware
    IntDefaultHandler,                      // RF Core Command Acknowledge
    IntDefaultHandler,                      // I2S
    IntDefaultHandler,                      // AUX software event 1
    WatchdogHandler,                        // Watchdog timer
	Timer0AIntHandler,                      // Timer 0 subtimer A
	Timer0BIntHandler,                      // Timer 0 subtimer B
	Timer1AIntHandler,                      // Timer 1 subtimer A
	Timer1BIntHandler,                      // Timer 1 subtimer B
	Timer2AIntHandler,                      // Timer 2 subtimer A
	Timer2BIntHandler,                      // Timer 2 subtimer B
	Timer3AIntHandler,                      // Timer 3 subtimer A
	Timer3BIntHandler,                      // Timer 3 subtimer B
    IntDefaultHandler,                      // Crypto Core Result available
    IntDefaultHandler,                      // uDMA Software
    IntDefaultHandler,                      // uDMA Error
    IntDefaultHandler,                      // Flash controller
    IntDefaultHandler,                      // Software Event 0
    IntDefaultHandler,                      // AUX combined event
    IntDefaultHandler,                      // AON programmable 0
    IntDefaultHandler,                      // Dynamic Programmable interrupt
                                            // source (Default: PRCM)
    IntDefaultHandler,                      // AUX Comparator A
    IntDefaultHandler,                      // AUX ADC new sample or ADC DMA
                                            // done, ADC underflow, ADC overflow
    IntDefaultHandler                       // TRNG event
};


//*****************************************************************************
//
//! This is the code that gets called when the processor first starts execution
//! following a reset event. Only the absolutely necessary set is performed,
//! after which the application supplied entry() routine is called. Any fancy
//! actions (such as making decisions based on the reset cause register, and
//! resetting the bits in that register) are left solely in the hands of the
//! application.
//
//*****************************************************************************
void
ResetISR(void)
{
    //
    // Final trim of device
    //
    trimDevice();

    //
    // Jump to the CCS C Initialization Routine.
    //
    __asm("    .global _c_int00\n"
            "    b.w     _c_int00");

    //
    // If we ever return signal Error
    //
    FaultISR();
}

//*****************************************************************************
//
//! This is the code that gets called when the processor receives a NMI. This
//! simply enters an infinite loop, preserving the system state for examination
//! by a debugger.
//
//*****************************************************************************
static void
NmiSR(void)
{
}

//*****************************************************************************
//
//! This is the code that gets called when the processor receives a fault
//! interrupt. This simply enters an infinite loop, preserving the system state
//! for examination by a debugger.
//
//*****************************************************************************
__attribute__( (naked) )
static void
FaultISR(void)
{
}


//*****************************************************************************
//
//! This is the code that gets called when the processor receives an unexpected
//! interrupt. This simply enters an infinite loop, preserving the system state
//! for examination by a debugger.
//
//*****************************************************************************
static void
IntDefaultHandler(void)
{
}

static void
WatchdogHandler(void)
{
}

void AONRTCIntHandler(void) {}
void Timer0AIntHandler(void) {}
void Timer0BIntHandler(void) {}
void Timer1AIntHandler(void) {}
void Timer1BIntHandler(void) {}
void Timer2AIntHandler(void) {}
void Timer2BIntHandler(void) {}
void Timer3AIntHandler(void) {}
void Timer3BIntHandler(void) {}
void UART0IntHandler(void) {}
void GPIOIntHandler(void) {}
void RFCCPE0IntHandler(void) {}
void RFCHardwareIntHandler(void) {}
void RFCCPE1IntHandler(void) {}

void trimDevice(void) {}
