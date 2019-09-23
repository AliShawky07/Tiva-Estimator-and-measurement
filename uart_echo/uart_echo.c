//*****************************************************************************
//
// uart_echo.c - Example for reading data from and writing data to the UART in
//               an interrupt driven fashion.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "inc/hw_gpio.h"
#include "inc/hw_timer.h"
#include "driverlib/timer.h"
#include "inc/hw_uart.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"


#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif


volatile uint32_t  g_ui32Flags;
volatile uint32_t  seconds;
volatile int32_t fitch = 0;
volatile uint32_t  CurrentTime;




void
Timer0IntHandler(void)
{
    char cOne, cTwo;

    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);


    seconds++;
    if(seconds>=60)
    {
        CurrentTime++;
        seconds=0;
    }

    ROM_IntMasterDisable();
    cOne = HWREGBITW(&g_ui32Flags, 0) ? '1' : '0';
    cTwo = HWREGBITW(&g_ui32Flags, 1) ? '1' : '0';
    ROM_IntMasterEnable();
}



void toggle(void)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

    SysCtlDelay(SysCtlClockGet() / 10 / 3);

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

    SysCtlDelay(SysCtlClockGet() / 10 / 3);
}

void
UART1_Configure(void)
{

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    ROM_GPIOPinConfigure(GPIO_PB0_U1RX);
    ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);

    UARTStdioConfig(1, 115200, 16000000);
}

void Switch_Enable(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

void
UART0_Configure(void)
{

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    UARTStdioConfig(0, 115200, 16000000);
}


void GetAndSendInitialTime(void)
{
    CurrentTime=  ( UARTCharGet(UART0_BASE) - (48) ) ;



    CurrentTime = ( (CurrentTime) * (10) ) + ( UARTCharGet(UART0_BASE) -(48) ) ;


    UARTCharPut(UART1_BASE,(CurrentTime));


}

void Timers_Init(void)
{


    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    ROM_IntMasterEnable();

    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet());

    ROM_IntEnable(INT_TIMER0A);

    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    ROM_TimerEnable(TIMER0_BASE, TIMER_A);


}



int
main(void)
{




    int32_t current_speed = 20;
    /*INITIALIZE TIMERS*/
    Timers_Init();
    /*INITIALIZE INPUT SITCHES*/
    Switch_Enable();
    /*INITIALIZE UART0& UART1*/
    UART1_Configure();
    UART0_Configure();

    GetAndSendInitialTime();

    ROM_FPULazyStackingEnable();

    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);



    while(1)
    {

        fitch = UARTCharGet(UART1_BASE);

        if( fitch )
        {
            UARTCharPut(UART1_BASE,CurrentTime);
            UARTCharPut(UART1_BASE,current_speed);
            toggle();

        }

        if ( GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4) == 0 )
        {
            while( GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4) == 0 );

            current_speed++;

            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
            SysCtlDelay(SysCtlClockGet() / 10 / 3);

            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
            SysCtlDelay(SysCtlClockGet() / 10 / 3);

        }
        else if ( GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0) == 0 )
        {
            while( GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0) == 0 );

            current_speed--;

            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

            SysCtlDelay(SysCtlClockGet() / 10 / 3);

            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

            SysCtlDelay(SysCtlClockGet() / 10 / 3);
        }

    }
}
