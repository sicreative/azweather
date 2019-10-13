/*   Copyright (c) SC Lee. All rights reserved.
   Licensed under the GNU GPLv3 License.

   Bare Metal program for LED soft PWM

   Normally not in use.

   Some of the code copies from Microsoft sample example https://github.com/Azure/azure-sphere-samples
  sCopyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License.

  


*/

#include <ctype.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <float.h>
#include "mt3620-baremetal.h"
#include "mt3620-intercore.h"
#include "mt3620-inter-message.h"
#include "mt3620-uart-poll.h"
#include "mt3620-gpio.h"
#include "mt3620-timer.h"
#include "mt3620-timer-poll.h"





#define LED_ON 0xFFFFFFFF
#define LED_OFF 0x0


extern uint32_t StackTop; // &StackTop == end of TCM0

static _Noreturn void DefaultExceptionHandler(void);

static void PrintBytes(const uint8_t *buf, int start, int end);
static void PrintGuid(const uint8_t *guid);

static const int LED1REDGPIO = 8;
static const int LED1BLUEGPIO = 10;
static const int LED1GREENGPIO = 9;

static const int LEDAPPGPIO = 4;
static const int LEDWIFIGPIO = 5;


static const int LED1REDMIXMAX = 255;
static const int LED1BLUEMIXMAX = 64;
static const int LED1GREENMIXMAX = 239;



uint32_t led1_flash = LED_OFF;
uint32_t ledwifi_flash = LED_OFF;
uint32_t ledapp_flash = LED_OFF;


//256 step
uint8_t count = 0;
uint8_t countx = 0;
uint8_t countxx = 0;
uint8_t led1red = 255;
uint8_t led1blue = 64;
uint8_t led1green = 48;
uint8_t ledwifi = 180;
uint8_t ledapp = 64;

uint8_t led1mask = 0;
uint8_t ledwifimask = 0;
uint8_t ledappmask = 0;

static _Noreturn void RTCoreMain(void);

//static void HandleBlinkTimerIrq(void);

//static void blink(void);




// ARM DDI0403E.d SB1.5.2-3
// From SB1.5.3, "The Vector table must be naturally aligned to a power of two whose alignment
// value is greater than or equal to (Number of Exceptions supported x 4), with a minimum alignment
// of 128 bytes.". The array is aligned in linker.ld, using the dedicated section ".vector_table".

// The exception vector table contains a stack pointer, 15 exception handlers, and an entry for
// each interrupt.
#define INTERRUPT_COUNT 100 // from datasheet
#define EXCEPTION_COUNT (16 + INTERRUPT_COUNT)
#define INT_TO_EXC(i_) (16 + (i_))
static const uintptr_t ExceptionVectorTable[EXCEPTION_COUNT]
    __attribute__((section(".vector_table"))) __attribute__((used)) = {
        [0] = (uintptr_t)&StackTop,                // Main Stack Pointer (MSP)
        [1] = (uintptr_t)RTCoreMain,               // Reset
        [2] = (uintptr_t)DefaultExceptionHandler,  // NMI
        [3] = (uintptr_t)DefaultExceptionHandler,  // HardFault
        [4] = (uintptr_t)DefaultExceptionHandler,  // MPU Fault
        [5] = (uintptr_t)DefaultExceptionHandler,  // Bus Fault
        [6] = (uintptr_t)DefaultExceptionHandler,  // Usage Fault
        [11] = (uintptr_t)DefaultExceptionHandler, // SVCall
        [12] = (uintptr_t)DefaultExceptionHandler, // Debug monitor
        [14] = (uintptr_t)DefaultExceptionHandler, // PendSV
        [15] = (uintptr_t)DefaultExceptionHandler, // SysTick
		[INT_TO_EXC(0)] = (uintptr_t)DefaultExceptionHandler,
		[INT_TO_EXC(1)] = (uintptr_t)Gpt_HandleIrq1,
		[INT_TO_EXC(2)... INT_TO_EXC(INTERRUPT_COUNT - 1)] = (uintptr_t)DefaultExceptionHandler };

static _Noreturn void DefaultExceptionHandler(void)
{
    for (;;) {
        // empty.
    }
}

static void PrintBytes(const uint8_t *buf, int start, int end)
{
    int step = (end >= start) ? +1 : -1;

    for (/* nop */; start != end; start += step) {
        Uart_WriteHexBytePoll(buf[start]);
    }
    Uart_WriteHexBytePoll(buf[end]);
}

static void PrintGuid(const uint8_t *guid)
{
    PrintBytes(guid, 3, 0); // 4-byte little-endian word
    Uart_WriteStringPoll("-");
    PrintBytes(guid, 5, 4); // 2-byte little-endian half
    Uart_WriteStringPoll("-");
    PrintBytes(guid, 7, 6); // 2-byte little-endian half
    Uart_WriteStringPoll("-");
    PrintBytes(guid, 8, 9); // 2 bytes
    Uart_WriteStringPoll("-");
    PrintBytes(guid, 10, 15); // 6 bytes
}






static inline uint8_t patterntrigger(uint32_t* pattern) {
	return (*pattern >> countxx & 0x1);
}
	
void flashupdate() {

	
	led1mask = patterntrigger(&led1_flash) * 255;

	ledwifimask = patterntrigger(&ledwifi_flash) * 255;

	ledappmask = patterntrigger(&ledapp_flash) * 255;
		
	
}








static _Noreturn void RTCoreMain(void)
{
	


	// SCB->VTOR = ExceptionVectorTable
	WriteReg32(SCB_BASE, 0x08, (uint32_t)ExceptionVectorTable);

	//Uart_Init();
	Uart_WriteStringPoll("--------------------------------\r\n");
	Uart_WriteStringPoll("LED_SoftPWM_RTApp_MT3620_BareMetal\r\n");
	Uart_WriteStringPoll("App built on: " __DATE__ ", " __TIME__ "\r\n");

	static const GpioBlock pwm1 = {
	.baseAddr = 0x38020000,.type = GpioBlock_PWM,.firstPin = 4,.pinCount = 4 };
	Mt3620_Gpio_AddBlock(&pwm1);

		static const GpioBlock pwm2 = {
	.baseAddr = 0x38030000,.type = GpioBlock_PWM,.firstPin = 8,.pinCount = 4 };

	Mt3620_Gpio_AddBlock(&pwm2);



	Mt3620_Gpio_ConfigurePinForOutput(LED1REDGPIO);
	Mt3620_Gpio_ConfigurePinForOutput(LED1BLUEGPIO);
	Mt3620_Gpio_ConfigurePinForOutput(LED1GREENGPIO);
	Mt3620_Gpio_ConfigurePinForOutput(LEDAPPGPIO);
	Mt3620_Gpio_ConfigurePinForOutput(LEDWIFIGPIO);


	init_inter_message();
	



	size_t datasize = INTER_BUFF_SIZE;
	uint8_t type = 0;
	uint8_t data[datasize];
	uint8_t buf[4];
	




	for (;;)
	{
		
		

		
	
		

		int8_t result = dequeue(&type, data, &datasize);
		if (!result && datasize > 0) {



			switch (type) {
			case LED1RED:
				led1red = data[0];
				break;
			case LED1BLUE:
				led1blue = data[0];
				break;
			case LED1GREEN:
				led1green = data[0];
				break;
			case LEDWIFI:
				ledwifi = data[0];
				break;
			case LEDAPP:
				ledapp = data[0];
				break;
			case LED1FLASH:
				memcpy(&led1_flash, data, 4);
				break;
			case LEDAPPFLASH:
				memcpy(&ledapp_flash, data, 4);
				break;
			case LEDWIFIFLASH:
				memcpy(&ledwifi_flash, data, 4);
				break;
			case LED1MIXCOLOR:
				led1red =(uint8_t) ((float)data[0] / (float)255 * (float)LED1REDMIXMAX);
				led1blue = (uint8_t)((float)data[1] / (float)255 * (float)LED1BLUEMIXMAX);
				led1green = (uint8_t)((float)data[2] / (float)255 * (float)LED1GREENMIXMAX);

			}
		}

		

		if (count == 0) {
			if (led1red & led1mask)
				Mt3620_Gpio_Write(LED1REDGPIO, false);
			if (ledwifi & ledwifimask)
				Mt3620_Gpio_Write(LEDWIFIGPIO, false);
			if (ledapp & ledappmask)
				Mt3620_Gpio_Write(LEDAPPGPIO, false);
		}
		if (count == (led1red & led1mask))
			Mt3620_Gpio_Write(LED1REDGPIO, true);
		if (count == (ledwifi & ledwifimask))
			Mt3620_Gpio_Write(LEDWIFIGPIO, true);
		if (count == (ledapp & ledappmask))
			Mt3620_Gpio_Write(LEDAPPGPIO, true);

		//green and blue will offset 1/3 for reduce flicker 
		uint8_t offset = count;
			offset+=72;
		if (offset == 0 && (led1blue & led1mask)) {
				Mt3620_Gpio_Write(LED1BLUEGPIO, false);
		}
		if (offset == (led1blue & led1mask))
			Mt3620_Gpio_Write(LED1BLUEGPIO, true);
		offset += 72;
		if (offset == 0 && (led1green & led1mask)) {
			Mt3620_Gpio_Write(LED1GREENGPIO, false);
		}
		if (offset == (led1green & led1mask ))
			Mt3620_Gpio_Write(LED1GREENGPIO, true);
		++count;
		if (!count) {
			++countx;
			if (!(countx %64)) {
				flashupdate();
				if (++countxx >= 32)
					countxx = 0;
				
					
			}
		}

		

	}


}

	

