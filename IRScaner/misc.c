#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_gpio.h>

#include "microrl/config.h"
#include <string.h>
#include <stdlib.h>
#include <misc.h>

extern _TimeInterval Intervals[];
extern uint32_t Index;
extern uint32_t StopIndex;
extern uint32_t DetectFreq;

/*
AVR platform specific implementation routines (for Atmega8, rewrite for your MC)
*/
#define _STM_32_VRESION_ "1.0"

// definition commands word
#define _CMD_HELP   "help"
#define _CMD_CLEAR  "clear"
#define _CMD_SHOW_CNT    "show_cnt"
#define _CMD_CLR_CNT    "clr_cnt"
#define _CMD_START    "start"
#define _CMD_STOP    "stop"
#define _CMD_PRINT    "print"
#define _CMD_SET    "set"


#define _NUM_OF_CMD 8

//available  commands
char * keyworld [] = {_CMD_HELP, _CMD_CLEAR, _CMD_SHOW_CNT, _CMD_CLR_CNT, _CMD_START, _CMD_STOP, _CMD_PRINT, _CMD_SET};

// array for comletion
char * compl_world [_NUM_OF_CMD + 1];



//*****************************************************************************
void print (char * str)
{

	int i = 0;
	while (str [i] != 0) {
		PrintChar(str[i++]);
	}
}

//*****************************************************************************
char get_char (void)
{
	while( USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET) ;

	return USART_ReceiveData(USART1);
}

//*****************************************************************************
void print_help (void)
{
	print ("Use TAB key for completion\n\rCommand:\n\r");
	print ("\tclear               - clear screen\n\r");
	print ("\tshow_cnt   - Show CNT value\n\r");
	print ("\tclr_cnt    - Clear CNT value\n\r");
}


//*****************************************************************************
// execute callback for microrl library
// do what you want here, but don't write to argv!!! read only!!
int execute (int argc, const char * const * argv)
{
	int i = 0;
	int ind = 0;
	uint32_t freq = 0;
	// just iterate through argv word and compare it with your commands
	while (i < argc) {
		if (strcmp (argv[i], _CMD_HELP) == 0) {
			print ("microrl v");
			print (MICRORL_LIB_VER);
			print (" library STM32 v");
			print (_STM_32_VRESION_);
			print("\n\r");
			print_help ();        // print help
		}

		else if (strcmp (argv[i], _CMD_CLEAR) == 0) {
			print ("\033[2J");    // ESC seq for clear entire screen
			print ("\033[H");     // ESC seq for move cursor at left-top corner
		} else if (strcmp (argv[i], _CMD_SHOW_CNT) == 0)
		{
			return 0;
		}
		else if (strcmp (argv[i], _CMD_CLR_CNT) == 0)
		{
			for (ind = 0; ind < MaxIntervals; ind++)
			{
				Intervals[ind].Time = 0;
				Intervals[ind].Value = 0;
			}
			return 0;
		}
		else if (strcmp (argv[i], _CMD_START) == 0)
		{
			Index = 0;
			StopIndex = 0;
			DetectFreq = 0;
			Intervals[Index++].Value  = (GPIOC->IDR & GPIO_Pin_0);
			TIM_Cmd(TIM2, DISABLE);
			/* TIM enable counter */
			TIM_Cmd(TIM3, ENABLE);

			NVIC_EnableIRQ(EXTI0_IRQn);
			print("Start ...\n\r");
			return 0;
		}
		else if (strcmp (argv[i], _CMD_STOP) == 0)
		{
			NVIC_DisableIRQ(EXTI0_IRQn);
			TIM_Cmd(TIM2, DISABLE);
			TIM_Cmd(TIM3, DISABLE);

			if (Index >= MaxIntervals)
			{
				print("Error: Index > MaxIntervals\n\r");
				return 1;
			}
			for (ind = 0; ind < Index; ind++)
				printf("[%03d] Value: %d, Time: %d\n\r", ind, Intervals[ind].Value, Intervals[ind].Time);

			printf("Freq: %d\n\r", DetectFreq);
			return 0;
		}
		else if (strcmp (argv[i], _CMD_PRINT) == 0)
		{
			if (Index >= MaxIntervals)
			{
				print("Error: Index > MaxIntervals\n\r");
				return 1;
			}
			for (ind = 0; ind < Index; ind++)
				printf("[%03d] Value: %d, Time: %d\n\r", ind, Intervals[ind].Value, Intervals[ind].Time);
			return 0;
		} else if (strcmp (argv[i], _CMD_SET) == 0)
		{
			if (++i < argc)
			{
				freq = atoi (argv[i]);
				freq = GetFrequencyInterval(freq);
				if (freq == 0)
				{
					print("Frequency out of range\n\r");
					return -1;
				}
				printf("SetFrequency [%d]\n\r", freq);
				SetCarrierFrequency(freq);
				return 0;
			}
			else
			{
				print ("Frequency value not found\n\r");
				return 1;
			}
			return 0;
		}
		else {
			print ("command: '");
			print ((char*)argv[i]);
			print ("' Not found.\n\r");
		}
		i++;
	}
	return 0;
}

#ifdef _USE_COMPLETE
//*****************************************************************************
// completion callback for microrl library
char ** complet (int argc, const char * const * argv)
{
	int j = 0;

	compl_world [0] = NULL;

	// if there is token in cmdline
	if (argc == 1) {
		// get last entered token
		char * bit = (char*)argv [argc-1];
		// iterate through our available token and match it
		for (int i = 0; i < _NUM_OF_CMD; i++) {
			// if token is matched (text is part of our token starting from 0 char)
			if (strstr(keyworld [i], bit) == keyworld [i]) {
				// add it to completion set
				compl_world [j++] = keyworld [i];
			}
		}
	} else { // if there is no token in cmdline, just print all available token
		for (; j < _NUM_OF_CMD; j++) {
			compl_world[j] = keyworld [j];
		}
	}

	// note! last ptr in array always must be NULL!!!
	compl_world [j] = NULL;
	// return set of variants
	return compl_world;
}
#endif

//*****************************************************************************
void sigint (void)
{
	print ("^C catched!\n\r");
}
