#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_gpio.h>

#include "microrl/config.h"
#include <string.h>
#include <stdlib.h>
#include <Terminal.h>
#include <API.h>

extern signed int printf(const char *pFormat, ...);
extern void PrintChar(const char c);

#define _STM_32_VRESION_ "1.0"

// definition commands word
#define _CMD_HELP	"help"
#define _CMD_CLEAR	"clear"
#define _CMD_SCAN	"scan"
#define _CMD_RUN	"run"
#define _CMD_SET    "set"
#define _CMD_PRINT  "print"
#define _CMD_PIN    "pin"



#define _NUM_OF_CMD 7

//available  commands
char * keyworld [] = {_CMD_HELP, _CMD_CLEAR, _CMD_SCAN, _CMD_RUN, _CMD_SET, _CMD_PRINT, _CMD_PIN};
// array for comletion
char * compl_world [_NUM_OF_CMD + 1];


static IRCode _DebugCodes[2];

static uint32_t const _DebugCodesCount = sizeof(_DebugCodes) / sizeof(_DebugCodes[0]);


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
	print ("\tscan n	- scanning ir code and stre to debug storage with number n\n\r");
	print ("\trun [code] [channel]	- send stored [code] to [channel]\n\r");
	print ("\tset frequency	- set carier frequency\n\r");
	print ("\tprint	[code] - print debug output [code]\n\r");
	print ("\tclear - clear display\n\r");
}


//*****************************************************************************
// execute callback for microrl library
// do what you want here, but don't write to argv!!! read only!!
int execute (int argc, const char * const * argv)
{
	int i = 0;
	uint32_t arg = 0, arg2 = 0;
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
		}
		else if (strcmp (argv[i], _CMD_SCAN) == 0)
		{
			if (++i < argc)
			{
				arg = atoi(argv[i]);
				if (arg >= _DebugCodesCount)
				{
					print("Code out of range\n\r");
					return -1;
				}
				Scan(&_DebugCodes[arg]);
				print("Scanning ...\n\r");
				while(IsScanning()) { ; }
				DebugPrint(&_DebugCodes[arg]);
				return 0;
			}
			else
			{
				print("Code number not found\n\r");
				return -1;
			}
		} else if (strcmp (argv[i], _CMD_PIN) == 0)
		{
			if (++i < argc)
			{
				arg = atoi (argv[i]);
				if (arg > MaxChannelNumber)
				{
					print("Pin number out of range\n\r");
					return -1;
				}

				if (++i < argc)
				{
					arg2 = atoi (argv[i]);
				}
				else
				{
					printf("Value not found\n\r");
					return -1;
				}
				SendCodeToChannel(NULL, 0); //
				SetOutValueToChannel(arg, arg2 > 0 ? 1 : 0);
				return 0;

			} else
			{
				print("Pin number not found\n\r");
				return -1;
			}

		} else if (strcmp (argv[i], _CMD_PRINT) == 0)
		{
			if (++i < argc)
			{
				arg = atoi (argv[i]);
				if (arg > (_DebugCodesCount-1))
				{
					print("Code number out of range\n\r");
					return -1;
				}
			}
			DebugPrint(&(_DebugCodes[arg]));
			return 0;
		} else if (strcmp (argv[i], _CMD_SET) == 0)
		{
			if (++i < argc)
			{
				arg = atoi (argv[i]);
				arg = GetFrequencyInterval(arg);
				if (arg == 0)
				{
					print("Frequency out of range\n\r");
					return -1;
				}
				printf("SetFrequency [%d]\n\r", (unsigned int)arg);
				SetCarrierFrequency(arg);
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
