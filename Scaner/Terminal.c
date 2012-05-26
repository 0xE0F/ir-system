#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_gpio.h>

#include <../microrl/config.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <../IR/IR.h>
#include <IRScanner.h>
#include <Terminal.h>

extern void PrintChar(const char c);
extern void RunScan(void);

static const char* Version = "1.0";

// definition commands word

#define _CMD_HELP	"help"
#define _CMD_CLEAR	"clear"
#define _CMD_SCAN	"scan"
#define _CMD_PRINT  "print"
#define _CMD_IR_DEBUG	"ir_debug"
#define _CMD_SHOW_PARAM	"params"
#define _CMD_SET_LENGTH_DELTA	"set_length_delta"
#define _CMD_SET_INTERVALS_DELTA	"set_interval_delta"


#define _NUM_OF_CMD 8

//available  commands
char * keyworld [] = {_CMD_HELP, _CMD_CLEAR, _CMD_SCAN, _CMD_PRINT, _CMD_IR_DEBUG, _CMD_SHOW_PARAM, _CMD_SET_LENGTH_DELTA, _CMD_SET_INTERVALS_DELTA};
// array for comletion
char * compl_world [_NUM_OF_CMD + 1];


extern IRCode IrCodes[];

//*****************************************************************************
void print (const char * str)
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
	print ("\thelp - this help\n\r");
	print ("\tclear - clear display\n\r");
	print ("\tscan	- scanning ir code\n\r");
	print ("\tparams	- show parameters\n\r");
	print ("\tprint	[0,1] - print debug output [code]\n\r");
	print ("\tir_debug [0,1] - on or off debug ir codes\n\r");
	print ("\tset_length_delta <digital> - set max difference value between length of codes\n\r");
	print ("\tset_interval_delta <digital> - set max difference value between interval of codes\n\r");
}


//*****************************************************************************
// execute callback for microrl library
// do what you want here, but don't write to argv!!! read only!!
int execute (int argc, const char * const * argv)
{
	int i = 0;
	// just iterate through argv word and compare it with your commands
	while (i < argc)
	{
		if (strcmp (argv[i], _CMD_HELP) == 0)
		{
			print ("microrl v");
			print (MICRORL_LIB_VER);
			print (" library STM32 v");
			print (Version);
			print("\n\r");
			print_help ();        // print help
		}

		else if (strcmp (argv[i], _CMD_CLEAR) == 0) {
			print ("\033[2J");    // ESC seq for clear entire screen
			print ("\033[H");     // ESC seq for move cursor at left-top corner
		}
		else if (strcmp (argv[i], _CMD_SCAN) == 0)
		{
			RunScan();
			return 0;
		} else if (strcmp (argv[i], _CMD_PRINT) == 0)
		{
			if (++i < argc)
			{
				unsigned int num = atoi(argv[i]);
				if (num > 1)
				{
					printf("Error: argument must be 0 or 1\n\r");
					return -1;
				}
				DebugPrint(&(IrCodes[num]));
				return 0;
			}
			else
			{
				print("Not enough arguments\n\r");
				return -1;
			}
			return 0;
		}
		else if (strcmp (argv[i], _CMD_IR_DEBUG) == 0)
		{
			if (++i < argc)
			{
				int res = atoi(argv[i]);
				DebugModeIr = (res > 0);
			}
			else
			{
				print("Not enough arguments\n\r");
				return -1;
			}
			return 0;
		}
		else if (strcmp (argv[i], _CMD_SET_LENGTH_DELTA) == 0)
		{
			if (++i < argc)
			{
				LengthDelataMax = atoi(argv[i]);
				printf("Max length delta: %u\n\r", (unsigned int)LengthDelataMax);
			}
			else
			{
				print("Not enough arguments\n\r");
				return -1;
			}
			return 0;
		}
		else if (strcmp (argv[i], _CMD_SET_INTERVALS_DELTA) == 0)
		{
			if (++i < argc)
			{
				IntervalDelataMax = atoi(argv[i]);
				printf("Max interval delta: %u\n\r", (unsigned int)IntervalDelataMax);
			}
			else
			{
				print("Not enough arguments\n\r");
				return -1;
			}
			return 0;
		}
		else if (strcmp (argv[i], _CMD_SHOW_PARAM) == 0)
		{
			printf("Ir debug mode: %u\n\r", (unsigned int)DebugModeIr);
			printf("Max length delta: %u\n\r", (unsigned int)LengthDelataMax);
			printf("Max interval delta: %u\n\r", (unsigned int)IntervalDelataMax);
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
