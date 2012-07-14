#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_gpio.h>

#include <../microrl/config.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <../IR/IR.h>
#include "Terminal.h"
#include "IRTransmitter.h"
#include <Storage/ffconf.h>
#include <Storage/ff.h>
#include <Storage/diskio.h>
#include <PlcTimers.h>
#include "Storage.h"
#include "NetWork.h"

extern void PrintChar(const char c);
extern void RunScan(void);
extern uint32_t TimeoutSndPackets;

static const char* Version = "1.0";

// definition commands word

#define _CMD_HELP	"help"
#define _CMD_CLEAR	"clear"
#define _CMD_ERASE  "erase"
#define _CMD_STATUS  "status"
#define _CMD_STORAGE  "storage"
#define _CMD_SHOW_PARAM	"params"
#define _CMD_SET_SND_TMT	"snd_timeout"
#define _CMD_SET_CARRIER_FREQ "freq"
#define _CMD_SET_CHANNEL_VALUE "channel"



#define _NUM_OF_CMD 9

//available  commands
char * keyworld [] = {_CMD_HELP, _CMD_CLEAR, _CMD_STATUS, _CMD_STORAGE, _CMD_SHOW_PARAM, _CMD_SET_CARRIER_FREQ, _CMD_SET_CHANNEL_VALUE, _CMD_ERASE, _CMD_SET_SND_TMT};
// array for comletion
char * compl_world [_NUM_OF_CMD + 1];

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
void print_help (void) {
	print ("Use TAB key for completion\n\rCommand:\n\r");
	print ("\thelp - this help\n\r");
	print ("\tclear - clear display\n\r");
	print ("\tparams	- show parameters\n\r");
	print ("\tstatus	- show status storage\n\r");
	print ("\tstorage	- show storage content\n\r");
	print ("\tstorage_debug [0,1] - on or off debug storage operations\n\r");
	print ("\tfreq <f> - set carrier frequency\n\r");
	print ("\tchannel [0..3] [0,1] - set channel value\n\r");
	print ("\tsnd_timeout value (in s) - set snd timeout between packets\n\r");
}


//*****************************************************************************
// execute callback for microrl library
// do what you want here, but don't write to argv!!! read only!!
int execute (int argc, const char * const * argv)
{
	int i = 0;
	// just iterate through argv word and compare it with your commands
	while (i < argc) {
		if (strcmp (argv[i], _CMD_HELP) == 0) {
			print ("microrl v");
			print (MICRORL_LIB_VER);
			print (" library STM32 v");
			print (Version);
			print("\n\r");
			print_help ();        // print help
		} else if (strcmp (argv[i], _CMD_CLEAR) == 0) {
			print ("\033[2J");    // ESC seq for clear entire screen
			print ("\033[H");     // ESC seq for move cursor at left-top corner
		} else if (strcmp (argv[i], _CMD_STATUS) == 0) {
			PrintStorageStatus();
			return 0;
		} else if (strcmp (argv[i], _CMD_STORAGE) == 0) {
			PrintConentStorage();
			return 0;
		} else if (strcmp (argv[i], _CMD_ERASE) == 0) {
			EraseStorage();
			return 0;
		} else if ( strcmp(argv[i], _CMD_SET_CARRIER_FREQ) == 0 ){

			if ( ++i < argc) {
				size_t freq = atoi(argv[i]);
				SetCarrierFrequency(freq);
			} else {
				print("Not enough arguments\n\r");
				return -1;
			}
			return 0;

		} else if ( strcmp(argv[i], _CMD_SET_SND_TMT) == 0 ){

			if ( ++i < argc) {
				TimeoutSndPackets = atof(argv[i]) / PLC_TIMER_PERIOD;
			} else {
				print("Not enough arguments\n\r");
				return -1;
			}

			return 0;

		} else if ( strcmp(argv[i], _CMD_SET_CHANNEL_VALUE) == 0 ) {

			if (++i < argc) {
				unsigned char channel = atoi(argv[i]);

				if (channel > MaxChannelNumber) {
					print("Channel number out of range\n\r");
					return -1;
				}

				if (++i < argc) {
					unsigned char value = atoi(argv[i]);

					if (value > 1) {
						printf("Channel value out of range\n\r");
						return -1;
					}

					SetOutValueToChannel(channel, value);
					return 0;
				}
			}

			print("Not enough arguments\n\r");
			return -1;

		} else if (strcmp (argv[i], _CMD_SHOW_PARAM) == 0) {
			printf("Network address: %u\n\r", (unsigned int) GetDeviceAddress());
			printf("Device type: %u\n\r", (unsigned int) GetDeviceType());
			printf("Ir debug mode: %u\n\r", (unsigned int)DebugModeIr);
			printf("Max length delta: %u\n\r", (unsigned int)LengthDelataMax);
			printf("Max interval delta: %u\n\r", (unsigned int)IntervalDelataMax);
			printf("Snd timeout (in plc resolution): %u\n\r", (unsigned int) TimeoutSndPackets);
			return 0;
		} else {
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
void sigint (void) {
	print ("^C catched!\n\r");
}
