#ifndef _MICRORL_MISC_H_
#define _MICRORL_MISC_H_

// print to stream callback
void print (char * str);

// get_char from stream
char get_char (void);

// execute callback
int execute (int argc, const char * const * argv);

// completion callback
char ** complet (int argc, const char * const * argv);

// ctrl+c callback
void sigint (void);

#define MaxIntervals 512

typedef struct
{
	uint32_t Time;
	uint32_t Value;
} _TimeInterval;

#endif
