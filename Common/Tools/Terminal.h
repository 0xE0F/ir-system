#ifndef _MICRORL_MISC_H_
#define _MICRORL_MISC_H_

extern signed int printf(const char *pFormat, ...);

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


#endif
