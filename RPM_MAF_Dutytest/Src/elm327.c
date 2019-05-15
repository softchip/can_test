#include "elm327.h"


void send_command(char * cmd)
{
	
	
}




void elm_reset(void)
{
    send_command("atz\n");	
}

