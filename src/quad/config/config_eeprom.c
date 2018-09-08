
#include "config.h"

void readEEPROM(void)
{
	/* Sanity check */
	
	
//	suspendRxSignal();
	
	/* Read flash */
	
	
//	setProfile(masterConfig.current_profile_index);
	
	validateAndFixConfig();
	activateConfig();
	
//	resumeRxSignal();
}
