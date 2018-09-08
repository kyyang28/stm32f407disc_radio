
#include "rx.h"			// including time.h

void processRx(timeUs_t currentTimeUs)
{
	calculateRxChannelsAndUpdateFailsafe(currentTimeUs);
}
