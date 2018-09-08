
#include "rc_controls.h"

static motorConfig_t *motorConfig;
static pidProfile_t *pidProfile;

/* true if arming is done via the sticks (as opposed to a switch) */
static bool isUsingSticksToArm = true;

bool isModeActivationConditionPresent(modeActivationCondition_t *modeActivationConditions, boxId_e modeId)
{
	uint8_t index;
	
	for (index = 0; index < MAX_MODE_ACTIVATION_CONDITION_COUNT; index++) {
		modeActivationCondition_t *modeActivationCondition = &modeActivationConditions[index];
		
		if (modeActivationCondition->modeId == modeId && IS_RANGE_USABLE(&modeActivationCondition->range)) {
			return true;
		}
	}
	
	return false;
}

void useRcControlsConfig(modeActivationCondition_t *modeActivationConditions, motorConfig_t *motorConfigToUse, pidProfile_t *pidProfileToUse)
{
	motorConfig = motorConfigToUse;
	pidProfile = pidProfileToUse;
	
	isUsingSticksToArm = !isModeActivationConditionPresent(modeActivationConditions, BOXARM);
}
