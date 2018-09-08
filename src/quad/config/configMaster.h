#ifndef __CONFIGMASTER_H
#define __CONFIGMASTER_H

#include "led.h"
#include "ledTimer.h"
#include "sound_beeper.h"
#include "serial.h"
#include "gyro.h"
#include "acceleration.h"
#include "rx_pwm.h"
#include "rx.h"
#include "rc_controls.h"
#include "platform.h"			// including target.h
#include "motors.h"				// including mixer.h

typedef struct master_s {
	uint8_t version;
	uint16_t size;
	uint8_t magic_be;			// magic number, should be 0xBE
	
	uint32_t enabledFeatures;
	
	rxConfig_t rxConfig;
	
	serialPinConfig_t serialPinConfig;
	serialConfig_t	serialConfig;
	
	gyroConfig_t gyroConfig;
	accelerometerConfig_t accelerometerConfig;
	LedStatusConfig_t ledStatusConfig;
	
	LedTimerConfig_t ledTimerConfig;
	
	/* motor related stuff */
	motorConfig_t motorConfig;
	
#ifdef USE_PWM
	pwmConfig_t pwmConfig;
#endif
	
#ifdef BEEPER
	beeperConfig_t beeperConfig;
#endif
	
	uint32_t beeper_off_flags;
	uint32_t preferred_beeper_off_flags;
	
	modeActivationProfile_t modeActivationProfile;
}master_t;

extern master_t masterConfig;

#define LedStatusConfig(x)					(&masterConfig.ledStatusConfig)
#define LedTimerConfig(x)					(&masterConfig.ledTimerConfig)
#define BeeperConfig(x)						(&masterConfig.beeperConfig)
#define SerialPinConfig(x) 					(&masterConfig.serialPinConfig)
#define SerialConfig(x)						(&masterConfig.serialConfig)
#define MotorConfig(x)						(&masterConfig.motorConfig)
#define PwmConfig(x)						(&masterConfig.pwmConfig)
#define GyroConfig(x)						(&masterConfig.gyroConfig)
#define AccelerometerConfig(x)				(&masterConfig.accelerometerConfig)
#define RxConfig(x)							(&masterConfig.rxConfig)
#define ModeActivationProfile(x)			(&masterConfig.modeActivationProfile)

#endif	// __CONFIGMASTER_H
