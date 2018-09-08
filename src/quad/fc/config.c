
#include <stdio.h>				// debugging purposes
#include <string.h>
#include "configMaster.h"		// including led.h, ledTimer.h, motors.h, sound_beeper.h, target.h, serial.h
#include "config_profile.h"
//#include "led.h"
//#include "ledTimer.h"			// for testing led timer ONLY
//#include "motors.h"				// including mixer.h
//#include "sound_beeper.h"
//#include "target.h"
//#include "serial.h"
#include "sensor.h"
#include "pwm_output.h"			// including timer.h
#include "feature.h"
#include "accgyro.h"
#include "filter.h"
#include "rc_controls.h"
#include "rx.h"
#include "common.h"
#include "maths.h"

#define BRUSHED_MOTORS_PWM_RATE 			16000
#define BRUSHLESS_MOTORS_PWM_RATE 			480

/* master config structure with data independent from profiles */
master_t masterConfig;
profile_t *currentProfile;

void ResetSerialPinConfig(serialPinConfig_t *pSerialPinConfig)
{
	for (int port = 0; port < SERIAL_PORT_MAX_INDEX; port++) {
		pSerialPinConfig->ioTagRx[port] = IO_TAG(NONE);
		pSerialPinConfig->ioTagTx[port] = IO_TAG(NONE);
	}
	
	for (int index = 0; index < SERIAL_PORT_COUNT; index++) {
		switch (serialPortIdentifiers[index]) {
			case SERIAL_PORT_USART1:
#ifdef USE_UART1
				pSerialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART1)] = IO_TAG(UART1_RX_PIN);
				pSerialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART1)] = IO_TAG(UART1_TX_PIN);
#endif
				break;

			case SERIAL_PORT_USART2:
#ifdef USE_UART2
//				pSerialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART2)] = IO_TAG(UART2_RX_PIN);
//				pSerialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART2)] = IO_TAG(UART2_TX_PIN);
#endif
				break;

			case SERIAL_PORT_USART3:
#ifdef USE_UART3
				pSerialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART3)] = IO_TAG(UART3_RX_PIN);
				pSerialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART3)] = IO_TAG(UART3_TX_PIN);
#endif
				break;

			case SERIAL_PORT_USART4:
				break;

			case SERIAL_PORT_USART5:
				break;

			case SERIAL_PORT_USART6:
#ifdef USE_UART6
				pSerialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART6)] = IO_TAG(UART6_RX_PIN);
				pSerialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART6)] = IO_TAG(UART6_TX_PIN);
#endif
				break;

			case SERIAL_PORT_USART7:
				break;

			case SERIAL_PORT_USART8:
				break;

			case SERIAL_PORT_SOFTSERIAL1:
				break;

			case SERIAL_PORT_SOFTSERIAL2:
				break;
			
			case SERIAL_PORT_USB_VCP:
				break;
			
			case SERIAL_PORT_NONE:
				break;
		}
	}
}

void ResetSerialConfig(serialConfig_t *serialConfig)
{
	memset(serialConfig, 0, sizeof(serialConfig_t));
	serialConfig->serial_update_rate_hz = 100;
	serialConfig->reboot_character		= 'R';
	
	for (int index = 0; index < SERIAL_PORT_COUNT; index++) {
		serialConfig->portConfigs[index].identifier			= serialPortIdentifiers[index];
		serialConfig->portConfigs[index].msp_baudrateIndex	= BAUD_115200;
		serialConfig->portConfigs[index].gps_baudrateIndex	= BAUD_115200;
//		serialConfig->portConfigs[index].gps_baudrateIndex	= BAUD_57600;
		serialConfig->portConfigs[index].blackbox_baudrateIndex	= BAUD_115200;		// blackbox port for debugging purposes
		serialConfig->portConfigs[index].telemetry_baudrateIndex = BAUD_AUTO;
	}
	
	serialConfig->portConfigs[0].functionMask = FUNCTION_MSP;
}

void ResetLedStatusConfig(LedStatusConfig_t *ledStatusConfig)
{
	for (int i = 0; i < LED_NUMBER; i++) {
		ledStatusConfig->ledTags[i] = IO_TAG_NONE;
	}
	
#ifdef LED3
	ledStatusConfig->ledTags[0] = IO_TAG(LED3);	// LED3 = PD13 ==> DEFIO_TAG__PD13 ==> 4D
#endif

#ifdef LED4
	ledStatusConfig->ledTags[1] = IO_TAG(LED4);	// LED4 = PD12 ==> DEFIO_TAG__PD12 ==> 4C
#endif

#ifdef LED5
	ledStatusConfig->ledTags[2] = IO_TAG(LED5);	// LED5 = PD14 ==> DEFIO_TAG__PD14 ==> 4E
#endif

#ifdef LED6
	ledStatusConfig->ledTags[3] = IO_TAG(LED6);	// LED6 = PD15 ==> DEFIO_TAG__PD15 ==> 4F
#endif
	
	ledStatusConfig->polarity = 0;
}

void ResetLedTimerConfig(LedTimerConfig_t *ledTimerConfig)
{
	int ledTimerIndex = 0;
	for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT && ledTimerIndex < LED_NUMBER; i++) {
		if (timerHardware[i].usageFlags & TIM_USE_LED) {
			ledTimerConfig->ioTags[ledTimerIndex] = timerHardware[i].tag;
			ledTimerIndex++;
		}
	}
}

void ResetMotorConfig(motorConfig_t *motorConfig)
{
	motorConfig->minthrottle = 1070;
	motorConfig->motorPwmRate = BRUSHLESS_MOTORS_PWM_RATE;		// BRUSHLESS_MOTORS_PWM_RATE = 480
	motorConfig->motorPwmProtocol = PWM_TYPE_STANDARD;
	motorConfig->maxthrottle = 2000;
	motorConfig->mincommand = 1000;
	motorConfig->useUnsyncedPwm = true;
	
	int motorIndex = 0;
	for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT && motorIndex < MAX_SUPPORTED_MOTORS; i++) {
		if (timerHardware[i].usageFlags & TIM_USE_MOTOR) {
			motorConfig->ioTags[motorIndex] = timerHardware[i].tag;
			motorIndex++;
		}
	}
}

#if defined(USE_PWM)		// PWM rx signals
void ResetPwmConfig(pwmConfig_t *pwmConfig)
{
	int inputIndex = 0;
	for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT && inputIndex < PWM_INPUT_PORT_COUNT; i++) {
		if (timerHardware[i].usageFlags & TIM_USE_PWM) {
			pwmConfig->ioTags[inputIndex] = timerHardware[i].tag;
			inputIndex++;
		}
	}
}
#endif

/*
 * serialPortIdentifiers[0] = USB_VCP
 * serialPortIdentifiers[1] = USART1
 * serialPortIdentifiers[2] = USART2
 * serialPortIdentifiers[3] = USART3
 * serialPortIdentifiers[4] = USART4
 * serialPortIdentifiers[5] = USART5
 * serialPortIdentifiers[6] = USART6
 * serialPortIdentifiers[7] = USART7
 * serialPortIdentifiers[8] = USART8
 * serialPortIdentifiers[9] = SOFTSERIAL1
 * serialPortIdentifiers[10] = SOFTSERIAL2
 */
void targetConfiguration(master_t *config)
{
	/* USART1 */
	int index = findSerialPortIndexByIdentifier(SERIAL_PORT_USART1);			// index = 0 (SERIAL_PORT_USART1)
	config->serialConfig.portConfigs[index].functionMask = FUNCTION_RX_SERIAL;

	/* USART2 */
//	index = findSerialPortIndexByIdentifier(SERIAL_PORT_USART2);
//	config->serialConfig.portConfigs[index].functionMask = FUNCTION_GPS;
	
	/* USART3 */
	index = findSerialPortIndexByIdentifier(SERIAL_PORT_USART3);		// index = 1 (serialPortIdentifiers[1] contains SERIAL_PORT_USART3 which is 2)
	config->serialConfig.portConfigs[index].functionMask = FUNCTION_BLACKBOX;			// USART3 used for printf debugger

	/* USART6 */
	index = findSerialPortIndexByIdentifier(SERIAL_PORT_USART6);		// index = 2 (serialPortIdentifiers[2] contains SERIAL_PORT_USART6 which is 5)
	config->serialConfig.portConfigs[index].functionMask = FUNCTION_TELEMETRY_FRSKY;	// USART6 used for FRSKY TELEMETRY
}

#ifdef BEEPER
void ResetBeeperConfig(beeperConfig_t *beeperConfig)
{
#ifdef BEEPER_INVERTED
	beeperConfig->isOpenDrain = false;
	beeperConfig->isInverted = true;
#else
	beeperConfig->isOpenDrain = false;			// use IO push-pull, with both transistors connected to the VCC and GND
//	beeperConfig->isOpenDrain = true;			// use IO open-drain, no transistor connected to VCC, only with transistor connected to the GND
	beeperConfig->isInverted = false;
#endif
	beeperConfig->ioTag = IO_TAG(BEEPER);
}
#endif

void CreateDefaultConfig(master_t *config)
{
	/* Clear all configuration */
	memset(config, 0, sizeof(master_t));
	
	/* Feature configuration */
	uint32_t *featuresPtr = &config->enabledFeatures;
	intFeatureClearAll(featuresPtr);
	intFeatureSet(DEFAULT_RX_FEATURE, featuresPtr);
	
	/* Global settings */
	config->gyroConfig.gyro_lpf = GYRO_LPF_256HZ;			// 256Hz default
#ifdef STM32F10X
	config->gyroConfig.gyro_sync_denom = 8;
//	config->pidConfig.pid_process_denom = 1;
#elif defined(USE_GYRO_SPI_MPU6000) || defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250)
	config->gyroConfig.gyro_sync_denom = 1;
//	config->pidConfig.pid_process_denom = 4;
#else
	config->gyroConfig.gyro_sync_denom = 4;
//	config->pidConfig.pid_process_denom = 2;
#endif	
	config->gyroConfig.gyro_soft_lpf_type = FILTER_PT1;
	config->gyroConfig.gyro_soft_lpf_hz = 90;
	config->gyroConfig.gyro_soft_notch_hz_1 = 400;
	config->gyroConfig.gyro_soft_notch_cutoff_1 = 300;
	config->gyroConfig.gyro_soft_notch_hz_2 = 200;
	config->gyroConfig.gyro_soft_notch_cutoff_2 = 100;
	
	config->gyroConfig.gyro_align = ALIGN_DEFAULT;
	config->accelerometerConfig.acc_align = ALIGN_DEFAULT;
	
	/* This threshold means how much average gyro reading could differ before re-calibration is triggered */
	config->gyroConfig.gyroMovementCalibrationThreshold = 48;		// moron_threshold of CLI, range from 0 to 200
	config->accelerometerConfig.acc_hardware = ACC_DEFAULT;
	
	ResetSerialPinConfig(&config->serialPinConfig);
	ResetSerialConfig(&config->serialConfig);
	ResetLedStatusConfig(&config->ledStatusConfig);
	ResetLedTimerConfig(&config->ledTimerConfig);
	ResetMotorConfig(&config->motorConfig);

#ifdef USE_PWM
	ResetPwmConfig(&config->pwmConfig);
	config->pwmConfig.inputFilteringMode = INPUT_FILTERING_DISABLED;
#endif

	config->rxConfig.serialrx_provider = SERIALRX_SBUS;
	config->rxConfig.halfDuplex = 0;
	config->rxConfig.rx_spi_protocol = 0;			// TODO: 0 for now
	config->rxConfig.sbus_inversion = 1;
	config->rxConfig.spektrum_sat_bind = 0;
	config->rxConfig.spektrum_sat_bind_autoreset = 1;
	config->rxConfig.midrc = 1500;
	config->rxConfig.mincheck = 1100;
	config->rxConfig.maxcheck = 1900;
	config->rxConfig.rx_min_usec = 885;				// any of first 4 channels below this value will trigger rx loss detection
	config->rxConfig.rx_max_usec = 2115;			// any of first 4 channels above this value will trigger rx loss detection

	/* Failsafe initialisation */
    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rxFailsafeChannelConfiguration_t *channelFailsafeConfiguration = &config->rxConfig.failsafe_channel_configurations[i];
		/* i < NON_AUX_CHANNEL_COUNT means i = 0, 1, 2, 3 which are ROLL, PITCH, THROTTLE, YAW
		 * Related to getRxfailValue() function
		 */
        channelFailsafeConfiguration->mode = (i < NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_MODE_AUTO : RX_FAILSAFE_MODE_HOLD;
        channelFailsafeConfiguration->step = (i == THROTTLE) ? CHANNEL_VALUE_TO_RXFAIL_STEP(config->rxConfig.rx_min_usec) : CHANNEL_VALUE_TO_RXFAIL_STEP(config->rxConfig.midrc);
	}

	config->rxConfig.rssi_channel = 0;
	config->rxConfig.rssi_scale = RSSI_SCALE_DEFAULT;
	config->rxConfig.rssi_invert = 0;

	config->rxConfig.rcInterpolation = RC_SMOOTHING_AUTO;
	config->rxConfig.rcInterpolationChannels = 0;
	config->rxConfig.rcInterpolationInterval = 19;
	config->rxConfig.fpvCamAngleDegrees = 0;
	config->rxConfig.max_aux_channel = DEFAULT_AUX_CHANNEL_COUNT;			// DEFAULT_AUX_CHANNEL_COUNT = 18 - 4 = 14
	
	config->rxConfig.airModeActivateThreshold = 1350;
	
	resetAllRxChannelRangeConfigurations(config->rxConfig.channelRanges);		// set min and max values

//#ifdef RX_CHANNELS_TAER
//	parseRcChannels("TAER1234", &config->rxConfig);
//#else
//	printf("%s, %s, %d\r\n", __FILE__, __FUNCTION__, __LINE__);
	parseRcChannels("AETR12", &config->rxConfig);		// 6 channels mapping, A: Roll, E: Pitch, T: Throttle, R: Yaw
//	parseRcChannels("AETR1234", &config->rxConfig);		// 8 channels mapping, A: Roll, E: Pitch, T: Throttle, R: Yaw
//#endif

#ifdef BEEPER
	ResetBeeperConfig(&config->beeperConfig);
#endif

	targetConfiguration(config);
}

static void ResetConfig(void)
{
	CreateDefaultConfig(&masterConfig);
}

void ResetEEPROM(void)
{
	ResetConfig();
}

void CheckEEPROMContainsValidData(void)
{
#if 0
	if (isEEPROMContentValid()) {
		return;
	}
#endif
	
	ResetEEPROM();
}

void activateConfig(void)
{
	useRcControlsConfig(ModeActivationProfile()->modeActivationConditions, &masterConfig.motorConfig, &currentProfile->pidProfile);
}

void validateAndFixGyroConfig(void)
{
	/* Prevent invalid notch cutoff */
	if (GyroConfig()->gyro_soft_notch_cutoff_1 >= GyroConfig()->gyro_soft_notch_hz_1) {
		GyroConfig()->gyro_soft_notch_hz_1 = 0;
	}
	
	if (GyroConfig()->gyro_soft_notch_cutoff_2 >= GyroConfig()->gyro_soft_notch_hz_2) {
		GyroConfig()->gyro_soft_notch_hz_2 = 0;
	}
	
	float samplingTime = 0.000125f;			// 1 / 8000 (gyro sampling frequency 8K) = 0.000125 
	
	// TODO: implement rest of the gyroConfig stuffs
}

void validateAndFixConfig(void)
{
	if (!(featureConfigured(FEATURE_RX_PARALLEL_PWM) || featureConfigured(FEATURE_RX_SERIAL))) {
		featureSet(DEFAULT_RX_FEATURE);
	}
	
	if (featureConfigured(FEATURE_RX_SERIAL)) {
		featureClear(FEATURE_RX_PARALLEL_PWM);
	}
	
	if (featureConfigured(FEATURE_RX_PARALLEL_PWM)) {
		featureClear(FEATURE_RX_SERIAL);
	}
	
	validateAndFixGyroConfig();		// TODO
}

void beeperOffSet(uint32_t mask)
{
	masterConfig.beeper_off_flags |= mask;
}

void beeperOffSetAll(uint8_t beeperCount)
{
	masterConfig.beeper_off_flags = (1 << beeperCount) - 1;
}

void beeperOffClear(uint32_t mask)
{
	masterConfig.beeper_off_flags &= ~(mask);
}

void beeperOffClearAll(void)
{
	masterConfig.beeper_off_flags = 0;
}

uint32_t getBeeperOffMask(void)
{
	return masterConfig.beeper_off_flags;
}

void setBeeperOffMask(uint32_t mask)
{
	masterConfig.beeper_off_flags = mask;
}

uint32_t getPreferredBeeperOffMask(void)
{
	return masterConfig.preferred_beeper_off_flags;
}

void setPreferredBeeperOffMask(uint32_t mask)
{
	masterConfig.preferred_beeper_off_flags = mask;
}
