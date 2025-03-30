/*
 * AHRSroutines.c
 *
 *  Created on: Mar 30, 2025
 *      Author: HeroPC
 */
#include "AHRSroutines.h"

FusionAhrs ahrs;
FusionAhrsSettings Ahrs_settings;
FusionOffset AHRS_offset;

FusionEuler AHRS_euler;
FusionVector AHRS_earth;
FusionAhrsInternalStates AHRS_FAIS;

clock_t AHRS_previousTimestamp;
clock_t AHRS_timestamp;

int AHRS_count = 0;


float Heading;


void AHRS_Full_Init(I2C_HandleTypeDef  hi2cX) {

	printf("\nAHRS Init ..........\n");
	printf("ADXL345 Init .....\n");
	ADXL345_Conf(hi2cX);
	ADXL344_SetMaxG(hi2cX, 2);
	printf("ADXL345 Calibration  - Don't move.....\n");
	ADXL345_Calibartion(hi2cX, 1000);
	printf("ADXL345 Calibration  - Finish \n");

	printf("QMC5883L Init .....\n");
	QMC5883L_Initialize(hi2cX, MODE_CONTROL_CONTINUOUS, OUTPUT_DATA_RATE_200HZ,
			FULL_SCALE_2G, OVER_SAMPLE_RATIO_512);

	printf("Init Gyro .....\n");
	InitGyro(hi2cX);
	printf("Calibration  Gyro - Don't move.....\n");
	Calibration_Gyro(hi2cX);
	printf("Calibration  Gyro - Finish. \n");

	printf("AHRS FusionOffsetInitialise \n");
 	FusionOffsetInitialise(&AHRS_offset, AHRS_SAMPLE_RATE);
	printf("AHRS FusionAhrsInitialise \n");
	FusionAhrsInitialise(&ahrs);

	Ahrs_settings.convention = FusionConventionNwu;
	Ahrs_settings.gain = 0.5f;
	Ahrs_settings.gyroscopeRange = 2000.0f; /* replace this with actual gyroscope range in degrees/s */
	Ahrs_settings.accelerationRejection = 10.0f;
	Ahrs_settings.magneticRejection = 10.0f;
	Ahrs_settings.recoveryTriggerPeriod = 5 * AHRS_SAMPLE_RATE; /* 5 seconds */

	printf("AHRS FusionAhrsSetSettings \n");
	FusionAhrsSetSettings(&ahrs, &Ahrs_settings);
	printf("AHRS Init complete \n");

};




void AHRS_Calculation(I2C_HandleTypeDef  hi2cX) {

	FusionVector AHRS_gyroscope;
	FusionVector AHRS_accelerometer;
	FusionVector AHRS_magnetometer;

	float AHRS_deltaTime;

	float cAccX,cAccY,cAccZ;
	float cMagX,cMagY,cMagZ;
	float GyroX,GyroY,GyroZ;

// 	QMC5883L_Read_Compensated(hi2cX, &cMagX, &cMagY, &cMagZ);
//	ADXL345_Read_G(hi2cX, &cAccX, &cAccY, &cAccZ);
// 	ReadGyro(hi2cX, &GyroX, &GyroY, &GyroZ);


	AHRS_gyroscope.array[0] = GyroX;
	AHRS_gyroscope.array[1] = GyroY;
	AHRS_gyroscope.array[2] = GyroZ;

	AHRS_accelerometer.array[0] = cAccX;
	AHRS_accelerometer.array[1] = cAccY;
	AHRS_accelerometer.array[2] = cAccZ;

	AHRS_magnetometer.array[0] = cMagX;
	AHRS_magnetometer.array[1] = cMagY;
	AHRS_magnetometer.array[2] = cMagZ;

	AHRS_timestamp =  HAL_GetTick()  ;
	AHRS_deltaTime = (float) (AHRS_timestamp - AHRS_previousTimestamp)	/ (float)1000.0;
	AHRS_previousTimestamp = AHRS_timestamp;




 // 	FusionAhrsUpdate(&ahrs, AHRS_gyroscope, AHRS_accelerometer, AHRS_magnetometer, AHRS_deltaTime);
 	AHRS_count++;


//	AHRS_euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
//	AHRS_earth = FusionAhrsGetEarthAcceleration(&ahrs);
};

void AHRS_Calculation_print(void) {

//	const FusionEuler euler = FusionQuaternionToEuler(
//			FusionAhrsGetQuaternion(&ahrs));
//	const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

//	FusionAhrsInternalStates FAIS = FusionAhrsGetInternalStates(&ahrs);

//	printf(
//			"AccX %0.3f AccY %0.3f, AccZ %0.3f, GyroX %0.3f, GyroY %0.3f, GyroZ %0.3f, MagX %0.3f, MagY %0.3f, MagZ %0.3f -- ",
//			accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z,
//			gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z,
//			magnetometer.axis.x, magnetometer.axis.y, magnetometer.axis.z);

//	printf("Roll %0.4f, Pitch %0.4f, Yaw %0.4f, X %0.4f, Y %0.4f, Z %0.4f\n",
//			euler.angle.roll, euler.angle.pitch, euler.angle.yaw, earth.axis.x,
//			earth.axis.y, earth.axis.z);

		printf("%0.4f \n",				AHRS_euler.angle.yaw );

//	printf("AE %0.3f, ART %0.3f, AI %i, ME %0.3f, MRT %0.3f, MI %i \n\n",
//			FAIS.accelerationError, FAIS.accelerationRecoveryTrigger,
//			FAIS.accelerometerIgnored, FAIS.magneticError,
//			FAIS.magneticRecoveryTrigger, FAIS.magnetometerIgnored);

}

