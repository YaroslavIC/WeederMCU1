/*
 * GY85.c
 *
 *  Created on: Mar 1, 2025
 *      Author: HeroPC
 */

#include "GY85.h"


float compass_scale;
float compass_declination_offset_radians;

void compass_SetDeclination( int declination_degs , int declination_mins, char declination_dir )
{
  // Convert declination to decimal degrees
  switch (declination_dir)
  {
    // North and East are positive
    case 'E':
    	compass_declination_offset_radians = 0 - ( declination_degs + (1 / 60 * declination_mins)) * (M_PI / 180);
      break;

    // South and West are negative
    case 'W':
    	compass_declination_offset_radians =  (( declination_degs + (1 / 60 * declination_mins) ) * (M_PI / 180));
      break;
  }
}

void SetSamplingMode(I2C_HandleTypeDef hi2cX, uint16_t sampling_mode , uint16_t rate, uint16_t measure)
{
  uint8_t data = (sampling_mode | rate | measure);
  HAL_I2C_Mem_Write(&hi2cX, HMC5883L_ADDRESS, COMPASS_CONFIG_REGISTER_A, 1, &data, 1, HAL_MAX_DELAY);
}


void SetScaleMode(I2C_HandleTypeDef hi2cX,uint8_t ScaleMode)
{
  switch (ScaleMode)
   {
    case COMPASS_SCALE_088:
    	compass_scale = 0.73;
      break;
    case COMPASS_SCALE_130:
    	compass_scale = 0.92;
      break;
    case COMPASS_SCALE_190:
    	compass_scale = 1.22;
      break;
    case COMPASS_SCALE_250:
    	compass_scale = 1.52;
      break;
    case COMPASS_SCALE_400:
    	compass_scale = 2.27;
      break;
    case COMPASS_SCALE_470:
    	compass_scale = 2.56;
      break;
    case COMPASS_SCALE_560:
    	compass_scale = 3.03;
      break;
    case COMPASS_SCALE_810:
    	compass_scale = 4.35;
      break;
    default:
    	compass_scale = 0.92;
      ScaleMode = COMPASS_SCALE_130;
  }

  HAL_I2C_Mem_Write(&hi2cX, HMC5883L_ADDRESS, COMPASS_CONFIG_REGISTER_B, 1, &ScaleMode, 1, HAL_MAX_DELAY);
}

void SetMeasureMode(I2C_HandleTypeDef hi2cX, uint8_t Measure)
{
  HAL_I2C_Mem_Write(&hi2cX, HMC5883L_ADDRESS, COMPASS_MODE_REGISTER, 1, &Measure, 1, HAL_MAX_DELAY);
}

uint16_t CompasReadAxis(I2C_HandleTypeDef hi2cX,uint16_t reg) {
  uint16_t buffer[1];
  HAL_I2C_Mem_Read(&hi2cX, HMC5883L_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&buffer, 2, HAL_MAX_DELAY);
  int16_t value = buffer[0] << 8 | buffer[1];
  return value;
}


float CompasRead6Axis(I2C_HandleTypeDef hi2cX )
{
  uint8_t buffer[6];
  HAL_I2C_Mem_Read(&hi2cX, HMC5883L_ADDRESS, COMPASS_DATA_REGISTER, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&buffer, 6, HAL_MAX_DELAY);
  int16_t x = ((buffer[0] << 8) | buffer[1]) * compass_scale;
  int16_t y = ((buffer[2] << 8) | buffer[3]) * compass_scale;
  int16_t z = ((buffer[4] << 8) | buffer[5]) * compass_scale;

  float heading = atan2(x, y);
  heading += compass_declination_offset_radians;

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * M_PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * M_PI)
    heading -= 2 * M_PI;

  // Convert radians to degrees for readability.
  return heading * 180 / M_PI;
}


// To compensate a compass for Tilt sensor and Compass
float compensate(float compass_X, float compass_Y, float compass_Z, float pitch, float roll)
{

  float IMU_roll = pitch * (M_PI / 180);
  float IMU_pitch = roll * (M_PI / 180);

  float XH = compass_X * cos(IMU_pitch) + compass_Y * sin(IMU_roll) * sin(IMU_pitch) - compass_Z * cos(IMU_roll) * sin(IMU_pitch);
  float YH = compass_Y * cos(IMU_roll) + compass_Z * sin(IMU_roll);
       // Azimuth = atan2(YH / XH)
  float Azimuth = atan2(YH , XH) * 180 / M_PI;
  Azimuth += compass_declination_offset_radians; // see https://www.magnetic-declination.com/

  if (Azimuth < 0) {
    Azimuth += 360;
  }
  else if (Azimuth >= 360) {
    Azimuth -= 360;
  }

  return Azimuth;
}




