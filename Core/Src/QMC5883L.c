/*
 * compass.c
 *
 *  Created on: Mar 2, 2025
 *      Author: HeroPC
 */

#include "QMC5883L.h"


uint8_t QMC5883L_Read_Reg(I2C_HandleTypeDef  hi2cX,uint8_t reg)
{
	uint8_t Buffer[1];
	HAL_I2C_Mem_Read(&hi2cX,QMC5883L_ADDRESS,reg,1,Buffer,1,10);
	return Buffer[0];
}

void QMC5883L_Write_Reg(I2C_HandleTypeDef  hi2cX,uint8_t reg, uint8_t data)
{
	uint8_t Buffer[2]={reg,data};
	HAL_I2C_Master_Transmit(&hi2cX,QMC5883L_ADDRESS,Buffer,2,10);
}


void QMC5883L_Read_Data(I2C_HandleTypeDef  hi2cX,int16_t *MagX,int16_t *MagY,int16_t *MagZ) // (-32768 / +32768)
{
	uint8_t buffer[6];
	HAL_I2C_Mem_Read(&hi2cX,QMC5883L_ADDRESS,QMC5883L_DATA_READ_X_LSB,1,buffer,6,10);
	*MagX=((int16_t)buffer[0] | (((int16_t)buffer[1])<<8));
	*MagY=((int16_t)buffer[2] | (((int16_t)buffer[3])<<8));
	*MagZ=((int16_t)buffer[4] | (((int16_t)buffer[5])<<8));
}


void QMC5883L_Read_Compensated(I2C_HandleTypeDef  hi2cX,float *CompensatedMagX,float *CompensatedMagY,float *CompensatedMagZ) // (-32768 / +32768)
{
	float b[3];
	float A[3][3];

	b[0]=68.2975;
	b[1]=-1705.8944;
	b[2]=-605.7223;

	A[0][0]=1.0167;
	A[0][1]=-0.018919;
	A[0][2]=6.0029e-05;
	A[1][0]=-0.018919;
	A[1][1]=1.0062;
	A[1][2]=0.0031491;
	A[2][0]=6.0029e-05;
	A[2][1]=0.0031491;
	A[2][2]=0.97787;


	int16_t MagX;
	int16_t MagY;
	int16_t MagZ;

 	QMC5883L_Read_Data(hi2cX,&MagX,&MagY,&MagZ);

	float cMagX = (float)MagX - b[0];
	float cMagY = (float)MagY - b[1];
	float cMagZ = (float)MagZ - b[2];

	*CompensatedMagX = (float)(cMagX*A[0][0]+cMagY*A[0][1]+cMagZ*A[0][2]);
	*CompensatedMagY = (float)(cMagX*A[1][0]+cMagY*A[1][1]+cMagZ*A[1][2]);
	*CompensatedMagZ = (float)(cMagX*A[2][0]+cMagY*A[2][1]+cMagZ*A[2][2]);

}


int16_t QMC5883L_Read_Temperature(I2C_HandleTypeDef  hi2cX)
{
	return (((int16_t)QMC5883L_Read_Reg(hi2cX,QMC5883L_TEMP_READ_LSB)) | (((int16_t)QMC5883L_Read_Reg(hi2cX,QMC5883L_TEMP_READ_MSB))<<8))/100;
}


void QMC5883L_Initialize(I2C_HandleTypeDef  hi2cX,_qmc5883l_MODE MODE,_qmc5883l_ODR ODR,_qmc5883l_RNG RNG,_qmc5883l_OSR OSR)
{
	QMC5883L_Write_Reg(hi2cX,QMC5883L_CONFIG_3,0x01);
	QMC5883L_Write_Reg(hi2cX,QMC5883L_CONFIG_1,MODE | ODR | RNG | OSR);
}

void QMC5883L_Reset(I2C_HandleTypeDef  hi2cX)
{
	QMC5883L_Write_Reg(hi2cX,QMC5883L_CONFIG_2,0x81);
}

void QMC5883L_InterruptConfig(I2C_HandleTypeDef  hi2cX,_qmc5883l_INT INT)
{
	if(INT==INTERRUPT_ENABLE){QMC5883L_Write_Reg(hi2cX,QMC5883L_CONFIG_2,0x00);}
	else {QMC5883L_Write_Reg(hi2cX,QMC5883L_CONFIG_2,0x01);}
}


_qmc5883l_status QMC5883L_DataIsReady(I2C_HandleTypeDef  hi2cX)
{
	uint8_t Buffer=QMC5883L_Read_Reg(hi2cX,QMC5883L_STATUS);
	if((Buffer&0x00)==0x00)	  {return NO_NEW_DATA;}
	else if((Buffer&0x01)==0X01){return NEW_DATA_IS_READY;}
	return NORMAL;
}

_qmc5883l_status QMC5883L_DataIsSkipped(I2C_HandleTypeDef  hi2cX)
{
	uint8_t Buffer=QMC5883L_Read_Reg(hi2cX,QMC5883L_STATUS);
	if((Buffer&0x00)==0X00)	  {return NORMAL;}
	else if((Buffer&0x04)==0X04){return DATA_SKIPPED_FOR_READING;}
		return NORMAL;
}

_qmc5883l_status QMC5883L_DataIsOverflow(I2C_HandleTypeDef  hi2cX)
{
	uint8_t Buffer=QMC5883L_Read_Reg(hi2cX,QMC5883L_STATUS);
	if((Buffer&0x00)==0X00)	  {return NORMAL;}
	else if((Buffer&0x02)==0X02){return DATA_OVERFLOW;}
		return NORMAL;
}



