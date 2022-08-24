#include "MPU-60X0.h"

uint8_t messageRegister;
uint8_t messageData;
uint8_t messageDataToTransfer[2];

uint8_t gyro_data[2];
uint8_t accel_data[2];

int Gyro;
int Accel;

/*
 * @brief	Initialize MPU60X0 module
 * @param	Slave_Addres Target device I2C slave address - 7 bits address value
 *          specified in datasheet no need of shifting before calling the function
 * @param	hi2c Pointer to a I2C_HandleTypeDef structure that contains the
 * 			configuration information for the specified I2C.
 */
HAL_StatusTypeDef MPU60X0_Init(uint16_t Slave_Addres, I2C_HandleTypeDef *hi2c)
{
	messageDataToTransfer[0] = PWR_MGMT_1;
	messageDataToTransfer[1] = 0xC0;			//Register restart value

	HAL_I2C_Master_Transmit(hi2c, Slave_Addres << 1, &messageDataToTransfer[0], 2, 10);

	HAL_Delay(110);

	messageDataToTransfer[0] = PWR_MGMT_1;
	messageDataToTransfer[1] = 0x00;			//Register wake up value

	return HAL_I2C_Master_Transmit(hi2c, Slave_Addres << 1, &messageDataToTransfer[0], 2, 10);
}

/*
 * @brief	Return accelerometer value of selected axis
 * @param	Slave_Addres Target device I2C slave address - 7 bits address value
 *          specified in datasheet no need of shifting before calling the function
 * @param	hi2c Pointer to a I2C_HandleTypeDef structure that contains the
 * 			configuration information for the specified I2C.
 * @param 	axis Char value of selected axis
 */
int MPU60X0_AccelReceive(uint16_t Slave_Addres, I2C_HandleTypeDef *hi2c, char axis)
{
	if(axis >=120)
		axis=axis-32;

	switch(axis)
	{
	case 'X':
		messageRegister = GYRO_XOUT_H;
		break;
	case 'Y':
		messageRegister = GYRO_XOUT_H;
		break;
	case 'Z':
		messageRegister = GYRO_XOUT_H;
		break;
	default:
		return 0;
	}

	HAL_I2C_Master_Transmit(hi2c, Slave_Addres << 1, &messageRegister, 1, 50);
	HAL_I2C_Master_Receive(hi2c, Slave_Addres << 1, &accel_data[0], 2, 10);

	Accel = (accel_data[0] << 8) + accel_data[1];
	if(Accel >= 32767) Accel = Accel - 65536;

	return Accel;
}

/*
 * @brief	Return gyroscope value of selected axis
 * @param	Slave_Addres Target device I2C slave address - 7 bits address value
 *          specified in datasheet no need of shifting before calling the function
 * @param	hi2c Pointer to a I2C_HandleTypeDef structure that contains the
 * 			configuration information for the specified I2C.
 * @param 	axis Char value of selected axis
 */
int MPU60X0_GyroReceive(uint16_t Slave_Addres, I2C_HandleTypeDef *hi2c, char axis)
{
	if(axis >=120)
		axis=axis-32;

	switch(axis)
	{
	case 'X':
		messageRegister = ACCEL_XOUT_H;
		break;
	case 'Y':
		messageRegister = ACCEL_YOUT_H;
		break;
	case 'Z':
		messageRegister = ACCEL_ZOUT_H;
		break;
	default:
		return 0;
	}

	HAL_I2C_Master_Transmit(hi2c, Slave_Addres << 1, &messageRegister, 1, 50);
	HAL_I2C_Master_Receive(hi2c, Slave_Addres << 1, &gyro_data[0], 2, 10);

	Gyro = (gyro_data[0] << 8) + gyro_data[1];
	if(Gyro >= 32767) Gyro = Gyro - 65536;

	return Gyro;
}

/*
 * @brief	Select strength of Digital Low Pass Filter sampling
 * @param	Slave_Addres Target device I2C slave address - 7 bits address value
 *          specified in datasheet no need of shifting before calling the function
 * @param	hi2c Pointer to a I2C_HandleTypeDef structure that contains the
 * 			configuration information for the specified I2C.
 * @param 	sampling Integer value of sampling rate specified in datasheet from 0 to 6
 */
HAL_StatusTypeDef MPU60X0_DLPF(uint16_t Slave_Addres, I2C_HandleTypeDef *hi2c, int sampling)
{
	messageDataToTransfer[0] = CONFIG;
	if(sampling <= 6 && sampling >= 0)
		messageDataToTransfer[1] = sampling;			//Sampling parameter value

	return HAL_I2C_Master_Transmit(hi2c, Slave_Addres << 1, &messageDataToTransfer[0], 2, 10);
}
