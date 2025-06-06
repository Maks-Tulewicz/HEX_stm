/*
 * pca9685.c - PCA9685 PWM Driver for STM32 HAL
 *
 * Created for Hexapod project - optimized for MG996R servos
 * Based on proven working I2C commands - no software reset
 *
 * Author: Hexapod Project
 * Date: 2025
 */

#include "pca9685.h"

/**
 * @brief Initialize PCA9685 controller (NO SOFTWARE RESET)
 *
 * This function uses the EXACT working sequence from manual testing:
 * 1. Test I2C communication
 * 2. Set MODE1 register to 0x20 (auto-increment enabled)
 * 3. Configure 50Hz frequency (prescaler = 121)
 * 4. No software reset to avoid instability
 */
bool PCA9685_Init(PCA9685_Handle_t *handle, I2C_HandleTypeDef *hi2c, uint8_t address)
{
	// Validate input parameters
	if (handle == NULL || hi2c == NULL)
	{
		return false;
	}

	// Initialize handle structure
	handle->hi2c = hi2c;
	handle->address = address;
	handle->ready = false;

	// Test I2C communication first
	if (HAL_I2C_IsDeviceReady(hi2c, address << 1, 3, 1000) != HAL_OK)
	{
		return false;
	}

	// Step 1: Set normal mode with auto-increment (WORKING VALUE: 0x20)
	uint8_t mode1_data = 0x20; // Auto-increment enabled (bit 5)
	if (HAL_I2C_Mem_Write(hi2c, address << 1, PCA9685_MODE1, 1, &mode1_data, 1, 1000) != HAL_OK)
	{
		return false;
	}

	// Step 2: Set frequency to 50Hz (WORKING PRESCALER: 121)
	// Formula: prescaler = round(25MHz/(4096*50Hz)) - 1 = 121
	uint8_t prescaler = 121;

	// Enter sleep mode to change prescaler
	uint8_t sleep_mode = 0x10; // Sleep bit set (bit 4)
	if (HAL_I2C_Mem_Write(hi2c, address << 1, PCA9685_MODE1, 1, &sleep_mode, 1, 1000) != HAL_OK)
	{
		return false;
	}

	// Write prescaler register
	if (HAL_I2C_Mem_Write(hi2c, address << 1, PCA9685_PRESCALE, 1, &prescaler, 1, 1000) != HAL_OK)
	{
		return false;
	}

	// Exit sleep mode (restore normal mode)
	if (HAL_I2C_Mem_Write(hi2c, address << 1, PCA9685_MODE1, 1, &mode1_data, 1, 1000) != HAL_OK)
	{
		return false;
	}

	// Small delay for oscillator to stabilize
	HAL_Delay(5);

	handle->ready = true;
	return true;
}

/**
 * @brief Set servo angle with EXTENDED PWM range for full 180°
 *
 * Testing wider PWM range for MG996R's full 180° rotation:
 * - 0°   -> SERVO_PWM_MIN (100) - extended minimum
 * - 90°  -> SERVO_PWM_MID (375) - tested working center
 * - 180° -> SERVO_PWM_MAX (650) - extended maximum
 */
bool PCA9685_SetServoAngle(PCA9685_Handle_t *handle, uint8_t channel, float angle)
{
	if (handle == NULL || !handle->ready || channel > 15)
	{
		return false;
	}

	// Limit angle to 0-180° range
	if (angle < 0)
		angle = 0;
	if (angle > 180)
		angle = 180;

	// Linear interpolation for full 180° range
	uint16_t pwm_range = SERVO_PWM_MAX - SERVO_PWM_MIN; // 650 - 100 = 550
	uint16_t pwm_value = SERVO_PWM_MIN + (uint16_t)((angle / 180.0f) * pwm_range);

	return PCA9685_SetPWM(handle, channel, pwm_value);
}

/**
 * @brief Test function to find optimal PWM range for MG996R
 * Use this to experiment with different PWM values
 */
bool PCA9685_TestPWMRange(PCA9685_Handle_t *handle, uint8_t channel, uint16_t pwm_min, uint16_t pwm_max)
{
	if (handle == NULL || !handle->ready || channel > 15)
	{
		return false;
	}

	// Test minimum position
	PCA9685_SetPWM(handle, channel, pwm_min);
	HAL_Delay(2000);

	// Test center position
	uint16_t pwm_mid = pwm_min + ((pwm_max - pwm_min) / 2);
	PCA9685_SetPWM(handle, channel, pwm_mid);
	HAL_Delay(2000);

	// Test maximum position
	PCA9685_SetPWM(handle, channel, pwm_max);
	HAL_Delay(2000);

	return true;
}

/**
 * @brief Set raw PWM value - uses EXACT working register write sequence
 *
 * This function replicates the proven manual I2C commands that made servo move:
 * HAL_I2C_Mem_Write(&hi2c1, 0x80, 0x06, 1, (uint8_t[]){0, 0, pwm_low, pwm_high}, 4, 1000);
 *
 * Each channel uses 4 registers: ON_L, ON_H, OFF_L, OFF_H
 * - ON = 0 (PWM starts at beginning of cycle)
 * - OFF = pwm_value (PWM ends after pwm_value steps)
 */
bool PCA9685_SetPWM(PCA9685_Handle_t *handle, uint8_t channel, uint16_t pwm_value)
{
	if (handle == NULL || !handle->ready || channel > 15)
	{
		return false;
	}

	// Limit PWM to 12-bit maximum
	if (pwm_value > 4095)
	{
		pwm_value = 4095;
	}

	// Calculate base register for this channel
	// Channel 0 = 0x06, Channel 1 = 0x0A, etc. (each channel uses 4 registers)
	uint8_t base_reg = PCA9685_LED0_ON_L + (4 * channel);

	// Prepare PWM data exactly like the working manual commands:
	// [ON_L, ON_H, OFF_L, OFF_H] = [0, 0, pwm_low, pwm_high]
	uint8_t pwm_data[4] = {
		0x00,					// ON_L: Low byte of ON time (0)
		0x00,					// ON_H: High byte of ON time (0)
		pwm_value & 0xFF,		// OFF_L: Low byte of OFF time
		(pwm_value >> 8) & 0xFF // OFF_H: High byte of OFF time
	};

	// Write all 4 registers in one transaction (auto-increment enabled)
	// This replicates: HAL_I2C_Mem_Write(&hi2c1, address<<1, base_reg, 1, pwm_data, 4, 1000)
	if (HAL_I2C_Mem_Write(handle->hi2c, handle->address << 1, base_reg, 1, pwm_data, 4, 1000) != HAL_OK)
	{
		return false;
	}

	return true;
}

/**
 * @brief Turn off PWM channel completely
 * Sets PWM value to 0 (no pulse output)
 */
bool PCA9685_SetChannelOff(PCA9685_Handle_t *handle, uint8_t channel)
{
	if (handle == NULL || !handle->ready || channel > 15)
	{
		return false;
	}

	// Set PWM to 0 (no pulse)
	return PCA9685_SetPWM(handle, channel, 0);
}