/*
 * pca9685.h - PCA9685 PWM Driver for STM32 HAL
 *
 * Created for Hexapod project - optimized for MG996R servos
 * Based on proven working I2C commands - no software reset
 *
 * Author: Hexapod Project
 * Date: 2025
 */

#ifndef PCA9685_H_
#define PCA9685_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

//
// PCA9685 I2C Addresses (7-bit, will be shifted automatically)
//
#define PCA9685_ADDRESS_1 0x40 // First PCA9685 (A0-A5 = GND)
#define PCA9685_ADDRESS_2 0x41 // Second PCA9685 (A0 = VCC, A1-A5 = GND)

//
// PCA9685 Register Addresses
//
#define PCA9685_MODE1 0x00	   // Mode register 1
#define PCA9685_PRESCALE 0xFE  // PWM frequency prescaler
#define PCA9685_LED0_ON_L 0x06 // LED0 output and brightness control byte 0

// PWM Configuration for MG996R servos - PROVEN working values from lamik repo
#define PCA9685_PWM_FREQUENCY 50 // Standard servo frequency: 50Hz

// Servo PWM pulse widths - TESTED AND WORKING values
#define SERVO_PWM_MIN 110 // 0° (proven working minimum)
#define SERVO_PWM_MAX 500 // 180° (proven working maximum)
#define SERVO_PWM_MID 305 // 90° (calculated: (110+500)/2 = 305)

//
// PCA9685 Handle structure
//
typedef struct
{
	I2C_HandleTypeDef *hi2c; // I2C handle pointer
	uint8_t address;		 // 7-bit I2C address (0x40 or 0x41)
	bool ready;				 // Ready flag
} PCA9685_Handle_t;

//
// Function prototypes
//

/**
 * @brief Initialize PCA9685 controller (NO SOFTWARE RESET)
 * Uses the proven working sequence from manual testing
 * @param handle: Pointer to PCA9685 handle structure
 * @param hi2c: Pointer to I2C handle (e.g., &hi2c1)
 * @param address: 7-bit I2C address (PCA9685_ADDRESS_1 or PCA9685_ADDRESS_2)
 * @retval bool: true if successful
 */
bool PCA9685_Init(PCA9685_Handle_t *handle, I2C_HandleTypeDef *hi2c, uint8_t address);

/**
 * @brief Set servo angle (0-180 degrees) - Using PROVEN values
 * Based on working values from lamik repo that you tested months ago
 * @param handle: Pointer to PCA9685 handle
 * @param channel: PWM channel (0-15)
 * @param angle: Servo angle in degrees (0.0 - 180.0)
 * @retval bool: true if successful
 */
bool PCA9685_SetServoAngle(PCA9685_Handle_t *handle, uint8_t channel, float angle);

/**
 * @brief Test different PWM ranges for finding MG996R limits
 * @param handle: Pointer to PCA9685 handle
 * @param channel: PWM channel (0-15)
 * @param pwm_min: Minimum PWM value to test
 * @param pwm_max: Maximum PWM value to test
 * @retval bool: true if successful
 */
bool PCA9685_TestPWMRange(PCA9685_Handle_t *handle, uint8_t channel, uint16_t pwm_min, uint16_t pwm_max);

/**
 * @brief Set raw PWM value (0-4095)
 * Direct register write - most reliable method
 * @param handle: Pointer to PCA9685 handle
 * @param channel: PWM channel (0-15)
 * @param pwm_value: PWM pulse width (0-4095)
 * @retval bool: true if successful
 */
bool PCA9685_SetPWM(PCA9685_Handle_t *handle, uint8_t channel, uint16_t pwm_value);

/**
 * @brief Turn off PWM on specific channel
 * @param handle: Pointer to PCA9685 handle
 * @param channel: PWM channel (0-15)
 * @retval bool: true if successful
 */
bool PCA9685_SetChannelOff(PCA9685_Handle_t *handle, uint8_t channel);

#endif /* PCA9685_H_ */