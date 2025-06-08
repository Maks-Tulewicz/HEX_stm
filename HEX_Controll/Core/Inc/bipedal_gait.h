/*
 * bipedal_gait.h - Bipedal gait dla hexapoda - FIXED HEADER
 *
 * Author: Hexapod Project
 * Date: 2025
 */

#ifndef BIPEDAL_GAIT_H
#define BIPEDAL_GAIT_H

#include "stm32f4xx_hal.h"
#include "pca9685.h"
#include "hexapod_kinematics.h"
#include <stdint.h>
#include <stdbool.h>

// Kierunki ruchu bipedal gait
typedef enum
{
    BIPEDAL_FORWARD = 0,
    BIPEDAL_BACKWARD,
    BIPEDAL_LEFT,
    BIPEDAL_RIGHT
} BipedalDirection_t;

// Konfiguracja bipedal gait
typedef struct
{
    float step_length;              // Długość kroku [cm]
    float lift_height;              // Wysokość podniesienia [cm]
    uint32_t step_duration_ms;      // Czas kroku jednej pary [ms]
    uint32_t hip_shift_duration_ms; // Czas przesuwu bioder [ms]
    int step_points;                // Punkty interpolacji kroku
    int hip_shift_points;           // Punkty interpolacji przesuwu bioder
    float step_height_base;         // Bazowa wysokość stania [cm]
} BipedalConfig_t;

// Publiczne funkcje API
bool bipedalGaitCycle(PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2, BipedalDirection_t direction);
bool bipedalGaitWalk(PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2, BipedalDirection_t direction, int num_cycles);

// Konfiguracja
void setBipedalConfig(float step_length, float lift_height, uint32_t step_duration, int step_points);
void printBipedalConfig(void);

#endif // BIPEDAL_GAIT_H