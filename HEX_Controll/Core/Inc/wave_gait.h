/*
 * wave_gait.h - Wave gait dla hexapoda
 *
 * Koncepcja: 6 nóg chodzących sekwencyjnie (jedna po drugiej)
 * Najbardziej stabilny gait - zawsze 5 nóg na ziemi
 *
 * Author: Hexapod Project
 * Date: 2025
 */

#ifndef WAVE_GAIT_H
#define WAVE_GAIT_H

#include "stm32f4xx_hal.h"
#include "pca9685.h"
#include "hexapod_kinematics.h"
#include <stdint.h>
#include <stdbool.h>

// Kierunki ruchu wave gait
typedef enum
{
    WAVE_FORWARD = 0,
    WAVE_BACKWARD,
    WAVE_LEFT,
    WAVE_RIGHT
} WaveDirection_t;

// Konfiguracja wave gait
typedef struct
{
    float step_length;         // Długość kroku [cm]
    float lift_height;         // Wysokość podniesienia [cm]
    uint32_t step_duration_ms; // Czas swing jednej nogi [ms]
    int step_points;           // Punkty interpolacji swing
    float step_height_base;    // Bazowa wysokość stania [cm]
} WaveConfig_t;

// API funkcje
bool waveGaitCycle(PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2, WaveDirection_t direction);
bool waveGaitWalk(PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2, WaveDirection_t direction, int num_cycles);

// Konfiguracja
void setWaveConfig(float step_length, float lift_height, uint32_t step_duration, int step_points);
void printWaveConfig(void);

#endif // WAVE_GAIT_H