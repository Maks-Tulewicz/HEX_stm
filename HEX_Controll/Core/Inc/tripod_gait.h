/*
 * tripod_gait.h - Tripod gait dla hexapoda
 *
 * Tripod gait = najbardziej stabilny chód hexapoda
 * - Zawsze 3 nogi na ziemi (stabilny trójkąt)
 * - Dwie grupy nóg pracujące na przemian
 */

#ifndef TRIPOD_GAIT_H
#define TRIPOD_GAIT_H

#include "hexapod_kinematics.h"
#include "test_positions.h"
#include "pca9685.h"

/**
 * @brief Kierunki ruchu w tripod gait
 */
typedef enum
{
    TRIPOD_FORWARD,   // Do przodu
    TRIPOD_BACKWARD,  // Do tyłu
    TRIPOD_LEFT,      // W lewo (sideways)
    TRIPOD_RIGHT,     // W prawo (sideways)
    TRIPOD_TURN_LEFT, // Obrót w lewo (w miejscu)
    TRIPOD_TURN_RIGHT // Obrót w prawo (w miejscu)
} TripodDirection_t;

/**
 * @brief Konfiguracja tripod gait
 */
typedef struct
{
    float step_length;           // Długość kroku [cm]
    float lift_height;           // Wysokość podniesienia nogi [cm]
    uint32_t swing_duration_ms;  // Czas swing phase [ms]
    uint32_t stance_duration_ms; // Czas stance phase [ms]
    int swing_points;            // Punkty interpolacji swing phase
    int stance_points;           // Punkty interpolacji stance phase
    float step_height_base;      // Bazowa wysokość stania [cm]
} TripodConfig_t;

// Globalna konfiguracja tripod gait
extern TripodConfig_t tripod_config;

/**
 * @brief Wykonaj jeden pełny cykl tripod gait
 *
 * Jeden cykl = 2 fazy:
 * 1. Grupa A (1,4,5) swing + Grupa B (2,3,6) stance
 * 2. Grupa B (2,3,6) swing + Grupa A (1,4,5) stance
 *
 * @param pca1: PCA9685 dla nóg lewych (1,3,5) lub NULL jeśli niedostępne
 * @param pca2: PCA9685 dla nóg prawych (2,4,6) lub NULL jeśli niedostępne
 * @param direction: Kierunek ruchu
 * @return: true jeśli sukces
 */
bool tripodGaitCycle(PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2, TripodDirection_t direction);

/**
 * @brief Wykonaj ciągłe chodzenie (wiele cykli tripod gait)
 *
 * @param pca1: PCA9685 dla nóg lewych
 * @param pca2: PCA9685 dla nóg prawych
 * @param direction: Kierunek ruchu
 * @param num_cycles: Liczba cykli do wykonania
 * @return: true jeśli sukces
 */
bool tripodGaitWalk(PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2,
                    TripodDirection_t direction, int num_cycles);

/**
 * @brief Ustaw parametry tripod gait w runtime
 *
 * @param step_length: Długość kroku [cm]
 * @param lift_height: Wysokość podniesienia [cm]
 * @param swing_duration: Czas swing phase [ms]
 * @param stance_duration: Czas stance phase [ms]
 * @param swing_points: Punkty interpolacji swing
 * @param stance_points: Punkty interpolacji stance
 */
void setTripodConfig(float step_length, float lift_height,
                     uint32_t swing_duration, uint32_t stance_duration,
                     int swing_points, int stance_points);

/**
 * @brief Wyświetl aktualną konfigurację tripod gait
 */
void printTripodConfig(void);

// MAKRA dla szybkiego użycia
#define TRIPOD_WALK_FORWARD(pca1, pca2, steps) tripodGaitWalk(pca1, pca2, TRIPOD_FORWARD, steps)
#define TRIPOD_WALK_BACKWARD(pca1, pca2, steps) tripodGaitWalk(pca1, pca2, TRIPOD_BACKWARD, steps)
#define TRIPOD_TURN_LEFT(pca1, pca2, steps) tripodGaitWalk(pca1, pca2, TRIPOD_TURN_LEFT, steps)
#define TRIPOD_TURN_RIGHT(pca1, pca2, steps) tripodGaitWalk(pca1, pca2, TRIPOD_TURN_RIGHT, steps)

// Predefiniowane konfiguracje
#define TRIPOD_CONFIG_SLOW {4.0f, 3.0f, 10, 10, 80, 40}  // Wolny, bezpieczny
#define TRIPOD_CONFIG_NORMAL {6.0f, 4.0f, 5, 5, 120, 60} // Normalny - TWOJE USTAWIENIA
#define TRIPOD_CONFIG_FAST {8.0f, 3.0f, 3, 3, 100, 50}   // Szybki

/**
 * @brief Grupy nóg w tripod gait
 *
 * GRUPA A (swing w fazie 1):
 * - Noga 1: Lewa przednia
 * - Noga 4: Prawa środkowa
 * - Noga 5: Lewa tylna
 *
 * GRUPA B (swing w fazie 2):
 * - Noga 2: Prawa przednia
 * - Noga 3: Lewa środkowa
 * - Noga 6: Prawa tylna
 *
 * Każda grupa tworzy stabilny trójkąt podparcia
 */

#endif // TRIPOD_GAIT_H