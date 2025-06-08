/*
 * step_functions.h - Funkcje kroku dla hexapoda
 * Interpolacja kubiczna dla płynnego ruchu nóg
 */

#ifndef STEP_FUNCTIONS_H
#define STEP_FUNCTIONS_H

#include "hexapod_kinematics.h"
#include "test_positions.h"
#include "pca9685.h"

/**
 * @brief Test pojedynczego kroku do przodu dla jednej nogi
 *
 * Wykonuje pełny krok z interpolacją kubiczną:
 * - Faza swing: podniesienie i przesunięcie do przodu (60% czasu)
 * - Faza stance: powrót po ziemi do pozycji bazowej (40% czasu)
 *
 * @param pca: Uchwyt PCA9685
 * @param leg_number: Numer nogi (1-6)
 * @param step_length: Długość kroku w kierunku Y [cm]
 * @param lift_height: Wysokość podniesienia nogi [cm]
 * @param step_duration_ms: Całkowity czas kroku [ms]
 * @param num_points: Liczba punktów interpolacji (zalecane: 20-50)
 * @return: true jeśli sukces
 */
bool testSingleStep(PCA9685_Handle_t *pca, int leg_number,
                    float step_length, float lift_height,
                    uint32_t step_duration_ms, int num_points);

/**
 * @brief Test kroku z domyślnymi parametrami (bezpieczne dla testów)
 *
 * Parametry:
 * - Długość kroku: 6cm
 * - Wysokość podniesienia: 3cm
 * - Czas kroku: 2 sekundy
 * - Punkty interpolacji: 30
 *
 * @param pca: Uchwyt PCA9685
 * @param leg_number: Numer nogi (1-6)
 * @return: true jeśli sukces
 */
bool testDefaultStep(PCA9685_Handle_t *pca, int leg_number);

/**
 * @brief Test różnych długości kroków dla kalibracji
 *
 * Testuje kroki: 3cm, 6cm, 9cm, 12cm
 * Pozwala ocenić maksymalną bezpieczną długość kroku
 *
 * @param pca: Uchwyt PCA9685
 * @param leg_number: Numer nogi (1-6)
 */
void testStepLengths(PCA9685_Handle_t *pca, int leg_number);

#endif // STEP_FUNCTIONS_H