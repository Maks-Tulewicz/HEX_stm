/*
 * test_positions.h - Testy i kalibracja hexapoda
 */

#ifndef TEST_POSITIONS_H
#define TEST_POSITIONS_H

#include "hexapod_kinematics.h"
#include "pca9685.h"

// FUNKCJE KALIBRACJI
void calibrationTest90Degrees(PCA9685_Handle_t *pca, int leg_number);
void testStraightLegPosition(PCA9685_Handle_t *pca, int leg_number);

// FUNKCJE POMOCNICZE
void setLegJoints(PCA9685_Handle_t *pca, int leg_number, float q1, float q2, float q3);
bool legIKToServos(int leg_number, float x, float y, float z, PCA9685_Handle_t *pca);

#endif // TEST_POSITIONS_H