/*
 * hexapod_kinematics.h - Kinematyka odwrotna hexapoda
 * Bazuje na działającym kodzie ROS
 */

#ifndef HEXAPOD_KINEMATICS_H
#define HEXAPOD_KINEMATICS_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

// Struktura dla pozycji 3D
typedef struct
{
    float x;
    float y;
    float z;
} Position3D_t;

// Struktura dla kątów stawów
typedef struct
{
    float hip;   // q1 - Kąt biodra [rad]
    float knee;  // q2 - Kąt kolana [rad]
    float ankle; // q3 - Kąt kostki [rad]
} JointAngles_t;

// Struktura origin nogi (z ROS)
typedef struct
{
    float x;          // Pozycja X origin biodra
    float y;          // Pozycja Y origin biodra
    bool invert_hip;  // Czy odwracać kąt biodra
    bool invert_knee; // Czy odwracać kąt kolana
} LegOrigin_t;

// Wymiary nóg hexapoda [cm] - z ROS
#define L1 5.8f  // Hip do knee
#define L2 10.5f // Knee do ankle
#define L3 20.5f // Ankle do końca stopy

// Origins nóg - z ROS, przeliczone na cm
extern const LegOrigin_t leg_origins[6];

// FUNKCJE KINEMATYKI
bool computeLegIK(int leg_number, float x, float y, float z,
                  float *q1, float *q2, float *q3);

bool debugLegIK(int leg_number, float x, float y, float z);

void testAllBasePositions(void);

#endif // HEXAPOD_KINEMATICS_H