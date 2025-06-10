/*
 * hexapod_kinematics.c - Kinematyka odwrotna hexapoda
 * Bazuje na działającym kodzie ROS - przepisane na STM32
 */

#include "hexapod_kinematics.h"

const LegOrigin_t leg_origins[6] = {
    {6.8956f, -7.7136f, false, false}, // Noga 1 - lewa przednia
    {-8.6608f, -7.7136f, true, true},  // Noga 2 - prawa przednia
    {10.1174f, 0.0645f, false, false}, // Noga 3 - lewa środkowa
    {-11.8826f, -0.0645f, true, true}, // Noga 4 - prawa środkowa
    {6.8956f, 7.8427f, false, false},  // Noga 5 - lewa tylna
    {-8.6608f, 7.8427f, true, true}    // Noga 6 - prawa tylna
};

// Kinematyka odwrotna - SKOPIOWANA Z ROS
bool computeLegIK(int leg_number, float x, float y, float z,
                  float *q1, float *q2, float *q3)
{
    if (leg_number < 1 || leg_number > 6 || q1 == NULL || q2 == NULL || q3 == NULL)
    {
        return false;
    }

    // Pobierz konfigurację dla danej nogi
    const LegOrigin_t *leg = &leg_origins[leg_number - 1];

    printf("Leg %d IK input - x: %.2f, y: %.2f, z: %.2f\n", leg_number, x, y, z);

    // 1. Przekształcenie do lokalnego układu współrzędnych nogi
    float local_x = x - leg->x;
    float local_y = y - leg->y;

    printf("Leg %d - local coords: x=%.3f, y=%.3f\n", leg_number, local_x, local_y);

    // 2. Obliczenie kąta biodra (obrót wokół osi Z)
    *q1 = atan2f(local_y, local_x);

    // Inwersja kąta biodra dla prawych nóg
    if (leg->invert_hip)
    {
        if (*q1 > 0)
            *q1 = *q1 - M_PI;
        else
            *q1 = *q1 + M_PI;
    }

    printf("Leg %d - hip angle before constraints: %.2f deg\n", leg_number, *q1 * 180.0f / M_PI);

    // 3. Obliczenie odległości radialnej od osi biodra
    float r = sqrtf(local_x * local_x + local_y * local_y) - L1;
    float h = -z; // Zmiana znaku, bo oś Z jest skierowana w dół

    printf("Leg %d - r=%.2f, h=%.2f\n", leg_number, r, h);

    // 4. Sprawdzenie czy punkt jest w zasięgu nogi
    float D2 = r * r + h * h;
    float D = sqrtf(D2);

    printf("Leg %d - distance D=%.2f, max_reach=%.2f, min_reach=%.2f\n",
           leg_number, D, L2 + L3, fabsf(L2 - L3));

    if (D > (L2 + L3) || D < fabsf(L2 - L3))
    {
        printf("Leg %d IK failed - Distance %.2f out of range [%.2f, %.2f]\n",
               leg_number, D, fabsf(L2 - L3), L2 + L3);
        printf("  Target: x=%.2f, y=%.2f, z=%.2f\n", x, y, z);
        printf("  Local: x=%.2f, y=%.2f\n", local_x, local_y);
        printf("  r=%.2f, h=%.2f\n", r, h);
        return false;
    }

    // 5. Obliczenie gamma (kąt między L2 i L3)
    float cos_gamma = (D2 - L2 * L2 - L3 * L3) / (2.0f * L2 * L3);
    cos_gamma = fmaxf(-1.0f, fminf(1.0f, cos_gamma));
    float gamma = acosf(cos_gamma);

    // 6. Obliczenie kąta kolana (q2)
    float alpha = atan2f(h, r);
    float beta = acosf((D2 + L2 * L2 - L3 * L3) / (2.0f * L2 * D));
    *q2 = -(alpha - beta);

    // 7. Obliczenie kąta kostki (q3)
    if (leg->invert_knee)
    {
        // Dla prawej strony (2,4,6)
        *q3 = gamma - M_PI;
    }
    else
    {
        // Dla lewej strony (1,3,5)
        *q3 = -(M_PI - gamma);
    }

    printf("Leg %d final angles [deg]: hip=%.1f, knee=%.1f, ankle=%.1f\n",
           leg_number, *q1 * 180.0f / M_PI, *q2 * 180.0f / M_PI, *q3 * 180.0f / M_PI);

    return true;
}

// Debug funkcja IK - SKOPIOWANA Z ROS
bool debugLegIK(int leg_number, float x, float y, float z)
{
    const LegOrigin_t *leg = &leg_origins[leg_number - 1];

    printf("=== DEBUG IK dla nogi %d ===\n", leg_number);
    printf("Cel: x=%.2f, y=%.2f, z=%.2f\n", x, y, z);
    printf("Origin nogi: x=%.3f, y=%.3f\n", leg->x, leg->y);
    printf("Flags: invert_hip=%s, invert_knee=%s\n",
           leg->invert_hip ? "true" : "false",
           leg->invert_knee ? "true" : "false");

    // Lokalne współrzędne
    float local_x = x - leg->x;
    float local_y = y - leg->y;
    printf("Lokalne: x=%.2f, y=%.2f\n", local_x, local_y);

    // Odległość radialna
    float r = sqrtf(local_x * local_x + local_y * local_y) - L1;
    float h = -z;
    float D = sqrtf(r * r + h * h);

    printf("r=%.2f, h=%.2f, D=%.2f\n", r, h, D);
    printf("Zasięg: min=%.2f, max=%.2f\n", fabsf(L2 - L3), L2 + L3);
    printf("Długości segmentów: L1=%.1f, L2=%.1f, L3=%.1f\n", L1, L2, L3);

    if (D > (L2 + L3))
    {
        printf("Cel za daleko! D=%.2f > max=%.2f (różnica: %.2f)\n",
               D, L2 + L3, D - (L2 + L3));
        return false;
    }

    if (D < fabsf(L2 - L3))
    {
        printf("Cel za blisko! D=%.2f < min=%.2f\n", D, fabsf(L2 - L3));
        return false;
    }

    printf("IK wykonalne - cel w zasięgu\n");

    // Przetestuj rzeczywiste IK
    float q1, q2, q3;
    bool ik_result = computeLegIK(leg_number, x, y, z, &q1, &q2, &q3);
    printf("Rezultat IK: %s\n", ik_result ? "SUCCESS" : "FAILED");

    if (ik_result)
    {
        printf("Kąty [deg]: hip=%.1f, knee=%.1f, ankle=%.1f\n",
               q1 * 180.0f / M_PI, q2 * 180.0f / M_PI, q3 * 180.0f / M_PI);
    }

    return ik_result;
}

// Test wszystkich pozycji bazowych - SKOPIOWANY Z ROS
void testAllBasePositions(void)
{
    printf("=== TESTOWANIE WSZYSTKICH POZYCJI BAZOWYCH ===\n");

    // Pozycje bazowe z ROS (w cm)
    const float base_positions[6][3] = {
        {18.0f, -15.0f, -24.0f},  // Noga 1
        {-18.0f, -15.0f, -24.0f}, // Noga 2
        {22.0f, 0.0f, -24.0f},    // Noga 3
        {-22.0f, 0.0f, -24.0f},   // Noga 4
        {18.0f, 15.0f, -24.0f},   // Noga 5
        {-18.0f, 15.0f, -24.0f}   // Noga 6
    };

    bool all_ok = true;

    for (int leg = 1; leg <= 6; leg++)
    {
        printf("\n--- NOGA %d ---\n", leg);
        float x = base_positions[leg - 1][0];
        float y = base_positions[leg - 1][1];
        float z = base_positions[leg - 1][2];

        bool result = debugLegIK(leg, x, y, z);
        if (!result)
            all_ok = false;

        // Test z krokiem do przodu
        float step_forward = 4.0f;
        printf("Test z krokiem do przodu (+%.1f):\n", step_forward);
        bool with_step = debugLegIK(leg, x, y + step_forward, z);
        if (!with_step)
            all_ok = false;

        // Test z krokiem do tyłu
        printf("Test z krokiem do tyłu (-%.1f):\n", step_forward);
        bool with_back_step = debugLegIK(leg, x, y - step_forward, z);
        if (!with_back_step)
            all_ok = false;
    }

    printf("\n=== PODSUMOWANIE TESTÓW ===\n");
    printf("Wszystkie testy: %s\n", all_ok ? "PASSED" : "FAILED");

    if (!all_ok)
    {
        printf("Niektóre pozycje są poza zasięgiem!\n");
        printf("Rozważ zmniejszenie step_length lub pozycji bazowych\n");
    }
}