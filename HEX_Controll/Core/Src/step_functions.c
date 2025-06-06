/*
 * step_functions.c - Funkcje kroku dla hexapoda
 * Interpolacja kubiczna dla płynnego ruchu nóg
 */

#include "step_functions.h"

/**
 * @brief Interpolacja kubiczna (smooth step) - optymalna dla robotyki
 *
 * Daje płynne przyspieszenie na początku i zwalnianie na końcu
 * Formula: 3*t^2 - 2*t^3 gdzie t ∈ [0,1]
 *
 * @param t: Parametr czasu (0.0 - 1.0)
 * @return: Interpolowana wartość (0.0 - 1.0)
 */
static float cubicInterpolation(float t)
{
    // Ograniczenie do [0,1]
    if (t <= 0.0f)
        return 0.0f;
    if (t >= 1.0f)
        return 1.0f;

    // Smooth step: 3*t^2 - 2*t^3
    return t * t * (3.0f - 2.0f * t);
}

/**
 * @brief Interpolacja liniowa między dwoma wartościami
 */
static float lerp(float start, float end, float t)
{
    return start + (end - start) * t;
}

/**
 * @brief Test pojedynczego kroku do przodu dla jednej nogi
 *
 * Wykonuje krok składający się z:
 * 1. Podniesienie nogi (faza swing)
 * 2. Przesunięcie do przodu
 * 3. Opuszczenie nogi
 * 4. Powrót do pozycji bazowej (opcjonalnie)
 *
 * @param pca: Uchwyt PCA9685
 * @param leg_number: Numer nogi (1-6)
 * @param step_length: Długość kroku [cm]
 * @param lift_height: Wysokość podniesienia [cm]
 * @param step_duration_ms: Czas całego kroku [ms]
 * @param num_points: Liczba punktów interpolacji (zalecane: 20-50)
 * @return: true jeśli sukces
 */
bool testSingleStep(PCA9685_Handle_t *pca, int leg_number,
                    float step_length, float lift_height,
                    uint32_t step_duration_ms, int num_points)
{
    if (leg_number < 1 || leg_number > 6 || num_points < 10)
    {
        printf("Błędne parametry kroku!\n");
        return false;
    }

    printf("\n=== TEST KROKU NOGI %d ===\n", leg_number);
    printf("Długość kroku: %.1f cm\n", step_length);
    printf("Wysokość podniesienia: %.1f cm\n", lift_height);
    printf("Czas kroku: %lu ms\n", step_duration_ms);
    printf("Punkty interpolacji: %d\n", num_points);

    // Pozycja bazowa dla nogi (z ROS)
    float base_x, base_y, base_z = -24.0f; // Sprawdzona wysokość

    // Ustaw pozycje bazowe na podstawie numeru nogi
    switch (leg_number)
    {
    case 1:
        base_x = 18.0f;
        base_y = -15.0f;
        break; // Lewa przednia
    case 2:
        base_x = -18.0f;
        base_y = -15.0f;
        break; // Prawa przednia
    case 3:
        base_x = 22.0f;
        base_y = 0.0f;
        break; // Lewa środkowa
    case 4:
        base_x = -22.0f;
        base_y = 0.0f;
        break; // Prawa środkowa
    case 5:
        base_x = 18.0f;
        base_y = 15.0f;
        break; // Lewa tylna
    case 6:
        base_x = -18.0f;
        base_y = 15.0f;
        break; // Prawa tylna
    default:
        printf("Nieprawidłowy numer nogi!\n");
        return false;
    }

    printf("Pozycja bazowa: (%.1f, %.1f, %.1f)\n", base_x, base_y, base_z);

    // Pozycja docelowa (krok do przodu)
    float target_x = base_x;
    float target_y = base_y - step_length; // Krok do przodu (y+)
    float target_z = base_z;

    printf("Pozycja docelowa: (%.1f, %.1f, %.1f)\n", target_x, target_y, target_z);

    // Sprawdź czy obie pozycje są osiągalne
    float q1, q2, q3;
    if (!computeLegIK(leg_number, base_x, base_y, base_z, &q1, &q2, &q3))
    {
        printf("❌ Pozycja bazowa nieosiągalna!\n");
        return false;
    }

    if (!computeLegIK(leg_number, target_x, target_y, target_z, &q1, &q2, &q3))
    {
        printf("❌ Pozycja docelowa nieosiągalna!\n");
        return false;
    }

    printf("✅ Obie pozycje osiągalne, rozpoczynam krok...\n\n");

    // Czas między punktami
    uint32_t point_delay = step_duration_ms / num_points;

    // FAZA 1: Krok (swing phase) - 60% czasu
    int swing_points = (int)(num_points * 0.6f);
    printf("--- FAZA SWING (%d punktów) ---\n", swing_points);

    for (int i = 0; i <= swing_points; i++)
    {
        float t = (float)i / (float)swing_points; // Parametr czasu [0,1]
        float smooth_t = cubicInterpolation(t);   // Smooth interpolacja

        // Interpolacja pozycji X,Y
        float current_x = lerp(base_x, target_x, smooth_t);
        float current_y = lerp(base_y, target_y, smooth_t);

        // Trajektoria Z - łuk (podniesienie w środku)
        float arc_height = 4.0f * lift_height * t * (1.0f - t); // Parabola
        float current_z = base_z - arc_height;                  // W górę (mniej ujemne Z)

        printf("Punkt %d/%d: t=%.2f, pos=(%.1f, %.1f, %.1f), lift=%.1f\n",
               i, swing_points, t, current_x, current_y, current_z, arc_height);

        // Oblicz IK i ustaw serwa
        if (computeLegIK(leg_number, current_x, current_y, current_z, &q1, &q2, &q3))
        {
            setLegJoints(pca, leg_number, q1, q2, q3);
        }
        else
        {
            printf("❌ IK failed w punkcie %d\n", i);
        }

        HAL_Delay(point_delay);
    }

    // FAZA 2: Powrót (stance phase) - 40% czasu
    int stance_points = num_points - swing_points;
    printf("\n--- FAZA STANCE (%d punktów) ---\n", stance_points);

    for (int i = 0; i <= stance_points; i++)
    {
        float t = (float)i / (float)stance_points;
        float smooth_t = cubicInterpolation(t);

        // Powrót po ziemi z docelowej do bazowej
        float current_x = lerp(target_x, base_x, smooth_t);
        float current_y = lerp(target_y, base_y, smooth_t);
        float current_z = target_z; // Pozostaje na ziemi

        printf("Punkt %d/%d: t=%.2f, pos=(%.1f, %.1f, %.1f)\n",
               i, stance_points, t, current_x, current_y, current_z);

        if (computeLegIK(leg_number, current_x, current_y, current_z, &q1, &q2, &q3))
        {
            setLegJoints(pca, leg_number, q1, q2, q3);
        }
        else
        {
            printf("❌ IK failed w fazie stance %d\n", i);
        }

        HAL_Delay(point_delay);
    }

    printf("\n✅ Krok zakończony! Noga powróciła do pozycji bazowej.\n");
    return true;
}

/**
 * @brief Test kroku z parametrami optymalnymi dla hexapoda
 */
bool testDefaultStep(PCA9685_Handle_t *pca, int leg_number)
{
    // Parametry zoptymalizowane dla początkowych testów
    const float STEP_LENGTH = 6.0f; // 6cm krok (bezpieczny)
    const float LIFT_HEIGHT = 4.0f; // 3cm podniesienie
    const uint32_t DURATION = 50;   // 2 sekundy (powolny test)
    const int POINTS = 200;         // 30 punktów (płynny ruch)

    return testSingleStep(pca, leg_number, STEP_LENGTH, LIFT_HEIGHT, DURATION, POINTS);
}

/**
 * @brief Test różnych długości kroków dla kalibracji
 */
void testStepLengths(PCA9685_Handle_t *pca, int leg_number)
{
    printf("\n=== TEST RÓŻNYCH DŁUGOŚCI KROKÓW ===\n");

    float step_lengths[] = {3.0f, 6.0f, 9.0f, 12.0f}; // cm
    int num_tests = sizeof(step_lengths) / sizeof(step_lengths[0]);

    for (int i = 0; i < num_tests; i++)
    {
        printf("\n>>> TEST %d/%d: Krok %.1f cm <<<\n", i + 1, num_tests, step_lengths[i]);

        bool success = testSingleStep(pca, leg_number, step_lengths[i], 3.0f, 2000, 25);

        if (success)
        {
            printf("✅ Krok %.1f cm: SUKCES\n", step_lengths[i]);
        }
        else
        {
            printf("❌ Krok %.1f cm: BŁĄD\n", step_lengths[i]);
        }

        printf("Czekam 3 sekundy przed następnym testem...\n");
        HAL_Delay(3000);
    }

    printf("\n=== KONIEC TESTÓW KROKÓW ===\n");
}