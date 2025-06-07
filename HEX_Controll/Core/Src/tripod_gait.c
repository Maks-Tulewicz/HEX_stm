/*
 * tripod_gait.c - Tripod gait dla hexapoda
 *
 * Tripod = najbardziej stabilny chód hexapoda
 * - Zawsze 3 nogi na ziemi (trójkąt stabilności)
 * - Grupa A: nogi 1,4,5 (lewa przednia, prawa środkowa, lewa tylna)
 * - Grupa B: nogi 2,3,6 (prawa przednia, lewa środkowa, prawa tylna)
 *
 * Fazy:
 * 1. Grupa A robi SWING (krok do przodu) + Grupa B robi STANCE (przesuw robota)
 * 2. Grupa B robi SWING (krok do przodu) + Grupa A robi STANCE (przesuw robota)
 */

#include "tripod_gait.h"

// Konfiguracja tripod gait
TripodConfig_t tripod_config = {
    .step_length = 6.0f,       // Długość kroku [cm]
    .lift_height = 4.0f,       // Wysokość podniesienia [cm]
    .swing_duration_ms = 5,    // Czas swing phase [ms] - TWOJE SZYBKIE USTAWIENIE
    .stance_duration_ms = 5,   // Czas stance phase [ms]
    .swing_points = 120,       // Punkty interpolacji swing - TWOJE USTAWIENIE
    .stance_points = 60,       // Punkty interpolacji stance (może być mniej)
    .step_height_base = -24.0f // Bazowa wysokość stania [cm]
};

/**
 * @brief Pozycje bazowe nóg - skopiowane z hexapod_kinematics.c
 */
static const float base_positions[6][3] = {
    {18.0f, -15.0f, -24.0f},  // Noga 1 - lewa przednia
    {-18.0f, -15.0f, -24.0f}, // Noga 2 - prawa przednia
    {22.0f, 0.0f, -24.0f},    // Noga 3 - lewa środkowa
    {-22.0f, 0.0f, -24.0f},   // Noga 4 - prawa środkowa
    {18.0f, 15.0f, -24.0f},   // Noga 5 - lewa tylna
    {-18.0f, 15.0f, -24.0f}   // Noga 6 - prawa tylna
};

/**
 * @brief Interpolacja kubiczna - smooth step
 */
static float cubicInterpolation(float t)
{
    if (t <= 0.0f)
        return 0.0f;
    if (t >= 1.0f)
        return 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

/**
 * @brief Interpolacja liniowa
 */
static float lerp(float start, float end, float t)
{
    return start + (end - start) * t;
}

/**
 * @brief Oblicz docelową pozycję dla kroku w danym kierunku
 */
static void calculateTargetPosition(int leg_number, TripodDirection_t direction,
                                    float *target_x, float *target_y, float *target_z)
{
    // Pozycja bazowa
    float base_x = base_positions[leg_number - 1][0];
    float base_y = base_positions[leg_number - 1][1];
    float base_z = base_positions[leg_number - 1][2];

    *target_x = base_x;
    *target_y = base_y;
    *target_z = base_z;

    // Modyfikacja na podstawie kierunku
    switch (direction)
    {
    case TRIPOD_FORWARD:
        *target_y = base_y - tripod_config.step_length; // Do przodu (Y-)
        break;
    case TRIPOD_BACKWARD:
        *target_y = base_y + tripod_config.step_length; // Do tyłu (Y+)
        break;
    case TRIPOD_LEFT:
        *target_x = base_x + tripod_config.step_length; // W lewo (X+)
        break;
    case TRIPOD_RIGHT:
        *target_x = base_x - tripod_config.step_length; // W prawo (X-)
        break;
    case TRIPOD_TURN_LEFT:
        // Obrót w lewo - przednie nogi w lewo, tylne w prawo
        if (leg_number == 1 || leg_number == 2)
        {
            *target_x = base_x + tripod_config.step_length; // Przednie w lewo
        }
        else if (leg_number == 5 || leg_number == 6)
        {
            *target_x = base_x - tripod_config.step_length; // Tylne w prawo
        }
        break;
    case TRIPOD_TURN_RIGHT:
        // Obrót w prawo - przednie nogi w prawo, tylne w lewo
        if (leg_number == 1 || leg_number == 2)
        {
            *target_x = base_x - tripod_config.step_length; // Przednie w prawo
        }
        else if (leg_number == 5 || leg_number == 6)
        {
            *target_x = base_x + tripod_config.step_length; // Tylne w lewo
        }
        break;
    default:
        break;
    }
}

/**
 * @brief Wykonaj SWING phase dla jednej nogi (podniesienie + krok do przodu)
 */
static bool executeSwingPhase(PCA9685_Handle_t *pca, int leg_number, TripodDirection_t direction)
{
    // Pozycja bazowa i docelowa
    float base_x = base_positions[leg_number - 1][0];
    float base_y = base_positions[leg_number - 1][1];
    float base_z = base_positions[leg_number - 1][2];

    float target_x, target_y, target_z;
    calculateTargetPosition(leg_number, direction, &target_x, &target_y, &target_z);

    // Sprawdź osiągalność docelowej pozycji
    float q1, q2, q3;
    if (!computeLegIK(leg_number, target_x, target_y, target_z, &q1, &q2, &q3))
    {
        printf("❌ Swing IK failed dla nogi %d\n", leg_number);
        return false;
    }

    // Timing
    uint32_t point_delay = tripod_config.swing_duration_ms * 1000 / tripod_config.swing_points; // w mikrosekundach
    uint32_t next_time = HAL_GetTick() * 1000;                                                  // Konwersja na mikrosekundy

    // Interpolacja swing phase
    for (int i = 0; i <= tripod_config.swing_points; i++)
    {
        float t = (float)i / (float)tripod_config.swing_points;
        float smooth_t = cubicInterpolation(t);

        // Pozycja X,Y - interpolacja do docelowej pozycji
        float current_x = lerp(base_x, target_x, smooth_t);
        float current_y = lerp(base_y, target_y, smooth_t);

        // Pozycja Z - trajektoria łuku (podniesienie)
        float arc_height = 4.0f * tripod_config.lift_height * t * (1.0f - t); // Parabola
        float current_z = base_z - arc_height;                                // W górę (mniej ujemne Z)

        // Oblicz IK i ustaw serwa
        if (computeLegIK(leg_number, current_x, current_y, current_z, &q1, &q2, &q3))
        {
            setLegJoints(pca, leg_number, q1, q2, q3);
        }
        else
        {
            printf("❌ Swing IK failed w punkcie %d dla nogi %d\n", i, leg_number);
        }

        // Precyzyjne timing
        next_time += point_delay;
        while ((HAL_GetTick() * 1000) < next_time)
        {
            // Busy wait dla precyzji
        }
    }

    return true;
}

/**
 * @brief Wykonaj STANCE phase dla jednej nogi (przesuwanie robota po ziemi)
 */
static bool executeStancePhase(PCA9685_Handle_t *pca, int leg_number, TripodDirection_t direction)
{
    // W stance phase noga przesuwa się przeciwnie do kierunku ruchu robota
    // (robot idzie do przodu = nogi w stance idą do tyłu względem korpusu)

    float base_x = base_positions[leg_number - 1][0];
    float base_y = base_positions[leg_number - 1][1];
    float base_z = base_positions[leg_number - 1][2];

    // Oblicz pozycję startową stance (przeciwna do kierunku ruchu)
    float start_x = base_x;
    float start_y = base_y;

    switch (direction)
    {
    case TRIPOD_FORWARD:
        start_y = base_y + tripod_config.step_length; // Start z tyłu (Y+)
        break;
    case TRIPOD_BACKWARD:
        start_y = base_y - tripod_config.step_length; // Start z przodu (Y-)
        break;
    case TRIPOD_LEFT:
        start_x = base_x - tripod_config.step_length; // Start z prawej (X-)
        break;
    case TRIPOD_RIGHT:
        start_x = base_x + tripod_config.step_length; // Start z lewej (X+)
        break;
    case TRIPOD_TURN_LEFT:
        if (leg_number == 1 || leg_number == 2)
        {
            start_x = base_x - tripod_config.step_length; // Przednie z prawej
        }
        else if (leg_number == 5 || leg_number == 6)
        {
            start_x = base_x + tripod_config.step_length; // Tylne z lewej
        }
        break;
    case TRIPOD_TURN_RIGHT:
        if (leg_number == 1 || leg_number == 2)
        {
            start_x = base_x + tripod_config.step_length; // Przednie z lewej
        }
        else if (leg_number == 5 || leg_number == 6)
        {
            start_x = base_x - tripod_config.step_length; // Tylne z prawej
        }
        break;
    default:
        start_x = base_x;
        start_y = base_y;
        break;
    }

    // Timing
    uint32_t point_delay = tripod_config.stance_duration_ms * 1000 / tripod_config.stance_points;
    uint32_t next_time = HAL_GetTick() * 1000;

    // Interpolacja stance phase - przesuwanie po ziemi
    for (int i = 0; i <= tripod_config.stance_points; i++)
    {
        float t = (float)i / (float)tripod_config.stance_points;
        float smooth_t = cubicInterpolation(t);

        // Pozycja - od start do base (przesuwanie robota)
        float current_x = lerp(start_x, base_x, smooth_t);
        float current_y = lerp(start_y, base_y, smooth_t);
        float current_z = base_z; // Zawsze na ziemi

        // Oblicz IK i ustaw serwa
        float q1, q2, q3;
        if (computeLegIK(leg_number, current_x, current_y, current_z, &q1, &q2, &q3))
        {
            setLegJoints(pca, leg_number, q1, q2, q3);
        }
        else
        {
            printf("❌ Stance IK failed w punkcie %d dla nogi %d\n", i, leg_number);
        }

        // Precyzyjne timing
        next_time += point_delay;
        while ((HAL_GetTick() * 1000) < next_time)
        {
            // Busy wait
        }
    }

    return true;
}

/**
 * @brief Wykonaj jeden cykl tripod gait
 */
bool tripodGaitCycle(PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2, TripodDirection_t direction)
{
    printf("\n=== TRIPOD GAIT CYCLE START ===\n");
    printf("Kierunek: %s\n", (direction == TRIPOD_FORWARD) ? "PRZÓD" : (direction == TRIPOD_BACKWARD) ? "TYŁ"
                                                                   : (direction == TRIPOD_LEFT)       ? "LEWO"
                                                                   : (direction == TRIPOD_RIGHT)      ? "PRAWO"
                                                                   : (direction == TRIPOD_TURN_LEFT)  ? "OBRÓT LEWO"
                                                                                                      : "OBRÓT PRAWO");

    // FAZA 1: Grupa A (1,4,5) SWING + Grupa B (2,3,6) STANCE
    printf("\n--- FAZA 1: Grupa A swing, Grupa B stance ---\n");

    // UWAGA: Implementacja tylko dla dostępnych nóg
    // Noga 3 (lewa środkowa) jest w grupie B, więc robi STANCE
    printf("Noga 3 (Grupa B) - STANCE phase\n");
    if (pca1 != NULL)
    {
        executeStancePhase(pca1, 3, direction);
    }

    // FAZA 2: Grupa B (2,3,6) SWING + Grupa A (1,4,5) STANCE
    printf("\n--- FAZA 2: Grupa B swing, Grupa A stance ---\n");

    // Noga 3 robi SWING
    printf("Noga 3 (Grupa B) - SWING phase\n");
    if (pca1 != NULL)
    {
        executeSwingPhase(pca1, 3, direction);
    }

    printf("\n✅ TRIPOD GAIT CYCLE ZAKOŃCZONY\n");
    return true;
}

/**
 * @brief Wykonaj wiele cykli tripod gait (ciągłe chodzenie)
 */
bool tripodGaitWalk(PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2,
                    TripodDirection_t direction, int num_cycles)
{
    printf("\n=== TRIPOD GAIT WALK START ===\n");
    printf("Liczba cykli: %d\n", num_cycles);
    printf("Konfiguracja:\n");
    printf("  Długość kroku: %.1f cm\n", tripod_config.step_length);
    printf("  Wysokość podniesienia: %.1f cm\n", tripod_config.lift_height);
    printf("  Czas swing: %lu ms\n", tripod_config.swing_duration_ms);
    printf("  Czas stance: %lu ms\n", tripod_config.stance_duration_ms);

    for (int cycle = 0; cycle < num_cycles; cycle++)
    {
        printf("\n>>> CYKL %d/%d <<<\n", cycle + 1, num_cycles);

        bool success = tripodGaitCycle(pca1, pca2, direction);
        if (!success)
        {
            printf("❌ Błąd w cyklu %d, przerywam chodzenie\n", cycle + 1);
            return false;
        }

        // Krótka pauza między cyklami (opcjonalnie)
        HAL_Delay(50);
    }

    printf("\n✅ TRIPOD GAIT WALK ZAKOŃCZONY\n");
    return true;
}

/**
 * @brief Ustaw konfigurację tripod gait
 */
void setTripodConfig(float step_length, float lift_height,
                     uint32_t swing_duration, uint32_t stance_duration,
                     int swing_points, int stance_points)
{
    tripod_config.step_length = step_length;
    tripod_config.lift_height = lift_height;
    tripod_config.swing_duration_ms = swing_duration;
    tripod_config.stance_duration_ms = stance_duration;
    tripod_config.swing_points = swing_points;
    tripod_config.stance_points = stance_points;

    printf("✅ Konfiguracja tripod zaktualizowana:\n");
    printf("  Długość kroku: %.1f cm\n", step_length);
    printf("  Wysokość podniesienia: %.1f cm\n", lift_height);
    printf("  Czas swing/stance: %lu/%lu ms\n", swing_duration, stance_duration);
    printf("  Punkty swing/stance: %d/%d\n", swing_points, stance_points);
}

/**
 * @brief Wyświetl aktualną konfigurację
 */
void printTripodConfig(void)
{
    printf("\n=== KONFIGURACJA TRIPOD GAIT ===\n");
    printf("Długość kroku: %.1f cm\n", tripod_config.step_length);
    printf("Wysokość podniesienia: %.1f cm\n", tripod_config.lift_height);
    printf("Czas swing: %lu ms\n", tripod_config.swing_duration_ms);
    printf("Czas stance: %lu ms\n", tripod_config.stance_duration_ms);
    printf("Punkty swing: %d\n", tripod_config.swing_points);
    printf("Punkty stance: %d\n", tripod_config.stance_points);
    printf("Wysokość bazowa: %.1f cm\n", tripod_config.step_height_base);
    printf("===============================\n");
}