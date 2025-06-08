/*
 * wave_gait.c - Wave gait dla hexapoda
 *
 * Koncepcja: 6 nóg chodzących sekwencyjnie z fazą stance shift
 * Sekwencja: 1 → 2 → 3 → 4 → 5 → 6
 *
 * SEKWENCJA:
 * 1. Noga X robi SWING (obecna pozycja → pozycja przednia), pozostałe STOJĄ
 * 2. FAZA STANCE: WSZYSTKIE nogi przesuwają się o 1/6 step_length do tyłu
 * 3. Powtórz dla kolejnej nogi
 */

#include "wave_gait.h"
#include <stdio.h>
#include <math.h>

// Konfiguracja wave gait
WaveConfig_t wave_config = {
    .step_length = 4.0f,       // Długość kroku [cm]
    .lift_height = 4.0f,       // Wysokość podniesienia [cm]
    .step_duration_ms = 10,    // Czas swing jednej nogi [ms] - SZYBKO
    .step_points = 50,         // Punkty interpolacji swing
    .step_height_base = -24.0f // Bazowa wysokość stania [cm]
};

// Pozycje bazowe nóg - z ROS
static const float base_positions[6][3] = {
    {18.0f, -15.0f, -24.0f},  // Noga 1 - lewa przednia
    {-18.0f, -15.0f, -24.0f}, // Noga 2 - prawa przednia
    {22.0f, 0.0f, -24.0f},    // Noga 3 - lewa środkowa
    {-22.0f, 0.0f, -24.0f},   // Noga 4 - prawa środkowa
    {18.0f, 15.0f, -24.0f},   // Noga 5 - lewa tylna
    {-18.0f, 15.0f, -24.0f}   // Noga 6 - prawa tylna
};

// Śledzenie aktualnych pozycji Y każdej nogi
static float leg_current_y[6];
static bool positions_initialized = false;

// Sekwencja wave - nogi chodzą jedna po drugiej
static const int wave_sequence[6] = {1, 2, 3, 4, 5, 6};

// Mapowanie kanałów PCA9685 z offsetami biodra
typedef struct
{
    uint8_t base_channel;
    float hip_offset_deg;
    bool is_left_side;
} LegMapping_t;

static const LegMapping_t leg_mapping[6] = {
    {0, 37.5f, true},   // Noga 1: I2C1, kanały 0-2, offset +37.5°
    {0, -37.5f, false}, // Noga 2: I2C2, kanały 0-2, offset -37.5°
    {3, 0.0f, true},    // Noga 3: I2C1, kanały 3-5, bez offsetu
    {3, 0.0f, false},   // Noga 4: I2C2, kanały 3-5, bez offsetu
    {6, -37.5f, true},  // Noga 5: I2C1, kanały 6-8, offset -37.5°
    {6, 37.5f, false}   // Noga 6: I2C2, kanały 6-8, offset +37.5°
};

/**
 * @brief Interpolacja kubiczna
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
 * @brief Inicjalizacja pozycji nóg
 */
static void initializeLegPositions(void)
{
    for (int i = 0; i < 6; i++)
    {
        leg_current_y[i] = base_positions[i][1]; // Pozycje bazowe
    }
    positions_initialized = true;
    printf("🔧 Wave: Pozycje nóg zainicjalizowane\n");
}

/**
 * @brief Ustaw kanały serw dla nogi z offsetem biodra
 */
static void setLegJointsWithOffset(int leg_number, float q1, float q2, float q3,
                                   PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2)
{
    if (leg_number < 1 || leg_number > 6)
        return;

    const LegMapping_t *mapping = &leg_mapping[leg_number - 1];
    PCA9685_Handle_t *pca_to_use = mapping->is_left_side ? pca1 : pca2;

    if (pca_to_use == NULL)
    {
        printf("⚠️  PCA dla nogi %d niedostępne\n", leg_number);
        return;
    }

    // Konwersja z offsetem
    float hip_deg = (q1 * 180.0f / M_PI) + mapping->hip_offset_deg;
    float knee_deg = q2 * 180.0f / M_PI;
    float ankle_deg = q3 * 180.0f / M_PI;

    // INWERSJA KOLAN dla prawych nóg (2,4,6) - odwrócony montaż silników
    if (!mapping->is_left_side)
    {
        knee_deg = -knee_deg;   // Prawe nogi: odwróć kolano
        ankle_deg = -ankle_deg; // Prawe nogi: odwróć kostkę
    }

    // Mapowanie na serwa
    float servo_hip = 90.0f + hip_deg;
    float servo_knee = 90.0f + knee_deg;
    float servo_ankle = 90.0f + ankle_deg;

    // Ograniczenia
    if (servo_hip < 0.0f)
        servo_hip = 0.0f;
    if (servo_hip > 180.0f)
        servo_hip = 180.0f;
    if (servo_knee < 0.0f)
        servo_knee = 0.0f;
    if (servo_knee > 180.0f)
        servo_knee = 180.0f;
    if (servo_ankle < 0.0f)
        servo_ankle = 0.0f;
    if (servo_ankle > 180.0f)
        servo_ankle = 180.0f;

    // Ustaw serwa
    PCA9685_SetServoAngle(pca_to_use, mapping->base_channel + 0, servo_hip);
    PCA9685_SetServoAngle(pca_to_use, mapping->base_channel + 1, servo_knee);
    PCA9685_SetServoAngle(pca_to_use, mapping->base_channel + 2, servo_ankle);
}

/**
 * @brief FAZA 1: Wykonaj SWING dla jednej nogi
 */
static bool executeSwingPhase(int leg_number, WaveDirection_t direction,
                              PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2)
{
    printf("\n--- FAZA SWING: Noga %d ---\n", leg_number);
    printf("SWING: noga %d | POZOSTAŁE: stoją bez ruchu\n", leg_number);

    uint32_t step_delay = wave_config.step_duration_ms / wave_config.step_points;
    if (step_delay == 0)
        step_delay = 1;

    int leg_index = leg_number - 1;

    // === FAZA SWING ===
    for (int i = 0; i <= wave_config.step_points; i++)
    {
        float t = (float)i / (float)wave_config.step_points;
        float smooth_t = cubicInterpolation(t);

        // SWING dla jednej nogi
        float base_x = base_positions[leg_index][0];
        float base_y = base_positions[leg_index][1];
        float base_z = base_positions[leg_index][2];

        // Swing: z obecnej pozycji do pozycji przedniej
        float swing_start_y = leg_current_y[leg_index];       // Obecna pozycja
        float swing_end_y = base_y - wave_config.step_length; // Pozycja przednia

        float current_x = base_x;
        float current_y = lerp(swing_start_y, swing_end_y, smooth_t);

        // Trajektoria łuku
        float arc_height = 4.0f * wave_config.lift_height * t * (1.0f - t);
        float current_z = base_z - arc_height;

        // Oblicz IK i ustaw serwa dla swing nogi
        float q1, q2, q3;
        if (computeLegIK(leg_number, current_x, current_y, current_z, &q1, &q2, &q3))
        {
            setLegJointsWithOffset(leg_number, q1, q2, q3, pca1, pca2);
        }

        // Zapisz pozycję końcową swing
        if (i == wave_config.step_points)
        {
            leg_current_y[leg_index] = swing_end_y;
            printf("SWING KONIEC Noga %d: y=%.1f\n", leg_number, swing_end_y);
        }

        // POZOSTAŁE NOGI: stoją w obecnych pozycjach (bez ruchu)
        for (int leg = 1; leg <= 6; leg++)
        {
            if (leg == leg_number)
                continue; // Pomiń swing leg

            int other_leg_index = leg - 1;
            float other_current_x = base_positions[other_leg_index][0];
            float other_current_y = leg_current_y[other_leg_index]; // Obecna pozycja (bez zmian)
            float other_current_z = base_positions[other_leg_index][2];

            if (computeLegIK(leg, other_current_x, other_current_y, other_current_z, &q1, &q2, &q3))
            {
                setLegJointsWithOffset(leg, q1, q2, q3, pca1, pca2);
            }
        }

        HAL_Delay(step_delay);
    }

    return true;
}

/**
 * @brief FAZA 2: Wykonaj STANCE SHIFT dla wszystkich nóg (1/6 zamiast 1/3)
 */
static bool executeStanceShift(WaveDirection_t direction,
                               PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2)
{
    printf("\n--- FAZA STANCE: Wszystkie nogi przesuwają się o 1/6 do tyłu ---\n");

    int stance_points = 20;                     // Mniej punktów dla stance (szybsze)
    uint32_t stance_delay = 10 / stance_points; // 10ms całkowity czas stance
    if (stance_delay == 0)
        stance_delay = 1;

    // KLUCZOWA RÓŻNICA: 1/6 zamiast 1/3 (bo 6 nóg zamiast 3 par)
    float stance_shift = wave_config.step_length / 6.0f;

    // Zapisz pozycje startowe stance
    float stance_start_y[6];
    for (int i = 0; i < 6; i++)
    {
        stance_start_y[i] = leg_current_y[i];
    }

    // === FAZA STANCE SHIFT ===
    for (int i = 0; i <= stance_points; i++)
    {
        float t = (float)i / (float)stance_points;
        float smooth_t = cubicInterpolation(t);

        // WSZYSTKIE NOGI przesuwają się o 1/6 do tyłu
        for (int leg = 1; leg <= 6; leg++)
        {
            int leg_index = leg - 1;

            float current_x = base_positions[leg_index][0];
            float current_z = base_positions[leg_index][2];

            // Stance shift: z obecnej pozycji o 1/6 do tyłu
            float stance_end_y = stance_start_y[leg_index] + stance_shift;
            float current_y = lerp(stance_start_y[leg_index], stance_end_y, smooth_t);

            float q1, q2, q3;
            if (computeLegIK(leg, current_x, current_y, current_z, &q1, &q2, &q3))
            {
                setLegJointsWithOffset(leg, q1, q2, q3, pca1, pca2);
            }

            // Zapisz nową pozycję
            if (i == stance_points)
            {
                leg_current_y[leg_index] = stance_end_y;
                if (leg == 1)
                    printf("STANCE SHIFT: +%.1f do tyłu (1/6)\n", stance_shift);
            }
        }

        HAL_Delay(stance_delay);
    }

    return true;
}

/**
 * @brief Wykonaj jeden krok jednej nogi (swing + stance shift)
 */
static bool executeLegStep(int leg_number, WaveDirection_t direction,
                           PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2)
{
    printf("\n=== KROK NOGI %d ===\n", leg_number);

    // FAZA 1: SWING dla nogi
    bool swing_success = executeSwingPhase(leg_number, direction, pca1, pca2);
    if (!swing_success)
    {
        printf("❌ Błąd w fazie swing nogi %d\n", leg_number);
        return false;
    }

    HAL_Delay(10); // Bardzo krótka pauza między fazami

    // FAZA 2: STANCE SHIFT dla wszystkich nóg
    bool stance_success = executeStanceShift(direction, pca1, pca2);
    if (!stance_success)
    {
        printf("❌ Błąd w fazie stance shift\n");
        return false;
    }

    return true;
}

/**
 * @brief Wykonaj jeden pełny cykl wave gait
 */
bool waveGaitCycle(PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2, WaveDirection_t direction)
{
    printf("\n=== WAVE GAIT CYCLE - SEKWENCYJNY ===\n");
    printf("Kierunek: %s\n",
           (direction == WAVE_FORWARD) ? "PRZÓD" : (direction == WAVE_BACKWARD) ? "TYŁ"
                                               : (direction == WAVE_LEFT)       ? "LEWO"
                                                                                : "PRAWO");

    // Inicjalizacja pozycji nóg
    if (!positions_initialized)
    {
        initializeLegPositions();
    }

    uint32_t cycle_start = HAL_GetTick();

    // SEKWENCJA 6 KROKÓW NÓŻEK (każdy krok = swing + stance shift)
    for (int step = 0; step < 6; step++)
    {
        int leg_number = wave_sequence[step];
        printf("\n>>> KROK %d/6: Noga %d <<<\n", step + 1, leg_number);

        bool success = executeLegStep(leg_number, direction, pca1, pca2);
        if (!success)
        {
            printf("❌ Błąd w kroku nogi %d\n", leg_number);
            return false;
        }

        HAL_Delay(5); // Bardzo krótka pauza między krokami
    }

    // Sprawdź pozycje końcowe
    printf("\n=== SPRAWDZENIE POZYCJI KOŃCOWYCH ===\n");
    for (int i = 0; i < 6; i++)
    {
        float expected_y = base_positions[i][1];
        float actual_y = leg_current_y[i];
        float diff = fabsf(actual_y - expected_y);

        printf("Noga %d: oczekiwane=%.1f, rzeczywiste=%.1f, różnica=%.1f\n",
               i + 1, expected_y, actual_y, diff);
    }

    uint32_t total_time = HAL_GetTick() - cycle_start;
    printf("\n✅ WAVE GAIT CYCLE ZAKOŃCZONY w %lu ms\n", total_time);

    return true;
}

/**
 * @brief Wykonaj ciągłe chodzenie wave
 */
bool waveGaitWalk(PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2,
                  WaveDirection_t direction, int num_cycles)
{
    printf("\n=== WAVE GAIT WALK START ===\n");
    printf("Liczba cykli: %d\n", num_cycles);

    for (int cycle = 0; cycle < num_cycles; cycle++)
    {
        printf("\n>>> CYKL WAVE %d/%d <<<\n", cycle + 1, num_cycles);

        bool success = waveGaitCycle(pca1, pca2, direction);
        if (!success)
        {
            printf("❌ Błąd w cyklu %d\n", cycle + 1);
            return false;
        }

        HAL_Delay(20); // Krótka pauza między cyklami
    }

    printf("\n✅ WAVE GAIT WALK ZAKOŃCZONY\n");
    return true;
}

/**
 * @brief Ustaw konfigurację wave gait
 */
void setWaveConfig(float step_length, float lift_height,
                   uint32_t step_duration, int step_points)
{
    wave_config.step_length = step_length;
    wave_config.lift_height = lift_height;
    wave_config.step_duration_ms = step_duration;
    wave_config.step_points = step_points;

    printf("✅ Konfiguracja wave zaktualizowana: krok=%.1fcm, podniesienie=%.1fcm, czas=%lums, punkty=%d\n",
           step_length, lift_height, step_duration, step_points);
}

/**
 * @brief Wyświetl aktualną konfigurację
 */
void printWaveConfig(void)
{
    printf("\n=== KONFIGURACJA WAVE GAIT ===\n");
    printf("Długość kroku: %.1f cm\n", wave_config.step_length);
    printf("Wysokość podniesienia: %.1f cm\n", wave_config.lift_height);
    printf("Czas swing: %lu ms\n", wave_config.step_duration_ms);
    printf("Punkty interpolacji: %d\n", wave_config.step_points);
    printf("Wysokość bazowa: %.1f cm\n", wave_config.step_height_base);
    printf("ALGORYTM: WAVE (sekwencyjny swing + stance shift o 1/6)\n");
    printf("SEKWENCJA: 1→2→3→4→5→6 (jedna noga na raz)\n");
    printf("STABILNOŚĆ: Najwyższa (zawsze 5 nóg na ziemi)\n");
    printf("===============================\n");
}