/*
 * bipedal_gait.c - Bipedal gait dla hexapoda - CORRECTED STANCE LOGIC
 *
 * Koncepcja: 3 pary nóg chodzące sekwencyjnie z fazą stance shift
 * - Para 0: Nogi 1,4 (lewa przednia + prawa środkowa)
 * - Para 1: Nogi 2,5 (prawa przednia + lewa tylna)
 * - Para 2: Nogi 3,6 (lewa środkowa + prawa tylna)
 *
 * POPRAWNA SEKWENCJA:
 * 1. Para X robi SWING (obecna pozycja → pozycja przednia), pozostałe STOJĄ
 * 2. FAZA STANCE: WSZYSTKIE nogi przesuwają się o 1/3 step_length do tyłu
 * 3. Powtórz dla kolejnej pary
 */

#include "bipedal_gait.h"
#include <stdio.h>
#include <math.h>

// Konfiguracja bipedal gait - ULTRA SZYBKA
BipedalConfig_t bipedal_config = {
    .step_length = 4.0f,       // Długość kroku [cm]
    .lift_height = 4.0f,       // Wysokość podniesienia [cm]
    .step_duration_ms = 50,    // Czas swing jednej pary [ms] - BARDZO SZYBKO
    .step_points = 20,         // Punkty interpolacji swing - DRASTYCZNIE MNIEJ
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

// Definicje par nóg
static const int leg_pairs[3][2] = {
    {1, 4}, // Para 0: lewa przednia + prawa środkowa
    {2, 5}, // Para 1: prawa przednia + lewa tylna
    {3, 6}  // Para 2: lewa środkowa + prawa tylna
};

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
    printf("🔧 Pozycje nóg zainicjalizowane\n");
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

    // USUNIĘTO INWERSJĘ KOLAN - wszystkie nogi mają ten sam kierunek

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
 * @brief FAZA 1: Wykonaj SWING dla pary nóg
 */
static bool executeSwingPhase(int pair_index, BipedalDirection_t direction,
                              PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2)
{
    printf("\n--- FAZA SWING: Para %d ---\n", pair_index);

    int swing_leg1 = leg_pairs[pair_index][0];
    int swing_leg2 = leg_pairs[pair_index][1];

    printf("SWING: nogi %d,%d | POZOSTAŁE: stoją bez ruchu\n", swing_leg1, swing_leg2);

    // ADAPTACYJNY DELAY - może być 0ms dla maksymalnej prędkości
    uint32_t step_delay = bipedal_config.step_duration_ms / bipedal_config.step_points;
    // NIE wymuszaj minimum 1ms - pozwól na 0ms dla ultra prędkości

    printf("Swing delay: %lu ms/punkt (total: %d punktów = %lu ms)\n",
           step_delay, bipedal_config.step_points, step_delay * bipedal_config.step_points);

    // === FAZA SWING ===
    for (int i = 0; i <= bipedal_config.step_points; i++)
    {
        float t = (float)i / (float)bipedal_config.step_points;
        float smooth_t = cubicInterpolation(t);

        // SWING dla pary
        for (int p = 0; p < 2; p++)
        {
            int leg_number = (p == 0) ? swing_leg1 : swing_leg2;
            int leg_index = leg_number - 1;

            float base_x = base_positions[leg_index][0];
            float base_y = base_positions[leg_index][1];
            float base_z = base_positions[leg_index][2];

            // Swing: z obecnej pozycji do pozycji przedniej
            float swing_start_y = leg_current_y[leg_index];          // Obecna pozycja
            float swing_end_y = base_y - bipedal_config.step_length; // Pozycja przednia

            float current_x = base_x;
            float current_y = lerp(swing_start_y, swing_end_y, smooth_t);

            // Trajektoria łuku
            float arc_height = 4.0f * bipedal_config.lift_height * t * (1.0f - t);
            float current_z = base_z - arc_height;

            // Oblicz IK i ustaw serwa
            float q1, q2, q3;
            if (computeLegIK(leg_number, current_x, current_y, current_z, &q1, &q2, &q3))
            {
                setLegJointsWithOffset(leg_number, q1, q2, q3, pca1, pca2);
            }

            // Zapisz pozycję końcową swing
            if (i == bipedal_config.step_points)
            {
                leg_current_y[leg_index] = swing_end_y;
                printf("SWING KONIEC Noga %d: y=%.1f\n", leg_number, swing_end_y);
            }
        }

        // POZOSTAŁE NOGI: stoją w obecnych pozycjach (bez ruchu)
        for (int leg = 1; leg <= 6; leg++)
        {
            if (leg == swing_leg1 || leg == swing_leg2)
                continue; // Pomiń swing legs

            int leg_index = leg - 1;
            float current_x = base_positions[leg_index][0];
            float current_y = leg_current_y[leg_index]; // Obecna pozycja (bez zmian)
            float current_z = base_positions[leg_index][2];

            float q1, q2, q3;
            if (computeLegIK(leg, current_x, current_y, current_z, &q1, &q2, &q3))
            {
                setLegJointsWithOffset(leg, q1, q2, q3, pca1, pca2);
            }
        }

        // USUŃ HAL_Delay dla maksymalnej prędkości!
        // HAL_Delay(step_delay);  // ← WYŁĄCZONE!
    }

    return true;
}

/**
 * @brief FAZA 2: Wykonaj STANCE SHIFT dla wszystkich nóg
 */
static bool executeStanceShift(BipedalDirection_t direction,
                               PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2)
{
    printf("\n--- FAZA STANCE: Wszystkie nogi przesuwają się o 1/3 do tyłu ---\n");

    // ULTRA SZYBKA STANCE - mniej punktów, szybszy delay
    int stance_points = 10;                     // DRASTYCZNIE MNIEJ punktów
    uint32_t stance_delay = 20 / stance_points; // 20ms całkowity czas stance (było 100ms)
    // Pozwól na 0ms delay dla ultra prędkości

    printf("Stance delay: %lu ms/punkt (total: %d punktów = %lu ms)\n",
           stance_delay, stance_points, stance_delay * stance_points);

    float stance_shift = bipedal_config.step_length / 3.0f;

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

        // WSZYSTKIE NOGI przesuwają się o 1/3 do tyłu
        for (int leg = 1; leg <= 6; leg++)
        {
            int leg_index = leg - 1;

            float current_x = base_positions[leg_index][0];
            float current_z = base_positions[leg_index][2];

            // Stance shift: z obecnej pozycji o 1/3 do tyłu
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
                    printf("STANCE SHIFT: +%.1f do tyłu\n", stance_shift);
            }
        }

        // USUŃ HAL_Delay dla maksymalnej prędkości!
        // HAL_Delay(stance_delay);  // ← WYŁĄCZONE!
    }

    return true;
}

/**
 * @brief Wykonaj jeden krok jednej pary (swing + stance shift)
 */
static bool executePairStep(int pair_index, BipedalDirection_t direction,
                            PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2)
{
    printf("\n=== KROK PARY %d ===\n", pair_index);

    // FAZA 1: SWING dla pary
    bool swing_success = executeSwingPhase(pair_index, direction, pca1, pca2);
    if (!swing_success)
    {
        printf("❌ Błąd w fazie swing pary %d\n", pair_index);
        return false;
    }

    // HAL_Delay(10); // USUŃ dla maksymalnej prędkości!

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
 * @brief Wykonaj jeden pełny cykl bipedal gait
 */
bool bipedalGaitCycle(PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2, BipedalDirection_t direction)
{
    printf("\n=== BIPEDAL GAIT CYCLE - 2-PHASE VERSION ===\n");
    printf("Kierunek: %s\n",
           (direction == BIPEDAL_FORWARD) ? "PRZÓD" : (direction == BIPEDAL_BACKWARD) ? "TYŁ"
                                                  : (direction == BIPEDAL_LEFT)       ? "LEWO"
                                                                                      : "PRAWO");

    // Inicjalizacja pozycji nóg
    if (!positions_initialized)
    {
        initializeLegPositions();
    }

    uint32_t cycle_start = HAL_GetTick();

    // SEKWENCJA 3 KROKÓW PAR (każdy krok = swing + stance shift)
    for (int pair = 0; pair < 3; pair++)
    {
        printf("\n>>> KROK %d/3 <<<\n", pair + 1);

        bool success = executePairStep(pair, direction, pca1, pca2);
        if (!success)
        {
            printf("❌ Błąd w kroku pary %d\n", pair);
            return false;
        }

        // HAL_Delay(20); // USUŃ dla maksymalnej prędkości!
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
    printf("\n✅ BIPEDAL GAIT CYCLE ZAKOŃCZONY w %lu ms\n", total_time);

    return true;
}

/**
 * @brief Wykonaj ciągłe chodzenie bipedal
 */
bool bipedalGaitWalk(PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2,
                     BipedalDirection_t direction, int num_cycles)
{
    printf("\n=== BIPEDAL GAIT WALK START ===\n");
    printf("Liczba cykli: %d\n", num_cycles);

    for (int cycle = 0; cycle < num_cycles; cycle++)
    {
        printf("\n>>> CYKL BIPEDAL %d/%d <<<\n", cycle + 1, num_cycles);

        bool success = bipedalGaitCycle(pca1, pca2, direction);
        if (!success)
        {
            printf("❌ Błąd w cyklu %d\n", cycle + 1);
            return false;
        }

        // HAL_Delay(50); // USUŃ dla maksymalnej prędkości!
    }

    printf("\n✅ BIPEDAL GAIT WALK ZAKOŃCZONY\n");
    return true;
}

/**
 * @brief Ustaw konfigurację bipedal gait
 */
void setBipedalConfig(float step_length, float lift_height,
                      uint32_t step_duration, int step_points)
{
    bipedal_config.step_length = step_length;
    bipedal_config.lift_height = lift_height;
    bipedal_config.step_duration_ms = step_duration;
    bipedal_config.step_points = step_points;

    printf("✅ Konfiguracja bipedal zaktualizowana: krok=%.1fcm, podniesienie=%.1fcm, czas=%lums, punkty=%d\n",
           step_length, lift_height, step_duration, step_points);
}

/**
 * @brief Wyświetl aktualną konfigurację
 */
void printBipedalConfig(void)
{
    printf("\n=== KONFIGURACJA BIPEDAL GAIT ===\n");
    printf("Długość kroku: %.1f cm\n", bipedal_config.step_length);
    printf("Wysokość podniesienia: %.1f cm\n", bipedal_config.lift_height);
    printf("Czas swing: %lu ms\n", bipedal_config.step_duration_ms);
    printf("Punkty interpolacji: %d\n", bipedal_config.step_points);
    printf("Wysokość bazowa: %.1f cm\n", bipedal_config.step_height_base);
    printf("ALGORYTM: 2-PHASE (swing + stance shift wszystkich nóg)\n");
    printf("================================\n");
}