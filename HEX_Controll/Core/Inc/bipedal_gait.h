/**
 * @file bipedal_gait.h
 * @brief Algorytm chodu bipedal (dwunożny) dla robota hexapod
 *
 * @details
 * Implementacja średnio-szybkiego chodu hexapoda bazującego na ruchu par nóg.
 * Bipedal gait charakteryzuje się sekwencyjnym ruchem trzech par nóg,
 * z fazą stance shift dla wszystkich pozostałych nóg między każdym krokiem pary.
 *
 * @section bipedal_concept Koncepcja bipedal gait
 *
 * **Podział nóg na pary:**
 * ```
 * Para 0: 1,4    Para 1: 2,5    Para 2: 3,6
 *
 *   1●────●2    ← Para 0,1
 *     │    │
 *   4●    ●5    ← Para 0,2
 *     │    │
 *   3●────●6    ← Para 1,2
 * ```
 *
 * **Sekwencja ruchu (2-fazowa):**
 * 1. **SWING Phase**: Para X robi krok (obecna pozycja → pozycja przednia)
 * 2. **STANCE SHIFT Phase**: WSZYSTKIE nogi przesuwają się o 1/3 step_length do tyłu
 * 3. Powtórzenie dla kolejnej pary (0→1→2→0...)
 *
 * **Zalety:**
 * - ⚡ **Ultra szybki** (wyłączone delay - ~60ms na cykl)
 * - 🔒 **Stabilny** (zawsze 4 nogi na ziemi podczas stance)
 * - 🎯 **Efektywny** energetycznie
 * - 📐 **Dobra kontrola** kierunku ruchu
 *
 * **Wady:**
 * - 🔧 Bardziej złożony algorytm niż tripod
 * - ⚖️ Wymaga precyzyjnej synchronizacji stance shift
 * - 🏔️ Mniej stabilny niż wave gait
 *
 * @section algorithm_details Szczegóły algorytmu
 *
 * **Kluczowa innowacja - 2-fazowy cycle:**
 *
 * Tradycyjny bipedal używał 6 faz (po jednej na parę + stance każdej).
 * Ten algorytm używa 2 faz na parę:
 *
 * 1. **SWING faza pary:**
 *    - Para X: podnosi się → łuk → opuszcza w pozycji przedniej
 *    - Pozostałe 4 nogi: stoją bez ruchu w obecnych pozycjach
 *
 * 2. **STANCE SHIFT wszystkich:**
 *    - WSZYSTKIE 6 nóg: przesuw o 1/3 step_length do tyłu po ziemi
 *    - Efekt: robot przesuwa się do przodu względem nóg
 *
 * **Matematyka stance shift:**
 * ```
 * stance_shift = step_length / 3.0f
 *
 * Po 3 parach: 3 × (1/3) = 1 × step_length
 * → Robot przesunął się o pełną długość kroku
 * ```
 *
 * @section ultra_speed_mode Ultra Speed Mode
 *
 * **Radykalne optymalizacje prędkości:**
 * -  **Wyłączone ALL HAL_Delay()** w pętlach interpolacji
 * -  **20 punktów interpolacji** swing (było 50)
 * -  **10 punktów interpolacji** stance (było 20)
 * -  **50ms swing + 20ms stance** = 70ms na parę
 * -  **210ms na pełny cykl** (3 pary × 70ms)
 *
 * **Porównanie prędkości:**
 * | Mode | Swing | Stance | Punkty | Cykl | Delay |
 * |------|-------|--------|--------|------|-------|
 * | **Bezpieczny** | 150ms | 100ms | 50/20 | 750ms | Tak |
 * | **Ultra (current)** | 50ms | 20ms | 20/10 | 210ms | x |
 * | **Przyrost** | **3x** | **5x** | **2.5x** | **3.6x** | **∞** |
 *
 * @section pair_mapping Mapowanie par nóg
 *
 * **Para 0: Nogi 1,4 (Lewa przednia + Prawa środkowa)**
 * - Geometria: Stabilny trójkąt z nogami 2,3,5,6 na ziemi
 * - Offsety: +37.5° (noga 1) + 0° (noga 4)
 * - Charakterystyka: Dobra stabilność przednia
 *
 * **Para 1: Nogi 2,5 (Prawa przednia + Lewa tylna)**
 * - Geometria: Diagonalny trójkąt stabilności
 * - Offsety: -37.5° (noga 2) + -37.5° (noga 5)
 * - Charakterystyka: Najdłuższa baza podparcia
 *
 * **Para 2: Nogi 3,6 (Lewa środkowa + Prawa tylna)**
 * - Geometria: Stabilny trójkąt z nogami 1,2,4,5 na ziemi
 * - Offsety: 0° (noga 3) + +37.5° (noga 6)
 * - Charakterystyka: Dobra stabilność tylna
 *
 * @section movement_directions Kierunki ruchu
 *
 * | Kierunek | Modyfikacja współrzędnych | Efekt |
 * |----------|---------------------------|-------|
 * | **FORWARD** | Y -= step_length | Ruch do przodu |
 * | **BACKWARD** | Y += step_length | Ruch do tyłu |
 * | **LEFT** | X += step_length | Ruch w lewo (sideways) |
 * | **RIGHT** | X -= step_length | Ruch w prawo (sideways) |
 *
 * @section hardware_integration Integracja sprzętowa
 *
 * **Dual I2C Architecture:**
 * - **I2C1 → PCA9685 #1**: Lewe nogi (1,3,5) - kanały 0-8
 * - **I2C2 → PCA9685 #2**: Prawe nogi (2,4,6) - kanały 0-8
 *
 * **Mapowanie par na kontrolery:**
 * - **Para 0 (1,4)**: I2C1[0-2] + I2C2[3-5] - Cross-controller
 * - **Para 1 (2,5)**: I2C2[0-2] + I2C1[6-8] - Cross-controller
 * - **Para 2 (3,6)**: I2C1[3-5] + I2C2[6-8] - Cross-controller
 *
 * @section performance_analysis Analiza wydajności
 *
 * **Teoretyczna prędkość robota:**
 * ```
 * step_length = 4.0 cm
 * cycle_time = 210 ms (ultra mode)
 *
 * speed = step_length / cycle_time = 4.0cm / 0.21s = 19.0 cm/s
 *
 * vs Tripod: 6.0cm / 0.12s = 50.0 cm/s
 * vs Wave: 4.0cm / 0.36s = 11.1 cm/s
 * ```
 *
 * **Ranking prędkości:** Tripod > Bipedal > Wave
 *
 * **Ranking stabilności:** Wave > Bipedal > Tripod
 *
 * @author Maksymilian Tulewicz
 * @date 2025
 * @version 1.0
 *
 * @see hexapod_kinematics.h dla obliczeń IK
 * @see tripod_gait.h dla porównania algorytmów
 */

#ifndef BIPEDAL_GAIT_H
#define BIPEDAL_GAIT_H

#include "stm32f4xx_hal.h"
#include "pca9685.h"
#include "hexapod_kinematics.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup Bipedal_Types Typy i enumeracje
 * @{
 */

/**
 * @brief Kierunki ruchu w bipedal gait
 *
 * @details
 * Enum definiujący dostępne kierunki poruszania się robota w trybie bipedal.
 * Bipedal gait nie implementuje obrotów w miejscu (TURN_LEFT/RIGHT) -
 * dla obrotów użyj tripod lub wave gait.
 */
typedef enum
{
    BIPEDAL_FORWARD = 0, ///< Ruch do przodu (Y -= step_length)
    BIPEDAL_BACKWARD,    ///< Ruch do tyłu (Y += step_length)
    BIPEDAL_LEFT,        ///< Ruch w lewo - sideways (X += step_length)
    BIPEDAL_RIGHT        ///< Ruch w prawo - sideways (X -= step_length)
} BipedalDirection_t;

/**
 * @brief Konfiguracja parametrów bipedal gait
 *
 * @details
 * Struktura zawierająca wszystkie konfigurowalne parametry algorytmu bipedal.
 * Ultra speed mode używa drastycznie zredukowanych wartości dla maksymalnej prędkości.
 *
 * **Ultra Speed Values (current):**
 * - step_length: 4.0 cm (bezpieczny kompromis)
 * - lift_height: 4.0 cm (wystarczający clearance)
 * - step_duration_ms: 50 ms (bardzo szybko!)
 * - step_points: 20 (minimum dla płynności)
 *
 * **Zalecane zakresy (bezpieczny mode):**
 * - step_length: 2.0-6.0 cm
 * - lift_height: 2.0-6.0 cm
 * - step_duration_ms: 100-300 ms
 * - step_points: 30-100
 */
typedef struct
{
    float step_length;         ///< Długość kroku [cm] - dystans przesunięcia pary nóg
    float lift_height;         ///< Wysokość podniesienia [cm] - clearance nad ziemią
    uint32_t step_duration_ms; ///< Czas swing jednej pary [ms] - ultra szybki: 50ms
    int step_points;           ///< Punkty interpolacji swing - ultra szybki: 20
    float step_height_base;    ///< Bazowa wysokość stania [cm] - pozycja Z w stance
} BipedalConfig_t;

/** @} */ // end of Bipedal_Types

/**
 * @defgroup Bipedal_Functions Funkcje publiczne API
 * @{
 */

/**
 * @brief Wykonaj jeden pełny cykl bipedal gait (3 pary nóg)
 *
 * @details
 * Główna funkcja algorytmu bipedal gait wykonująca jeden kompletny cykl
 * składający się z 3 kroków par nóg. Każdy krok pary to 2 fazy:
 *
 * **Sekwencja pełnego cyklu:**
 * 1. **Para 0 (1,4)**: SWING + STANCE SHIFT wszystkich
 * 2. **Para 1 (2,5)**: SWING + STANCE SHIFT wszystkich
 * 3. **Para 2 (3,6)**: SWING + STANCE SHIFT wszystkich
 *
 * **Szczegóły każdego kroku pary:**
 *
 * **FAZA SWING (50ms, 20 punktów):**
 * ```
 * for (para w [0,1,2]) {
 *   Para: podniesienie → łuk → opuszczenie (pozycja przednia)
 *   Pozostałe 4 nogi: bez ruchu (utrzymanie obecnej pozycji)
 * }
 * ```
 *
 * **FAZA STANCE SHIFT (20ms, 10 punktów):**
 * ```
 * WSZYSTKIE 6 nóg: przesuw o 1/3 step_length do tyłu po ziemi
 * Efekt: robot przesuwa się do przodu względem otoczenia
 * ```
 *
 * **Automatyczna inicjalizacja pozycji:**
 * - Pierwsze wywołanie inicjalizuje pozycje nóg
 * - Trackowanie pozycji Y każdej nogi w runtime
 * - Automatyczna korekcja dryfu pozycyjnego
 *
 * @param[in] pca1 Wskaźnik na kontroler lewych nóg (I2C1) lub NULL
 * @param[in] pca2 Wskaźnik na kontroler prawych nóg (I2C2) lub NULL
 * @param[in] direction Kierunek ruchu (BIPEDAL_FORWARD, BACKWARD, LEFT, RIGHT)
 *
 * @return true Cykl wykonany pomyślnie
 * @return false Błąd podczas wykonywania (IK failuje, I2C error)
 *
 * @note Funkcja automatycznie sprawdza dostępność kontrolerów
 * @note W przypadku NULL dla pca1/pca2 - odpowiednie nogi są pomijane
 * @warning Ultra speed mode może być niestabilny na nierównym terenie
 *
 * @code{.c}
 * // Jeden ultra szybki cykl do przodu
 * if (!bipedalGaitCycle(&pca1, &pca2, BIPEDAL_FORWARD)) {
 *     printf("Błąd w cyklu bipedal!\n");
 * }
 *
 * // Ruch w lewo (sideways)
 * bipedalGaitCycle(&pca1, &pca2, BIPEDAL_LEFT);
 * @endcode
 *
 * @see setBipedalConfig() dla dostrajania prędkości
 * @see bipedalGaitWalk() dla ciągłego chodzenia
 */
bool bipedalGaitCycle(PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2, BipedalDirection_t direction);

/**
 * @brief Wykonaj ciągłe chodzenie bipedal (wiele cykli)
 *
 * @details
 * Funkcja wysokiego poziomu wykonująca określoną liczbę cykli bipedal gait
 * z ultra speed optimizations. Najszybszy sposób poruszania się po
 * płaskim terenie (po tripod gait).
 *
 * **Sekwencja działania:**
 * 1. Wyświetlenie informacji o konfiguracji ultra speed
 * 2. Sprawdzenie statusu dual I2C controllers
 * 3. Wykonanie num_cycles ultra szybkich cykli
 * 4. Minimalne pauzy między cyklami (50ms)
 * 5. Raportowanie całkowitego czasu wykonania
 *
 * **Performance Characteristics:**
 * ```
 * 1 cycle = 3 pairs × 70ms = 210ms
 * 5 cycles = 5 × 210ms + 4 × 50ms = 1.25s
 * Distance = 5 × 4.0cm = 20cm
 * Speed = 20cm / 1.25s = 16 cm/s
 * ```
 *
 * **Error Handling:**
 * - Stop na pierwszym błędzie cyklu
 * - Szczegółowe logi przyczyny błędu
 * - Bezpieczne zatrzymanie w ostatniej pozycji
 * - Możliwość kontynuacji po naprawie błędu
 *
 * @param[in] pca1 Wskaźnik na kontroler lewych nóg (I2C1)
 * @param[in] pca2 Wskaźnik na kontroler prawych nóg (I2C2)
 * @param[in] direction Kierunek ruchu przez wszystkie cykle
 * @param[in] num_cycles Liczba cykli (1-20 zalecane dla ultra speed)
 *
 * @return true Wszystkie cykle wykonane pomyślnie
 * @return false Błąd w którymś z cykli (szczegóły w logach)
 *
 * @note Czas blokowania: num_cycles × 210ms + (num_cycles-1) × 50ms
 * @note Ultra speed może powodować przegrzanie serw przy długich sesjach
 * @warning Nie zalecane >10 cykli bez pauzy chłodzącej
 *
 * @code{.c}
 * // Ultra szybki sprint (3 cykle = ~60cm)
 * bipedalGaitWalk(&pca1, &pca2, BIPEDAL_FORWARD, 3);
 *
 * // Średni spacer z chłodzeniem
 * for (int i = 0; i < 3; i++) {
 *     bipedalGaitWalk(&pca1, &pca2, BIPEDAL_FORWARD, 5);
 *     HAL_Delay(2000); // Cooling break
 * }
 *
 * // Quick sideways maneuver
 * bipedalGaitWalk(&pca1, &pca2, BIPEDAL_LEFT, 2);
 * @endcode
 *
 * @see bipedalGaitCycle() wykonywana wewnętrznie
 */
bool bipedalGaitWalk(PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2,
                     BipedalDirection_t direction, int num_cycles);

/**
 * @brief Ustaw konfigurację bipedal gait w runtime
 *
 * @details
 * Funkcja pozwalająca na dostrajanie parametrów ultra speed mode
 * lub przełączenie na bardziej konserwatywne ustawienia.
 *
 * **Predefiniowane zestawy:**
 *
 * **Ultra Speed (current):**
 * ```c
 * setBipedalConfig(4.0f, 4.0f, 50, 20);
 * // Efekt: Maksymalna prędkość, akceptowalna płynność
 * ```
 *
 * **Balanced Mode:**
 * ```c
 * setBipedalConfig(5.0f, 4.0f, 100, 40);
 * // Efekt: Dobra prędkość + lepsza płynność
 * ```
 *
 * **Safe Mode:**
 * ```c
 * setBipedalConfig(3.0f, 5.0f, 150, 60);
 * // Efekt: Bezpieczny na nierównym terenie
 * ```
 *
 * **Wpływ parametrów:**
 * - **↑ step_length**: Większe kroki = szybsze przemieszczanie
 * - **↑ lift_height**: Wyższy clearance = bezpieczniej
 * - **↓ step_duration_ms**: Szybsze ruchy = mniej stabilne
 * - **↑ step_points**: Płynniejsze = wolniejsze
 *
 * @param[in] step_length Długość kroku [cm] (1.0-8.0)
 * @param[in] lift_height Wysokość podniesienia [cm] (1.0-8.0)
 * @param[in] step_duration Czas swing [ms] (20-300)
 * @param[in] step_points Punkty interpolacji (10-100)
 *
 * @note Zmiany globalne - dotyczą wszystkich kolejnych cykli
 * @note Funkcja loguje nową konfigurację
 * @warning Ekstremalne wartości mogą destabilizować robota
 *\
 *
 * @see printBipedalConfig() dla sprawdzenia aktualnych ustawień
 */
void setBipedalConfig(float step_length, float lift_height,
                      uint32_t step_duration, int step_points);

/**
 * @brief Wyświetl aktualną konfigurację bipedal gait
 *
 * @details
 * Funkcja diagnostyczna wypisująca kompletne informacje o aktualnej
 * konfiguracji ultra speed bipedal gait oraz charakterystykę algorytmu.
 *
 * **Wyświetlane informacje:**
 * - Wszystkie parametry konfiguracyjne
 * - Status ultra speed mode vs safe mode
 * - Szacowany czas cyklu i prędkość robota
 * - Opis algorytmu 2-fazowego (swing + stance shift)
 * - Mapowanie par nóg i offsety bioder
 * - Porównanie z innymi algorytmami (tripod, wave)
 *
 * **Przykład output:**
 * ```
 * === KONFIGURACJA BIPEDAL GAIT ===
 * Długość kroku: 4.0 cm
 * Wysokość podniesienia: 4.0 cm
 * Czas swing: 50 ms
 * Punkty interpolacji: 20
 * ALGORYTM: 2-PHASE (swing + stance shift wszystkich nóg)
 * STABILNOŚĆ: 4 nogi na ziemi podczas stance
 * PRĘDKOŚĆ: Szybka (bez delay)
 * ================================
 * ```
 *
 * @note Funkcja tylko wyświetla - nie modyfikuje konfiguracji
 * @note Output przez printf() (UART2 w tym projekcie)
 *
 * @code{.c}
 * // Sprawdzenie przed testem wydajności
 * printf("=== DIAGNOSTYKA BIPEDAL ===\n");
 * printBipedalConfig();
 * printf("===========================\n");
 * @endcode
 *
 * @see setBipedalConfig() dla modyfikacji ustawień
 */
void printBipedalConfig(void);

/** @} */ // end of Bipedal_Functions

/**
 * @defgroup Bipedal_Implementation Szczegóły implementacji
 * @{
 */

/**
 * @brief Wewnętrzne śledzenie pozycji nóg
 *
 * @details
 * Algorytm bipedal musi śledzić aktualną pozycję Y każdej nogi,
 * ponieważ stance shift modyfikuje pozycje wszystkich nóg równocześnie.
 *
 * **Static variables (wewnętrzne):**
 * ```c
 * static float leg_current_y[6];          // Aktualne pozycje Y
 * static bool positions_initialized;      // Flaga inicjalizacji
 * ```
 *
 * **Inicjalizacja przy pierwszym użyciu:**
 * ```c
 * if (!positions_initialized) {
 *     for (int i = 0; i < 6; i++) {
 *         leg_current_y[i] = base_positions[i][1]; // Pozycje bazowe
 *     }
 *     positions_initialized = true;
 * }
 * ```
 *
 * **Aktualizacja po każdym stance shift:**
 * ```c
 * stance_shift = step_length / 3.0f;
 * for (int leg = 1; leg <= 6; leg++) {
 *     leg_current_y[leg-1] += stance_shift; // Przesuw o 1/3 do tyłu
 * }
 * ```
 */

/**
 * @brief Mapowanie sprzętowe z offsetami bioder
 *
 * **Wewnętrzna struktura mapowania:**
 * ```c
 * typedef struct {
 *     uint8_t base_channel;  // Bazowy kanał PCA9685 (0, 3, 6)
 *     float hip_offset_deg;  // Offset biodra z URDF [stopnie]
 *     bool is_left_side;     // true = I2C1, false = I2C2
 * } LegMapping_t;
 *
 * static const LegMapping_t leg_mapping[6] = {
 *     {0, 37.5f, true},   // Noga 1: I2C1[0-2], +37.5°
 *     {0, -37.5f, false}, // Noga 2: I2C2[0-2], -37.5°
 *     {3, 0.0f, true},    // Noga 3: I2C1[3-5], 0°
 *     {3, 0.0f, false},   // Noga 4: I2C2[3-5], 0°
 *     {6, -37.5f, true},  // Noga 5: I2C1[6-8], -37.5°
 *     {6, 37.5f, false}   // Noga 6: I2C2[6-8], +37.5°
 * };
 * ```
 *
 * **Automatyczne uwzględnienie offsetów:**
 * ```c
 * float hip_deg = (q1 * 180.0f / M_PI) + mapping->hip_offset_deg;
 * float servo_hip = 90.0f + hip_deg;  // Mapowanie na servo
 * PCA9685_SetServoAngle(pca_to_use, mapping->base_channel + 0, servo_hip);
 * ```
 */

/** @} */ // end of Bipedal_Implementation

#endif // BIPEDAL_GAIT_H