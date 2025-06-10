/**
 * @file wave_gait.h
 * @brief Algorytm chodu wave (falowy) dla robota hexapod
 *
 * @details
 * Implementacja najstabilniejszego chodu dla robotów 6-nożnych.
 * Wave gait charakteryzuje się sekwencyjnym ruchem pojedynczych nóg,
 * zapewniając maksymalną stabilność kosztem prędkości ruchu.
 *
 * @section wave_concept Koncepcja wave gait
 *
 * **Sekwencja ruchu nóg:**
 * ```
 * 1 → 2 → 3 → 4 → 5 → 6 → 1 → ...
 *
 *   1●────●2    ← Sekwencja: 1→2
 *     │    │
 *   3●    ●4    ← Sekwencja: 3→4
 *     │    │
 *   5●────●6    ← Sekwencja: 5→6
 * ```
 *
 * **Algorytm (2-fazowy na każdą nogę):**
 * 1. **SWING Phase**: Noga X robi krok (obecna pozycja → pozycja przednia)
 * 2. **STANCE SHIFT Phase**: WSZYSTKIE nogi przesuwają się o 1/6 step_length do tyłu
 * 3. Powtórzenie dla kolejnej nogi (1→2→3→4→5→6→1...)
 *
 * **Zalety:**
 * -  **Najstabilniejszy** (zawsze 5 nóg na ziemi)
 * -  **Najlepszy na nierównym terenie**
 * -  **Minimalne przechylenia** korpusu
 * -  **Łatwy debug** (jedna noga na raz)
 * -  **Maksymalne podparcie** w każdej chwili
 *
 * **Wady:**
 * -  **Najwolniejszy** ze wszystkich chodów
 * -  **6 faz na cykl** (vs 2 w tripod)
 * -  **Więcej obliczeń** kinematyki
 *
 * @section algorithm_details Szczegóły algorytmu
 *
 * **Kluczowa różnica vs inne chody:**
 *
 * | Gait | Nogi w ruchu | Nogi na ziemi | Faz/cykl | Stabilność |
 * |------|--------------|---------------|----------|------------|
 * | **Wave** | **1** | **5** | **6** | najlepsza |
 * | Bipedal | 2 | 4 | 3 | dobra |
 * | Tripod | 3 | 3 | 2 | ok |
 *
 *  * **Matematyka stance shift:**
 * ```
 * stance_shift = step_length / 6.0f
 *
 * Po 6 nogach: 6 × (1/6) = 1 × step_length
 * → Robot przesunął się o pełną długość kroku
 * ```
 *
 * **Sekwencja pełnego cyklu:**
 * 1. **Noga 1**: SWING + STANCE SHIFT wszystkich o 1/6
 * 2. **Noga 2**: SWING + STANCE SHIFT wszystkich o 1/6
 * 3. **Noga 3**: SWING + STANCE SHIFT wszystkich o 1/6
 * 4. **Noga 4**: SWING + STANCE SHIFT wszystkich o 1/6
 * 5. **Noga 5**: SWING + STANCE SHIFT wszystkich o 1/6
 * 6. **Noga 6**: SWING + STANCE SHIFT wszystkich o 1/6
 *
 *
 * **Zastosowania wave gait:**
 * -  Trudny, nierówny teren
 * -  Precyzyjne pozycjonowanie
 * -  Testowanie i kalibracja
 * -  Sytuacje wymagające maksymalnej stabilności
 *
 *
 * @section base_positions Zoptymalizowane pozycje bazowe
 *
 * **Przybliżone pozycje do ciała (vs inne gaits):**
 * ```c
 * // Wave gait - zmniejszona dźwignia dla stabilności
 * {15.0f, -12.0f, -24.0f},  // Noga 1 (było 18.0f, -15.0f w tripod)
 * {-15.0f, -12.0f, -24.0f}, // Noga 2 (było -18.0f, -15.0f w tripod)
 * {18.0f, 0.0f, -24.0f},    // Noga 3 (bez zmian - środkowe)
 * {-18.0f, 0.0f, -24.0f},   // Noga 4 (bez zmian - środkowe)
 * {15.0f, 12.0f, -24.0f},   // Noga 5 (było 18.0f, 15.0f w tripod)
 * {-15.0f, 12.0f, -24.0f}   // Noga 6 (było -18.0f, 15.0f w tripod)
 * ```
 *
 * **Powód zmian:**
 * - **Krótsze dźwignie** = mniejsze momenty obrotowe
 * - **Lepsze właściwości kinematyczne** dla małych kroków
 * - **Wyższa precyzja** pozycjonowania
 * - **Mniejsze obciążenia** mechaniczne serw
 *
 * @author Maksymilian Tulewicz
 * @date 2025
 * @version 1.0
 *
 * @see hexapod_kinematics.h dla obliczeń IK
 * @see tripod_gait.h dla porównania z szybkim chodem
 * @see bipedal_gait.h dla porównania z chodem średnim
 */

#ifndef WAVE_GAIT_H
#define WAVE_GAIT_H

#include "stm32f4xx_hal.h"
#include "pca9685.h"
#include "hexapod_kinematics.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup Wave_Types Typy i enumeracje
 * @{
 */

/**
 * @brief Kierunki ruchu w wave gait
 *
 * @details
 * Enum definiujący dostępne kierunki poruszania się robota w trybie wave.
 * Wave gait nie implementuje obrotów w miejscu - dla precyzyjnych obrotów
 * zaleca się kombinację LEFT/RIGHT z krótkimi krokami.
 */
typedef enum
{
    WAVE_FORWARD = 0, ///< Ruch do przodu (Y -= step_length)
    WAVE_BACKWARD,    ///< Ruch do tyłu (Y += step_length)
    WAVE_LEFT,        ///< Ruch w lewo - sideways (X += step_length)
    WAVE_RIGHT        ///< Ruch w prawo - sideways (X -= step_length)
} WaveDirection_t;

/**
 * @brief Konfiguracja parametrów wave gait
 *
 * @details
 * Struktura zawierająca wszystkie konfigurowalne parametry algorytmu wave.
 * Wartości zoptymalizowane dla maksymalnej stabilności przy akceptowalnej prędkości.
 *
 * **Zalecane wartości (current):**
 * - step_length: 4.0 cm (bezpieczny krok)
 * - lift_height: 4.0 cm (wystarczający clearance)
 * - step_duration_ms: 10 ms (szybkie ale kontrolowane)
 * - step_points: 50 (dobra płynność)
 *
 * **Zakresy dla różnych zastosowań:**
 * - **Precyzyjne**: step_length 2.0-3.0 cm, step_duration 50-100 ms
 * - **Standardowe**: step_length 3.0-5.0 cm, step_duration 10-30 ms
 * - **Szybkie**: step_length 5.0-6.0 cm, step_duration 5-15 ms
 */
typedef struct
{
    float step_length;         ///< Długość kroku [cm] - dystans przesunięcia nogi
    float lift_height;         ///< Wysokość podniesienia [cm] - clearance nad ziemią
    uint32_t step_duration_ms; ///< Czas swing jednej nogi [ms] - szybki: 10ms
    int step_points;           ///< Punkty interpolacji swing - płynny: 50
    float step_height_base;    ///< Bazowa wysokość stania [cm] - pozycja Z w stance
} WaveConfig_t;

/** @} */ // end of Wave_Types

/**
 * @defgroup Wave_Functions Funkcje publiczne API
 * @{
 */

/**
 * @brief Wykonaj jeden pełny cykl wave gait (6 nóg sekwencyjnie)
 *
 * @details
 * Główna funkcja algorytmu wave gait wykonująca jeden kompletny cykl
 * składający się z 6 kroków pojedynczych nóg. Każdy krok to 2 fazy:
 *
 * **Sekwencja pełnego cyklu:**
 * 1. **Noga 1**: SWING + STANCE SHIFT wszystkich o 1/6
 * 2. **Noga 2**: SWING + STANCE SHIFT wszystkich o 1/6
 * 3. **Noga 3**: SWING + STANCE SHIFT wszystkich o 1/6
 * 4. **Noga 4**: SWING + STANCE SHIFT wszystkich o 1/6
 * 5. **Noga 5**: SWING + STANCE SHIFT wszystkich o 1/6
 * 6. **Noga 6**: SWING + STANCE SHIFT wszystkich o 1/6
 *
 * **Szczegóły każdego kroku nogi:**
 *
 * **FAZA SWING (10ms, 50 punktów):**
 * ```
 * for (noga w [1,2,3,4,5,6]) {
 *   Noga: podniesienie → łuk → opuszczenie (pozycja przednia)
 *   Pozostałe 5 nóg: bez ruchu (utrzymanie obecnej pozycji)
 * }
 * ```
 *
 * **FAZA STANCE SHIFT (kratka time, 20 punktów):**
 * ```
 * WSZYSTKIE 6 nóg: przesuw o 1/6 step_length do tyłu po ziemi
 * Efekt: robot przesuwa się do przodu względem otoczenia
 * ```
 *
 * **Automatyczna inicjalizacja pozycji:**
 * - Pierwsze wywołanie ustawia pozycje bazowe (przybliżone do ciała)
 * - Runtime tracking pozycji Y każdej nogi
 * - Automatyczna korekcja pozycyjnego drift
 *
 * @param[in] pca1 Wskaźnik na kontroler lewych nóg (I2C1) lub NULL
 * @param[in] pca2 Wskaźnik na kontroler prawych nóg (I2C2) lub NULL
 * @param[in] direction Kierunek ruchu (WAVE_FORWARD, BACKWARD, LEFT, RIGHT)
 *
 * @return true Cykl wykonany pomyślnie
 * @return false Błąd podczas wykonywania (IK fail, I2C error)
 *
 * @note Funkcja automatycznie sprawdza dostępność kontrolerów
 * @note W przypadku NULL dla pca1/pca2 - odpowiednie nogi są pomijane
 * @note Wave gait jest najbezpieczniejszy dla eksperymentów
 *
 * @code{.c}
 * // Jeden stabilny cykl do przodu
 * if (!waveGaitCycle(&pca1, &pca2, WAVE_FORWARD)) {
 *     printf("Błąd w cyklu wave - sprawdź kinematykę\n");
 * }
 *
 * // Precyzyjny ruch w lewo
 * waveGaitCycle(&pca1, &pca2, WAVE_LEFT);
 * @endcode
 *
 * @see setWaveConfig() dla dostrajania parametrów
 * @see waveGaitWalk() dla ciągłego chodzenia
 */
bool waveGaitCycle(PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2, WaveDirection_t direction);

/**
 * @brief Wykonaj ciągłe chodzenie wave (wiele cykli)
 *
 * @details
 * Funkcja wysokiego poziomu wykonująca określoną liczbę cykli wave gait.
 * Najstabilniejszy sposób poruszania się robota, szczególnie przydatny
 * na nierównym terenie lub przy przenoszeniu ładunków.
 *
 * **Sekwencja działania:**
 * 1. Wyświetlenie informacji o konfiguracji stabilności
 * 2. Sprawdzenie statusu dual I2C controllers
 * 3. Wykonanie num_cycles stabilnych cykli wave
 * 4. Krótkie pauzy między cyklami (20ms dla stabilności)
 * 5. Raportowanie całkowitego czasu i średniej prędkości
 *

 * **Error Handling:**
 * - Kontynuacja po błędzie pojedynczej nogi (5 pozostałych)
 * - Szczegółowe logi przyczyny błędów per noga
 * - Bezpieczne zatrzymanie w stabilnej pozycji
 * - Możliwość natychmiastowej kontynuacji
 *
 * @param[in] pca1 Wskaźnik na kontroler lewych nóg (I2C1)
 * @param[in] pca2 Wskaźnik na kontroler prawych nóg (I2C2)
 * @param[in] direction Kierunek ruchu przez wszystkie cykle
 * @param[in] num_cycles Liczba cykli (1-50 zalecane)
 *
 * @return true Wszystkie cykle wykonane pomyślnie
 * @return false Błąd w którymś z cykli (szczegóły w logach)
 *
 * @note Czas blokowania: num_cycles × 360ms + (num_cycles-1) × 20ms
 * @note Wave może działać przez dłuższe okresy bez przegrzania serw
 * @note Zalecany dla długich tras i precyzyjnych manewrów
 *
 * @code{.c}
 * // Stabilny spacer (5 cykli = ~20cm)
 * waveGaitWalk(&pca1, &pca2, WAVE_FORWARD, 5);
 *
 * // Długa trasa z maksymalną stabilnością
 * for (int segment = 0; segment < 10; segment++) {
 *     waveGaitWalk(&pca1, &pca2, WAVE_FORWARD, 10);
 *     HAL_Delay(500); // Krótkadła nawigacyjna
 * }

 * @endcode
 *
 * @see waveGaitCycle() wykonywana wewnętrznie
 */
bool waveGaitWalk(PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2,
                  WaveDirection_t direction, int num_cycles);

/**
 * @brief Ustaw konfigurację wave gait w runtime
 *
 * @details
 * Funkcja pozwalająca na dostrajanie parametrów wave gait dla różnych
 * zastosowań - od precyzyjnych manewrów po względnie szybkie przemieszczanie.
 *
 * **Predefiniowane zestawy:**
 *
 * **Ultra Stable (precyzyjne manewry):**
 * ```c
 * setWaveConfig(2.0f, 5.0f, 50, 80);
 * // Efekt: Maksymalna precyzja, bardzo wolno
 * ```
 *
 * **Balanced (current):**
 * ```c
 * setWaveConfig(4.0f, 4.0f, 10, 50);
 * // Efekt: Dobra stabilność + akceptowalna prędkość
 * ```
 *
 * **Fast Stable:**
 * ```c
 * setWaveConfig(5.0f, 3.0f, 5, 30);
 * // Efekt: Najszybszy wave możliwy przy zachowaniu stabilności
 * ```
 *
 * **Heavy Payload:**
 * ```c
 * setWaveConfig(3.0f, 6.0f, 30, 70);
 * // Efekt: Wysokie podniesienie + wolne ruchy dla ładunku
 * ```
 *
 * **Wpływ parametrów na stabilność:**
 * - **↓ step_length**: Mniejsze ruchy = wyższa stabilność
 * - **↑ lift_height**: Wyższy clearance = bezpieczniej na nierównym terenie
 * - **↑ step_duration_ms**: Wolniejsze ruchy = płynniejsze, stabilniejsze
 * - **↑ step_points**: Płynniejsza trajektoria = mniej wibracji
 *
 * @param[in] step_length Długość kroku [cm] (1.0-6.0)
 * @param[in] lift_height Wysokość podniesienia [cm] (2.0-8.0)
 * @param[in] step_duration Czas swing [ms] (5-100)
 * @param[in] step_points Punkty interpolacji (20-100)
 *
 * @note Zmiany globalne - dotyczą wszystkich kolejnych cykli
 * @note Wave toleruje ekstremalne wartości lepiej niż inne gaits
 * @note Funkcja loguje nową konfigurację z analizą stabilności
 *
 * @code{.c}
 * // Przełączenie na tryb precyzyjny
 * setWaveConfig(2.5f, 5.0f, 40, 70);
 *
 * // Szybki wave dla płaskiego terenu
 * setWaveConfig(5.0f, 3.0f, 8, 35);
 *
 * // Powrót do ustawień domyślnych
 * setWaveConfig(4.0f, 4.0f, 10, 50);
 * @endcode
 *
 * @see printWaveConfig() dla sprawdzenia aktualnych ustawień
 */
void setWaveConfig(float step_length, float lift_height,
                   uint32_t step_duration, int step_points);

/**
 * @brief Wyświetl aktualną konfigurację wave gait
 *
 * @details
 * Funkcja diagnostyczna wypisująca kompletne informacje o aktualnej
 * konfiguracji wave gait oraz charakterystykę algorytmu stabilności.
 *
 * **Wyświetlane informacje:**
 * - Wszystkie parametry konfiguracyjne
 * - Analiza stabilności (5 nóg na ziemi)
 * - Szacowany czas cyklu i prędkość robota
 * - Opis algorytmu sekwencyjnego
 * - Sekwencja nóg (1→2→3→4→5→6)
 * - Porównanie z innymi algorytmami
 * - Zalecane zastosowania
 *
 * **Przykład output:**
 * ```
 * === KONFIGURACJA WAVE GAIT ===
 * Długość kroku: 4.0 cm
 * Wysokość podniesienia: 4.0 cm
 * Czas swing: 10 ms
 * Punkty interpolacji: 50
 * ALGORYTM: WAVE (sekwencyjny swing + stance shift o 1/6)
 * SEKWENCJA: 1→2→3→4→5→6 (jedna noga na raz)
 * STABILNOŚĆ: Najwyższa (zawsze 5 nóg na ziemi)
 * ===============================
 * ```
 *
 * @note Funkcja tylko wyświetla - nie modyfikuje konfiguracji
 * @note Output przez printf() (UART2 w tym projekcie)
 * @note Przydatna do dokumentowania testów i eksperymentów
 *
 * @code{.c}
 * // Sprawdzenie przed testem stabilności
 * printf("=== DIAGNOSTYKA WAVE ===\n");
 * printWaveConfig();
 * printf("========================\n");
 * @endcode
 *
 * @see setWaveConfig() dla modyfikacji ustawień
 */
void printWaveConfig(void);

/** @} */ // end of Wave_Functions

/**
 * @defgroup Wave_Implementation Szczegóły implementacji
 * @{
 */

/**
 * @brief Sekwencja wave - tablica kolejności nóg
 *
 * @details
 * Wewnętrzna stała definiująca kolejność ruchu nóg w wave gait.
 * Zaprojektowana dla maksymalnej stabilności geometrycznej.
 *
 * ```c
 * static const int wave_sequence[6] = {1, 2, 3, 4, 5, 6};
 * ```
 *
 * **Alternatywne sekwencje (nie implementowane):**
 * - **Optimized:** {1, 4, 2, 5, 3, 6} - alternatywnie lewe/prawe
 * - **Diagonal:** {1, 6, 3, 4, 2, 5} - diagonalny pattern
 * - **Reverse:** {6, 5, 4, 3, 2, 1} - dla ruchu do tyłu
 *



/**
 * @brief Stance shift o 1/6 kroku
 *
 * @details
 * Kluczowa różnica wave vs inne gaits - najmniejszy stance shift.
 *
 * **Porównanie stance shift:**
 * - **Wave**: 1/6 step_length (najbardziej płynny)
 * - **Bipedal**: 1/3 step_length (średni)
 * - **Tripod**: 1/2 step_length (największy skok)
 *
 * **Matematyka dla step_length = 4.0cm:**
 * ```
 * Wave stance_shift = 4.0cm / 6 = 0.67cm per step
 * Bipedal stance_shift = 4.0cm / 3 = 1.33cm per step
 * Tripod stance_shift = 4.0cm / 2 = 2.0cm per step
 * ```
 *
 * **Efekt na stabilność:**
 * - Najmniejsze przyspieszenia centrum masy
 * - Minimalne siły inercyjne
 * - Płynne transfery obciążeń
 * - Najwyższa kontrola pozycji
 */

/** @} */ // end of Wave_Implementation

#endif // WAVE_GAIT_H