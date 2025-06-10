/**
 * @file tripod_gait.h
 * @brief Algorytm chodu tripod dla robota hexapod
 *
 * @details
 * Implementacja najszybszego i najczęściej używanego chodu dla robotów 6-nożnych.
 * Tripod gait charakteryzuje się ruchem w dwóch grupach nóg na przemian,
 * zapewniając stabilny trójkąt podparcia przy wysokiej prędkości ruchu.
 *
 * @section tripod_concept Koncepcja tripod gait
 *
 * **Podział nóg na grupy:**
 * ```
 * Grupa A: 1, 4, 5    Grupa B: 2, 3, 6
 *
 *   1A●────●2B  ← Przód
 *     │    │
 *   3B●    ●4A  ← Środek
 *     │    │
 *   5A●────●6B  ← Tył
 * ```
 *
 * **Sekwencja ruchu:**
 * 1. **Faza 1**: Grupa A robi SWING (krok) + Grupa B robi STANCE (podtrzymuje)
 * 2. **Faza 2**: Grupa B robi SWING (krok) + Grupa A robi STANCE (podtrzymuje)
 *
 * **Zalety:**
 * - ⚡ **Najszybszy** chód dla hexapoda
 * -  **Stabilny trójkąt** podparcia (zawsze 3 nogi na ziemi)
 * -  **Prosta implementacja** (tylko 2 fazy)
 * -  **Mocne podparcie** w każdej chwili
 *
 * **Wady:**
 * -  Mniej stabilny niż wave gait (tylko 3 nogi na ziemi)
 * -  Problemy na nierównym terenie
 * -  Wymaga precyzyjnej synchronizacji
 *
 * @section movement_types Typy ruchu
 *
 * | Kierunek | Opis | Modyfikacja nóg |
 * |----------|------|-----------------|
 * | **FORWARD** | Do przodu | Y -= step_length |
 * | **BACKWARD** | Do tyłu | Y += step_length |
 * | **LEFT** | W lewo (sideways) | X += step_length |
 * | **RIGHT** | W prawo (sideways) | X -= step_length |
 * | **TURN_LEFT** | Obrót w lewo | Przednie→lewo, Tylne→prawo |
 * | **TURN_RIGHT** | Obrót w prawo | Przednie→prawo, Tylne→lewo |
 *
 * @section offset_system System offsetów bioder
 *
 * **Offsety z analizy URDF (ograniczenia mechaniczne):**
 *
 * | Noga | Pozycja | Offset biodra | Powód |
 * |------|---------|---------------|--------|
 * | 1 | Lewa przednia | **+37.5°** | Ograniczony ruch do tyłu |
 * | 2 | Prawa przednia | **-37.5°** | Ograniczony ruch do tyłu |
 * | 3 | Lewa środkowa | **0°** | Pełny zakres ruchu |
 * | 4 | Prawa środkowa | **0°** | Pełny zakres ruchu |
 * | 5 | Lewa tylna | **-37.5°** | Ograniczony ruch do przodu |
 * | 6 | Prawa tylna | **+37.5°** | Ograniczony ruch do przodu |
 *
 *
 * @section hardware_mapping Mapowanie sprzętowe
 *
 * **Kontrolery I2C:**
 * - **I2C1 (pca1)**: Lewe nogi 1,3,5 (adres 0x40)
 * - **I2C2 (pca2)**: Prawe nogi 2,4,6 (adres 0x40)
 *
 * **Mapowanie kanałów PCA9685:**
 * | Noga | I2C | Kanały | Hip | Knee | Ankle |
 * |------|-----|--------|-----|------|-------|
 * | 1 | I2C1 | 0-2 | 0 | 1 | 2 |
 * | 2 | I2C2 | 0-2 | 0 | 1 | 2 |
 * | 3 | I2C1 | 3-5 | 3 | 4 | 5 |
 * | 4 | I2C2 | 3-5 | 3 | 4 | 5 |
 * | 5 | I2C1 | 6-8 | 6 | 7 | 8 |
 * | 6 | I2C2 | 6-8 | 6 | 7 | 8 |
 *
 * @author Maksymilian Tulewicz
 * @date 2025
 * @version 1.0
 *
 * @see hexapod_kinematics.h dla obliczeń kinematyki odwrotnej
 * @see pca9685.h dla sterowania serwomechanizmami
 */
#ifndef TRIPOD_GAIT_H
#define TRIPOD_GAIT_H

#include "hexapod_kinematics.h"
#include "test_positions.h"
#include "pca9685.h"
#include "stm32f4xx_hal.h"

/**
 * @defgroup Tripod_Types Typy i enumeracje
 * @{
 */

/**
 * @brief Kierunki ruchu w tripod gait
 *
 * @details
 * Enum definiujący wszystkie możliwe kierunki poruszania się robota
 * w trybie tripod gait. Każdy kierunek ma specyficzny algorytm
 * modyfikacji pozycji nóg.
 */
typedef enum
{
    TRIPOD_FORWARD,   ///< Ruch do przodu (Y -= step_length)
    TRIPOD_BACKWARD,  ///< Ruch do tyłu (Y += step_length)
    TRIPOD_LEFT,      ///< Ruch w lewo - sideways (X += step_length)
    TRIPOD_RIGHT,     ///< Ruch w prawo - sideways (X -= step_length)
    TRIPOD_TURN_LEFT, ///< Obrót w lewo - przednie nogi w lewo, tylne w prawo
    TRIPOD_TURN_RIGHT ///< Obrót w prawo - przednie nogi w prawo, tylne w lewo
} TripodDirection_t;

/**
 * @brief Konfiguracja parametrów tripod gait
 *
 * @details
 * Struktura zawierająca wszystkie konfigurowalne parametry algorytmu.
 * Pozwala na dostrajanie prędkości, płynności i charakterystyki ruchu
 * w runtime bez rekompilacji kodu.
 *
 * **Zalecane zakresy wartości:**
 * - step_length: 2.0-8.0 cm (ograniczone kinetyką)
 * - lift_height: 2.0-6.0 cm (ograniczone clearance)
 * - duration: 50-500 ms (kompromis prędkość/stabilność)
 * - points: 10-200 (kompromis płynność/wydajność)
 */
typedef struct
{
    float step_length;           ///< Długość kroku [cm] - dystans przesunięcia nogi
    float lift_height;           ///< Wysokość podniesienia nogi [cm] - clearance nad ziemią
    uint32_t swing_duration_ms;  ///< Czas swing phase [ms] - czas ruchu nogi w powietrzu
    uint32_t stance_duration_ms; ///< Czas stance phase [ms] - czas podtrzymywania robota
    int swing_points;            ///< Punkty interpolacji swing phase - więcej = płynniej
    int stance_points;           ///< Punkty interpolacji stance phase - więcej = płynniej
    float step_height_base;      ///< Bazowa wysokość stania [cm] - pozycja Z w stance
} TripodConfig_t;

/** @} */ // end of Tripod_Types

/**
 * @defgroup Tripod_Config Konfiguracja i stałe
 * @{
 */

/**
 * @brief Globalna konfiguracja tripod gait
 *
 * @details
 * Zewnętrzna zmienna globalna przechowująca aktualną konfigurację.
 * Inicjalizowana domyślnymi wartościami bezpiecznymi, ale może być
 * modyfikowana w runtime przez setTripodConfig().
 *
 * **Wartości domyślne (BEZPIECZNE):**
 * - step_length: 6.0 cm
 * - lift_height: 4.0 cm
 * - swing_duration: 150 ms
 * - stance_duration: 150 ms
 * - swing_points: 150
 * - stance_points: 150
 */
extern TripodConfig_t tripod_config;

/** @} */ // end of Tripod_Config

/**
 * @defgroup Tripod_Functions Funkcje publiczne API
 * @{
 */

/**
 * @brief Wykonaj jeden pełny cykl tripod gait
 *
 * @details
 * Główna funkcja algorytmu tripod gait. Wykonuje jeden kompletny cykl
 * składający się z dwóch faz:
 *
 * **Faza 1 (150ms + interpolacja):**
 * - Grupa A (1,4,5): SWING - ruch nóg w powietrzu
 * - Grupa B (2,3,6): STANCE - podtrzymywanie robota na ziemi
 *
 * **Faza 2 (150ms + interpolacja):**
 * - Grupa A (1,4,5): STANCE - podtrzymywanie robota na ziemi
 * - Grupa B (2,3,6): SWING - ruch nóg w powietrzu
 *
 * **Algorytm SWING (noga w powietrzu):**
 * 1. Podniesienie nogi z pozycji tylnej
 * 2. Łukowa trajektoria nad ziemią (lift_height)
 * 3. Opuszczenie nogi w pozycji przedniej
 *
 * **Algorytm STANCE (noga na ziemi):**
 * 1. Płynne przesuwanie nogi po ziemi
 * 2. Z pozycji przedniej do pozycji tylnej
 * 3. Synchronicznie z całą grupą
 *
 * **Ultra szybka optymalizacja:**
 * - Używa 30 punktów zamiast 150 (5x szybciej)
 * - Brak HAL_Delay() w pętlach interpolacji
 * - Całkowity czas cyklu: ~120ms zamiast 300ms
 *
 * @param[in] pca1 Wskaźnik na kontroler PCA9685 lewych nóg (I2C1) lub NULL
 * @param[in] pca2 Wskaźnik na kontroler PCA9685 prawych nóg (I2C2) lub NULL
 * @param[in] direction Kierunek ruchu (TRIPOD_FORWARD, TRIPOD_BACKWARD, etc.)
 *
 * @return true Cykl wykonany pomyślnie
 * @return false Błąd podczas wykonywania (problemy I2C, kinematyka)
 *
 * @note Funkcja automatycznie sprawdza czy kontrolery są dostępne
 * @note W przypadku NULL dla pca1/pca2 - odpowiednie nogi są pomijane
 * @warning Upewnij się, że robot jest w pozycji stojącej przed pierwszym wywołaniem
 *
 * @code{.c}
 * // Jeden cykl tripod do przodu
 * if (!tripodGaitCycle(&pca1, &pca2, TRIPOD_FORWARD)) {
 *     printf("Błąd wykonania cyklu tripod!\n");
 * }
 *
 * // Obrót w lewo (w miejscu)
 * tripodGaitCycle(&pca1, &pca2, TRIPOD_TURN_LEFT);
 * @endcode
 *
 * @see setTripodConfig() dla dostrajania parametrów
 * @see tripodGaitWalk() dla ciągłego chodzenia
 */
bool tripodGaitCycle(PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2, TripodDirection_t direction);

/**
 * @brief Wykonaj ciągłe chodzenie tripod gait (wiele cykli)
 *
 * @details
 * Funkcja wysokiego poziomu wykonująca określoną liczbę cykli tripod gait
 * z automatyczną obsługą błędów i pauzami między cyklami.
 *
 * **Sekwencja działania:**
 * 1. Wyświetlenie informacji o konfiguracji
 * 2. Sprawdzenie statusu kontrolerów I2C
 * 3. Wykonanie num_cycles cykli tripod
 * 4. Pauza 100ms między cyklami (dla stabilności)
 * 5. Raportowanie czasu wykonania
 *
 * **Obsługa błędów:**
 * - Stop przy pierwszym błędzie cyklu
 * - Szczegółowe logi o przyczynie błędu
 * - Bezpieczne zatrzymanie w ostatniej pozycji
 *
 * @param[in] pca1 Wskaźnik na kontroler PCA9685 lewych nóg (I2C1)
 * @param[in] pca2 Wskaźnik na kontroler PCA9685 prawych nóg (I2C2)
 * @param[in] direction Kierunek ruchu przez wszystkie cykle
 * @param[in] num_cycles Liczba cykli do wykonania (1-100 zalecane)
 *
 * @return true Wszystkie cykle wykonane pomyślnie
 * @return false Błąd w którymś z cykli (szczegóły w logach)
 *
 * @note Funkcja blokuje wykonanie na czas: num_cycles × (~120ms + 100ms)
 * @note Dla num_cycles=5: ~1.1 sekundy całkowity czas
 * @warning Duża liczba cykli może prowadzić do przegrzania serw
 *
 * @code{.c}
 * // Krótki spacer do przodu (5 cykli = ~1m)
 * tripodGaitWalk(&pca1, &pca2, TRIPOD_FORWARD, 5);
 *
 * // Długi spacer (20 cykli = ~4m)
 * if (!tripodGaitWalk(&pca1, &pca2, TRIPOD_FORWARD, 20)) {
 *     printf("Spacer przerwany - sprawdź hardware\n");
 * }
 *
 * // Obrót o ~180° (8 cykli)
 * tripodGaitWalk(&pca1, &pca2, TRIPOD_TURN_LEFT, 8);
 * @endcode
 *
 * @see tripodGaitCycle() wykonywana wewnętrznie
 */
bool tripodGaitWalk(PCA9685_Handle_t *pca1, PCA9685_Handle_t *pca2,
                    TripodDirection_t direction, int num_cycles);

/**
 * @brief Ustaw parametry tripod gait w runtime
 *
 * @details
 * Funkcja konfiguracyjna pozwalająca na dostrajanie wszystkich parametrów
 * algorytmu bez rekompilacji kodu. Nowe ustawienia wchodzą w życie
 * natychmiast dla kolejnych wywołań tripodGaitCycle().
 *
 * **Walidacja parametrów:**
 * - step_length: 1.0-10.0 cm (ograniczenia kinematyczne)
 * - lift_height: 1.0-8.0 cm (clearance i stabilność)
 * - duration: 10-1000 ms (fizyczne ograniczenia serw)
 * - points: 5-500 (kompromis wydajność/płynność)
 *
 * **Efekty zmian:**
 * - **↑ step_length**: Większe kroki = szybsze przemieszczanie
 * - **↑ lift_height**: Wyższa clearance = bezpieczniej na nierównym terenie
 * - **↓ duration**: Szybsze ruchy = mniej stabilne
 * - **↑ points**: Płynniejsze ruchy = wolniejsze wykonanie
 *
 * @param[in] step_length Długość kroku [cm]
 * @param[in] lift_height Wysokość podniesienia [cm]
 * @param[in] swing_duration Czas swing phase [ms]
 * @param[in] stance_duration Czas stance phase [ms]
 * @param[in] swing_points Punkty interpolacji swing
 * @param[in] stance_points Punkty interpolacji stance
 *
 * @note Zmiany są globalne i dotyczą wszystkich kolejnych cykli
 * @note Funkcja automatycznie loguje nową konfigurację
 * @warning Ekstremalne wartości mogą prowadzić do niestabilności
 *
 * @code{.c}
 * // Konfiguracja dla szybkiego ruchu
 * setTripodConfig(8.0f, 3.0f, 80, 80, 50, 50);
 *
 * // Konfiguracja dla precyzyjnych manewrów
 * setTripodConfig(3.0f, 5.0f, 200, 200, 200, 200);
 *
 * // Przywrócenie ustawień domyślnych
 * tripod_config = (TripodConfig_t)TRIPOD_CONFIG_NORMAL;
 * @endcode
 *
 * @see printTripodConfig() dla sprawdzenia aktualnych ustawień
 * @see TRIPOD_CONFIG_* makra dla predefiniowanych konfiguracji
 */
void setTripodConfig(float step_length, float lift_height,
                     uint32_t swing_duration, uint32_t stance_duration,
                     int swing_points, int stance_points);

/**
 * @brief Wyświetl aktualną konfigurację tripod gait
 *
 * @details
 * Funkcja diagnostyczna wypisująca kompletne informacje o aktualnej
 * konfiguracji algorytmu tripod gait. Przydatna do debugowania
 * i weryfikacji ustawień.
 *
 * **Wyświetlane informacje:**
 * - Wszystkie parametry konfiguracyjne
 * - Szacowany czas wykonania cyklu
 * - Teoretyczna prędkość robota
 * - Grupa nóg i offsety bioder
 * - Status algorytmu (ultra szybki/normalny)
 *
 * @note Funkcja tylko wyświetla - nie modyfikuje konfiguracji
 * @note Output kierowany przez printf() (UART2 w tym projekcie)
 *
 * @code{.c}
 * // Sprawdzenie konfiguracji przed testem
 * printf("=== AKTUALNY SETUP ===\n");
 * printTripodConfig();
 * printf("=====================\n");
 * @endcode
 *
 * @see setTripodConfig() dla modyfikacji parametrów
 */
void printTripodConfig(void);

/** @} */ // end of Tripod_Functions

/**
 * @defgroup Tripod_Macros Makra dla szybkiego użycia
 * @{
 */

/**
 * @brief Makra wysokiego poziomu dla typowych ruchów
 *
 * @details
 * Uproszczone interfejsy dla najczęściej używanych ruchów.
 * Automatycznie używają odpowiednich kierunków i liczby kroków.
 */
///@{
#define TRIPOD_WALK_FORWARD(pca1, pca2, steps) tripodGaitWalk(pca1, pca2, TRIPOD_FORWARD, steps)   ///< Spacer do przodu
#define TRIPOD_WALK_BACKWARD(pca1, pca2, steps) tripodGaitWalk(pca1, pca2, TRIPOD_BACKWARD, steps) ///< Spacer do tyłu
#define TRIPOD_TURN_LEFT(pca1, pca2, steps) tripodGaitWalk(pca1, pca2, TRIPOD_TURN_LEFT, steps)    ///< Obrót w lewo
#define TRIPOD_TURN_RIGHT(pca1, pca2, steps) tripodGaitWalk(pca1, pca2, TRIPOD_TURN_RIGHT, steps)  ///< Obrót w prawo
///@}

/** @} */ // end of Tripod_Macros

/**
 * @defgroup Tripod_Advanced Zaawansowane informacje
 * @{
 */

/**
 * @brief Grupy nóg w tripod gait z offsetami bioder
 *
 * @details
 * Podział nóg na dwie grupy tworzące stabilne trójkąty podparcia.
 * Offsety bioder pochodzą z analizy URDF joint limits i zapewniają
 * optymalne wykorzystanie mechanicznego zakresu ruchu każdej nogi.
 *
 * **GRUPA A (swing w fazie 1):**
 * - **Noga 1**: Lewa przednia (offset +37.5°) - ograniczony ruch do tyłu
 * - **Noga 4**: Prawa środkowa (offset 0°) - pełny zakres ruchu
 * - **Noga 5**: Lewa tylna (offset -37.5°) - ograniczony ruch do przodu
 *
 * **GRUPA B (swing w fazie 2):**
 * - **Noga 2**: Prawa przednia (offset -37.5°) - ograniczony ruch do tyłu
 * - **Noga 3**: Lewa środkowa (offset 0°) - pełny zakres ruchu
 * - **Noga 6**: Prawa tylna (offset +37.5°) - ograniczony ruch do przodu
 *
 * **Stabilność trójkątów:**
 * Każda grupa tworzy stabilny trójkąt:
 * - Grupa A: przód-lewy ↔ środek-prawy ↔ tył-lewy
 * - Grupa B: przód-prawy ↔ środek-lewy ↔ tył-prawy
 *
 * **Pochodzenie offsetów:**
 * Offsety pochodzą z analizy URDF joint limits - przednie i tylne nogi
 * mają asymetryczne zakresy ruchu z powodu ograniczeń mechanicznych
 * korpusu robota. Środkowe nogi mają symetryczne zakresy.
 */

/**
 * @brief Optymalizacje wydajności
 *
 * **Ultra szybki mode (aktualny):**
 * - Redukcja punktów interpolacji z 150 do 30 (5x szybciej)
 * - Usunięcie HAL_Delay() z pętli interpolacji (pure speed)
 * - Minimalizacja logów debug w czasie rzeczywistym
 * - Optymalizacja obliczeń matematycznych (cache friendly)
 *
 * **Rezultaty optymalizacji:**
 * - Czas cyklu: 120ms vs 300ms (2.5x szybciej)
 * - Prędkość robota: ~2 cm/s vs 0.8 cm/s
 * - Płynność: nadal akceptowalna dla MG996R
 * - Stabilność: zachowana dzięki tripod geometry
 *
 * **Przełączanie na bezpieczny mode:**
 * ```c
 * setTripodConfig(6.0f, 4.0f, 150, 150, 150, 150); // Bezpieczny
 * vs
 * setTripodConfig(6.0f, 4.0f, 50, 50, 30, 30);     // Ultra szybki
 * ```
 */

/** @} */ // end of Tripod_Advanced

/**
 * @defgroup Tripod_Examples Przykłady użycia
 * @{
 */

/**
 * @example tripod_basic.c
 * Podstawowy przykład tripod gait:
 *
 * @code{.c}
 * #include "tripod_gait.h"
 *
 * PCA9685_Handle_t pca1, pca2;
 *
 * void basic_tripod_demo() {
 *     // Inicjalizacja kontrolerów
 *     PCA9685_Init(&pca1, &hi2c1, PCA9685_ADDRESS_1);
 *     PCA9685_Init(&pca2, &hi2c2, PCA9685_ADDRESS_1);
 *
 *     // Konfiguracja dla środkowej prędkości
 *     setTripodConfig(5.0f, 4.0f, 100, 100, 80, 80);
 *
 *     // Spacer do przodu (10 kroków = ~2m)
 *     TRIPOD_WALK_FORWARD(&pca1, &pca2, 10);
 *
 *     // Obrót w prawo (4 kroki = ~90°)
 *     TRIPOD_TURN_RIGHT(&pca1, &pca2, 4);
 *
 *     // Spacer w lewo (5 kroków)
 *     tripodGaitWalk(&pca1, &pca2, TRIPOD_LEFT, 5);
 * }
 * @endcode
 */

/** @} */ // end of Tripod_Examples

#endif // TRIPOD_GAIT_H