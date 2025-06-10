/**
 * @file hexapod_kinematics.h
 * @brief Kinematyka odwrotna dla 6-nożnego robota hexapod
 *
 * @details
 * System kinematyki odwrotnej (Inverse Kinematics) dla hexapoda bazujący
 * na sprawdzonych parametrach z systemu ROS. Każda noga ma 3 stopnie swobody:
 * - **Hip (biodro)**: Obrót wokół osi Z (yaw)
 * - **Knee (kolano)**: Obrót wokół osi Y (pitch)
 * - **Ankle (kostka)**: Obrót wokół osi Y (pitch)
 *
 * @section coordinate_system Układ współrzędnych
 *
 * **Globalny układ robota:**
 * - **X**: Do przodu (dodatnie) / Do tyłu (ujemne)
 * - **Y**: W lewo (dodatnie) / W prawo (ujemne)
 * - **Z**: Do góry (dodatnie) / Do dołu (ujemne)
 * - **Pochodzenie**: Środek korpusu robota
 *
 * **Lokalny układ nogi:**
 * - Transformacja przez pozycję origin każdej nogi
 * - Uwzględnienie inwersji dla prawych nóg
 *
 * @section leg_geometry Geometria nóg
 *
 * **Długości segmentów (sprawdzone z ROS):**
 * - **L1**: 5.5 cm (biodro - od osi obrotu do kolana)
 * - **L2**: 12.5 cm (udo - od kolana do kostki)
 * - **L3**: 15.5 cm (podudzie - od kostki do stopy)
 * - **Zasięg maksymalny**: L2 + L3 = 28.0 cm
 * - **Zasięg minimalny**: |L2 - L3| = 3.0 cm
 *
 * @section positions_base Pozycje bazowe nóg
 *
 * Pozycje stóp w pozycji stojącej (Z = -24cm):
 *
 * | Noga | Pozycja | X [cm] | Y [cm] | Z [cm] | Origin X | Origin Y |
 * |------|---------|--------|--------|--------|----------|----------|
 * | 1 | Lewa przednia | 18.0 | -15.0 | -24.0 | 6.90 | -7.71 |
 * | 2 | Prawa przednia | -18.0 | -15.0 | -24.0 | -8.66 | -7.71 |
 * | 3 | Lewa środkowa | 22.0 | 0.0 | -24.0 | 10.12 | 0.06 |
 * | 4 | Prawa środkowa | -22.0 | 0.0 | -24.0 | -11.88 | -0.06 |
 * | 5 | Lewa tylna | 18.0 | 15.0 | -24.0 | 6.90 | 7.84 |
 * | 6 | Prawa tylna | -18.0 | 15.0 | -24.0 | -8.66 | 7.84 |
 *
 * @section inverse_kinematics Algorytm kinematyki odwrotnej
 *
 * **Kroki obliczenia dla punktu (x, y, z):**
 *
 * 1. **Transformacja do układu lokalnego:**
 *    ```
 *    local_x = x - origin_x
 *    local_y = y - origin_y
 *    ```
 *
 * 2. **Kąt biodra (q1):**
 *    ```
 *    q1 = atan2(local_y, local_x)
 *    if (invert_hip) q1 = q1 ± π
 *    ```
 *
 * 3. **Odległość radialna:**
 *    ```
 *    r = sqrt(local_x² + local_y²) - L1
 *    h = -z  (inwersja osi Z)
 *    D = sqrt(r² + h²)
 *    ```
 *
 * 4. **Sprawdzenie zasięgu:**
 *    ```
 *    if (D > L2 + L3) return FAIL  // Za daleko
 *    if (D < |L2 - L3|) return FAIL  // Za blisko
 *    ```
 *
 * 5. **Kąty kolana i kostki:**
 *    ```
 *    γ = acos((D² - L2² - L3²) / (2×L2×L3))  // Kąt między L2 i L3
 *    α = atan2(h, r)                         // Kąt do celu
 *    β = acos((D² + L2² - L3²) / (2×L2×D))   // Kąt w trójkącie
 *
 *    q2 = -(α - β)      // Kąt kolana
 *    q3 = γ - π (lewa)  // Kąt kostki
 *    q3 = -(π - γ) (prawa)
 *    ```
 *
 * @section error_handling Obsługa błędów
 *
 * **Typowe przyczyny failowania IK:**
 * - Punkt poza zasięgiem nogi (D > 28cm lub D < 3cm)
 * - Nieprawidłowe parametry wejściowe (NULL pointers)
 * - Numeryczne problemy (dzielenie przez zero, acos poza [-1,1])
 *
 * **Debugging:**
 * - Użyj debugLegIK() do analizy konkretnej pozycji
 * - testAllBasePositions() weryfikuje wszystkie pozycje bazowe
 *
 * @author Hexapod Project Team
 * @date 2025
 * @version 1.0
 *
 * @see ROS hexapod_description package - źródło parametrów
 * @see https://en.wikipedia.org/wiki/Inverse_kinematics - teoria IK
 * @see https://kcir.pwr.edu.pl/~mucha/Pracki/Rafal_Gierczak_praca_magisterska.pdf - praca magisterska o podobnej tematyce
 */

#ifndef HEXAPOD_KINEMATICS_H
#define HEXAPOD_KINEMATICS_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

/**
 * @defgroup Kinematics_Constants Stałe kinematyczne
 * @brief Parametry geometryczne robota z systemu ROS
 * @{
 */

/**
 * @brief Długości segmentów nóg [cm]
 *
 * @details
 * Parametry zweryfikowane w symulacji ROS i rzeczywistym robocie.
 * Bazują na specyfikacji mechanicznej hexapoda z servami MG996R.
 *
 * **Kalibracja:**
 * - Zmierzone z modelu CAD
 * - Potwierdzone testami kinematyki w ROS Gazebo
 * - Dopasowane do rzeczywistych wymiarów serw
 */
///@{
#define L1 5.5f  ///< Długość biodra [cm] - od osi obrotu biodra do osi kolana
#define L2 12.5f ///< Długość uda [cm] - od osi kolana do osi kostki
#define L3 15.5f ///< Długość podudzia [cm] - od osi kostki do końca stopy
///@}

/** @} */ // end of Kinematics_Constants

/**
 * @defgroup Kinematics_Types Struktury i typy danych
 * @{
 */

/**
 * @brief Struktura pozycji origin pojedynczej nogi
 *
 * @details
 * Każda noga ma inną pozycję punktu obrotu biodra (origin) względem
 * centrum robota. Dodatkowo prawe nogi mają odwrócone kierunki obrotu.
 *
 * **Pochodzenie danych:**
 * - Wyeksportowane z URDF (Unified Robot Description Format)
 * - Przeskalowane z metrów na centymetry (*100)
 * - Potwierdzone w symulacji ROS
 */
typedef struct
{
    float x;          ///< Pozycja X origin względem centrum robota [cm]
    float y;          ///< Pozycja Y origin względem centrum robota [cm]
    bool invert_hip;  ///< Czy inwertować kąt biodra (prawe nogi)
    bool invert_knee; ///< Czy inwertować kąt kolana (prawe nogi)
} LegOrigin_t;

/**
 * @brief Struktura reprezentująca pozycję 3D
 */
typedef struct
{
    float x; ///< Współrzędna X [cm]
    float y; ///< Współrzędna Y [cm]
    float z; ///< Współrzędna Z [cm]
} Position3D_t;

/**
 * @brief Struktura kątów stawów nogi
 */
typedef struct
{
    float hip;   ///< Kąt biodra [radiany]
    float knee;  ///< Kąt kolana [radiany]
    float ankle; ///< Kąt kostki [radiany]
} JointAngles_t;

/**
 * @defgroup Kinematics_Data Dane konfiguracyjne
 * @{
 */

/**
 * @brief Tablica pozycji origin wszystkich 6 nóg
 *
 * @details
 * Indeks tablicy = numer_nogi - 1 (nogi numerowane 1-6).
 *
 * **Układ nóg (widok z góry):**
 * ```
 *   1 ●────● 2    Przód
 *     │    │
 *   3 ●    ● 4    Środek
 *     │    │
 *   5 ●────● 6    Tył
 * ```
 *
 * **Konwencja inwersji:**
 * - **Lewe nogi (1,3,5)**: invert_hip=false, invert_knee=false
 * - **Prawe nogi (2,4,6)**: invert_hip=true, invert_knee=true
 */
extern const LegOrigin_t leg_origins[6];

/** @} */ // end of Kinematics_Data

/**
 * @defgroup Kinematics_Functions Funkcje publiczne API
 * @{
 */

/**
 * @brief Oblicz kinematykę odwrotną dla pojedynczej nogi
 *
 * @details
 * Główna funkcja systemu kinematycznego. Konwertuje zadaną pozycję 3D
 * końcówki nogi (stopy) na kąty trzech stawów: biodra, kolana i kostki.
 *
 * **Algorytm:**
 * 1. Sprawdzenie poprawności parametrów
 * 2. Transformacja do układu lokalnego nogi
 * 3. Obliczenie kąta biodra (atan2)
 * 4. Sprawdzenie zasięgu (geometryczne ograniczenia)
 * 5. Rozwiązanie trójkąta dla kolan i kostki (prawo cosinusów)
 * 6. Uwzględnienie inwersji dla prawych nóg
 *
 * **Ograniczenia zasięgu:**
 * - Maksimum: 28.0 cm (L2 + L3)
 * - Minimum: 3.0 cm (|L2 - L3|)
 * - Sprawdzenie przed obliczeniami matematycznymi
 *
 * **Obsługa inwersji:**
 * - Prawe nogi mają odwrócone kierunki obrotu
 * - Automatyczne uwzględnienie w obliczeniach
 * - Bazuje na flagach invert_hip i invert_knee
 *
 * @param[in] leg_number Numer nogi (1-6)
 * @param[in] x Pozycja X końcówki nogi [cm]
 * @param[in] y Pozycja Y końcówki nogi [cm]
 * @param[in] z Pozycja Z końcówki nogi [cm]
 * @param[out] q1 Kąt biodra [radiany]
 * @param[out] q2 Kąt kolana [radiany]
 * @param[out] q3 Kąt kostki [radiany]
 *
 * @return true Kinematyka obliczona pomyślnie
 * @return false Błąd (punkt poza zasięgiem lub nieprawidłowe parametry)
 *
 * @note Funkcja drukuje szczegółowe informacje debug przez printf()
 * @warning Kąty są w radianach! Użyj RAD_TO_DEG dla konwersji na stopnie
 *
 * @code{.c}
 * // Przykład użycia - noga 1 w pozycji bazowej
 * float hip, knee, ankle;
 * if (computeLegIK(1, 18.0f, -15.0f, -24.0f, &hip, &knee, &ankle)) {
 *     printf("Kąty: %.1f°, %.1f°, %.1f°\n",
 *            hip * RAD_TO_DEG, knee * RAD_TO_DEG, ankle * RAD_TO_DEG);
 * }
 * @endcode
 *
 * @see debugLegIK() dla szczegółowej analizy obliczeń
 * @see leg_origins tablica pozycji origin nóg
 */
bool computeLegIK(int leg_number, float x, float y, float z,
                  float *q1, float *q2, float *q3);

/**
 * @brief Szczegółowa analiza kinematyki odwrotnej z debugiem
 *
 * @details
 * Rozszerzona wersja computeLegIK() z kompletnym debugowaniem.
 * Wypisuje każdy krok obliczeń, sprawdza ograniczenia zasięgu
 * i identyfikuje przyczyny ewentualnych błędów.
 *
 * **Informacje debug:**
 * - Pozycje origin i lokalne współrzędne
 * - Odległości radialne i sprawdzenie zasięgu
 * - Parametry geometryczne (L1, L2, L3)
 * - Kąty pośrednie (α, β, γ)
 * - Końcowe kąty stawów w stopniach
 *
 * **Analiza błędów:**
 * - "Za daleko" - punkt poza maksymalnym zasięgiem
 * - "Za blisko" - punkt w martwej strefie centralnej
 * - Problemy numeryczne w acos()
 *
 * @param[in] leg_number Numer nogi (1-6)
 * @param[in] x Pozycja X końcówki nogi [cm]
 * @param[in] y Pozycja Y końcówki nogi [cm]
 * @param[in] z Pozycja Z końcówki nogi [cm]
 *
 * @return true Kinematyka wykonalna
 * @return false Kinematyka niewykonalna (z wyjaśnieniem przyczyny)
 *
 * @note Funkcja tylko analizuje - nie zwraca kątów!
 * @note Obszerny output - używaj dla diagnostyki, nie w pętlach
 *
 * @code{.c}
 * // Analiza problematycznej pozycji
 * if (!debugLegIK(3, 30.0f, 0.0f, -20.0f)) {
 *     printf("Pozycja nieosiągalna - sprawdź logi powyżej\n");
 * }
 * @endcode
 *
 * @see computeLegIK() dla normalnych obliczeń IK
 */
bool debugLegIK(int leg_number, float x, float y, float z);

/**
 * @brief Test wszystkich pozycji bazowych oraz z krokami
 *
 * @details
 * Kompleksowa funkcja testująca kinematykę dla wszystkich 6 nóg
 * w trzech scenariuszach:
 * 1. **Pozycja bazowa** - standardowa pozycja stojąca
 * 2. **Krok do przodu** - pozycja + 4cm w kierunku Y-
 * 3. **Krok do tyłu** - pozycja - 4cm w kierunku Y+
 *
 * **Raportowanie:**
 * - Szczegółowe logi dla każdej nogi i pozycji
 * - Podsumowanie PASSED/FAILED na końcu
 * - Rekomendacje w przypadku błędów
 *
 * **Zastosowanie:**
 * - Weryfikacja po zmianach parametrów kinematycznych
 * - Sprawdzenie przed uruchomieniem chodów
 * - Diagnostyka problemów z zasięgiem
 *
 * @note Funkcja nie przyjmuje parametrów - testuje predefiniowane pozycje
 * @note Długość trwania: ~2-3 sekundy (18 testów IK + logi)
 *
 * @code{.c}
 * // Test przed startem systemu
 * void startup_diagnostics() {
 *     printf("=== DIAGNOSTYKA KINEMATYKI ===\n");
 *     testAllBasePositions();
 *     printf("===============================\n");
 * }
 * @endcode
 *
 * @see debugLegIK() używana wewnętrznie dla każdej pozycji
 */
void testAllBasePositions(void);

/** @} */ // end of Kinematics_Functions
#endif    // HEXAPOD_KINEMATICS_H