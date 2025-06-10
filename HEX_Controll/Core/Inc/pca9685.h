/**
 * @file pca9685.h
 * @brief Sterownik PCA9685 PWM Controller dla STM32 HAL
 *
 * @details
 * Biblioteka sterownika dla 16-kanałowego kontrolera PWM PCA9685
 * zoptymalizowana dla serwomechanizmów MG996R w projekcie hexapoda.
 *
 * **Kluczowe funkcje:**
 * - Dual I2C support (I2C1 + I2C2)
 * - Sprawdzone wartości PWM dla MG996R
 * - Inicjalizacja bez software reset (stabilność)
 * - Precyzyjne ustawienie częstotliwości 50Hz
 *
 * **Testowane konfiguracje:**
 * - STM32F446RE + 2x PCA9685
 * - 18x serwomechanizmy MG996R
 * - I2C speed: 100kHz (możliwe 400kHz)
 *
 * @section hardware_setup Konfiguracja sprzętowa
 *
 * **Połączenia I2C:**
 * ```
 * PCA9685 #1 (Lewe nogi):
 * - SDA → I2C1_SDA (PB9)
 * - SCL → I2C1_SCL (PB8)
 * - VCC → 3.3V
 * - GND → GND
 * - V+ → 5V (zasilanie serw)
 *
 * PCA9685 #2 (Prawe nogi):
 * - SDA → I2C2_SDA (PB3)
 * - SCL → I2C2_SCL (PB10)
 * - VCC → 3.3V
 * - GND → GND
 * - V+ → 5V (zasilanie serw)
 * ```
 *
 * **Adresy I2C:**
 * - PCA9685 #1: 0x40 (A0-A5 = GND)
 * - PCA9685 #2: 0x40 (A0-A5 = GND, różne magistrale)
 *
 * @section servo_mapping Mapowanie serw
 *
 * | Kanał | Funkcja | Noga | Zakres ruchu |
 * |-------|---------|------|--------------|
 * | 0-2 | Hip/Knee/Ankle | 1 lub 2 | 0-180° |
 * | 3-5 | Hip/Knee/Ankle | 3 lub 4 | 0-180° |
 * | 6-8 | Hip/Knee/Ankle | 5 lub 6 | 0-180° |
 *
 * @section pwm_values Sprawdzone wartości PWM
 *
 * **MG996R Metal Gear Servo:**
 * - **0°**: 110 PWM (1.07ms pulse width)
 * - **90°**: 305 PWM (1.48ms pulse width)
 * - **180°**: 500 PWM (2.44ms pulse width)
 * - **Częstotliwość**: 50Hz (20ms period)
 *
 * @warning Nie używaj software reset! Powoduje niestabilność komunikacji I2C
 *
 * @author Maksymilian Tulewicz
 * @date 2025
 * @version 1.0
 *
 * @see https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf - Datasheet PCA9685
 * @see https://www.towerpro.com.tw/product/mg996r/ - Specyfikacja MG996R
 */

#ifndef PCA9685_H_
#define PCA9685_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup PCA9685_Constants Stałe konfiguracyjne PCA9685
 * @{
 */

/**
 * @brief Adresy I2C kontrolerów PCA9685 (7-bitowe)
 *
 * @details
 * Adresy są automatycznie przesuwane przez HAL (<<1 dla 8-bit address).
 * Oba kontrolery używają tego samego adresu ale różnych magistral I2C.
 */
///@{
#define PCA9685_ADDRESS_1 0x40 ///< Pierwszy PCA9685 (A0-A5 = GND)
#define PCA9685_ADDRESS_2 0x41 ///< Drugi PCA9685 (A0 = VCC, A1-A5 = GND)
///@}

/**
 * @brief Rejestry PCA9685 - kluczowe adresy
 *
 * @details
 * Tylko najważniejsze rejestry używane w implementacji.
 * Pełna mapa rejestrów dostępna w datasheet NXP.
 */
///@{
#define PCA9685_MODE1 0x00	   ///< Rejestr trybu 1 (auto-increment, sleep)
#define PCA9685_PRESCALE 0xFE  ///< Prescaler częstotliwości PWM
#define PCA9685_LED0_ON_L 0x06 ///< Pierwszy rejestr kanału LED0 (ON_L)
///@}

/**
 * @brief Konfiguracja PWM dla serwomechanizmów
 */
///@{
#define PCA9685_PWM_FREQUENCY 50 ///< Standardowa częstotliwość serw: 50Hz
///@}

/**
 * @brief Sprawdzone wartości PWM dla MG996R
 *
 * @details
 * Te wartości zostały empirycznie dobrane i przetestowane
 * dla pełnego zakresu ruchu 180° serwomechanizmów MG996R.
 *
 * **Kalkulacje:**
 * - 50Hz = 20ms period
 * - 4096 steps per period
 * - 1 step = 20ms/4096 = 4.88μs
 * - Servo range: ~1-2.5ms pulse width
 */
///@{
#define SERVO_PWM_MIN 110 ///< 0° (537μs pulse width)
#define SERVO_PWM_MAX 500 ///< 180° (2.44ms pulse width)
#define SERVO_PWM_MID 305 ///< 90° (1.49ms pulse width)
///@}

/** @} */ // end of PCA9685_Constants

/**
 * @defgroup PCA9685_Types Struktury i typy danych
 * @{
 */

/**
 * @brief Struktura handlera PCA9685
 *
 * @details
 * Przechowuje wszystkie niezbędne informacje o kontrolerze PCA9685.
 * Każdy kontroler musi mieć własną instancję tej struktury.
 *
 * @note Struktura musi być zainicjalizowana przed użyciem funkcji PCA9685_Init()
 */
typedef struct
{
	I2C_HandleTypeDef *hi2c; ///< Wskaźnik na handle I2C (np. &hi2c1)
	uint8_t address;		 ///< 7-bitowy adres I2C urządzenia
	bool ready;				 ///< Flaga gotowości (true po poprawnej inicjalizacji)
} PCA9685_Handle_t;

/** @} */ // end of PCA9685_Types

/**
 * @defgroup PCA9685_Functions Funkcje publiczne API
 * @{
 */

/**
 * @brief Inicjalizacja kontrolera PCA9685
 *
 * @details
 * **Sekwencja inicjalizacji (bez software reset!):**
 * 1. Test komunikacji I2C (HAL_I2C_IsDeviceReady)
 * 2. Ustawienie MODE1 = 0x20 (auto-increment enabled)
 * 3. Wejście w tryb sleep dla zmiany prescaler
 * 4. Ustawienie prescaler = 121 (50Hz)
 * 5. Wyjście z trybu sleep
 * 6. Delay 5ms na stabilizację oscylatora
 *
 * **Dlaczego brak software reset?**
 * Software reset (MODE1 bit 7) powoduje problemy z komunikacją I2C
 * w niektórych konfiguracjach STM32. Manual reset jest bezpieczniejszy.
 *
 * @param[in,out] handle Wskaźnik na strukturę handlera (musi być zaalokowana)
 * @param[in] hi2c Wskaźnik na handle I2C HAL (np. &hi2c1, &hi2c2)
 * @param[in] address 7-bitowy adres I2C (PCA9685_ADDRESS_1 lub PCA9685_ADDRESS_2)
 *
 * @return true Inicjalizacja zakończona sukcesem
 * @return false Błąd inicjalizacji (sprawdź połączenia I2C)
 *
 * @warning Upewnij się, że I2C jest poprawnie skonfigurowany przed wywołaniem
 * @warning handle->ready zostanie ustawiony na false w przypadku błędu
 *
 * @code{.c}
 * // Przykład użycia
 * PCA9685_Handle_t pca1;
 * if (!PCA9685_Init(&pca1, &hi2c1, PCA9685_ADDRESS_1)) {
 *     // Obsługa błędu - miganie LED, restart, itp.
 * }
 * @endcode
 */
bool PCA9685_Init(PCA9685_Handle_t *handle, I2C_HandleTypeDef *hi2c, uint8_t address);

/**
 * @brief Ustawienie kąta serwomechanizmu (0-180°)
 *
 * @details
 * Funkcja konwertuje kąt w stopniach na odpowiednią wartość PWM
 * używając sprawdzonych wartości dla MG996R:
 *
 * **Mapowanie kątów:**
 * - 0° → 110 PWM
 * - 90° → 305 PWM
 * - 180° → 500 PWM
 *
 * **Interpolacja liniowa:**
 * ```
 * pwm_value = SERVO_PWM_MIN + (angle/180.0) * (SERVO_PWM_MAX - SERVO_PWM_MIN)
 * ```
 *
 * @param[in] handle Wskaźnik na zainicjalizowany handel PCA9685
 * @param[in] channel Kanał PWM (0-15)
 * @param[in] angle Kąt w stopniach (0.0 - 180.0)
 *
 * @return true Kąt ustawiony pomyślnie
 * @return false Błąd (nieprawidłowy handle, kanał lub komunikacja I2C)
 *
 * @note Kąt jest automatycznie ograniczany do zakresu 0-180°
 * @warning Upewnij się, że serwomechanizm jest zasilany (V+ na PCA9685)
 *
 * @code{.c}
 * // Ustawienie biodra nogi 1 na 45°
 * PCA9685_SetServoAngle(&pca1, 0, 45.0f);
 *
 * // Ustawienie kolana nogi 1 na pozycję neutralną
 * PCA9685_SetServoAngle(&pca1, 1, 90.0f);
 * @endcode
 */
bool PCA9685_SetServoAngle(PCA9685_Handle_t *handle, uint8_t channel, float angle);

/**
 * @brief Test różnych zakresów PWM dla kalibracji serw
 *
 * @details
 * Funkcja testowa pozwalająca na eksperymentalne określenie
 * optymalnych wartości PWM dla konkretnego modelu serwomechanizmu.
 *
 * **Sekwencja testu:**
 * 1. Ustawienie PWM minimum → delay 2s
 * 2. Ustawienie PWM środek → delay 2s
 * 3. Ustawienie PWM maksimum → delay 2s
 *
 * @param[in] handle Wskaźnik na zainicjalizowany handel PCA9685
 * @param[in] channel Kanał PWM (0-15)
 * @param[in] pwm_min Minimalna wartość PWM do testu
 * @param[in] pwm_max Maksymalna wartość PWM do testu
 *
 * @return true Test wykonany pomyślnie
 * @return false Błąd (nieprawidłowy handle, kanał lub komunikacja I2C)
 *
 * @note Funkcja blokuje wykonanie na 6 sekund (3x 2s delay)
 * @warning Używaj ostrożnie! Niewłaściwe wartości PWM mogą uszkodzić serwo
 *
 * @code{.c}
 * // Test szerokiego zakresu dla znajdowania limitów
 * PCA9685_TestPWMRange(&pca1, 0, 100, 600);
 *
 * // Test wąskiego zakresu dla precyzji
 * PCA9685_TestPWMRange(&pca1, 0, 300, 310);
 * @endcode
 */
bool PCA9685_TestPWMRange(PCA9685_Handle_t *handle, uint8_t channel, uint16_t pwm_min, uint16_t pwm_max);

/**
 * @brief Ustawienie surowej wartości PWM (0-4095)
 *
 * @details
 * Niskopoziomowa funkcja bezpośrednio zapisująca wartość PWM do rejestrów.
 * Replikuje sprawdzoną sekwencję z manualnych testów I2C.
 *
 * **Rejestry kanału (każdy kanał = 4 rejestry):**
 * - ON_L: 0x00 (start PWM w cyklu)
 * - ON_H: 0x00
 * - OFF_L: pwm_value & 0xFF (koniec PWM w cyklu)
 * - OFF_H: (pwm_value >> 8) & 0xFF
 *
 * **Kalkulacja czasu pulsu:**
 * ```
 * pulse_width = (pwm_value / 4096) * 20ms
 * ```
 *
 * @param[in] handle Wskaźnik na zainicjalizowany handel PCA9685
 * @param[in] channel Kanał PWM (0-15)
 * @param[in] pwm_value Wartość PWM (0-4095, automatycznie ograniczana)
 *
 * @return true Wartość PWM ustawiona pomyślnie
 * @return false Błąd (nieprawidłowy handle, kanał lub komunikacja I2C)
 *
 * @note Funkcja automatycznie ogranicza pwm_value do maksimum 4095
 * @see PCA9685_SetServoAngle() dla wygodniejszego ustawiania kątów
 *
 * @code{.c}
 * // Bezpośrednie ustawienie PWM (dla zaawansowanych)
 * PCA9685_SetPWM(&pca1, 0, 305);  // ~90° dla MG996R
 *
 * // Wylączenie kanału (brak pulsu)
 * PCA9685_SetPWM(&pca1, 0, 0);
 * @endcode
 */
bool PCA9685_SetPWM(PCA9685_Handle_t *handle, uint8_t channel, uint16_t pwm_value);

/**
 * @brief Całkowite wylączenie kanału PWM
 *
 * @details
 * Ustawia wartość PWM na 0, co skutkuje brakiem pulsu wyjściowego.
 * Przydatne do oszczędzania energii lub "odłączenia" serwomechanizmu.
 *
 * @param[in] handle Wskaźnik na zainicjalizowany handel PCA9685
 * @param[in] channel Kanał PWM (0-15)
 *
 * @return true Kanał wyłączony pomyślnie
 * @return false Błąd komunikacji I2C
 *
 * @note Equivalent: PCA9685_SetPWM(handle, channel, 0)
 * @warning Servo może stracić kontrolę pozycji po wyłączeniu PWM
 *
 * @code{.c}
 * // Wylączenie wszystkich kanałów dla oszczędzania energii
 * for (int i = 0; i < 16; i++) {
 *     PCA9685_SetChannelOff(&pca1, i);
 * }
 * @endcode
 */
bool PCA9685_SetChannelOff(PCA9685_Handle_t *handle, uint8_t channel);

/** @} */ // end of PCA9685_Functions

/**
 * @defgroup PCA9685_Examples Przykłady użycia
 * @{
 */

/**
 * @example pca9685_basic_example.c
 *
 * Podstawowy przykład inicjalizacji i sterowania pojedynczym serwem:
 *
 * @code{.c}
 * #include "pca9685.h"
 *
 * PCA9685_Handle_t pca;
 *
 * void setup_servo_controller() {
 *     // Inicjalizacja kontrolera
 *     if (!PCA9685_Init(&pca, &hi2c1, PCA9685_ADDRESS_1)) {
 *         Error_Handler(); // Błąd inicjalizacji
 *     }
 *
 *     // Test pozycji
 *     PCA9685_SetServoAngle(&pca, 0, 0.0f);    // 0°
 *     HAL_Delay(1000);
 *     PCA9685_SetServoAngle(&pca, 0, 90.0f);   // 90°
 *     HAL_Delay(1000);
 *     PCA9685_SetServoAngle(&pca, 0, 180.0f);  // 180°
 * }
 * @endcode
 */

/**
 * @example pca9685_dual_controller.c
 *
 * Przykład użycia dwóch kontrolerów PCA9685:
 *
 * @code{.c}
 * PCA9685_Handle_t pca_left, pca_right;
 *
 * void setup_dual_controllers() {
 *     // Inicjalizacja lewego kontrolera (I2C1)
 *     if (!PCA9685_Init(&pca_left, &hi2c1, PCA9685_ADDRESS_1)) {
 *         Error_Handler();
 *     }
 *
 *     // Inicjalizacja prawego kontrolera (I2C2)
 *     if (!PCA9685_Init(&pca_right, &hi2c2, PCA9685_ADDRESS_1)) {
 *         Error_Handler();
 *     }
 *
 *     // Synchroniczne ustawienie serw
 *     PCA9685_SetServoAngle(&pca_left, 0, 45.0f);   // Lewa noga 1
 *     PCA9685_SetServoAngle(&pca_right, 0, 135.0f); // Prawa noga 2
 * }
 * @endcode
 */

/** @} */ // end of PCA9685_Examples

#endif /* PCA9685_H_ */