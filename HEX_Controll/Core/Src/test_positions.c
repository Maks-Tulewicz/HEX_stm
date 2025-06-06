/*
 * test_positions.c - Testy i kalibracja hexapoda
 * Zaczynamy od zera z działającą kinematyką z ROS
 */

#include "test_positions.h"

// Funkcja do ustawienia kątów stawów nogi na serwa
void setLegJoints(PCA9685_Handle_t *pca, int leg_number, float q1, float q2, float q3)
{
    // Mapowanie kanałów dla nóg (I2C1 - nogi lewe)
    uint8_t base_channel;
    if (leg_number == 1)
        base_channel = 0; // Noga 1: kanały 0,1,2
    else if (leg_number == 3)
        base_channel = 3; // Noga 3: kanały 3,4,5
    else if (leg_number == 5)
        base_channel = 6; // Noga 5: kanały 6,7,8
    else
    {
        printf("Noga %d nie jest obsługiwana (tylko 1,3,5)\n", leg_number);
        return;
    }

    // Konwersja radianów na stopnie
    float hip_deg = q1 * 180.0f / M_PI;
    float knee_deg = q2 * 180.0f / M_PI;
    float ankle_deg = q3 * 180.0f / M_PI;

    // TYMCZASOWE mapowanie - do kalibracji
    // Zakładamy że 0 rad = 90° servo (pozycja neutralna)
    float servo_hip = 90.0f + hip_deg;
    float servo_knee = 90.0f + knee_deg;
    float servo_ankle = 90.0f + ankle_deg;

    // Ograniczenia serw
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

    printf("Noga %d - IK angles [deg]: hip=%.1f, knee=%.1f, ankle=%.1f\n",
           leg_number, hip_deg, knee_deg, ankle_deg);
    printf("Noga %d - Servo angles [deg]: hip=%.1f, knee=%.1f, ankle=%.1f\n",
           leg_number, servo_hip, servo_knee, servo_ankle);
    printf("Noga %d - Kanały: %d, %d, %d\n",
           leg_number, base_channel, base_channel + 1, base_channel + 2);

    // Ustaw serwa
    PCA9685_SetServoAngle(pca, base_channel + 0, servo_hip);
    PCA9685_SetServoAngle(pca, base_channel + 1, servo_knee);
    PCA9685_SetServoAngle(pca, base_channel + 2, servo_ankle);
}

// Test kalibracyjny - wszystkie serwa na 90°
void calibrationTest90Degrees(PCA9685_Handle_t *pca, int leg_number)
{
    printf("\n"
           "================================================================================\n"
           "=== KALIBRACJA - WSZYSTKIE SERWA NA 90° ===\n"
           "================================================================================\n");

    printf("NOGA %d - POZYCJA KALIBRACYJNA\n", leg_number);
    printf("Wszystkie serwa ustawione na 90° (pozycja neutralna)\n\n");

    // Mapowanie kanałów
    uint8_t base_channel;
    if (leg_number == 1)
        base_channel = 0;
    else if (leg_number == 3)
        base_channel = 3;
    else if (leg_number == 5)
        base_channel = 6;
    else
    {
        printf("Noga %d nie obsługiwana!\n", leg_number);
        return;
    }

    printf("Kanały serw: %d (hip), %d (knee), %d (ankle)\n",
           base_channel, base_channel + 1, base_channel + 2);

    // Ustaw wszystkie na 90°
    printf("\nUstawiam wszystkie serwa na 90°...\n");
    PCA9685_SetServoAngle(pca, base_channel + 0, 90.0f); // Hip
    PCA9685_SetServoAngle(pca, base_channel + 1, 90.0f); // Knee
    PCA9685_SetServoAngle(pca, base_channel + 2, 90.0f); // Ankle

    printf("✓ Serwa ustawione!\n");
    printf("\nSPRAWDŹ POZYCJĘ NOGI:\n");
    printf("1. Czy wszystkie serwa są w pozycji środkowej?\n");
    printf("2. Zmierz odległość końcówki stopy od osi obrotu biodra\n");
    printf("3. Zapisz pomiar: _____ cm\n");
    printf("4. Czy pozycja wygląda sensownie mechanicznie?\n");

    printf("\nCzekam 15 sekund na sprawdzenie...\n");
    HAL_Delay(15000);
}

// Test pozycji wyprostowanej nogi obliczonej z IK
void testStraightLegPosition(PCA9685_Handle_t *pca, int leg_number)
{
    printf("\n"
           "================================================================================\n"
           "=== TEST WYPROSTOWANEJ NOGI Z IK ===\n"
           "================================================================================\n");

    printf("NOGA %d - POZYCJA OBLICZONA Z KINEMATYKI ODWROTNEJ\n", leg_number);

    // Pozycja bazowa dla nogi 3 z ROS
    float target_x, target_y, target_z;

    if (leg_number == 3)
    {
        target_x = 22.0f;  // 22cm od centrum
        target_y = 0.0f;   // Prosto w bok (noga środkowa)
        target_z = -24.0f; // 24cm w dół
    }
    else
    {
        printf("Pozycja nie zdefiniowana dla nogi %d\n", leg_number);
        return;
    }

    printf("Pozycja docelowa: x=%.1f, y=%.1f, z=%.1f cm\n", target_x, target_y, target_z);

    float distance = sqrtf(target_x * target_x + target_y * target_y);
    printf("Odległość 2D od centrum: %.1f cm\n", distance);

    // Sprawdź IK
    printf("\nObliczam kinematykę odwrotną...\n");
    float q1, q2, q3;
    bool ik_success = computeLegIK(leg_number, target_x, target_y, target_z, &q1, &q2, &q3);

    if (!ik_success)
    {
        printf("✗ IK FAILED - pozycja poza zasięgiem!\n");
        return;
    }

    printf("✓ IK SUCCESS!\n");
    printf("Obliczone kąty [rad]: hip=%.3f, knee=%.3f, ankle=%.3f\n", q1, q2, q3);
    printf("Obliczone kąty [deg]: hip=%.1f, knee=%.1f, ankle=%.1f\n",
           q1 * 180.0f / M_PI, q2 * 180.0f / M_PI, q3 * 180.0f / M_PI);

    // Ustaw serwa
    printf("\nUstawiam nogę w obliczonej pozycji...\n");
    setLegJoints(pca, leg_number, q1, q2, q3);

    printf("\n✓ NOGA USTAWIONA!\n");
    printf("\nSPRAWDŹ POZYCJĘ:\n");
    printf("1. Zmierz RZECZYWISTĄ odległość końcówki od centrum: _____ cm\n");
    printf("   (powinna być ~%.1f cm)\n", distance);
    printf("2. Czy noga wygląda wyprostowana?\n");
    printf("3. Czy pozycja Y jest prawidłowa (%.1f cm)?\n", target_y);
    printf("4. Czy wysokość Z jest prawidłowa (%.1f cm w dół)?\n", -target_z);

    printf("\nCzekam 15 sekund na pomiary...\n");
    HAL_Delay(15000);

    printf("\nPORÓWNANIE:\n");
    printf("Obliczona odległość: %.1f cm\n", distance);
    printf("Rzeczywista odległość: _____ cm (WPISZ!)\n");
    printf("Różnica: _____ cm\n");

    if (distance > 20.0f)
    {
        printf("UWAGA: Odległość > 20cm może być za duża dla stabilności!\n");
    }
}

// Funkcja pomocnicza - IK + ustawienie serw w jednym
bool legIKToServos(int leg_number, float x, float y, float z, PCA9685_Handle_t *pca)
{
    float q1, q2, q3;
    bool success = computeLegIK(leg_number, x, y, z, &q1, &q2, &q3);

    if (success)
    {
        setLegJoints(pca, leg_number, q1, q2, q3);
        printf("Pozycja (%.1f, %.1f, %.1f) ustawiona dla nogi %d\n", x, y, z, leg_number);
    }
    else
    {
        printf("IK failed dla pozycji (%.1f, %.1f, %.1f) nogi %d\n", x, y, z, leg_number);
    }

    return success;
}