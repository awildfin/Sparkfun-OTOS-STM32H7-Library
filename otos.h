/*!
 * @file otos.h
 * @brief Driver for the SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS) in C.
 * @details This file implements a C driver for interacting with the OTOS sensor.
 * It provides functions for initialization, configuration, reading position,
 * velocity, acceleration, status, as well as self-test and calibration functions.
 * The modular design allows data acquisition in standard units (Meters/Radians)
 * and conversion to other units (CM/Inches/Degrees) via utility functions.
 * @author Dhafin & Bandhayudha
 * @date June 2025
 * @version 1.0.0
 * @copyright Copyright (c) 2025, [Dhafin Putra Madryana / Bandhayudha].
 */
#ifndef OTOS_H
#define OTOS_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>


// --------- DEFINE ---------
// OTOS I2C ADDRESS
/*! @brief Default 7-bit I2C address of the OTOS sensor (0x17), shifted to 8-bit for HAL I2C. */
#define OTOS_I2C_ADDR       (0x17 << 1)

// OTOS REGISTER ADDRESS
/*! @brief Product ID register address. */
#define OTOS_REG_PRODUCT_ID      0x00
/*! @brief Hardware Version register address. */
#define OTOS_REG_HW_VERSION      0x01
/*! @brief Firmware Version register address. */
#define OTOS_REG_FW_VERSION      0x02
/*! @brief Linear Scalar register address. */
#define OTOS_REG_SCALAR_LINEAR   0x04
/*! @brief Angular Scalar register address. */
#define OTOS_REG_SCALAR_ANGULAR  0x05
/*! @brief IMU Calibration register address. */
#define OTOS_REG_IMU_CALIB       0x06
/*! @brief Reset Tracking register address. */
#define OTOS_REG_RESET_TRACK     0x07
/*! @brief Signal Process Configuration register address. */
#define OTOS_REG_SIGNAL_PROCESS  0x0E
/*! @brief Self Test register address. */
#define OTOS_REG_SELF_TEST       0x0F
/*! @brief Offset X Low Byte register address. */
#define OTOS_REG_OFFSET_X_L      0x10
/*! @brief Offset X High Byte register address. */
#define OTOS_REG_OFFSET_X_H      0x11
/*! @brief Offset Y Low Byte register address. */
#define OTOS_REG_OFFSET_Y_L      0x12
/*! @brief Offset Y High Byte register address. */
#define OTOS_REG_OFFSET_Y_H      0x13
/*! @brief Offset Heading Low Byte register address. */
#define OTOS_REG_OFFSET_H_L      0x14
/*! @brief Offset Heading High Byte register address. */
#define OTOS_REG_OFFSET_H_H      0x15
/*! @brief Position X Low Byte register address. */
#define OTOS_REG_POS_X_L         0x20
/*! @brief Position X High Byte register address. */
#define OTOS_REG_POS_X_H         0x21
/*! @brief Position Y Low Byte register address. */
#define OTOS_REG_POS_Y_L         0x22
/*! @brief Position Y High Byte register address. */
#define OTOS_REG_POS_Y_H         0x23
/*! @brief Position Heading Low Byte register address. */
#define OTOS_REG_POS_H_L         0x24
/*! @brief Position Heading High Byte register address. */
#define OTOS_REG_POS_H_H         0x25
/*! @brief Velocity X Low Byte register address. */
#define OTOS_REG_VEL_XL          0x26
/*! @brief Velocity X High Byte register address. */
#define OTOS_REG_VEL_XH          0x27
/*! @brief Velocity Y Low Byte register address. */
#define OTOS_REG_VEL_YL          0x28
/*! @brief Velocity Y High Byte register address. */
#define OTOS_REG_VEL_YH          0x29
/*! @brief Velocity Heading Low Byte register address. */
#define OTOS_REG_VEL_HL          0x2A
/*! @brief Velocity Heading High Byte register address. */
#define OTOS_REG_VEL_HH          0x2B
/*! @brief Acceleration X Low Byte register address. */
#define OTOS_REG_ACC_XL          0x2C
/*! @brief Acceleration X High Byte register address. */
#define OTOS_REG_ACC_XH          0x2D
/*! @brief Acceleration Y Low Byte register address. */
#define OTOS_REG_ACC_YL          0x2E
/*! @brief Acceleration Y High Byte register address. */
#define OTOS_REG_ACC_YH          0x2F
/*! @brief Acceleration Heading Low Byte register address. */
#define OTOS_REG_ACC_HL          0x30
/*! @brief Acceleration Heading High Byte register address. */
#define OTOS_REG_ACC_HH          0x31
/*! @brief Position Standard Deviation X Low Byte register address. */
#define OTOS_REG_POS_STD_XL      0x32
/*! @brief Position Standard Deviation X High Byte register address. */
#define OTOS_REG_POS_STD_XH      0x33
/*! @brief Position Standard Deviation Y Low Byte register address. */
#define OTOS_REG_POS_STD_YL      0x34
/*! @brief Position Standard Deviation Y High Byte register address. */
#define OTOS_REG_POS_STD_YH      0x35
/*! @brief Position Standard Deviation H Low Byte register address. */
#define OTOS_REG_POS_STD_HL      0x36
/*! @brief Position Standard Deviation H High Byte register address. */
#define OTOS_REG_POS_STD_HH      0x37
/*! @brief Velocity Standard Deviation X Low Byte register address. */
#define OTOS_REG_VEL_STD_XL      0x38
/*! @brief Velocity Standard Deviation X High Byte register address. */
#define OTOS_REG_VEL_STD_XH      0x39
/*! @brief Velocity Standard Deviation Y Low Byte register address. */
#define OTOS_REG_VEL_STD_YL      0x3A
/*! @brief Velocity Standard Deviation Y High Byte register address. */
#define OTOS_REG_VEL_STD_YH      0x3B
/*! @brief Velocity Standard Deviation H Low Byte register address. */
#define OTOS_REG_VEL_STD_HL      0x3C
/*! @brief Velocity Standard Deviation H High Byte register address. */
#define OTOS_REG_VEL_STD_HH      0x3D
/*! @brief Acceleration Standard Deviation X Low Byte register address. */
#define OTOS_REG_ACC_STD_XL      0x3E
/*! @brief Alamat register Acceleration Standard Deviation X High Byte. */
#define OTOS_REG_ACC_STD_XH      0x3F
/*! @brief Alamat register Acceleration Standard Deviation Y Low Byte. */
#define OTOS_REG_ACC_STD_YL      0x40
/*! @brief Alamat register Acceleration Standard Deviation Y High Byte. */
#define OTOS_REG_ACC_STD_YH      0x41
/*! @brief Alamat register Acceleration Standard Deviation H Low Byte. */
#define OTOS_REG_ACC_STD_HL      0x42
/*! @brief Alamat register Acceleration Standard Deviation H High Byte. */
#define OTOS_REG_ACC_STD_HH      0x43
/*! @brief Status register address. */
#define OTOS_REG_STATUS 		 0x1F
/*! @brief Expected Product ID value (0x5F). */
#define OTOS_PRODUCT_ID_VAL      0x5F

// OTOS CONSTANT
/*! @brief Conversion factor from raw counts to Meters for linear position. */
#define OTOS_METER_PER_COUNT        (10.0f / 32768.0f)
/*! @brief Conversion factor from raw counts to Radians for angular position. */
#define OTOS_RADIAN_PER_COUNT       ((float)M_PI / 32768.0f)
/*! @brief Conversion factor from Meters to Centimeters. */
#define OTOS_COUNT_TO_CM            (OTOS_METER_PER_COUNT * 100.0f)
/*! @brief Conversion factor from Meters to Inches. */
#define OTOS_METER_TO_INCH          39.37f
/*! @brief Conversion factor from Inches to Meters. */
#define OTOS_INCH_TO_METER          (1.0f / OTOS_METER_TO_INCH)
/*! @brief Conversion factor from Radians to Degrees. */
#define OTOS_RADIAN_TO_DEGREE       (180.0f / (float)M_PI)
/*! @brief Conversion factor from Degrees to Radians. */
#define OTOS_DEGREE_TO_RADIAN       ((float)M_PI / 180.0f)

/*! @brief Conversion factor from raw counts to Meters per second (linear velocity). */
#define OTOS_MPS_PER_COUNT          (5.0f / 32768.0f)
/*! @brief Conversion factor from raw counts to Meters per second squared (linear acceleration). */
#define OTOS_MPSS_PER_COUNT         ((16.0f * 9.80665f) / 32768.0f)
/*! @brief Conversion factor from raw counts to Radians per second (angular velocity). */
#define OTOS_RPS_PER_COUNT          (2000.0f * OTOS_DEGREE_TO_RADIAN / 32768.0f)
/*! @brief Conversion factor from raw counts to Radians per second squared (angular acceleration). */
#define OTOS_RPSS_PER_COUNT         ((float)M_PI * 1000.0f / 32768.0f)

/*! @brief Minimum allowed scalar value (0.872f). */
#define OTOS_MIN_SCALAR_VALUE       0.872f
/*! @brief Maximum allowed scalar value (1.127f). */
#define OTOS_MAX_SCALAR_VALUE       1.127f
/*! @brief Scalar step resolution (0.001f). */
#define OTOS_SCALAR_RESOLUTION      0.001f


// --------- UNION/STRUCT ---------

/*!
 * @brief Union for sensor signal processing configuration register.
 * @details Allows access to register 0x0E as a whole byte (`value`)
 * or as named bit-fields (`enLut`, `enAcc`, `enRot`, `enVar`).
 */
typedef union {
    struct {
        uint8_t enLut : 1;    ///< @brief Bit 0: Enables internal lookup table calibration for optical sensor.
        uint8_t enAcc : 1;    ///< @brief Bit 1: Enables feeding accelerometer data to Kalman filters.
        uint8_t enRot : 1;    ///< @brief Bit 2: Enables rotation of IMU and optical sensor data by heading angle.
        uint8_t enVar : 1;    ///< @brief Bit 3: Enables usage of correct sensor variance in Kalman filters.
        uint8_t reserved : 4; ///< @brief Bit 4-7: Reserved bits, do not use.
    };
    uint8_t value; ///< @brief Raw register value.
} sfe_otos_signal_process_config_t;

/*!
 * @brief Union for sensor status register.
 * @details Allows access to register 0x1F as a whole byte (`value`)
 * or as named bit-fields indicating warnings or errors.
 */
typedef union {
    struct {
        uint8_t warnTiltAngle : 1;       ///< @brief Bit 0: Returns 1 if tilt angle threshold has been exceeded.
        uint8_t warnOpticalTracking : 1; ///< @brief Bit 1: Returns 1 if optical tracking is unreliable.
        uint8_t reserved : 4;            ///< @brief Bit 2-5: Reserved bits, do not use.
        uint8_t errorPaa : 1;            ///< @brief Bit 6: Returns 1 if optical sensor (PAA) has a fatal error.
        uint8_t errorLsm : 1;            ///< @brief Bit 7: Returns 1 if IMU (LSM) has a fatal error.
    };
    uint8_t value; ///< @brief Raw register value.
} sfe_otos_status_t;

/*!
 * @brief Union for sensor hardware or firmware version.
 * @details Allows access to version registers as a whole byte (`value`)
 * or as separate bit-fields for major and minor versions.
 */
typedef union {
    struct {
        uint8_t minor : 4; ///< @brief Minor version number (rightmost 4 bits).
        uint8_t major : 4; ///< @brief Major version number (leftmost 4 bits).
    };
    uint8_t value; ///< @brief Raw register value.
} sfe_otos_version_t;

/*!
 * @brief Union for Self-Test configuration and status.
 * @details Allows access to the Self-Test register as a whole byte (`value`)
 * or as bit-fields to start or check test progress/results.
 */
typedef union {
    struct {
        uint8_t start : 1;      ///< @brief Bit 0: Write 1 to start the self test.
        uint8_t inProgress : 1; ///< @brief Bit 1: Returns 1 while the self test is in progress.
        uint8_t pass : 1;       ///< @brief Bit 2: Returns 1 if the self test passed.
        uint8_t fail : 1;       ///< @brief Bit 3: Returns 1 if the self test failed.
        uint8_t reserved : 4;   ///< @brief Bit 4-7: Reserved bits, do not use.
    };
    uint8_t value; ///< @brief Raw register value.
} sfe_otos_self_test_config_t;

/*!
 * @brief Structure to store linear position (X, Y) in Meters.
 * @details This is the standard output format for the linear position components.
 */
typedef struct {
    float x_m;      ///< @brief Position X in Meters.
    float y_m;      ///< @brief Position Y in Meters.
} OTOS_LinearPosition_t;

/*!
 * @brief Structure to store angular position (Heading) in Radians.
 * @details This is the standard output format for the angular position components.
 */
typedef struct {
    float h_rad;    ///< @brief Heading in Radians.
} OTOS_AngularPosition_t;

/*!
 * @brief Structure to store linear position (X, Y) in Centimeters.
 * @details Used for output converted to Centimeters.
 */
typedef struct {
    float x_cm;     ///< @brief Position X in Centimeters.
    float y_cm;     ///< @brief Position Y in Centimeters.
} OTOS_LinearPosition_CM_t;

/*!
 * @brief Structure to store linear position (X, Y) in Inches.
 * @details Used for output converted to Inches.
 */
typedef struct {
    float x_in;     ///< @brief Position X in Inches.
    float y_in;     ///< @brief Position Y in Inches.
} OTOS_LinearPosition_Inch_t;

/*!
 * @brief Structure to store angular position (Heading) in Degrees.
 * @details Used for output converted to Degrees.
 */
typedef struct {
    float h_deg;    ///< @brief Heading in Degrees.
} OTOS_AngularPosition_Deg_t;

/*!
 * @brief Structure to store linear and angular velocity.
 * @details Linear velocity (X,Y) in Meters/second. Angular velocity (Heading) in Radians/second.
 */
typedef struct {
    float x_m_s;      ///< @brief Velocity X in Meters/second.
    float y_m_s;      ///< @brief Velocity Y in Meters/second.
    float h_rad_s;    ///< @brief Angular velocity in Radians/second.
} OTOS_Velocity_t;

/*!
 * @brief Structure to store linear and angular acceleration.
 * @details Linear acceleration (X,Y) in Meters/second squared. Angular acceleration (Heading) in Radians/second squared.
 */
typedef struct {
    float x_m_s2;     ///< @brief Acceleration X in Meters/second^2.
    float y_m_s2;     ///< @brief Acceleration Y in Meters/second^2.
    float h_rad_s2;   ///< @brief Angular acceleration in Radians/second^2.
} OTOS_Acceleration_t;

/*!
 * @brief Structure to store Standard Deviation (StdDev) of position.
 * @details Used to read position tracking accuracy. Values are in Meters and Radians.
 */
typedef struct {
    float x_m_std;      ///< @brief Position X Standard Deviation in Meters.
    float y_m_std;      ///< @brief Position Y Standard Deviation in Meters.
    float h_rad_std;    ///< @brief Heading Standard Deviation in Radians.
} OTOS_PosStdDev_t;

/*!
 * @brief Structure to store Standard Deviation (StdDev) of velocity.
 * @details Used to read velocity tracking accuracy. Values are in Meters/second and Radians/second.
 */
typedef struct {
    float x_m_s_std;    ///< @brief Velocity X Standard Deviation in Meters/second.
    float y_m_s_std;    ///< @brief Velocity Y Standard Deviation in Meters/second.
    float h_rad_s_std;  ///< @brief Angular velocity Standard Deviation in Radians/second.
} OTOS_VelStdDev_t;

/*!
 * @brief Structure to store Standard Deviation (StdDev) of acceleration.
 * @details Used to read acceleration tracking accuracy. Values are in Meters/second squared and Radians/second squared.
 */
typedef struct {
    float x_m_s2_std;   ///< @brief Acceleration X Standard Deviation in Meters/second^2.
    float y_m_s2_std;   ///< @brief Acceleration Y Standard Deviation in Meters/second^2.
    float h_rad_s2_std; ///< @brief Angular acceleration Standard Deviation in Radians/second^2.
} OTOS_AccStdDev_t;

/*!
 * @brief Main structure representing a single instance of the OTOS sensor.
 * @details Stores the I2C handle used to communicate with this physical sensor.
 */
typedef struct {
    I2C_HandleTypeDef *hi2c_handle; ///< @brief Pointer to the HAL I2C handle used for communication.
} QwiicOTOS;


// --------- PROTOTYPE FUNCTION ---------

/*!
 * @brief Initializes the OTOS sensor instance (software-only).
 * @details This function sets up the OTOS sensor instance's data structure, linking it
 * with the provided HAL I2C handle. This is the first step before
 * configuring the physical sensor hardware.
 * @param instance Pointer to the QwiicOTOS structure to be initialized.
 * @param hi2c_handle_in Pointer to the HAL I2C handle from the microcontroller.
 * @return true if instance initialization is successful, false if instance or handle is NULL.
 */
bool    OTOS_init(QwiicOTOS *instance, I2C_HandleTypeDef *hi2c_handle_in);

/*!
 * @brief Configures the physical OTOS sensor hardware.
 * @details This function performs a series of initial configuration steps on the sensor,
 * including Product ID verification, setting default signal processing configuration (all TRUE),
 * and setting default scalar values (linear and angular to 1.0f).
 * IMU calibration and tracking reset are not included here and must be called separately.
 * @param instance Pointer to the QwiicOTOS sensor instance to be configured.
 * @return true if hardware configuration is successful, false if there is a failure (e.g., sensor not connected or wrong Product ID).
 */
bool    OTOS_begin_config(QwiicOTOS *instance);

/*!
 * @brief Resets the OTOS sensor's odometry tracking to the origin (0,0,0).
 * @details This resets the sensor's position (X, Y) and heading to zero.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 */
void    OTOS_resetTracking(QwiicOTOS *instance);

/*!
 * @brief Sets the signal processing configuration on the OTOS sensor.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param config The sfe_otos_signal_process_config_t structure containing bit settings.
 * @return true if the configuration is successfully written, false if an I2C communication failure occurs.
 */
bool    OTOS_setSignalProcessingConfig(QwiicOTOS *instance, sfe_otos_signal_process_config_t config);

/*!
 * @brief Sets the linear scalar factor on the OTOS sensor.
 * @details This scalar compensates for linear measurement scaling issues.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param scalar Linear scalar value (float), must be between 0.872 and 1.127.
 * @return true if the scalar is successfully set, false if the value is out of bounds or an I2C communication failure occurs.
 */
bool    OTOS_setLinearScalar(QwiicOTOS *instance, float scalar);

/*!
 * @brief Sets the angular scalar factor on the OTOS sensor.
 * @details This scalar compensates for angular measurement scaling issues.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param scalar Angular scalar value (float), must be between 0.872 and 1.127.
 * @return true if the scalar is successfully set, false if the value is out of bounds or an I2C communication failure occurs.
 */
bool    OTOS_setAngularScalar(QwiicOTOS *instance, float scalar);

/*!
 * @brief Performs an IMU calibration on the OTOS sensor.
 * @details This calibration removes accelerometer and gyroscope offsets.
 * @note The sensor must be perfectly still and level during calibration!
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param numSamples Number of samples to take for calibration (0-255).
 * @param waitUntilDone true to wait until calibration completes, false to run it asynchronously.
 * @return true if calibration successfully started/completed, false if a failure or timeout occurs.
 */
bool    OTOS_calibrateImu(QwiicOTOS *instance, uint8_t numSamples, bool waitUntilDone);

/*!
 * @brief Gets the firmware and hardware version of the OTOS sensor.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param hwVersion_out Pointer to the sfe_otos_version_t structure to store the hardware version.
 * @param fwVersion_out Pointer to the sfe_otos_version_t structure to store the firmware version.
 * @return true if successfully read the version, false if an I2C communication failure occurs.
 */
bool    OTOS_getVersionInfo(QwiicOTOS *instance, sfe_otos_version_t *hwVersion_out, sfe_otos_version_t *fwVersion_out);

/*!
 * @brief Performs a self-test on the OTOS sensor.
 * @details Runs internal diagnostics of the sensor to check its functionality.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @return true if the self-test passed, false if it failed or timed out.
 */
bool    OTOS_selfTest(QwiicOTOS *instance);

/*!
 * @brief Gets the position measured by the OTOS sensor.
 * @details Returns linear position (X, Y) in Meters and heading (H) in Radians.
 * This is the standard output from the sensor.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param linear_out Pointer to the OTOS_LinearPosition_t structure to store the linear position.
 * @param angular_out Pointer to the OTOS_AngularPosition_t structure to store the angular position.
 * @return true if successfully read the position, false if an I2C communication failure occurs.
 */
bool    OTOS_getPosition(QwiicOTOS *instance, OTOS_LinearPosition_t *linear_out, OTOS_AngularPosition_t *angular_out);

/*!
 * @brief Sets the initial position of the OTOS sensor.
 * @details This allows the sensor to start tracking from a specified (X,Y,H) position.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param x_m Position X in Meters.
 * @param y_m Position Y in Meters.
 * @param h_rad Heading in Radians.
 * @return true if the position is successfully set, false if an I2C communication failure occurs.
 */
bool    OTOS_setPosition(QwiicOTOS *instance, float x_m, float y_m, float h_rad);

/*!
 * @brief Gets the velocity measured by the OTOS sensor.
 * @details Returns linear velocity (X, Y) in Meters/second and angular velocity (H) in Radians/second.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param vel_out Pointer to the OTOS_Velocity_t structure to store the velocity.
 * @return true if successfully read the velocity, false if an I2C communication failure occurs.
 */
bool    OTOS_getVelocity(QwiicOTOS *instance, OTOS_Velocity_t *vel_out);

/*!
 * @brief Gets the acceleration measured by the OTOS sensor.
 * @details Returns linear acceleration (X, Y) in Meters/second squared and angular acceleration (H) in Radians/second squared.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param acc_out Pointer to the OTOS_Acceleration_t structure to store the acceleration.
 * @return true if successfully read the acceleration, false if an I2C communication failure occurs.
 */
bool    OTOS_getAcceleration(QwiicOTOS *instance, OTOS_Acceleration_t *acc_out);

/*!
 * @brief Gets the Standard Deviation (StdDev) of position measured by the OTOS.
 * @details This value is a statistical indication of position tracking accuracy.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param std_out Pointer to the OTOS_PosStdDev_t structure to store the position standard deviation.
 * @return true if successfully read the standard deviation, false if an I2C communication failure occurs.
 */
bool    OTOS_getPosStdDev(QwiicOTOS *instance, OTOS_PosStdDev_t *std_out);

/*!
 * @brief Gets the Standard Deviation (StdDev) of velocity measured by the OTOS.
 * @details This value is a statistical indication of velocity tracking accuracy.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param std_out Pointer to the OTOS_VelStdDev_t structure to store the velocity standard deviation.
 * @return true if successfully read the standard deviation, false if an I2C communication failure occurs.
 */
bool    OTOS_getVelStdDev(QwiicOTOS *instance, OTOS_VelStdDev_t *std_out);

/*!
 * @brief Gets the Standard Deviation (StdDev) of acceleration measured by the OTOS.
 * @details This value is a statistical indication of acceleration tracking accuracy.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param std_out Pointer to the OTOS_AccStdDev_t structure to store the acceleration standard deviation.
 * @return true if successfully read the standard deviation, false if an I2C communication failure occurs.
 */
bool    OTOS_getAccStdDev(QwiicOTOS *instance, OTOS_AccStdDev_t *std_out);

/*!
 * @brief Gets the status register from the OTOS sensor.
 * @details This status includes warnings (e.g., tilt, unreliable tracking)
 * and fatal sensor component errors.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param status_out Pointer to the sfe_otos_status_t structure to store the sensor status.
 * @return true if successfully read the status, false if an I2C communication failure occurs.
 */
bool    OTOS_getStatus(QwiicOTOS *instance, sfe_otos_status_t *status_out);

/*!
 * @brief Sets the offset of the OTOS sensor's position.
 * @details This offset is used to shift the sensor's tracking reference point
 * relative to the robot's center. Input values are in Centimeters and Degrees.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param x_offset_cm Offset X in Centimeters.
 * @param y_offset_cm Offset Y in Centimeters.
 * @param h_offset_deg Offset Heading in Degrees.
 * @return true if the offset is successfully set, false if an I2C communication failure occurs.
 */
bool    OTOS_setOffset(QwiicOTOS *instance, float x_offset_cm, float y_offset_cm, float h_offset_deg);

// --- Prototipe Fungsi Utility Konversi ---
/*!
 * @brief Converts linear position from Meters to Centimeters.
 * @param m_pos Pointer to the OTOS_LinearPosition_t structure (input in Meters).
 * @param cm_pos_out Pointer to the OTOS_LinearPosition_CM_t structure (output in Centimeters).
 */
void    OTOS_convertLinear_m_to_cm(OTOS_LinearPosition_t *m_pos, OTOS_LinearPosition_CM_t *cm_pos_out);

/*!
 * @brief Converts linear position from Meters to Inches.
 * @param m_pos Pointer to the OTOS_LinearPosition_t structure (input in Meters).
 * @param in_pos_out Pointer to the OTOS_LinearPosition_Inch_t structure (output in Inches).
 */
void    OTOS_convertLinear_m_to_inch(OTOS_LinearPosition_t *m_pos, OTOS_LinearPosition_Inch_t *in_pos_out);

/*!
 * @brief Converts angular position from Radians to Degrees.
 * @param rad_pos Pointer to the OTOS_AngularPosition_t structure (input in Radians).
 * @param deg_pos_out Pointer to the OTOS_AngularPosition_Deg_t (output in Degrees).
 */
void    OTOS_convertAngular_rad_to_deg(OTOS_AngularPosition_t *rad_pos, OTOS_AngularPosition_Deg_t *deg_pos_out);

/*!
 * @brief Converts an angle value from Degrees to Radians.
 * @param degrees Angle in Degrees.
 * @return Angle in Radians.
 */
float 	OTOS_degreesToRadians(float degrees);

/*!
 * @brief Converts an angle value from Radians to Degrees.
 * @param radians Angle in Radians.
 * @return Angle in Degrees.
 */
float 	OTOS_radiansToDegrees(float radians);

#endif
