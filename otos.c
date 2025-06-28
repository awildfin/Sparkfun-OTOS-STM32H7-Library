/*!
 * @file otos.c
 * @brief C driver implementation for the SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS).
 * @details This file contains the implementation of the functions declared in otos.h,
 * providing a C interface for the OTOS sensor. The implementation involves
 * I2C communication using the STM32 HAL, data conversion, and status handling.
 */

// --------- INCLUDES ---------
#include "otos.h"
#include "stm32h7xx_hal.h" // Required for HAL_I2C_Mem_Read/Write, HAL_Delay, etc.


// --------- HAL FUNCTION ---------
// External declaration for the HAL_Delay function. Its implementation is elsewhere (e.g., main.c).
extern void HAL_Delay(uint32_t Delay);


// --------- I2C HELPER FUNCTIONS ---------
/*!
 * @brief Reads a number of bytes from an OTOS sensor register via I2C.
 * @details Internal helper function for I2C read operations.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param reg The sensor register address to start reading from.
 * @param buf Pointer to the buffer where the read data will be stored.
 * @param len The number of bytes to read.
 * @return true if the I2C read operation was successful, false otherwise.
 */
static bool OTOS_readBytes(QwiicOTOS *instance, uint8_t reg, uint8_t *buf, uint8_t len) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(instance->hi2c_handle, OTOS_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, HAL_MAX_DELAY);
    return (status == HAL_OK);
}

/*!
 * @brief Writes a single byte to an OTOS sensor register via I2C.
 * @details Internal helper function for single-byte I2C write operations.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param reg The sensor register address to write to.
 * @param val The byte value to be written.
 * @return true if the I2C write operation was successful, false otherwise.
 */
static bool OTOS_writeByte(QwiicOTOS *instance, uint8_t reg, uint8_t val) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(instance->hi2c_handle, OTOS_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY);
    return (status == HAL_OK);
}

/*!
 * @brief Writes multiple bytes to an OTOS sensor register via I2C.
 * @details Internal helper function for burst I2C write operations (multiple consecutive bytes).
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param reg The sensor register address to start writing to.
 * @param buf Pointer to the buffer containing the data to be written.
 * @param len The number of bytes to write.
 * @return true if the I2C write operation was successful, false otherwise.
 */
static bool OTOS_writeBytes(QwiicOTOS *instance, uint8_t reg, uint8_t *buf, uint8_t len) {
    return (HAL_I2C_Mem_Write(instance->hi2c_handle, OTOS_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, HAL_MAX_DELAY) == HAL_OK);
}

/*!
 * @brief Internal helper function to read pose data (position, velocity, acceleration, std dev).
 * @details Reads 6 bytes of pose data from the sensor starting at start_reg, reconstructs them into int16_t,
 * and converts to a float in standard units (Meters/Radians).
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param start_reg The starting register address for the pose reading (e.g., OTOS_REG_POS_X_L).
 * @param linear_out Pointer to an OTOS_LinearPosition_t struct to store linear components.
 * @param angular_out Pointer to an OTOS_AngularPosition_t struct to store the angular component.
 * @param linear_conversion_factor The conversion factor from counts to linear units (Meters).
 * @param angular_conversion_factor The conversion factor from counts to angular units (Radians).
 * @return true if reading and conversion were successful, false otherwise.
 */
static bool OTOS_readPoseData_internal(QwiicOTOS *instance, uint8_t start_reg, OTOS_LinearPosition_t *linear_out, OTOS_AngularPosition_t *angular_out, float linear_conversion_factor, float angular_conversion_factor) {
    int16_t x_counts, y_counts, h_counts;
    uint8_t buf[6];

    if (!OTOS_readBytes(instance, start_reg, buf, 6)) return false;

    // Reconstruct 16-bit values from raw bytes (Little-Endian: Low Byte, High Byte)
    x_counts = (int16_t)((buf[1] << 8) | buf[0]);
    y_counts = (int16_t)((buf[3] << 8) | buf[2]);
    h_counts = (int16_t)((buf[5] << 8) | buf[4]);

    // Convert counts to the specified units (Meters/Radians)
    linear_out->x_m = (float)x_counts * linear_conversion_factor;
    linear_out->y_m = (float)y_counts * linear_conversion_factor;
    angular_out->h_rad = (float)h_counts * angular_conversion_factor;

    return true;
}

/*!
 * @brief Internal helper function to write pose data to the sensor.
 * @details Converts float values (Meters/Radians) into int16_t counts, packs them into 6 bytes,
 * and writes them in a burst to the sensor registers.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param start_reg The starting register address for the pose write (e.g., OTOS_REG_POS_X_L).
 * @param x_value The X value in Meters/CM/Inches.
 * @param y_value The Y value in Meters/CM/Inches.
 * @param h_value The Heading value in Radians/Degrees.
 * @param linear_conversion_factor_inv The inverse conversion factor from linear units to counts (counts per unit).
 * @param angular_conversion_factor_inv The inverse conversion factor from angular units to counts (counts per unit).
 * @return true if writing and conversion were successful, false otherwise.
 */
static bool OTOS_writePoseData_internal(QwiicOTOS *instance, uint8_t start_reg, float x_value, float y_value, float h_value, float linear_conversion_factor_inv, float angular_conversion_factor_inv) {
    // Convert float to int16_t counts (using rounding)
    int16_t x_counts = (int16_t)roundf(x_value * linear_conversion_factor_inv);
    int16_t y_counts = (int16_t)roundf(y_value * linear_conversion_factor_inv);
    int16_t h_counts = (int16_t)roundf(h_value * angular_conversion_factor_inv);

    // Pack the count values into a 6-byte buffer (Little-Endian)
    uint8_t buf[6] = {
        (uint8_t)(x_counts & 0xFF), (uint8_t)(x_counts >> 8),
        (uint8_t)(y_counts & 0xFF), (uint8_t)(y_counts >> 8),
        (uint8_t)(h_counts & 0xFF), (uint8_t)(h_counts >> 8)
    };

    // Write these 6 bytes in a burst to the register
    return OTOS_writeBytes(instance, start_reg, buf, 6);
}


// --------- OTOS I2C INIT ---------
/*!
 * @brief Initializes an OTOS sensor instance (non-hardware).
 * @details This function prepares the OTOS sensor instance data structure, linking it
 * with the provided HAL I2C handle. This is the first step before
 * configuring the physical sensor hardware.
 * @param instance Pointer to the QwiicOTOS structure to be initialized.
 * @param hi2c_handle_in Pointer to the microcontroller's HAL I2C handle.
 * @return true if the instance initialization is successful, false if the instance or handle is NULL.
 */
bool OTOS_init(QwiicOTOS *instance, I2C_HandleTypeDef *hi2c_handle_in) {
    if (instance == NULL || hi2c_handle_in == NULL) return false;
    instance->hi2c_handle = hi2c_handle_in;
    return true;
}

// --------- SIGNAL PROCESSING ---------
/*!
 * @brief Sets the signal processing configuration on the OTOS sensor.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param config An sfe_otos_signal_process_config_t struct containing the bit settings.
 * @return true if the configuration was successfully written, false on I2C communication failure.
 */
bool OTOS_setSignalProcessingConfig(QwiicOTOS *instance, sfe_otos_signal_process_config_t config) {
    return OTOS_writeByte(instance, OTOS_REG_SIGNAL_PROCESS, config.value);
}

// --------- SET SCALAR ---------
// LINEAR
/*!
 * @brief Sets the linear scaling factor on the OTOS sensor.
 * @details This scalar compensates for linear measurement scaling issues.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param scalar The linear scalar value (float), must be between 0.872 and 1.127.
 * @return true if the scalar was set successfully, false if the value is out of bounds or on I2C communication failure.
 */
bool OTOS_setLinearScalar(QwiicOTOS *instance, float scalar) {
    if (scalar < OTOS_MIN_SCALAR_VALUE || scalar > OTOS_MAX_SCALAR_VALUE) { return false; }
    // Convert float to int8_t (signed) based on the sensor's formula
    int8_t rawScalar_signed = (int8_t)roundf((scalar - 1.0f) * 1000.0f);
    return OTOS_writeByte(instance, OTOS_REG_SCALAR_LINEAR, (uint8_t)rawScalar_signed);
}

// ANGULAR
/*!
 * @brief Sets the angular scaling factor on the OTOS sensor.
 * @details This scalar compensates for angular measurement scaling issues.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param scalar The angular scalar value (float), must be between 0.872 and 1.127.
 * @return true if the scalar was set successfully, false if the value is out of bounds or on I2C communication failure.
 */
bool OTOS_setAngularScalar(QwiicOTOS *instance, float scalar) {
    if (scalar < OTOS_MIN_SCALAR_VALUE || scalar > OTOS_MAX_SCALAR_VALUE) { return false; }
    // Convert float to int8_t (signed) based on the sensor's formula
    int8_t rawScalar_signed = (int8_t)roundf((scalar - 1.0f) * 1000.0f);
    return OTOS_writeByte(instance, OTOS_REG_SCALAR_ANGULAR, (uint8_t)rawScalar_signed);
}


// --------- CALIBRATE IMU ---------
/*!
 * @brief Performs an IMU calibration on the OTOS sensor.
 * @details This calibration removes accelerometer and gyroscope offsets.
 * @note The sensor must be completely still and level during calibration!
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param numSamples The number of samples to take for calibration (0-255).
 * @param waitUntilDone true to block until calibration is complete, false to run it asynchronously.
 * @return true if calibration successfully started/completed, false on failure or timeout.
 */
bool OTOS_calibrateImu(QwiicOTOS *instance, uint8_t numSamples, bool waitUntilDone) {
    if (!OTOS_writeByte(instance, OTOS_REG_IMU_CALIB, numSamples)) { return false; }
    if (waitUntilDone) {
        uint8_t progress = numSamples;
        for (uint32_t timeout_ms = 0; timeout_ms < 1000; timeout_ms++) {
            if (!OTOS_readBytes(instance, OTOS_REG_IMU_CALIB, &progress, 1)) { return false; }
            if (progress == 0) { return true; }
            HAL_Delay(1);
        }
        return false; // Timeout
    }
    return true;
}

// --------- RESET TRACKING ---------
/*!
 * @brief Resets the odometry tracking of the OTOS sensor to the origin.
 * @details This sets the position (X, Y) and heading of the sensor to zero.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 */
void OTOS_resetTracking(QwiicOTOS *instance) {
    OTOS_writeByte(instance, OTOS_REG_RESET_TRACK, 0x01);
}


// --------- SET OFFSET ---------
/*!
 * @brief Sets the position offset of the OTOS sensor.
 * @details This offset is used to shift the sensor's tracking reference point
 * relative to the center of the robot. Input values are in Meters and Degrees.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param x_offset_m The X offset in Meters.
 * @param y_offset_m The Y offset in Meters.
 * @param h_offset_deg The Heading offset in Degrees.
 * @return true if the offset was set successfully, false on I2C communication failure.
 */
bool OTOS_setOffset(QwiicOTOS *instance, float x_offset_m, float y_offset_m, float h_offset_deg) {
    float m_per_count_inv = 1.0f / OTOS_METER_PER_COUNT;
    float deg_per_count_inv = 1.0f / (OTOS_RADIAN_PER_COUNT * OTOS_RADIAN_TO_DEGREE);
    return OTOS_writePoseData_internal(instance, OTOS_REG_OFFSET_X_L, x_offset_m, y_offset_m, h_offset_deg, m_per_count_inv, deg_per_count_inv);
}

// --------- SET POSITION ---------
/*!
 * @brief Sets the current position of the OTOS sensor.
 * @details This allows the sensor to start tracking from a specified (X,Y,H) position.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param x_m The X position in Meters.
 * @param y_m The Y position in Meters.
 * @param h_rad The Heading in Radians.
 * @return true if the position was set successfully, false on I2C communication failure.
 */
bool OTOS_setPosition(QwiicOTOS *instance, float x_m, float y_m, float h_rad) {
    float meters_per_count_inv = 1.0f / OTOS_METER_PER_COUNT;
    float radians_per_count_inv = 1.0f / OTOS_RADIAN_PER_COUNT;
    return OTOS_writePoseData_internal(instance, OTOS_REG_POS_X_L, x_m, y_m, h_rad, meters_per_count_inv, radians_per_count_inv);
}


// --------- GET POSITION ---------
/*!
 * @brief Gets the position measured by the OTOS sensor.
 * @details Returns the linear position (X, Y) in Meters and heading (H) in Radians.
 * This is the standard output of the sensor.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param linear_out Pointer to an OTOS_LinearPosition_t struct to store the linear position.
 * @param angular_out Pointer to an OTOS_AngularPosition_t struct to store the angular position.
 * @return true if the position was read successfully, false on I2C communication failure.
 */
bool OTOS_getPosition(QwiicOTOS *instance, OTOS_LinearPosition_t *linear_out, OTOS_AngularPosition_t *angular_out) {
    return OTOS_readPoseData_internal(instance, OTOS_REG_POS_X_L, linear_out, angular_out, OTOS_METER_PER_COUNT, OTOS_RADIAN_PER_COUNT);
}

// --------- GET VELOCITY ---------
/*!
 * @brief Gets the velocity measured by the OTOS sensor.
 * @details Returns the linear velocity (X, Y) in Meters/second and angular velocity (H) in Radians/second.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param vel_out Pointer to an OTOS_Velocity_t struct to store the velocity.
 * @return true if the velocity was read successfully, false on I2C communication failure.
 */
bool OTOS_getVelocity(QwiicOTOS *instance, OTOS_Velocity_t *vel_out) {
    OTOS_LinearPosition_t temp_linear;
    OTOS_AngularPosition_t temp_angular;
    if (!OTOS_readPoseData_internal(instance, OTOS_REG_VEL_XL, &temp_linear, &temp_angular, OTOS_MPS_PER_COUNT, OTOS_RPS_PER_COUNT)) {
        return false;
    }
    vel_out->x_m_s = temp_linear.x_m;
    vel_out->y_m_s = temp_linear.y_m;
    vel_out->h_rad_s = temp_angular.h_rad;
    return true;
}

// --------- GET ACCELERATION ---------
/*!
 * @brief Gets the acceleration measured by the OTOS sensor.
 * @details Returns the linear acceleration (X, Y) in Meters/second^2 and angular acceleration (H) in Radians/second^2.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param acc_out Pointer to an OTOS_Acceleration_t struct to store the acceleration.
 * @return true if the acceleration was read successfully, false on I2C communication failure.
 */
bool OTOS_getAcceleration(QwiicOTOS *instance, OTOS_Acceleration_t *acc_out) {
    OTOS_LinearPosition_t temp_linear;
    OTOS_AngularPosition_t temp_angular;
    if (!OTOS_readPoseData_internal(instance, OTOS_REG_ACC_XL, &temp_linear, &temp_angular, OTOS_MPSS_PER_COUNT, OTOS_RPSS_PER_COUNT)) {
        return false;
    }
    acc_out->x_m_s2 = temp_linear.x_m;
    acc_out->y_m_s2 = temp_linear.y_m;
    acc_out->h_rad_s2 = temp_angular.h_rad;
    return true;
}

// --------- GET STANDARD DEVIATION ---------
// POSITION
/*!
 * @brief Gets the Standard Deviation of the position measured by OTOS.
 * @details This value is a statistical indication of the position tracking accuracy.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param std_out Pointer to an OTOS_PosStdDev_t struct to store the position standard deviation.
 * @return true if the standard deviation was read successfully, false on I2C communication failure.
 */
bool OTOS_getPosStdDev(QwiicOTOS *instance, OTOS_PosStdDev_t *std_out) {
    OTOS_LinearPosition_t temp_linear;
    OTOS_AngularPosition_t temp_angular;
    if (!OTOS_readPoseData_internal(instance, OTOS_REG_POS_STD_XL, &temp_linear, &temp_angular, OTOS_METER_PER_COUNT, OTOS_RADIAN_PER_COUNT)) {
        return false;
    }
    std_out->x_m_std = temp_linear.x_m;
    std_out->y_m_std = temp_linear.y_m;
    std_out->h_rad_std = temp_angular.h_rad;
    return true;
}

// VELOCITY
/*!
 * @brief Gets the Standard Deviation of the velocity measured by OTOS.
 * @details This value is a statistical indication of the velocity tracking accuracy.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param std_out Pointer to an OTOS_VelStdDev_t struct to store the velocity standard deviation.
 * @return true if the standard deviation was read successfully, false on I2C communication failure.
 */
bool OTOS_getVelStdDev(QwiicOTOS *instance, OTOS_VelStdDev_t *std_out) {
    OTOS_LinearPosition_t temp_linear;
    OTOS_AngularPosition_t temp_angular;
    if (!OTOS_readPoseData_internal(instance, OTOS_REG_VEL_STD_XL, &temp_linear, &temp_angular, OTOS_MPS_PER_COUNT, OTOS_RPS_PER_COUNT)) {
        return false;
    }
    std_out->x_m_s_std = temp_linear.x_m;
    std_out->y_m_s_std = temp_linear.y_m;
    std_out->h_rad_s_std = temp_angular.h_rad;
    return true;
}

// ACCELERATION
/*!
 * @brief Gets the Standard Deviation of the acceleration measured by OTOS.
 * @details This value is a statistical indication of the acceleration tracking accuracy.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param std_out Pointer to an OTOS_AccStdDev_t struct to store the acceleration standard deviation.
 * @return true if the standard deviation was read successfully, false on I2C communication failure.
 */
bool OTOS_getAccStdDev(QwiicOTOS *instance, OTOS_AccStdDev_t *std_out) {
    OTOS_LinearPosition_t temp_linear;
    OTOS_AngularPosition_t temp_angular;
    if (!OTOS_readPoseData_internal(instance, OTOS_REG_ACC_STD_XL, &temp_linear, &temp_angular, OTOS_MPSS_PER_COUNT, OTOS_RPSS_PER_COUNT)) {
        return false;
    }
    std_out->x_m_s2_std = temp_linear.x_m;
    std_out->y_m_s2_std = temp_linear.y_m;
    std_out->h_rad_s2_std = temp_angular.h_rad;
    return true;
}

// --------- GET FIRMWARE/HARDWARE VERSION ---------
/*!
 * @brief Gets the hardware and firmware versions of the OTOS sensor.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param hwVersion_out Pointer to an sfe_otos_version_t struct to store the hardware version.
 * @param fwVersion_out Pointer to an sfe_otos_version_t struct to store the firmware version.
 * @return true if the versions were read successfully, false on I2C communication failure.
 */
bool OTOS_getVersionInfo(QwiicOTOS *instance, sfe_otos_version_t *hwVersion_out, sfe_otos_version_t *fwVersion_out) {
    uint8_t rawData[2]; // Buffer to read 2 bytes (HW Version and FW Version)
    // Read sequentially from the Hardware Version (0x01) and Firmware Version (0x02) registers
    if (!OTOS_readBytes(instance, OTOS_REG_HW_VERSION, rawData, 2)) {
        return false;
    }

    // Store the first byte in hwVersion_out, the second in fwVersion_out
    hwVersion_out->value = rawData[0]; // Hardware Version
    fwVersion_out->value = rawData[1]; // Firmware Version

    return true;
}

// --------- SELF TEST ---------
/*!
 * @brief Performs a self-test on the OTOS sensor.
 * @details Runs the sensor's internal diagnostics to check its functionality.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @return true if the self-test passed, false on failure or timeout.
 */
bool OTOS_selfTest(QwiicOTOS *instance) {
    sfe_otos_self_test_config_t selfTest;
    selfTest.start = 1; // Set the 'start' bit to 1 to begin the self-test

    if (!OTOS_writeByte(instance, OTOS_REG_SELF_TEST, selfTest.value)) {
        return false; // Failed to start the self-test
    }

    // Loop until the self-test is complete or timeout occurs
    // SparkFun states this takes ~20ms
    for (uint32_t timeout_ms = 0; timeout_ms < 10; timeout_ms++) {
        HAL_Delay(5); // Wait 5ms
        if (!OTOS_readBytes(instance, OTOS_REG_SELF_TEST, &(selfTest.value), 1)) {
            return false; // Failed to read self-test status
        }
        if (selfTest.inProgress == 0) { // If the inProgress bit is 0, the test is finished
            break;
        }
    }

    // Check if the self-test was successful (pass bit = 1)
    if (selfTest.pass == 1) {
        return true;
    }
    return false; // Self-test failed or timed out
}


// --------- GET STATUS ---------
/*!
 * @brief Gets the status register from the OTOS sensor.
 * @details The status includes warnings (e.g., excessive tilt, unreliable tracking)
 * and fatal errors of the sensor components.
 * @param instance Pointer to the QwiicOTOS sensor instance.
 * @param status_out Pointer to an sfe_otos_status_t struct to store the sensor status.
 * @return true if the status was read successfully, false on I2C communication failure.
 */
bool OTOS_getStatus(QwiicOTOS *instance, sfe_otos_status_t *status_out) {
    return OTOS_readBytes(instance, OTOS_REG_STATUS, &(status_out->value), 1);
}


// --------- OTOS CONFIGURATION INIT ---------
/*!
 * @brief Configures the physical OTOS sensor hardware.
 * @details This function performs a series of initial configuration steps on the sensor,
 * including verifying the Product ID, setting default signal processing configurations (all TRUE),
 * and setting default scalar values (linear and angular to 1.0f).
 * IMU calibration and tracking reset are not included here and must be called separately.
 * @param instance Pointer to the QwiicOTOS instance to be configured.
 * @return true if the hardware configuration is successful, false on any failure (e.g., sensor not connected or wrong Product ID).
 */
bool OTOS_begin_config(QwiicOTOS *instance) {
    uint8_t productId = 0;
    if (!OTOS_readBytes(instance, OTOS_REG_PRODUCT_ID, &productId, 1)) {
        return false; // Failed to read Product ID
    }
    if (productId != OTOS_PRODUCT_ID_VAL) {
        return false; // Incorrect Product ID
    }

    sfe_otos_signal_process_config_t default_config = {0};
    default_config.enRot = true;
    default_config.enAcc = true;
    default_config.enLut = true;
    default_config.enVar = true;
    if (!OTOS_setSignalProcessingConfig(instance, default_config)) {
        return false;
    }

    if (!OTOS_setLinearScalar(instance, 1.0f)) {
        return false;
    }

    if (!OTOS_setAngularScalar(instance, 1.0f)) {
        return false;
    }

    return true;
}


// --------- CONVERT ---------
/*!
 * @brief Converts a linear position from Meters to Centimeters.
 * @param m_pos Pointer to an OTOS_LinearPosition_t struct (input in Meters).
 * @param cm_pos_out Pointer to an OTOS_LinearPosition_CM_t struct (output in Centimeters).
 */
void OTOS_convertLinear_m_to_cm(OTOS_LinearPosition_t *m_pos, OTOS_LinearPosition_CM_t *cm_pos_out) {
    cm_pos_out->x_cm = m_pos->x_m * 100.0f;
    cm_pos_out->y_cm = m_pos->y_m * 100.0f;
}

/*!
 * @brief Converts a linear position from Meters to Inches.
 * @param m_pos Pointer to an OTOS_LinearPosition_t struct (input in Meters).
 * @param in_pos_out Pointer to an OTOS_LinearPosition_Inch_t struct (output in Inches).
 */
void OTOS_convertLinear_m_to_inch(OTOS_LinearPosition_t *m_pos, OTOS_LinearPosition_Inch_t *in_pos_out) {
    in_pos_out->x_in = m_pos->x_m * OTOS_METER_TO_INCH;
    in_pos_out->y_in = m_pos->y_m * OTOS_METER_TO_INCH;
}

/*!
 * @brief Converts an angular position from Radians to Degrees.
 * @param rad_pos Pointer to an OTOS_AngularPosition_t struct (input in Radians).
 * @param deg_pos_out Pointer to an OTOS_AngularPosition_Deg_t struct (output in Degrees).
 */
void OTOS_convertAngular_rad_to_deg(OTOS_AngularPosition_t *rad_pos, OTOS_AngularPosition_Deg_t *deg_pos_out) {
    deg_pos_out->h_deg = OTOS_radiansToDegrees(rad_pos->h_rad);
}

/*!
 * @brief Converts an angle value from Degrees to Radians.
 * @param degrees The angle in Degrees.
 * @return The angle in Radians.
 */
float OTOS_degreesToRadians(float degrees) {
    return degrees * OTOS_DEGREE_TO_RADIAN;
}

/*!
 * @brief Converts an angle value from Radians to Degrees.
 * @param radians The angle in Radians.
 * @return The angle in Degrees.
 */
float OTOS_radiansToDegrees(float radians) {
    return radians * OTOS_RADIAN_TO_DEGREE;
}
