/*
 * mpu.h
 *
 *  Created on: May 3, 2021
 *      Author: anind
 */

#ifndef INC_MPU_H_
#define INC_MPU_H_

#include "math.h"

#include "main.h"
#include "usart.h"


//Magnetometer Registers
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L	 0x03  // data
#define AK8963_XOUT_H	 0x04
#define AK8963_YOUT_L	 0x05
#define AK8963_YOUT_H	 0x06
#define AK8963_ZOUT_L	 0x07
#define AK8963_ZOUT_H	 0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

// #define X_FINE_GAIN      0x03 // [7:0] fine gain
// #define Y_FINE_GAIN      0x04
// #define Z_FINE_GAIN      0x05
// #define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
// #define XA_OFFSET_L_TC   0x07
// #define YA_OFFSET_H      0x08
// #define YA_OFFSET_L_TC   0x09
// #define ZA_OFFSET_H      0x0A
// #define ZA_OFFSET_L_TC   0x0B

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define MPU_CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

#define PWR_RESET           0x80
#define DIS_GYRO            0x07
#define PWR_CYCLE           0x20

#define AK8963_HXL          0x03
#define AK8963_CNTL1        0x0A
#define AK8963_PWR_DOWN     0x00
#define AK8963_CNT_MEAS1    0x12
#define AK8963_CNT_MEAS2    0x16
#define AK8963_FUSE_ROM     0x0F
#define AK8963_CNTL2        0x0B
#define AK8963_RESET        0x01
#define AK8963_ASA          0x10
#define AK8963_WHO_AM_I     0x00











#define WHOAMI_VALUE 0x71
#define MPU9250_ADDRESS (0x69<<1) // Device address when ADO = 1
#define AK8963_ADDRESS (0x0C<<1)  //  Address of magnetometer
#define AK8963_WHOAMI_VALUE 0x48

#define CALIB_GYRO_SENSITIVITY 131 // LSB/degrees/sec
#define CALIB_ACCEL_SENSITIVITY 16384
#define MAG_MODE 0x06 // 0x02 for 8 Hz, 0x06 for 100 Hz continuous magnetometer data read

enum class ACCEL_FS_SEL
{
    A2G,
    A4G,
    A8G,
    A16G
};
enum class GYRO_FS_SEL
{
    G250DPS,
    G500DPS,
    G1000DPS,
    G2000DPS
};
enum class MAG_OUTPUT_BITS
{
    M14BITS,
    M16BITS
};

enum class FIFO_SAMPLE_RATE : uint8_t
{
    SMPL_1000HZ,
    SMPL_500HZ,
    SMPL_333HZ,
    SMPL_250HZ,
    SMPL_200HZ,
    SMPL_167HZ,
    SMPL_143HZ,
    SMPL_125HZ,
};

enum class GYRO_DLPF_CFG : uint8_t
{
    DLPF_250HZ,
    DLPF_184HZ,
    DLPF_92HZ,
    DLPF_41HZ,
    DLPF_20HZ,
    DLPF_10HZ,
    DLPF_5HZ,
    DLPF_3600HZ,
};

enum class ACCEL_DLPF_CFG : uint8_t
{
    DLPF_218HZ_0,
    DLPF_218HZ_1,
    DLPF_99HZ,
    DLPF_45HZ,
    DLPF_21HZ,
    DLPF_10HZ,
    DLPF_5HZ,
    DLPF_420HZ,
};

enum class ACCEL_LP_ODR : uint8_t
{
    LP_ACCEL_ODR_0_24HZ,
    LP_ACCEL_ODR_0_49HZ,
    LP_ACCEL_ODR_0_98HZ,
    LP_ACCEL_ODR_1_95HZ,
    LP_ACCEL_ODR_3_91HZ,
    LP_ACCEL_ODR_7_81HZ,
    LP_ACCEL_ODR_15_63HZ,
    LP_ACCEL_ODR_31_25HZ,
    LP_ACCEL_ODR_62_50HZ,
    LP_ACCEL_ODR_125HZ,
    LP_ACCEL_ODR_250HZ,
    LP_ACCEL_ODR_500HZ
};

struct MPU9250Setting
{
    ACCEL_FS_SEL accel_fs_sel{ACCEL_FS_SEL::A16G};
    GYRO_FS_SEL gyro_fs_sel{GYRO_FS_SEL::G2000DPS};
    MAG_OUTPUT_BITS mag_output_bits{MAG_OUTPUT_BITS::M16BITS};
    FIFO_SAMPLE_RATE fifo_sample_rate{FIFO_SAMPLE_RATE::SMPL_200HZ};
    uint8_t gyro_fchoice{0x03};
    GYRO_DLPF_CFG gyro_dlpf_cfg{GYRO_DLPF_CFG::DLPF_41HZ};
    uint8_t accel_fchoice{0x01};
    ACCEL_DLPF_CFG accel_dlpf_cfg{ACCEL_DLPF_CFG::DLPF_45HZ};
};


class MPU9250 {
	// settings
	MPU9250Setting setting;

	float acc_resolution { 0.f };  // scale resolutions per LSB for the sensors
	float gyro_resolution { 0.f }; // scale resolutions per LSB for the sensors
	float mag_resolution { 0.f };  // scale resolutions per LSB for the sensors

	// Calibration Parameters
	float acc_bias[3] { 0., 0., 0. }; // acc calibration value in ACCEL_FS_SEL: 2g
	float gyro_bias[3] { 0., 0., 0. }; // gyro calibration value in GYRO_FS_SEL: 250dps
	float mag_bias_factory[3] { 0., 0., 0. };
	float mag_bias[3] { 0., 0., 0. }; // mag calibration value in MAG_OUTPUT_BITS: 16BITS
	float mag_scale[3] { 1., 1., 1. };
	float magnetic_declination = -7.51; // Japan, 24th June

	// Self Test
	float self_test_result[6] { 0.f }; // holds results of gyro and accelerometer self test

	// IMU Data
	float a[3] { 0.f, 0.f, 0.f };
	float g[3] { 0.f, 0.f, 0.f };
	float m[3] { 0.f, 0.f, 0.f };
	float lin_acc[3] { 0.f, 0.f, 0.f }; // linear acceleration (acceleration with gravity component subtracted)

	// Other settings
	bool has_connected { false };
	// I2C
	I2C_HandleTypeDef *mpui2c_handle;
	uint8_t i2c_err_;

public:
	bool setup(I2C_HandleTypeDef *i2c_handle);

	void calibrateAccelGyro();
	void calibrateMag(uint16_t sample_count);
	bool isConnectedMPU9250();
	bool isConnectedAK8963();

	bool available();
	bool update();

	float getAcc(const uint8_t i) const;
	float getGyro(const uint8_t i) const;
	float getMag(const uint8_t i) const;
	float getLinearAcc(const uint8_t i) const;

	float getAccBias(const uint8_t i) const;
	float getGyroBias(const uint8_t i) const;
	float getMagBias(const uint8_t i) const;
	float getMagScale(const uint8_t i) const;

	void setAccBias(const float x, const float y, const float z);
	void setGyroBias(const float x, const float y, const float z);
	void setMagBias(const float x, const float y, const float z);
	void setMagScale(const float x, const float y, const float z);
	void setMagneticDeclination(const float d);

	bool selftest();
	void enableWakeOnMotion(uint16_t womThresh_mg);

private:
	void initMPU9250();
	void initAK8963();
	void update_accel_gyro();
	void read_accel_gyro(int16_t *destination);
	void update_mag();
	void read_mag(int16_t *destination);
	void calibrate_acc_gyro_impl();
	void set_acc_gyro_to_calibration();
	void collect_acc_gyro_data_to(float *a_bias, float *g_bias);
	void write_accel_offset();
	void write_gyro_offset();
	void calibrate_mag_impl(uint16_t sample_count);
	void collect_mag_data_to(float *m_bias, float *m_scale,
			uint16_t sample_count);
	bool self_test_impl();
	float get_acc_resolution(const ACCEL_FS_SEL accel_af_sel) const;
	float get_gyro_resolution(const GYRO_FS_SEL gyro_fs_sel) const;
	float get_mag_resolution(const MAG_OUTPUT_BITS mag_output_bits) const;

	void write_byte(uint8_t address, uint8_t subAddress, uint8_t data);
	uint8_t read_byte(uint8_t address, uint8_t subAddress);
	void read_bytes(uint8_t address, uint8_t subAddress, uint8_t count,
			uint8_t *dest);
	void print_i2c_error();
};

#endif /* INC_MPU_H_ */
