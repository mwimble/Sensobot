/*
 * Sensorts.h
 *
 *  Created on: Jan 31, 2015
 *      Author: michaelwimble
 */

#ifndef SENSORS_H_
#define SENSORS_H_

namespace PositionSensors {

class Sensors {
private:
	// SENSOR CALIBRATION
	/*****************************************************************/
	// How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
	// Put MIN/MAX and OFFSET readings for your board here!
	// Accelerometer
	// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
	#define ACCEL_X_MIN ((float) -250)
	#define ACCEL_X_MAX ((float) 250)
	#define ACCEL_Y_MIN ((float) -250)
	#define ACCEL_Y_MAX ((float) 250)
	#define ACCEL_Z_MIN ((float) -250)
	#define ACCEL_Z_MAX ((float) 250)

	// Magnetometer (standard calibration mode)
	// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
	#define MAGN_X_MIN ((float) -600)
	#define MAGN_X_MAX ((float) 600)
	#define MAGN_Y_MIN ((float) -600)
	#define MAGN_Y_MAX ((float) 600)
	#define MAGN_Z_MIN ((float) -600)
	#define MAGN_Z_MAX ((float) 600)

	// Magnetometer (extended calibration mode)
	// Uncommend to use extended magnetometer calibration (compensates hard & soft iron errors)
	//#define CALIBRATION__MAGN_USE_EXTENDED true
	//const float magn_ellipsoid_center[3] = {0, 0, 0};
	//const float magn_ellipsoid_transform[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

	// Gyroscope
	// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
	#define GYRO_AVERAGE_OFFSET_X ((float) 0.0)
	#define GYRO_AVERAGE_OFFSET_Y ((float) 0.0)
	#define GYRO_AVERAGE_OFFSET_Z ((float) 0.0)

	// Sensor calibration scale and offset values
	#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
	#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
	#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
	#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
	#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
	#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

	#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
	#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
	#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
	#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
	#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
	#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))


	// Gain for gyroscope (ITG-3200)
	#define GYRO_GAIN 0.06957 // Same gain on all axes
	#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

	// DCM parameters
	#define Kp_ROLLPITCH 0.02f
	#define Ki_ROLLPITCH 0.00002f
	#define Kp_YAW 1.2f
	#define Ki_YAW 0.00002f

	#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration
	#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
	#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

	/*
	// Calibration example:

	// "accel x,y,z (min/max) = -277.00/264.00  -256.00/278.00  -299.00/235.00"
	#define ACCEL_X_MIN ((float) -277)
	#define ACCEL_X_MAX ((float) 264)
	#define ACCEL_Y_MIN ((float) -256)
	#define ACCEL_Y_MAX ((float) 278)
	#define ACCEL_Z_MIN ((float) -299)
	#define ACCEL_Z_MAX ((float) 235)

	// "magn x,y,z (min/max) = -511.00/581.00  -516.00/568.00  -489.00/486.00"
	//#define MAGN_X_MIN ((float) -511)
	//#define MAGN_X_MAX ((float) 581)
	//#define MAGN_Y_MIN ((float) -516)
	//#define MAGN_Y_MAX ((float) 568)
	//#define MAGN_Z_MIN ((float) -489)
	//#define MAGN_Z_MAX ((float) 486)

	// Extended magn
	#define CALIBRATION__MAGN_USE_EXTENDED true
	const float magn_ellipsoid_center[3] = {91.5, -13.5, -48.1};
	const float magn_ellipsoid_transform[3][3] = {{0.902, -0.00354, 0.000636}, {-0.00354, 0.9, -0.00599}, {0.000636, -0.00599, 1}};

	// Extended magn (with Sennheiser HD 485 headphones)
	//#define CALIBRATION__MAGN_USE_EXTENDED true
	//const float magn_ellipsoid_center[3] = {72.3360, 23.0954, 53.6261};
	//const float magn_ellipsoid_transform[3][3] = {{0.879685, 0.000540833, -0.0106054}, {0.000540833, 0.891086, -0.0130338}, {-0.0106054, -0.0130338, 0.997494}};

	//"gyro x,y,z (current/average) = -40.00/-42.05  98.00/96.20  -18.00/-18.36"
	#define GYRO_AVERAGE_OFFSET_X ((float) -42.05)
	#define GYRO_AVERAGE_OFFSET_Y ((float) 96.20)
	#define GYRO_AVERAGE_OFFSET_Z ((float) -18.36)
	*/

	static const bool DEBUG = false;
	static const int ACCELEROMETER_ADDR = 0x53;
	static const int ACCELEROMETER_BLOCK_START = 0x1D;
	static const int ACCELEROMETER_BLOCK_END = 0x39;
	int	accelerometerFileHandle;
	signed char accelerometerData[ACCELEROMETER_BLOCK_END + 1];

	static const int COMPASS_ADDR = 0x1E;
	static const int COMPASS_BLOCK_START = 0;
	static const int COMPASS_BLOCK_END = 12;
	int	compassFileHandle;
	signed char compassData[COMPASS_BLOCK_END + 1];
	const float compassScale = 0.92;
	struct CompassRawData {
		float x_raw;
		float y_raw;
		float z_raw;
	};

	CompassRawData compassAxisData;

	static const int GYRO_ADDR = 0x68;
	int	gyroFileHandle;

	/*
	Santa Clara California
	Latitude: 37 21' 14.8" N
	Longitude: 121 57' 18.9" W
	Magnetic declination: +13 37'
	Declination is POSITIVE (EAST)
	Inclination: 60  58'
	Magnetic field strength: 48359.9 nT
	 */
	const float declinationRadians = 1.0640690672992;

	// Sensor variables
	float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
	float accel_min[3];
	float accel_max[3];

	float magnetom[3];
	float magnetom_min[3];
	float magnetom_max[3];
	float magnetom_tmp[3];

	float gyro[3];
	float gyro_average[3];
	int gyro_num_samples;

	// DCM variables
	float MAG_Heading;
	float Accel_Vector[3]; // Store the acceleration in a vector
	float Gyro_Vector[3]; // Store the gyros turn rate in a vector
	float Omega_Vector[3]; // Corrected Gyro_Vector data
	float Omega_P[3]; // Omega Proportional correction
	float Omega_I[3]; // Omega Integrator
	float Omega[3];
	float errorRollPitch[3];
	float errorYaw[3];
	float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
	float Temporary_Matrix[3][3];

	// Euler angles
	float yaw;
	float pitch;
	float roll;

	// DCM timing in the main loop
	unsigned long timestamp;
	unsigned long timestamp_old;
	float G_Dt; // Integration time for DCM algorithm

	float abs(float val);

	void initializeAccelerometer();
	void initializeCompass();
	void initializeGyro();
	static void sleepMs(long ms);
	static int openI2C(const char* deviceName, const int deviceAddress);
	static int sendToDevice(
			const int fileHandle,
			const unsigned char* data,
			const int dataLen,
			const int deviceAddress,
			const char* operation);
	static int sendToAndReceiveFromDevice(
			const int fileHandle,
			const unsigned char* sendData,
			const int sendDataLen,
			const signed char* receiveData,
			const int receiveDataLen,
			const int deviceAddress,
			const char* operation);

	float getCompassHeading();
	void readGyro();
	void readAccel();
	void readMagn();
	void readSensors();
	void resetSensorFusion();
	void normalize(void);
	float constrain(float x, float a, float b);
	void driftCorrection(void);
	void matrixUpdate(void);
	void eulerAngles(void);
	static float vectorDotProduct(const float v1[3], const float v2[3]);
	static void vectorCrossProduct(float out[3], const float v1[3], const float v2[3]);
	static void vectorScale(float out[3], const float v[3], float scale);
	static void vectorAdd(float out[3], const float v1[3], const float v2[3]);
	static void matrixMultiply(const float a[3][3], const float b[3][3], float out[3][3]);
	static void matrixVectorMultiply(const float a[3][3], const float b[3], float out[3]);
	static void initRotationMatrix(float m[3][3], float yaw, float pitch, float roll);
	void compassHeading();

public:
	enum Axis {
		X,
		Y,
		Z
	};

	Sensors();
	virtual ~Sensors();

	void dumpAccelerometerSensorBlock();

	void getAccelerometerBlock();

	float gForce(Axis axis);

	void dumpCompassSensorBlock();

	void getCompassBlock();

};

} /* namespace PositionSensors */

#endif /* SENSORS_H_ */
