/*
 * Sensors.cpp
 *
 *  Created on: Jan 31, 2015
 *      Author: michaelwimble
 */

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include "Sensors.h"

namespace PositionSensors {

Sensors::Sensors() {
	//DCM_Matrix = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	//Update_Matrix = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
	initializeAccelerometer();
	initializeCompass();
	initializeGyro();
}

float Sensors::abs(float val) {
	return val < 0 ? -val : val;
}

Sensors::~Sensors() {
	if (accelerometerFileHandle && ((close(accelerometerFileHandle)) != 0)) {
		fprintf(stderr, "[Sensors::~Sensor] accelerometerFileHandle close error: %s\n", strerror(errno));
	}

	if (compassFileHandle && ((close(compassFileHandle)) != 0)) {
		fprintf(stderr, "[Sensors::~Sensor] compassFileHandle close error: %s\n", strerror(errno));
	}

	if (gyroFileHandle && ((close(gyroFileHandle)) != 0)) {
		fprintf(stderr, "[Sensors::~Sensor] gyroFileHandle close error: %s\n", strerror(errno));
	}
}

void Sensors::sleepMs(long ms) {
	struct timespec nanoTime = {0, ms * 1000000};
	struct timespec remaining;
	nanosleep(&nanoTime, &remaining);
}

int Sensors::openI2C(const char* deviceName, const int deviceAddress) {
	int result;
	char filename[16];
	sprintf(filename, "/dev/i2c-2");
	if (Sensors::DEBUG) {
		fprintf(stderr, "[Sensors::openI2C] Opening %s\n", filename);
	}

	if ((result = open(filename, O_RDWR)) < 0) {
		fprintf(stderr, "[Sensors::openI2C] %s open error: %s\n", deviceName, strerror(errno));
		return -1;
	}

	if (ioctl(result, I2C_SLAVE, deviceAddress) < 0) {
		fprintf(stderr, "[Sensors::openI2C] %s ioctl error: %s\n", deviceName, strerror(errno));
		return -1;
	}

	return result;
}

int Sensors::sendToDevice(
		const int fileHandle,
		const unsigned char* data,
		const int dataLen,
		const int deviceAddress,
		const char* operation) {
	struct i2c_rdwr_ioctl_data msgset;
	struct i2c_msg msgs[2];

	if (Sensors::DEBUG) {
		fprintf(stderr, "[Sensors::sendToDevice] fileHandle: %d\n", fileHandle);
		fprintf(stderr, "...data[0]: %02X\n", data[0]);
		fprintf(stderr, "...dataLen: %d\n", dataLen);
		fprintf(stderr, "...deviceAddress: %2X\n", deviceAddress);
		fprintf(stderr, "...operation: %s\n", operation);
	}

	msgs[0].addr = deviceAddress;
	msgs[0].flags = 0;
	msgs[0].len = dataLen;
	msgs[0].buf = (__u8*) data;
	msgset .nmsgs = 1;
	msgset.msgs = msgs;
	if (Sensors::DEBUG) {
		if (int xxx = ioctl(fileHandle, I2C_RDWR, &msgset) < 0) {
			fprintf(stderr, "[Sensors::sendToDevice] (%d) %s error: %s\n", xxx, operation, strerror(errno));
			return -1;
		}
	}

	Sensors::sleepMs(5);
	if (Sensors::DEBUG) {
		fprintf(stderr, "[Sensors::sendToDevice] success of %s\n", operation);
	}

	return 0;
}

int Sensors::sendToAndReceiveFromDevice(
		const int fileHandle,
		const unsigned char* sendData,
		const int sendDataLen,
		const signed char* receiveData,
		const int receiveDataLen,
		const int deviceAddress,
		const char* operation) {
	struct i2c_rdwr_ioctl_data msgset;
	struct i2c_msg msgs[2];

	if (Sensors::DEBUG) {
		fprintf(stderr, "[Sensors::sendToAndReceiveFromDevice] fileHandle: %d\n", fileHandle);
		fprintf(stderr, "...sendData[0]: %02X\n", sendData[0]);
		fprintf(stderr, "...sendDataLen: %d\n", sendDataLen);
		fprintf(stderr, "...deviceAddress: %2X\n", deviceAddress);
		fprintf(stderr, "...operation: %s\n", operation);
	}

	msgs[0].addr = deviceAddress;
	msgs[0].flags = 0;
	msgs[0].len = sendDataLen;
	msgs[0].buf = (__u8*) sendData;

	msgs[1].addr = deviceAddress;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = receiveDataLen;
	msgs[1].buf = (__u8*) receiveData;

	msgset .nmsgs = 2;
	msgset.msgs = msgs;
	if (ioctl(fileHandle, I2C_RDWR, &msgset) < 0) {
		fprintf(stderr, "[Sensors::sendToAndReceiveFromDevice] %s error: %s\n", operation, strerror(errno));
		return -1;
	}

	if (Sensors::DEBUG) {
		fprintf(stderr, "...raw data block: ");
		int listOffset = 0;
		int i;
		for (i = 0; i < receiveDataLen; i++) {
			fprintf(stderr, "%02X: %02X, ", listOffset++, receiveData[i]);
		}

		fprintf(stderr, "\n");
	}

	Sensors::sleepMs(5);
	return 0;
}

void Sensors::initializeAccelerometer() {
	if ((accelerometerFileHandle = Sensors::openI2C("Accelerometer", ACCELEROMETER_ADDR)) <= 0) {
		return;
	}

	unsigned char data[2];
	data[0] = 0x2D; // Power register
	data[1] = 0x08; // Measure mode.
	int result;
	if ((result = Sensors::sendToDevice(accelerometerFileHandle, data, 2, Sensors::ACCELEROMETER_ADDR, "Accelerometer Measurement Mode")) < 0) {
		fprintf(stderr, "[Sensors::initializeAccelerometer] #1 result: %d\n", result);
		return;
	}

	data[0] = 0x31; // Data format register
	data[1] = 0x08; // Full resolution.
	if ((result = Sensors::sendToDevice(accelerometerFileHandle, data, 2, Sensors::ACCELEROMETER_ADDR, "Accelerometer Full Resolution")) < 0) {
		fprintf(stderr, "[Sensors::initializeAccelerometer] #2 result: %d\n", result);
		return;
	}

	data[0] = 0x38; // FIFO_CTL
	data[1] = 0x84; // Stream watermark level.
	if ((result = Sensors::sendToDevice(accelerometerFileHandle, data, 2, Sensors::ACCELEROMETER_ADDR, "Accelerometer Stream Watermark")) < 0) {
		fprintf(stderr, "[Sensors::initializeAccelerometer] #3 result: %d\n", result);
		return;
	}

	data[0] = 0x2C; // Rate
	data[1] = 0x09; // 50Hz.
	if ((result = Sensors::sendToDevice(accelerometerFileHandle, data, 2, Sensors::ACCELEROMETER_ADDR, "Accelerometer 50Hz Rate")) < 0) {
		fprintf(stderr,"[Sensors::initializeAccelerometer] #4 result: %d\n", result);
		return;
	}

	if (Sensors::DEBUG) {
		fprintf(stderr, "[Sensors::initializeAcceleromete] Accelerometer initialized\n");
	}
}

void Sensors::getAccelerometerBlock() {
	unsigned char data[1] = { ACCELEROMETER_BLOCK_START };
	if ((Sensors::sendToAndReceiveFromDevice(
			accelerometerFileHandle,
			data,
			sizeof(data),
			&accelerometerData[ACCELEROMETER_BLOCK_START],
			ACCELEROMETER_BLOCK_END - ACCELEROMETER_BLOCK_START + 1,
			Sensors::ACCELEROMETER_ADDR, "Accelerometer Block Read")) < 0) {
		fprintf(stderr, "[Sensors::getAccelerometerBlock] fail\n");
		return;
	}

	if (Sensors::DEBUG) {
		fprintf(stderr, "[Sensors::getAccelerometerBlock] Accelerometer raw data block: ");
		int listOffset = ACCELEROMETER_BLOCK_START;
		for (unsigned int i = 0; i < (ACCELEROMETER_BLOCK_END - ACCELEROMETER_BLOCK_START + 1); i++) {
			fprintf(stderr, "%02X: %02X, ", listOffset++,
					accelerometerData[ACCELEROMETER_BLOCK_START + i]);
		}

		fprintf(stderr, "\n");
	}
}

void Sensors::dumpAccelerometerSensorBlock() {
	printf("THRESH_TAP:     0x%02X (%3.2fmg)\n", accelerometerData[0x1D],
			accelerometerData[0x1D] * 62.5);
	printf("OFSX:           0x%02X (%3.2fmg)\n", accelerometerData[0x1E],
			((signed char) accelerometerData[0x1E]) * 15.6);
	printf("OFSY:           0x%02X (%3.2fmg)\n", accelerometerData[0x1F],
			((signed char) accelerometerData[0x1F]) * 15.6);
	printf("OFSZ:           0x%02X (%3.2fmg)\n", accelerometerData[0x20],
			((signed char) accelerometerData[0x20]) * 15.6);
	printf("DUR:            0x%02X (%3.2fms)\n", accelerometerData[0x21],
			accelerometerData[0x21] * 625.0 / 1000.0);
	printf("Latent:         0x%02X (%3.2fms)\n", accelerometerData[0x22],
			accelerometerData[0x22] * 1.25);
	printf("Window:         0x%02X (%3.2fms)\n", accelerometerData[0x23],
			accelerometerData[0x23] * 1.25);
	printf("THRESH_ACT:     0x%02X (%3.2fmg)\n", accelerometerData[0x24],
			accelerometerData[0x24] * 62.5);
	printf("THRESH_INACT:   0x%02X (%3.2fmg)\n", accelerometerData[0x25],
			accelerometerData[0x25] * 62.5);
	printf("TIME_INACT:     0x%02X (%ds)\n", accelerometerData[0x26],
			accelerometerData[0x26]);
	printf("ACT_INACT_CTL:  0x%02X (ACT:%s, ActX:%s, ActY:%s, ActZ:%s, INACT:%s, InactX:%s, InactY:%s, InactZZ:%s)\n",
			accelerometerData[0x27],
			(accelerometerData[0x27] & 0x80 ? "AC" : "DC"),
			(accelerometerData[0x27] & 0x40 ? "ENA" : "dis"),
			(accelerometerData[0x27] & 0x20 ? "ENA" : "dis"),
			(accelerometerData[0x27] & 0x10 ? "ENA" : "dis"),
			(accelerometerData[0x27] & 0x08 ? "AC" : "DC"),
			(accelerometerData[0x27] & 0x04 ? "ENA" : "dis"),
			(accelerometerData[0x27] & 0x02 ? "ENA" : "dis"),
			(accelerometerData[0x27] & 0x01 ? "ENA" : "dis"));
	printf("THRESH_FF:      0x%02X (%3.2fmg)\n", accelerometerData[0x28],
			accelerometerData[0x28] * 62.5);
	printf("TIME_FF:        0x%02X (%dms)\n", accelerometerData[0x29],
			(accelerometerData[0x29] >> 4) * 5);
	printf("TAP_AXES:       0x%02X (Supp:%s, TAPXen:%s, TAPYen:%s, TAPZen:%s)\n",
			accelerometerData[0x2A],
			(accelerometerData[0x2A] & 0x08 ? "Y" : "n"),
			(accelerometerData[0x2A] & 0x04 ? "Y" : "n"),
			(accelerometerData[0x2A] & 0x02 ? "Y" : "n"),
			(accelerometerData[0x2A] & 0x01 ? "Y" : "n"));
	printf("ACC_TAP_STATUS: 0x%02X (Sources ACTX:%s, ACTY:%s, ACTZ:%s, Asleep:%s, TAPX: %s, TAPY:%s, TAPZ:%s)\n",
			accelerometerData[0x2B],
			(accelerometerData[0x2B] & 0x40 ? "Y" : "n"),
			(accelerometerData[0x2B] & 0x20 ? "Y" : "n"),
			(accelerometerData[0x2B] & 0x10 ? "Y" : "n"),
			(accelerometerData[0x2B] & 0x08 ? "Y" : "n"),
			(accelerometerData[0x2B] & 0x04 ? "Y" : "n"),
			(accelerometerData[0x2B] & 0x02 ? "Y" : "n"),
			(accelerometerData[0x2B] & 0x01 ? "Y" : "n"));
	printf("BW_RATE:        0x%02X (LOWP:%s, RATE:%d)\n", accelerometerData[0x2C],
			(accelerometerData[0x2C] & 0x10 ? "Y" : "n"),
			accelerometerData[0x2C] & 0x0F);
	printf("POWER_CTL:      0x%02X (LINK:%s, AUTO_SLEEP:%s, Measure:%s, Sleep:%s, Wakeup:%d)\n",
			accelerometerData[0x2D],
			(accelerometerData[0x2D] & 0x20 ? "Y" : "n"),
			(accelerometerData[0x2D] & 0x10 ? "Y" : "n"),
			(accelerometerData[0x2D] & 0x08 ? "meas" : "stby"),
			(accelerometerData[0x2D] & 0x04 ? "SLEEP" : "norm"),
			accelerometerData[0x2D] & 0x03);
	printf("INT_ENABLE:     0x%02X (DataReady:%s, SingleTap:%s, DoubleTap:%s, Activity:%s, Inactivity:%s, FreeFall:%s, WaterMark:%s, Overrun:%s)\n",
			accelerometerData[0x2E],
			(accelerometerData[0x2E] & 0x80 ? "Y" : "n"),
			(accelerometerData[0x2E] & 0x40 ? "Y" : "n"),
			(accelerometerData[0x2E] & 0x20 ? "Y" : "n"),
			(accelerometerData[0x2E] & 0x10 ? "Y" : "n"),
			(accelerometerData[0x2E] & 0x08 ? "Y" : "n"),
			(accelerometerData[0x2E] & 0x04 ? "Y" : "n"),
			(accelerometerData[0x2E] & 0x02 ? "Y" : "n"),
			(accelerometerData[0x2E] & 0x01 ? "Y" : "n"));
	printf("INT_MAP:        0x%02X (DataReady:%s, SingleTap:%s, DoubleTap:%s, Activity:%s, Inactivity:%s, FreeFall:%s, WaterMark:%s, Overrun:%s)\n",
			accelerometerData[0x2F],
			(accelerometerData[0x2F] & 0x80 ? "INT2" : "INT1"),
			(accelerometerData[0x2F] & 0x40 ? "INT2" : "INT1"),
			(accelerometerData[0x2F] & 0x20 ? "INT2" : "INT1"),
			(accelerometerData[0x2F] & 0x10 ? "INT2" : "INT1"),
			(accelerometerData[0x2F] & 0x08 ? "INT2" : "INT1"),
			(accelerometerData[0x2F] & 0x04 ? "INT2" : "INT1"),
			(accelerometerData[0x2F] & 0x02 ? "INT2" : "INT1"),
			(accelerometerData[0x2F] & 0x01 ? "INT2" : "INT1"));
	printf("INT_SOURCE:     0x%02X (DataReady:%s, SingleTap:%s, DoubleTap:%s, Activity:%s, Inactivity:%s, FreeFall:%s, WaterMark:%s, Overrun:%s)\n",
			accelerometerData[0x30],
			(accelerometerData[0x30] & 0x80 ? "Y" : "n"),
			(accelerometerData[0x30] & 0x40 ? "Y" : "n"),
			(accelerometerData[0x30] & 0x20 ? "Y" : "n"),
			(accelerometerData[0x30] & 0x10 ? "Y" : "n"),
			(accelerometerData[0x30] & 0x08 ? "Y" : "n"),
			(accelerometerData[0x30] & 0x04 ? "Y" : "n"),
			(accelerometerData[0x30] & 0x02 ? "Y" : "n"),
			(accelerometerData[0x30] & 0x01 ? "Y" : "n"));
	const char* gRange[] = { "2g", "4g", "8g", "16g" };
	printf("DATA_FORMAT:    0x%02X (SelfTest:%s, SPI:%s, IntInvert:%s, FullRes:%s, Justify:%s, Range%s\n",
			accelerometerData[0x31],
			(accelerometerData[0x31] & 0x80 ? "Y" : "n"),
			(accelerometerData[0x31] & 0x40 ? "3wire" : "4wire"),
			(accelerometerData[0x31] & 0x20 ? "actHigh" : "actLow"),
			(accelerometerData[0x31] & 0x08 ? "4mg/bit" : "10-bit"),
			(accelerometerData[0x31] & 0x04 ? "left" : "right"),
			gRange[accelerometerData[0x31] & 0x03]);
	short value;
	value = accelerometerData[0x33] << 8 | accelerometerData[0x32];
	printf("X:             %d 0x%04X (%.4fg)\n", value, (unsigned short) value, gForce(X));
	value = accelerometerData[0x35] << 8 | accelerometerData[0x34];
	printf("Y:             %d 0x%04X (%.4fg)\n", value, (unsigned short) value, gForce(Y));
	value = accelerometerData[0x37] << 8 | accelerometerData[0x36];
	printf("Z:             %d 0x%04X (%.4fg)\n", value, (unsigned short) value, gForce(Z));
	const char* fifoMode[] = { "bypass", "FIFO", "stream", "trigger" };
	printf("FIFO_MODE:     0x%02X (Mode:%s, trigger:%s, samples:%d)\n",
			accelerometerData[0x38], fifoMode[accelerometerData[0x38] >> 6],
			(accelerometerData[0x38] & 0x20 ? "int2" : "int1"),
			accelerometerData[0x38] & 0x1F);
	printf("FIFO_STATUS:   0x%02X (triggered:%s, entries:%d)\n",
			accelerometerData[0x39],
			(accelerometerData[0x39] & 0x80 ? "Y" : "n"),
			accelerometerData[0x39] & 0x3F);
}

void Sensors::initializeCompass() {
	if ((compassFileHandle = Sensors::openI2C("Magnetometer", COMPASS_ADDR)) <= 0) {
		return;
	}

	unsigned char data[] = {
		0x00,	// Register address to start writing.
		0x70,	// 011 => 8 samples averaged, 100 => 15 Hz output rate, 00 => Normal config.
		0x20,   // 001 => +/- 1.3 Ga (default), 0.92mG/LSb, output range F800->007F (-2048->2047).
		0x00    // Continue-measurement mode.
	};

	int xxx;
	if ((xxx = Sensors::sendToDevice(compassFileHandle, data,  sizeof(data), Sensors::COMPASS_ADDR, "Compass Setup")) < 0) {
		fprintf(stderr, "[Sensors::initializeCompass] (%d) fail\n", xxx);
		return;
	}

	if (Sensors::DEBUG) {
		fprintf(stderr, "[Sensors::initializeCompass] Compass initialized\n");
	}
}

void Sensors::getCompassBlock() {
	unsigned char data[2];

	data[0] = 0;
	if ((Sensors::sendToAndReceiveFromDevice(
			compassFileHandle,
			data,
			1,
			compassData,
			3,
			Sensors::COMPASS_ADDR, "Compass Block Read pt1")) < 0) {
		return;
	}

	data[0] = 3;
	if ((Sensors::sendToAndReceiveFromDevice(
			compassFileHandle,
			data,
			1,
			&compassData[3],
			6,
			Sensors::COMPASS_ADDR, "Compass Block Read pt2")) < 0) {
		return;
	}

	data[0] = 9;
	if ((Sensors::sendToAndReceiveFromDevice(
			compassFileHandle,
			data,
			1,
			&compassData[9],
			4,
			Sensors::COMPASS_ADDR, "Compass Block Read pt3")) < 0) {
		return;
	}

	// Now convert coordinates frames to be more interesting.
	// Set temp = y; y = -z; z = -temp;
	short oldY = (compassData[5] << 8) || compassData[6]; // old Y
	short oldZ = (compassData[7] << 8) || compassData[8]; // old Z
	oldY *= -1;
	oldZ *= -1;
	compassData[5] = oldZ >> 8;
	compassData[6] = oldZ && 0xFF;
	compassData[7] = oldY >> 8;
	compassData[8] = oldY && 0xFF;

	if (Sensors::DEBUG) {
		fprintf(stderr, "[Sensors::getCompassBlock] Compass raw data block: ");
		int listOffset = COMPASS_BLOCK_START;
		for (unsigned int i = 0; i < (COMPASS_BLOCK_END - COMPASS_BLOCK_START + 1); i++) {
			fprintf(stderr, "%02X: %02X, ", listOffset++, compassData[COMPASS_BLOCK_START + i]);
		}

		fprintf(stderr, "\n");
	}
}

void Sensors::dumpCompassSensorBlock() {
	const int samplesAveraged[] = {1, 2, 4, 8};
	const char* outputRage[] = {
		"0.75Hz", "1.5Hz", "3Hz", "7.5Hz",
		"15Hz", "30Hz", "75Hz", "???"
	};
	const char* measurementMode[] = {
		"Normal", "+bias", "-bias", "???"
	};
	const char* gain[] = {
		"0.88", "1.3", "1.9", "2.5"
		"4.0", "4.7", "5.6", "8.1"
	};
	const char* mode[] = {
		"continuous", "single", "idle(10)", "idle(11)"
	};

	printf("CRA:            0x%02X (avgd samp:%d, rate:%s, config:%s\n",
		   compassData[0x00],
		   samplesAveraged[(compassData[0x00] & 0x60) >> 5],
		   outputRage[(compassData[0x00] & 0x1C) >> 2],
		   measurementMode[compassData[0x00] & 0x3]);
	printf("CRB:            0x%02X (+/-%s Ga)\n",
		   compassData[0x01],
		   gain[compassData[0x01] >> 5]);
	printf("MODE:           0x%02X (%s)\n",
		   compassData[0x02],
		   mode[compassData[0x02]]);
	short value = (compassData[0x03] << 8) | compassData[0x04];
	compassAxisData.x_raw = value;
	printf("X:              0x%02X 0x%02X (%d)\n", compassData[0x03], compassData[0x04], value);
	value = (compassData[0x05] << 8) | compassData[0x06];
	compassAxisData.y_raw = value;
	printf("Y:              0x%02X 0x%02X (%d)\n", compassData[0x05], compassData[0x06], value);
	value = (compassData[0x07] << 8) | compassData[0x08];
	compassAxisData.z_raw = value;
	printf("Z:              0x%02X 0x%02X (%d)\n", compassData[0x07], compassData[0x08], value);
	printf("STATUS:         0x%02X (lock:%s, rdy:%s)\n",
		   compassData[0x09],
		   compassData[0x09] & 0x02 ? "Y" : "n",
		   compassData[0x09] & 0x01 ? "Y" : "n");
	printf("ID:             0x%02X%02X%02X\n",
		   compassData[0x0A],
		   compassData[0x0B],
		   compassData[0x0C]);
}

void Sensors::readMagn() {
	unsigned char data[2];
	data[0] = 3;
	if ((Sensors::sendToAndReceiveFromDevice(
			compassFileHandle,
			data,
			1,
			&compassData[3],
			6,
			Sensors::COMPASS_ADDR, "Compass Block Read pt2")) < 0) {
		return;
	}

}

void Sensors::readSensors() {
	readGyro();
	readAccel();
	readMagn();
}

void Sensors::resetSensorFusion() {
	  float temp1[3];
	  float temp2[3];
	  float xAxis[] = {1.0f, 0.0f, 0.0f};

	  readSensors();
	  //timestamp = millis();

	  // GET PITCH
	  // Using y-z-plane-component/x-component of gravity vector
	  pitch = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));

	  // GET ROLL
	  // Compensate pitch of gravity vector
	  vectorCrossProduct(temp1, accel, xAxis);
	  vectorCrossProduct(temp2, xAxis, temp1);
	  // Normally using x-z-plane-component/y-component of compensated gravity vector
	  // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
	  // Since we compensated for pitch, x-z-plane-component equals z-component:
	  roll = atan2(temp2[1], temp2[2]);

	  // GET YAW
	  compassHeading();
	  yaw = MAG_Heading;

	  // Init rotation matrix
	  initRotationMatrix(DCM_Matrix, yaw, pitch, roll);
}
void Sensors::compassHeading() {
  float mag_x;
  float mag_y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;

  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);

  // Tilt compensated magnetic field X
  mag_x = magnetom[0] * cos_pitch + magnetom[1] * sin_roll * sin_pitch + magnetom[2] * cos_roll * sin_pitch;
  // Tilt compensated magnetic field Y
  mag_y = magnetom[1] * cos_roll - magnetom[2] * sin_roll;
  // Magnetic Heading
  MAG_Heading = atan2(-mag_y, mag_x);
}

float Sensors::getCompassHeading() {
	float heading = atan2(compassAxisData.y_raw, compassAxisData.x_raw) + declinationRadians;
	printf("compassAxisData.y_raw: %f, x_raw: %f, atan: %f\n", compassAxisData.y_raw, compassAxisData.x_raw, heading);
	if (heading < 0) {
		heading += (2 * M_PI);
	}

	if (heading > (2 * M_PI)) {
		heading -= (2 * M_PI);
	}

	return heading * (180.0 / M_PI);
}

float Sensors::gForce(Axis axis) {
	short value = 0.0;
	if (axis == X) {
		value = (accelerometerData[0x33] << 8 | accelerometerData[0x32]);
	}

	if (axis == Y) {
		value = (accelerometerData[0x35] << 8 | accelerometerData[0x34]);
	}

	if (axis == Z) {
		value = (accelerometerData[0x37] << 8 | accelerometerData[0x36]);
	}

	return value / 256.0;
}

void Sensors::initializeGyro() {
	if ((gyroFileHandle = Sensors::openI2C("Gyro", GYRO_ADDR)) <= 0) {
		return;
	}

	int result;
	unsigned char data[2];
	data[0] = 0x3E; // Power management
	data[1] = 0x80; // Reset to defaults
	if ((result = Sensors::sendToDevice(accelerometerFileHandle, data, 2, Sensors::ACCELEROMETER_ADDR, "Accelerometer 50Hz Rate")) < 0) {
		fprintf(stderr,"[Sensors::initializeGyro] Reset power management result: %d\n", result);
		return;
	}

	data[0] = 0x16; // DLPF
	data[1] = 0x1B; // Full scale. Refresh at 42Hz
	if ((result = Sensors::sendToDevice(accelerometerFileHandle, data, 2, Sensors::ACCELEROMETER_ADDR, "Accelerometer 50Hz Rate")) < 0) {
		fprintf(stderr,"[Sensors::initializeGyro] DLPF result: %d\n", result);
		return;
	}

	data[0] = 0x15; // Sample Rate Divider
	data[1] = 0x0A; // 50Hz
	if ((result = Sensors::sendToDevice(accelerometerFileHandle, data, 2, Sensors::ACCELEROMETER_ADDR, "Accelerometer 50Hz Rate")) < 0) {
		fprintf(stderr,"[Sensors::initializeGyro] Sample Rate Divider result: %d\n", result);
		return;
	}

	data[0] = 0x3E; // Poser management
	data[1] = 0x00; // Clock sync to internal osc.
	if ((result = Sensors::sendToDevice(accelerometerFileHandle, data, 2, Sensors::ACCELEROMETER_ADDR, "Accelerometer 50Hz Rate")) < 0) {
		fprintf(stderr,"[Sensors::initializeGyro] Clock sync to internal: %d\n", result);
		return;
	}

	if (Sensors::DEBUG) {
		fprintf(stderr, "[Sensors::initializeGyro] Gyroscope initialized\n");
	}
}

/* This file is part of the Razor AHRS Firmware */

// DCM algorithm

/**************************************************/
void Sensors::normalize(void) {
  float error=0;
  float temporary[3][3];
  float renorm=0;

  error= -vectorDotProduct(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

  vectorScale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
  vectorScale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19

  vectorAdd(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
  vectorAdd(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19

  vectorCrossProduct(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20

  renorm= .5 *(3 - vectorDotProduct(&temporary[0][0],&temporary[0][0])); //eq.21
  vectorScale(&DCM_Matrix[0][0], &temporary[0][0], renorm);

  renorm= .5 *(3 - vectorDotProduct(&temporary[1][0],&temporary[1][0])); //eq.21
  vectorScale(&DCM_Matrix[1][0], &temporary[1][0], renorm);

  renorm= .5 *(3 - vectorDotProduct(&temporary[2][0],&temporary[2][0])); //eq.21
  vectorScale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

float Sensors::constrain(float x, float a, float b) {
	if (x < a) return a;
	else if (x > b) return b;
	else return x;
}

/**************************************************/
void Sensors::driftCorrection(void) {
  float mag_heading_x;
  float mag_heading_y;
  float errorCourse;
  //Compensation the Roll, Pitch and Yaw drift.
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  float Accel_magnitude;
  float Accel_weight;


  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  //

  vectorCrossProduct(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
  vectorScale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);

  vectorScale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
  vectorAdd(Omega_I,Omega_I,Scaled_Omega_I);

  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading

  mag_heading_x = cos(MAG_Heading);
  mag_heading_y = sin(MAG_Heading);
  errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
  vectorScale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

  vectorScale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01proportional of YAW.
  vectorAdd(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.

  vectorScale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
  vectorAdd(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
}

void Sensors::matrixUpdate(void)
{
  Gyro_Vector[0]=GYRO_SCALED_RAD(gyro[0]); //gyro x roll
  Gyro_Vector[1]=GYRO_SCALED_RAD(gyro[1]); //gyro y pitch
  Gyro_Vector[2]=GYRO_SCALED_RAD(gyro[2]); //gyro z yaw

  Accel_Vector[0]=accel[0];
  Accel_Vector[1]=accel[1];
  Accel_Vector[2]=accel[2];

  vectorAdd(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
  vectorAdd(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term

#if DEBUG__NO_DRIFT_CORRECTION == true // Do not use drift correction
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Gyro_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Gyro_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Gyro_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Gyro_Vector[0];
  Update_Matrix[2][0]=-G_Dt*Gyro_Vector[1];
  Update_Matrix[2][1]=G_Dt*Gyro_Vector[0];
  Update_Matrix[2][2]=0;
#else // Use drift correction
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
  Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
  Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
  Update_Matrix[2][2]=0;
#endif

  matrixMultiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c

  for(int x=0; x<3; x++) //Matrix Addition (update)
  {
    for(int y=0; y<3; y++)
    {
      DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
    }
  }
}

void Sensors::eulerAngles(void) {
  pitch = -asin(DCM_Matrix[2][0]);
  roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
  yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
}

/* This file is part of the Razor AHRS Firmware */

// Computes the dot product of two vectors
float Sensors::vectorDotProduct(const float v1[3], const float v2[3]) {
  float result = 0;

  for(int c = 0; c < 3; c++)
  {
    result += v1[c] * v2[c];
  }

  return result;
}

// Computes the cross product of two vectors
// out has to different from v1 and v2 (no in-place)!
void Sensors::vectorCrossProduct(float out[3], const float v1[3], const float v2[3]) {
  out[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
  out[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
  out[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

// Multiply the vector by a scalar
void Sensors::vectorScale(float out[3], const float v[3], float scale) {
  for(int c = 0; c < 3; c++) {
    out[c] = v[c] * scale;
  }
}

// Adds two vectors
void Sensors::vectorAdd(float out[3], const float v1[3], const float v2[3]) {
  for(int c = 0; c < 3; c++) {
    out[c] = v1[c] + v2[c];
  }
}

// Multiply two 3x3 matrices: out = a * b
// out has to different from a and b (no in-place)!
void Sensors::matrixMultiply(const float a[3][3], const float b[3][3], float out[3][3]) {
  for(int x = 0; x < 3; x++) { // rows
    for(int y = 0; y < 3; y++)  // columns
    {
      out[x][y] = a[x][0] * b[0][y] + a[x][1] * b[1][y] + a[x][2] * b[2][y];
    }
  }
}

// Multiply 3x3 matrix with vector: out = a * b
// out has to different from b (no in-place)!
void Sensors::matrixVectorMultiply(const float a[3][3], const float b[3], float out[3]) {
  for(int x = 0; x < 3; x++) {
    out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
  }
}

// Init rotation matrix using euler angles
void Sensors::initRotationMatrix(float m[3][3], float yaw, float pitch, float roll) {
  float c1 = cos(roll);
  float s1 = sin(roll);
  float c2 = cos(pitch);
  float s2 = sin(pitch);
  float c3 = cos(yaw);
  float s3 = sin(yaw);

  // Euler angles, right-handed, intrinsic, XYZ convention
  // (which means: rotate around body axes Z, Y', X'')
  m[0][0] = c2 * c3;
  m[0][1] = c3 * s1 * s2 - c1 * s3;
  m[0][2] = s1 * s3 + c1 * c3 * s2;

  m[1][0] = c2 * s3;
  m[1][1] = c1 * c3 + s1 * s2 * s3;
  m[1][2] = c1 * s2 * s3 - c3 * s1;

  m[2][0] = -s2;
  m[2][1] = c2 * s1;
  m[2][2] = c1 * c2;
}

} /* namespace PositionSensors */

