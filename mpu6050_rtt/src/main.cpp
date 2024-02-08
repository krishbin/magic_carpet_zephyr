/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <zephyr/logging/log.h>
#include "Kalman.h"

LOG_MODULE_REGISTER(cdc_acm_echo, LOG_LEVEL_INF);

#define RAD_TO_DEG 57.29578

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double temp;

Kalman kalmanX;
Kalman kalmanY;
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

static bool rx_throttled;

static const char *now_str(void)
{
	static char buf[16]; /* ...HH:MM:SS.MMM */
	uint32_t now = k_uptime_get_32();
	unsigned int ms = now % MSEC_PER_SEC;
	unsigned int s;
	unsigned int min;
	unsigned int h;

	now /= MSEC_PER_SEC;
	s = now % 60U;
	now /= 60U;
	min = now % 60U;
	now /= 60U;
	h = now;

	snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u",
			 h, min, s, ms);
	return buf;
}

static int process_mpu6050(const struct device *dev)
{
	struct sensor_value temperature;
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	int rc = sensor_sample_fetch(dev);

	if (rc == 0)
	{
		rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ,
								accel);
	}
	if (rc == 0)
	{
		rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ,
								gyro);
	}
	if (rc == 0)
	{
		rc = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP,
								&temperature);
	}

	accX = sensor_value_to_double(&accel[0]),
	accY = sensor_value_to_double(&accel[1]),
	accZ = sensor_value_to_double(&accel[2]),
	gyroX = sensor_value_to_double(&gyro[0]),
	gyroY = sensor_value_to_double(&gyro[1]),
	gyroZ = sensor_value_to_double(&gyro[2]);
	temp = sensor_value_to_double(&temperature);

	if (rc == 0)
	{
#ifdef DEBUG
		printf("[%s]:%g Cel"
			   "  accel %f %f %f m/s/s"
			   "  gyro  %f %f %f rad/s\n",
			   now_str(),
			   temp,
			   accX, accY, accZ,
			   gyroX, gyroY, gyroZ);
#endif
	}
	else
	{
		LOG_ERR("sample fetch/get failed: %d\n", rc);
	}

	return rc;
}

#ifdef CONFIG_MPU6050_TRIGGER
static struct sensor_trigger trigger;

static void handle_mpu6050_drdy(const struct device *dev,
								const struct sensor_trigger *trig)
{
	int rc = process_mpu6050(dev);

	if (rc != 0)
	{
		printf("cancelling trigger due to failure: %d\n", rc);
		(void)sensor_trigger_set(dev, trig, NULL);
		return;
	}
}
#endif /* CONFIG_MPU6050_TRIGGER */


int main(void)
{
	const struct device *const mpu6050 = DEVICE_DT_GET_ONE(invensense_mpu6050);
	// const struct device *const qdec = DEVICE_DT_GET(DT_ALIAS(qdec0));

	if (!device_is_ready(mpu6050))
	{
		LOG_WRN("Device %s is not ready\n", mpu6050->name);
		return 0;
	}

	if (!device_is_ready(qdec)) {
		LOG_WRN("Device qdec is not ready\n");
		return 0;
	}

#ifdef CONFIG_MPU6050_TRIGGER
	trigger = (struct sensor_trigger){
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};
	if (sensor_trigger_set(mpu6050, &trigger,
						   handle_mpu6050_drdy) < 0)
	{
		LOG_WRN("Cannot configure trigger\n");
		return 0;
	}
	printf("Configured for triggered sampling.\n");
#endif
	process_mpu6050(mpu6050);
#ifdef RESTRICT_PITCH // Eq. 25 and 26
	double roll = atan2(accY, accZ) * RAD_TO_DEG;
	double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
	double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
	kalmanX.setAngle(roll); // Set starting angle
	kalmanY.setAngle(pitch);

	uint32_t start_cycles, end_cycles, delta_cycles;
	uint64_t delta_time_us;
	double dt;
	double gyroXrate, gyroYrate;

	start_cycles = k_cycle_get_32();
	while (!IS_ENABLED(CONFIG_MPU6050_TRIGGER))
	{
		end_cycles = k_cycle_get_32();
		delta_cycles = end_cycles - start_cycles;
		delta_time_us = k_cyc_to_us_floor64(delta_cycles);
		dt = (double)delta_time_us / 1000000.0;
		start_cycles = k_cycle_get_32();
		int rc = process_mpu6050(mpu6050);

		if (rc != 0)
		{
			break;
		}
#ifdef RESTRICT_PITCH // Eq. 25 and 26
		roll = atan2(accY, accZ) * RAD_TO_DEG;
		pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
		roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
		pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
		gyroXrate = gyroX / 131.0; // Convert to deg/s
		gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
		{
			kalmanX.setAngle(roll);
			kalAngleX = roll;
		}
		else
			kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

		if (abs(kalAngleX) > 90)
			gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
		kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90))
		{
			kalmanY.setAngle(pitch);
			kalAngleY = pitch;
		}
		else
			kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

		if (abs(kalAngleY) > 90)
			gyroXrate = -gyroXrate;						   // Invert rate, so it fits the restriced accelerometer reading
		kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

		// printf(" Angle X: %f\tAngle Y: %f\n", kalAngleX, kalAngleY);
		// printf("%f\n", kalAngleY);


		struct sensor_value val;
		rc = sensor_sample_fetch(qdec);
		if (rc != 0) {
			printf("Failed to fetch sample (%d)\n", rc);
			return 0;
		}

		rc = sensor_channel_get(qdec, SENSOR_CHAN_ROTATION, &val);
		if (rc != 0) {
			printf("Failed to get data (%d)\n", rc);
			return 0;
		// }

		printf("Position = %d degrees\n", val.val1);

		k_sleep(K_MSEC(100));
	}

	/* triggered runs with its own thread after exit */
	return 0;
}
