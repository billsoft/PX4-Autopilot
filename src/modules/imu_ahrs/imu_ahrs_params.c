/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file imu_ahrs_params.c
 * IMU AHRS module parameters
 *
 * @author Your Name <your.email@example.com>
 */

/**
 * IMU AHRS Enable
 *
 * Enable/disable the IMU AHRS module
 *
 * @boolean
 * @reboot_required true
 * @group IMU AHRS
 *
 * @value 0 Disabled
 * @value 1 Enabled
 */
PARAM_DEFINE_INT32(SYS_IMU_AHRS, 0);

/**
 * IMU AHRS Update Rate
 *
 * Defines the update rate of the AHRS algorithm
 *
 * @unit Hz
 * @min 10
 * @max 200
 * @decimal 0
 * @increment 10
 * @reboot_required true
 * @group IMU AHRS
 *
 * @value 50 50 Hz
 * @value 100 100 Hz
 * @value 200 200 Hz
 */
PARAM_DEFINE_INT32(IMU_AHRS_FREQ_HZ, 100);

/**
 * Accelerometer Weight
 *
 * Weight of accelerometer data in complementary filter.
 * Higher values give faster response but more noise sensitivity.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group IMU AHRS
 */
PARAM_DEFINE_FLOAT(IMU_AHRS_W_ACC, 0.02f);

/**
 * Magnetometer Weight
 *
 * Weight of magnetometer data in complementary filter.
 * Reserved for future magnetometer fusion.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group IMU AHRS
 */
PARAM_DEFINE_FLOAT(IMU_AHRS_W_MAG, 0.0f);
