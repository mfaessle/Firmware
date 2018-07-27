/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <px4_module.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/sensor_gyro.h>

extern "C" __EXPORT int mf_fw_control_main(int argc, char *argv[]);

enum class ControlMode {
	MANUAL,
	ACRO
};

struct State {
    uint64_t timestamp;
    float roll;
    float pitch;
    float roll_rate;
    float pitch_rate;
    float throttle;
};

class MfFwControl : public ModuleBase<MfFwControl>
{
public:
	MfFwControl();

	virtual ~MfFwControl() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static MfFwControl *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(int parameter_update_sub, bool force = false);

	void publish_actuator_controls();
    void reset_on_mode_change();

    State _state{};
    State _setpoint{};
    State _prev_setpoint{};

    sensor_gyro_s _prev_gyro{};
    float _roll_integrator{0.0f};
    float _pitch_integrator{0.0f};
	actuator_controls_s _actuators {};

    int _gyro_sub{-1};
    int _manual_control_sp_sub{-1};
    int _rc_sub{-1};

	orb_advert_t _actuators_0_pub{nullptr};

    // Parameters
    struct {
        float r_p;
        float r_i;
        float r_ff;
        float r_integrator_max;
        float trim_roll;

        float p_p;
        float p_i;
        float p_ff;
        float p_integrator_max;
        float trim_pitch;

        float man_roll_scale;
        float man_pitch_scale;

        float acro_max_x_rate_rad;
        float acro_max_y_rate_rad;
    } _parameters{};

    struct {
        param_t r_p;
        param_t r_i;
        param_t r_ff;
        param_t r_integrator_max;
        param_t trim_roll;
        
        param_t p_p;
        param_t p_i;
        param_t p_ff;
        param_t p_integrator_max;
        param_t trim_pitch;

        param_t man_roll_scale;
        param_t man_pitch_scale;

        param_t acro_max_x_rate;
        param_t acro_max_y_rate;
    } _parameter_handles{};
};

