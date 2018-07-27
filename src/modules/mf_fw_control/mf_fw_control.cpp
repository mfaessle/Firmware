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

#include "mf_fw_control.h"

#include <mathlib/math/Functions.hpp>
#include <mathlib/math/Limits.hpp>

#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/manual_control_setpoint.h>


int MfFwControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mf_fw_control", "module");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int MfFwControl::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int MfFwControl::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}

int MfFwControl::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("mf_fw_control",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

MfFwControl *MfFwControl::instantiate(int argc, char *argv[])
{
	MfFwControl *instance = new MfFwControl();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

MfFwControl::MfFwControl()
{
    _parameter_handles.r_p = param_find("FW_RR_P");
    _parameter_handles.r_i = param_find("FW_RR_I");
    _parameter_handles.r_ff = param_find("FW_RR_FF");
    _parameter_handles.r_integrator_max = param_find("FW_RR_IMAX");
    _parameter_handles.trim_roll = param_find("TRIM_ROLL");
    
    _parameter_handles.p_p = param_find("FW_PR_P");
    _parameter_handles.p_i = param_find("FW_PR_I");
    _parameter_handles.p_ff = param_find("FW_PR_FF");
    _parameter_handles.p_integrator_max = param_find("FW_PR_IMAX");
    _parameter_handles.trim_pitch = param_find("TRIM_PITCH");

    _parameter_handles.man_roll_scale = param_find("FW_MAN_R_SC");
    _parameter_handles.man_pitch_scale = param_find("FW_MAN_P_SC");

    _parameter_handles.acro_max_x_rate = param_find("FW_ACRO_X_MAX");
    _parameter_handles.acro_max_y_rate = param_find("FW_ACRO_Y_MAX");
}

void MfFwControl::run()
{
	// Subscribers
	_gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_rc_sub = orb_subscribe(ORB_ID(input_rc));

	px4_pollfd_struct_t fds[3];
	fds[0].fd = _gyro_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _manual_control_sp_sub;
	fds[1].events = POLLIN;
	fds[2].fd = _rc_sub;
	fds[2].events = POLLIN;

	// Publisher
	_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);

	ControlMode control_mode = ControlMode::MANUAL;

	// initialize parameters
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	parameters_update(parameter_update_sub, true);

    //PX4_INFO("r_p: %5.4f", double(_parameters.r_p));

	while (!should_exit()) {

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) 
		{
			// Timeout: let the loop run anyway, don't do `continue` here
			continue;
		} 
		else if (pret < 0) 
		{
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			usleep(50000);
			continue;
		} 
		
		if (fds[0].revents & POLLIN) 
		{
			struct sensor_gyro_s sensor_gyro;
			orb_copy(ORB_ID(sensor_gyro), _gyro_sub, &sensor_gyro);
			
            if (control_mode == ControlMode::MANUAL)
            {
                // Noting to do here
            }
            else
            {
                // Set state
                _state.timestamp = sensor_gyro.timestamp;
                _state.roll_rate = sensor_gyro.x;
                _state.pitch_rate = -sensor_gyro.y;

                // Integrate roll and pitch
                if (_prev_gyro.timestamp != 0)
                {
                    // Do integration only if we have a previous measurement
                    const float dt = (sensor_gyro.timestamp - _prev_gyro.timestamp) / 1000000.0f;
                    _state.roll += _prev_gyro.x * dt;
                    _state.pitch -= _prev_gyro.y * dt;
                }
                _prev_gyro = sensor_gyro;

                // Do control
                _actuators.control[actuator_controls_s::INDEX_ROLL] = 
                    _parameters.r_p * (_setpoint.roll_rate - _state.roll_rate) +
                    _parameters.r_i * (_setpoint.roll - _state.roll) +
                    _parameters.r_ff * _setpoint.roll_rate;
                _actuators.control[actuator_controls_s::INDEX_PITCH] = -(
                    _parameters.p_p * (_setpoint.pitch_rate - _state.pitch_rate) +
                    _parameters.p_i * (_setpoint.pitch - _state.pitch) +
                    _parameters.p_ff * _setpoint.pitch_rate);
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] =
                    _setpoint.throttle;

                publish_actuator_controls();
            }
		}

		if (fds[1].revents & POLLIN) 
		{
			struct manual_control_setpoint_s manual_control_setpoint;
			orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &manual_control_setpoint);

			if (control_mode == ControlMode::MANUAL)
			{
				_actuators.control[actuator_controls_s::INDEX_ROLL] = 
                    manual_control_setpoint.y * _parameters.man_roll_scale + 
                    _parameters.trim_roll;
				_actuators.control[actuator_controls_s::INDEX_PITCH] = 
                    -manual_control_setpoint.x * _parameters.man_pitch_scale + 
                    _parameters.trim_pitch;
				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = 
                    manual_control_setpoint.z;
				
				publish_actuator_controls();
			}
			else
			{
				// Set the setpoint for acro mode according to rc inputs
                _setpoint.timestamp = manual_control_setpoint.timestamp;
                _setpoint.roll_rate = manual_control_setpoint.y * _parameters.acro_max_x_rate_rad;
                _setpoint.pitch_rate = manual_control_setpoint.x * _parameters.acro_max_y_rate_rad;

                // Angle integration
                if (_prev_setpoint.timestamp != 0)
                {
                    const float dt = (manual_control_setpoint.timestamp - _prev_setpoint.timestamp) / 1000000.0f;
                    float new_roll_setpoint = _prev_setpoint.roll + _prev_setpoint.roll_rate * dt;
                    float new_pitch_setpoint = _prev_setpoint.pitch + _prev_setpoint.pitch_rate * dt;

                    if (float(fabs(new_roll_setpoint - _state.roll)) > _parameters.r_integrator_max)
                    {
                        if (fabs(_prev_setpoint.roll_rate) > 1e-3)
                        {
                            // If there is some RC rate input we update the setpoint to be 
                            // the allowed maximum away from the current state
                            if (new_roll_setpoint > _state.roll)
                            {
                                new_roll_setpoint = _state.roll + _parameters.r_integrator_max;
                            }
                            else
                            {
                                new_roll_setpoint = _state.roll - _parameters.r_integrator_max;
                            }
                        }
                        else
                        {
                            // No RC rate input so the angle difference probably was caused by 
                            // disturbance in the state -> leave the angle setpoint constant
                            new_roll_setpoint = _prev_setpoint.roll;
                        }
                    }
                    if (float(fabs(new_pitch_setpoint - _state.pitch)) > _parameters.p_integrator_max)
                    {
                        if (fabs(_prev_setpoint.pitch_rate) > 1e-3)
                        {
                            // If there is some RC rate input we update the setpoint to be 
                            // the allowed maximum away from the current state
                            if (new_pitch_setpoint > _state.pitch)
                            {
                                new_pitch_setpoint = _state.pitch + _parameters.p_integrator_max;
                            }
                            else
                            {
                                new_pitch_setpoint = _state.pitch - _parameters.p_integrator_max;
                            }
                        }
                        else
                        {
                            // No RC rate input so the angle difference probably was caused by 
                            // disturbance in the state -> leave the angle setpoint constant
                            new_pitch_setpoint = _prev_setpoint.pitch;
                        }
                    }

                    _setpoint.roll = new_roll_setpoint;
                    _setpoint.pitch = new_pitch_setpoint;
                }

                // Throttle
                _setpoint.throttle = manual_control_setpoint.z;

                _prev_setpoint = _setpoint;
			}
		}

		if (fds[2].revents & POLLIN) 
		{
			struct input_rc_s input_rc;
			orb_copy(ORB_ID(input_rc), _rc_sub, &input_rc);

			if (input_rc.values[5] > 1750)
			{
                if (control_mode == ControlMode::MANUAL)
                {
    				control_mode = ControlMode::ACRO;
    				PX4_INFO("Switched to ACRO mode");

    				// Reset state and setpoints
                    reset_on_mode_change();
                }
			}
			else if (control_mode == ControlMode::ACRO)
			{
				control_mode = ControlMode::MANUAL;
				PX4_INFO("Switched to MANUAL mode");

				// Reset state and setpoints
                reset_on_mode_change();
			}
		}

		parameters_update(parameter_update_sub);
	}

	orb_unsubscribe(_gyro_sub);
	orb_unsubscribe(parameter_update_sub);
}

void MfFwControl::publish_actuator_controls()
{
	// Saturation on Roll and Pitch that guarantees at least 50% control action
	// for each input but more if the other requires less
	float roll = _actuators.control[actuator_controls_s::INDEX_ROLL];
	float pitch = _actuators.control[actuator_controls_s::INDEX_PITCH];

	if (fabs(roll) + fabs(pitch) > 1.0)
	{
		if (fabs(roll) >= fabs(pitch))
		{
			pitch = math::sign(pitch) * math::min(fabs(pitch), 0.5);
			roll = math::sign(roll) * (1.0 - fabs(pitch));
		}
		else
		{
			roll = math::sign(roll) * math::min(fabs(roll), 0.5);
			pitch = math::sign(pitch) * (1.0 - fabs(roll));
		}
		_actuators.control[actuator_controls_s::INDEX_ROLL] = roll;
		_actuators.control[actuator_controls_s::INDEX_PITCH] = pitch;
	}

	orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);
}

void MfFwControl::reset_on_mode_change()
{
    _prev_gyro = {};
    _state = {};
    _setpoint = {};
    _prev_setpoint = {};
}

void MfFwControl::parameters_update(int parameter_update_sub, bool force)
{
    param_get(_parameter_handles.r_p, &(_parameters.r_p));
    param_get(_parameter_handles.r_i, &(_parameters.r_i));
    param_get(_parameter_handles.r_ff, &(_parameters.r_ff));
    param_get(_parameter_handles.r_integrator_max, &(_parameters.r_integrator_max));
    param_get(_parameter_handles.trim_roll, &(_parameters.trim_roll));
 
    param_get(_parameter_handles.p_p, &(_parameters.p_p));
    param_get(_parameter_handles.p_i, &(_parameters.p_i));
    param_get(_parameter_handles.p_ff, &(_parameters.p_ff));
    param_get(_parameter_handles.p_integrator_max, &(_parameters.p_integrator_max));
    param_get(_parameter_handles.trim_pitch, &(_parameters.trim_pitch));   

    param_get(_parameter_handles.man_roll_scale, &(_parameters.man_roll_scale));
    param_get(_parameter_handles.man_pitch_scale, &(_parameters.man_pitch_scale));

    param_get(_parameter_handles.acro_max_x_rate, &(_parameters.acro_max_x_rate_rad));
    param_get(_parameter_handles.acro_max_y_rate, &(_parameters.acro_max_y_rate_rad));
    _parameters.acro_max_x_rate_rad = math::radians(_parameters.acro_max_x_rate_rad);
    _parameters.acro_max_y_rate_rad = math::radians(_parameters.acro_max_y_rate_rad);
}

int mf_fw_control_main(int argc, char *argv[])
{
	return MfFwControl::main(argc, argv);
}
