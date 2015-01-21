/*
	This file is part of my quadcopter project.
	https://github.com/mkschreder/bettercopter

	This software is firmware project is free software: you can 
	redistribute it and/or modify it under the terms of the GNU General 
	Public License as published by the Free Software Foundation, either 
	version 3 of the License, or (at your option) any later version.

	This software is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with martink firmware.  If not, see <http://www.gnu.org/licenses/>.

	Author: Arduino Team
*/

/// @file	PID.h
/// @brief	Generic PID algorithm
/// @author based on arduino pid 

#ifndef __PID_H__
#define __PID_H__

#include <inttypes.h>
#include <stdlib.h>
#include <math.h>               // for fabs()

// Examples for _filter:
// f_cut = 10 Hz -> _alpha = 0.385869
// f_cut = 15 Hz -> _alpha = 0.485194
// f_cut = 20 Hz -> _alpha = 0.556864
// f_cut = 25 Hz -> _alpha = 0.611015
// f_cut = 30 Hz -> _alpha = 0.653373
#define PID_D_TERM_FILTER 0.385869f    // Default 100Hz Filter Rate with 20Hz Cutoff Frequency

/// @class	PID
/// @brief	Object managing one PID control
class PID {
public:

    /// Constructor for PID that saves its settings to EEPROM
    ///
    /// @note	PIDs must be named to avoid either multiple parameters with the
    ///			same name, or an overly complex constructor.
    ///
    /// @param  initial_p       Initial value for the P term.
    /// @param  initial_i       Initial value for the I term.
    /// @param  initial_d       Initial value for the D term.
    /// @param  initial_imax    Initial value for the imax term.4
    ///
    PID(
        const float &   initial_p = 0.0,
        const float &   initial_i = 0.0,
        const float &   initial_d = 0.0,
        const int16_t & initial_imax = 0.0):
        _integrator(0),
        _last_input(0),
        _d_lpf_alpha(PID_D_TERM_FILTER)
    {
			_kp = initial_p;
			_ki = initial_i;
			_kd = initial_d;
			_imax = abs(initial_imax);

			_last_derivative = NAN;
    }

    /// Iterate the PID, return the new control value
    ///
    /// Positive error produces positive output.
    ///
    /// @param error	The measured error value
    /// @param dt		The time delta in milliseconds (note
    ///					that update interval cannot be more
    ///					than 65.535 seconds due to limited range
    ///					of the data type).
    /// @param scaler	An arbitrary scale factor
    ///
    /// @returns		The updated control output.
    ///
    float       get_pid(float error, float dt);
    float       get_pi(float error, float dt);
    float       get_p(float error) const;
    float       get_i(float error, float dt);
    float       get_d(float error, float dt);

    /// Reset the PID integrator
    ///
    void        reset_I();

    /// Load gain properties
    ///
    void        load_gains();

    /// Save gain properties
    ///
    void        save_gains();
    
    /// Sets filter Alpha for D-term LPF
    void        set_d_lpf_alpha(int16_t cutoff_frequency, float time_step);

    /// @name	parameter accessors
    //@{

    /// Overload the function call operator to permit relatively easy initialisation
    void operator        () (const float    p,
                             const float    i,
                             const float    d,
                             const int16_t  imaxval) {
        _kp = p; _ki = i; _kd = d; _imax = abs(imaxval);
    }

    // accessors
    float       kP() const { return _kp; }
    float       kI() const { return _ki; }
    float       kD() const { return _kd; }
    int16_t     imax() const { return _imax; }
    void        kP(const float v) { _kp = v; }
    void        kI(const float v) { _ki = v; }
    void        kD(const float v) { _kd = v; }
    void        imax(const int16_t v) { _imax = (abs(v)); }
    float       get_integrator() const { return _integrator; }
    void        set_integrator(float i) { _integrator = i; }

protected:
    float        _kp;
    float        _ki;
    float        _kd;
    int16_t        _imax;

    float           _integrator;                                ///< integrator value
    float           _last_input;                                ///< last input for derivative
    float           _last_derivative;                           ///< last derivative for low-pass filter
    float           _d_lpf_alpha;                               ///< alpha used in D-term LPF
};

#endif // __PID_H__
