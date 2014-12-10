/**
	This file is part of martink project.

	martink firmware project is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	martink firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with martink firmware.  If not, see <http://www.gnu.org/licenses/>.

	Author: Martin K. Schröder
	Email: info@fortmax.se
	Github: https://github.com/mkschreder
*/

#pragma once

#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtc/quaternion.hpp>

/**
 * Implements a sensor interface that tells the quadcopter it's orientation
 */
 
class SensorProvider {
public:
	// current position and orientation 
	virtual glm::quat getOrientation() = 0;
	virtual glm::mat3 getMatrix() = 0; 
	virtual glm::vec3	getPosition() = 0;
	virtual glm::vec3 getAccelerometer() = 0; 
	
	// rate of change methods
	virtual glm::vec3 getAngularVelocity() = 0;
	virtual glm::vec3 getLinearVelocity() = 0;
	
	virtual void update(
		int16_t ax, int16_t ay, int16_t az, 
		int16_t gx, int16_t gy, int16_t gz, 
		int16_t mx, int16_t my, int16_t mz, 
		int16_t A, int16_t P, int16_t T, float dt) = 0;
};
