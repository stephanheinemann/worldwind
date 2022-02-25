/**
 * Copyright (c) 2021, Stephan Heinemann (UVic Center for Aerospace Research)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.cfar.swim.worldwind.registries.aircraft;

import com.cfar.swim.worldwind.aircraft.H135;
import com.cfar.swim.worldwind.aircraft.CombatIdentification;

/**
 * Realizes the properties bean of an H135 aircraft.
 * 
 * @author Stephan Heinemann
 *
 */
public class H135Properties extends AircraftProperties {
	
	/** the default serial identification of this H135 aircraft properties bean */
	private static final long serialVersionUID = 1L;
	
	/**
	 * Constructs a new H135 aircraft properties bean.
	 */
	public H135Properties() {
		this.setApproachRateOfDescent(H135.APPROACH_RATE_OF_DESCENT);
		this.setApproachSpeed(H135.APPROACH_SPEED);
		this.setCombatIdentification(CombatIdentification.FRIEND);
		this.setCruiseClimbSpeed(H135.CRUISE_CLIMB_SPEED);
		this.setCruiseDescentSpeed(H135.CRUISE_DESCENT_SPEED);
		this.setCruiseRateOfClimb(H135.CRUISE_RATE_OF_CLIMB);
		this.setCruiseRateOfDescent(H135.CRUISE_RATE_OF_DESCENT);
		this.setCruiseSpeed(H135.CRUISE_SPEED);
		this.setMaximumAngleOfClimb(H135.MAX_ANGLE_OF_CLIMB.degrees);
		this.setMaximumAngleOfClimbSpeed(H135.MAX_ANGLE_OF_CLIMB_SPEED);
		this.setMaximumGlideSpeed(H135.MAX_GLIDE_SPEED);
		this.setMaximumRateOfClimb(H135.MAX_RATE_OF_CLIMB);
		this.setMaximumRateOfClimbSpeed(H135.MAX_RATE_OF_CLIMB_SPEED);
		this.setMaximumRateOfDescent(H135.MAX_RATE_OF_DESCENT);
		this.setMaximumRateOfDescentSpeed(H135.MAX_RATE_OF_DESCENT_SPEED);
		this.setMaximumSpeed(H135.MAX_SPEED);
		this.setSeparationRadius(H135.SEPARATION_RADIUS);
	}
	
}
