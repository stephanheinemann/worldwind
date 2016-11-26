/**
 * Copyright (c) 2016, Stephan Heinemann (UVic Center for Aerospace Research)
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
package com.cfar.swim.worldwind.aircraft;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.symbology.SymbologyConstants;

public class Iris extends Quadcopter {

	// TODO: use environment AirDataIntervals
	public static final double MAX_ANGLE_OF_CLIMB_SPEED = 10d;
	public static final double MAX_RATE_OF_CLIMB_SPEED = 10d;
	public static final double CRUISE_CLIMB_SPEED = 15d;
	public static final double CRUISE_SPEED = 15d;
	public static final double CRUISE_DESCENT_SPEED = 15d;
	public static final double APPROACH_SPEED = 5d;
	public static final double MAX_GLIDE_SPEED = 0d;
	public static final double MAX_RATE_OF_DESCENT_SPEED = 10d;
	public static final double MAX_SPEED = 20d;
	
	public static final double MAX_RATE_OF_CLIMB = 10d;
	public static final double CRUISE_RATE_OF_CLIMB = 2d;
	public static final double CRUISE_RATE_OF_DESCENT = 2d;
	public static final double APPROACH_RATE_OF_DESCENT = 5d;
	public static final double MAX_RATE_OF_DESCENT = 10d;
	public static final Angle MAX_ANGLE_OF_CLIMB = Angle.POS90;

	// TODO: horizontal versus, vertical separation
	// obstacle ellipsoid instead of obstacle sphere
	// compute horizontal separation from separation times and cruise speed
	public static final double SEPARATION_RADIUS = 500d;
	
	public Iris(Position position, double radius, CombatIdentification cid) {
		super(position, radius, cid);
		
		this.capabilities = new Capabilities();
		this.capabilities.setMaximumAngleOfClimbSpeed(Iris.MAX_ANGLE_OF_CLIMB_SPEED);
		this.capabilities.setMaximumRateOfClimb(Iris.MAX_RATE_OF_CLIMB);
		this.capabilities.setCruiseClimbSpeed(Iris.CRUISE_CLIMB_SPEED);
		this.capabilities.setCruiseSpeed(Iris.CRUISE_SPEED);
		this.capabilities.setCruiseDescentSpeed(Iris.CRUISE_DESCENT_SPEED);
		this.capabilities.setApproachSpeed(Iris.APPROACH_SPEED);
		this.capabilities.setMaximumGlideSpeed(Iris.MAX_GLIDE_SPEED);
		this.capabilities.setMaximumRateOfDescentSpeed(Iris.MAX_RATE_OF_DESCENT_SPEED);
		this.capabilities.setMaximumSpeed(Iris.MAX_SPEED);
		this.capabilities.setMaximumRateOfClimb(Iris.MAX_RATE_OF_CLIMB);
		this.capabilities.setCruiseRateOfClimb(Iris.CRUISE_RATE_OF_CLIMB);
		this.capabilities.setCruiseRateOfDescent(Iris.CRUISE_RATE_OF_DESCENT);
		this.capabilities.setApproachRateOfDescent(Iris.APPROACH_RATE_OF_DESCENT);
		this.capabilities.setMaximumRateOfDescent(Iris.MAX_RATE_OF_DESCENT);
		this.capabilities.setMaximumAngleOfClimb(Iris.MAX_ANGLE_OF_CLIMB);
	
		// TODO: use actual live data for symbol annotations...
		this.getDepiction().setModifier(SymbologyConstants.SPEED, Iris.CRUISE_SPEED);
	}

}
