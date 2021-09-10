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
package com.cfar.swim.worldwind.aircraft;

import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.util.Identifiable;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.symbology.SymbologyConstants;

/**
 * Realizes a 3DR Iris quadcopter.
 * 
 * @author Stephan Heinemann
 *
 */
public class Iris extends Quadcopter {

	// TODO: use environment AirDataIntervals
	
	/** the maximum angle of climb speed of this Iris in m/s */
	public static final double MAX_ANGLE_OF_CLIMB_SPEED = 10d;
	
	/** the maximum rate of climb speed of this Iris in m/s */
	public static final double MAX_RATE_OF_CLIMB_SPEED = 10d;
	
	/** the cruise climb speed of this Iris in m/s */
	public static final double CRUISE_CLIMB_SPEED = 15d;
	
	/** the cruise speed of this Iris in m/s */
	public static final double CRUISE_SPEED = 15d;
	
	/** the cruise descent speed of this Iris in m/s */
	public static final double CRUISE_DESCENT_SPEED = 15d;
	
	/** the approach speed of this Iris in m/s */
	public static final double APPROACH_SPEED = 5d;
	
	/** the maximum glide speed of this Iris in m/s */
	public static final double MAX_GLIDE_SPEED = 0d;
	
	/** the maximum rate of descent speed of this Iris in m/s */
	public static final double MAX_RATE_OF_DESCENT_SPEED = 10d;
	
	/** the maximum speed of this Iris in m/s */
	public static final double MAX_SPEED = 20d;
	
	/** the maximum rate of climb of this Iris in m/s */
	public static final double MAX_RATE_OF_CLIMB = 10d;
	
	/** the cruise rate of climb of this Iris in m/s */
	public static final double CRUISE_RATE_OF_CLIMB = 2d;
	
	/** the cruise rate of descent of this Iris in m/s */
	public static final double CRUISE_RATE_OF_DESCENT = 2d;
	
	/** the approach rate of descent of this Iris in m/s */
	public static final double APPROACH_RATE_OF_DESCENT = 5d;
	
	/** the maximum rate of descent of this Iris in m/s */
	public static final double MAX_RATE_OF_DESCENT = 10d;
	
	/** the maximum angle of climb of this Iris in degrees */
	public static final Angle MAX_ANGLE_OF_CLIMB = Angle.POS90;

	// TODO: horizontal versus, vertical separation
	// obstacle ellipsoid instead of obstacle sphere
	// compute horizontal separation from separation times and cruise speed
	
	/** the separation radius of this Iris */
	public static final double SEPARATION_RADIUS = 50d;
	
	/**
	 * Constructs a new Iris at a specified position with a specified
	 * separation radius and combat identification.
	 * 
	 * @param position the position
	 * @param radius the separation radius
	 * @param cid the combat identification
	 */
	public Iris(Position position, double radius, CombatIdentification cid) {
		super(position, radius, cid);
		
		this.capabilities = new Capabilities();
		
		this.capabilities.setApproachRateOfDescent(Iris.APPROACH_RATE_OF_DESCENT);
		this.capabilities.setApproachSpeed(Iris.APPROACH_SPEED);
		this.capabilities.setCruiseClimbSpeed(Iris.CRUISE_CLIMB_SPEED);
		this.capabilities.setCruiseDescentSpeed(Iris.CRUISE_DESCENT_SPEED);
		this.capabilities.setCruiseRateOfClimb(Iris.CRUISE_RATE_OF_CLIMB);
		this.capabilities.setCruiseRateOfDescent(Iris.CRUISE_RATE_OF_DESCENT);
		this.capabilities.setCruiseSpeed(Iris.CRUISE_SPEED);
		this.capabilities.setMaximumAngleOfClimb(Iris.MAX_ANGLE_OF_CLIMB);
		this.capabilities.setMaximumAngleOfClimbSpeed(Iris.MAX_ANGLE_OF_CLIMB_SPEED);
		this.capabilities.setMaximumGlideSpeed(Iris.MAX_GLIDE_SPEED);
		this.capabilities.setMaximumRateOfClimb(Iris.MAX_RATE_OF_CLIMB);
		this.capabilities.setMaximumRateOfClimbSpeed(Iris.MAX_RATE_OF_CLIMB_SPEED);
		this.capabilities.setMaximumRateOfDescent(Iris.MAX_RATE_OF_DESCENT);
		this.capabilities.setMaximumRateOfDescentSpeed(Iris.MAX_RATE_OF_DESCENT_SPEED);
		this.capabilities.setMaximumSpeed(Iris.MAX_SPEED);
	
		// TODO: use actual live data for symbol annotations...
		this.getDepiction().setModifier(SymbologyConstants.SPEED, Iris.CRUISE_SPEED);
	}
	
	/**
	 * Gets the identifier of this Iris.
	 * 
	 * @return the identifier of this Iris
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return Specification.AIRCRAFT_IRIS_ID;
	}
	
}
