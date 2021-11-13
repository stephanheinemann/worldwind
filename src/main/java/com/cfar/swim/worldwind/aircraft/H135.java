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
import com.cfar.swim.worldwind.render.airspaces.ObstacleSphere;
import com.cfar.swim.worldwind.util.Identifiable;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.symbology.SymbologyConstants;

/**
 * Realizes a H135 helicopter aircraft.
 * 
 * @author Stephan Heinemann
 *
 */
public class H135 extends RotaryWingCivilAircraft {
	
	/** the maximum angle of climb speed of this H135 in m/s */
	public static final double MAX_ANGLE_OF_CLIMB_SPEED = 8d; // m/s = 1500 ft/min
	
	/** the maximum rate of climb speed of this H135 in m/s */
	public static final double MAX_RATE_OF_CLIMB_SPEED = 8d; // m/s = 1500 ft/min
	
	/** the cruise climb speed of this H135 in m/s */
	public static final double CRUISE_CLIMB_SPEED = 50d; // m/s = 97 KTAS
	
	/** the cruise speed of this H135 in m/s */
	public static final double CRUISE_SPEED = 70d; // m/s = 137 KTAS
	
	/** the cruise descent speed of this H135 in m/s */
	public static final double CRUISE_DESCENT_SPEED = 70d; // m/s = 137 KTAS
	
	/** the approach speed of this H135 in m/s */
	public static final double APPROACH_SPEED = 50d; // m/s = 97 KTAS
	
	/** the maximum glide speed of this H135 in m/s */
	public static final double MAX_GLIDE_SPEED = 11d; // m/s = 2200 ft/min (auto-rotation)
	
	/** the maximum rate of descent speed of this H135 in m/s */
	public static final double MAX_RATE_OF_DESCENT_SPEED = 11d; // m/s = 2200 ft/min (auto-rotation)
	
	/** the maximum speed of this H135 in m/s */
	public static final double MAX_SPEED = 72d; // m/s = 140 KTAS
	
	/** the maximum rate of climb of this H135 in m/s */
	public static final double MAX_RATE_OF_CLIMB = 8d; // m/s = 1500 ft/min
	
	/** the cruise rate of climb of this H135 in m/s */
	public static final double CRUISE_RATE_OF_CLIMB = 5d; // m/s = 1000 ft/min
	
	/** the cruise rate of descent of this H135 in m/s */
	public static final double CRUISE_RATE_OF_DESCENT = 2.5d; // m/s = 500 ft/min
	
	/** the approach rate of descent of this H135 in m/s */
	public static final double APPROACH_RATE_OF_DESCENT = 2.5d; // m/s = 500 ft/min
	
	/** the maximum rate of descent of this H135 in m/s */
	public static final double MAX_RATE_OF_DESCENT = 11d; // m/s = 2200 ft/min (auto-rotation)
	
	/** the maximum angle of climb of this H135 in degrees */
	public static final Angle MAX_ANGLE_OF_CLIMB = Angle.fromDegrees(90d);
	
	// TODO: horizontal versus, vertical separation
	// obstacle ellipsoid instead of obstacle sphere
	// compute horizontal separation from separation times and cruise speed
	
	/** the separation radius of this H135 */
	public static final double SEPARATION_RADIUS = 100d;
	
	/**
	 * Constructs a new H135 at a specified position with a specified
	 * separation radius and combat identification.
	 * 
	 * @param position the position
	 * @param radius the separation radius
	 * @param cid the combat identification
	 */
	public H135(Position position, double radius, CombatIdentification cid) {
		super(position, radius, cid);
		this.capabilities = new Capabilities();
		
		this.capabilities.setApproachRateOfDescent(H135.APPROACH_RATE_OF_DESCENT);
		this.capabilities.setApproachSpeed(H135.APPROACH_SPEED);
		this.capabilities.setCruiseClimbSpeed(H135.CRUISE_CLIMB_SPEED);
		this.capabilities.setCruiseDescentSpeed(H135.CRUISE_DESCENT_SPEED);
		this.capabilities.setCruiseRateOfClimb(H135.CRUISE_RATE_OF_CLIMB);
		this.capabilities.setCruiseRateOfDescent(H135.CRUISE_RATE_OF_DESCENT);
		this.capabilities.setCruiseSpeed(H135.CRUISE_SPEED);
		this.capabilities.setMaximumAngleOfClimb(H135.MAX_ANGLE_OF_CLIMB);
		this.capabilities.setMaximumAngleOfClimbSpeed(H135.MAX_ANGLE_OF_CLIMB_SPEED);
		this.capabilities.setMaximumGlideSpeed(H135.MAX_GLIDE_SPEED);
		this.capabilities.setMaximumRateOfClimb(H135.MAX_RATE_OF_CLIMB);
		this.capabilities.setMaximumRateOfClimbSpeed(H135.MAX_RATE_OF_CLIMB_SPEED);
		this.capabilities.setMaximumRateOfDescent(H135.MAX_RATE_OF_DESCENT);
		this.capabilities.setMaximumRateOfDescentSpeed(H135.MAX_RATE_OF_DESCENT_SPEED);
		this.capabilities.setMaximumSpeed(H135.MAX_SPEED);
		
		// TODO: use actual live data for symbol annotations...
		this.getDepiction().setModifier(SymbologyConstants.SPEED, H135.CRUISE_SPEED);
	}
	
	/**
	 * Gets the identifier of this H135.
	 * 
	 * @return the identifier of this H135
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return Specification.AIRCRAFT_H135_ID;
	}
	
	/**
	 * Interpolates the midpoint H135 between this and another H135. The
	 * interpolation considers the spatial and temporal attributes and modifies
	 * this H135 accordingly.
	 * 
	 * @param other the other H135
	 * 
	 * @return the interpolated midpoint H135 between this and the other H135
	 * 
	 * @see ObstacleSphere#interpolate(ObstacleSphere)
	 */
	@Override
	public H135 interpolate(ObstacleSphere other) {
		ObstacleSphere interpolant = super.interpolate(other);
		H135 h135 = new H135(interpolant.getCenter(),
				interpolant.getRadius(), this.getCombatIdentification());
		h135.setCostInterval(interpolant.getCostInterval());
		return h135;
	}
	
}
