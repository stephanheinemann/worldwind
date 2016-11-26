package com.cfar.swim.worldwind.registries.aircraft;

import com.cfar.swim.worldwind.aircraft.CombatIdentification;
import com.cfar.swim.worldwind.aircraft.A320;

/**
 * Realizes the properties bean of an A320 aircraft.
 * 
 * @author Stephan Heinemann
 *
 */
public class A320Properties extends AircraftProperties {

	/**
	 * Constructs a new A320 aircraft properties bean.
	 */
	public A320Properties() {
		this.setApproachRateOfDescent(A320.APPROACH_RATE_OF_DESCENT);
		this.setApproachSpeed(A320.APPROACH_SPEED);
		this.setCombatIdentification(CombatIdentification.FRIEND);
		this.setCruiseClimbSpeed(A320.CRUISE_CLIMB_SPEED);
		this.setCruiseDescentSpeed(A320.CRUISE_DESCENT_SPEED);
		this.setCruiseRateOfClimb(A320.CRUISE_RATE_OF_CLIMB);
		this.setCruiseRateOfDescent(A320.CRUISE_RATE_OF_DESCENT);
		this.setCruiseSpeed(A320.CRUISE_SPEED);
		this.setMaximumAngleOfClimb(A320.MAX_ANGLE_OF_CLIMB.degrees);
		this.setMaximumAngleOfClimbSpeed(A320.MAX_ANGLE_OF_CLIMB_SPEED);
		this.setMaximumGlideSpeed(A320.MAX_GLIDE_SPEED);
		this.setMaximumRateOfClimb(A320.MAX_RATE_OF_CLIMB);
		this.setMaximumRateOfClimbSpeed(A320.MAX_RATE_OF_CLIMB_SPEED);
		this.setMaximumRateOfDescent(A320.MAX_RATE_OF_DESCENT);
		this.setMaximumRateOfDescentSpeed(A320.MAX_RATE_OF_DESCENT_SPEED);
		this.setMaximumSpeed(A320.MAX_SPEED);
		this.setSeparationRadius(A320.SEPARATION_RADIUS);
	}

}
