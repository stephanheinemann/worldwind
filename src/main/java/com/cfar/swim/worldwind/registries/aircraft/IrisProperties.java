package com.cfar.swim.worldwind.registries.aircraft;

import com.cfar.swim.worldwind.aircraft.CombatIdentification;
import com.cfar.swim.worldwind.aircraft.Iris;

/**
 * Realizes the properties bean of an Iris aircraft.
 * 
 * @author Stephan Heinemann
 *
 */
public class IrisProperties extends AircraftProperties {

	/**
	 * Constructs a new Iris aircraft properties bean.
	 */
	public IrisProperties() {
		this.setApproachRateOfDescent(Iris.APPROACH_RATE_OF_DESCENT);
		this.setApproachSpeed(Iris.APPROACH_SPEED);
		this.setCombatIdentification(CombatIdentification.FRIEND);
		this.setCruiseClimbSpeed(Iris.CRUISE_CLIMB_SPEED);
		this.setCruiseDescentSpeed(Iris.CRUISE_DESCENT_SPEED);
		this.setCruiseRateOfClimb(Iris.CRUISE_RATE_OF_CLIMB);
		this.setCruiseRateOfDescent(Iris.CRUISE_RATE_OF_DESCENT);
		this.setCruiseSpeed(Iris.CRUISE_SPEED);
		this.setMaximumAngleOfClimb(Iris.MAX_ANGLE_OF_CLIMB.degrees);
		this.setMaximumAngleOfClimbSpeed(Iris.MAX_ANGLE_OF_CLIMB_SPEED);
		this.setMaximumGlideSpeed(Iris.MAX_GLIDE_SPEED);
		this.setMaximumRateOfClimb(Iris.MAX_RATE_OF_CLIMB);
		this.setMaximumRateOfClimbSpeed(Iris.MAX_RATE_OF_CLIMB_SPEED);
		this.setMaximumRateOfDescent(Iris.MAX_RATE_OF_DESCENT);
		this.setMaximumRateOfDescentSpeed(Iris.MAX_RATE_OF_DESCENT_SPEED);
		this.setMaximumSpeed(Iris.MAX_SPEED);
		this.setSeparationRadius(Iris.SEPARATION_RADIUS);
	}

}
