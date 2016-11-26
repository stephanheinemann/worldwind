package com.cfar.swim.worldwind.registries.aircraft;

import com.cfar.swim.worldwind.aircraft.A320;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Iris;
import com.cfar.swim.worldwind.registries.Factory;
import com.cfar.swim.worldwind.registries.Specification;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;


/**
 * Realizes an aircraft factory to create aircraft according to
 * customized aircraft specifications.
 * 
 * @author Stephan Heinemann
 * 
 * @see Factory
 * @see Specification
 */
public class AircraftFactory implements Factory<Aircraft> {

	/**
	 * Creates a new aircraft according to a customized aircraft specification.
	 * 
	 * @param specification the customized aircraft specification
	 * 
	 * @return the created aircraft
	 * 
	 * @see Factory#createInstance(Specification)
	 */
	@Override
	public Aircraft createInstance(Specification<Aircraft> specification) {
		Aircraft aircraft = null;
		
		AircraftProperties properties = (AircraftProperties) specification.getProperties();
		
		if (specification.getId().equals(Specification.AIRCRAFT_IRIS_ID)) {
			aircraft = new Iris(Position.ZERO, properties.getSeparationRadius(), properties.getCombatIdentification());
		} else if (specification.getId().equals(Specification.AIRCRAFT_A320_ID)) {
			aircraft = new A320(Position.ZERO, properties.getSeparationRadius(), properties.getCombatIdentification());
		}
		
		aircraft.getCapabilities().setApproachRateOfDescent(properties.getApproachRateOfDescent());
		aircraft.getCapabilities().setApproachSpeed(properties.getApproachSpeed());
		aircraft.getCapabilities().setCruiseClimbSpeed(properties.getCruiseClimbSpeed());
		aircraft.getCapabilities().setCruiseDescentSpeed(properties.getCruiseDescentSpeed());
		aircraft.getCapabilities().setCruiseRateOfClimb(properties.getCruiseRateOfClimb());
		aircraft.getCapabilities().setCruiseRateOfDescent(properties.getCruiseRateOfDescent());
		aircraft.getCapabilities().setCruiseSpeed(properties.getCruiseSpeed());
		aircraft.getCapabilities().setMaximumAngleOfClimb(Angle.fromDegrees(properties.getMaximumAngleOfClimb()));
		aircraft.getCapabilities().setMaximumAngleOfClimbSpeed(properties.getMaximumAngleOfClimbSpeed());
		aircraft.getCapabilities().setMaximumGlideSpeed(properties.getMaximumGlideSpeed());
		aircraft.getCapabilities().setMaximumRateOfClimb(properties.getMaximumRateOfClimb());
		aircraft.getCapabilities().setMaximumRateOfClimbSpeed(properties.getMaximumRateOfClimbSpeed());
		aircraft.getCapabilities().setMaximumRateOfDescent(properties.getMaximumRateOfDescent());
		aircraft.getCapabilities().setMaximumRateOfDescentSpeed(properties.getMaximumRateOfDescent());
		aircraft.getCapabilities().setMaximumSpeed(properties.getMaximumSpeed());
		
		return aircraft;
	}

}
