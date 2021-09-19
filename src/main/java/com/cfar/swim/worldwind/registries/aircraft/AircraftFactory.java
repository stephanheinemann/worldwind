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

import com.cfar.swim.worldwind.aircraft.A320;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Iris;
import com.cfar.swim.worldwind.registries.AbstractFactory;
import com.cfar.swim.worldwind.registries.Specification;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;


/**
 * Realizes an aircraft factory to create aircraft according to
 * customized aircraft specifications.
 * 
 * @author Stephan Heinemann
 * 
 * @see AbstractFactory
 * @see Specification
 */
public class AircraftFactory extends AbstractFactory<Aircraft> {

	/**
	 * Constructs a new aircraft factory without a customized specification.
	 * 
	 * @see AbstractFactory
	 */
	public AircraftFactory() {
		super();
	}
	
	/**
	 * Constructs a new aircraft factory to create registered aircraft
	 * according to a customized aircraft specification.
	 * 
	 * @param specification the aircraft specification describing the
	 *                      registered aircraft
	 * 
	 * @see AbstractFactory
	 */
	public AircraftFactory(Specification<Aircraft> specification) {
		super(specification);
	}

	/**
	 * Creates a new aircraft according to the customized aircraft
	 * specification of this aircraft factory.
	 * 
	 * @return the created aircraft, or null if no aircraft could be created
	 * 
	 * @see AbstractFactory#createInstance()
	 */
	@Override
	public Aircraft createInstance() {
		Aircraft aircraft = null;
		
		if (this.hasSpecification()) {
			AircraftProperties properties = (AircraftProperties) this.getSpecification().getProperties();
			
			if (this.getSpecification().getId().equals(Specification.AIRCRAFT_IRIS_ID)) {
				aircraft = new Iris(Position.ZERO, properties.getSeparationRadius(), properties.getCombatIdentification());
			} else if (this.getSpecification().getId().equals(Specification.AIRCRAFT_A320_ID)) {
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
		}
		
		return aircraft;
	}

}
