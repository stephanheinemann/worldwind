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

import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.aircraft.AircraftProperties;
import com.cfar.swim.worldwind.render.airspaces.ObstacleSphere;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.Material;
import gov.nasa.worldwind.symbology.milstd2525.MilStd2525Constants;

/**
 * Abstracts an aircraft.
 * 
 * @author Stephan Heinemann
 *
 */
public abstract class Aircraft extends ObstacleSphere implements FactoryProduct {
	
	// TODO: consider extending and obstacle ellipsoid instead of sphere
	// to reflect different vertical and horizontal speeds
	
	/** the combat identification of this aircraft */
	protected final CombatIdentification cid;
	
	/** the capabilities of this aircraft */
	protected Capabilities capabilities = null;
	
	/**
	 * Constructs a new aircraft at a specified position with a specified
	 * separation radius and combat identification.
	 * 
	 * @param position the position
	 * @param radius the separation radius
	 * @param cid the combat identification
	 */
	public Aircraft(Position position, double radius, CombatIdentification cid) {
		super(position, radius);
		this.cid = cid;
	}
	
	/**
	 * Gets the combat identification of this aircraft.
	 * 
	 * @return the combat identification of this aircraft
	 */
	public CombatIdentification getCombatIdentification() {
		return this.cid;
	}
	
	/**
	 * Gets the capabilities of this aircraft.
	 * 
	 * @return the capabilities of this aircraft
	 */
	public Capabilities getCapabilities() {
		return this.capabilities;
	}
	
	/**
	 * Sets the capabilities of this aircraft.
	 * 
	 * @param capabilities the capabilities to be set
	 */
	public void setCapabililities(Capabilities capabilities) {
		this.capabilities = capabilities;
	}
	
	/**
	 * Gets the symbol identification of this aircraft from a specified
	 * combat identification.
	 * 
	 * @param cid the combat identification
	 * 
	 * @return the symbol identification of this aircraft
	 */
	protected abstract String getSymbolIdentifier(CombatIdentification cid);
	
	/**
	 * Gets the material (appearance) of this aircraft from a specified
	 * combat identification.
	 * 
	 * @param cid the combat identification
	 * 
	 * @return the material of this aircraft
	 */
	protected Material getMaterial(CombatIdentification cid) {
		switch(cid) {
		case UNKNOWN:
			return MilStd2525Constants.MATERIAL_UNKNOWN;
		case FRIEND:
			return MilStd2525Constants.MATERIAL_FRIEND;
		case NEUTRAL:
			return MilStd2525Constants.MATERIAL_NEUTRAL;
		case HOSTILE:
			return MilStd2525Constants.MATERIAL_HOSTILE;
		default:
			return MilStd2525Constants.MATERIAL_UNKNOWN;
		}
	}
	
	/**
	 * Determines whether or not this aircraft matches a specification.
	 * 
	 * @param specification the specification to be matched
	 * 
	 * @return true if the this aircraft matches the specification,
	 *         false otherwise
	 * 
	 * @see FactoryProduct#matches(Specification)
	 */
	@Override
	public boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = false;
		
		if ((null != specification)
				&& specification.getId().equals(this.getId())
				&& (specification.getProperties() instanceof AircraftProperties)) {
			
			AircraftProperties properties = (AircraftProperties) specification.getProperties();
			if (this.getRadius() == properties.getSeparationRadius()
					&& (this.getCombatIdentification() == properties.getCombatIdentification())) {
				Capabilities capabilities = new Capabilities();
				capabilities.setApproachRateOfDescent(properties.getApproachRateOfDescent());
				capabilities.setApproachSpeed(properties.getApproachSpeed());
				capabilities.setCruiseClimbSpeed(properties.getCruiseClimbSpeed());
				capabilities.setCruiseDescentSpeed(properties.getCruiseDescentSpeed());
				capabilities.setCruiseRateOfClimb(properties.getCruiseRateOfClimb());
				capabilities.setCruiseRateOfDescent(properties.getCruiseRateOfDescent());
				capabilities.setCruiseSpeed(properties.getCruiseSpeed());
				capabilities.setMaximumAngleOfClimb(Angle.fromDegrees(properties.getMaximumAngleOfClimb()));
				capabilities.setMaximumAngleOfClimbSpeed(properties.getMaximumAngleOfClimbSpeed());
				capabilities.setMaximumGlideSpeed(properties.getMaximumGlideSpeed());
				capabilities.setMaximumRateOfClimb(properties.getMaximumRateOfClimb());
				capabilities.setMaximumRateOfClimbSpeed(properties.getMaximumRateOfClimbSpeed());
				capabilities.setMaximumRateOfDescent(properties.getMaximumRateOfDescent());
				capabilities.setMaximumRateOfDescentSpeed(properties.getMaximumRateOfDescentSpeed());
				capabilities.setMaximumSpeed(properties.getMaximumSpeed());
				matches = this.capabilities.equals(capabilities);
			}
		}
		
		return matches;
	}
	
	/**
	 * Updates this aircraft according to a specification.
	 * 
	 * @param specification the specification to be used for the update
	 * 
	 * @return true if this aircraft has been updated, false otherwise
	 * 
	 * @see FactoryProduct#update(Specification)
	 */
	@Override
	public boolean update(Specification<? extends FactoryProduct> specification) {
		boolean updated = false;
		
		if ((null != specification)
				&& specification.getId().equals(this.getId())
				&& (specification.getProperties() instanceof AircraftProperties)
				&& !this.matches(specification)) {
			
			AircraftProperties properties = (AircraftProperties) specification.getProperties();
			this.setRadius(properties.getSeparationRadius());
			Capabilities capabilities = this.getCapabilities();
			capabilities.setApproachRateOfDescent(properties.getApproachRateOfDescent());
			capabilities.setApproachSpeed(properties.getApproachSpeed());
			capabilities.setCruiseClimbSpeed(properties.getCruiseClimbSpeed());
			capabilities.setCruiseDescentSpeed(properties.getCruiseDescentSpeed());
			capabilities.setCruiseRateOfClimb(properties.getCruiseRateOfClimb());
			capabilities.setCruiseRateOfDescent(properties.getCruiseRateOfDescent());
			capabilities.setCruiseSpeed(properties.getCruiseSpeed());
			capabilities.setMaximumAngleOfClimb(Angle.fromDegrees(properties.getMaximumAngleOfClimb()));
			capabilities.setMaximumAngleOfClimbSpeed(properties.getMaximumAngleOfClimbSpeed());
			capabilities.setMaximumGlideSpeed(properties.getMaximumGlideSpeed());
			capabilities.setMaximumRateOfClimb(properties.getMaximumRateOfClimb());
			capabilities.setMaximumRateOfClimbSpeed(properties.getMaximumRateOfClimbSpeed());
			capabilities.setMaximumRateOfDescent(properties.getMaximumRateOfDescent());
			capabilities.setMaximumRateOfDescentSpeed(properties.getMaximumRateOfDescentSpeed());
			capabilities.setMaximumSpeed(properties.getMaximumSpeed());
			updated = true;
		}
		
		return updated;
	}
	
}
