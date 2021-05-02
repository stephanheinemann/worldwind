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

import java.beans.IntrospectionException;
import java.beans.PropertyDescriptor;

import com.cfar.swim.worldwind.registries.PropertiesBeanInfo;

/**
 * Realizes an aircraft properties bean information customizing the descriptors
 * for each property.
 * 
 * @author Stephan Heinemann
 *
 */
public class AircraftPropertiesBeanInfo extends PropertiesBeanInfo {
	
	/**
	 * Constructs an aircraft properties bean information.
	 */
	public AircraftPropertiesBeanInfo() {
		super(AircraftProperties.class);
	}
	
	/**
	 * Customizes the property descriptors for each property of an aircraft
	 * properties bean.
	 * 
	 * @return the array of customized property descriptors
	 * 
	 * @see java.beans.SimpleBeanInfo#getPropertyDescriptors()
	 */
	@Override
	public PropertyDescriptor[] getPropertyDescriptors() {
		PropertyDescriptor[] descriptors = super.getPropertyDescriptors();
		
		try {
			PropertyDescriptor combatIdentification = this.createPropertyDescriptor(
					"combatIdentification",
					this.dictionary.getString("property.aircraft.combatIdentification.name"),
					this.dictionary.getString("property.aircraft.combatIdentification.description"),
					this.dictionary.getString("property.aircraft.category.identification"));
			PropertyDescriptor separationRadius = this.createPropertyDescriptor(
					"separationRadius",
					this.dictionary.getString("property.aircraft.separationRadius.name"),
					this.dictionary.getString("property.aircraft.separationRadius.description"),
					this.dictionary.getString("property.aircraft.category.identification"));
			PropertyDescriptor maximumSpeed = this.createPropertyDescriptor(
					"maximumSpeed",
					this.dictionary.getString("property.aircraft.maximumSpeed.name"),
					this.dictionary.getString("property.aircraft.maximumSpeed.description"),
					this.dictionary.getString("property.aircraft.category.limits"));
			PropertyDescriptor maximumGlideSpeed = this.createPropertyDescriptor(
					"maximumGlideSpeed",
					this.dictionary.getString("property.aircraft.maximumGlideSpeed.name"),
					this.dictionary.getString("property.aircraft.maximumGlideSpeed.description"),
					this.dictionary.getString("property.aircraft.category.limits"));
			PropertyDescriptor maximumRateOfClimb = this.createPropertyDescriptor(
					"maximumRateOfClimb",
					this.dictionary.getString("property.aircraft.maximumRateOfClimb.name"),
					this.dictionary.getString("property.aircraft.maximumRateOfClimb.description"),
					this.dictionary.getString("property.aircraft.category.limits"));
			PropertyDescriptor maximumRateOfClimbSpeed = this.createPropertyDescriptor(
					"maximumRateOfClimbSpeed",
					this.dictionary.getString("property.aircraft.maximumRateOfClimbSpeed.name"),
					this.dictionary.getString("property.aircraft.maximumRateOfClimbSpeed.description"),
					this.dictionary.getString("property.aircraft.category.limits"));
			PropertyDescriptor maximumRateOfDescent = this.createPropertyDescriptor(
					"maximumRateOfDescent",
					this.dictionary.getString("property.aircraft.maximumRateOfDescent.name"),
					this.dictionary.getString("property.aircraft.maximumRateOfDescent.description"),
					this.dictionary.getString("property.aircraft.category.limits"));
			PropertyDescriptor maximumRateOfDescentSpeed = this.createPropertyDescriptor(
					"maximumRateOfDescentSpeed",
					this.dictionary.getString("property.aircraft.maximumRateOfDescentSpeed.name"),
					this.dictionary.getString("property.aircraft.maximumRateOfDescentSpeed.description"),
					this.dictionary.getString("property.aircraft.category.limits"));
			PropertyDescriptor maximumAngleOfClimb = this.createPropertyDescriptor(
					"maximumAngleOfClimb",
					this.dictionary.getString("property.aircraft.maximumAngleOfClimb.name"),
					this.dictionary.getString("property.aircraft.maximumAngleOfClimb.description"),
					this.dictionary.getString("property.aircraft.category.limits"));
			PropertyDescriptor maximumAngleOfClimbSpeed = this.createPropertyDescriptor(
					"maximumAngleOfClimbSpeed",
					this.dictionary.getString("property.aircraft.maximumAngleOfClimbSpeed.name"),
					this.dictionary.getString("property.aircraft.maximumAngleOfClimbSpeed.description"),
					this.dictionary.getString("property.aircraft.category.limits"));
			PropertyDescriptor cruiseSpeed = this.createPropertyDescriptor(
					"cruiseSpeed",
					this.dictionary.getString("property.aircraft.cruiseSpeed.name"),
					this.dictionary.getString("property.aircraft.cruiseSpeed.description"),
					this.dictionary.getString("property.aircraft.category.cruise"));
			PropertyDescriptor cruiseRateOfClimb = this.createPropertyDescriptor(
					"cruiseRateOfClimb",
					this.dictionary.getString("property.aircraft.cruiseRateOfClimb.name"),
					this.dictionary.getString("property.aircraft.cruiseRateOfClimb.description"),
					this.dictionary.getString("property.aircraft.category.cruise"));
			PropertyDescriptor cruiseClimbSpeed = this.createPropertyDescriptor(
					"cruiseClimbSpeed",
					this.dictionary.getString("property.aircraft.cruiseClimbSpeed.name"),
					this.dictionary.getString("property.aircraft.cruiseClimbSpeed.description"),
					this.dictionary.getString("property.aircraft.category.cruise"));
			PropertyDescriptor cruiseRateOfDescent = this.createPropertyDescriptor(
					"cruiseRateOfDescent",
					this.dictionary.getString("property.aircraft.cruiseRateOfDescent.name"),
					this.dictionary.getString("property.aircraft.cruiseRateOfDescent.description"),
					this.dictionary.getString("property.aircraft.category.cruise"));
			PropertyDescriptor cruiseDescentSpeed = this.createPropertyDescriptor(
					"cruiseDescentSpeed",
					this.dictionary.getString("property.aircraft.cruiseDescentSpeed.name"),
					this.dictionary.getString("property.aircraft.cruiseDescentSpeed.description"),
					this.dictionary.getString("property.aircraft.category.cruise"));
			PropertyDescriptor approachRateOfDescent = this.createPropertyDescriptor(
					"approachRateOfDescent",
					this.dictionary.getString("property.aircraft.approachRateOfDescent.name"),
					this.dictionary.getString("property.aircraft.approachRateOfDescent.description"),
					this.dictionary.getString("property.aircraft.category.approach"));
			PropertyDescriptor approachSpeed = this.createPropertyDescriptor(
					"approachSpeed",
					this.dictionary.getString("property.aircraft.approachSpeed.name"),
					this.dictionary.getString("property.aircraft.approachSpeed.description"),
					this.dictionary.getString("property.aircraft.category.approach"));

			descriptors = new PropertyDescriptor[] {
					combatIdentification, separationRadius,
					maximumSpeed, maximumGlideSpeed, maximumRateOfClimb,
					maximumRateOfClimbSpeed, maximumRateOfDescent,
					maximumRateOfDescentSpeed, maximumAngleOfClimb,
					maximumAngleOfClimbSpeed,
					cruiseSpeed, cruiseRateOfClimb, cruiseClimbSpeed,
					cruiseRateOfDescent, cruiseDescentSpeed,
					approachSpeed, approachRateOfDescent };
		
		} catch (IntrospectionException e) {
			e.printStackTrace();
		}
		
		return descriptors;
	}
	
}
