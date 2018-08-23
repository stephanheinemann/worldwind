/**
 * Copyright (c) 2018, Manuel Rosa (UVic Center for Aerospace Research)
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

import java.beans.BeanDescriptor;
import java.beans.IntrospectionException;
import java.beans.PropertyDescriptor;
import java.beans.SimpleBeanInfo;
import java.util.Arrays;
import java.util.stream.Stream;

import org.controlsfx.property.BeanProperty;

/**
 * Realizes an Aircraft properties bean info with the property descriptors for
 * each parameter.
 * 
 * @author Manuel Rosa
 *
 */
public class AircraftPropertiesBeanInfo extends SimpleBeanInfo {

	/** the class which this bean info refers to */
	private final static Class<AircraftProperties> beanClass = AircraftProperties.class;

	/** the category of parameters that refer to cruise values */
	protected final static String CATEGORY_CRUISE = "Cruise Parameters";

	/** the category of parameters that refer to maximum values */
	protected final static String CATEGORY_MAXIMUM = "Maximum Parameters";

	/** the category of parameters that refer to approach values */
	protected final static String CATEGORY_APPROACH = "Approach Parameters";

	/** the category of parameters that refer to aircraft identification */
	protected final static String CATEGORY_IDENTIFICATION = "Aircraft Identification";

	/**
	 * Customizes the property descriptors for each parameter of an Aircraft.
	 * 
	 * @return the array of property descriptors
	 * 
	 * @see java.beans.SimpleBeanInfo#getPropertyDescriptors()
	 */
	public PropertyDescriptor[] getPropertyDescriptors() {
		try {
			PropertyDescriptor cid = this.createProperty(beanClass, "combatIdentification",
					"Combat Identification",
					"the combat identification of this aircraft",
					CATEGORY_IDENTIFICATION);

			PropertyDescriptor radius = this.createProperty(beanClass, "separationRadius",
					"Separation Radius (m)",
					"the radius of separation this aircraft should maintain from others",
					CATEGORY_IDENTIFICATION);

			PropertyDescriptor maximumSpeed = this.createProperty(beanClass, "maximumSpeed",
					"Max Speed (m/s)",
					"the maximum speed of this aircraft",
					CATEGORY_MAXIMUM);
			PropertyDescriptor maximumGlideSpeed = this.createProperty(beanClass, "maximumGlideSpeed",
					"Max Glide Speed (m/s)",
					"the maximum glide speed of this aircraft",
					CATEGORY_MAXIMUM);
			PropertyDescriptor maximumRateOfClimb = this.createProperty(beanClass, "maximumRateOfClimb",
					"Max Rate of Climb (m/s)",
					"the maximum rate of climb of this aircraft",
					CATEGORY_MAXIMUM);
			PropertyDescriptor maximumRateOfClimbSpeed = this.createProperty(beanClass, "maximumRateOfClimbSpeed",
					"Max Rate of Climb Speed (m/s)",
					"the maximum rate of climb speed of this aircraft",
					CATEGORY_MAXIMUM);
			PropertyDescriptor maximumRateOfDescent = this.createProperty(beanClass, "maximumRateOfDescent",
					"Max Rate of Descent (m/s)",
					"the maximum rate of descent of this aircraft",
					CATEGORY_MAXIMUM);
			PropertyDescriptor maximumRateOfDescentSpeed = this.createProperty(beanClass, "maximumRateOfDescentSpeed",
					"Max Rate of Descent Speed (m/s)",
					"the maximum rate of descent speed of this aircraft",
					CATEGORY_MAXIMUM);
			PropertyDescriptor maximumAngleOfClimb = this.createProperty(beanClass, "maximumAngleOfClimb",
					"Max Angle of Climb (deg)",
					"the maximum angle of climb of this aircraft",
					CATEGORY_MAXIMUM);
			PropertyDescriptor maximumAngleOfClimbSpeed = this.createProperty(beanClass, "maximumAngleOfClimbSpeed",
					"Max Angle of Climb Speed (m/s)",
					"the maximum angle of climb speed of this aircraft",
					CATEGORY_MAXIMUM);

			PropertyDescriptor cruiseSpeed = this.createProperty(beanClass, "cruiseSpeed",
					"Cruise Speed (m/s)",
					"the cruise speed of this aircraft",
					CATEGORY_CRUISE);
			PropertyDescriptor cruiseRateOfClimb = this.createProperty(beanClass, "cruiseRateOfClimb",
					"Cruise Rate of Climb (m/s)",
					"the cruise rate of climb of this aircraft",
					CATEGORY_CRUISE);
			PropertyDescriptor cruiseClimbSpeed = this.createProperty(beanClass, "cruiseClimbSpeed",
					"Cruise Climb Speed (m/s)",
					"the cruise rate of climb speed of this aircraft",
					CATEGORY_CRUISE);
			PropertyDescriptor cruiseRateOfDescent = this.createProperty(beanClass, "cruiseRateOfDescent",
					"Cruise Rate of Descent (m/s)",
					"the cruise rate of descent of this aircraft",
					CATEGORY_CRUISE);
			PropertyDescriptor cruiseDescentSpeed = this.createProperty(beanClass, "cruiseDescentSpeed",
					"Cruise Descent Speed (m/s)",
					"the cruise rate of descent speed of this aircraft",
					CATEGORY_CRUISE);

			PropertyDescriptor approachSpeed = this.createProperty(beanClass, "approachSpeed",
					"Approach Speed (m/s)",
					"the approach speed of this aircraft",
					CATEGORY_APPROACH);
			PropertyDescriptor approachRateOfDescent = this.createProperty(beanClass, "approachRateOfDescent",
					"Approach Rate of Descent (m/s)",
					"the approach rate of descent of this aircraft",
					CATEGORY_APPROACH);

			PropertyDescriptor rv[] = { cid, radius,
					maximumSpeed, maximumGlideSpeed, maximumRateOfClimb, maximumRateOfClimbSpeed, maximumRateOfDescent,
					maximumRateOfDescentSpeed, maximumAngleOfClimb, maximumAngleOfClimbSpeed,
					cruiseSpeed, cruiseRateOfClimb, cruiseClimbSpeed, cruiseRateOfDescent, cruiseDescentSpeed,
					approachSpeed, approachRateOfDescent };

			return rv;
		} catch (IntrospectionException e) {
			throw new Error(e.toString());
		}
	}

	/**
	 * Creates a property descriptor for a given parameter.
	 * 
	 * @param beanClass the class this parameters belongs to
	 * @param propertyName the parameter name
	 * @param displayName the name to be displayed in the user interface
	 * @param description the description of this parameter
	 * @param category the category of this parameter
	 * 
	 * @return the property descriptor created accordingly to the given inputs
	 * 
	 * @throws IntrospectionException
	 */
	public PropertyDescriptor createProperty(Class<?> beanClass, String propertyName, String displayName,
			String description, String category) throws IntrospectionException {

		PropertyDescriptor property = new PropertyDescriptor(propertyName, beanClass);
		property.setDisplayName(displayName);
		property.setShortDescription(description);
		property.setValue(BeanProperty.CATEGORY_LABEL_KEY, category);

		return property;
	}

	/**
	 * Adds two arrays of property descriptors into a new one containing all the
	 * property descriptors from the two initial arrays.
	 * 
	 * @param rvOld the old array of property descriptors
	 * @param rvNew the new array of property descriptors
	 * 
	 * @return the sum of the two arrays of property descriptors
	 */
	public PropertyDescriptor[] addPropertyDescriptors(PropertyDescriptor[] rvOld, PropertyDescriptor[] rvNew) {
		return Stream.concat(Arrays.stream(rvOld), Arrays.stream(rvNew))
				.toArray(PropertyDescriptor[]::new);
	}

	/**
	 * Gets the bean descriptor of this Aircraft properties bean info.
	 * 
	 * @return the bean descriptor of this class
	 * 
	 * @see java.beans.SimpleBeanInfo#getBeanDescriptor()
	 */
	public BeanDescriptor getBeanDescriptor() {
		return new BeanDescriptor(beanClass, null);
	}
}
