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
package com.cfar.swim.worldwind.registries.environments;

import java.beans.BeanDescriptor;
import java.beans.IntrospectionException;
import java.beans.PropertyDescriptor;
import java.beans.SimpleBeanInfo;
import java.util.Arrays;
import java.util.stream.Stream;

import org.controlsfx.property.BeanProperty;

/**
 * Realizes an Environment properties bean info with the property descriptors
 * for each parameter.
 * 
 * @author Manuel Rosa
 *
 */
public class EnvironmentPropertiesBeanInfo extends SimpleBeanInfo {

	/** the class which this bean info refers to */
	private final static Class<EnvironmentProperties> beanClass = EnvironmentProperties.class;

	/** the category of parameters that delimit the environment boundaries */
	protected final static String CATEGORY_BOUNDARIES = "Environment Boundaries";

	/**
	 * Customizes the property descriptors for each parameter of an Environment.
	 * 
	 * @return the array of property descriptors
	 * 
	 * @see java.beans.SimpleBeanInfo#getPropertyDescriptors()
	 */
	public PropertyDescriptor[] getPropertyDescriptors() {
		try {
			PropertyDescriptor floor = this.createProperty(beanClass, "floor",
					"Floor Altitude (m)",
					"the floor altitude for this environment",
					CATEGORY_BOUNDARIES);

			PropertyDescriptor ceiling = this.createProperty(beanClass, "ceiling",
					"Ceiling Altitude (m)",
					"the floor altitude for this environment",
					CATEGORY_BOUNDARIES);

			PropertyDescriptor rv[] = { floor, ceiling };

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
	 * Gets the bean descriptor of this Environment properties bean info.
	 * 
	 * @return the bean descriptor of this class
	 * 
	 * @see java.beans.SimpleBeanInfo#getBeanDescriptor()
	 */
	public BeanDescriptor getBeanDescriptor() {
		return new BeanDescriptor(beanClass, null);
	}
}
