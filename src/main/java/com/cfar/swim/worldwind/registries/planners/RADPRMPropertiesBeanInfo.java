/**
 * Copyright (c) 2018, Henrique Ferreira (UVic Center for Aerospace Research)
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
package com.cfar.swim.worldwind.registries.planners;

import java.beans.BeanDescriptor;
import java.beans.IntrospectionException;
import java.beans.PropertyDescriptor;

/**
 * Realizes a RAD PRM properties bean info with the property descriptors for
 * each parameter.
 * 
 * @author Henrique Ferreira
 *
 */
public class RADPRMPropertiesBeanInfo extends FADPRMPropertiesBeanInfo {

	/** the class which this bean info refers to */
	private final static Class<RADPRMProperties> beanClass = RADPRMProperties.class;

	/** the category of parameters that are sampling related */
	protected final static String CATEGORY_REPAIR = "Repairing Parameters";

	/**
	 * Customizes the property descriptors for each parameter of a RAD PRM planner.
	 * 
	 * @return the array of property descriptors
	 * 
	 * @see com.cfar.swim.worldwind.registries.planners.AbstractPlannerPropertiesBeanInfo#getPropertyDescriptors()
	 */
	@Override
	public PropertyDescriptor[] getPropertyDescriptors() {

		try {
			PropertyDescriptor repairFraction = this.createProperty(beanClass, "repairFraction",
					"Repair Fraction",
					"the fraction of waypoint that should be removed in the repair step",
					CATEGORY_REPAIR);

			PropertyDescriptor rvNew[] = {repairFraction};
			PropertyDescriptor rvOld[] = super.getPropertyDescriptors();
			PropertyDescriptor rv[] = this.addPropertyDescriptors(rvOld, rvNew);

			return rv;
		} catch (IntrospectionException e) {
			throw new Error(e.toString());
		}
	}

	/**
	 * Gets the bean descriptor of this RAD PRM properties bean info.
	 * 
	 * @return the bean descriptor of this class
	 * 
	 * @see com.cfar.swim.worldwind.registries.planners.AbstractPlannerPropertiesBeanInfo#getBeanDescriptor()
	 */
	public BeanDescriptor getBeanDescriptor() {
		return new BeanDescriptor(beanClass, null);
	}
}
