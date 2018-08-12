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
package com.cfar.swim.worldwind.registries.planners;

import java.beans.BeanDescriptor;
import java.beans.IntrospectionException;
import java.beans.PropertyDescriptor;

/**
 * Realizes the properties bean of a basic RRTree planner with the property
 * descriptors for each parameter.
 * 
 * @author Manuel Rosa
 *
 */
public class RRTreePropertiesBeanInfo extends AbstractPlannerPropertiesBeanInfo {

	/** the class which this bean info refers to */
	private final static Class<RRTreeProperties> beanClass = RRTreeProperties.class;
	
	/** the category of parameters that are sampling related */
	protected final static String CATEGORY_SAMPLING = "Sampling Parameters";

	/**
	 * Customizes the property descriptors for each parameter of a RRTree planner.
	 * 
	 * @return the array of property descriptors
	 * 
	 * @see com.cfar.swim.worldwind.registries.planners.AbstractPlannerPropertiesBeanInfo#getPropertyDescriptors()
	 */
	@Override
	public PropertyDescriptor[] getPropertyDescriptors() {
		
		try {
			PropertyDescriptor maxIter = this.createProperty(beanClass, "maxIter",
					"Max Iterations",
					"the maximum number of sampling iterations",
					CATEGORY_SAMPLING);
			PropertyDescriptor strategy = this.createProperty(beanClass, "strategy",
					"Strategy",
					"the expansion strategy for the planner",
					CATEGORY_SAMPLING);
			PropertyDescriptor extension = this.createProperty(beanClass, "extension",
					"Extension",
					"the extension technique for the planner",
					CATEGORY_SAMPLING);
			PropertyDescriptor epsilon = this.createProperty(beanClass, "epsilon",
					"Epsilon (m)",
					"the maximum distance to extend a waypoint in the tree",
					CATEGORY_SAMPLING);
			PropertyDescriptor bias = this.createProperty(beanClass, "bias",
					"Goal Bias (%)",
					"the bias of the sampling algorithm towards goal",
					CATEGORY_SAMPLING);
			PropertyDescriptor goalThreshold = this.createProperty(beanClass, "goalThreshold",
					"Goal Threshold (m)",
					"the distance defining the goal region",
					CATEGORY_SAMPLING);
			
			PropertyDescriptor rvNew[] = {maxIter, strategy, extension, epsilon, bias, goalThreshold};
			PropertyDescriptor rvOld[] = super.getPropertyDescriptors();
			PropertyDescriptor rv[] = this.addPropertyDescriptors(rvOld, rvNew);

			return rv;
		} catch (IntrospectionException e) {
			throw new Error(e.toString());
		}
	}

	/**
	 * Gets the bean descriptor of this RRTree properties bean info.
	 * 
	 * @return the bean descriptor of this class
	 */
	public BeanDescriptor getBeanDescriptor() {
		return new BeanDescriptor(beanClass, null);
	}
}
