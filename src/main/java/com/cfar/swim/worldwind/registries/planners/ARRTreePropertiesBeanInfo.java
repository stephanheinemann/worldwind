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
 * Realizes the properties bean of an anytime RRTree planner with the property
 * descriptors for each parameter.
 * 
 * @author Manuel Rosa
 *
 */
public class ARRTreePropertiesBeanInfo extends RRTreePropertiesBeanInfo {

	/** the class which this bean info refers to */
	private final static Class<ARRTreeProperties> beanClass = ARRTreeProperties.class;

	/** the category of parameters that are anytime related */
	protected final static String CATEGORY_ANYTIME = "Anytime Parameters";
	
	/** the category of parameters that are online related */
	protected final static String CATEGORY_ONLINE = "Online Parameters";

	/**
	 * Customizes the property descriptors for each parameter of an anytime RRTree
	 * planner.
	 * 
	 * @return the array of property descriptors
	 * 
	 * @see com.cfar.swim.worldwind.registries.planners.AbstractPlannerPropertiesBeanInfo#getPropertyDescriptors()
	 */
	@Override
	public PropertyDescriptor[] getPropertyDescriptors() {

		try {
			PropertyDescriptor minimumQuality = this.createProperty(beanClass, "minimumQuality",
					"Initial Cost Bias",
					"the initial relative weight of costs (low values prioritize coverage of the configuration space)",
					CATEGORY_ANYTIME);
			PropertyDescriptor maximumQuality = this.createProperty(beanClass, "maximumQuality",
					"Final Cost Bias",
					"the final relative weight of costs (high values prioritize selection of waypoints with optimal costs)",
					CATEGORY_ANYTIME);
			PropertyDescriptor qualityImprovement = this.createProperty(beanClass, "qualityImprovement",
					"Improvement Factor",
					"the quality improvement value",
					CATEGORY_ANYTIME);
			PropertyDescriptor online = this.createProperty(beanClass, "online",
					"Online Planning",
					"the starting position of the plan is updated as the aircraft moves until it reaches the goal",
					CATEGORY_ONLINE);
			PropertyDescriptor updateStep = this.createProperty(beanClass, "updateStep",
					"Time Step (s)",
					"the time step to update the current position of the aircraft",
					CATEGORY_ONLINE);
			PropertyDescriptor positionThreshold = this.createProperty(beanClass, "positionThreshold",
					"Position Threshold (m)",
					"the distance threshold to consider a position displacement as worthy of a new plan",
					CATEGORY_ONLINE);

			PropertyDescriptor rvNew[] = {minimumQuality, maximumQuality, qualityImprovement, online, updateStep, positionThreshold};
			PropertyDescriptor rvOld[] = super.getPropertyDescriptors();
			PropertyDescriptor rv[] = this.addPropertyDescriptors(rvOld, rvNew);

			return rv;
		} catch (IntrospectionException e) {
			throw new Error(e.toString());
		}
	}

	/**
	 * Gets the bean descriptor of this anytime RRTree properties bean info.
	 * 
	 * @return the bean descriptor of this class
	 */
	public BeanDescriptor getBeanDescriptor() {
		return new BeanDescriptor(beanClass, null);
	}

}
