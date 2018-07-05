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
 * Realizes a FA PRM properties bean info with the property descriptors for
 * each parameter.
 * 
 * @author Henrique Ferreira
 *
 */
public class FAPRMPropertiesBeanInfo extends AbstractPlannerPropertiesBeanInfo {

	/** the class which this bean info refers to */
	private final static Class<FAPRMProperties> beanClass = FAPRMProperties.class;

	/** the category of parameters that are sampling related */
	protected final static String CATEGORY_SAMPLING = "Sampling Parameters";

	/** the category of parameters that are anytime related */
	protected final static String CATEGORY_ANYTIME = "Anytime Parameters";

	/** the category of parameters that are desirability related */
	protected final static String CATEGORY_DESIRABILITY = "Desirability Parameters";

	/**
	 * Customizes the property descriptors for each parameter of a FA PRM planner.
	 * 
	 * @return the array of property descriptors
	 * 
	 * @see com.cfar.swim.worldwind.registries.planners.AbstractPlannerPropertiesBeanInfo#getPropertyDescriptors()
	 */
	@Override
	public PropertyDescriptor[] getPropertyDescriptors() {

		try {
			PropertyDescriptor maxNeighbors = this.createProperty(beanClass, "maxNeighbors",
					"Max Neighbors",
					"the maximum number of neighbors of a single waypoint",
					CATEGORY_SAMPLING);
			PropertyDescriptor maxDistance = this.createProperty(beanClass, "maxDistance",
					"Max Distance (m)",
					"the maximum distance between two connectable waypoints",
					CATEGORY_SAMPLING);
			PropertyDescriptor bias = this.createProperty(beanClass, "bias",
					"Goal Bias (%)",
					"the bias of the sampling algorithm towards goal",
					CATEGORY_SAMPLING);
			PropertyDescriptor minimumQuality = this.createProperty(beanClass, "minimumQuality",
					"Initial Beta",
					"the minimum quality (low quality prioritizes coverage of the configuration space)",
					CATEGORY_ANYTIME);
			PropertyDescriptor maximumQuality = this.createProperty(beanClass, "maximumQuality",
					"Final Beta",
					"the maximum quality (high quality prioritizes selection of waypoints with higher f-values)",
					CATEGORY_ANYTIME);
			PropertyDescriptor qualityImprovement = this.createProperty(beanClass, "qualityImprovement",
					"Increase Step",
					"the quality improvement value",
					CATEGORY_ANYTIME);
			PropertyDescriptor lambda = this.createProperty(beanClass, "lambda",
					"Lambda",
					"the parameter that weights the desirability zones influence on the path cost",
					CATEGORY_DESIRABILITY);

			PropertyDescriptor rvNew[] = { maxNeighbors, maxDistance, minimumQuality, maximumQuality,
					qualityImprovement, bias, lambda };
			PropertyDescriptor rvOld[] = super.getPropertyDescriptors();
			PropertyDescriptor rv[] = this.addPropertyDescriptors(rvOld, rvNew);

			return rv;
		} catch (IntrospectionException e) {
			throw new Error(e.toString());
		}
	}

	/**
	 * Gets the bean descriptor of this FA PRM properties bean info.
	 * 
	 * @return the bean descriptor of this class
	 * 
	 * @see com.cfar.swim.worldwind.registries.planners.AbstractPlannerPropertiesBeanInfo#getBeanDescriptor()
	 */
	public BeanDescriptor getBeanDescriptor() {
		return new BeanDescriptor(beanClass, null);
	}
}
