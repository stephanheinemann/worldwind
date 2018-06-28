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
import java.beans.SimpleBeanInfo;

/**
 * @author Manuel Rosa
 *
 */
public class AbstractPlannerPropertiesBeanInfo extends SimpleBeanInfo {

	private final static Class<RRTreeProperties> beanClass = RRTreeProperties.class;

	public PropertyDescriptor[] getPropertyDescriptors() {
		try {
			PropertyDescriptor costPolicy = new PropertyDescriptor("costPolicy", beanClass);
			costPolicy.setShortDescription("the cost policy of this planner");
			costPolicy.setDisplayName("Cost Policy");
			costPolicy.setExpert(false);

			PropertyDescriptor riskPolicy = new PropertyDescriptor("riskPolicy", beanClass);
			riskPolicy.setShortDescription("the risk policy of this planner");
			riskPolicy.setDisplayName("Risk Policy");
			riskPolicy.setExpert(false);

			PropertyDescriptor rv[] = { costPolicy, riskPolicy };

			return rv;
		} catch (IntrospectionException e) {
			throw new Error(e.toString());
		}
	}

	public BeanDescriptor getBeanDescriptor() {
		return new BeanDescriptor(beanClass, null);
	}
}
