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
package com.cfar.swim.worldwind.registries.planners.rrt;

import java.beans.IntrospectionException;
import java.beans.PropertyDescriptor;
import java.util.Arrays;
import java.util.stream.Stream;

/**
 * Realizes an OADRRT properties bean information customizing the descriptors
 * for each property.
 * 
 * @author Stephan Heinemann
 *
 */
public class OADRRTreePropertiesBeanInfo extends ADRRTreePropertiesBeanInfo {
	
	/**
	 * Constructs an OADRRT properties bean information.
	 */
	public OADRRTreePropertiesBeanInfo() {
		super(OADRRTreeProperties.class);
	}
	
	/**
	 * Customizes the property descriptors for each property of an OADRRT
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
			PropertyDescriptor minDeliberation = this.createPropertyDescriptor(
					"minDeliberation",
					this.dictionary.getString("property.planner.oadrrt.minDeliberation.name"),
					this.dictionary.getString("property.planner.oadrrt.minDeliberation.description"),
					this.dictionary.getString("property.planner.category.online"));
			PropertyDescriptor maxDeliberation = this.createPropertyDescriptor(
					"maxDeliberation",
					this.dictionary.getString("property.planner.oadrrt.maxDeliberation.name"),
					this.dictionary.getString("property.planner.oadrrt.maxDeliberation.description"),
					this.dictionary.getString("property.planner.category.online"));
			PropertyDescriptor maxCrossTrackError = this.createPropertyDescriptor(
					"maxCrossTrackError",
					this.dictionary.getString("property.planner.oadrrt.maxCrossTrackError.name"),
					this.dictionary.getString("property.planner.oadrrt.maxCrossTrackError.description"),
					this.dictionary.getString("property.planner.category.online"));
			PropertyDescriptor maxTimingError = this.createPropertyDescriptor(
					"maxTimingError",
					this.dictionary.getString("property.planner.oadrrt.maxTimingError.name"),
					this.dictionary.getString("property.planner.oadrrt.maxTimingError.description"),
					this.dictionary.getString("property.planner.category.online"));
			PropertyDescriptor maxTakeOffHorizontalError = this.createPropertyDescriptor(
					"maxTakeOffHorizontalError",
					this.dictionary.getString("property.planner.oadrrt.maxTakeOffHorizontalError.name"),
					this.dictionary.getString("property.planner.oadrrt.maxTakeOffHorizontalError.description"),
					this.dictionary.getString("property.planner.category.online"));
			PropertyDescriptor maxTakeOffTimingError = this.createPropertyDescriptor(
					"maxTakeOffTimingError",
					this.dictionary.getString("property.planner.oadrrt.maxTakeOffTimingError.name"),
					this.dictionary.getString("property.planner.oadrrt.maxTakeOffTimingError.description"),
					this.dictionary.getString("property.planner.category.online"));
			PropertyDescriptor maxLandingHorizontalError = this.createPropertyDescriptor(
					"maxLandingHorizontalError",
					this.dictionary.getString("property.planner.oadrrt.maxLandingHorizontalError.name"),
					this.dictionary.getString("property.planner.oadrrt.maxLandingHorizontalError.description"),
					this.dictionary.getString("property.planner.category.online"));
			PropertyDescriptor maxLandingTimingError = this.createPropertyDescriptor(
					"maxLandingTimingError",
					this.dictionary.getString("property.planner.oadrrt.maxLandingTimingError.name"),
					this.dictionary.getString("property.planner.oadrrt.maxLandingTimingError.description"),
					this.dictionary.getString("property.planner.category.online"));
			
			PropertyDescriptor[] oadrrtDescriptors = new PropertyDescriptor[] {
					minDeliberation,
					maxDeliberation,
					maxCrossTrackError,
					maxTimingError,
					maxTakeOffHorizontalError,
					maxTakeOffTimingError,
					maxLandingHorizontalError,
					maxLandingTimingError};
			descriptors = Stream.concat(
					Arrays.stream(descriptors), Arrays.stream(oadrrtDescriptors))
					.toArray(PropertyDescriptor[]::new);
		
		} catch (IntrospectionException e) {
			e.printStackTrace();
		}
		
		return descriptors;
	}
	
}
