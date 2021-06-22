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
* Realizes an ARRT properties bean information customizing the descriptors for
* each property.
* 
* @author Manuel Rosa
* @author Stephan Heinemann
*
*/
public class ARRTreePropertiesBeanInfo extends RRTreePropertiesBeanInfo {
	
	/**
	 * Constructs an ARRT properties bean information.
	 */
	public ARRTreePropertiesBeanInfo() {
		super(ARRTreeProperties.class);
	}
	
	/**
	 * Customizes the property descriptors for each property of an ARRT
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
			PropertyDescriptor neighborLimit = this.createPropertyDescriptor(
					"neighborLimit",
					this.dictionary.getString("property.planner.arrt.neighborLimit.name"),
					this.dictionary.getString("property.planner.arrt.neighborLimit.description"),
					this.dictionary.getString("property.planner.category.heuristic"));
			PropertyDescriptor initialCostBias = this.createPropertyDescriptor(
					"minimumQuality",
					this.dictionary.getString("property.planner.arrt.minimumQuality.name"),
					this.dictionary.getString("property.planner.arrt.minimumQuality.description"),
					this.dictionary.getString("property.planner.category.anytime"));
			PropertyDescriptor finalCostBias = this.createPropertyDescriptor(
					"maximumQuality",
					this.dictionary.getString("property.planner.arrt.maximumQuality.name"),
					this.dictionary.getString("property.planner.arrt.maximumQuality.description"),
					this.dictionary.getString("property.planner.category.anytime"));
			PropertyDescriptor improvementFactor = this.createPropertyDescriptor(
					"qualityImprovement",
					this.dictionary.getString("property.planner.arrt.qualityImprovement.name"),
					this.dictionary.getString("property.planner.arrt.qualityImprovement.description"),
					this.dictionary.getString("property.planner.category.anytime"));
		
			PropertyDescriptor[] arrtDescriptors = new PropertyDescriptor[] {
					neighborLimit,
					initialCostBias,
					finalCostBias,
					improvementFactor};
			descriptors = Stream.concat(
					Arrays.stream(descriptors), Arrays.stream(arrtDescriptors))
					.toArray(PropertyDescriptor[]::new);
		
		} catch (IntrospectionException e) {
			e.printStackTrace();
		}
		
		return descriptors;
	}
	
}
