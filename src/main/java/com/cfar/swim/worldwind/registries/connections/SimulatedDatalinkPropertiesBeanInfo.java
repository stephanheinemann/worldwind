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
package com.cfar.swim.worldwind.registries.connections;

import java.beans.IntrospectionException;
import java.beans.PropertyDescriptor;
import java.util.Arrays;
import java.util.stream.Stream;

/**
 * Realizes a simulated datalink properties bean information customizing the
 * descriptors for each property.
 * 
 * @author Stephan Heinemann
 *
 */
public class SimulatedDatalinkPropertiesBeanInfo extends DatalinkPropertiesBeanInfo {
	
	/**
	 * Constructs a simulated datalink properties bean information.
	 */
	public SimulatedDatalinkPropertiesBeanInfo() {
		super(SimulatedDatalinkProperties.class);
	}
	
	/**
	 * Customizes the property descriptors for each property of a simulated
	 * datalink properties bean.
	 * 
	 * @return the array of customized property descriptors
	 * 
	 * @see java.beans.SimpleBeanInfo#getPropertyDescriptors()
	 */
	@Override
	public PropertyDescriptor[] getPropertyDescriptors() {
		PropertyDescriptor[] descriptors = super.getPropertyDescriptors();
		
		try {
			PropertyDescriptor uplinkDelay = this.createPropertyDescriptor(
					"uplinkDelay",
					this.dictionary.getString("property.connection.datalink.simulated.uplinkDelay.name"),
					this.dictionary.getString("property.connection.datalink.simulated.uplinkDelay.description"),
					this.dictionary.getString("property.connection.datalink.category.connection"));
			PropertyDescriptor maxCrossTrackError = this.createPropertyDescriptor(
					"maxCrossTrackError",
					this.dictionary.getString("property.connection.datalink.simulated.maxCrossTrackError.name"),
					this.dictionary.getString("property.connection.datalink.simulated.maxCrossTrackError.description"),
					this.dictionary.getString("property.connection.datalink.category.error"));
			PropertyDescriptor maxTimingError = this.createPropertyDescriptor(
					"maxTimingError",
					this.dictionary.getString("property.connection.datalink.simulated.maxTimingError.name"),
					this.dictionary.getString("property.connection.datalink.simulated.maxTimingError.description"),
					this.dictionary.getString("property.connection.datalink.category.error"));
			PropertyDescriptor errorProbability = this.createPropertyDescriptor(
					"errorProbability",
					this.dictionary.getString("property.connection.datalink.simulated.errorProbability.name"),
					this.dictionary.getString("property.connection.datalink.simulated.errorProbability.description"),
					this.dictionary.getString("property.connection.datalink.category.error"));
			
			PropertyDescriptor[] sdlDescriptors = new PropertyDescriptor[] {
					uplinkDelay,
					maxCrossTrackError,
					maxTimingError,
					errorProbability};
			
			descriptors = Stream.concat(
					Arrays.stream(descriptors), Arrays.stream(sdlDescriptors))
					.toArray(PropertyDescriptor[]::new);
			
		} catch (IntrospectionException e) {
			e.printStackTrace();
		}
		
		return descriptors;
	}
	
}
