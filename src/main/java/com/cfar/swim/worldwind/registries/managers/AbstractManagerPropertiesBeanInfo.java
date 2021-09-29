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
package com.cfar.swim.worldwind.registries.managers;

import java.beans.IntrospectionException;
import java.beans.PropertyDescriptor;

import com.cfar.swim.worldwind.registries.PropertiesBeanInfo;

/**
* Realizes an abstract manager properties bean information customizing the
* descriptors for each property.
* 
* @author Stephan Heinemann
*
*/
public class AbstractManagerPropertiesBeanInfo extends PropertiesBeanInfo {
	
	/**
	 * Constructs an abstract manager properties bean information.
	 */
	public AbstractManagerPropertiesBeanInfo() {
		super(AbstractManagerProperties.class);
	}
	
	/**
	 * Constructs an abstract manager properties bean information.
	 * 
	 * @param beanClass the properties bean class of the properties bean
	 */
	public AbstractManagerPropertiesBeanInfo(
			Class<? extends AbstractManagerProperties> beanClass) {
		super(beanClass);
	}
	
	/**
	 * Customizes the property descriptors for each property of an abstract
	 * manager properties bean.
	 * 
	 * @return the array of customized property descriptors
	 * 
	 * @see java.beans.SimpleBeanInfo#getPropertyDescriptors()
	 */
	@Override
	public PropertyDescriptor[] getPropertyDescriptors() {
		PropertyDescriptor[] descriptors = super.getPropertyDescriptors();
		
		try {
			PropertyDescriptor costPolicy = this.createPropertyDescriptor(
					"costPolicy",
					this.dictionary.getString("property.manager.costPolicy.name"),
					this.dictionary.getString("property.manager.costPolicy.description"),
					this.dictionary.getString("property.manager.category.policies"));
			PropertyDescriptor riskPolicy = this.createPropertyDescriptor(
					"riskPolicy",
					this.dictionary.getString("property.manager.riskPolicy.name"),
					this.dictionary.getString("property.manager.riskPolicy.description"),
					this.dictionary.getString("property.manager.category.policies"));
			PropertyDescriptor featureHorizon = this.createPropertyDescriptor(
					"featureHorizon",
					this.dictionary.getString("property.manager.featureHorizon.name"),
					this.dictionary.getString("property.manager.featureHorizon.description"),
					this.dictionary.getString("property.manager.category.managing"));
			PropertyDescriptor minDeliberation = this.createPropertyDescriptor(
					"minDeliberation",
					this.dictionary.getString("property.manager.minDeliberation.name"),
					this.dictionary.getString("property.manager.minDeliberation.description"),
					this.dictionary.getString("property.manager.category.online"));
			PropertyDescriptor maxDeliberation = this.createPropertyDescriptor(
					"maxDeliberation",
					this.dictionary.getString("property.manager.maxDeliberation.name"),
					this.dictionary.getString("property.manager.maxDeliberation.description"),
					this.dictionary.getString("property.manager.category.online"));
			PropertyDescriptor maxCrossTrackError = this.createPropertyDescriptor(
					"maxCrossTrackError",
					this.dictionary.getString("property.manager.maxCrossTrackError.name"),
					this.dictionary.getString("property.manager.maxCrossTrackError.description"),
					this.dictionary.getString("property.manager.category.online"));
			PropertyDescriptor maxTimingError = this.createPropertyDescriptor(
					"maxTimingError",
					this.dictionary.getString("property.manager.maxTimingError.name"),
					this.dictionary.getString("property.manager.maxTimingError.description"),
					this.dictionary.getString("property.manager.category.online"));
			PropertyDescriptor maxTakeOffHorizontalError = this.createPropertyDescriptor(
					"maxTakeOffHorizontalError",
					this.dictionary.getString("property.manager.maxTakeOffHorizontalError.name"),
					this.dictionary.getString("property.manager.maxTakeOffHorizontalError.description"),
					this.dictionary.getString("property.manager.category.online"));
			PropertyDescriptor maxTakeOffTimingError = this.createPropertyDescriptor(
					"maxTakeOffTimingError",
					this.dictionary.getString("property.manager.maxTakeOffTimingError.name"),
					this.dictionary.getString("property.manager.maxTakeOffTimingError.description"),
					this.dictionary.getString("property.manager.category.online"));
			PropertyDescriptor maxLandingHorizontalError = this.createPropertyDescriptor(
					"maxLandingHorizontalError",
					this.dictionary.getString("property.manager.maxLandingHorizontalError.name"),
					this.dictionary.getString("property.manager.maxLandingHorizontalError.description"),
					this.dictionary.getString("property.manager.category.online"));
			PropertyDescriptor maxLandingTimingError = this.createPropertyDescriptor(
					"maxLandingTimingError",
					this.dictionary.getString("property.manager.maxLandingTimingError.name"),
					this.dictionary.getString("property.manager.maxLandingTimingError.description"),
					this.dictionary.getString("property.manager.category.online"));
		
			descriptors = new PropertyDescriptor[] {
					costPolicy,
					riskPolicy,
					featureHorizon,
					minDeliberation,
					maxDeliberation,
					maxCrossTrackError,
					maxTimingError,
					maxTakeOffHorizontalError,
					maxTakeOffTimingError,
					maxLandingHorizontalError,
					maxLandingTimingError};
		
		} catch (IntrospectionException e) {
			e.printStackTrace();
		}
		
		return descriptors;
	}

}
