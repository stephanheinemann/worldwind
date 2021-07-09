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

import com.cfar.swim.worldwind.registries.PropertiesBeanInfo;

/**
 * Realizes a SWIM connection properties bean information customizing the
 * descriptors for each property.
 * 
 * @author Stephan Heinemann
 *
 */
public class SwimConnectionPropertiesBeanInfo extends PropertiesBeanInfo {

	/**
	 * Constructs a SWIM connection properties bean information.
	 */
	public SwimConnectionPropertiesBeanInfo() {
		super(SwimConnectionProperties.class);
	}
	
	/**
	 * Constructs a SWIM connection properties bean information.
	 * 
	 * @param beanClass the properties bean class of the properties bean
	 */
	public SwimConnectionPropertiesBeanInfo(
			Class<? extends SwimConnectionProperties> beanClass) {
		super(beanClass);
	}

	/**
	 * Customizes the property descriptors for each property of a SWIM
	 * connection properties bean.
	 * 
	 * @return the array of customized property descriptors
	 * 
	 * @see java.beans.SimpleBeanInfo#getPropertyDescriptors()
	 */
	@Override
	public PropertyDescriptor[] getPropertyDescriptors() {
		PropertyDescriptor[] descriptors = super.getPropertyDescriptors();
		
		try {
			PropertyDescriptor subscribesAIXM = this.createPropertyDescriptor(
					"subscribesAIXM",
					this.dictionary.getString("property.connection.swim.subscribesAIXM.name"),
					this.dictionary.getString("property.connection.swim.subscribesAIXM.description"),
					this.dictionary.getString("property.connection.swim.category.topics"));
			PropertyDescriptor subscribesFIXM = this.createPropertyDescriptor(
					"subscribesFIXM",
					this.dictionary.getString("property.connection.swim.subscribesFIXM.name"),
					this.dictionary.getString("property.connection.swim.subscribesFIXM.description"),
					this.dictionary.getString("property.connection.swim.category.topics"));
			PropertyDescriptor subscribesWXXM = this.createPropertyDescriptor(
					"subscribesWXXM",
					this.dictionary.getString("property.connection.swim.subscribesWXXM.name"),
					this.dictionary.getString("property.connection.swim.subscribesAIXM.description"),
					this.dictionary.getString("property.connection.swim.category.topics"));
			PropertyDescriptor subscribesIWXXM = this.createPropertyDescriptor(
					"subscribesIWXXM",
					this.dictionary.getString("property.connection.swim.subscribesIWXXM.name"),
					this.dictionary.getString("property.connection.swim.subscribesAIXM.description"),
					this.dictionary.getString("property.connection.swim.category.topics"));
			PropertyDescriptor subscribesAMXM = this.createPropertyDescriptor(
					"subscribesAMXM",
					this.dictionary.getString("property.connection.swim.subscribesAMXM.name"),
					this.dictionary.getString("property.connection.swim.subscribesAIXM.description"),
					this.dictionary.getString("property.connection.swim.category.topics"));
			
			descriptors = new PropertyDescriptor[] {
					subscribesAIXM,
					subscribesFIXM,
					subscribesWXXM,
					subscribesIWXXM,
					subscribesAMXM};
		
		} catch (IntrospectionException e) {
			e.printStackTrace();
		}
		
		return descriptors;
	}
	
}
