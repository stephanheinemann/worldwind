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
package com.cfar.swim.worldwind.registries;

import java.beans.BeanDescriptor;
import java.beans.IntrospectionException;
import java.beans.PropertyDescriptor;
import java.beans.SimpleBeanInfo;
import java.util.ResourceBundle;

import org.controlsfx.property.BeanProperty;

import com.cfar.swim.worldwind.util.ResourceBundleLoader;

/**
 * Abstracts a properties bean information.
 * 
 * @author Stephan Heinemann
 *
 */
public abstract class PropertiesBeanInfo extends SimpleBeanInfo {

	/** the dictionary of this properties bean information */
	protected final ResourceBundle dictionary = ResourceBundleLoader.getDictionaryBundle();
	
	/** the properties bean class of this properties bean information */
	private final Class<? extends Properties<?>> beanClass;
	
	/**
	 * Constructs a properties bean information.
	 * 
	 * @param beanClass the properties bean class of the properties bean
	 */
	public PropertiesBeanInfo(Class<? extends Properties<?>> beanClass) {
		this.beanClass = beanClass;
	}
	
	/**
	 * Gets the bean descriptor of this properties bean information.
	 * 
	 * @return the bean descriptor of this properties bean information
	 * 
	 * @see java.beans.SimpleBeanInfo#getBeanDescriptor()
	 */
	@Override
	public BeanDescriptor getBeanDescriptor() {
		return new BeanDescriptor(this.beanClass);
	}
	
	/**
	 * Creates a property descriptor for a property of the properties bean
	 * class of this properties bean information.
	 * 
	 * @param propertyName the name of the property
	 * @param displayName the display name of the property
	 * @param description the description of the property
	 * @param category the category of the property
	 * 
	 * @return the property descriptor of the property
	 * 
	 * @throws IntrospectionException
	 */
	public PropertyDescriptor createPropertyDescriptor(
			String propertyName,
			String displayName,
			String description,
			String category) throws IntrospectionException {
		
		PropertyDescriptor property = new PropertyDescriptor(
				propertyName, this.beanClass);
		property.setDisplayName(displayName);
		property.setShortDescription(description);
		property.setValue(BeanProperty.CATEGORY_LABEL_KEY, category);
		
		return property;
	}
	
}
