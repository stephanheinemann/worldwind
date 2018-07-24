/**
 * Copyright (c) 2016, Stephan Heinemann (UVic Center for Aerospace Research)
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
package com.cfar.swim.worldwind.registries.environments;

import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.registries.Properties;

/**
 * Abstracts environment properties applicable to all environments.
 * 
 * @author Stephan Heinemann
 *
 */
public abstract class EnvironmentProperties implements Properties<Environment> {

	/** the floor of this environment properties bean */
	protected double floor;
	
	/** the ceiling of this environment properties bean */
	protected double ceiling;
	
	/**
	 * Constructs a new environment properties bean using default floor and
	 * ceiling property values.
	 */
	protected EnvironmentProperties() {
		this.floor = 95d;
		this.ceiling = 109d;
	}
	
	/**
	 * Constructs a new environment properties bean with specified floor and
	 * ceiling property values.
	 * 
	 * @param floor the floor of this environment properties bean in meters
	 * @param ceiling the ceiling of this environment properties bean in meters
	 */
	public EnvironmentProperties(double floor, double ceiling) {
		this.floor = floor;
		this.ceiling = ceiling;
	}
	
	/**
	 * Gets the floor of this environment properties bean.
	 * 
	 * @return the floor of this environment properties bean in meters
	 */
	public double getFloor() {
		return floor;
	}
	
	/**
	 * Sets the floor of this environment properties bean.
	 * 
	 * @param floor the floor to be set in meters
	 */
	public void setFloor(double floor) {
		this.floor = floor;
	}
	
	/**
	 * Gets the ceiling of this environment properties bean.
	 * 
	 * @return the ceiling of this environment properties bean in meters
	 */
	public double getCeiling() {
		return ceiling;
	}
	
	/**
	 * Sets the ceiling of this environment properties bean.
	 * 
	 * @param ceiling the ceiling to be set in meters
	 */
	public void setCeiling(double ceiling) {
		this.ceiling = ceiling;
	}
	
	/**
	 * Clones this environment properties bean.
	 * 
	 * @return a clone of this environment properties bean
	 * 
	 * @see Properties#clone()
	 */
	@Override
	public EnvironmentProperties clone() {
		EnvironmentProperties clone = null;
		try {
			clone = (EnvironmentProperties) super.clone();
		} catch (CloneNotSupportedException e) {
			e.printStackTrace();
		}
		return clone;
	}
	
}
