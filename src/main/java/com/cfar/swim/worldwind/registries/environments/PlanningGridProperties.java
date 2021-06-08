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
package com.cfar.swim.worldwind.registries.environments;

import java.util.Objects;

import javax.validation.constraints.Max;
import javax.validation.constraints.Min;

/**
 * Realizes the properties bean of a planning grid environment.
 * 
 * @author Stephan Heinemann
 *
 */
public class PlanningGridProperties extends EnvironmentProperties {

	/** the longest axis division of this planning grid properties bean */
	@Min(value = 1, message = "{property.environment.grid.division.min}")
	@Max(value = Integer.MAX_VALUE, message = "{property.environment.grid.division.max}")
	private int division;
	
	/**
	 * Constructs a new planning grid properties bean with a default
	 * division (number of cells) for the longest axis.
	 */
	public PlanningGridProperties() {
		this.division = 10;
	}
	
	/**
	 * Constructs a new planning grid properties bean with a specified
	 * division (number of cells) for the longest axis.
	 * 
	 * @param division the division for the longest axis
	 */
	public PlanningGridProperties(int division) {
		this.division = division;
	}
	
	/**
	 * Gets the division (number of cells) for the longest axis of this
	 * planning grid properties bean.
	 * 
	 * @return the division for the longest axis of this planning grid
	 *         properties bean
	 */
	public int getDivision() {
		return this.division;
	}
	
	/**
	 * Sets the division (number of cells) for the longest axis of this
	 * planning grid properties bean.
	 * 
	 * @param division the division for the longest axis to be set
	 */
	public void setDivision(int division) {
		this.division = division;
	}
	
	/**
	 * Determines whether or not this planning grid properties bean equals
	 * another planning grid properties bean based on their aggregated
	 * properties.
	 * 
	 * @param o the other planning grid properties bean
	 * 
	 * @return true, if the aggregated properties of this planning grid
	 *         properties bean equal the aggregated properties of the other
	 *         planning grid properties bean, false otherwise
	 * 
	 * @see EnvironmentProperties#equals(Object)
	 */
	@Override
	public final boolean equals(Object o) {
		boolean equals = super.equals(o);
		
		if (equals && (o instanceof PlanningGridProperties)) {
			PlanningGridProperties pgp = (PlanningGridProperties) o;
			equals = (this.division == pgp.division);
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this planning grid properties bean based on its
	 * aggregated properties.
	 * 
	 * @return the hash code of this planning grid properties bean based on its
	 *         aggregated properties
	 * 
	 * @see EnvironmentProperties#hashCode()
	 */
	@Override
	public final int hashCode() {
		return Objects.hash(super.hashCode(), this.division);
	}
	
}
