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

import javax.validation.constraints.DecimalMax;
import javax.validation.constraints.DecimalMin;

/**
 * Realizes the properties bean of a RL environment.
 * 
 * @author Rafaela Seguro
 *
 */
public class RLEnvironmentProperties extends EnvironmentProperties {
	
	/** the default serial identification of this RL environment properties bean */
	private static final long serialVersionUID = 1L;
	
	/** the resolution of this planning continuum properties bean */
	@DecimalMin(value = "0", message = "{property.environment.rlenv.resolution.min}")
	@DecimalMax(value = "100000", message = "{property.environment.rlenv.resolution.max}")
    private double resolution = 50d;

    /**
     * Gets the resolution of this RL environment properties bean.
     * 
     * @return the resolution of this RL environment properties bean
     */
    public double getResolution() {
            return resolution;
    }

    /**
     * Sets the resolution of this RL environment properties bean.
     * 
     * @param resolution the resolution to be set
     */
    public void setResolution(double resolution) {
            this.resolution = resolution;
    }
    
    /**
	 * Determines whether or not this RL environment properties bean equals
	 * another RL environment properties bean based on their aggregated
	 * properties.
	 * 
	 * @param o the other RL environment properties bean
	 * 
	 * @return true, if the aggregated properties of this RL environment
	 *         properties bean equal the aggregated properties of the other
	 *         RL environment properties bean, false otherwise
	 * 
	 * @see EnvironmentProperties#equals(Object)
	 */
	@Override
	public final boolean equals(Object o) {
		boolean equals = super.equals(o);
		
		if (equals && (o instanceof RLEnvironmentProperties)) {
			RLEnvironmentProperties pcp = (RLEnvironmentProperties) o;
			equals = (this.resolution == pcp.resolution);
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this RL environment properties bean based on
	 * its aggregated properties.
	 * 
	 * @return the hash code of this RL environment properties bean based
	 *         on its aggregated properties
	 * 
	 * @see EnvironmentProperties#hashCode()
	 */
	@Override
	public final int hashCode() {
		return Objects.hash(super.hashCode(), this.resolution);
	}
	
}
