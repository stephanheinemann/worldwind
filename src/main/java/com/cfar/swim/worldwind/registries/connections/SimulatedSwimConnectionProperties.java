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

import java.util.Objects;

/**
 * Realizes the properties bean of a simulation SWIM connection.
 * 
 * @author Stephan Heinemann
 *
 */
public class SimulatedSwimConnectionProperties extends SwimConnectionProperties {

	/** the default resource directory of this simulated SWIM connection properties bean */
	public static final String SWIM_RESOURCE_DIRECTORY = "swim/xml";
	
	/** the default update period of this simulated SWIM connection properties bean */
	public static final int SWIM_UPDATE_PERIOD = 5000;
	
	/** the default update probability of this simulated SWIM connection properties bean */
	public static final float SWIM_UPDATE_PROBABILITY = 0.5f;
	
	/** the default update quantity of this simulated SWIM connection properties bean */
	public static final int SWIM_UPDATE_QUANTITY = 1;
	
	/** the resource directory of this simulated SWIM connection properties bean */
	private String resourceDirectory;
	
	/** the update period of this simulated SWIM connection properties bean */
	private long updatePeriod; // ms
	
	/** the update probability of this simulated SWIM connection properties bean */
	private float updateProbability;
	
	/** the update quantity of this simulated SWIM connection properties bean */
	private int updateQuantity;
	
	/**
	 * Constructs a new simulated SWIM connection properties bean.
	 */
	public SimulatedSwimConnectionProperties() {
		this.resourceDirectory = SimulatedSwimConnectionProperties.SWIM_RESOURCE_DIRECTORY;
		this.updatePeriod = SimulatedSwimConnectionProperties.SWIM_UPDATE_PERIOD;
		this.updateProbability = SimulatedSwimConnectionProperties.SWIM_UPDATE_PROBABILITY;
		this.updateQuantity = SimulatedSwimConnectionProperties.SWIM_UPDATE_QUANTITY;
	}
	
	/**
	 * Gets the resource directory of this simulated SWIM connection properties bean.
	 * 
	 * @return the resource directory of this simulated SWIM connection properties bean
	 */
	public String getResourceDirectory() {
		return this.resourceDirectory;
	}

	/**
	 * Sets the resource directory of this simulated SWIM connection properties bean.
	 * 
	 * @param resourceDirectory the resource directory to be set
	 */
	public void setResourceDirectory(String resourceDirectory) {
		this.resourceDirectory = resourceDirectory;
	}
	
	/**
	 * Gets the update period of this simulated SWIM connection properties bean.
	 * 
	 * @return the update period of this simulated SWIM connection properties bean
	 */
	public long getUpdatePeriod() {
		return this.updatePeriod;
	}
	
	/**
	 * Sets the update period of this simulated SWIM connection properties bean.
	 * 
	 * @param updatePeriod the update period to be set
	 */
	public void setUpdatePeriod(long updatePeriod) {
		this.updatePeriod = updatePeriod;
	}
	
	/**
	 * Gets the update probability of this simulated SWIM connection properties bean.
	 * 
	 * @return the update probability of this simulated SWIM connection properties bean
	 */
	public float getUpdateProbability() {
		return this.updateProbability;
	}
	
	/**
	 * Sets the update probability of this simulated SWIM connection properties bean.
	 * 
	 * @param updateProbability the update probability to be set
	 */
	public void setUpdateProbability(float updateProbability) {
		this.updateProbability = updateProbability;
	}
	
	/**
	 * Gets the update quantity of this simulated SWIM connection properties bean.
	 * 
	 * @return the update quantity of this simulated SWIM connection properties bean
	 */
	public int getUpdateQuantity() {
		return this.updateQuantity;
	}
	
	/**
	 * Sets the update quantity of this simulated SWIM connection properties bean.
	 * 
	 * @param updateQuantity the update quantity to be set
	 */
	public void setUpdateQuantity(int updateQuantity) {
		this.updateQuantity = updateQuantity;
	}
	
	/**
	 * Determines whether or not this simulated SWIM connection properties bean
	 * equals another simulated SWIM connection properties bean based on their
	 * aggregated properties.
	 * 
	 * @param o the other simulated SWIM connection properties bean
	 * 
	 * @return true, if the aggregated properties of this simulated SWIM
	 *         connection properties bean equal the aggregated properties of
	 *         the other simulated SWIM connection properties bean,
	 *         false otherwise
	 * 
	 * @see SwimConnectionProperties#equals(Object)
	 */
	@Override
	public final boolean equals(Object o) {
		boolean equals = super.equals(o);
		
		if (equals && (o instanceof SimulatedSwimConnectionProperties)) {
			SimulatedSwimConnectionProperties sscp = (SimulatedSwimConnectionProperties) o;
			equals = (this.resourceDirectory.equals(sscp.resourceDirectory))
					&& (this.updatePeriod == sscp.updatePeriod)
					&& (this.updateProbability == sscp.updateProbability)
					&& (this.updateQuantity == sscp.updateQuantity);
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this simulated SWIM connection properties bean
	 * based on its aggregated properties.
	 * 
	 * @return the hash code of this simulated SWIM connection properties bean
	 *         based on its aggregated properties
	 * 
	 * @see SwimConnectionProperties#hashCode()
	 */
	@Override
	public final int hashCode() {
		return Objects.hash(
				super.hashCode(),
				this.resourceDirectory,
				this.updatePeriod,
				this.updateProbability,
				this.updateQuantity);
	}
	
}
