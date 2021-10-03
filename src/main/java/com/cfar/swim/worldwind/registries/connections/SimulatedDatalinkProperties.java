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

import javax.validation.constraints.DecimalMax;
import javax.validation.constraints.DecimalMin;
import javax.validation.constraints.Max;
import javax.validation.constraints.Min;

/**
 * Realizes the properties bean of a simulated datalink.
 * 
 * @author Stephan Heinemann
 *
 */
public class SimulatedDatalinkProperties extends DatalinkProperties {
	
	/** the default serial identification of this simulated datalink properties bean */
	private static final long serialVersionUID = 1L;
	
	// TODO: aircraft (simulated performance)
	
	/** the uplink delay of this simulated datalink properties bean */
	@Min(value = 0, message = "{property.connection.datalink.simulated.uplinkDelay.min}")
	@Max(value = Long.MAX_VALUE, message = "{property.connection.datalink.simulated.uplinkDelay.max}")
	private long uplinkDelay = 500;
	
	/** the maximum possible cross track error of this simulated datalink properties bean */
	@Min(value = 0, message = "{property.connection.datalink.simulated.maxCrossTrackError.min}")
	@Max(value = Long.MAX_VALUE, message = "{property.connection.datalink.simulated.maxCrossTrackError.max}")
	private long maxCrossTrackError = 10;
	
	/** the maximum possible timing error of this simulated datalink properties bean */
	@Min(value = 0, message = "{property.connection.datalink.simulated.maxTimingError.min}")
	@Max(value = Long.MAX_VALUE, message = "{property.connection.datalink.simulated.maxTimingError.max}")
	private long maxTimingError = 60;
	
	/** the error probability of this simulated datalink properties bean */
	@DecimalMin(value = "0", message = "{property.connection.datalink.simulated.errorProbability.min}")
	@DecimalMax(value = "1", message = "{property.connection.datalink.simulated.errorProbability.max}")
	private float errorProbability;
	
	/**
	 * Gets the uplink delay of this simulated datalink properties bean in
	 * milliseconds.
	 * 
	 * @return the uplink delay of this simulated datalink properties bean in
	 *         milliseconds
	 */
	public long getUplinkDelay() {
		return this.uplinkDelay;
	}
	
	/**
	 * Sets the uplink delay of this simulated datalink properties bean in
	 * milliseconds.
	 * 
	 * @param uplinkDelay the uplink delay to be set in milliseconds
	 */
	public void setUplinkDelay(long uplinkDelay) {
		this.uplinkDelay = uplinkDelay;
	}
	
	/**
	 * Gets the maximum possible cross track error of this simulated datalink
	 * properties bean in meters.
	 * 
	 * @return the maximum possible cross track error of this simulated
	 *         datalink properties bean in meters
	 */
	public long getMaxCrossTrackError() {
		return this.maxCrossTrackError;
	}
	
	/**
	 * Sets the maximum possible cross track error of this simulated datalink
	 * properties bean in meters.
	 * 
	 * @param maxCrossTrackError the maximum possible cross track error to be
	 *                           set in meters
	 */
	public void setMaxCrossTrackError(long maxCrossTrackError) {
		this.maxCrossTrackError = maxCrossTrackError;
	}
	
	/**
	 * Gets the maximum possible timing error of this simulated datalink
	 * properties bean in seconds.
	 * 
	 * @return the maximum possible timing error of this simulated datalink
	 *         properties bean in seconds
	 */
	public long getMaxTimingError() {
		return this.maxTimingError;
	}
	
	/**
	 * Sets the maximum possible timing error of this simulated datalink
	 * properties bean in seconds.
	 * 
	 * @param maxTimingError the maximum possible timing error to be set in
	 *                       seconds
	 */
	public void setMaxTimingError(long maxTimingError) {
		this.maxTimingError = maxTimingError;
	}
	
	/**
	 * Gets the error probability of this simulated datalink.
	 * 
	 * @return the error probability of this simulated datalink
	 */
	public float getErrorProbability() {
		return this.errorProbability;
	}
	
	/**
	 * Sets the error probability of this simulated datalink.
	 * 
	 * @param errorProbability the error probability to be set
	 */
	public void setErrorProbability(float errorProbability) {
		this.errorProbability = errorProbability;
	}
	
	/**
	 * Determines whether or not this simulated datalink properties bean equals
	 * another simulated datalink properties bean based on their aggregated
	 * properties.
	 * 
	 * @param o the other simulated datalink properties bean
	 * 
	 * @return true, if the aggregated properties of this simulated datalink
	 *         properties bean equal the aggregated properties of the other
	 *         simulated datalink properties bean, false otherwise
	 * 
	 * @see DatalinkProperties#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = super.equals(o);
		
		if (equals) {
			SimulatedDatalinkProperties sdlp = (SimulatedDatalinkProperties) o;
			equals = (this.uplinkDelay == sdlp.uplinkDelay)
					&& (this.maxCrossTrackError == sdlp.maxCrossTrackError)
					&& (this.maxTimingError == sdlp.maxTimingError)
					&& (this.errorProbability == sdlp.errorProbability);
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this simulated datalink properties bean based on
	 * its aggregated properties.
	 * 
	 * @return the hash code of this simulated datalink properties bean based
	 *         on its aggregated properties
	 * 
	 * @see DatalinkProperties#hashCode()
	 */
	@Override
	public int hashCode() {
		return Objects.hash(
				super.hashCode(),
				this.uplinkDelay,
				this.maxCrossTrackError,
				this.maxTimingError,
				this.errorProbability);
	}
	
}
