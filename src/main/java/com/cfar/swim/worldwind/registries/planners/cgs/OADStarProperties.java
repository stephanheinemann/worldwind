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
package com.cfar.swim.worldwind.registries.planners.cgs;

import java.util.Objects;

import javax.validation.constraints.Max;
import javax.validation.constraints.Min;

import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.registries.planners.OnlinePlannerProperties;

/**
 * Realizes the properties bean of an online anytime dynamic A* planner.
 * 
 * @author Stephan Heinemann
 *
 */
@OADStarValidDeliberation
public class OADStarProperties extends ADStarProperties implements OnlinePlannerProperties {
	
	/** the default serial identification of this OAD* planner properties bean */
	private static final long serialVersionUID = 1L;
	
	/** the minimum deliberation duration of this OAD* planner properties bean */
	private long minDeliberation = 10l;
	
	/** the maximum deliberation duration of this OAD* planner properties bean */
	private long maxDeliberation = 60l;
	
	/** the maximum acceptable cross track error of this OAD* planner properties bean */
	@Min(value = 0, message = "{property.planner.oads.maxCrossTrackError.min}")
	@Max(value = Long.MAX_VALUE, message = "{property.planner.oads.maxCrossTrackError.max}")
	private long maxCrossTrackError = 10l;
	
	/** the maximum acceptable timing error of this OAD* planner properties bean */
	@Min(value = 0, message = "{property.planner.oads.maxTimingError.min}")
	@Max(value = Long.MAX_VALUE, message = "{property.planner.oads.maxTimingError.max}")
	private long maxTimingError = 60l;
	
	/** the maximum acceptable horizontal take-off error of this OAD* planner properties bean */
	@Min(value = 0, message = "{property.planner.oads.maxTakeOffHorizontalError.min}")
	@Max(value = Long.MAX_VALUE, message = "{property.planner.oads.maxTakeOffHorizontalError.max}")
	private long maxTakeOffHorizontalError = 5l;
	
	/** the maximum acceptable take-off timing error of this OAD* planner properties bean */
	@Min(value = 0, message = "{property.planner.oads.maxTakeOffTimingError.min}")
	@Max(value = Long.MAX_VALUE, message = "{property.planner.oads.maxTakeOffTimingError.max}")
	private long maxTakeOffTimingError = 30l;
	
	/** the maximum acceptable horizontal landing error of this OAD* planner properties bean */
	@Min(value = 0, message = "{property.planner.oads.maxLandingHorizontalError.min}")
	@Max(value = Long.MAX_VALUE, message = "{property.planner.oads.maxLandingHorizontalError.max}")
	private long maxLandingHorizontalError = 5l;
	
	/** the maximum acceptable landing timing error of this OAD* planner properties bean */
	@Min(value = 0, message = "{property.planner.oads.maxLandingTimingError.min}")
	@Max(value = Long.MAX_VALUE, message = "{property.planner.oads.maxLandingTimingError.max}")
	private long maxLandingTimingError = 60l;
	
	/**
	 * Constructs a new OAD* planner properties bean.
	 */
	public OADStarProperties() {
		super();
	}
	
	/**
	 * Constructs a new OAD* planner properties bean with specified cost
	 * and risk policy property values.
	 * 
	 * @param costPolicy the cost policy of this OAD* planner properties bean
	 * @param riskPolicy the risk policy of this OAD* planner properties bean
	 */
	public OADStarProperties(CostPolicy costPolicy, RiskPolicy riskPolicy) {
		super(costPolicy, riskPolicy);
	}
	
	/**
	 * Gets the minimum deliberation duration of this OAD* planner properties
	 * bean in seconds.
	 * 
	 * @return the minimum deliberation duration of this OAD* planner
	 *         properties bean in seconds
	 * 
	 * @see OnlinePlannerProperties#getMinDeliberation()
	 */
	@Override
	public long getMinDeliberation() {
		return this.minDeliberation;
	}
	
	/**
	 * Sets the minimum deliberation duration of this OAD* planner properties
	 * bean in seconds.
	 * 
	 * @param minDeliberation the minimum deliberation duration to be set in
	 *                        seconds
	 * 
	 * @see OnlinePlannerProperties#setMinDeliberation(long)
	 */
	@Override
	public void setMinDeliberation(long minDeliberation) {
		this.minDeliberation = minDeliberation;
	}
	
	/**
	 * Gets the maximum deliberation duration of this OAD* planner properties
	 * bean in seconds.
	 * 
	 * @return the maximum deliberation duration of this OAD* planner
	 *         properties bean in seconds
	 * 
	 * @see OnlinePlannerProperties#getMaxDeliberation()
	 */
	@Override
	public long getMaxDeliberation() {
		return this.maxDeliberation;
	}
	
	/**
	 * Sets the maximum deliberation duration of this OAD* planner properties
	 * bean in seconds.
	 * 
	 * @param maxDeliberation the maximum deliberation duration to be set in
	 *                        seconds
	 * 
	 * @see OnlinePlannerProperties#setMaxDeliberation(long)
	 */
	@Override
	public void setMaxDeliberation(long maxDeliberation) {
		this.maxDeliberation = maxDeliberation;
	}
	
	/**
	 * Gets the maximum acceptable cross track error of this OAD* planner
	 * properties bean in meters.
	 * 
	 * @return the maximum acceptable cross track error of this OAD* planner
	 *         properties bean in meters
	 * 
	 * @see OnlinePlannerProperties#getMaxCrossTrackError()
	 */
	@Override
	public long getMaxCrossTrackError() {
		return this.maxCrossTrackError;
	}
	
	/**
	 * Sets the maximum acceptable cross track error of this OAD* planner
	 * properties bean in meters.
	 * 
	 * @param maxCrossTrackError the maximum acceptable cross track error to be
	 *                           set in meters
	 * 
	 * @see OnlinePlannerProperties#setMaxCrossTrackError(long)
	 */
	@Override
	public void setMaxCrossTrackError(long maxCrossTrackError) {
		this.maxCrossTrackError = maxCrossTrackError;
	}
	
	/**
	 * Gets the maximum acceptable timing error of this OAD* planner properties
	 * bean in seconds.
	 * 
	 * @return the maximum acceptable timing error of this OAD* planner
	 *         properties bean in seconds
	 * 
	 * @see OnlinePlannerProperties#getMaxTimingError()
	 */
	@Override
	public long getMaxTimingError() {
		return this.maxTimingError;
	}
	
	/**
	 * Sets the maximum acceptable timing error of this OAD* planner properties
	 * bean in seconds.
	 * 
	 * @param maxTimingError the maximum acceptable timing error to be set in
	 *                       seconds
	 * 
	 * @see OnlinePlannerProperties#setMaxTimingError(long)
	 */
	@Override
	public void setMaxTimingError(long maxTimingError) {
		this.maxTimingError = maxTimingError;
	}
	
	/**
	 * Gets the maximum horizontal take-off error of this OAD* planner
	 * properties bean in meters.
	 * 
	 * @return the maximum horizontal take-off error of this OAD* planner
	 *         properties bean in meters
	 *
	 * @see OnlinePlannerProperties#getMaxTakeOffHorizontalError()
	 */
	@Override
	public long getMaxTakeOffHorizontalError() {
		return this.maxTakeOffHorizontalError;
	}
	
	/**
	 * Sets the maximum horizontal take-off error of this OAD* planner
	 * properties bean in meters.
	 * 
	 * @param maxTakeOffHorizontalError the maximum horizontal take-off error
	 *                                  to be set in meters
	 *
	 * @see OnlinePlannerProperties#setMaxTakeOffHorizontalError(long)
	 */
	@Override
	public void setMaxTakeOffHorizontalError(long maxTakeOffHorizontalError) {
		this.maxTakeOffHorizontalError = maxTakeOffHorizontalError;
	}
	
	/**
	 * Gets the maximum take-off timing error of this OAD* planner properties
	 * bean in seconds.
	 * 
	 * @return the maximum take-off timing error of this OAD* planner
	 *         properties bean in seconds
	 *
	 * @see OnlinePlannerProperties#getMaxTakeOffTimingError()
	 */
	@Override
	public long getMaxTakeOffTimingError() {
		return this.maxTakeOffTimingError;
	}
	
	/**
	 * Sets the maximum take-off timing error of this OAD* planner properties
	 * bean in seconds.
	 * 
	 * @param maxTakeOffTimingError the maximum take-off timing error to be set
	 *                              in seconds
	 *
	 * @see OnlinePlannerProperties#setMaxTakeOffTimingError(long)
	 */
	@Override
	public void setMaxTakeOffTimingError(long maxTakeOffTimingError) {
		this.maxTakeOffTimingError = maxTakeOffTimingError;
	}
	
	/**
	 * Gets the maximum horizontal landing error of this OAD* planner
	 * properties bean in meters.
	 * 
	 * @return the maximum horizontal landing error of this OAD* planner
	 *         properties bean in meters
	 *
	 * @see OnlinePlannerProperties#getMaxLandingHorizontalError()
	 */
	@Override
	public long getMaxLandingHorizontalError() {
		return this.maxLandingHorizontalError;
	}
	
	/**
	 * Sets the maximum horizontal landing error of this OAD* planner
	 * properties bean in meters.
	 * 
	 * @param maxLandingHorizontalError the maximum horizontal landing error
	 *                                  to be set in meters
	 *
	 * @see OnlinePlannerProperties#setMaxLandingHorizontalError(long)
	 */
	@Override
	public void setMaxLandingHorizontalError(long maxLandingHorizontalError) {
		this.maxLandingHorizontalError = maxLandingHorizontalError;
	}
	
	/**
	 * Gets the maximum landing timing error of this OAD* planner properties
	 * bean in seconds.
	 * 
	 * @return the maximum landing timing error of this OAD* planner
	 *         properties bean in seconds
	 *
	 * @see OnlinePlannerProperties#getMaxLandingTimingError()
	 */
	@Override
	public long getMaxLandingTimingError() {
		return this.maxLandingTimingError;
	}
	
	/**
	 * Sets the maximum landing timing error of this OAD* planner properties
	 * bean in seconds.
	 * 
	 * @param maxLandingTimingError the maximum take-off timing error to be set
	 *                              in seconds
	 *
	 * @see OnlinePlannerProperties#setMaxLandingTimingError(long)
	 */
	@Override
	public void setMaxLandingTimingError(long maxLandingTimingError) {
		this.maxLandingTimingError = maxLandingTimingError;
	}

	/**
	 * Determines whether or not this OAD* planner properties bean equals
	 * another OAD* planner properties bean based on their aggregated
	 * properties.
	 * 
	 * @param o the other OAD* planner properties bean
	 * 
	 * @return true, if the aggregated properties of this OAD* planner
	 *         properties bean equal the aggregated properties of the other
	 *         OAD* planner properties bean, false otherwise
	 * 
	 * @see ADStarProperties#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = super.equals(o);
		
		if (equals) {
			OADStarProperties properties = (OADStarProperties) o;
			equals = (this.minDeliberation == properties.minDeliberation)
					&& (this.maxDeliberation == properties.maxDeliberation)
					&& (this.maxCrossTrackError == properties.maxCrossTrackError)
					&& (this.maxTimingError == properties.maxTimingError)
					&& (this.maxTakeOffHorizontalError == properties.maxTakeOffHorizontalError)
					&& (this.maxTakeOffTimingError == properties.maxTakeOffTimingError)
					&& (this.maxLandingHorizontalError == properties.maxLandingHorizontalError)
					&& (this.maxLandingTimingError == properties.maxLandingTimingError);
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this OAD* planner properties bean based on its
	 * aggregated properties.
	 * 
	 * @return the hash code of this OAD* planner properties bean based on
	 *         its aggregated properties
	 * 
	 * @see ADStarProperties#hashCode()
	 */
	@Override
	public int hashCode() {
		return Objects.hash(
				super.hashCode(),
				this.minDeliberation,
				this.maxDeliberation,
				this.maxCrossTrackError,
				this.maxTimingError,
				this.maxTakeOffHorizontalError,
				this.maxTakeOffTimingError,
				this.maxLandingHorizontalError,
				this.maxLandingTimingError);
	}

}
