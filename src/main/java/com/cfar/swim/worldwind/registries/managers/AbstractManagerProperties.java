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

import java.util.Objects;

import javax.validation.constraints.DecimalMax;
import javax.validation.constraints.DecimalMin;
import javax.validation.constraints.Max;
import javax.validation.constraints.Min;

import com.cfar.swim.worldwind.managing.Features;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.registries.Properties;

/**
 * Abstracts manager properties applicable to all managers.
 * 
 * @author Stephan Heinemann
 *
 */
@AbstractManagerValidDeliberation
public abstract class AbstractManagerProperties implements ManagerProperties {
	
	/** the cost policy of this manager properties bean */
	CostPolicy costPolicy;
	
	/** the risk policy of this manager properties bean */
	RiskPolicy riskPolicy;
	
	/** the feature horizon of this manager properties bean */
	@DecimalMin(value = "0", message = "{property.manager.featureHorizon.min}")
	@DecimalMax(value = "60", message = "{property.manager.featureHorizon.max}")
	double featureHorizon = 5l;
	
	/** the minimum deliberation duration of this manager properties bean */
	private long minDeliberation = 10l;
	
	/** the maximum deliberation duration of this manager properties bean */
	private long maxDeliberation = 60l;
	
	/** the maximum acceptable cross track error of this manager properties bean */
	@Min(value = 0, message = "{property.manager.maxCrossTrackError.min}")
	@Max(value = Long.MAX_VALUE, message = "{property.manager.maxCrossTrackError.max}")
	private long maxCrossTrackError = 10l;
	
	/** the maximum acceptable timing error of this manager properties bean */
	@Min(value = 0, message = "{property.manager.maxTimingError.min}")
	@Max(value = Long.MAX_VALUE, message = "{property.manager.maxTimingError.max}")
	private long maxTimingError = 60l;
	
	/** the maximum acceptable horizontal take-off error of this manager properties bean */
	@Min(value = 0, message = "{property.manager.maxTakeOffHorizontalError.min}")
	@Max(value = Long.MAX_VALUE, message = "{property.manager.maxTakeOffHorizontalError.max}")
	private long maxTakeOffHorizontalError = 5l;
	
	/** the maximum acceptable take-off timing error of this manager properties bean */
	@Min(value = 0, message = "{property.manager.maxTakeOffTimingError.min}")
	@Max(value = Long.MAX_VALUE, message = "{property.manager.maxTakeOffTimingError.max}")
	private long maxTakeOffTimingError = 30l;
	
	/** the maximum acceptable horizontal landing error of this manager properties bean */
	@Min(value = 0, message = "{property.manager.maxLandingHorizontalError.min}")
	@Max(value = Long.MAX_VALUE, message = "{property.manager.maxLandingHorizontalError.max}")
	private long maxLandingHorizontalError = 5l;
	
	/** the maximum acceptable landing timing error of this manager properties bean */
	@Min(value = 0, message = "{property.manager.maxLandingTimingError.min}")
	@Max(value = Long.MAX_VALUE, message = "{property.manager.maxLandingTimingError.max}")
	private long maxLandingTimingError = 60l;
	
	/**
	 * Constructs a new manager properties bean using default cost and risk
	 * policy property values.
	 */
	public AbstractManagerProperties() {
		this.costPolicy = CostPolicy.AVERAGE;
		this.riskPolicy = RiskPolicy.SAFETY;
		this.featureHorizon = Features.FEATURE_HORIZON.getSeconds() / 60d;
	}
	
	/**
	 * Constructs a new manager properties bean with specified cost and risk
	 * policy property values.
	 * 
	 * @param costPolicy the cost policy of this manager properties bean
	 * @param riskPolicy the risk policy of this manager properties bean
	 */
	public AbstractManagerProperties(CostPolicy costPolicy, RiskPolicy riskPolicy) {
		this.costPolicy = costPolicy;
		this.riskPolicy = riskPolicy;
	}
	
	/**
	 * Gets the cost policy of this manager properties bean.
	 * 
	 * @return the cost policy of this manager properties bean
	 * 
	 * @see ManagerProperties#getCostPolicy()
	 */
	@Override
	public CostPolicy getCostPolicy() {
		return this.costPolicy;
	}
	
	/**
	 * Sets the cost policy of this manager properties bean.
	 * 
	 * @param costPolicy the cost policy to be set
	 * 
	 * @see ManagerProperties#setCostPolicy(CostPolicy)
	 */
	@Override
	public void setCostPolicy(CostPolicy costPolicy) {
		this.costPolicy = costPolicy;
	}
	
	/**
	 * Gets the risk policy of this manager properties bean.
	 * 
	 * @return the risk policy of this manager properties bean
	 * 
	 * @see ManagerProperties#getRiskPolicy()
	 */
	@Override
	public RiskPolicy getRiskPolicy() {
		return this.riskPolicy;
	}
	
	/**
	 * Sets the risk policy of this manager properties bean.
	 * 
	 * @param riskPolicy the risk policy to be set
	 * 
	 * @see ManagerProperties#setRiskPolicy(RiskPolicy)
	 */
	@Override
	public void setRiskPolicy(RiskPolicy riskPolicy) {
		this.riskPolicy = riskPolicy;
	}
	
	/**
	 * Gets the feature horizon of this manager properties bean.
	 * 
	 * @return the feature horizon of this manager properties bean
	 * 
	 * @see ManagerProperties#getFeatureHorizon()
	 */
	@Override
	public double getFeatureHorizon() {
		return this.featureHorizon;
	}
	
	/**
	 * Sets the feature horizon of this manager properties bean.
	 * 
	 * @param featureHorizon the feature horizon to be set
	 * 
	 * @see ManagerProperties#setFeatureHorizon(double)
	 */
	@Override
	public void setFeatureHorizon(double featureHorizon) {
		this.featureHorizon = featureHorizon;
	}
	
	/**
	 * Gets the minimum deliberation duration of this manager properties bean
	 * in seconds.
	 * 
	 * @return the minimum deliberation duration of this manager properties
	 *         bean in seconds
	 * 
	 * @see ManagerProperties#getMinDeliberation()
	 */
	@Override
	public long getMinDeliberation() {
		return this.minDeliberation;
	}
	
	/**
	 * Sets the minimum deliberation duration of this manager properties bean
	 * in seconds.
	 * 
	 * @param minDeliberation the minimum deliberation duration to be set in
	 *                        seconds
	 * 
	 * @see ManagerProperties#setMinDeliberation(long)
	 */
	@Override
	public void setMinDeliberation(long minDeliberation) {
		this.minDeliberation = minDeliberation;
	}
	
	/**
	 * Gets the maximum deliberation duration of this manager properties bean
	 * in seconds.
	 * 
	 * @return the maximum deliberation duration of this manager properties
	 *         bean in seconds
	 * 
	 * @see ManagerProperties#getMaxDeliberation()
	 */
	@Override
	public long getMaxDeliberation() {
		return this.maxDeliberation;
	}
	
	/**
	 * Sets the maximum deliberation duration of this manager properties bean
	 * in seconds.
	 * 
	 * @param maxDeliberation the maximum deliberation duration to be set in
	 *                        seconds
	 * 
	 * @see ManagerProperties#setMaxDeliberation(long)
	 */
	@Override
	public void setMaxDeliberation(long maxDeliberation) {
		this.maxDeliberation = maxDeliberation;
	}
	
	/**
	 * Gets the maximum acceptable cross track error of this manager properties
	 * bean in meters.
	 * 
	 * @return the maximum acceptable cross track error of this manager
	 *         properties bean in meters
	 * 
	 * @see ManagerProperties#getMaxCrossTrackError()
	 */
	@Override
	public long getMaxCrossTrackError() {
		return this.maxCrossTrackError;
	}
	
	/**
	 * Sets the maximum acceptable cross track error of this manager properties
	 * bean in meters.
	 * 
	 * @param maxCrossTrackError the maximum acceptable cross track error to be
	 *                           set in meters
	 * 
	 * @see ManagerProperties#setMaxCrossTrackError(long)
	 */
	@Override
	public void setMaxCrossTrackError(long maxCrossTrackError) {
		this.maxCrossTrackError = maxCrossTrackError;
	}
	
	/**
	 * Gets the maximum acceptable timing error of this manager properties bean
	 * in seconds.
	 * 
	 * @return the maximum acceptable timing error of this manager properties
	 *         bean in seconds
	 * 
	 * @see ManagerProperties#getMaxTimingError()
	 */
	@Override
	public long getMaxTimingError() {
		return this.maxTimingError;
	}
	
	/**
	 * Sets the maximum acceptable timing error of this manager properties bean
	 * in seconds.
	 * 
	 * @param maxTimingError the maximum acceptable timing error to be set in
	 *                       seconds
	 * 
	 * @see ManagerProperties#setMaxTimingError(long)
	 */
	@Override
	public void setMaxTimingError(long maxTimingError) {
		this.maxTimingError = maxTimingError;
	}
	
	/**
	 * Gets the maximum horizontal take-off error of this manager properties
	 * bean in meters.
	 * 
	 * @return the maximum horizontal take-off error of this manager properties
	 *         bean in meters
	 *
	 * @see ManagerProperties#getMaxTakeOffHorizontalError()
	 */
	@Override
	public long getMaxTakeOffHorizontalError() {
		return this.maxTakeOffHorizontalError;
	}
	
	/**
	 * Sets the maximum horizontal take-off error of this manager properties
	 * bean in meters.
	 * 
	 * @param maxTakeOffHorizontalError the maximum horizontal take-off error
	 *                                  to be set in meters
	 *
	 * @see ManagerProperties#setMaxTakeOffHorizontalError(long)
	 */
	@Override
	public void setMaxTakeOffHorizontalError(long maxTakeOffHorizontalError) {
		this.maxTakeOffHorizontalError = maxTakeOffHorizontalError;
	}
	
	/**
	 * Gets the maximum take-off timing error of this manager properties bean
	 * in seconds.
	 * 
	 * @return the maximum take-off timing error of this manager properties
	 *         bean in seconds
	 *
	 * @see ManagerProperties#getMaxTakeOffTimingError()
	 */
	@Override
	public long getMaxTakeOffTimingError() {
		return this.maxTakeOffTimingError;
	}
	
	/**
	 * Sets the maximum take-off timing error of this manager properties bean
	 * in seconds.
	 * 
	 * @param maxTakeOffTimingError the maximum take-off timing error to be set
	 *                              in seconds
	 *
	 * @see ManagerProperties#setMaxTakeOffTimingError(long)
	 */
	@Override
	public void setMaxTakeOffTimingError(long maxTakeOffTimingError) {
		this.maxTakeOffTimingError = maxTakeOffTimingError;
	}
	
	/**
	 * Gets the maximum horizontal landing error of this manager properties
	 * bean in meters.
	 * 
	 * @return the maximum horizontal landing error of this manager properties
	 *         bean in meters
	 *
	 * @see ManagerProperties#getMaxLandingHorizontalError()
	 */
	@Override
	public long getMaxLandingHorizontalError() {
		return this.maxLandingHorizontalError;
	}
	
	/**
	 * Sets the maximum horizontal landing error of this manager properties
	 * bean in meters.
	 * 
	 * @param maxLandingHorizontalError the maximum horizontal landing error
	 *                                  to be set in meters
	 *
	 * @see ManagerProperties#setMaxLandingHorizontalError(long)
	 */
	@Override
	public void setMaxLandingHorizontalError(long maxLandingHorizontalError) {
		this.maxLandingHorizontalError = maxLandingHorizontalError;
	}
	
	/**
	 * Gets the maximum landing timing error of this manager properties bean in
	 * seconds.
	 * 
	 * @return the maximum landing timing error of this manager properties bean
	 *         in seconds
	 *
	 * @see ManagerProperties#getMaxLandingTimingError()
	 */
	@Override
	public long getMaxLandingTimingError() {
		return this.maxLandingTimingError;
	}
	
	/**
	 * Sets the maximum landing timing error of this manager properties bean in
	 * seconds.
	 * 
	 * @param maxLandingTimingError the maximum take-off timing error to be set
	 *                              in seconds
	 *
	 * @see ManagerProperties#setMaxLandingTimingError(long)
	 */
	@Override
	public void setMaxLandingTimingError(long maxLandingTimingError) {
		this.maxLandingTimingError = maxLandingTimingError;
	}
	
	/**
	 * Clones this manager properties bean.
	 * 
	 * @return a clone of this manager properties bean
	 * 
	 * @see Properties#clone()
	 */
	@Override
	public AbstractManagerProperties clone() {
		AbstractManagerProperties clone = null;
		try {
			clone = (AbstractManagerProperties) super.clone();
		} catch (CloneNotSupportedException e) {
			e.printStackTrace();
		}
		return clone;
	}
	
	/**
	 * Determines whether or not this abstract manager properties bean equals
	 * another abstract manager properties bean based on their aggregated
	 * properties.
	 * 
	 * @param o the other abstract manager properties bean
	 * 
	 * @return true, if the aggregated properties of this abstract manager
	 *         properties bean equal the aggregated properties of the other
	 *         abstract manager properties bean, false otherwise
	 * 
	 * @see Object#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = false;
		
		if (this == o) {
			equals = true;
		} else if ((null != o) && (this.getClass() == o.getClass())) {
			AbstractManagerProperties properties = (AbstractManagerProperties) o;
			equals = (this.costPolicy.equals(properties.costPolicy))
					&& (this.riskPolicy.equals(properties.riskPolicy))
					&& (this.featureHorizon == properties.featureHorizon)
					&& (this.minDeliberation == properties.minDeliberation)
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
	 * Gets the hash code of this abstract manager properties bean based on its
	 * aggregated properties.
	 * 
	 * @return the hash code of this abstract manager properties bean based on
	 *         its aggregated properties
	 * 
	 * @see Object#hashCode()
	 */
	@Override
	public int hashCode() {
		return Objects.hash(
				this.costPolicy,
				this.riskPolicy,
				this.featureHorizon,
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
