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
package com.cfar.swim.worldwind.registries.planners.rrt;

import java.util.Objects;

import javax.validation.constraints.Max;
import javax.validation.constraints.Min;

import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.registries.planners.OnlinePlannerProperties;

/**
 * Realizes the properties bean of an online anytime dynamic RRT planner.
 * 
 * @author Stephan Heinemann
 *
 */
public class OADRRTreeProperties extends ADRRTreeProperties implements OnlinePlannerProperties {
	
	/** the maximum acceptable cross track error of this OADRRT planner properties bean */
	@Min(value = 0, message = "{property.planner.oadrrt.maxCrossTrackError.min}")
	@Max(value = Long.MAX_VALUE, message = "{property.planner.oadrrt.maxCrossTrackError.max}")
	private long maxCrossTrackError = 10;
	
	/** the maximum acceptable timing error of this OADRRT planner properties bean */
	@Min(value = 0, message = "{property.planner.oadrrt.maxTimingError.min}")
	@Max(value = Long.MAX_VALUE, message = "{property.planner.oadrrt.maxTimingError.max}")
	private long maxTimingError = 60;
	
	/**
	 * Constructs a new OADRRT planner properties bean.
	 */
	public OADRRTreeProperties() {
		super();
	}
	
	/**
	 * Constructs a new OADRRT planner properties bean with specified cost
	 * and risk policy property values.
	 * 
	 * @param costPolicy the cost policy of this OADRRT planner properties bean
	 * @param riskPolicy the risk policy of this OADRRT planner properties bean
	 */
	public OADRRTreeProperties(CostPolicy costPolicy, RiskPolicy riskPolicy) {
		super(costPolicy, riskPolicy);
	}
	
	/**
	 * Gets the maximum acceptable cross track error of this OADRRT planner
	 * properties bean in meters.
	 * 
	 * @return the maximum acceptable cross track error of this OADRRT planner
	 *         properties bean in meters
	 * 
	 * @see OnlinePlannerProperties#getMaxCrossTrackError()
	 */
	@Override
	public long getMaxCrossTrackError() {
		return this.maxCrossTrackError;
	}
	
	/**
	 * Sets the maximum acceptable cross track error of this OADRRT planner
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
	 * Gets the maximum acceptable timing error of this OADRRT planner
	 * properties bean in seconds.
	 * 
	 * @return the maximum acceptable timing error of this OADRRT planner
	 *         properties bean in seconds
	 * 
	 * @see OnlinePlannerProperties#getMaxTimingError()
	 */
	@Override
	public long getMaxTimingError() {
		return this.maxTimingError;
	}
	
	/**
	 * Sets the maximum acceptable timing error of this OADRRT planner
	 * properties bean in seconds.
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
	 * Determines whether or not this OADRRT planner properties bean equals
	 * another OADRRT planner properties bean based on their aggregated
	 * properties.
	 * 
	 * @param o the other OADRRT planner properties bean
	 * 
	 * @return true, if the aggregated properties of this OADRRT planner
	 *         properties bean equal the aggregated properties of the other
	 *         OADRRT planner properties bean, false otherwise
	 * 
	 * @see ADRRTreeProperties#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = super.equals(o);
		
		if (equals) {
			OADRRTreeProperties oadrrtp = (OADRRTreeProperties) o;
			equals = (this.maxCrossTrackError == oadrrtp.maxCrossTrackError)
					&& (this.maxTimingError == oadrrtp.maxTimingError);
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this OADRRT planner properties bean based on its
	 * aggregated properties.
	 * 
	 * @return the hash code of this OADRRT planner properties bean based on
	 *         its aggregated properties
	 * 
	 * @see ADRRTreeProperties#hashCode()
	 */
	@Override
	public int hashCode() {
		return Objects.hash(
				super.hashCode(),
				this.maxCrossTrackError,
				this.maxTimingError);
	}
	
}
