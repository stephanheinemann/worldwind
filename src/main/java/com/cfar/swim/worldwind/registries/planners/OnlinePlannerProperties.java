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
package com.cfar.swim.worldwind.registries.planners;

/**
 * Describes the properties bean of an online planner.
 * 
 * @author Stephan Heinemann
 */
public interface OnlinePlannerProperties extends PlannerProperties {
	
	/**
	 * Gets the maximum acceptable cross track error of this online planner
	 * properties bean in meters.
	 * 
	 * @return the maximum acceptable cross track error of this online planner
	 *         properties bean in meters
	 */
	public long getMaxCrossTrackError();
	
	/**
	 * Sets the maximum acceptable cross track error of this online planner
	 * properties bean in meters.
	 * 
	 * @param maxCrossTrackError the maximum acceptable cross track error to be
	 *                           set in meters
	 */
	public void setMaxCrossTrackError(long maxCrossTrackError);
	
	/**
	 * Gets the maximum acceptable timing error of this online planner
	 * properties bean in seconds.
	 * 
	 * @return the maximum acceptable timing error of this online planner
	 *         properties bean in seconds
	 */
	public long getMaxTimingError();
	
	/**
	 * Sets the maximum acceptable timing error of this online planner
	 * properties bean in seconds.
	 * 
	 * @param maxTimingError the maximum acceptable timing error to be set in
	 *                       seconds
	 */
	public void setMaxTimingError(long maxTimingError);
	
	/**
	 * Gets the maximum horizontal take-off error of this online planner
	 * properties bean in meters.
	 * 
	 * @return the maximum horizontal take-off error of this online planner
	 *         properties bean in meters
	 */
	public long getMaxTakeOffHorizontalError();
	
	/**
	 * Sets the maximum horizontal take-off error of this online planner
	 * properties bean in meters.
	 * 
	 * @param maxTakeOffHorizontalError the maximum horizontal take-off error
	 *                                  to be set in meters
	 */
	public void setMaxTakeOffHorizontalError(long maxTakeOffHorizontalError);
	
	/**
	 * Gets the maximum take-off timing error of this online planner properties
	 * bean in seconds.
	 * 
	 * @return the maximum take-off timing error of this online planner
	 *         properties bean in seconds
	 */
	public long getMaxTakeOffTimingError();
	
	/**
	 * Sets the maximum take-off timing error of this online planner properties
	 * bean in seconds.
	 * 
	 * @param maxTakeOffTimingError the maximum take-off timing error to be set
	 *                              in seconds
	 */
	public void setMaxTakeOffTimingError(long maxTakeOffTimingError);
	
	/**
	 * Gets the maximum horizontal landing error of this online planner
	 * properties bean in meters.
	 * 
	 * @return the maximum horizontal landing error of this online planner
	 *         properties bean in meters
	 */
	public long getMaxLandingHorizontalError();
	
	/**
	 * Sets the maximum horizontal landing error of this online planner
	 * properties bean in meters.
	 * 
	 * @param maxLandingHorizontalError the maximum horizontal landing error
	 *                                  to be set in meters
	 */
	public void setMaxLandingHorizontalError(long maxLandingHorizontalError);
	
	/**
	 * Gets the maximum landing timing error of this online planner properties
	 * bean in seconds.
	 * 
	 * @return the maximum landing timing error of this online planner
	 *         properties bean in seconds
	 */
	public long getMaxLandingTimingError();
	
	/**
	 * Sets the maximum landing timing error of this online planner properties
	 * bean in seconds.
	 * 
	 * @param maxLandingTimingError the maximum take-off timing error to be set
	 *                              in seconds
	 */
	public void setMaxLandingTimingError(long maxLandingTimingError);
	
}
