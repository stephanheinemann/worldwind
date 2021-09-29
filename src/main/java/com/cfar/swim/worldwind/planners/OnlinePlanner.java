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
package com.cfar.swim.worldwind.planners;

import java.time.Duration;

import com.cfar.swim.worldwind.connections.DatalinkTracker;
import com.cfar.swim.worldwind.planning.Waypoint;

/**
 * Describes an online planner featuring a datalink connection to communicate
 * with a planned aircraft.
 * 
 * @author Stephan Heinemann
 *
 */
public interface OnlinePlanner extends Planner, DatalinkTracker {
	
	/**
	 * Gets the minimum deliberation duration of this online planner.
	 * 
	 * @return the minimum deliberation duration of this online planner
	 */
	public Duration getMinDeliberation();
	
	/**
	 * Sets the minimum deliberation duration of this online planner.
	 * 
	 * @param minDeliberation the minimum deliberation duration to be set
	 */
	public void setMinDeliberation(Duration minDeliberation);
	
	/**
	 * Gets the maximum deliberation duration of this online planner.
	 * 
	 * @return the maximum deliberation duration of this online planner
	 */
	public Duration getMaxDeliberation();
	
	/**
	 * Sets the maximum deliberation duration of this online planner.
	 * 
	 * @param maxDeliberation the maximum deliberation duration to be set
	 */
	public void setMaxDeliberation(Duration maxDeliberation);
	
	/**
	 * Gets the previous waypoint of this online planner.
	 * 
	 * @return the previous waypoint of this online planner if any,
	 *         null otherwise
	 */
	public Waypoint getPreviousWaypoint();
	
	/**
	 * Determines whether or not this online planner has a previous waypoint.
	 * 
	 * @return true if this online planner has a previous waypoint,
	 *         false otherwise
	 */
	public boolean hasPreviousWaypoint();
	
	/**
	 * Gets the next waypoint of this online planner.
	 * 
	 * @return the next waypoint of this online planner if any, null otherwise
	 */
	public Waypoint getNextWaypoint();
	
	/**
	 * Determines whether or not this online planner has a next waypoint.
	 * 
	 * @return true if this online planner has a next waypoint, false otherwise
	 */
	public boolean hasNextWaypoint();
	
	/**
	 * Gets the active part of this online planner.
	 * 
	 * @return the active part of this online planner
	 */
	public int getActivePart();
	
	/**
	 * Gets the mission loader of this online planner.
	 * 
	 * @return the mission loader of this online planner
	 */
	public MissionLoader getMissionLoader();
	
}
