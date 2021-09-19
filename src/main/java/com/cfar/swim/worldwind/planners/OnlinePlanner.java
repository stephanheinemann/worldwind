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
	
	// TODO: minimum planning time (start planning at suitable future waypoint)
	// use future waypoint with maximum planning time (legs not affected by dynamic obstacles)
	// only upload mission when planning completed according to available deliberation time
	// check affected waypoints: current deliberation time (criticality)
	// only upload mission if current deliberation time is exhausted or maximum quality achieved
	// run timer for available deliberation time and stop planner (setMaximumQuality once exhausted)
	// then upload latest revised mission
	// the autonomic manager can pick the best quality one and upload it via datalink and display
	// it in the source scenario
	/*
	public Duration getMinimumDeliberation();
	public void setMinimumDeliberation(Duration minimumDeliberation);
	public Duration getMaximumDeliberation();
	public void setMaximumDeliberation(Duration maximumDeliberation);
	*/
	
	/**
	 * Gets the next waypoint of this online planner.
	 * 
	 * @return the next waypoint of this online planner
	 */
	public Waypoint getNextWaypoint();
	
	/**
	 * Determines whether or not this online planner has a next waypoint.
	 * 
	 * @return true if this online planner has a next waypoint, false otherwise
	 */
	public boolean hasNextWaypoint();
	
}
