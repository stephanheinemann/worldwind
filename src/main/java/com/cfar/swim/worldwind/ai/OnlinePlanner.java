/**
 * Copyright (c) 2018, Manuel Rosa (UVic Center for Aerospace Research)
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
package com.cfar.swim.worldwind.ai;

import com.cfar.swim.worldwind.connections.Datalink;
import com.cfar.swim.worldwind.planning.Waypoint;

/**
 * Describes a planner with online capabilities, meaning it can interleave the
 * planning and execution steps. The online planner continuously revises the
 * plan considering the current aircraft position until the goal region is
 * reached.
 * 
 * @author Manuel Rosa
 * @author Henrique Ferreira
 *
 */
public interface OnlinePlanner extends Planner {

	/**
	 * Checks if the online capabilities of the planner mode are active or not.
	 * 
	 * @return true if the planner mode is set to online, false otherwise
	 */
	public boolean isOnline();
	
	/**
	 * Gets the datalink connection of this planner.
	 * 
	 * @return the datalink
	 */
	public Datalink getDatalink();

	/**
	 * Sets the datalink connection of this planner.
	 * 
	 * @param datalink the datalink to set
	 */
	public void setDatalink(Datalink datalink);

	/**
	 * Gets the distance threshold to consider a position displacement as worthy of a new plan.
	 * 
	 * @return the distance threshold for each position
	 */
	public double getPositionThreshold();

	/**
	 * Checks if the current position of the aircraft is inside the goal region.
	 * 
	 * @return true if aircraft is inside the goal region, false otherwise
	 */
	public boolean isInsideGoalRegion();
	
	/**
	 * Gets the current position of the aircraft.
	 * 
	 * @return the current position of the aircraft
	 */
	public Waypoint getAircraftTimedPosition();
	
	/**
	 * Sets the current position of the aircraft.
	 * 
	 * @param aircraftPosition the current position of the aircraft
	 */
	public void setAircraftTimedPosition(Waypoint aircraftPosition);
	
	/**
	 * Updates the current position of the aircraft in the planner by reading its
	 * actual position from an external source.
	 */
	public void updateAircraftTimedPosition();

}
