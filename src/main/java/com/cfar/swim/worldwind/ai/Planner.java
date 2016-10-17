/**
 * Copyright (c) 2016, Stephan Heinemann (UVic Center for Aerospace Research)
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

import java.time.ZonedDateTime;
import java.util.List;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.RiskPolicy;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.Path;

/**
 * Describes a motion planner for an aircraft in an environment using cost
 * and risk policies.
 * 
 * @author Stephan Heinemann
 *
 */
public interface Planner {

	/**
	 * Gets the aircraft of this planner.
	 * 
	 * @return the aircraft of this planner
	 */
	public Aircraft getAircraft();
	
	/**
	 * Gets the environment of this planner.
	 * 
	 * @return the environment of this planner
	 */
	public Environment getEnvironment();
	
	/**
	 * Gets the cost policy of this planner.
	 * 
	 * @return the cost policy of this planner
	 */
	public CostPolicy getCostPolicy();
	
	/**
	 * Sets the cost policy of this planner.
	 * 
	 * @param costPolicy the cost policy of this planner
	 */
	public void setCostPolicy(CostPolicy costPolicy);
	
	/**
	 * Gets the risk policy of this planner.
	 * 
	 * @return the risk policy of this planner
	 */
	public RiskPolicy getRiskPolicy();
	
	/**
	 * Sets the risk policy of this planner.
	 * 
	 * @param riskPolicy the risk policy of this planner
	 */
	public void setRiskPolicy(RiskPolicy riskPolicy);
	
	/**
	 * Plans a path from an origin to a destination at a specified estimated
	 * time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @return the planned path from the origin to the destination with the
	 *         estimated time of departure
	 */
	public Path plan(Position origin, Position destination, ZonedDateTime etd);
	
	/**
	 * Plans a path from an origin to a destination along waypoints at a
	 * specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param waypoints the waypoints in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @return the planned path from the origin to the destination along the
	 *         waypoints with the estimated time of departure
	 */
	public Path plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd);
	
	/**
	 * Gets a copy of the last computed plan.
	 * 
	 * @return a copy of the last computed plan
	 */
	public List<? extends Position> getPlan();
	
	// TODO: think about applicable heuristics (e.g., Euclidian distance)
	// TODO: minimum ground clearances, altitude restrictions
	// TODO: include capabilities (e.g., velocities, rates) and obtain cost at time
	// TODO: construct grid from globe and positions
	// TODO: think about feasibility and limited deliberation time
	// TODO: think about policies - risk, reward tradeoffs
	// TODO: think about interaction (airspace or obstacle could provide CPDLC interface)
	// TODO: changing altitudes may require CPDLC clearances depending on airspace
	// TODO: computed paths can be associated with symbols (e.g. minimum risk path)

	// TODO: think about required time constraints for waypoints (4D positions)
	// TODO: higher costs might be acceptable to meet timing constraints
}
