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
package com.cfar.swim.worldwind.ai.adstar;

import com.cfar.swim.worldwind.ai.arastar.ARAStarPlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Environment;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes an Anytime Dynamic A* planner (AD*) that plans a trajectory of
 * an aircraft in an environment considering a local cost and risk policy.
 * 
 * @author Stephan Heinemann
 *
 */
public class ADStarPlanner extends ARAStarPlanner {

	/**
	 * Constructs an AD* planner for a specified aircraft and environment
	 * using default local cost and risk policies without an initial inflation
	 * applied to the heuristic.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see ARAStarPlanner#ARAStarPlanner(Aircraft, Environment)
	 */
	public ADStarPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}
	
	/**
	 * Creates an AD* waypoint at a specified position.
	 * 
	 * @param position the position
	 * 
	 * @return the AD* waypoint at the specified position
	 */
	@Override
	protected ADStarWaypoint createWaypoint(Position position) {
		return new ADStarWaypoint(position);
	}
	
	/**
	 * Gets the start AD* waypoint of this AD* planner.
	 * 
	 * @return the start AD* waypoint of this AD* planner
	 * 
	 * @see ARAStarPlanner#getStart()
	 */
	@Override
	protected ADStarWaypoint getStart() {
		return (ADStarWaypoint) super.getStart();
	}
	
	/**
	 * Gets the goal AD* waypoint of this AD* planner.
	 * 
	 * @return the goal AD* waypoint of this AD* planner
	 * 
	 * @see ARAStarPlanner#getGoal()
	 */
	@Override
	protected ADStarWaypoint getGoal() {
		return (ADStarWaypoint) super.getGoal();
	}
	
	/**
	 * Determines whether or not AD* waypoints can be expanded.
	 * 
	 * @return true if AD* waypoints can be expanded, false otherwise
	 * 
	 * @see ARAStarPlanner#canExpand()
	 */
	@Override
	protected boolean canExpand() {
		return super.canExpand() ||
				(this.getGoal().isUnderConsistent());
	}

}
