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
package com.cfar.swim.worldwind.ai.rrt.drrt;

import java.time.ZonedDateTime;
import java.util.List;

import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreePlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.render.Obstacle;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes a dynamic RRT planner that plans a trajectory of an aircraft in an
 * environment considering a local cost and risk policy. The planner provides an
 * initial plan and then continuously checks if new obstacles are added. When an
 * obstacle affects the current plan, a new one is computed with information
 * 
 * @author Manuel Rosa
 *
 */
public class DRRTreePlanner extends RRTreePlanner {

	/**
	 * Constructs a basic RRT planner for a specified aircraft and environment using
	 * default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see RRTreePlanner#RRTreePlanner(Aircraft, Environment)
	 */
	public DRRTreePlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}

	/**
	 * Constructs a basic RRT planner for a specified aircraft and environment using
	 * default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * @param epsilon the maximum distance to extend a waypoint in the tree
	 * @param bias the bias of the sampling algorithm towards goal
	 * @param maxIter the maximum number of sampling iterations
	 * 
	 * @see RRTreePlanner#RRTreePlanner(Aircraft, Environment, double, int, int)
	 */
	public DRRTreePlanner(Aircraft aircraft, Environment environment, double epsilon, int bias, int maxIter) {
		super(aircraft, environment, epsilon, bias, maxIter);
	}

	/**
	 * Gets the start DRRT waypoint of this DRRT planner.
	 * 
	 * @return the start DRRT waypoint of this DRRT planner
	 * 
	 * @see RRTreePlanner#getStart()
	 */
	@Override
	public DRRTreeWaypoint getStart() {
		return (DRRTreeWaypoint) super.getStart();
	}
	
	/**
	 * Gets the goal DRRT waypoint of this RRT planner.
	 * 
	 * @return the goal DRRT waypoint of this DRRT planner
	 * 
	 * @see RRTreePlanner#getGoal()
	 */
	@Override
	public DRRTreeWaypoint getGoal() {
		return (DRRTreeWaypoint) super.getGoal();
	}
	
	/**
	 * Gets the newest DRRT waypoint added to the tree.
	 * 
	 * @return the waypointNew the newest waypoint added to the tree
	 * 
	 * @see RRTreePlanner#getWaypointNew()
	 */
	@Override
	public DRRTreeWaypoint getWaypointNew() {
		return (DRRTreeWaypoint) super.getWaypointNew();
	}
	
	
	/**
	 * Regrows an RRTree by trimming the invalidated nodes and computing a new one
	 */
	protected void regrowRRT() {
		// TODO: Implement
	}

	/**
	 * Searches the tree and recursively invalidates all nodes whose parent is
	 * invalidated
	 */
	protected void trimRRT() {
		// TODO: Implement
	}

	/**
	 * Invalidates all the waypoints affected by the new obstacle
	 * 
	 * @param obstacle the new obstacle in the environment
	 */
	protected void invalidateWaypoints(Obstacle obstacle) {
		// TODO: Implement
	}
	
	/**
	 * Plans a trajectory from an origin to a destination at a specified estimated
	 * time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @return the planned trajectory from the origin to the destination with the
	 *         estimated time of departure
	 * 
	 * @see RRTreePlanner#plan(Position, Position, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		// TODO: Implement
		return super.plan(origin, destination, etd);
	}
	
	/**
	 * Plans a trajectory from an origin to a destination along waypoints at a
	 * specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param waypoints the waypoints in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @return the planned trajectory from the origin to the destination along the
	 *         waypoints with the estimated time of departure
	 * 
	 * @see RRTreePlanner#plan(Position, Position, List, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
		// TODO: Implement
		return super.plan(origin, destination, waypoints, etd);
	}

}
