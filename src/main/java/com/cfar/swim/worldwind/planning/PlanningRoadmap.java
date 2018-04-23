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
package com.cfar.swim.worldwind.planning;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import com.cfar.swim.worldwind.geom.Box;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Globe;

/**
 * Realizes a planning roadmap that extends a planning continuum, by
 * incorporating waypoint and edge lists. Can be used for motion planning.
 * 
 * @author Henrique Ferreira
 *
 */
public class PlanningRoadmap extends PlanningContinuum implements DiscreteEnvironment {

	/** the list of sampled waypoints of this roadmap */
	private List<Waypoint> waypointList = new ArrayList<>();

	/** the list of edges of this roadmap */
	private List<Edge> edgeList = new ArrayList<>();

	/**
	 * Constructs a planning roadmap based on a box, a waypoint list and a edge list
	 * 
	 * @param box the box used to define this environment
	 * @param waypointList the list of waypoints
	 * @param edgeList the list of edges
	 */
	public PlanningRoadmap(Box box, List<Waypoint> waypointList, List<Edge> edgeList, Globe globe) {
		super(box);
		this.waypointList = waypointList;
		this.edgeList = edgeList;
		this.setGlobe(globe);
	}

	/**
	 * Gets the list of waypoints of this planning roadmap.
	 * 
	 * @return the list of waypoints of this planning roadmap.
	 */
	public List<? extends Waypoint> getWaypointList() {
		return waypointList;
	}

	/**
	 * Sets the list of waypoints of this planning roadmap.
	 * 
	 * @param waypointList the new list of waypoints
	 */
	public void setWaypointList(List<Waypoint> waypointList) {
		this.waypointList = waypointList;
	}

	/**
	 * Gets the list of edges of this planning roadmap.
	 * 
	 * @return the list of edges of this planning roadmap.
	 */
	public List<Edge> getEdgeList() {
		return edgeList;
	}

	/**
	 * Sets the list of edges of this planning roadmap.
	 * 
	 * @param edgeList the new list of edges
	 */
	public void setEdgeList(List<Edge> edgeList) {
		this.edgeList = edgeList;
	}

	/**
	 * TODO
	 * @param position
	 * @param neighbor
	 * @param start
	 * @param end
	 * @param costPolicy
	 * @param riskPolicy
	 * @return
	
	 * @see com.cfar.swim.worldwind.planning.PlanningContinuum#getLegCost(gov.nasa.worldwind.geom.Position, gov.nasa.worldwind.geom.Position, java.time.ZonedDateTime, java.time.ZonedDateTime, com.cfar.swim.worldwind.planning.CostPolicy, com.cfar.swim.worldwind.planning.RiskPolicy)
	 */
	@Override
	public double getLegCost(Position position, Position neighbor, ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {
		// TODO Auto-generated method stub
		return 0;
	}

	/**
	 * TODO
	 * @param neighbor
	 * @param start
	 * @param end
	 * @param costPolicy
	 * @param riskPolicy
	 * @return
	
	 * @see com.cfar.swim.worldwind.planning.PlanningContinuum#getLegCost(com.cfar.swim.worldwind.planning.Environment, java.time.ZonedDateTime, java.time.ZonedDateTime, com.cfar.swim.worldwind.planning.CostPolicy, com.cfar.swim.worldwind.planning.RiskPolicy)
	 */
	@Override
	public double getLegCost(Environment neighbor, ZonedDateTime start, ZonedDateTime end, CostPolicy costPolicy,
			RiskPolicy riskPolicy) {
		// TODO Auto-generated method stub
		return 0;
	}

	/**
	 * Gets the neighbors positions of a position in this roadmap.
	 * 
	 * @param position the position in global coordinates
	 * @return the neighbors of the position in this planning roadmap
	 * 
	 * @see com.cfar.swim.worldwind.planning.DiscreteEnvironment#getNeighbors(gov.nasa.worldwind.geom.Position)
	 */
	@Override
	public Set<Position> getNeighbors(Position position) {
		// TODO: review. Look for a better way to cast a set of Waypoints to a
		// set of positions. Does it make sense to have a method working with positions
		// instead of Waypoints?
		Waypoint waypoint = null;
		waypoint = this.getWaypoint(position);

		Set<Waypoint> sampledNeighbors = this.getNeighbors(waypoint);

		Set<Position> neighbors = new HashSet<Position>();

		for (Waypoint wpt : sampledNeighbors) {
			Position pos = (Position) wpt;
			neighbors.add(pos);
		}

		return neighbors;
	}

	/**
	 * Gets the neighbors waypoints of a specific waypoint in this roadmap.
	 * 
	 * @param waypoint the sampled waypoint
	 * @return the neighboring sampled waypoints in this roadmap
	 */
	public Set<Waypoint> getNeighbors(Waypoint waypoint) {
		Set<Waypoint> neighbors = new HashSet<Waypoint>();

		if (null != this.getGlobe()) {
			for (Edge edge : edgeList) {
				if (waypoint.equals(edge.getPosition1()))
					neighbors.add((Waypoint) edge.getPosition2());
				if (waypoint.equals(edge.getPosition2()))
					neighbors.add((Waypoint) edge.getPosition1());
			}

		} else {
			throw new IllegalStateException("globe is not set");
		}

		return neighbors;
	}

	/**
	 * Checks if a position coincides with a waypoint in this planning roadmap.
	 * 
	 * @param position the position in global coordinates
	 * 
	 * @return true if the position is a waypoint in this planning roadmap, false
	 *         otherwise
	 * @see com.cfar.swim.worldwind.planning.DiscreteEnvironment#isWaypoint(gov.nasa.worldwind.geom.Position)
	 */
	@Override
	public boolean isWaypoint(Position position) {
		for (Waypoint waypoint : waypointList) {
			if (position.equals(waypoint))
				return true;
		}
		return false;
	}

	/**
	 * Gets the waypoint from the waypoint list whose position coincides with a
	 * specific position.
	 * 
	 * @param position the position in global coordinates
	 * 
	 * @return the waypoint from the waypoint list
	 */
	public Waypoint getWaypoint(Position position) {
		for (Waypoint waypoint : waypointList) {
			if (position.equals(waypoint))
				return waypoint;
		}
		return null;
	}

	/**
	 * TODO
	 * @param position
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.DiscreteEnvironment#getAdjacentWaypoints(gov.nasa.worldwind.geom.Position)
	 */
	@Override
	public Set<Position> getAdjacentWaypoints(Position position) {
		// TODO Auto-generated method stub
		return null;
	}

	/**
	 * Indicates whether or not a position is adjacent to a waypoint in this
	 * planning roadmap.
	 * 
	 * @param position the position in globe coordinates
	 * @param waypoint the waypoint in globe coordinates
	 * 
	 * @return true if the position is adjacent to the waypoint in this planning
	 *         roadmap, false otherwise
	 * 
	 * @see com.cfar.swim.worldwind.planning.DiscreteEnvironment#isAdjacentWaypoint(gov.nasa.worldwind.geom.Position,
	 *      gov.nasa.worldwind.geom.Position)
	 */
	@Override
	public boolean isAdjacentWaypoint(Position position, Position waypoint) {
		for (Edge edge : edgeList) {
			if (position.equals(edge.getPosition1()) && waypoint.equals(edge.getPosition2()))
				return true;
			if (position.equals(edge.getPosition2()) && waypoint.equals(edge.getPosition1()))
				return true;
		}
		return false;
	}

	/**
	 * Gets the step cost from an origin to a destination position within this
	 * planning roadmap between a start and an end time given a cost policy and risk
	 * policy.
	 * 
	 * @param origin the origin position in globe coordinates
	 * @param destination the destination position in globe coordinates
	 * @param start the start time
	 * @param end the end time
	 * @param costPolicy the cost policy
	 * @param riskPolicy the risk policy
	 * 
	 * @return the step cost from the origin to the destination position
	 * 
	 * @see com.cfar.swim.worldwind.planning.PlanningContinuum#getStepCost(gov.nasa.worldwind.geom.Position,
	 *      gov.nasa.worldwind.geom.Position, java.time.ZonedDateTime,
	 *      java.time.ZonedDateTime, com.cfar.swim.worldwind.planning.CostPolicy,
	 *      com.cfar.swim.worldwind.planning.RiskPolicy)
	 */
	@Override
	public double getStepCost(Position origin, Position destination, ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {

		return super.getStepCost(origin, destination, start, end, costPolicy, riskPolicy);
	}
}
