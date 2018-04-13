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
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import com.cfar.swim.worldwind.ai.continuum.SampledWaypoint;
import com.cfar.swim.worldwind.geom.Box;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes a planning roadmap that extends a planning continuum, by
 * incorporating waypoint and edge lists. Can be used for motion planning.
 * 
 * @author Henrique Ferreira
 *
 */
public class PlanningRoadmap extends PlanningContinuum implements Environment {

	// TODO: extends SampledWaypoint??? waypointList defined here or in BasicPRM
	// planner?
	/** the list of sampled waypoints of this roadmap */
	private List<? extends Waypoint> waypointList = null;

	/** the list of edges of this roadmap */
	private List<Edge> edgeList = null;

	/**
	 * Constructs a planning roadmap based on a box, a waypoint list and a edge list
	 * 
	 * @param box
	 * @param waypointList the list of waypoints
	 * @param edgeList the list of edges
	 */
	public PlanningRoadmap(Box box, List<? extends Waypoint> waypointList, List<Edge> edgeList) {
		super(box);
		this.waypointList = waypointList;
		this.edgeList = edgeList;
		this.update();
	}

	/**
	 * Gets the list of waypoints of this Planning Roadmap.
	 * 
	 * @return the list of waypoints of this Planning Roadmap
	 */
	public List<? extends Waypoint> getWaypointList() {
		return waypointList;
	}
	
	/**
	 * Sets the list of waypoints of this Planning Roadmap.
	 * 
	 * @param edgeList the list of waypoints of this Planning Roadmap
	 */

	public void setWaypointList(List<? extends Waypoint> waypointList) {
		this.waypointList = waypointList;
	}
	
	/**
	 * Gets the list of edges of this Planning Roadmap.
	 * 
	 * @return the list of edges of this Planning Roadmap
	 */

	public List<Edge> getEdgeList() {
		return edgeList;
	}
	
	/**
	 * Sets the list of edges of this Planning Roadmap.
	 * 
	 * @param edgeList the list of edges of this Planning Roadmap
	 */

	public void setEdgeList(List<Edge> edgeList) {
		this.edgeList = edgeList;
	}
	
	/**
	 * TODO: NOT DONE
	 * 
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

	@Override
	public double getLegCost(Environment neighbor, ZonedDateTime start, ZonedDateTime end, CostPolicy costPolicy,
			RiskPolicy riskPolicy) {
		// TODO Auto-generated method stub
		return 0;
	}

	public Set<SampledWaypoint> getNeighbors(SampledWaypoint waypoint) {
		Set<SampledWaypoint> neighbors = new HashSet<SampledWaypoint>();

		if (null != this.getGlobe()) {
			for (Edge edge : edgeList) {
				if (waypoint.equals(edge.getWpt1()))
					neighbors.add(edge.getWpt2());
				if (waypoint.equals(edge.getWpt2()))
					neighbors.add(edge.getWpt1());
			}

		} else {
			throw new IllegalStateException("globe is not set");
		}

		return neighbors;
	}

}
