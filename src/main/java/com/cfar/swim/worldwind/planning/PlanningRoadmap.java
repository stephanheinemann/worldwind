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
import java.util.List;
import java.util.Optional;
import java.util.Set;

import com.cfar.swim.worldwind.ai.prm.basicprm.BasicPRM;
import com.cfar.swim.worldwind.ai.prm.lazyprm.LazyPRM;
import com.cfar.swim.worldwind.geom.Box;

import gov.nasa.worldwind.geom.Line;
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

	/** the maximum number of sampling iterations */
	public final int MAX_ITER;

	/** the maximum number of neighbors a waypoint can be connected to */
	public final int MAX_NEIGHBORS;

	/** the maximum distance between two neighboring waypoints */
	public final double MAX_DIST;
	
	public RoadmapConstructor roadmapConstructor;
	
	/**
	 * Constructs a planning roadmap based on a box, a waypoint list and a edge list
	 * 
	 * @param box the box used to define this environment
	 * @param waypointList the list of waypoints
	 * @param edgeList the list of edges
	 */
	@SuppressWarnings("unchecked")
	public PlanningRoadmap(Box box, List<? extends Waypoint> waypointList, List<Edge> edgeList, Globe globe) {
		super(box);
		super.setWaypointList((List<Waypoint>)waypointList);
		super.setEdgeList(edgeList);
		this.setGlobe(globe);
		MAX_ITER = 1000;
		MAX_NEIGHBORS = 30;
		MAX_DIST = 200d;
	}
	
	/**
	 * Constructs a planning roadmap based on a box.
	 * 
	 * @param box the box used to define this environment
	 * @param resolution the resolution of this planning continuum
	 */
	public PlanningRoadmap(Box box, double resolution, RoadmapConstructor roadmapConstructor, Globe globe, int maxIter, int maxNeighbors, double maxDist) {
		super(box, resolution);
		this.setGlobe(globe);
		MAX_ITER = maxIter;
		MAX_NEIGHBORS = maxNeighbors;
		MAX_DIST = maxDist;
		this.roadmapConstructor=roadmapConstructor;
		this.constructRoadmap();
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
		return super.getNeighbors(position);
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
		for (Waypoint waypoint : this.getWaypointList()) {
			if (position.equals(waypoint))
				return waypoint;
		}
		return null;
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

		Edge edge = null;
		Optional<Edge> optEdge = this.getEdge(origin, destination);
		if (!optEdge.isPresent()) {
			Line line = new Line(this.getGlobe().computePointFromPosition(origin), this.getGlobe().computePointFromPosition(destination));
			edge = new Edge(origin, destination, line);
			edge.setCostIntervals(this.embedIntervalTree(edge.getLine()));
			this.getEdgeList().add(edge);
			//throw new IllegalStateException("no edge containing both positions");
		}
		else
			edge = optEdge.get();

		double stepCost = 0d, distance, cost;

		distance = this.getNormalizedDistance(origin, destination);

		cost = edge.calculateCost(start, end);

		if (riskPolicy.satisfies(cost - 1)) {
			cost = distance * cost;
		} else {
			cost = Double.POSITIVE_INFINITY;
		}
		// TODO: Review how to apply cost Policy
		stepCost = cost;

		return stepCost;
	}
	
	/**
	 * Updates this planning continuum.
	 */
	protected void update() {
		this.updateActiveCost();
		this.updateAppearance();
		this.updateVisibility();
		this.updateEdges();
	}
	
	protected void updateEdges() {
		for(Edge edge : this.getEdgeList()) {
			edge.setCostIntervals(this.embedIntervalTree(edge.getLine()));
		}
	}
	
	protected void constructRoadmap() {
		if(this.roadmapConstructor==RoadmapConstructor.BASICPRM) {
			BasicPRM basicPRM = new BasicPRM(this, MAX_ITER, MAX_NEIGHBORS, MAX_DIST);
			basicPRM.construct();
		}
		if(this.roadmapConstructor==RoadmapConstructor.LAZYPRM) {
			LazyPRM lazyPRM = new LazyPRM(this, MAX_ITER, MAX_NEIGHBORS, MAX_DIST);
			lazyPRM.construct();
		}
	}
}