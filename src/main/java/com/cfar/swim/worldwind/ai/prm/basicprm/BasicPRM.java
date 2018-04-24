/**
 * Copyright (c) 2018, Henrique Ferreira (UVic Center for Aerospace Research)
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
package com.cfar.swim.worldwind.ai.prm.basicprm;

import java.util.List;

import com.cfar.swim.worldwind.ai.AbstractPlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Edge;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.PlanningContinuum;
import com.cfar.swim.worldwind.planning.PlanningRoadmap;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Line;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Globe;

/**
 * Realizes a basic PRM planner that constructs a Planning Roadmap by sampling
 * points in a continuous environment and plans a trajectory of an aircraft in
 * an environment considering a local cost and risk policy.
 * 
 * @author Henrique Ferreira
 *
 */
public class BasicPRM {

	/** the maximum number of sampling iterations */
	public int maxIter;

	/** the maximum number of neighbors a waypoint can be connected to */
	public int maxNeighbors;

	/** the maximum distance between two neighboring waypoints */
	public double maxDist;
	
	public Environment environment;

	/**
	 * Constructs a basic PRM planner for a specified aircraft and environment,
	 * using default local cost and risk policies. Also, this planner is constructed
	 * with a specified maximum number of iterations (Waypoints), a maximum
	 * number of neighbors (of a single Waypoint) and a maximum distance between two
	 * connected neighbors.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * @param maxIter the maximum number of iterations
	 * @param maxNeighbors the maximum number of neighbors
	 * @param maxDist the maximum distance between two connected neighbors
	 * 
	 * @see AbstractPlanner#AbstractPlanner(Aircraft, Environment)
	 */
	public BasicPRM(Environment environment, int maxIter, int maxNeighbors, double maxDist) {
		this.environment = environment;
		this.maxIter = maxIter;
		this.maxNeighbors = maxNeighbors;
		this.maxDist = maxDist;
	}
	
	/**
	 * Gets the continuum environment of this planner
	 * 
	 * @return the continuum environment
	 */
	public PlanningRoadmap getEnvironment() {
		return (PlanningRoadmap) environment;
	}
	
	/**
	 * Gets the list of already sampled waypoints
	 * 
	 * @return the list of waypoints
	 */
	@SuppressWarnings("unchecked")
	public List<Waypoint> getWaypointList() {
		return (List<Waypoint>) this.getEnvironment().getWaypointList();
	}

	/**
	 * Sets the list of waypoints previously sampled
	 * 
	 * @param waypointList the list of waypoints to set
	 * 
	 */
	@SuppressWarnings("unchecked")
	public void setWaypointList(List<? extends Waypoint> waypointList) {
		this.getEnvironment().setWaypointList((List<Waypoint>) waypointList);
	}
	
	/**
	 * Gets the list of already sampled edges
	 * 
	 * @return the list of edges
	 */
	public List<Edge> getEdgeList() {
		return this.getEnvironment().getEdgeList();
	}
	
	/**
	 * Sets the list of edges previously sampled
	 * 
	 * @param edgetList the list of edges to set
	 * 
	 */
	public void setEdgeList(List<Edge> edgeList) {
		this.getEnvironment().setEdgeList(edgeList);
	}


	/**
	 * Creates a Waypoint at a specified position.
	 * 
	 * @param position the position
	 * 
	 * @return the Waypoint at the specified position
	 */
	protected Waypoint createWaypoint(Position position) {
		Waypoint newWaypoint = new Waypoint(position);
		newWaypoint.setEto(this.getEnvironment().getTime());
		return newWaypoint;
	}

	/**
	 * Connects this waypoint to another waypoints already sampled, which are closer
	 * than a MAX_DIST. The maximum number of neighbors a waypoint can be connected
	 * to is defined by MAX_NEIGHBORS.
	 * 
	 * @param waypoint the BasicPRM waypoint to be connected
	 */
	protected void connectWaypoint(Waypoint waypoint) {
		int numConnectedNeighbor = 0;

		this.getEnvironment().sortNearest(waypoint);

		for (Waypoint neighbor : this.getWaypointList()) {
			if (environment.getDistance(neighbor, waypoint) < this.maxDist
					&& numConnectedNeighbor < this.maxNeighbors) {
				if (!this.getEnvironment().checkConflict(neighbor, waypoint)) {
					numConnectedNeighbor++;
					this.createEdge(waypoint, neighbor);
				}
			}
		}
	}

	/**
	 * Creates an edge between a source waypoint and a target waypoing, and adds it
	 * to the edge list
	 * 
	 * @param source the source Basic PRM waypoint
	 * @param target the target Basic PRM waypoint
	 */
	protected void createEdge(Waypoint source, Waypoint target) {
		Globe globe = this.getEnvironment().getGlobe();
		Line line = new Line(globe.computePointFromPosition(source), globe.computePointFromPosition(target));
		Edge edgeNew = new Edge(source, target, line);
		edgeNew.setCostIntervals(this.getEnvironment().embedIntervalTree(edgeNew.getLine()));
		this.getEdgeList().add(edgeNew);
	}

	/**
	 * Initializes the planner clearing the waypoint, edge and plan lists.
	 */
	protected void initialize() {
		this.getWaypointList().clear();
		this.getEdgeList().clear();
	}

	/**
	 * Creates the roadmap by sampling positions from a continuous environment.
	 * First, checks if the waypoint position has conflicts with terrain. Then the
	 * IntervalTree is embedded and the waypoint is added to the waypoint list.
	 * After that, tries to connect this waypoint to others already sampled.
	 */
	public void construct() {
		int num = 0;

		while (num < this.maxIter) {
			Waypoint waypoint = this.createWaypoint(this.getEnvironment().sampleRandomPosition());

			if (!this.getEnvironment().checkConflict(waypoint)) {
				this.getWaypointList().add(waypoint);
				this.connectWaypoint(waypoint);
				num++;
			}
		}

	}

	/**
	 * Extends the roadmap to incorporate the origin and destination positions.
	 * 
	 * @param origin the origin position in global coordinates
	 * @param destination the destination position in global coordinates
	 */
	protected void extendsConstruction(Position origin, Position destination) {
		Waypoint start = this.createWaypoint(origin);
		Waypoint goal = this.createWaypoint(destination);

		if (!this.getEnvironment().checkConflict(start)) {
			this.getWaypointList().add(start);
			this.connectWaypoint(start);
		}

		if (!this.getEnvironment().checkConflict(goal)) {
			this.getWaypointList().add(goal);
			this.connectWaypoint(goal);
		}
	}

	/**
	 * Extends the roadmap to incorporate the origin, intermediate and destination
	 * positions.
	 * 
	 * @param origin the origin position in global coordinates
	 * @param destination the destination position in global coordinates
	 * @param waypoints the list of intermediate positions in global coordinates
	 */
	protected void extendsConstruction(Position origin, Position destination, List<Position> waypoints) {
		Waypoint start = this.createWaypoint(origin);
		Waypoint goal = this.createWaypoint(destination);

		if (!this.getEnvironment().checkConflict(start)) {
			this.getWaypointList().add(start);
			this.connectWaypoint(start);
		}

		if (!this.getEnvironment().checkConflict(goal)) {
			this.getWaypointList().add(goal);
			this.connectWaypoint(goal);
		}

		for (Position pos : waypoints) {
			Waypoint waypoint = this.createWaypoint(pos);

			if (!this.getEnvironment().checkConflict(waypoint)) {
				this.getWaypointList().add(waypoint);
				this.connectWaypoint(waypoint);
			}
		}
	}

	public boolean supports(Environment environment) {
		boolean supports = false;
		if(null != environment)
			supports =true;

		if (supports) {
			supports = (environment instanceof PlanningContinuum);
		}

		return supports;
	}

}
