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

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

import com.cfar.swim.worldwind.ai.AbstractPlanner;
import com.cfar.swim.worldwind.ai.AbstractSampler;
import com.cfar.swim.worldwind.ai.Planner;
import com.cfar.swim.worldwind.ai.SampledWaypoint;
import com.cfar.swim.worldwind.ai.astar.astar.ForwardAStarPlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.planning.Edge;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.PlanningContinuum;
import com.cfar.swim.worldwind.planning.PlanningRoadmap;
import com.cfar.swim.worldwind.planning.Trajectory;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;

/**
 * Realizes a basic PRM planner that constructs a Planning Roadmap by sampling
 * points in a continuous environment and plans a trajectory of an aircraft in
 * an environment considering a local cost and risk policy.
 * 
 * @author Henrique Ferreira
 *
 */
public class BasicPRM extends AbstractSampler {

	/** the maximum number of sampling iterations */
	public final int MAX_ITER;

	/** the maximum number of neighbors a waypoint can be connected to */
	public final int MAX_NEIGHBORS;

	/** the maximum distance between two neighboring waypoints */
	public final double MAX_DIST;

	/** the list of already created edges */
	private List<Edge> edgeList = new ArrayList<Edge>();

	/** the roadmap used for this planner */
	private PlanningRoadmap roadmap = null;

	/**
	 * Constructs a basic PRM planner for a specified aircraft and environment using
	 * default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see AbstractPlanner#AbstractPlanner(Aircraft, Environment)
	 */
	public BasicPRM(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		MAX_ITER = 1000;
		MAX_NEIGHBORS = 30;
		MAX_DIST = 200d;
	}

	/**
	 * Constructs a basic PRM planner for a specified aircraft and environment,
	 * using default local cost and risk policies. Also, this planner is constructed
	 * with a specified maximum number of iterations (SampledWaypoints), a maximum
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
	public BasicPRM(Aircraft aircraft, Environment environment, int maxIter, int maxNeighbors, double maxDist) {
		super(aircraft, environment);
		MAX_ITER = maxIter;
		MAX_NEIGHBORS = maxNeighbors;
		MAX_DIST = maxDist;
	}
	
	/**
	 * Gets the list of edges of this basic PRM planner.
	 * 
	 * @return the list of edges of this basic PRM planner
	 */
	protected List<Edge> getEdgeList() {
		return edgeList;
	}

	/**
	 * Sets the list of edges of this basic PRM planner.
	 * 
	 * @param edgeList the list of edges of this basic PRM planner
	 */
	protected void setEdgeList(List<Edge> edgeList) {
		this.edgeList = edgeList;
	}

	/**
	 * Gets the roadmap of this basic PRM planner.
	 * 
	 * @return the Planning roadmap of this basic PRM planner
	 */
	protected PlanningRoadmap getRoadmap() {
		return roadmap;
	}

	/**
	 * Sets the roadmap of this basic PRM planner.
	 * 
	 * @param roadmap the Planning roadmap of this basic PRM planner
	 */
	protected void setRoadmap(PlanningRoadmap roadmap) {
		this.roadmap = roadmap;
	}

	/**
	 * Indicates whether or not this Basic PRM planner has a valid roadmap
	 * 
	 * @return true if the planner has a valid roadmap
	 */
	protected boolean hasRoadmap() {
		if (this.getRoadmap() == null)
			return false;
		return true;
	}

	/**
	 * Updates the roadmap of this basic PRM planner by updating the waypoint list
	 * and edge list
	 * 
	 * @param roadmap the Planning roadmap of this basic PRM planner
	 */
	protected void updateRoadmap(PlanningRoadmap roadmap) {
		roadmap.setWaypointList(this.getWaypointList());
		roadmap.setEdgeList(this.getEdgeList());
		return;
	}

	/**
	 * Creates a new box equivalent to the one used to construct the
	 * PlanningContinuum of this Basic PRM planner
	 * 
	 * @param cont the planning Continuum environment of this Basic PRM planner
	 * 
	 * @return the box of the environment of this planner
	 */
	protected Box createBox(PlanningContinuum cont) {
		Vec4 origin = cont.getOrigin();
		Vec4[] unitaxes = cont.getUnitAxes();
		double rLength = cont.getRLength();
		double sLength = cont.getSLength();
		double tLength = cont.getTLength();
		Box box = new Box(origin, unitaxes, rLength, sLength, tLength);
		return box;
	}

	/**
	 * Indicates whether or not the Planning Continuum environment and roadmap are
	 * compatible.
	 * 
	 * @return true if both environments have the same corners
	 */
	protected boolean checkEnvironmentCompatibility() {
		if (this.getContinuumEnvironment().getCorners() == this.getRoadmap().getCorners())
			return true;

		return false;
	}

	/**
	 * Creates a SampledWaypoint at a specified position.
	 * 
	 * @param position the position
	 * 
	 * @return the SampledWaypoint at the specified position
	 */
	protected SampledWaypoint createWaypoint(Position position) {
		SampledWaypoint newWaypoint = new SampledWaypoint(position);
		newWaypoint.setAto(this.getContinuumEnvironment().getTime());
		return newWaypoint;
	}

	/**
	 * Connects this waypoint to another waypoints already sampled, which are closer
	 * than a MAX_DIST. The maximum number of neighbors a waypoint can be connected
	 * to is defined by MAX_NEIGHBORS.
	 * 
	 * @param waypoint the BasicPRM waypoint to be connected
	 */
	protected void connectWaypoint(SampledWaypoint waypoint) {
		int numConnectedNeighbor = 0;

		this.sortNearest(waypoint);

		for (SampledWaypoint neighbor : this.getWaypointList()) {
			if (super.getEnvironment().getDistance(neighbor, waypoint) < MAX_DIST
					&& numConnectedNeighbor < MAX_NEIGHBORS) {
				if (!this.checkConflict(neighbor, waypoint)) {
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
	protected void createEdge(SampledWaypoint source, SampledWaypoint target) {
		Edge newedge = new Edge(source, target);
		this.edgeList.add(newedge);
	}

	/**
	 * Initializes the planner clearing the waypoint, edge and plan lists.
	 */
	protected void initialize() {
		this.getWaypointList().clear();
		this.edgeList.clear();
	}

	/**
	 * Creates the roadmap by sampling positions from a continuous environment.
	 * First, checks if the waypoint position has conflicts with terrain. Then the
	 * IntervalTree is embedded and the waypoint is added to the waypoint list.
	 * After that, tries to connect this waypoint to others already sampled.
	 */
	protected void construct() {
		int num = 0;

		while (num < MAX_ITER) {
			SampledWaypoint waypoint = this.createWaypoint(this.sampleRandomPosition());

			if (!this.checkConflict(waypoint)) {
				waypoint.setCostIntervals(this.getContinuumEnvironment().embedIntervalTree(waypoint));
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
		SampledWaypoint start = this.createWaypoint(origin);
		SampledWaypoint goal = this.createWaypoint(destination);

		if (!this.checkConflict(start)) {
			start.setCostIntervals(this.getContinuumEnvironment().embedIntervalTree(start));
			this.getWaypointList().add(start);
			this.connectWaypoint(start);
		}

		if (!this.checkConflict(goal)) {
			goal.setCostIntervals(this.getContinuumEnvironment().embedIntervalTree(goal));
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
		SampledWaypoint start = this.createWaypoint(origin);
		SampledWaypoint goal = this.createWaypoint(destination);

		if (!this.checkConflict(start)) {
			start.setCostIntervals(this.getContinuumEnvironment().embedIntervalTree(start));
			this.getWaypointList().add(start);
			this.connectWaypoint(start);
		}

		if (!this.checkConflict(goal)) {
			goal.setCostIntervals(this.getContinuumEnvironment().embedIntervalTree(goal));
			this.getWaypointList().add(goal);
			this.connectWaypoint(goal);
		}

		for (Position pos : waypoints) {
			SampledWaypoint waypoint = this.createWaypoint(pos);

			if (!this.checkConflict(waypoint)) {
				waypoint.setCostIntervals(this.getContinuumEnvironment().embedIntervalTree(waypoint));
				this.getWaypointList().add(waypoint);
				this.connectWaypoint(waypoint);
			}
		}
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
	 * @see Planner#plan(Position, Position, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		if (!this.hasRoadmap() || !this.checkEnvironmentCompatibility()) {
			this.initialize();
			this.construct();
			this.extendsConstruction(origin, destination);

			Box box = this.createBox(this.getContinuumEnvironment());
			PlanningRoadmap roadmap = new PlanningRoadmap(box, this.getWaypointList(), this.getEdgeList(),
					this.getContinuumEnvironment().getGlobe());

			ForwardAStarPlanner aStar = new ForwardAStarPlanner(this.getAircraft(), roadmap);
			aStar.setCostPolicy(this.getCostPolicy());
			aStar.setRiskPolicy(this.getRiskPolicy());

			Trajectory trajectory = aStar.plan(origin, destination, etd);

			this.revisePlan(trajectory);

			return trajectory;
		} else {
			this.extendsConstruction(origin, destination);

			this.updateRoadmap(this.getRoadmap());

			ForwardAStarPlanner aStar = new ForwardAStarPlanner(this.getAircraft(), this.getRoadmap());
			aStar.setCostPolicy(this.getCostPolicy());
			aStar.setRiskPolicy(this.getRiskPolicy());

			Trajectory trajectory = aStar.plan(origin, destination, etd);

			this.revisePlan(trajectory);

			return trajectory;
		}
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
	 * @see Planner#plan(Position, Position, List, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
		if (!this.hasRoadmap() || !this.checkEnvironmentCompatibility()) {
			this.initialize();
			this.construct();
			this.extendsConstruction(origin, destination, waypoints);

			Box box = this.createBox(this.getContinuumEnvironment());
			PlanningRoadmap roadmap = new PlanningRoadmap(box, this.getWaypointList(), this.getEdgeList(),
					this.getContinuumEnvironment().getGlobe());

			ForwardAStarPlanner aStar = new ForwardAStarPlanner(this.getAircraft(), roadmap);
			aStar.setCostPolicy(this.getCostPolicy());
			aStar.setRiskPolicy(this.getRiskPolicy());

			Trajectory trajectory = aStar.plan(origin, destination, waypoints, etd);

			this.revisePlan(trajectory);

			return trajectory;
		} else {
			this.extendsConstruction(origin, destination, waypoints);

			this.updateRoadmap(this.getRoadmap());

			ForwardAStarPlanner aStar = new ForwardAStarPlanner(this.getAircraft(), this.getRoadmap());
			aStar.setCostPolicy(this.getCostPolicy());
			aStar.setRiskPolicy(this.getRiskPolicy());

			Trajectory trajectory = aStar.plan(origin, destination, waypoints, etd);

			this.revisePlan(trajectory);

			return trajectory;
		}
	}
	

	/**
	 * Indicates whether or not this Basic PRM planner supports a specified
	 * environment.
	 * 
	 * @param environment the environment
	 * 
	 * @return true if the environment is a planning continuum, false otherwise
	 * 
	 * @see PlanningContinuum
	 */
	@Override
	public boolean supports(Environment environment) {
		boolean supports = super.supports(environment);

		if (supports) {
			supports = (environment instanceof PlanningContinuum);
		}

		return supports;
	}
}
