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
package com.cfar.swim.worldwind.ai.continuum.basicprm;

import java.time.ZonedDateTime;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import com.cfar.swim.worldwind.ai.AbstractPlanner;
import com.cfar.swim.worldwind.ai.Planner;
import com.cfar.swim.worldwind.ai.continuum.AbstractSampler;
import com.cfar.swim.worldwind.ai.discrete.astar.ForwardAStarPlanner;
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

	/**
	 * TODO: User input parameters. Consider as tunable parameters...
	 */
	public static final int MAX_NEIGHBORS = 30;

	public static final int MAX_DIST = 30;

	public static final int MAX_SAMPLED_WAYPOINTS = 2000;

	/** the list of already sampled waypoints */
	private List<BasicPRMWaypoint> waypointList = null;

	/** the list of already created edges */
	private List<Edge> edgeList = null;

	/** the last computed plan */
	private final LinkedList<BasicPRMWaypoint> plan = new LinkedList<>();

	/** the roadmap to be created after sampling */
	public PlanningRoadmap roadmap = null;

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
	}

	/**
	 * Gets the list of edges of this basic PRM planner.
	 * 
	 * @return the list of edges of this basic PRM planner
	 */
	public List<Edge> getEdgeList() {
		return edgeList;
	}

	/**
	 * Sets the list of edges of this basic PRM planner.
	 * 
	 * @param edgeList the list of edges of this basic PRM planner
	 */
	public void setEdgeList(List<Edge> edgeList) {
		this.edgeList = edgeList;
	}

	/**
	 * Gets the roadmap of this basic PRM planner.
	 * 
	 * @return the Planning roadmap of this basic PRM planner
	 */
	public PlanningRoadmap getRoadmap() {
		return roadmap;
	}

	/**
	 * Sets the roadmap of this basic PRM planner.
	 * 
	 * @param roadmap the Planning roadmap of this basic PRM planner
	 */
	public void setRoadmap(PlanningRoadmap roadmap) {
		this.roadmap = roadmap;
	}

	/**
	 * Indicates whether or not this Basic PRM planner has a valid roadmap
	 * 
	 * @return true if the planner has a valid roadmap
	 */
	public boolean hasRoadmap() {
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
	public void updateRoadmap(PlanningRoadmap roadmap) {
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
	public Box createBox(PlanningContinuum cont) {
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
	public boolean checkEnvironmentCompatibility() {
		if (this.getContinuumEnvironment().getCorners() == this.getRoadmap().getCorners())
			return true;

		return false;
	}

	/**
	 * Creates a BasicPRMWaypoint at a specified position.
	 * 
	 * @param position the position
	 * 
	 * @return the BasicPRMWaypoint at the specified position
	 */
	protected BasicPRMWaypoint createWaypoint(Position position) {
		return new BasicPRMWaypoint(position);
	}

	/**
	 * Adds a first BasicPRM waypoint to the current plan.
	 * 
	 * @param waypoint the first BasicPRM waypoint to be added to the current plan
	 */
	protected void addFirstWaypoint(BasicPRMWaypoint waypoint) {
		this.plan.addFirst(waypoint);
	}

	/**
	 * Gets the first BasicPRM waypoint of the current plan.
	 * 
	 * @return the first BasicPRM waypoint of the current plan
	 */
	protected BasicPRMWaypoint getFirstWaypoint() {
		return this.plan.getFirst();
	}

	/**
	 * Clears the waypoints of the current plan.
	 */
	protected void clearPlan() {
		this.plan.clear();
	}

	/**
	 * Connects this waypoint to another waypoints already sampled, which are closer
	 * than a MAX_DIST. TODO: NOT DONE
	 * 
	 * @param waypoint the BasicPRM waypoint to be connected
	 */
	public void connectWaypoint(BasicPRMWaypoint waypoint) {
		int numConnectedNeighbor = 0;

		this.waypointList = this.sortNearest(waypoint);

		for (BasicPRMWaypoint neighbor : this.waypointList) {
			// TODO: transform if statements into methods for distance and maximum connected
			// neighbors
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
	 * Sorts the waypoint list by increasing distance from a specific waypoint.
	 * 
	 * @param waypoint the BasicPRM waypoint
	 * 
	 * @return the waypoint list sorted by increasing distance
	 */
	public List<BasicPRMWaypoint> sortNearest(BasicPRMWaypoint waypoint) {

		// sorts the list by increasing distance to waypoint
		Collections.sort(this.waypointList,
				(a, b) -> super.getEnvironment().getDistance(waypoint, a) < super.getEnvironment().getDistance(waypoint,
						b) ? -1
								: super.getEnvironment().getDistance(waypoint, a) == super.getEnvironment()
										.getDistance(waypoint, b) ? 0 : 1);
		return waypointList;
	}

	/**
	 * TODO: NOT DONE
	 * 
	 * @param wp
	 * @param nb
	 */
	public void createEdge(BasicPRMWaypoint wp, BasicPRMWaypoint nb) {
		Edge newedge = new Edge(wp, nb);
		this.edgeList.add(newedge);
	}

	/**
	 * Initializes the planner clearing the waypoint, edge and plan lists.
	 */
	protected void initialize() {

		this.waypointList.clear();
		this.edgeList.clear();
		this.clearPlan();

	}

	/**
	 * TODO: NOT DONE
	 */
	public void construct() {

		int num = 0;

		/* Sample BasicPRMWaypoints, collision check */
		while (num < MAX_SAMPLED_WAYPOINTS) {

			BasicPRMWaypoint waypoint = this.createWaypoint(this.sampleRandomPosition());

			if (!this.checkConflict(waypoint)) {
				this.waypointList.add(waypoint);
				this.connectWaypoint(waypoint);
				num++;
			}
		}

	}

	/**
	 * TODO : NOT DONE
	 * 
	 * @param origin
	 * @param destination
	 */
	public void extendsConstruction(Position origin, Position destination) {

		BasicPRMWaypoint start = this.createWaypoint(origin);

		BasicPRMWaypoint goal = this.createWaypoint(origin);

		if (!this.checkConflict(start)) {
			this.waypointList.add(start);
			this.connectWaypoint(start);
		}

		if (!this.checkConflict(goal)) {
			this.waypointList.add(goal);
			this.connectWaypoint(goal);
		}
	}

	/**
	 * TODO : NOT DONE
	 * 
	 * @param origin
	 * @param destination
	 * @param waypoints
	 */
	public void extendsConstruction(Position origin, Position destination, List<Position> waypoints) {

		BasicPRMWaypoint start = this.createWaypoint(origin);

		BasicPRMWaypoint goal = this.createWaypoint(origin);

		if (!this.checkConflict(start)) {
			this.waypointList.add(start);
			this.connectWaypoint(start);
		}

		if (!this.checkConflict(goal)) {
			this.waypointList.add(goal);
			this.connectWaypoint(goal);
		}

		for (Position pos : waypoints) {

			BasicPRMWaypoint waypoint = this.createWaypoint(pos);

			if (!this.checkConflict(waypoint)) {
				this.waypointList.add(waypoint);
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
			PlanningRoadmap roadmap = new PlanningRoadmap(box, this.getWaypointList(), this.getEdgeList());

			ForwardAStarPlanner aStar = new ForwardAStarPlanner(this.getAircraft(), roadmap);

			Trajectory trajectory = aStar.plan(origin, destination, etd);

			return trajectory;
		} else {
			this.extendsConstruction(origin, destination);

			this.updateRoadmap(roadmap);

			ForwardAStarPlanner aStar = new ForwardAStarPlanner(this.getAircraft(), roadmap);

			Trajectory trajectory = aStar.plan(origin, destination, etd);

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
			PlanningRoadmap roadmap = new PlanningRoadmap(box, this.getWaypointList(), this.getEdgeList());

			ForwardAStarPlanner aStar = new ForwardAStarPlanner(this.getAircraft(), roadmap);

			Trajectory trajectory = aStar.plan(origin, destination, waypoints, etd);

			return trajectory;
		} else {
			this.extendsConstruction(origin, destination, waypoints);

			this.updateRoadmap(roadmap);

			ForwardAStarPlanner aStar = new ForwardAStarPlanner(this.getAircraft(), roadmap);

			Trajectory trajectory = aStar.plan(origin, destination, waypoints, etd);

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
