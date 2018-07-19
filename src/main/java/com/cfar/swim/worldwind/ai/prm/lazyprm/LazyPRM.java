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
package com.cfar.swim.worldwind.ai.prm.lazyprm;

import java.time.ZonedDateTime;
import java.util.HashSet;
import java.util.List;

import com.cfar.swim.worldwind.ai.AbstractPlanner;
import com.cfar.swim.worldwind.ai.Planner;
import com.cfar.swim.worldwind.ai.prm.basicprm.BasicPRM;
import com.cfar.swim.worldwind.ai.prm.basicprm.QueryMode;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Edge;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.google.common.collect.Iterables;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes a Lazy PRM planner that constructs a roadmap by sampling points in a
 * continuous environment, without taking terrain obstacles into account and
 * plans a trajectory of an aircraft in an environment considering a local cost
 * and risk policy. The terrain obstacles (untraversable) are only considered
 * after the path is found.
 * 
 * @author Henrique Ferreira
 *
 */
public class LazyPRM extends BasicPRM {

	/**
	 * Constructs a lazy PRM planner for a specified aircraft and environment using
	 * default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see AbstractPlanner#AbstractPlanner(Aircraft, Environment)
	 */
	public LazyPRM(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}

	/**
	 * Connects a given waypoint to another waypoints already sampled, which are
	 * closer than a maximum distance. The maximum number of neighbors a waypoint
	 * can be connected to is another constraint. Checks if the two waypoints are
	 * connectable, but does not check if there is a conflict with terrain
	 * obstacles.
	 * 
	 * @param waypoint the waypoint to be connected
	 */
	@Override
	protected void connectWaypoint(Waypoint waypoint) {
		int numConnectedNeighbor = 0;

		this.getEnvironment().sortNearest(waypoint);

		for (Waypoint neighbor : this.getWaypointList()) {
			if (this.areConnectable(waypoint, neighbor, numConnectedNeighbor)) {
				numConnectedNeighbor++;
				super.createEdge(waypoint, neighbor);
			}
		}
	}

	/**
	 * Creates the roadmap by sampling positions from a continuous environment. It
	 * doesn't check if the waypoint position has conflicts with terrain. Then the
	 * waypoint is added to the waypoint list. After that, tries to connect this
	 * waypoint to others already sampled.
	 */
	@Override
	protected void construct() {
		int num = 0;

		while (num < maxIter) {
			Waypoint waypoint = this.createWaypoint(this.getEnvironment().sampleRandomPosition());
			this.getWaypointList().add(waypoint);
			this.connectWaypoint(waypoint);
			num++;
		}

	}

	/**
	 * Corrects a trajectory, checking if any of its waypoints or edges is in
	 * conflict with terrain obstacles.
	 * 
	 * @param trajectory the planned trajectory
	 * 
	 * @return true if this trajectory is feasible, false otherwise
	 */
	public boolean correctTrajectory(Trajectory trajectory) {
		if (trajectory == null)
			return false;

		HashSet<Waypoint> conflictWaypoints = new HashSet<Waypoint>();
		HashSet<Edge> conflictEdges = new HashSet<Edge>();

		for (Waypoint waypoint : trajectory.getWaypoints()) {
			if (this.getEnvironment().checkConflict(waypoint, getAircraft()))
				conflictWaypoints.add(waypoint);
		}

		if (!conflictWaypoints.isEmpty()) {
			this.correctWaypoints(conflictWaypoints);
			return false;
		}
		Waypoint wpt1 = Iterables.get(trajectory.getWaypoints(), 0);
		Waypoint wpt2;
		for (int i = 0; i < trajectory.getLength() - 1; i++) {
			wpt2 = Iterables.get(trajectory.getWaypoints(), i + 1);
			if (this.getEnvironment().checkConflict(wpt1, wpt2, getAircraft())) {
				conflictEdges.add(new Edge(wpt1, wpt2));
			}
			wpt1 = wpt2;
		}
		if (!conflictEdges.isEmpty()) {
			this.correctEdges(conflictEdges);
			return false;
		}
		return true;
	}
	
	/**
	 * Corrects the waypoint and edge list by removing the waypoints that are in
	 * conflict with terrain obstacles.
	 * 
	 * @param conflictWaypoints the set of waypoints that are in conflict with
	 *            terrain obstacles
	 */
	public void correctWaypoints(HashSet<Waypoint> conflictWaypoints) {
		for (Waypoint waypoint : conflictWaypoints) {
			this.getWaypointList().remove(waypoint);
			this.getEdgeList().removeIf(s -> s.getPosition1().equals(waypoint) || s.getPosition2().equals(waypoint));
		}
	}

	/**
	 * Corrects the edge list by removing the edges that are in conflict with
	 * terrain obstacles.
	 * 
	 * @param conflictEdges the set of edges that are in conflict with terrain
	 *            obstacles
	 */
	public void correctEdges(HashSet<Edge> conflictEdges) {
		for (Edge edge : conflictEdges) {
			this.getEdgeList().remove(edge);
		}
	}
	
	/**
	 * Extends the roadmap to incorporate the origin and destination positions.
	 * 
	 * @param origin the origin position in globe coordinates
	 * @param destination the destination position in globe coordinates
	 */
	@Override
	protected void extendsConstruction(Position origin, Position destination) {
		Waypoint start = this.createWaypoint(origin);
		Waypoint goal = this.createWaypoint(destination);

		// Start and goal may be located at inacessible positions (conflict with terrain
		// is not checked)
		this.getWaypointList().add(start);
		this.connectWaypoint(start);

		this.getWaypointList().add(goal);
		this.connectWaypoint(goal);
	}

	/**
	 * Extends the roadmap to incorporate the origin, intermediate and destination
	 * positions.
	 * 
	 * @param origin the origin position in globe coordinates
	 * @param destination the destination position in globe coordinates
	 * @param waypoints the list of intermediate positions in globe coordinates
	 */
	@Override
	protected void extendsConstruction(Position origin, Position destination, List<Position> waypoints) {
		Waypoint start = this.createWaypoint(origin);
		Waypoint goal = this.createWaypoint(destination);

		// waypoints may be located at inacessible positions (conflict with terrain
		// is not checked)
		this.getWaypointList().add(start);
		this.connectWaypoint(start);

		this.getWaypointList().add(goal);
		this.connectWaypoint(goal);

		for (Position pos : waypoints) {
			Waypoint waypoint = this.createWaypoint(pos);
			this.getWaypointList().add(waypoint);
			this.connectWaypoint(waypoint);
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
	 * @see com.cfar.swim.worldwind.ai.prm.basicprm.BasicPRM#plan(gov.nasa.worldwind.geom.Position,
	 *      gov.nasa.worldwind.geom.Position, java.time.ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		Trajectory trajectory = null;
		if (this.getEnvironment().getEdgeList().isEmpty()) {
			this.setMode(QueryMode.SINGLE);
		}

		if (this.getMode() == QueryMode.SINGLE) {
			this.initialize();
			this.construct();
			this.extendsConstruction(origin, destination);
			while (!this.correctTrajectory(trajectory)) {
				trajectory = this.findPath(origin, destination, etd, this.planner);
			}
		} else if (this.getMode() == QueryMode.MULTIPLE) {
			this.extendsConstruction(origin, destination);
			while (!this.correctTrajectory(trajectory)) {
				trajectory = this.findPath(origin, destination, etd, this.planner);
			}
		}

		this.revisePlan(trajectory);
		return trajectory;
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
		Trajectory trajectory = null;
		if (this.getEnvironment().getEdgeList().isEmpty()) {
			this.setMode(QueryMode.SINGLE);
		}

		if (this.getMode() == QueryMode.SINGLE) {
			this.initialize();
			this.construct();
			this.extendsConstruction(origin, destination);
			while (!this.correctTrajectory(trajectory)) {
				trajectory = this.findPath(origin, destination, etd, waypoints, this.planner);
			}
		} else if (this.getMode() == QueryMode.MULTIPLE) {
			this.extendsConstruction(origin, destination);
			while (!this.correctTrajectory(trajectory)) {
				trajectory = this.findPath(origin, destination, etd, waypoints, this.planner);
			}
		}

		this.revisePlan(trajectory);
		return trajectory;
	}
}