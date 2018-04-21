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
import com.cfar.swim.worldwind.ai.astar.astar.ForwardAStarPlanner;
import com.cfar.swim.worldwind.ai.prm.basicprm.BasicPRM;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.PlanningRoadmap;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes a Lazy PRM planner that constructs a Planning Roadmap by sampling
 * points in a continuous environment, without taking terrain obstacles into
 * account and plans a trajectory of an aircraft in an environment considering a
 * local cost and risk policy.
 * 
 * @author Henrique Ferreira
 *
 */
public class LazyPRM extends BasicPRM {

	/**
	 * Constructs a basic PRM planner for a specified aircraft and environment using
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
	 * Constructs a lazy PRM planner for a specified aircraft and environment using
	 * default local cost and risk policies. Also, this planner is constructed with
	 * a specified maximum number of iterations (Waypoints), a maximum number
	 * of neighbors (of a single Waypoint) and a maximum distance between two
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
	public LazyPRM(Aircraft aircraft, Environment environment, int maxIter, int maxNeighbors, double maxDist) {
		super(aircraft, environment, maxIter, maxNeighbors, maxDist);
	}

	/**
	 * Connects this waypoint to another waypoints already sampled, which are closer
	 * than a MAX_DIST. The maximum number of neighbors a waypoint can be connected
	 * to is defined by MAX_NEIGHBORS.
	 * 
	 * @param waypoint the BasicPRM waypoint to be connected
	 */
	@Override
	protected void connectWaypoint(Waypoint waypoint) {
		int numConnectedNeighbor = 0;

		this.getEnvironment().sortNearest(waypoint);

		for (Waypoint neighbor : this.getWaypointList()) {
			if (super.getEnvironment().getDistance(neighbor, waypoint) < MAX_DIST
					&& numConnectedNeighbor < MAX_NEIGHBORS) {
				numConnectedNeighbor++;
				this.createEdge(waypoint, neighbor);
			}
		}
	}

	/**
	 * Creates the roadmap by sampling positions from a continuous environment. It
	 * doesn't check if the waypoint position has conflicts with terrain. The
	 * IntervalTree is embedded and the waypoint is added to the waypoint list.
	 * After that, tries to connect this waypoint to others already sampled.
	 */
	@Override
	protected void construct() {
		int num = 0;

		while (num < MAX_ITER) {
			Waypoint waypoint = this.createWaypoint(this.sampleRandomPosition());
			waypoint.setCostIntervals(this.getEnvironment().embedIntervalTree(waypoint));
			this.getWaypointList().add(waypoint);
			this.connectWaypoint(waypoint);
			num++;
		}

	}

	/**
	 * Extends the roadmap to incorporate the origin and destination positions.
	 * 
	 * @param origin the origin position in global coordinates
	 * @param destination the destination position in global coordinates
	 */
	@Override
	protected void extendsConstruction(Position origin, Position destination) {
		Waypoint start = this.createWaypoint(origin);
		Waypoint goal = this.createWaypoint(destination);

		// Start and goal may be located at inacessible positions (conflict with terrain
		// is not checked)
		start.setCostIntervals(this.getEnvironment().embedIntervalTree(start));
		this.getWaypointList().add(start);
		this.connectWaypoint(start);

		goal.setCostIntervals(this.getEnvironment().embedIntervalTree(goal));
		this.getWaypointList().add(goal);
		this.connectWaypoint(goal);
	}

	/**
	 * Extends the roadmap to incorporate the origin, intermediate and destination
	 * positions.
	 * 
	 * @param origin the origin position in global coordinates
	 * @param destination the destination position in global coordinates
	 * @param waypoints the list of intermediate positions in global coordinates
	 */
	@Override
	protected void extendsConstruction(Position origin, Position destination, List<Position> waypoints) {
		Waypoint start = this.createWaypoint(origin);
		Waypoint goal = this.createWaypoint(destination);

		// waypoints may be located at inacessible positions (conflict with terrain
		// is not checked)
		start.setCostIntervals(this.getEnvironment().embedIntervalTree(start));
		this.getWaypointList().add(start);
		this.connectWaypoint(start);

		goal.setCostIntervals(this.getEnvironment().embedIntervalTree(goal));
		this.getWaypointList().add(goal);
		this.connectWaypoint(goal);

		for (Position pos : waypoints) {
			Waypoint waypoint = this.createWaypoint(pos);
			waypoint.setCostIntervals(this.getEnvironment().embedIntervalTree(waypoint));
			this.getWaypointList().add(waypoint);
			this.connectWaypoint(waypoint);
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
	protected boolean correctTrajectory(Trajectory trajectory) {
		// TODO: only waypoint conflict checks are done. Edge conflicts are still not
		// done
		if (trajectory == null)
			return false;

		HashSet<Waypoint> conflictWaypoints = new HashSet<Waypoint>();

		for (Waypoint waypoint : trajectory.getWaypoints()) {
			if (this.getEnvironment().checkConflict(waypoint))
				conflictWaypoints.add(waypoint);
		}

		if (!conflictWaypoints.isEmpty()) {
			this.correctLists(conflictWaypoints);
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
	protected void correctLists(HashSet<Waypoint> conflictWaypoints) {
		for (Waypoint waypoint : conflictWaypoints) {
			this.getWaypointList().remove(waypoint);
			this.getEdgeList().removeIf(s -> s.getPosition1().equals(waypoint) || s.getPosition2().equals(waypoint));
		}
		return;
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

		if (!this.hasRoadmap() || !this.checkEnvironmentCompatibility()) {
			this.initialize();
			this.construct();
			this.extendsConstruction(origin, destination);

			Box box = this.createBox(this.getEnvironment());
			PlanningRoadmap roadmap = new PlanningRoadmap(box, this.getWaypointList(), this.getEdgeList(),
					this.getEnvironment().getGlobe());

			while (!this.correctTrajectory(trajectory)) {
				this.updateRoadmap(roadmap);
				ForwardAStarPlanner aStar = new ForwardAStarPlanner(this.getAircraft(), roadmap);
				aStar.setCostPolicy(this.getCostPolicy());
				aStar.setRiskPolicy(this.getRiskPolicy());
				trajectory = aStar.plan(origin, destination, etd);
			}

			this.revisePlan(trajectory);

			return trajectory;
		} else {
			this.extendsConstruction(origin, destination);

			this.updateRoadmap(this.getRoadmap());

			while (!this.correctTrajectory(trajectory)) {
				this.updateRoadmap(this.getRoadmap());
				ForwardAStarPlanner aStar = new ForwardAStarPlanner(this.getAircraft(), this.getRoadmap());
				aStar.setCostPolicy(this.getCostPolicy());
				aStar.setRiskPolicy(this.getRiskPolicy());

				trajectory = aStar.plan(origin, destination, etd);
			}

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
		Trajectory trajectory = null;

		if (!this.hasRoadmap() || !this.checkEnvironmentCompatibility()) {
			this.initialize();
			this.construct();
			this.extendsConstruction(origin, destination, waypoints);

			Box box = this.createBox(this.getEnvironment());
			PlanningRoadmap roadmap = new PlanningRoadmap(box, this.getWaypointList(), this.getEdgeList(),
					this.getEnvironment().getGlobe());

			while (!this.correctTrajectory(trajectory)) {
				this.updateRoadmap(roadmap);
				ForwardAStarPlanner aStar = new ForwardAStarPlanner(this.getAircraft(), roadmap);
				aStar.setCostPolicy(this.getCostPolicy());
				aStar.setRiskPolicy(this.getRiskPolicy());

				trajectory = aStar.plan(origin, destination, waypoints, etd);
			}

			this.revisePlan(trajectory);

			return trajectory;
		} else {
			this.extendsConstruction(origin, destination, waypoints);

			this.updateRoadmap(this.getRoadmap());

			while (!this.correctTrajectory(trajectory)) {
				this.updateRoadmap(this.getRoadmap());
				ForwardAStarPlanner aStar = new ForwardAStarPlanner(this.getAircraft(), this.getRoadmap());
				aStar.setCostPolicy(this.getCostPolicy());
				aStar.setRiskPolicy(this.getRiskPolicy());

				trajectory = aStar.plan(origin, destination, waypoints, etd);
			}

			this.revisePlan(trajectory);

			return trajectory;
		}
	}
}
