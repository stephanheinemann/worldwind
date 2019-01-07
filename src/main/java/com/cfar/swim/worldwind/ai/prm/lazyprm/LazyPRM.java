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
import java.util.List;

import com.cfar.swim.worldwind.ai.AbstractPlanner;
import com.cfar.swim.worldwind.ai.Planner;
import com.cfar.swim.worldwind.ai.astar.arastar.ARAStarPlanner;
import com.cfar.swim.worldwind.ai.astar.astar.ForwardAStarPlanner;
import com.cfar.swim.worldwind.ai.prm.basicprm.BasicPRM;
import com.cfar.swim.worldwind.ai.prm.rigidprm.QueryMode;
import com.cfar.swim.worldwind.ai.prm.rigidprm.QueryPlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Edge;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.SamplingEnvironment;
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
			// checks conflicts with nodes => half lazy mode
			// if collision wasn't checked for nodes => full lazy mode
			if (!this.getEnvironment().checkConflict(waypoint, getAircraft())) {
				this.getWaypointList().add(waypoint);
				this.connectWaypoint(waypoint);
				num++;
			}
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
	public boolean correctTrajectoryFull(ForwardAStarPlanner aStar, Trajectory trajectory) {
		if (trajectory == null)
			return false;
		
		int index;
		for(int i = 0; i < trajectory.getLength(); i++) {
			// if i is even then select a waypoint in the natural order starting from start
			// if i is odd then select a waypoint in the inverse order starting from goal
			index = i/2; 
			if((i % 2) == 1) {
				index = trajectory.getLength() - 1 - index;		
			}
			Waypoint waypoint = Iterables.get(trajectory.getWaypoints(), index);
			Waypoint waypointBefore = null;
			if(index>0)
				waypointBefore = Iterables.get(trajectory.getWaypoints(), index-1);
			else
				waypointBefore = Iterables.get(trajectory.getWaypoints(), index);
			
			if (this.getEnvironment().checkConflict(waypoint, getAircraft())) {
				this.getWaypointList().removeIf(s -> s.equals(waypoint));
				this.getEdgeList().removeIf(s -> s.getPosition1().equals(waypoint) || s.getPosition2().equals(waypoint));
				aStar.correctWaypoint(waypoint, waypointBefore);
				return false;
			}	
		}
		
		for (int i = 0; i < trajectory.getLength() - 2; i++) {
			index = i/2; 
			if((i % 2) == 1) {
				index = trajectory.getLength() - 2 - index;		
			}
			Waypoint waypointBefore = Iterables.get(trajectory.getWaypoints(), index);
			Waypoint waypoint = Iterables.get(trajectory.getWaypoints(), index+1);
			if (this.getEnvironment().checkConflict(waypointBefore, waypoint, getAircraft())) {
				this.getEdgeList().remove(new Edge(waypointBefore, waypoint));
				aStar.correctWaypoint(waypoint, waypointBefore);
				return false;
			}
		}
		return true;
	}
	
	
	/**
	 * Corrects a trajectory, checking if any of its waypoints or edges is in
	 * conflict with terrain obstacles.
	 * 
	 * @param trajectory the planned trajectory
	 * 
	 * @return true if this trajectory is feasible, false otherwise
	 */
	public boolean correctTrajectoryHalf(ForwardAStarPlanner aStar, Trajectory trajectory) {
		if (trajectory == null)
			return false;
		
		int index;
		for (int i = 0; i < trajectory.getLength() - 2; i++) {
			index = i/2; 
			if((i % 2) == 1) {
				index = trajectory.getLength() - 2 - index;		
			}
			Waypoint waypointBefore = Iterables.get(trajectory.getWaypoints(), index);
			Waypoint waypoint = Iterables.get(trajectory.getWaypoints(), index+1);
			if (this.getEnvironment().checkConflict(waypointBefore, waypoint, getAircraft())) {
				this.getEdgeList().remove(new Edge(waypointBefore, waypoint));
				aStar.correctWaypoint(waypoint, waypointBefore);
				return false;
			}
		}
		return true;
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
	 * Invokes a query planner to find a trajectory from an origin to a destination
	 * at a specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 * @param planner the planner used to find a path in this populated environment
	 * 
	 * @return the planned trajectory from the origin to the destination with the
	 *         estimated time of departure
	 */
	public Trajectory findPath(Position origin, Position destination, ZonedDateTime etd, QueryPlanner planner) {
		Trajectory trajectory = null;
		switch (planner) {
		case FAS:
			ForwardAStarPlanner aStar = new ForwardAStarPlanner(this.getAircraft(), this.getEnvironment());
			aStar.setCostPolicy(this.getCostPolicy());
			aStar.setRiskPolicy(this.getRiskPolicy());
			this.setRevisionListeners(aStar);
			trajectory = aStar.plan(origin, destination, etd);
			while (!this.correctTrajectoryHalf(aStar, trajectory)) {
				trajectory = aStar.continueComputing();
			}
			break;
		case ARA:
			ARAStarPlanner araStar = new ARAStarPlanner(this.getAircraft(), this.getEnvironment());
			araStar.setCostPolicy(this.getCostPolicy());
			araStar.setRiskPolicy(this.getRiskPolicy());
			araStar.setMinimumQuality(this.getMinimumQuality());
			araStar.setMaximumQuality(this.getMaximumQuality());
			araStar.setQualityImprovement(this.getQualityImprovement());
			this.setRevisionListeners(araStar);
			trajectory = araStar.plan(origin, destination, etd);
			while (!this.correctTrajectoryHalf(araStar, trajectory)) {
				trajectory = araStar.continueComputing();
			}
			break;
		/*
		 * case AD: ADStarPlanner adStar = new ADStarPlanner(this.getAircraft(),
		 * this.getEnvironment()); adStar.setCostPolicy(this.getCostPolicy());
		 * adStar.setRiskPolicy(this.getRiskPolicy());
		 * adStar.addPlanRevisionListener(new PlanRevisionListener() {
		 * 
		 * @Override public void revisePlan(Trajectory trajectory) { for
		 * (PlanRevisionListener listener : planRevisionListeners) {
		 * listener.revisePlan(trajectory); } } public void reviseObstacle() { for
		 * (PlanRevisionListener listener : planRevisionListeners) {
		 * listener.reviseObstacle(); } } }); trajectory = adStar.plan(origin,
		 * destination, etd); break;
		 */
		}
		return trajectory;
	}

	/**
	 * Invokes a query planner to find a trajectory from an origin to a destination
	 * along waypoints at a specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 * @param planner the planner used to find a path in this populated environment
	 * 
	 * @return the planned trajectory from the origin to the destination with the
	 *         estimated time of departure
	 */

	public Trajectory findPath(Position origin, Position destination, ZonedDateTime etd, List<Position> waypoints,
			QueryPlanner planner) {
		Trajectory trajectory = null;

		switch (planner) {
		case FAS:
			ForwardAStarPlanner aStar = new ForwardAStarPlanner(this.getAircraft(), this.getEnvironment());
			aStar.setCostPolicy(this.getCostPolicy());
			aStar.setRiskPolicy(this.getRiskPolicy());
			this.setRevisionListeners(aStar);
			trajectory = aStar.plan(origin, destination, waypoints, etd);
			while (!this.correctTrajectoryHalf(aStar, trajectory)) {
				trajectory = aStar.plan(origin, destination, waypoints, etd);
			}
			break;
		case ARA:
			ARAStarPlanner araStar = new ARAStarPlanner(this.getAircraft(), this.getEnvironment());
			araStar.setCostPolicy(this.getCostPolicy());
			araStar.setRiskPolicy(this.getRiskPolicy());
			araStar.setMinimumQuality(this.getMinimumQuality());
			araStar.setMaximumQuality(this.getMaximumQuality());
			araStar.setQualityImprovement(this.getQualityImprovement());
			this.setRevisionListeners(araStar);
			trajectory = araStar.plan(origin, destination, waypoints, etd);
			while (!this.correctTrajectoryHalf(araStar, trajectory)) {
				trajectory = araStar.plan(origin, destination, waypoints, etd);
			}
			break;
		/*
		 * case AD: ADStarPlanner adStar = new ADStarPlanner(this.getAircraft(),
		 * this.getEnvironment()); adStar.setCostPolicy(this.getCostPolicy());
		 * adStar.setRiskPolicy(this.getRiskPolicy());
		 * adStar.addPlanRevisionListener(new PlanRevisionListener() {
		 * 
		 * @Override public void revisePlan(Trajectory trajectory) { for
		 * (PlanRevisionListener listener : planRevisionListeners) {
		 * listener.revisePlan(trajectory); } }
		 * 
		 * @Override public void reviseObstacle() { for (PlanRevisionListener listener :
		 * planRevisionListeners) { listener.reviseObstacle(); } } }); trajectory =
		 * adStar.plan(origin, destination, waypoints, etd); break;
		 */
		}
		return trajectory;
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
			super.postConstruction();
			trajectory = this.findPath(origin, destination, etd, this.planner);
		} else if (this.getMode() == QueryMode.MULTIPLE) {
			this.extendsConstruction(origin, destination);
			trajectory = this.findPath(origin, destination, etd, this.planner);
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
			super.postConstruction();
			trajectory = this.findPath(origin, destination, etd, waypoints, this.planner);
		} else if (this.getMode() == QueryMode.MULTIPLE) {
			this.extendsConstruction(origin, destination);
			trajectory = this.findPath(origin, destination, etd, waypoints, this.planner);
		}

		this.revisePlan(trajectory);
		return trajectory;
	}
}