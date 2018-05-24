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
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.ListIterator;
import java.util.Random;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreePlanner;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreeWaypoint;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Edge;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
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

	
	protected boolean obstacleFlag = false;

	/**
	 * Constructs a dynamic RRT planner for a specified aircraft and environment using
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
	 * Constructs a dynamic RRT planner for a specified aircraft and environment using
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
	 * Creates an DRRT waypoint at a specified position.
	 * 
	 * @param position the position
	 * 
	 * @return the DRRT waypoint at the specified position
	 * 
	 * @see RRTreePlanner#createWaypoint(Position)
	 */
	@Override
	protected DRRTreeWaypoint createWaypoint(Position position) {
		return new DRRTreeWaypoint(position);
	}

	/**
	 * Gets the child waypoint from the two waypoints in the given edge
	 * 
	 * @param edge the edge to be considered
	 * 
	 * @return the child waypoint of the edge
	 */
	protected DRRTreeWaypoint getEdgeChild(Edge edge) {
		DRRTreeWaypoint waypoint1 = (DRRTreeWaypoint) edge.getPosition1();
		DRRTreeWaypoint waypoint2 = (DRRTreeWaypoint) edge.getPosition2();

		return waypoint2.getParent().equals(waypoint1) ? waypoint2 : waypoint1;
	}

	/**
	 * Updates the list of edges to be in accordance to the list of waypoints, i.e.
	 * removes edges that contain waypoints (at least 1) which are no longer in the
	 * waypoint list.
	 */
	// TODO: move to environment
	protected void updateEdgeList() {
		List<Edge> validEdges = new ArrayList<Edge>();

		for (Edge edge : this.getEdgeList()) {
			DRRTreeWaypoint waypoint1 = (DRRTreeWaypoint) edge.getPosition1();
			if (!this.getWaypointList().contains(waypoint1))
				continue;

			DRRTreeWaypoint waypoint2 = (DRRTreeWaypoint) edge.getPosition2();
			if (!this.getWaypointList().contains(waypoint2))
				continue;

			validEdges.add(edge);
		}

		this.setEdgeList(validEdges);
	}

	/**
	 * Regrows a RRTree by trimming the invalidated nodes and computing a new one.
	 */
	protected void regrowRRT() {
		this.trimRRT();
		this.compute();
	}

	/**
	 * Searches the tree and recursively invalidates all nodes whose parent is
	 * invalidated.
	 */
	protected void trimRRT() {
		List<DRRTreeWaypoint> validWaypoints = new ArrayList<DRRTreeWaypoint>();

		for (RRTreeWaypoint wpt : this.getWaypointList()) {
			DRRTreeWaypoint waypoint = (DRRTreeWaypoint) wpt;
			DRRTreeWaypoint parent = waypoint.getParent();
			// Special case for start (no parent)
			if (parent != null) {
				if (!parent.isValid())
					waypoint.setValidity(false);
			} else {
				waypoint.setValidity(true);
			}
			if (waypoint.isValid())
				validWaypoints.add(waypoint);
		}

		this.setWaypointList(validWaypoints);
		this.updateEdgeList();
	}

	/**
	 * Invalidates all the waypoints affected by the new obstacle
	 * 
	 * @param obstacle the new obstacle in the environment
	 */
	protected void invalidateWaypoints(Obstacle obstacle) {
		for (Edge edge : this.findAffectedEdges(obstacle)) {
			this.getEdgeChild(edge).setValidity(false);
		}
	}

	/**
	 * Finds all the edges in the edge list which are affected by the given obstacle
	 * 
	 * @param obstacle the obstacle to be considered for intersection
	 * 
	 * @return the list of affected edges
	 */
	protected List<Edge> findAffectedEdges(Obstacle obstacle) {
		return this.getEdgeList().stream()
				.filter(e -> obstacle.getExtent(this.getEnvironment().getGlobe()).intersects(e.getLine()))
				.collect(Collectors.toList());
	}

	/**
	 * Checks whether the current path to goal contains nodes which are marked as
	 * invalid due to new obstacles in the environment
	 * 
	 * @return true is the path is invalid, false otherwise
	 */
	protected boolean isPathValid() {
		DRRTreeWaypoint dynamicWaypoint;
		int index;
		for (Waypoint waypoint : this.getPlan()) {
			index = this.getWaypointList().indexOf(waypoint);
			if (index >= 0) {
				dynamicWaypoint = (DRRTreeWaypoint) this.getWaypointList().get(index);
				if (!dynamicWaypoint.isValid()) {
					return false;
				}
			}
		}
		return true;
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

		HashSet<Obstacle> diffObstacles = new HashSet<>();

		this.initialize(origin, destination, etd);
		this.compute();
		Trajectory trajectory = this.createTrajectory();
		this.revisePlan(trajectory);

		// Check for new obstacles until the goal is reached
		while (!this.checkGoal(getStart())) {
			// Move to next waypoint and check for new obstacles
			this.moveToNext();
			System.out.println("\nNew start position... " + this.getStart().getInfo() + "\nWptL="
					+ this.getWaypointList().size() + " EdgL=" + this.getEdgeList().size());

			diffObstacles = this.getNewObstacles();
			if (!diffObstacles.isEmpty()) {
				// Invalidate Waypoints affected by new obstacles
				for (Obstacle obstacle : diffObstacles)
					this.invalidateWaypoints(obstacle);

				// Check if the current path is still valid
				if (!this.isPathValid()) {
					this.regrowRRT();
					System.out.println("[Path Regorwn] Sizes: WptL=" + this.getWaypointList().size() + " EdgL="
							+ this.getEdgeList().size());
				}
			}

			// Print Plan
			System.out.println("Current Path #" + this.getPlan().size());
			for (Waypoint wpt : this.getPlan())
				System.out.println(wpt + " Cost=" + wpt.getCost());

			// Update trajectory reflecting the modified plan
			trajectory = this.createTrajectory();
			this.revisePlan(trajectory);
		}

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
	 * @see RRTreePlanner#plan(Position, Position, List, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
		// TODO: Implement
		return super.plan(origin, destination, waypoints, etd);
	}

	/**
	 * Simulates the movement of the aircraft to the next waypoint in the plan.
	 */
	protected void moveToNext() {
		DRRTreeWaypoint next = (DRRTreeWaypoint) this.getWaypointList()
				.get(getWaypointList().indexOf(getNext(getStart())));
		if (next == null)
			return;

		next.setParent(null);
		next.setValidity(true);
		this.setStart(next);
		this.setWaypointNew(getStart());

		// Remove waypoints not deriving from new start
		List<DRRTreeWaypoint> soundWaypoints = new ArrayList<DRRTreeWaypoint>();
		soundWaypoints.add(getStart());
		for (RRTreeWaypoint wpt : this.getWaypointList()) {
			DRRTreeWaypoint waypoint = (DRRTreeWaypoint) wpt;
			ArrayList<DRRTreeWaypoint> predecessors = waypoint.getPredecessors();
			if (predecessors.contains(next)) {
				soundWaypoints.add(waypoint);
			}
		}

		this.setWaypointList(soundWaypoints);
		this.updateEdgeList();
		this.getPlan().remove(0);
	}

	/**
	 * Gets the waypoint in the plan after the current waypoint.
	 * 
	 * @param current the current waypoint
	 * @return the next waypoint in the plan, null if last waypoint
	 */
	protected Waypoint getNext(Waypoint current) {
		ListIterator<Waypoint> listIterator = this.getPlan().listIterator();
		while (listIterator.hasNext()) {
			if (current.equals(listIterator.next()))
				break;
		}
		if (listIterator.hasNext())
			return listIterator.next();
		else
			return null;
	}

	/**
	 * Gets the new obstacles present in the environment and compares to the
	 * previous.
	 * 
	 * @return the hashset containing the difference between the previous set of
	 *         obstacles and the new
	 */
	@SuppressWarnings("unchecked")
	protected HashSet<Obstacle> getNewObstacles() {
		HashSet<Obstacle> beforeObstacles = (HashSet<Obstacle>) this.getEnvironment().getObstacles().clone();

		// Adds one obstacle with certain probability
		int rand = new Random().nextInt(100 - 1) + 1;
		if (rand <= 30 && !obstacleFlag) {
			System.out.println("\t!!!!!!\t ADDING OBSTACLE\t!!!!!!");
			this.reviseObstacle();
			obstacleFlag = true;
		}

		HashSet<Obstacle> afterObstacles = (HashSet<Obstacle>) this.getEnvironment().getObstacles().clone();

		HashSet<Obstacle> removedObstacles = new HashSet<Obstacle>(beforeObstacles);
		removedObstacles.removeAll(afterObstacles);

		HashSet<Obstacle> addedObstacles = new HashSet<Obstacle>(afterObstacles);
		addedObstacles.removeAll(beforeObstacles);

		HashSet<Obstacle> diffObstacles = new HashSet<Obstacle>();
		diffObstacles.addAll(removedObstacles);
		diffObstacles.addAll(addedObstacles);
		System.out.println("--- DiffObstacles size: " + diffObstacles.size());

		// Sleep block to allow visualization
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		return diffObstacles;
	}
}
