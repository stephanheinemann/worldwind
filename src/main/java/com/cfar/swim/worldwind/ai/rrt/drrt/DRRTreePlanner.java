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
import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.ai.DynamicPlanner;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Extension;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreePlanner;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreeWaypoint;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Strategy;
import com.cfar.swim.worldwind.ai.rrt.hrrt.HRRTreePlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Edge;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.TimeInterval;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.render.Obstacle;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes a dynamic RRT planner that plans a trajectory of an aircraft in an
 * environment considering a local cost and risk policy. The planner provides an
 * initial plan and then continuously checks if new obstacles are added. When an
 * obstacle affects the current plan, a new one is computed with information.
 * Implemented as an extension of the HRRT so that cost regions can be
 * considered.
 * 
 * @author Manuel Rosa
 *
 */
public class DRRTreePlanner extends HRRTreePlanner implements DynamicPlanner {

	// TODO: temporary solution to add obstacle once
	private boolean obstacleFlag = false;

	/** the set with the difference between previous and current obstacle sets */
	private HashSet<Obstacle> diffObstacles;

	/**
	 * Constructs a dynamic RRT planner for a specified aircraft and environment
	 * using default local cost and risk policies.
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
	 * Constructs a dynamic RRT planner for a specified aircraft and environment
	 * using default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * @param epsilon the maximum distance to extend a waypoint in the tree
	 * @param bias the bias of the sampling algorithm towards goal
	 * @param maxIter the maximum number of sampling iterations
	 * @param strategy the expanding strategy for the planner
	 * @param extension the extension technique for the planner
	 * 
	 * @see RRTreePlanner#RRTreePlanner(Aircraft, Environment, double, int, int,
	 *      Strategy, Extension)
	 */
	public DRRTreePlanner(Aircraft aircraft, Environment environment, double epsilon, int bias, int maxIter,
			Strategy strategy, Extension extension, double prob, int neighbors) {
		super(aircraft, environment, epsilon, bias, maxIter, strategy, extension, prob, neighbors);
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
	 * Gets the set of different obstacles containing the difference between the
	 * previous obstacle set and the current one.
	 * 
	 * @return the set of different obstacles containing the difference between the
	 *         previous obstacle set and the current one
	 */
	public HashSet<Obstacle> getDiffObstacles() {
		return diffObstacles;
	}

	/**
	 * Sets the set of different obstacles containing the difference between the
	 * previous obstacle set and the current one.
	 * 
	 * @param diffObstacles the set of different obstacles containing the difference
	 *            between the previous obstacle set and the current one
	 */
	public void setDiffObstacles(HashSet<Obstacle> diffObstacles) {
		this.diffObstacles = diffObstacles;
	}

	/**
	 * Updates the set of different obstacles in the planner by saving the old
	 * obstacles set, calling a function to allow the introduction of new obstacles
	 * and computing the difference between the two.
	 */
	@SuppressWarnings("unchecked")
	public void updateObstacles() {
		HashSet<Obstacle> oldObstacles = (HashSet<Obstacle>) this.getEnvironment().getObstacles().clone();

		// TODO: Temporary solution
		// Adds one obstacle with certain probability
		int rand = new Random().nextInt(100 - 1) + 1;
		if (rand <= 30 && !obstacleFlag) {
			System.out.println("\t!!!!!!\t ADDING OBSTACLE\t!!!!!!");
			this.reviseObstacle();
			obstacleFlag = true;
		}

		// this.reviseObstacle();
		this.setDiffObstacles(getEnvironment().getDiffObstacles(oldObstacles));
	}

	/**
	 * Checks whether or not the changes made to the environment are significant to
	 * take further actions.
	 * 
	 * @return true if the changes are significant, false otherwise
	 */
	public boolean isSignificantChange() {
		int invalid = 0, total = this.getWaypointList().size();

		for (RRTreeWaypoint wpt : this.getWaypointList()) {
			DRRTreeWaypoint waypoint = (DRRTreeWaypoint) wpt;
			if (!waypoint.isValid()) {
				invalid++;
			}
		}

		// If more than 50% of the waypoints became invalid it is significant change
		return ((double) invalid) / total >= 0.5;
	}

	/**
	 * Repairs/regrows a RRTree by trimming the invalidated nodes and computing a
	 * new one.
	 */
	public void repair() {
		this.trimRRT();
		this.compute();
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
	 * Saves the current data in the planner and environment to a dynamic tree.
	 * 
	 * @param tree the tree where to save the current data
	 */
	@SuppressWarnings("unchecked")
	protected void saveToTree(DRRTree tree) {
		tree.setWaypointList((ArrayList<RRTreeWaypoint>) ((ArrayList<RRTreeWaypoint>) this.getWaypointList()).clone());
		tree.setEdgeList((ArrayList<Edge>) ((ArrayList<Edge>) this.getEdgeList()).clone());
		tree.setPlan((LinkedList<Waypoint>) this.getPlan().clone());
	}

	/**
	 * Loads the data from an anytime tree to the current planner.
	 * 
	 * @param tree the tree from where to load the data
	 */
	protected void loadFromTree(DRRTree tree) {
		this.setWaypointList(tree.getWaypointList());
		this.setEdgeList(tree.getEdgeList());
		this.setPlan(tree.getPlan());
	}

	/**
	 * Gets the child waypoint from the two waypoints in the given edge.
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
	 * Gets the child waypoint from the two waypoints in the given edge.
	 * 
	 * @param edge the edge to be considered
	 * 
	 * @return the child waypoint of the edge
	 */
	protected DRRTreeWaypoint getEdgeParent(Edge edge) {
		DRRTreeWaypoint waypoint1 = (DRRTreeWaypoint) edge.getPosition1();
		DRRTreeWaypoint waypoint2 = (DRRTreeWaypoint) edge.getPosition2();

		return waypoint2.getParent().equals(waypoint1) ? waypoint1 : waypoint2;
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
	 * Initializes the planner to plan from an origin to a destination at a
	 * specified estimated time of departure without clearing its expendable lists.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 */
	protected void softInitialize(Position origin, Position destination, ZonedDateTime etd) {
		this.setGoal(this.createWaypoint(destination));
		this.getGoal().setH(0d);

		RRTreeWaypoint start = this.createWaypoint(origin);
		start.setEto(etd);
		start.setG(0d);
		start.setH(this.computeHeuristic(start, this.getGoal()));

		this.setStart(start);
		this.addVertex(start);
		this.setWaypointNew(start);

		this.setCostOpt(start.getF());
		this.setCostMax(start.getF());
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
	 * Invalidates all the waypoints affected by the new obstacle.
	 * 
	 * @param obstacle the new obstacle in the environment
	 */
	protected void invalidateWaypoints(Obstacle obstacle) {
		for (Edge edge : this.findAffectedEdgesSpaceTime(obstacle)) {
			this.getEdgeChild(edge).setValidity(false);
		}
	}

	/**
	 * Finds all the edges in the edge list which are affected by the given obstacle
	 * considering spatial and temporal intersections.
	 * 
	 * @param obstacle the obstacle to be considered for intersection
	 * 
	 * @return the list of affected edges
	 */
	protected List<Edge> findAffectedEdgesSpaceTime(Obstacle obstacle) {
		List<Edge> affectedEdgesSpace = this.getEdgeList().stream()
				.filter(e -> obstacle.getExtent(this.getEnvironment().getGlobe()).intersects(e.getLine()))
				.collect(Collectors.toList());

		List<Edge> affectedEdgesSpaceTime = new ArrayList<Edge>();
		DRRTreeWaypoint parent, child;
		for (Edge edge : affectedEdgesSpace) {
			parent = this.getEdgeParent(edge);
			child = this.getEdgeChild(edge);
			TimeInterval interval = new TimeInterval(parent.getEto(), child.getEto());
			if (obstacle.getCostInterval().intersects(interval))
				affectedEdgesSpaceTime.add(edge);
		}

		return affectedEdgesSpaceTime;
	}

	/**
	 * Checks whether the current path to goal contains nodes which are marked as
	 * invalid due to new obstacles in the environment.
	 * 
	 * @return true is the path is invalid, false otherwise
	 */
	protected boolean isPathValid() {
		return this.isPathValid(getPlan());
	}

	/**
	 * Checks whether a path contains nodes which are marked as invalid due to new
	 * obstacles in the environment.
	 * 
	 * @param path the path to be checked
	 * 
	 * @return true is the path is invalid, false otherwise
	 */
	protected boolean isPathValid(LinkedList<Waypoint> path) {
		DRRTreeWaypoint dynamicWaypoint;
		int index;
		for (Waypoint waypoint : path) {
			index = this.getWaypointList().indexOf(waypoint);
			// If it is still in the tree check validity
			if (index >= 0) {
				dynamicWaypoint = (DRRTreeWaypoint) this.getWaypointList().get(index);
				if (!dynamicWaypoint.isValid())
					return false;
			}
			// If it is not in the tree return invalid
			else
				return false;
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

		this.initialize(origin, destination, etd);
		this.compute();
		Trajectory trajectory = this.createTrajectory();
		this.revisePlan(trajectory);

		// Check for new obstacles until the goal is reached
		// while (!this.checkGoal(getStart())) {
		for (int i = 0; i < 50; i++) {
			// TODO: Move to next waypoint and check for new obstacles

			this.updateObstacles();
			if (!this.getDiffObstacles().isEmpty()) {
				// Invalidate Waypoints affected by new obstacles
				for (Obstacle obstacle : diffObstacles)
					this.invalidateWaypoints(obstacle);

				// Check if the current path is still valid
				if (!this.isPathValid()) {
					this.repair();
					System.out.println("[Path Regorwn] Sizes: WptL=" + this.getWaypointList().size() + " EdgL="
							+ this.getEdgeList().size());
				}
				// Update trajectory reflecting the modified plan
				trajectory = this.createTrajectory();
				this.revisePlan(trajectory);
			}

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

		// Multiple-waypoint-plan variables
		LinkedList<Waypoint> plan = new LinkedList<>();
		Waypoint currentOrigin = new Waypoint(origin), currentDestination;
		ZonedDateTime currentEtd = etd;
		Trajectory trajectory, part;

		// collect intermediate destinations
		ArrayList<Waypoint> destinations = waypoints.stream().map(Waypoint::new)
				.collect(Collectors.toCollection(ArrayList::new));
		destinations.add(new Waypoint(destination));
		int dim = destinations.size();

		// Auxiliary Trees
		DRRTree[] tree = new DRRTree[dim];
		for (int i = 0; i < dim; i++) {
			tree[i] = new DRRTree();
		}

		// Auxiliary variables
		boolean previousValid = true;

		// plan and concatenate partial trajectories
		for (int i = 0; i < dim; i++) {
			currentDestination = destinations.get(i);

			if (!(currentOrigin.equals(currentDestination))) {
				// plan partial trajectory
				this.initialize(currentOrigin, currentDestination, currentEtd);
				this.compute();

				this.saveToTree(tree[i]);
				part = this.createTrajectory();

				// append partial trajectory to plan
				if ((!plan.isEmpty()) && (!part.isEmpty())) {
					plan.pollLast();
				}

				for (Waypoint waypoint : part.getWaypoints()) {
					plan.add(waypoint);
				}
				if (plan.peekLast().equals(currentOrigin)) {
					// if no plan could be found, return an empty trajectory
					trajectory = new Trajectory();
					this.revisePlan(trajectory);
					return trajectory;
				} else {
					currentOrigin = plan.peekLast();
					currentEtd = currentOrigin.getEto();
				}
			}
		}

		// Update trajectory reflecting the first plan
		trajectory = this.createTrajectory(plan);
		this.revisePlan(trajectory);

		// Check for new obstacles until the goal is reached
		// while (!this.checkGoal(getStart())) {
		for (int j = 0; j < 50; j++) {
			// TODO: Move to next waypoint and check for new obstacles

			this.updateObstacles();
			if (!this.getDiffObstacles().isEmpty()) {
				// reset variables for multiple waypoint planning
				currentOrigin = new Waypoint(origin);
				currentEtd = etd;
				plan.clear();
				previousValid = true;

				// Check which path became invalid
				for (int i = 0; i < dim; i++) {
					this.loadFromTree(tree[i]);

					// Invalidate Waypoints affected by new obstacles
					for (Obstacle obstacle : diffObstacles)
						this.invalidateWaypoints(obstacle);

					// Check if the partial path is still valid
					// If this path and the previous components are still valid - do nothing
					if (previousValid && this.isPathValid()) {
					}
					// Path is invalid and should be recomputed
					else {
						// If one of the previous paths was invalid all becomes invalid - clear
						if (!previousValid) {
							this.clearExpendables();
						}
						// If this path is the first invalid - trim
						else {
							previousValid = false;
							this.trimRRT();
						}
						// plan partial trajectory
						currentDestination = destinations.get(i);
						this.softInitialize(currentOrigin, currentDestination, currentEtd);
						this.compute();

						this.saveToTree(tree[i]);
					}
					part = this.createTrajectory(tree[i].getPlan());

					// append partial trajectory to plan
					if ((!plan.isEmpty()) && (!part.isEmpty())) {
						part.pollFirst();
					}

					for (Waypoint waypoint : part.getWaypoints()) {
						plan.add(waypoint);
					}

					if (plan.peekLast().equals(currentOrigin)) {
						// if no plan could be found, return an empty trajectory
						trajectory = new Trajectory();
						this.revisePlan(trajectory);
						return trajectory;
					} else {
						currentOrigin = plan.peekLast();
						currentEtd = currentOrigin.getEto();
					}
				}
				// Update trajectory reflecting the modified plan
				trajectory = this.createTrajectory(plan);
				this.revisePlan(trajectory);
			}
		}

		return trajectory;

	}

}
