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
package com.cfar.swim.worldwind.ai.prm.fadprm;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.ai.AbstractPlanner;
import com.cfar.swim.worldwind.ai.Planner;
import com.cfar.swim.worldwind.ai.prm.faprm.FAPRMPlanner;
import com.cfar.swim.worldwind.ai.prm.faprm.FAPRMWaypoint;
import com.cfar.swim.worldwind.ai.prm.rigidprm.QueryMode;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.planning.Edge;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.render.Obstacle;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;

/**
 * Realizes a Flexible Anytime Dynamic Probabilistic Roadmap planner (FADPRM)
 * that plans a trajectory of an aircraft in an environment considering a local
 * cost and risk policy.
 * 
 * @author Henrique Ferreira
 *
 */
public class FADPRMPlanner extends FAPRMPlanner {

	/** the number of times the planner was called */
	protected ArrayList<Integer> counters = new ArrayList<>();

	/** the cost of the best path for each iteration of the algorithm */
	protected ArrayList<ArrayList<Double>> pathCosts = new ArrayList<>();

	/** the number of times the planner was called */
	protected int counter = 0;

	/** the cost of the best path for each iteration of the algorithm */
	protected ArrayList<Double> pathCost = new ArrayList<>();

	/**
	 * Constructs a FADPRM planner for a specified aircraft and environment using
	 * default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see AbstractPlanner#AbstractPlanner(Aircraft, Environment)
	 */
	public FADPRMPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}

	/**
	 * Gets the list of already sampled FADPRM waypoints
	 * 
	 * @return the list of waypoints
	 */
	@SuppressWarnings("unchecked")
	public List<? extends FADPRMWaypoint> getWaypointList() {
		return (List<FADPRMWaypoint>) this.getEnvironment().getWaypointList();
	}

	/**
	 * Polls an FADPRM waypoint from the expandable FADPRM waypoints.
	 * 
	 * @return the polled FADPRM waypoint from the expandable FADPRM waypoints if
	 *         any, null otherwise
	 */
	protected FADPRMWaypoint pollExpandable() {
		return (FADPRMWaypoint) super.pollExpandable();
	}

	/**
	 * Gets the start FAPRM waypoint of this FADPRM planner.
	 * 
	 * @return the start FAPRM waypoint of this FADPRM planner
	 */
	protected FADPRMWaypoint getStart() {
		return (FADPRMWaypoint) super.getStart();
	}

	/**
	 * Gets the goal FAPRM waypoint of this FADPRM planner.
	 * 
	 * @return the goal FAPRM waypoint of this FADPRM planner
	 */
	protected FADPRMWaypoint getGoal() {
		return (FADPRMWaypoint) super.getGoal();
	}

	/**
	 * Gets the counter value of this FADPRM planner.
	 * 
	 * @return the counter the number of times the planner was called
	 */
	public int getCounter() {
		return counter;
	}

	/**
	 * Sets the counter value of this FADPRM planner.
	 * 
	 * @param counter the counter to set
	 */
	public void setCounter(int counter) {
		this.counter = counter;
	}

	/**
	 * Increments the counter value by 1.
	 */
	public void incrementCounter() {
		this.setCounter(this.getCounter() + 1);
	}

	/**
	 * Stores the path cost for a particular search.
	 */
	protected void addPathCost() {
		this.pathCost.add(this.getGoal().getCost());
	}

	/**
	 * Creates a waypoint at a specified position, initializes its search value to
	 * 0, its beta to the current inflation, sets the waypoint desirability and
	 * lambda. Finally, adds it to the waypoint list.
	 * 
	 * @param position the position in global coordinates
	 * @return the new created waypoint
	 */
	@SuppressWarnings("unchecked")
	protected FADPRMWaypoint createWaypoint(Position position) {
		FADPRMWaypoint newWaypoint = new FADPRMWaypoint(position);
		newWaypoint.setEto(this.getEnvironment().getTime());
		newWaypoint.setSearch(0);
		newWaypoint.setBeta(this.getInflation());
		newWaypoint.setDensity(0);
		((List<FADPRMWaypoint>) this.getWaypointList()).add(newWaypoint);
		return newWaypoint;
	}

	/**
	 * Creates a trajectory of the computed plan.
	 * 
	 * @return the trajectory of the computed plan
	 */
	@SuppressWarnings("unchecked")
	protected Trajectory createTrajectory() {
		return new Trajectory((List<FADPRMWaypoint>) this.plan.clone());
	}

	/**
	 * Initializes a number of open backup priority queues.
	 * 
	 * @param size the number of open backup priority queues
	 */
	@Override
	protected void initBackups(int size) {
		this.backups.clear();
		this.pathCosts.clear();
		this.counters.clear();

		for (int waypointIndex = 0; waypointIndex < size; waypointIndex++) {
			this.backups.add(waypointIndex, new ArrayList<FAPRMWaypoint>());
			this.pathCosts.add(waypointIndex, new ArrayList<Double>());
			this.pathCosts.get(waypointIndex).add(0d);
			this.counters.add(waypointIndex, 1);
		}
		this.backupIndex = -1;
	}

	/**
	 * Determines whether or not a backup can be performed.
	 * 
	 * @return true if a backup can be performed, false otherwise
	 */
	@Override
	protected boolean canBackup() {
		return (-1 < this.backupIndex) && (this.backupIndex < this.backups.size());
	}

	/**
	 * Determines whether or not a backup is available.
	 * 
	 * @return true if a backup is available, false otherwise
	 */
	@Override
	protected boolean hasBackup() {
		return this.canBackup() && (!this.backups.get(this.backupIndex).isEmpty());
	}

	/**
	 * Backs up the expandable FAPRM waypoints for improvement.
	 * 
	 * @return true if a backup has been performed, false otherwise
	 */
	@Override
	protected boolean backup() {
		boolean backedup = false;
		if (this.canBackup()) {
			this.backups.get(this.backupIndex).clear();
			// TODO: make clones
			for (FAPRMWaypoint waypoint : this.getWaypointList()) {
				FAPRMWaypoint newWaypoint = waypoint.clone();
				// newWaypoint.setPai(waypoint.getParent());
				this.backups.get(this.backupIndex).add(newWaypoint);
			}

			for (FAPRMWaypoint wpt : this.backups.get(this.backupIndex)) {
				FAPRMWaypoint wptMirror = this.getWaypointList().stream().filter(s -> s.equals(wpt)).findFirst().get();
				FAPRMWaypoint parentMirror = wptMirror.getParent();
				FAPRMWaypoint parent;
				if (parentMirror == null) {
					parent = null;
				} else {
					parent = this.backups.get(this.backupIndex).stream().filter(s -> s.equals(parentMirror)).findFirst()
							.get();
				}
				wpt.setParent(parent);
			}

			this.pathCosts.get(this.backupIndex).addAll(this.pathCost);
			this.counters.set(this.backupIndex, this.counter);
			backedup = true;
			this.getWaypointList().clear();
		}
		return backedup;
	}

	/**
	 * Restores expandable FAPRM waypoints for improvement.
	 * 
	 * @return true if a restoral has been performed, false otherwise
	 */
	@Override
	protected boolean restore() {
		boolean restored = false;
		if (this.hasBackup()) {
			for (FAPRMWaypoint waypoint : this.backups.get(this.backupIndex)) {
				waypoint.setBeta(this.getInflation());
			}
			this.setWaypointList(this.backups.get(this.backupIndex));
			restored = true;
		}
		this.pathCost = this.pathCosts.get(this.backupIndex);
		this.counter = this.counters.get(this.backupIndex);
		return restored;
	}

	/**
	 * Updates a given waypoint, improving its heuristic value and updating its
	 * counter value.
	 * 
	 * @param waypoint the waypoint to be updated
	 */
	protected void updateWaypoint(FADPRMWaypoint waypoint) {
		if (waypoint.getSearch() != 0 && waypoint.getSearch() != this.getCounter()) {
			if (waypoint.getCost() + waypoint.getH() < pathCost.get(waypoint.getSearch())) {
				waypoint.setH(pathCost.get(waypoint.getSearch()) - waypoint.getCost());
			}
		} else if (waypoint.getSearch() == 0) {
			waypoint.setCost(Double.POSITIVE_INFINITY);
		}
		waypoint.setSearch(counter);
	}

	/**
	 * Updates all successors of a given waypoint.
	 * 
	 * @param source the predecessor of all waypoints to be updated
	 */
	@SuppressWarnings("unchecked")
	@Override
	public void updateNeighbors(FAPRMWaypoint source) {
		for (FADPRMWaypoint waypoint : (Set<FADPRMWaypoint>) source.getNeighbors()) {
			this.updateWaypoint(waypoint);
			this.computeCost(source, waypoint);
			if (!this.isExpanded(waypoint)) {
				this.removeExpandable(waypoint);
				this.addExpandable(waypoint);
			}
		}
	}
	
	/**
	 * Computes a path between a start and a goal waypoint.
	 */
	@Override
	protected void findPath() {
		while (this.canExpand()) {
			FADPRMWaypoint source = this.pollExpandable();
			if (source.equals(this.getGoal())) {
				this.connectPlan(source);
				return;
			}
			FADPRMWaypoint newSuccessor;
			if (this.connectToGoal(source)) {
				newSuccessor = this.getGoal();
				this.getGoal().addNeighbor(source);
				source.addNeighbor(this.getGoal());
			} else {
				newSuccessor = this.expand(source);
			}
			this.updateDensity(newSuccessor);
			this.updateNeighbors(source);
			if (this.isExpandable(source)) {
				this.removeExpandable(source);
			}
			this.addExpanded(source);
		}
		return;
	}

	/**
	 * Computes a plan.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @return the planned trajectory from the origin to the destination with the
	 *         estimated time of departure
	 */
	@Override
	protected Trajectory findFeasiblePath() {
		Trajectory trajectory = null;

		this.clearExpandables();
		this.clearExpanded();
		this.addExpandable(this.getStart());
		this.findPath();
		trajectory = this.createTrajectory();
		this.incrementCounter();
		this.addPathCost();

		return trajectory;
	}

	/**
	 * Initializes the planner to plan from an origin to a destination at a
	 * specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 */
	@Override
	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {
		this.backupIndex++;
		if (this.improving) {
			this.restore();
		} else {
			this.setInflation(this.getMinimumQuality());
			this.setCounter(1);
			this.pathCost.add(0d);
		}
		this.clearExpandables();
		this.clearExpanded();
		this.clearWaypoints();
		this.setStart(this.createWaypoint(origin));

		this.setGoal(this.createWaypoint(destination));

		// if MULTIPLE query mode is active, then reinitialize cost for all waypoints
		for (FADPRMWaypoint waypoint : this.getWaypointList()) {
			waypoint.setCost(Double.POSITIVE_INFINITY);
			waypoint.setH(this.getEnvironment().getNormalizedDistance(waypoint, this.getGoal()));
			waypoint.setEto(this.getEnvironment().getTime());
			waypoint.setBeta(this.getInflation());
			waypoint.setSearch(0);
		}

		this.updateWaypoint(this.getGoal());
		this.updateWaypoint(this.getStart());

		this.getStart().setCost(0d);
		this.getStart().setH(this.getEnvironment().getNormalizedDistance(this.getStart(), this.getGoal()));
		this.getStart().setEto(etd);

		this.getGoal().setH(0d);

		this.addExpandable(this.getStart());
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
		// if planner is invoked again with the same environment, and query mode is
		// single, then lists are cleared
		if (this.getMode() == QueryMode.SINGLE) {
			this.getWaypointList().clear();
			this.getEdgeList().clear();
		}

		this.initialize(origin, destination, etd);
		this.setCostBound(Double.POSITIVE_INFINITY);
		Trajectory trajectory = this.findFeasiblePath();
		this.revisePlan(trajectory);
		this.printData(trajectory);
		do {
			// Anytime
			if (!this.isInflated()) {
				this.updateCostBound();
				trajectory = this.improve();
				this.printData(trajectory);
				this.revisePlan(trajectory);

			}

		} while (!isInflated());

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
		// if planner is invoked again with the same environment, lists are cleared
		this.getWaypointList().clear();
		this.getEdgeList().clear();

		this.initBackups(waypoints.size() + 1);
		Trajectory trajectory = this.oneTimePlan(origin, destination, waypoints, etd);
		this.improving = true;
		while (!this.isInflated()) {
			this.inflate();
			this.backupIndex = -1;
			trajectory = this.oneTimePlan(origin, destination, waypoints, etd);
		}
		this.improving = false;

		return trajectory;
	}

	/**
	 * Plans a one time trajectory from an origin to a destination along waypoints
	 * at a specified estimated time of departure, considering the current beta
	 * value.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param waypoints the waypoints in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @return the planned trajectory from the origin to the destination along the
	 *         waypoints with the estimated time of departure
	 */
	public Trajectory oneTimePlan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
		LinkedList<Waypoint> plan = new LinkedList<>();
		Waypoint currentOrigin = new Waypoint(origin);
		ZonedDateTime currentEtd = etd;

		// collect intermediate destinations
		ArrayList<Waypoint> destinations = waypoints.stream().map(Waypoint::new)
				.collect(Collectors.toCollection(ArrayList::new));
		destinations.add(new Waypoint(destination));

		// plan and concatenate partial trajectories
		for (Waypoint currentDestination : destinations) {
			if (!(currentOrigin.equals(currentDestination))) {
				// plan partial trajectory
				this.initialize(currentOrigin, currentDestination, currentEtd);
				Trajectory part = this.findFeasiblePath();
				this.backup();
				// append partial trajectory to plan
				if ((!plan.isEmpty()) && (!part.isEmpty())) {
					plan.pollLast();
				}

				for (Waypoint waypoint : part.getWaypoints()) {
					plan.add(waypoint);
				}
				if (plan.peekLast().equals(currentOrigin)) {
					// if no plan could be found, return an empty trajectory
					Trajectory trajectory = new Trajectory();
					this.revisePlan(trajectory);
					return trajectory;
				} else {
					currentOrigin = plan.peekLast();
					currentEtd = currentOrigin.getEto();
				}
			}
		}
		Trajectory trajectory = new Trajectory(plan);
		this.revisePlan(trajectory);
		return trajectory;
	}
/**
	public void planDynamic() {
		// TODO : implement the algorithm's capability to deal with dynamic obstacles
		// added during the path computation, and while the aircraft is moving

		// HashSet<Obstacle> diffObstacles = this.getNewObstacles();
		// this.updateEdges(diffObstacles);
		// diffObstacles.clear();
		System.out.println("entered playdynamic");
		boolean obstacleFlag = !this.getEnvironment().getObstacles().isEmpty();
		// IF Datalink is connected, realize a dynamic planner, updating start and
		// replanning as long as start waypoint is far from goal
		Trajectory trajectory = this.findFeasiblePath();
		if (this.reviseDatalinkPlan()) {
			while (!this.connectToGoal(this.getStart())) {
				try {
					Thread.sleep(10000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				if (!obstacleFlag) {
					// Adds one obstacle with certain probability MANUEL CODEE
					int rand = new Random().nextInt(100 - 1) + 1;
					if (rand <= 40) {
						System.out.println("\t!!!!!!\t ADDING OBSTACLE\t!!!!!!");
						HashSet<Obstacle> diffObstacles = this.getNewObstacles();
						System.out.println("new obstacles size " + diffObstacles.size());
						HashSet<Edge> affectedEdges = this.getAffectedEdges(diffObstacles);
						// this.correctEdges(affectedEdges);
						obstacleFlag = true;
						this.setInflation(0d);
					} else {
						System.out.println("Obstacle not added");
					}
				}
				// if (!this.updateStart()) {
				// System.out.println("start has been updated");
				this.addExpandable(this.getStart());
				System.out.println("computing");
				trajectory = this.findFeasiblePath();
				System.out.println("computed");
				this.revisePlan(trajectory);
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				this.reviseDatalinkPlan();
				// }
				// else {
				// System.out.println("start hasnt been updated");
				// }
			}
		}
	}*/

	/**
	 * Expands a FAPRM waypoint towards its neighbors in the environment.
	 * 
	 * @param waypoint the FAPRM waypoint to be expanded
	 * 
	 * @return the new sampled waypoint
	 */
	protected FADPRMWaypoint expand(FADPRMWaypoint waypoint) {
		Position newPosition = this.sampleTarget(this.getBias(), waypoint);
		FADPRMWaypoint newWaypoint = this.createWaypoint(newPosition);

		newWaypoint.setH(this.getEnvironment().getNormalizedDistance(newWaypoint, this.getGoal()));

		this.createEdge(waypoint, newWaypoint);
		waypoint.addNeighbor(newWaypoint);
		newWaypoint.addNeighbor(waypoint);
		this.computeCost(waypoint, newWaypoint);

		int numConnectedNeighbor = 1;
		this.getEnvironment().sortNearest(newWaypoint);

		for (FADPRMWaypoint neighbor : this.getWaypointList()) {
			if (neighbor.equals(newWaypoint) || neighbor.equals(waypoint))
				continue;
			if (this.areConnectable(neighbor, newWaypoint, numConnectedNeighbor)) {
				numConnectedNeighbor++;
				this.createEdge(neighbor, newWaypoint);
				neighbor.addNeighbor(newWaypoint);
				newWaypoint.addNeighbor(neighbor);
				this.computeCost(neighbor, newWaypoint);
			}
		}
		return newWaypoint;
	}
	
	/**
	 * Expands a FADPRM waypoint towards its neighbors in the environment.
	 * 
	 * @param waypoint the FADPRM waypoint to be expanded
	 * 
	 * @return the new sampled waypoint
	 */
	/**
	protected FADPRMWaypoint expand(FADPRMWaypoint waypoint) {
		// Position newPosition = this.sampleBiased(this.getBias(), waypoint);
		// FADPRMWaypoint newWaypoint = this.createWaypoint(newPosition);
		// this.updateWaypoint(newWaypoint);
		//
		// newWaypoint.setDistanceToGoal(this.getEnvironment().getNormalizedDistance(newWaypoint,
		// this.getGoal()));
		//
		// this.createEdge(waypoint, newWaypoint);
		// waypoint.addNeighbor(newWaypoint);
		// newWaypoint.addNeighbor(waypoint);
		//
		// int numConnectedNeighbor = 1;
		// this.getEnvironment().sortNearest(newWaypoint);
		//
		// for (FAPRMWaypoint neighbor : this.getWaypointList()) {
		// if (neighbor.equals(newWaypoint) || neighbor.equals(waypoint))
		// continue;
		// if (this.areConnectable(neighbor, newWaypoint, numConnectedNeighbor)) {
		// numConnectedNeighbor++;
		// this.createEdge(neighbor, newWaypoint);
		// neighbor.addNeighbor(newWaypoint);
		// newWaypoint.addNeighbor(neighbor);
		// }
		// }
		// return newWaypoint;

		// new expand as it is written in paper for conference about prm
		Position newPosition = this.sampleTarget(this.getBias(), waypoint);
		FADPRMWaypoint newWaypoint = this.createWaypoint(newPosition);
		this.updateWaypoint(newWaypoint);

		newWaypoint.setH(this.getEnvironment().getNormalizedDistance(newWaypoint, this.getGoal()));

		this.createEdge(waypoint, newWaypoint);
		waypoint.addNeighbor(newWaypoint);
		newWaypoint.addNeighbor(waypoint);

		int numConnectedNeighbor = 1;
		this.getEnvironment().sortNearest(newWaypoint);

		for (FAPRMWaypoint neighbor : this.getWaypointList()) {
			if (neighbor.equals(newWaypoint) || neighbor.equals(waypoint))
				continue;
			if (this.areConnectable(neighbor, newWaypoint, numConnectedNeighbor)) {
				numConnectedNeighbor++;
				this.createEdge(neighbor, newWaypoint);
				neighbor.addNeighbor(newWaypoint);
				newWaypoint.addNeighbor(neighbor);
			}
		}

		for (FAPRMWaypoint neighbor : newWaypoint.getNeighbors()) {
			this.computeCost(neighbor, newWaypoint);
		}
		return newWaypoint;

	}*/

	/**
	 * Computes the estimated cost of a specified target FAPRM waypoint when reached
	 * via a specified source FAPRM waypoint.
	 * 
	 * @param source the source FAPRM waypoint in globe coordinates
	 * @param target the target FAPRM waypoint in globe coordinates
	 */
	protected void computeCost(FAPRMWaypoint source, FAPRMWaypoint target) {

		Path leg = new Path(source, target);
		Capabilities capabilities = this.getAircraft().getCapabilities();
		Globe globe = this.getEnvironment().getGlobe();
		ZonedDateTime end = capabilities.getEstimatedTime(leg, globe, source.getEto());

		double cost = this.getEnvironment().getDesirabilityStepCost(source, target, source.getEto(), end,
				this.getCostPolicy(), this.getRiskPolicy());

		if (source.getCost() + cost < target.getCost()) {
			target.setCost(source.getCost() + cost);
			target.setEto(end);
			target.setParent(source);
		}
		return;

	}

	/**
	 * Gets a set of obstacles that were either removed or added, since the last
	 * time the obstacles were checked.
	 * 
	 * @return the set of different obstacles
	 */
	@SuppressWarnings("unchecked")
	public HashSet<Obstacle> getNewObstacles() {
		HashSet<Obstacle> diffObstacles = new HashSet<Obstacle>();
		HashSet<Obstacle> beforeObstacles = (HashSet<Obstacle>) this.getEnvironment().getObstacles().clone();
		this.reviseObstacle();
		HashSet<Obstacle> afterObstacles = this.getEnvironment().getObstacles();
		for (Obstacle obstacle : beforeObstacles) {
			if (!afterObstacles.contains(obstacle)) {
				diffObstacles.add(obstacle);
			}
		}
		for (Obstacle obstacle : afterObstacles) {
			if (!diffObstacles.contains(obstacle)) {
				diffObstacles.add(obstacle);
			}
		}
		return diffObstacles;
	}

	/**
	 * Gets the affected edges by a set of obstacles and adds the cost interval to
	 * the edges' interval trees.
	 * 
	 * @param diffObstacles the set of different obstacles
	 * 
	 * @return the set of affected edges
	 */
	public HashSet<Edge> getAffectedEdges(HashSet<Obstacle> diffObstacles) {
		HashSet<Edge> affectedEdges = new HashSet<Edge>();
		for (Obstacle obstacle : diffObstacles) {
			for (Edge edge : this.getEnvironment().getEdgeList()) {
				if (obstacle.getExtent(this.getEnvironment().getGlobe()).intersects(edge.getLine())) {
					edge.setCostIntervals(this.getEnvironment().embedIntervalTree(edge.getLine()));
					affectedEdges.add(edge);
				}

			}
		}
		return affectedEdges;
	}

	public void propagate() {
		// TODO: paste here propagate corrections developed for FAPRM. Start with all
		// waypoints inside the obstacles and then propagate
		return;
	}
	
	/**
	 * Gets a set of obstacles that were either removed or added, since the last
	 * time the obstacles were checked.
	 * 
	 * @return the set of different obstacles
	 */
	/**
	@SuppressWarnings("unchecked")
	public HashSet<Obstacle> getNewObstacles() {
		HashSet<Obstacle> diffObstacles = new HashSet<Obstacle>();
		HashSet<Obstacle> beforeObstacles = (HashSet<Obstacle>) this.getEnvironment().getObstacles().clone();
		this.reviseObstacle();
		HashSet<Obstacle> afterObstacles = this.getEnvironment().getObstacles();
		for (Obstacle obstacle : beforeObstacles) {
			if (!afterObstacles.contains(obstacle)) {
				diffObstacles.add(obstacle);
			}
		}
		for (Obstacle obstacle : afterObstacles) {
			if (!diffObstacles.contains(obstacle)) {
				diffObstacles.add(obstacle);
			}
		}
		return diffObstacles;
	}*/

	/**
	 * Gets the affected edges by a set of obstacles and adds the cost interval to
	 * the edges' interval trees.
	 * 
	 * @param diffObstacles the set of different obstacles
	 * 
	 * @return the set of affected edges
	 */
	/**
	public HashSet<Edge> getAffectedEdges(HashSet<Obstacle> diffObstacles) {
		HashSet<Edge> affectedEdges = new HashSet<Edge>();
		for (Obstacle obstacle : diffObstacles) {
			for (Edge edge : this.getEnvironment().getEdgeList()) {
				if (obstacle.getExtent(this.getEnvironment().getGlobe()).intersects(edge.getLine())) {
					edge.setCostIntervals(this.getEnvironment().embedIntervalTree(edge.getLine()));
					affectedEdges.add(edge);
				}

			}
		}
		return affectedEdges;
	}*/
}
