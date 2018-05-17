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

import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.ai.AbstractPlanner;
import com.cfar.swim.worldwind.ai.AnytimePlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.planning.Edge;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.SamplingEnvironment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.render.Obstacle;
import com.google.common.collect.Iterables;

import gov.nasa.worldwind.geom.Line;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;

/**
 * Realizes a Flexible Anytime Dynamic Probabilistic Roadmap planner (FADPRM)
 * that plans a trajectory of an aircraft in an environment considering a local
 * cost and risk policy.
 * 
 * 
 * @author Henrique Ferreira
 *
 */
public class FADPRMPlanner extends AbstractPlanner implements AnytimePlanner {

	/** the maximum number of neighbors a waypoint can be connected to */
	public final int MAX_NEIGHBORS = 30;

	/** the maximum distance between two neighboring waypoints */
	public final double MAX_DIST = 400;

	/** the priority queue of expandable FADPRM waypoints */
	protected PriorityQueue<FADPRMWaypoint> open = new PriorityQueue<>();

	/** the set of expanded FADPRM waypoints */
	protected Set<FADPRMWaypoint> closed = new HashSet<>();

	/** the start FADPRM waypoint */
	private FADPRMWaypoint start = null;

	/** the goal FADPRM waypoint */
	private FADPRMWaypoint goal = null;

	/** the number of times the planner was called */
	private int counter = 0;

	/** the cost of the best path for each iteration of the algorithm */
	private List<Double> pathCost = new ArrayList<Double>();

	/** the initial beta factor used to sort the open list */
	private double initialBeta;

	/** the final beta factor used to sort the open list */
	private double finalBeta;

	/** the current beta factor used to sort the open list */
	private double beta;

	/** the increase amount to be applied to the current beta */
	private double stepBeta;

	/** the last computed plan */
	private final LinkedList<FADPRMWaypoint> plan = new LinkedList<>();

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

	protected void addPathCost() {
		this.pathCost.add(this.getGoal().getG());
	}

	/**
	 * Gets the minimum quality (initial beta) of this FADPRM planner.
	 * 
	 * @return the minimum quality (initial beta) of this FADPRM planner
	 * 
	 * @see AnytimePlanner#getMinimumQuality()
	 */
	@Override
	public double getMinimumQuality() {
		return this.initialBeta;
	}

	/**
	 * Sets the minimum quality (initial beta) of this FADPRM planner.
	 * 
	 * @param initialBeta the minimum quality (initial beta) of this FADPRM planner
	 * 
	 * @throws IllegalArgumentException if the initial beta is invalid
	 * 
	 * @see AnytimePlanner#setMinimumQuality(double)
	 */
	@Override
	public void setMinimumQuality(double initialBeta) {
		if ((0d <= initialBeta) && (initialBeta <= this.finalBeta)) {
			this.initialBeta = initialBeta;
		} else {
			throw new IllegalArgumentException("initial inflation is invalid");
		}
	}

	/**
	 * Gets the maximum quality (final beta) of this FADPRM planner.
	 * 
	 * @return the maximum quality (final beta) of this FADPRM planner
	 * 
	 * @see AnytimePlanner#getMaximumQuality()
	 */
	@Override
	public double getMaximumQuality() {
		return this.finalBeta;
	}

	/**
	 * Sets the maximum quality (final beta) of this FADPRM planner.
	 * 
	 * @param finalBeta the maximum quality (final beta) of this FADPRM planner
	 * 
	 * @throws IllegalArgumentException if the final inflation is invalid
	 * 
	 * @see AnytimePlanner#setMaximumQuality(double)
	 */
	@Override
	public void setMaximumQuality(double finalBeta) {
		if ((1d >= finalBeta) && (this.initialBeta <= finalBeta)) {
			this.finalBeta = finalBeta;
		} else {
			throw new IllegalArgumentException("final deflation is invalid");
		}
	}

	/**
	 * Gets the quality improvement (stepBeta) of this FADPRM planner.
	 * 
	 * @return the quality improvement (stepBeta) of this FADPRM planner
	 * 
	 * @see AnytimePlanner#getQualityImprovement()
	 */
	@Override
	public double getQualityImprovement() {
		return this.stepBeta;
	}

	/**
	 * Sets the quality improvement (stepBeta) of this FADPRM planner.
	 * 
	 * @param deflationAmount the quality improvement (stepBeta) of this FADPRM
	 *            planner
	 * 
	 * @throws IllegalArgumentException if the deflation amount is invalid
	 * 
	 * @see AnytimePlanner#setQualityImprovement(double)
	 */
	@Override
	public void setQualityImprovement(double stepBeta) {
		if (0d < stepBeta) {
			this.stepBeta = stepBeta;
		} else {
			throw new IllegalArgumentException("deflation amount is invalid");
		}
	}

	/**
	 * Gets the list of already sampled waypoints
	 * 
	 * @return the list of waypoints
	 */
	@SuppressWarnings("unchecked")
	public List<FADPRMWaypoint> getWaypointList() {
		return (List<FADPRMWaypoint>) this.getEnvironment().getWaypointList();
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
	 * Increases the current beta by stepBeta, or otherwise to the final beta.
	 */
	protected void inflate() {
		if (this.finalBeta > (this.beta + this.stepBeta)) {
			this.beta += this.stepBeta;
		} else {
			this.beta = this.finalBeta;
		}
	}

	/**
	 * Determines whether or not the final beta has been reached.
	 * 
	 * @return true if the final beta has been reached, false otherwise
	 */
	protected boolean isInflated() {
		return (this.beta == this.finalBeta);
	}

	/**
	 * Gets the current beta of this FADPRM planner.
	 * 
	 * @return the current beta of this FADPRM planner
	 */
	protected double getInflation() {
		return this.beta;
	}

	/**
	 * Sets the current beta of this FADPRM planner.
	 * 
	 * @param beta the current beta of this FADPRM planner
	 */
	protected void setInflation(double beta) {
		this.beta = beta;
	}

	/**
	 * Gets the sampling environment of this planner
	 * 
	 * @return the sampling environment
	 */
	public SamplingEnvironment getEnvironment() {
		return (SamplingEnvironment) super.getEnvironment();
	}

	/**
	 * Gets the start FADPRM waypoint of this FADPRM planner.
	 * 
	 * @return the start FADPRM waypoint of this FADPRM planner
	 */
	protected FADPRMWaypoint getStart() {
		return this.start;
	}

	/**
	 * Sets the start FADPRM waypoint of this FADPRM planner.
	 * 
	 * @param start the start waypoint of this FADPRM planner
	 */
	protected void setStart(FADPRMWaypoint start) {
		this.start = start;
	}

	/**
	 * Gets the goal FADPRM waypoint of this FADPRM planner.
	 * 
	 * @return the goal FADPRM waypoint of this FADPRM planner
	 */
	protected FADPRMWaypoint getGoal() {
		return this.goal;
	}

	/**
	 * Sets the goal FADPRM waypoint of this FADPRM planner.
	 * 
	 * @param goal the goal FADPRM waypoint of this FADPRM planner
	 */
	protected void setGoal(FADPRMWaypoint goal) {
		this.goal = goal;
	}

	/**
	 * Adds an FADPRM waypoint to the expandable FADPRM waypoints.
	 * 
	 * @param waypoint the FADPRM waypoint
	 * 
	 * @return true if the FADPRM waypoint has been added to the expandable FADPRM
	 *         waypoints, false otherwise
	 */
	protected boolean addExpandable(FADPRMWaypoint waypoint) {
		return this.open.add(waypoint);
	}

	/**
	 * Removes an FADPRM waypoint from the expandable FADPRM waypoints.
	 * 
	 * @param waypoint the FADPRM waypoint
	 * 
	 * @return true if the FADPRM waypoint has been removed from the expandable
	 *         FADPRM waypoints, false otherwise
	 */
	protected boolean removeExpandable(FADPRMWaypoint waypoint) {
		return this.open.remove(waypoint);
	}

	/**
	 * Clears the expandable FADPRM waypoints.
	 */
	protected void clearExpandables() {
		this.open.clear();
	}

	/**
	 * Determines whether or not an FADPRM waypoint is expandable.
	 * 
	 * @param waypoint the FADPRM waypoint
	 * 
	 * @return true if the FADPRM waypoint is expandable, false otherwise
	 */
	protected boolean isExpandable(FADPRMWaypoint waypoint) {
		return this.open.contains(waypoint);
	}

	/**
	 * Determines whether or not FADPRM waypoints can be expanded.
	 * 
	 * @return true if FADPRM waypoints can be expanded, false otherwise
	 */
	protected boolean canExpand() {
		return !(this.open.isEmpty());
	}

	/**
	 * Polls an FADPRM waypoint from the expandable FADPRM waypoints.
	 * 
	 * @return the polled FADPRM waypoint from the expandable FADPRM waypoints if
	 *         any, null otherwise
	 */
	protected FADPRMWaypoint pollExpandable() {
		return this.open.poll();
	}

	/**
	 * Adds an FADPRM waypoint to the expanded FADPRM waypoints.
	 * 
	 * @param waypoint the FADPRM waypoint
	 * 
	 * @return true if the FADPRM waypoint has been added to the expanded FADPRM
	 *         waypoints, false otherwise
	 */
	protected boolean addExpanded(FADPRMWaypoint waypoint) {
		return this.closed.add(waypoint);
	}

	/**
	 * Clears the expanded FADPRM waypoints.
	 */
	protected void clearExpanded() {
		this.closed.clear();
	}

	/**
	 * Gets the first FADPRM waypoint of the current plan.
	 * 
	 * @return the first FADPRM waypoint of the current plan
	 */
	protected FADPRMWaypoint getFirstWaypoint() {
		return this.plan.getFirst();
	}

	/**
	 * Adds a first FADPRM waypoint to the current plan.
	 * 
	 * @param waypoint the first FADPRM waypoint to be added to the current plan
	 */
	protected void addFirstWaypoint(FADPRMWaypoint waypoint) {
		this.plan.addFirst(waypoint);
	}

	/**
	 * Clears the waypoints of the current plan.
	 */
	protected void clearWaypoints() {
		this.plan.clear();
	}

	/**
	 * Connects a plan of specified FADPRM waypoints.
	 * 
	 * @param waypoint the last FADPRM waypoint of a computed plan
	 */
	protected void connectPlan(FADPRMWaypoint waypoint) {
		this.clearWaypoints();
		waypoint.setDtg(0);
		while ((null != waypoint)) {
			this.addFirstWaypoint(waypoint.clone());
			waypoint = waypoint.getParent();
			if (null != waypoint) {
				// TODO: this is not good enough and environment airdata intervals are required
				waypoint.setTtg(Duration.between(waypoint.getEto(), this.getFirstWaypoint().getEto()));
				waypoint.setDtg(this.getEnvironment().getDistance(waypoint, this.getFirstWaypoint()));
			}
		}
	}

	public Position samplePosition(Position position) {
		Vec4 point = this.getEnvironment().getGlobe().computePointFromPosition(position);
		List<Vec4> corners = new ArrayList<Vec4>();

		double halfDistance = this.MAX_DIST / Math.sqrt(3);

		corners.add(point.add3(-halfDistance, +halfDistance, -halfDistance));
		corners.add(point.add3(+halfDistance, +halfDistance, -halfDistance));
		corners.add(point.add3(+halfDistance, -halfDistance, -halfDistance));
		corners.add(point.add3(+halfDistance, -halfDistance, +halfDistance));
		corners.add(point.add3(-halfDistance, -halfDistance, +halfDistance));
		corners.add(point.add3(-halfDistance, +halfDistance, +halfDistance));

		SamplingEnvironment env = new SamplingEnvironment(
				new Box(gov.nasa.worldwind.geom.Box.computeBoundingBox(corners)));
		env.setGlobe(this.getEnvironment().getGlobe());
		Position pos = env.sampleRandomPosition();
		while (this.getEnvironment().checkConflict(pos)
				|| !this.getAircraft().getCapabilities().isFeasible(position, pos, this.getEnvironment().getGlobe())) {
			pos = env.sampleRandomPosition();
		}
		return pos;
	}

	/**
	 * Indicates whether or not two waypoints are connectable.
	 * 
	 * @param target the target A* waypoint in globe coordinates
	 * @param waypoint the waypoint in globe coordinates
	 * @param neighbor the neighbor in globe coordinates
	 * @param num the number of connected neighbors to the waypoint
	 * 
	 * @return true if the two waypoints are connectable, false otherwise
	 */
	protected boolean areConnectable(Waypoint waypoint, Waypoint neighbor, int num) {
		boolean connectable = false;

		Capabilities capabilities = this.getAircraft().getCapabilities();

		if (super.getEnvironment().getDistance(neighbor, waypoint) < MAX_DIST && num < MAX_NEIGHBORS) {
			if (capabilities.isFeasible(waypoint, neighbor, this.getEnvironment().getGlobe())
					|| capabilities.isFeasible(neighbor, waypoint, this.getEnvironment().getGlobe())) {
				connectable = true;
			}
		}

		return connectable;
	}

	/**
	 * Expands an FADPRM waypoint towards its neighbors in the environment.
	 * 
	 * @param waypoint the FADPRM waypoint to be expanded
	 * 
	 * @return the new sampled waypoint
	 */
	protected FADPRMWaypoint expand(FADPRMWaypoint waypoint) {
		int numConnectedNeighbor = 0;

		Position newPosition = this.samplePosition(waypoint);
		while (!this.getEnvironment().contains(newPosition))
			newPosition = this.samplePosition(waypoint);

		FADPRMWaypoint newWaypoint = this.createWaypoint(newPosition);
		this.getEnvironment().sortNearest(newWaypoint);

		this.createEdge(waypoint, newWaypoint);
		this.computeCost(waypoint, newWaypoint);

		for (FADPRMWaypoint neighbor : this.getWaypointList()) {
			if (this.areConnectable(neighbor, newWaypoint, numConnectedNeighbor) && !neighbor.equals(this.getGoal())) {
				numConnectedNeighbor++;
				this.createEdge(neighbor, newWaypoint);
				this.computeCost(neighbor, newWaypoint);
			}
		}
		return newWaypoint;
	}

	/**
	 * Creates a waypoint at a specified position, initializes its search value to
	 * 0, and adds it to the waypoint list.
	 * 
	 * @param position the position in global coordinates
	 * @return the new created waypoint
	 */
	protected FADPRMWaypoint createWaypoint(Position position) {
		FADPRMWaypoint newWaypoint = new FADPRMWaypoint(position);
		newWaypoint.setEto(this.getEnvironment().getTime());
		newWaypoint.setSearch(0);
		this.getWaypointList().add(newWaypoint);
		return newWaypoint;
	}

	/**
	 * Creates an edge between a source waypoint and a target waypoing, and adds it
	 * to the edge list
	 * 
	 * @param source the source Basic PRM waypoint
	 * @param target the target Basic PRM waypoint
	 */
	protected void createEdge(FADPRMWaypoint source, FADPRMWaypoint target) {
		Vec4 sourcePoint = this.getEnvironment().getGlobe().computePointFromPosition(source);
		Vec4 targetPoint = this.getEnvironment().getGlobe().computePointFromPosition(target);

		Edge edgeNew = new Edge(source, target, new Line(sourcePoint, targetPoint));
		edgeNew.setCostIntervals(this.getEnvironment().embedIntervalTree(edgeNew.getLine()));
		this.getEdgeList().add(edgeNew);
	}

	/**
	 * Computes the estimated cost of a specified target FADPRM waypoint when
	 * reached via a specified source FADPRM waypoint.
	 * 
	 * @param source the source FADPRM waypoint in globe coordinates
	 * @param target the target FADPRM waypoint in globe coordinates
	 */
	protected double computeCost(FADPRMWaypoint source, FADPRMWaypoint target) {

		Path leg = new Path(source, target);
		Capabilities capabilities = this.getAircraft().getCapabilities();
		Globe globe = this.getEnvironment().getGlobe();
		// TODO: catch IllegalArgumentException (incapable) and exit
		ZonedDateTime end = capabilities.getEstimatedTime(leg, globe, source.getEto());

		double distance = this.getEnvironment().getNormalizedDistance(source, target);
		// target.setCost(source.getCost() + distance);
		double pathDD = 1 / (this.getEnvironment().getStepCost(source, target, source.getEto(), end,
				this.getCostPolicy(), this.getRiskPolicy()) / distance);
		if ((((source.getPathDD() + pathDD) / 2) / (1 + target.getLambda() * (source.getCost() + distance)) > target
				.getG())) {
			target.setPathDD((source.getPathDD() + pathDD) / 2);
			target.setCost(source.getCost() + distance);
			target.setG(target.getPathDD() / (1 + target.getLambda() * target.getCost()));
			target.setEto(end);
			target.setParent(source);
		}
		return ((source.getPathDD() + pathDD) / 2) / (1 + target.getLambda() * (source.getCost() + distance))
				- source.getG();

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

	public void consistencyProcedure(Set<Edge> affectedEdges) {
		this.clearExpandables();
		for (Edge edge : affectedEdges) {
			FADPRMWaypoint source = (FADPRMWaypoint) edge.getPosition1();
			FADPRMWaypoint target = (FADPRMWaypoint) edge.getPosition2();
			if (source.getParent().equals(target)) {
				source = (FADPRMWaypoint) edge.getPosition2();
				target = (FADPRMWaypoint) edge.getPosition1();
			}
			if (source.equals(this.getGoal()))
				continue;

			this.updateWaypoint(source);
			this.updateWaypoint(target);
			// if (source.getH() < this.computeCost(source, target) - target.getH()) {
			// source.setH(this.computeCost(source, target) - target.getH());
			// this.removeExpandable(source);
			// this.addExpandable(source);
			// }
			if (target.getH() < this.computeCost(source, target) + source.getH()) {
				target.setH(this.computeCost(source, target) + source.getH());
				this.removeExpandable(target);
				this.addExpandable(target);
			}
		}
		while (this.canExpand()) {
			FADPRMWaypoint source = this.pollExpandable();
			for(FADPRMWaypoint successor : source.getSuccessors()) {
				if (!successor.equals(this.getGoal())) {
					this.updateWaypoint(successor);
					if (successor.getH() < this.computeCost(successor, source) + source.getH()) {
						successor.setH(this.computeCost(successor, source) + source.getH());
						this.removeExpandable(successor);
						this.addExpandable(successor);
					}
				}
			}
		}
		// while (this.canExpand()) {
		// FADPRMWaypoint source = this.pollExpandable();
		// FADPRMWaypoint parent = source.getParent();
		// if (!parent.equals(this.getGoal())) {
		// this.updateWaypoint(parent);
		// if (parent.getH() < this.computeCost(parent, source) - source.getH()) {
		// parent.setH(this.computeCost(parent, source) - source.getH());
		// this.removeExpandable(parent);
		// this.addExpandable(parent);
		//
		// }
		// }
		// }
	}

	public HashSet<Obstacle> getNewObstacles() {
		HashSet<Obstacle> diffObstacles = new HashSet<Obstacle>();
		HashSet<Obstacle> beforeObstacles = this.getEnvironment().getObstacles();
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

	public Set<Edge> updateAffectedEdges(HashSet<Obstacle> diffObstacles) {
		Set<Edge> affectedEdges = new HashSet<Edge>();
		for (Obstacle obstacle : diffObstacles) {
			for (Edge edge : this.getEnvironment().getEdgeList()) {
				if (obstacle.getExtent(this.getEnvironment().getGlobe()).intersects(edge.getLine()))
					affectedEdges.add(edge);
			}
		}

		return affectedEdges;
	}

	public void updateEdgeCosts(HashSet<Obstacle> diffObstacles) {
		Set<Edge> affectedEdges = this.updateAffectedEdges(diffObstacles);
		if (!affectedEdges.isEmpty()) {
			this.setInflation(0d);
//			System.out.println("consistency procedure");
			this.consistencyProcedure(affectedEdges);
//			System.out.println("ending consistency procedure");
		}
		return;
	}

	/**
	 * Improves a plan incrementally.
	 */
	protected Trajectory improve(Position origin, Position destination, ZonedDateTime etd) {
		Trajectory trajectory = null;
		this.inflate();
		if (this.isInflated()) {
//			System.out.println("ALREADY MAX IMPROVEDDDDDDDD");
//			if (!this.updateStart()) {
//				this.clearExpandables();
//				this.addExpandable(this.getStart());
//			}
		}
		this.clearExpanded();
		trajectory = this.compute(origin, destination, etd);
		return trajectory;
	}

	protected void updateWaypoint(FADPRMWaypoint waypoint) {
		// if (waypoint.getSearch() == this.getCounter())
		// return;
		if (waypoint.getSearch() != 0 && waypoint.getSearch() != this.getCounter()) {
			if (waypoint.getG() + waypoint.getH() < pathCost.get((waypoint.getSearch()))) {
				waypoint.setH(pathCost.get(waypoint.getSearch()) - waypoint.getG());
				waypoint.setG(0);
			}
		} else if (waypoint.getSearch() == 0) {
			waypoint.setG(0);
		}
		waypoint.setSearch(counter);
	}

	public void updateSuccessors(FADPRMWaypoint source) {
		for (FADPRMWaypoint waypoint : source.getSuccessors()) {
			this.updateWaypoint(waypoint);
			waypoint.setBeta(this.getInflation());
			this.addExpandable(waypoint);
		}
	}

	@SuppressWarnings("unchecked")
	public void updateDensity(FADPRMWaypoint waypoint) {
		int density = 0;
		for (FADPRMWaypoint neighbor : (List<FADPRMWaypoint>) this.getEnvironment().getWaypointList()) {
			if (this.getEnvironment().getNormalizedDistance(waypoint, neighbor) < 0.2) {
				density++;
				neighbor.incrementDensity();
			}
		}
		waypoint.setDensity(density);
		return;
	}

	public boolean connectToGoal(FADPRMWaypoint source) {
		if (this.areConnectable(source, this.getGoal(), 0)) {
			if (!this.getEdgeList().contains(new Edge(source, this.getGoal()))) {
				this.createEdge(source, this.getGoal());
			}
			return true;
		}
		return false;
	}

	protected void computeOrImprovePath() {
		while (this.canExpand()) {
			FADPRMWaypoint source = this.pollExpandable();
			if (source.equals(this.getGoal())) {
//				System.out.println("goal getcost: " + this.getGoal().getCost());
				this.connectPlan(source);
				return;
			}
			if (this.connectToGoal(source)) {
				this.computeCost(source, this.getGoal());
				source.addSuccessor(this.getGoal());
			}
			FADPRMWaypoint newSuccessor = this.expand(source);
			this.updateDensity(newSuccessor);
			source.addSuccessor(newSuccessor);
			this.updateSuccessors(source);
			this.addExpanded(source);
		}
		return;
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
		if (trajectory == null)
			return false;

		HashSet<Waypoint> conflictWaypoints = new HashSet<Waypoint>();
		HashSet<Edge> conflictEdges = new HashSet<Edge>();

		for (Waypoint waypoint : trajectory.getWaypoints()) {
			if (this.getEnvironment().checkConflict(waypoint))
				conflictWaypoints.add(waypoint);
		}

		if (!conflictWaypoints.isEmpty()) {
			this.correctListsW(conflictWaypoints);
			return false;
		}
		Waypoint wpt1 = Iterables.get(trajectory.getWaypoints(), 0);
		Waypoint wpt2;
		for (int i = 0; i < trajectory.getLength() - 1; i++) {
			wpt2 = Iterables.get(trajectory.getWaypoints(), i + 1);
			if (this.getEnvironment().checkConflict(wpt1, wpt2)) {
				conflictEdges.add(new Edge(wpt1, wpt2));
			}
			wpt1 = wpt2;
		}
		if (!conflictEdges.isEmpty()) {
			this.correctListsE(conflictEdges);
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
	protected void correctListsW(HashSet<Waypoint> conflictWaypoints) {
		for (Waypoint waypoint : conflictWaypoints) {
			this.getEnvironment().getWaypointList().remove(waypoint);
			this.getEnvironment().getEdgeList()
					.removeIf(s -> s.getPosition1().equals(waypoint) || s.getPosition2().equals(waypoint));
		}
	}

	/**
	 * Corrects the waypoint and edge list by removing the edges that are in
	 * conflict with terrain obstacles.
	 * 
	 * @param conflictEdges the set of edges that are in conflict with terrain
	 *            obstacles
	 */
	protected void correctListsE(HashSet<Edge> conflictEdges) {
		for (Edge edge : conflictEdges) {
			this.getEnvironment().getEdgeList().remove(edge);
		}
	}

	protected Trajectory compute(Position origin, Position destination, ZonedDateTime etd) {
		Trajectory trajectory = null;

		while (!this.correctTrajectory(trajectory)) {
			this.computeOrImprovePath();
			trajectory = this.createTrajectory();
			this.incrementCounter();
			this.addPathCost();
		}
		this.revisePlan(trajectory);
		return trajectory;
	}

	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {
		this.clearExpandables();
		this.clearExpanded();

		this.setStart(this.createWaypoint(origin));
		this.getStart().setEto(etd);

		this.setGoal(this.createWaypoint(destination));

		this.setCounter(1);

		this.pathCost.add(0d);
		this.updateWaypoint(this.getStart());
		this.updateWaypoint(this.getGoal());

		this.addExpandable(this.getStart());
	}

	public boolean updateStart() {
		FADPRMWaypoint oldStart = this.getStart();
		for (FADPRMWaypoint waypoint : this.getWaypointList()) {
			if (waypoint.equals(this.plan.get(1))) {
				this.setStart(this.plan.get(1));
				this.getStart().setParent(null);
				waypoint.setParent(null);
			}
		}
		FADPRMWaypoint newStart = this.getStart();
		return newStart.equals(oldStart);
	}

	/**
	 * @param origin
	 * @param destination
	 * @param etd
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.ai.Planner#plan(gov.nasa.worldwind.geom.Position,
	 *      gov.nasa.worldwind.geom.Position, java.time.ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		this.initialize(origin, destination, etd);
		this.setInflation(this.getMinimumQuality());
		Trajectory trajectory = this.compute(origin, destination, etd);
		HashSet<Obstacle> diffObstacles = this.getNewObstacles();
		while (!this.getStart().equals(this.getGoal())) {
//			this.updateEdgeCosts(diffObstacles);
//			System.out.println("diffObstacles " +diffObstacles.size());
			this.improve(origin, destination, etd);
			diffObstacles.clear();
		}
//		System.out.println("finished ITS OVERRRRRRRRRRRRRR");
		return trajectory;
	}

	/**
	 * @param origin
	 * @param destination
	 * @param waypoints
	 * @param etd
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.ai.Planner#plan(gov.nasa.worldwind.geom.Position,
	 *      gov.nasa.worldwind.geom.Position, java.util.List,
	 *      java.time.ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
		this.setInflation(this.getMinimumQuality());
		Trajectory trajectory = this.oneTimePlan(origin, destination, waypoints, etd);

		while (!this.isInflated()) {
			this.inflate();
			this.addExpandable(this.getStart());
			this.clearExpanded();
			trajectory = this.oneTimePlan(origin, destination, waypoints, etd);

			this.revisePlan(trajectory);
		}

		return trajectory;
	}

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
				Trajectory part = this.compute(currentOrigin, currentDestination, currentEtd);

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
	 * Indicates whether or not this FADPRM planner supports a specified
	 * environment.
	 * 
	 * @param environment the environment
	 * 
	 * @return true if the environment is a planning continuum, false otherwise
	 * 
	 * @see SamplingEnvironment
	 */
	@Override
	public boolean supports(Environment environment) {
		boolean supports = super.supports(environment);

		if (supports) {
			supports = (environment instanceof SamplingEnvironment);
		}

		return supports;
	}
}
