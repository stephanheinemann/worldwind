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
import java.util.PriorityQueue;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.ai.AbstractPlanner;
import com.cfar.swim.worldwind.ai.AnytimePlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.planning.Edge;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.PlanningContinuum;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;

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

	/**
	 * @param aircraft
	 * @param environment
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
	 * @param counter
	 *            the counter to set
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
	 * @param initialBeta
	 *            the minimum quality (initial beta) of this FADPRM planner
	 * 
	 * @throws IllegalArgumentException
	 *             if the initial beta is invalid
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
	 * @param finalBeta
	 *            the maximum quality (final beta) of this FADPRM planner
	 * 
	 * @throws IllegalArgumentException
	 *             if the final inflation is invalid
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
	 * @param deflationAmount
	 *            the quality improvement (stepBeta) of this FADPRM planner
	 * 
	 * @throws IllegalArgumentException
	 *             if the deflation amount is invalid
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
	 * @param beta
	 *            the current beta of this FADPRM planner
	 */
	protected void setInflation(double beta) {
		this.beta = beta;
	}

	/**
	 * Gets the continuum environment of this planner
	 * 
	 * @return the continuum environment
	 */
	public PlanningContinuum getEnvironment() {
		return (PlanningContinuum) super.getEnvironment();
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
	 * @param waypointList
	 *            the list of waypoints to set
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
	 * @param edgetList
	 *            the list of edges to set
	 * 
	 */
	public void setEdgeList(List<Edge> edgeList) {
		this.getEnvironment().setEdgeList(edgeList);
	}

	/**
	 * Gets the start FADPRM waypoint of this forward FADPRM planner.
	 * 
	 * @return the start FADPRM waypoint of this forward FADPRM planner
	 */
	protected FADPRMWaypoint getStart() {
		return this.start;
	}

	/**
	 * Sets the start FADPRM waypoint of this forward FADPRM planner.
	 * 
	 * @param start
	 *            the start waypoint of this forward FADPRM planner
	 */
	protected void setStart(FADPRMWaypoint start) {
		this.start = start;
	}

	/**
	 * Gets the goal FADPRM waypoint of this forward FADPRM planner.
	 * 
	 * @return the goal FADPRM waypoint of this forward FADPRM planner
	 */
	protected FADPRMWaypoint getGoal() {
		return this.goal;
	}

	/**
	 * Sets the goal FADPRM waypoint of this forward FADPRM planner.
	 * 
	 * @param goal
	 *            the goal FADPRM waypoint of this forward FADPRM planner
	 */
	protected void setGoal(FADPRMWaypoint goal) {
		this.goal = goal;
	}

	/**
	 * Indicates whether or not a position is within the start region.
	 * 
	 * @param position
	 *            the position
	 * 
	 * @return true if the position is within the start region, false otherwise
	 */
	protected boolean isInStartRegion(Position position) {
		return this.getEnvironment().getDistance(position, this.getStart()) < 200d;
	}

	/**
	 * Indicates whether or not a position is within the goal region.
	 * 
	 * @param position
	 *            the position
	 * 
	 * @return true if the position is within the goal region, false otherwise
	 */
	protected boolean isInGoalRegion(Position position) {
		return this.getEnvironment().getDistance(position, this.getGoal()) < 200d;
	}

	/**
	 * Adds an FADPRM waypoint to the expandable FADPRM waypoints.
	 * 
	 * @param waypoint
	 *            the FADPRM waypoint
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
	 * @param waypoint
	 *            the FADPRM waypoint
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
	 * @param waypoint
	 *            the FADPRM waypoint
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
	 * @param waypoint
	 *            the FADPRM waypoint
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
	 * Remake this function
	 * 
	 * @param position
	 * @return
	 */
	public Position getCornerBox(Position position) {
		Vec4 point = this.getEnvironment().getGlobe().computePointFromPosition(position);
		List<Vec4> corners = new ArrayList<Vec4>();

		// TODO: create box according to aircraft dimensions
		double halfDistance = 40d;

		corners.add(point.add3(-halfDistance, +halfDistance, -halfDistance));
		corners.add(point.add3(+halfDistance, +halfDistance, -halfDistance));
		corners.add(point.add3(+halfDistance, -halfDistance, -halfDistance));
		corners.add(point.add3(+halfDistance, -halfDistance, +halfDistance));
		corners.add(point.add3(-halfDistance, -halfDistance, +halfDistance));
		corners.add(point.add3(-halfDistance, +halfDistance, +halfDistance));

		Random generator = new Random();
		int i = generator.nextInt(6);

		return this.getEnvironment().getGlobe().computePositionFromPoint(corners.get(i));
	}

	/**
	 * Expands an FADPRM waypoint towards its neighbors in the environment.
	 * 
	 * @param waypoint
	 *            the FADPRM waypoint to be expanded
	 * 
	 * @return the new sampled waypoint
	 */
	protected FADPRMWaypoint expand(FADPRMWaypoint waypoint) {

		Position newPosition = this.getCornerBox(waypoint);
		while (!this.getEnvironment().contains(newPosition))
			newPosition = this.getCornerBox(waypoint);

		FADPRMWaypoint newWaypoint = this.createWaypoint(newPosition);
		this.updateWaypointH(newWaypoint);
		this.createEdge(waypoint, newWaypoint);
		return newWaypoint;
	}

	/**
	 * Creates an edge between a source waypoint and a target waypoint, and adds it
	 * to the edge list
	 * 
	 * @param source
	 *            the source FADPRM waypoint
	 * @param target
	 *            the target FADPRM waypoint
	 */
	protected void createEdge(Waypoint source, Waypoint target) {
		Globe globe = this.getEnvironment().getGlobe();
		Line line = new Line(globe.computePointFromPosition(source), globe.computePointFromPosition(target));
		Edge edgeNew = new Edge(source, target, line);
		edgeNew.setCostIntervals(this.getEnvironment().embedIntervalTree(edgeNew.getLine()));
		this.getEdgeList().add(edgeNew);
	}

	/**
	 * Creates a waypoint at a specified position, initializes its search value to
	 * 0, and adds it to the waypoint list.
	 * 
	 * @param position
	 *            the position in global coordinates
	 * @return the new created waypoint
	 */
	protected FADPRMWaypoint createWaypoint(Position position) {
		FADPRMWaypoint newWaypoint = new FADPRMWaypoint(position);
		newWaypoint.setSearch(0);
		this.getWaypointList().add(newWaypoint);
		return newWaypoint;
	}

	/**
	 * Computes the estimated cost of a specified target FADPRM waypoint when
	 * reached via a specified source FADPRM waypoint.
	 * 
	 * @param source
	 *            the source FADPRM waypoint in globe coordinates
	 * @param target
	 *            the target FADPRM waypoint in globe coordinates
	 */
	protected void computeCost(FADPRMWaypoint source, FADPRMWaypoint target) {

		Path leg = new Path(source, target);
		Capabilities capabilities = this.getAircraft().getCapabilities();
		Globe globe = this.getEnvironment().getGlobe();
		// TODO: catch IllegalArgumentException (incapable) and exit
		ZonedDateTime end = capabilities.getEstimatedTime(leg, globe, source.getEto());
		target.setEto(end);

		double distance = this.getEnvironment().getNormalizedDistance(source, target);

		double pathDD = 1 / (this.getEnvironment().getStepCost(source, target, source.getEto(), end,
				this.getCostPolicy(), this.getRiskPolicy()) / distance);

		target.setCost(source.getCost() + distance);
		target.setPathDD((source.getPathDD() + pathDD) / 2);
		target.setG(target.getPathDD() / (1 + target.getLambda() * distance));
		// this.updateWaypointH(target);
	}

	protected void updateWaypoint(FADPRMWaypoint waypoint) {
		if (waypoint.getSearch() != 0) {
			if (waypoint.getG() + waypoint.getH() < pathCost.get((waypoint.getSearch() - 1))) {
				waypoint.setH(pathCost.get(waypoint.getSearch() - 1) - waypoint.getG());
				waypoint.setG(0);
			}
		} else if (waypoint.getSearch() == 0) {
			waypoint.setG(0);
		}
		waypoint.setSearch(counter);
	}

	protected void addPathCost() {
		this.pathCost.add(this.getGoal().getSearch() - 1, this.getGoal().getG());
	}

	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {
		this.clearExpandables();
		this.clearExpanded();

		this.setStart(this.createWaypoint(origin));
		this.getStart().setEto(etd);
		// start set h to goal?

		this.setGoal(this.createWaypoint(destination));

		this.setCounter(1);

		this.pathCost.add(0, 0d);
		this.updateWaypoint(this.getStart());
		this.updateWaypointH(this.getStart());
		this.updateWaypoint(this.getGoal());
		this.updateWaypointH(this.getGoal());

		this.addExpandable(this.getStart());
	}

	protected Trajectory compute(Position origin, Position destination, ZonedDateTime etd) {
		Trajectory trajectory = null;

		while (!this.isFeasible(trajectory)) {
			this.rearrangeRoadmap();
			this.computeOrImprovePath();
			this.incrementCounter();
			trajectory = this.aStarPlan(origin, destination, etd);
			this.pathCost.add(this.getGoal().getG());

		}
		return trajectory;
	}

	protected boolean isFeasible(Trajectory trajectory) {
		if (trajectory == null)
			return false;
		return true;
	}

	protected void rearrangeRoadmap() {
		return;
		// TODO: update edge and waypoint lists according to the last computed
		// trajectory
	}

	protected void computeOrImprovePath() {
		while (this.canExpand()) {
			FADPRMWaypoint source = this.pollExpandable();

			// alternative approach: check if source is close enough to goal using
			// environment normalized distance.
			if (this.isInGoalRegion(source)) {
				this.createEdge(source, this.getGoal());
				this.computeCost(source, this.getGoal());
				this.getGoal().setH(0d);
				return;
			}

			FADPRMWaypoint newSuccessor = this.expand(source);
			this.updateDensity(newSuccessor);
			source.addSuccessor(newSuccessor);

			for (FADPRMWaypoint waypoint : source.getSuccessors()) {
				this.updateWaypoint(waypoint);
				this.computeCost(source, waypoint);
				this.addExpandable(waypoint);
			}
			this.addExpanded(source);

		}
		System.out.println("Open is empty");
	}

	protected void updateWaypointH(FADPRMWaypoint waypoint) {

		// Path leg = new Path(waypoint, this.getGoal());
		// Capabilities capabilities = this.getAircraft().getCapabilities();
		// Globe globe = this.getEnvironment().getGlobe();
		// // TODO: catch IllegalArgumentException (incapable) and exit
		// ZonedDateTime end = capabilities.getEstimatedTime(leg, globe,
		// waypoint.getEto());
		//
		// double distance = this.getEnvironment().getNormalizedDistance(waypoint,
		// this.getGoal());
		//
		// // Edge edge = this.createEdge(waypoint, this.getGoal());
		// Line line = new
		// Line(this.getEnvironment().getGlobe().computePointFromPosition(waypoint),
		// this.getEnvironment().getGlobe().computePointFromPosition(this.getGoal()));
		// Edge edge = new Edge(waypoint, this.getGoal(), line);
		// edge.setCostIntervals(this.getEnvironment().embedIntervalTree(edge.getLine()));
		// // waypoint eto not set
		// double pathDD = 1 / edge.calculateCost(waypoint.getEto(), end);
		//
		// waypoint.setH(pathDD / (1 + waypoint.getLambda() * distance));

		waypoint.setH(this.getEnvironment().getNormalizedDistance(waypoint, this.getGoal()));
	}

	/**
	 * Improves a plan incrementally.
	 */
	protected void improve(Position origin, Position destination, ZonedDateTime etd) {
		while (!this.isInflated()) {
			this.inflate();
			for (FADPRMWaypoint waypoint : this.getWaypointList()) {
				waypoint.setBeta(this.getInflation());
			}
			this.addExpandable(this.getStart());
			this.clearExpanded();
			Trajectory trajectory = this.compute(origin, destination, etd);
			this.revisePlan(trajectory);
		}
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
		this.revisePlan(trajectory);
		this.improve(origin, destination, etd);
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
			for (FADPRMWaypoint waypoint : this.getWaypointList()) {
				waypoint.setBeta(this.getInflation());
			}
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

	public void updateDensity(FADPRMWaypoint waypoint) {
		long density = 0;

		for (FADPRMWaypoint neighbor : this.getWaypointList()) {
			if (this.getEnvironment().getNormalizedDistance(waypoint, neighbor) < 0.1) {
				density++;
				neighbor.incrementDensity();
			}
		}

		waypoint.setDensity((int) density);
	}

	/**
	 * Indicates whether or not this FADPRM planner supports a specified
	 * environment.
	 * 
	 * @param environment
	 *            the environment
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

	/**
	 * Creates a new box equivalent to the one used to construct the
	 * PlanningContinuum of this Basic PRM planner
	 * 
	 * @param cont
	 *            the planning Continuum environment of this Basic PRM planner
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

	public Trajectory aStarPlan(Position origin, Position destination, ZonedDateTime etd) {
//		Box box = this.createBox(this.getEnvironment());
//		PlanningRoadmap roadmap = new PlanningRoadmap(box, this.getWaypointList(), this.getEdgeList(),
//				this.getEnvironment().getGlobe());
//
//		ForwardAStarPlanner aStar = new ForwardAStarPlanner(this.getAircraft(), roadmap);
//		aStar.setCostPolicy(this.getCostPolicy());
//		aStar.setRiskPolicy(this.getRiskPolicy());
//		return aStar.plan(origin, destination, etd);
		return null;
	}
}
