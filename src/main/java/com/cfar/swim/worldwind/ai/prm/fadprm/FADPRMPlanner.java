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
import java.util.List;
import java.util.PriorityQueue;
import java.util.Random;
import java.util.Set;

import com.cfar.swim.worldwind.ai.AbstractPlanner;
import com.cfar.swim.worldwind.ai.AnytimePlanner;
import com.cfar.swim.worldwind.ai.astar.astar.ForwardAStarPlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.planning.Edge;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.PlanningContinuum;
import com.cfar.swim.worldwind.planning.PlanningRoadmap;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Line;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;

/**
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

	/** the start region */
	private Box startRegion;

	/** the goal region */
	private Box goalRegion;

	/** the number of times the planner was called */
	private int counter = 0;

	/** the cost of the best path for each iteration of the algorithm */
	private List<Double> pathCost = new ArrayList<Double>();

	/** the initial inflation factor applied to the heuristic function */
	private double initialInflation;

	/** the final inflation factor applied the heuristic function */
	private double finalInflation;

	/** the current inflation factor applied to the heuristic function */
	private double inflation;

	/** the deflation amount to be applied to the current inflation */
	private double inflationAmount;

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
	 * Gets the minimum quality (initial inflation) of this ARFADPRM planner.
	 * 
	 * @return the minimum quality (initial inflation) of this ARFADPRM planner
	 * 
	 * @see AnytimePlanner#getMinimumQuality()
	 */
	@Override
	public double getMinimumQuality() {
		return this.initialInflation;
	}

	/**
	 * Sets the minimum quality (initial inflation) of this ARFADPRM planner.
	 * 
	 * @param initialInflation the minimum quality (initial inflation) of this
	 *            ARFADPRM planner
	 * 
	 * @throws IllegalArgumentException if the initial inflation is invalid
	 * 
	 * @see AnytimePlanner#setMinimumQuality(double)
	 */
	@Override
	public void setMinimumQuality(double initialInflation) {
		if ((0d <= initialInflation) && (initialInflation <= this.finalInflation)) {
			this.initialInflation = initialInflation;
		} else {
			throw new IllegalArgumentException("initial inflation is invalid");
		}
	}

	/**
	 * Gets the maximum quality (final inflation) of this FADPRM planner.
	 * 
	 * @return the maximum quality (final inflation) of this FADPRM planner
	 * 
	 * @see AnytimePlanner#getMaximumQuality()
	 */
	@Override
	public double getMaximumQuality() {
		return this.finalInflation;
	}

	/**
	 * Sets the maximum quality (initial inflation) of this ARFADPRM planner.
	 * 
	 * @param finalInflation the maximum quality (final inflation) of this ARFADPRM
	 *            planner
	 * 
	 * @throws IllegalArgumentException if the final inflation is invalid
	 * 
	 * @see AnytimePlanner#setMaximumQuality(double)
	 */
	@Override
	public void setMaximumQuality(double finalInflation) {
		if ((1d >= finalInflation) && (this.initialInflation <= finalInflation)) {
			this.finalInflation = finalInflation;
		} else {
			throw new IllegalArgumentException("final deflation is invalid");
		}
	}

	/**
	 * Gets the quality improvement (deflation amount) of this ARFADPRM planner.
	 * 
	 * @return the quality improvement (deflation amount) of this ARFADPRM planner
	 * 
	 * @see AnytimePlanner#getQualityImprovement()
	 */
	@Override
	public double getQualityImprovement() {
		return this.inflationAmount;
	}

	/**
	 * Sets the quality improvement (deflation amount) of this ARFADPRM planner.
	 * 
	 * @param deflationAmount the quality improvement (deflation amount) of this
	 *            ARFADPRM planner
	 * 
	 * @throws IllegalArgumentException if the deflation amount is invalid
	 * 
	 * @see AnytimePlanner#setQualityImprovement(double)
	 */
	@Override
	public void setQualityImprovement(double inflationAmount) {
		if (0d < inflationAmount) {
			this.inflationAmount = inflationAmount;
		} else {
			throw new IllegalArgumentException("deflation amount is invalid");
		}
	}

	/**
	 * Deflates the current inflation by the deflation amount, or otherwise to the
	 * final deflation.
	 */
	protected void inflate() {
		if (this.finalInflation > (this.inflation + this.inflationAmount)) {
			this.inflation += this.inflationAmount;
		} else {
			this.inflation = this.finalInflation;
		}
	}

	/**
	 * Determines whether or not the final deflation has been reached.
	 * 
	 * @return true if the final deflation has been reached, false otherwise
	 */
	protected boolean isInflated() {
		return (this.inflation == this.finalInflation);
	}

	/**
	 * Gets the current inflation of this ARFADPRM planner.
	 * 
	 * @return the current inflation of this ARFADPRM planner
	 */
	protected double getInflation() {
		return this.inflation;
	}

	/**
	 * Sets the current inflation of this ARFADPRM planner.
	 * 
	 * @param inflation the current inflation of this ARFADPRM planner
	 */
	protected void setInflation(double inflation) {
		this.inflation = inflation;
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
	 * @param start the start waypoint of this forward FADPRM planner
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
	 * @param goal the goal FADPRM waypoint of this forward FADPRM planner
	 */
	protected void setGoal(FADPRMWaypoint goal) {
		this.goal = goal;
	}

	/**
	 * Sets the start region of this forward FADPRM planner.
	 * 
	 * @param startRegion the start region of this forward FADPRM planner
	 */
	protected void setStartRegion(Box startRegion) {
		this.startRegion = startRegion;
	}

	/**
	 * Indicates whether or not a position is within the start region.
	 * 
	 * @param position the position
	 * 
	 * @return true if the position is within the start region, false otherwise
	 */
	protected boolean isInStartRegion(Position position) {
		return this.startRegion.contains(this.getEnvironment().getGlobe().computePointFromPosition(position));
	}

	/**
	 * Sets the goal region of this forward FADPRM planner.
	 * 
	 * @param goalRegion the goal region of this forward FADPRM planner
	 */
	protected void setGoalRegion(Box goalRegion) {
		this.goalRegion = goalRegion;
	}

	/**
	 * Indicates whether or not a position is within the goal region.
	 * 
	 * @param position the position
	 * 
	 * @return true if the position is within the goal region, false otherwise
	 */
	protected boolean isInGoalRegion(Position position) {
		return this.goalRegion.contains(this.getEnvironment().getGlobe().computePointFromPosition(position));
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

	public Position getCornerBox(Position position) {
		Vec4 point = this.getEnvironment().getGlobe().computePointFromPosition(position);
		List<Vec4> corners = new ArrayList<Vec4>();

		// TODO: create box according to aircraft dimensions
		double halfDistance = 50d;

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
	
	public Box createBoundingBox(Position position) {
		Vec4 point = this.getEnvironment().getGlobe().computePointFromPosition(position);
		List<Vec4> corners = new ArrayList<Vec4>();

		// TODO: create box according to aircraft dimensions
		double halfDistance = 100d;

		corners.add(point.add3(-halfDistance, +halfDistance, -halfDistance));
		corners.add(point.add3(+halfDistance, +halfDistance, -halfDistance));
		corners.add(point.add3(+halfDistance, -halfDistance, -halfDistance));
		corners.add(point.add3(+halfDistance, -halfDistance, +halfDistance));
		corners.add(point.add3(-halfDistance, -halfDistance, +halfDistance));
		corners.add(point.add3(-halfDistance, +halfDistance, +halfDistance));

		return new Box(gov.nasa.worldwind.geom.Box.computeBoundingBox(corners));
	}

	/**
	 * Expands an FADPRM waypoint towards its neighbors in the environment.
	 * 
	 * @param waypoint the FADPRM waypoint to be expanded
	 * 
	 * @return the neighbors of the expanded FADPRM waypoint
	 */
	protected FADPRMWaypoint expand(FADPRMWaypoint waypoint) {

		Position newPosition = getCornerBox(waypoint);
		while (!this.getEnvironment().contains(newPosition))
			newPosition = getCornerBox(waypoint);

		FADPRMWaypoint newWaypoint = this.createWaypoint(newPosition);

		this.createEdge(waypoint, newWaypoint);
		return newWaypoint;
	}

	/**
	 * Creates an edge between a source waypoint and a target waypoing, and adds it
	 * to the edge list
	 * 
	 * @param source the source Basic PRM waypoint
	 * @param target the target Basic PRM waypoint
	 */
	protected void createEdge(Waypoint source, Waypoint target) {
		Globe globe = this.getEnvironment().getGlobe();
		Line line = new Line(globe.computePointFromPosition(source), globe.computePointFromPosition(target));
		Edge edgeNew = new Edge(source, target, line);
		edgeNew.setCostIntervals(this.getEnvironment().embedIntervalTree(edgeNew.getLine()));
		this.getEdgeList().add(edgeNew);
	}

	protected FADPRMWaypoint createWaypoint(Position position) {
		FADPRMWaypoint newWaypoint = new FADPRMWaypoint(position);
		newWaypoint.setEto(this.getEnvironment().getTime());
		newWaypoint.setSearch(0);
		this.getWaypointList().add(newWaypoint);
		return newWaypoint;
	}

	/**
	 * Computes the estimated cost of a specified target FADPRM waypoint when
	 * reached via a specified source FADPRM waypoint.
	 * 
	 * @param source the source FADPRM waypoint in globe coordinates
	 * @param target the target FADPRM waypoint in globe coordinates
	 */
	protected void computeCost(FADPRMWaypoint source, FADPRMWaypoint target) {
		Path leg = new Path(source, target);
		Capabilities capabilities = this.getAircraft().getCapabilities();
		Globe globe = this.getEnvironment().getGlobe();
		// TODO: catch IllegalArgumentException (incapable) and exit
		ZonedDateTime end = capabilities.getEstimatedTime(leg, globe, source.getEto());

		double distance = this.getEnvironment().getNormalizedDistance(source, target);

		double pathDD = 1 / (this.getEnvironment().getStepCost(source, target, source.getEto(), end,
				this.getCostPolicy(), this.getRiskPolicy()) / distance);

		target.setCost(source.getCost() + distance);
		target.setPathDD((source.getPathDD() + pathDD) / 2);
		target.setG(target.getPathDD() / (1 + target.getLambda() * distance));
		target.setEto(end);
		this.updateWaypointH(target);
	}

	protected void updateWaypoint(FADPRMWaypoint waypoint) {
		if (waypoint.getSearch() != 0) {
			if (waypoint.getG() + waypoint.getH() < pathCost.get((waypoint.getSearch()))) {
				waypoint.setH(pathCost.get(waypoint.getSearch()) - waypoint.getG());
				waypoint.setG(0);
			}
		} else if (waypoint.getSearch() == 0) {
			waypoint.setG(0);
		}
		waypoint.setSearch(counter);
	}

	protected void addPathCost() {
		this.pathCost.add(this.getGoal().getSearch(), this.getGoal().getG());
	}

	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {
		this.clearExpandables();
		this.clearExpanded();
		
		this.setStart(this.createWaypoint(origin));
		this.getStart().setEto(etd);
		//start set h to goal?

		this.setGoal(this.createWaypoint(destination));

		this.setInflation(this.getMinimumQuality());
		this.setCounter(1);

		this.pathCost.add(0, 0d);
		this.updateWaypoint(this.getStart());
		this.updateWaypoint(this.getGoal());
		this.addPathCost();
		
		this.setStartRegion(this.createBoundingBox(origin));
		this.setGoalRegion(this.createBoundingBox(destination));

		this.addExpandable(this.getStart());
	}

	protected Trajectory compute(Position origin, Position destination, ZonedDateTime etd) {
		Trajectory trajectory = null;
		
		while(!this.isFeasible(trajectory)) {
			this.rearrangeRoadmap();
			this.computeOrImprovePath();
			this.addPathCost();
			trajectory = this.aStarPlan(origin, destination, etd);
			this.incrementCounter();
			
		}
		return trajectory;
	}
	
	protected boolean isFeasible(Trajectory trajectory) {
		if(trajectory==null)
			return false;
		return true;
	}
	
	protected void rearrangeRoadmap() {
		return;
		// TODO: update edge and waypoint lists according to the last computed trajectory
	}

	protected void computeOrImprovePath() {
		while (this.canExpand()) {
			FADPRMWaypoint source = this.pollExpandable();

			// alternative approach: check if source is close enough to goal using environment normalized distance.
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

		Path leg = new Path(waypoint, this.getGoal());
		Capabilities capabilities = this.getAircraft().getCapabilities();
		Globe globe = this.getEnvironment().getGlobe();
		// TODO: catch IllegalArgumentException (incapable) and exit
		ZonedDateTime end = capabilities.getEstimatedTime(leg, globe, waypoint.getEto());

		double distance = this.getEnvironment().getNormalizedDistance(waypoint, this.getGoal());

		// Edge edge = this.createEdge(waypoint, this.getGoal());
		Line line = new Line(this.getEnvironment().getGlobe().computePointFromPosition(waypoint),
				this.getEnvironment().getGlobe().computePointFromPosition(this.getGoal()));
		Edge edge = new Edge(waypoint, this.getGoal(), line);
		edge.setCostIntervals(this.getEnvironment().embedIntervalTree(edge.getLine()));
		// waypoint eto not set
		double pathDD = 1 / edge.calculateCost(waypoint.getEto(), end);

		waypoint.setH(pathDD / (1 + waypoint.getLambda() * distance));
	}

	/**
	 * Improves a plan incrementally.
	 */
	protected void improve(Position origin, Position destination, ZonedDateTime etd) {
		while (!this.isInflated()) {
			this.inflate();
			System.out.println("waypointlistsize: "+this.getWaypointList().size());
			for (FADPRMWaypoint waypoint : this.getWaypointList()) {
				waypoint.setBeta(this.getInflation());
			}
			this.addExpandable(this.getStart());
			this.clearExpanded();
			Trajectory trajectory = this.compute(origin, destination, etd);
			System.out.println("distance: " + trajectory.getCost());
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
		Trajectory trajectory = this.compute(origin, destination, etd);
		System.out.println("distance: " + trajectory.getCost());
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
		// TODO Auto-generated method stub
		return null;
	}

	public void updateDensity(FADPRMWaypoint waypoint) {
		long density = 0;
		
//		density = this.getWaypointList().stream().filter(t -> this.getEnvironment().getNormalizedDistance(waypoint, t) < 0.1)
//		.count();
		for(FADPRMWaypoint neighbor : this.getWaypointList()) {
			if(this.getEnvironment().getNormalizedDistance(waypoint, neighbor) < 0.1) {
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
	
	
	public Trajectory aStarPlan(Position origin, Position destination, ZonedDateTime etd) {
		Box box = this.createBox(this.getEnvironment());
		PlanningRoadmap roadmap = new PlanningRoadmap(box, this.getWaypointList(), this.getEdgeList(),
				this.getEnvironment().getGlobe());

		ForwardAStarPlanner aStar = new ForwardAStarPlanner(this.getAircraft(), roadmap);
		aStar.setCostPolicy(this.getCostPolicy());
		aStar.setRiskPolicy(this.getRiskPolicy());
		return aStar.plan(origin, destination, etd);
	}
}
