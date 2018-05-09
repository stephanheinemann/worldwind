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
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.PlanningContinuum;
import com.cfar.swim.worldwind.planning.SamplingEnvironment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;

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
	 * Gets the set of all FADPRM waypoints already sampled.
	 * 
	 * @return the set of all sampled FADPRM waypoints.
	 */
	protected Set<FADPRMWaypoint> getAllWaypoints() {
		Set<FADPRMWaypoint> waypoints = new HashSet<FADPRMWaypoint>();
		waypoints = this.getEnvironment().getAllPositions().stream().map(this::createWaypoint)
				.collect(Collectors.toSet());
		return waypoints;
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

		double halfDistance = this.getEnvironment().getDiameter() * 0.2;

		corners.add(point.add3(-halfDistance, +halfDistance, -halfDistance));
		corners.add(point.add3(+halfDistance, +halfDistance, -halfDistance));
		corners.add(point.add3(+halfDistance, -halfDistance, -halfDistance));
		corners.add(point.add3(+halfDistance, -halfDistance, +halfDistance));
		corners.add(point.add3(-halfDistance, -halfDistance, +halfDistance));
		corners.add(point.add3(-halfDistance, +halfDistance, +halfDistance));

		PlanningContinuum env = new PlanningContinuum(new Box(gov.nasa.worldwind.geom.Box.computeBoundingBox(corners)));
		env.setGlobe(this.getEnvironment().getGlobe());
		Position pos = env.sampleRandomPosition();
		while (this.getEnvironment().checkConflict(pos)) {
			pos = env.sampleRandomPosition();
		}
		return pos;
	}

	/**
	 * Expands an FADPRM waypoint towards its neighbors in the environment.
	 * 
	 * @param waypoint the FADPRM waypoint to be expanded
	 * 
	 * @return the new sampled waypoint
	 */
	protected FADPRMWaypoint expand(FADPRMWaypoint waypoint) {
		Position newPosition = this.samplePosition(waypoint);
		while (!this.getEnvironment().contains(newPosition))
			newPosition = this.samplePosition(waypoint);

		FADPRMWaypoint newWaypoint = this.createWaypoint(newPosition);
		this.getEnvironment().addChild(waypoint, newWaypoint);
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
		newWaypoint.setSearch(0);
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
		target.setEto(end);

		double distance = this.getEnvironment().getNormalizedDistance(source, target);
		target.setCost(source.getCost() + distance);

		// TODO: review desirable zones
		double pathDD = 1 / (this.getEnvironment().getStepCost(source, target, source.getEto(), end,
				this.getCostPolicy(), this.getRiskPolicy()) / distance);
		target.setPathDD((source.getPathDD() + pathDD) / 2);
		target.setG(target.getPathDD() / (1 + target.getLambda() * target.getCost()));
		target.setParent(source);
	}
	
	/**
	 * Computes the estimated cost of a specified target FADPRM waypoint when
	 * reached via a specified source FADPRM waypoint.
	 * 
	 * @param source the source FADPRM waypoint in globe coordinates
	 * @param target the target FADPRM waypoint in globe coordinates
	 */
	protected void computeCostGoal(FADPRMWaypoint source, FADPRMWaypoint target) {

		Path leg = new Path(source, target);
		Capabilities capabilities = this.getAircraft().getCapabilities();
		Globe globe = this.getEnvironment().getGlobe();
		// TODO: catch IllegalArgumentException (incapable) and exit
		ZonedDateTime end = capabilities.getEstimatedTime(leg, globe, source.getEto());

		double distance = this.getEnvironment().getNormalizedDistance(source, target);

		// TODO: review desirable zones
		double pathDD = 1 / (this.getEnvironment().getStepCost(source, target, source.getEto(), end,
				this.getCostPolicy(), this.getRiskPolicy()) / distance);
		if((pathDD / (1 + target.getLambda() * (source.getCost()+distance)) > target.getG())) {
			target.setPathDD((source.getPathDD() + pathDD) / 2);
			target.setCost(source.getCost() + distance);
			target.setG(target.getPathDD() / (1 + target.getLambda() * target.getCost()));
			target.setEto(end);
			target.setParent(source);
		}
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

	public void consistencyProcedure(Set<SamplingEnvironment> decreasedEdges) {
		this.clearExpandables();
		for (SamplingEnvironment env : decreasedEdges) {
			FADPRMWaypoint origin = (FADPRMWaypoint) env.getGlobe().computePositionFromPoint(env.getOrigin());
			FADPRMWaypoint opposite = (FADPRMWaypoint) env.getGlobe().computePositionFromPoint(env.get3DOpposite());
			this.updateWaypoint(origin);
			this.updateWaypoint(opposite);
			if (origin.getParent().equals(opposite)) {
				// not distance, but cost of going from origin to opposite
				if (origin.getH() > this.getEnvironment().getNormalizedDistance(origin, opposite) + opposite.getH()) {
					origin.setH(this.getEnvironment().getNormalizedDistance(origin, opposite) + opposite.getH());
					this.removeExpandable(origin);
					this.addExpandable(origin);
				}
			} else if (opposite.getParent().equals(origin)) {
				if (opposite.getH() > this.getEnvironment().getNormalizedDistance(origin, opposite) + origin.getH()) {
					opposite.setH(this.getEnvironment().getNormalizedDistance(origin, opposite) + origin.getH());
					this.removeExpandable(opposite);
					this.addExpandable(opposite);
				}
			}
		}
		while (this.canExpand()) {
			FADPRMWaypoint source = this.pollExpandable(); // not poll, but instead the one with lowest key
			FADPRMWaypoint parent = source.getParent();
			if (!parent.equals(this.getGoal())) {
				this.updateWaypoint(parent);
				for (FADPRMWaypoint successor : parent.getSuccessors()) {
					if (parent.getH() > this.getEnvironment().getNormalizedDistance(parent, successor)
							+ successor.getH()) {
						parent.setH(this.getEnvironment().getNormalizedDistance(parent, successor) + successor.getH());
						this.removeExpandable(parent);
						this.addExpandable(parent);
					}
				}
			}
		}
	}

	public void updateEdgeCosts() {
		Set<SamplingEnvironment> decreasedEdges = null;
		// TODO: create method in sampling environment to update edges cost
		// decreasedEdges = this.getEnvironment().updateEdges();
		this.consistencyProcedure(decreasedEdges);
		this.setInflation(0d);
		return;
	}

	/**
	 * Improves a plan incrementally.
	 */
	protected Trajectory improve(Position origin, Position destination, ZonedDateTime etd) {
		Trajectory trajectory = null;
		while (!this.isInflated()) {
			this.inflate();
			this.addExpandable(this.getStart());
			this.clearExpanded();
			trajectory = this.compute(origin, destination, etd);
		}
		return trajectory;
	}

	protected void updateWaypoint(FADPRMWaypoint waypoint) {
		if (waypoint.getSearch() == this.getCounter())
			return;
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

	public void updateSuccessors(FADPRMWaypoint source) {
		for (FADPRMWaypoint waypoint : source.getSuccessors()) {
			this.updateWaypoint(waypoint);
			this.computeCost(source, waypoint);
			waypoint.setBeta(this.getInflation());
			this.addExpandable(waypoint);
		}
	}

	public void updateDensity(FADPRMWaypoint waypoint) {
		int density = 0;
		for (FADPRMWaypoint neighbor : this.getAllWaypoints()) {
			if (this.getEnvironment().getNormalizedDistance(waypoint, neighbor) < 0.2) {
				density++;
				neighbor.incrementDensity();
			}
		}
		waypoint.setDensity(density);
		return;
	}

	public boolean connectToGoal(FADPRMWaypoint source) {
		if (this.getEnvironment().getNormalizedDistance(source, this.getGoal()) < 0.1) {
			this.getEnvironment().addChild(source, this.getGoal());
			return true;
		}
		return false;
	}

	protected void computeOrImprovePath() {
		while (this.canExpand()) {
			FADPRMWaypoint source = this.pollExpandable();
			if(source.equals(this.getGoal())) {
				System.out.println("goal getcost: " + this.getGoal().getCost());
				this.connectPlan(source);
				return;
			}
			if (this.connectToGoal(source) ) {
				this.computeCostGoal(source, this.getGoal());
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

	protected void rearrangeRoadmap() {
		return;
		// TODO: update edge and waypoint lists according to the last computed
		// trajectory
	}

	protected boolean isFeasible(Trajectory trajectory) {
		if (trajectory == null)
			return false;
		return true;
	}

	protected Trajectory compute(Position origin, Position destination, ZonedDateTime etd) {
		Trajectory trajectory = null;

		while (!this.isFeasible(trajectory)) {
			this.rearrangeRoadmap();
			this.computeOrImprovePath();
			trajectory = this.createTrajectory();
			this.revisePlan(trajectory);
			this.incrementCounter();
			this.addPathCost();
		}
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
		// while (true) {
		// this.updateEdgeCosts();
		// this.improve(origin, destination, etd);
		// }
		trajectory = this.improve(origin, destination, etd);
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
	 * @see PlanningContinuum
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
