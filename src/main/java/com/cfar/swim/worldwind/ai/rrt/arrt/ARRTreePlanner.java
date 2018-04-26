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
package com.cfar.swim.worldwind.ai.rrt.arrt;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.ai.AnytimePlanner;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreePlanner;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreeWaypoint;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes an anytime RRT planner that plans a trajectory of an aircraft in an
 * environment considering a local cost and risk policy. The planner provides
 * various solutions depending on the deliberating time it is given and it
 * continuously improves the cost of each solution by a given factor
 * 
 * @author Manuel Rosa
 *
 */
public class ARRTreePlanner extends RRTreePlanner implements AnytimePlanner {

	// TODO: How to define this values?
	private static final int MAX_SAMPLE_ATTEMPTS = 50;
	private static final int MAX_NEIGHBORS = 5;

	/** the initial relative weight of costs to calculate the cost of a waypoint */
	private double initialCostBias; // Change to final?

	/** the final relative weight of costs to calculate the cost of a waypoint */
	private double finalCostBias; // Change to final?

	/** the improvement factor for the cost of each new generated solution */
	private double improvementFactor; // Change to final?

	/** the relative weight of distances to calculate the cost of a waypoint */
	private double distBias = 1d;

	/** the relative weight of costs to calculate the cost of a waypoint */
	private double costBias = 0d;

	/** the step used to modify the relative weights of distances and costs */
	// TODO: might be interesting to define proportionally to improvementFactor
	private double step = 0.1;

	/** the cost value bounding any new solution to be generated */
	private double costBound = Double.POSITIVE_INFINITY;

	/** the last computed plan */
	private LinkedList<LinkedList<Waypoint>> backupPlans = new LinkedList<>();
	private double[] backupCostBounds;

	/**
	 * Constructs an anytime RRT planner for a specified aircraft and environment
	 * using default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see RRTreePlanner#RRTreePlanner(Aircraft, Environment)
	 */
	public ARRTreePlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}

	/**
	 * Constructs an anytime RRT planner for a specified aircraft and environment
	 * using default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * @param epsilon the maximum distance to extend a waypoint in the tree
	 * @param bias the bias of the sampling algorithm towards goal
	 * @param maxIter the maximum number of sampling iterations
	 * 
	 * @see RRTreePlanner#RRTreePlanner(Aircraft, Environment, double, int, int)
	 */
	public ARRTreePlanner(Aircraft aircraft, Environment environment, double epsilon, int bias, int maxIter) {
		super(aircraft, environment, epsilon, bias, maxIter);
	}

	/**
	 * Gets the minimum quality (initial cost bias) of this ARRT planner.
	 * 
	 * @return the minimum quality (initial cost bias) of this ARRT planner
	 * 
	 * @see AnytimePlanner#getMinimumQuality()
	 */
	@Override
	public double getMinimumQuality() {
		return initialCostBias;
	}

	/**
	 * Sets the minimum quality (initial inflation) of this ARRT planner.
	 * 
	 * @param initialCostBias the minimum quality (initial cost bias) of this ARRT
	 *            planner
	 * 
	 * @see AnytimePlanner#setMinimumQuality(double)
	 */
	@Override
	public void setMinimumQuality(double initialCostBias) {
		this.initialCostBias = initialCostBias;
	}

	/**
	 * Gets the maximum quality (final cost bias) of this ARRT planner.
	 * 
	 * @return the maximum quality (final cost bias) of this ARRT planner
	 * 
	 * @see AnytimePlanner#getMaximumQuality()
	 */
	@Override
	public double getMaximumQuality() {
		return finalCostBias;
	}

	/**
	 * Sets the maximum quality (final cost bias) of this ARRT planner.
	 * 
	 * @param finalCostBias the maximum quality (final cost bias) of this ARRT
	 *            planner
	 * 
	 * @throws IllegalArgumentException if the final inflation is invalid
	 * 
	 * @see AnytimePlanner#setMaximumQuality(double)
	 */
	@Override
	public void setMaximumQuality(double finalCostBias) {
		this.finalCostBias = finalCostBias;
	}

	/**
	 * Gets the quality improvement of this ARRT planner.
	 * 
	 * @return the quality improvement of this ARRT planner
	 * 
	 * @see AnytimePlanner#getQualityImprovement()
	 */
	@Override
	public double getQualityImprovement() {
		return improvementFactor;
	}

	/**
	 * Sets the quality improvement of this ARRT planner.
	 * 
	 * @param improvementFactor the quality improvement of this ARRT planner
	 * 
	 * @see AnytimePlanner#setQualityImprovement(double)
	 */
	@Override
	public void setQualityImprovement(double improvementFactor) {
		this.improvementFactor = improvementFactor;
	}

	/**
	 * Gets the relative weight of distances to calculate the cost of a waypoint
	 * 
	 * @return the the relative weight of distances
	 */
	public double getDistBias() {
		return distBias;
	}

	/**
	 * Sets the relative weight of distances to calculate the cost of a waypoint
	 * 
	 * @param distBias the relative weight of distances to set
	 */
	public void setDistBias(double distBias) {
		this.distBias = distBias;
	}

	/**
	 * Gets the relative weight of costs to calculate the cost of a waypoint
	 * 
	 * @return the relative weight of costs
	 */
	public double getCostBias() {
		return costBias;
	}

	/**
	 * Sets the relative weight of costs to calculate the cost of a waypoint
	 * 
	 * @param costBias the relative weight of costs to set
	 */
	public void setCostBias(double costBias) {
		this.costBias = costBias;
	}

	/**
	 * Gets the step used to modify the relative weights of distances and costs
	 * 
	 * @return the step used to modify the relative weights
	 */
	public double getStep() {
		return step;
	}

	/**
	 * Sets the step used to modify the relative weights of distances and costs
	 * 
	 * @param step the step used to modify the relative weights to set
	 */
	public void setStep(double step) {
		this.step = step;
	}

	/**
	 * Gets the cost value bounding any new solution to be generated
	 * 
	 * @return the bounding cost value
	 */
	public double getCostBound() {
		return costBound;
	}

	/**
	 * Sets the cost value bounding any new solution to be generated
	 * 
	 * @param costBound the bounding cost value to set
	 */
	public void setCostBound(double costBound) {
		this.costBound = costBound;
	}

	// TODO comment
	@SuppressWarnings("unchecked")
	protected Trajectory partialTrajectory(Trajectory trajectory, Waypoint origin, Waypoint destination) {
		if (trajectory.isEmpty())
			return trajectory;

		LinkedList<Waypoint> partialPlan = new LinkedList<>();
		boolean contained = false;

		for (Waypoint waypoint : trajectory.getWaypoints()) {
			if (!contained)
				contained = waypoint == origin;
			if (contained) {
				partialPlan.add(waypoint);
				if (waypoint == destination)
					break;
			}
		}

		return new Trajectory((List<Waypoint>) partialPlan.clone());
	}

	/**
	 * Checks whether or not the current solution can be considered as having the
	 * maximum improvement possible.
	 * 
	 * @return true if improvements reached a maximum, false otherwise
	 */
	protected boolean isImproved(double costOld) {
		double costDiff = (costOld - this.getGoal().getCost()) / costOld;
		System.out.println("CostDif=" + costDiff + "\tCostOld=" + costOld + "\tCostGoal=" + this.getGoal().getCost());
		return costDiff <= 0.05 && getCostBias() >= getMaximumQuality();
	}

	/**
	 * Updates the cost value bounding the next solution to be generated
	 */
	protected void updateCostBound() {
		this.setCostBound((1 - improvementFactor) * getGoal().getCost());
		System.out.println("New cost bound: " + this.getCostBound());
	}

	/**
	 * Updates the relative weights of distances and costs.
	 */
	protected void updateWeights() {
		this.setDistBias(distBias - step < 1 - getMaximumQuality() ? 1 - getMaximumQuality() : distBias - step);
		this.setCostBias(costBias + step > getMaximumQuality() ? getMaximumQuality() : costBias + step);
		System.out.println("CostBias=" + this.getCostBias() + " DistBias=" + this.getDistBias());
	}

	/**
	 * Samples a new waypoint from the environment with a given bias to the goal and
	 * respecting the cost bound value defined for the desired solution.
	 * 
	 * @param bias the percentage value of bias to sample the goal
	 * 
	 * @return waypoint the RRTreeWaypoint sampled
	 */
	protected RRTreeWaypoint sampleTarget(double bias) {
		RRTreeWaypoint waypoint = null;
		int rand = new Random().nextInt(100 - 1) + 1;

		if (rand <= bias) {
			return this.getGoal();
		}

		// Most points sampled from environment will not be inside heuristic region
		// TODO: Should another sampling method be used? Heuristically based sampling?
		double heuristic = Double.POSITIVE_INFINITY;
		int attempts = 0;
		while (heuristic >= this.getCostBound()) {
			if (attempts > MAX_SAMPLE_ATTEMPTS)
				return null;
			waypoint = this.sampleRandom();
			heuristic = this.computeHeuristic(getStart(), waypoint) + this.computeHeuristic(waypoint, getGoal());
			attempts++;
		}

		return waypoint;
	}

	/**
	 * Extends the tree in the direction of the target waypoint, choosing to expand
	 * the waypoint from the neighbors in the tree sorted by increasing selected
	 * cost
	 * 
	 * @param waypointSamp the waypoint set as the goal for extension
	 * 
	 * @return true if an extension was successful and false otherwise
	 */
	@SuppressWarnings("unchecked")
	protected boolean extendToTarget(RRTreeWaypoint waypointSamp) {
		RRTreeWaypoint waypointNew;

		ArrayList<RRTreeWaypoint> neighborsList = (ArrayList<RRTreeWaypoint>) this.getEnvironment()
				.findNearest(waypointSamp, MAX_NEIGHBORS);

		neighborsList = this.sortSelectedCost(neighborsList, waypointSamp);

		for (RRTreeWaypoint waypointNear : neighborsList) {
			// TODO: Review alternative extension strategies
			if (this.newWaypoint(waypointSamp, waypointNear)) {
				waypointNew = this.getWaypointNew();
				this.addEdge(waypointNew);

				waypointNew.setG(computeCost(waypointNew));
				waypointNew.setH(computeHeuristic(waypointNew, getGoal()));

				if (waypointNew.getF() < this.getCostBound()) {
					this.addVertex(waypointNew);
					this.setWaypointNew(waypointNew);
					return true;
				} else {
					// Edge should only stay in tree if previous test is passed
					this.removeEdge(waypointNew);
				}
			}
		}

		return false;
	}

	/**
	 * Selects the cost between two waypoints considering the relative weights of
	 * distance and cost at the moment of the call
	 * 
	 * @param waypoint the new waypoint to the tree towards
	 * @param waypointNear the parent waypoint in the tree
	 * 
	 * @return the cost of the new waypoint with specific relative weights
	 */
	protected double selectCost(RRTreeWaypoint waypoint, RRTreeWaypoint waypointNear) {
		return this.getDistBias() * this.getEnvironment().getDistance(waypoint, waypointNear)
				+ this.getCostBias() * waypoint.getCost();
	}

	/**
	 * Initializes the planner to plan from an origin to a destination at a
	 * specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @see RRTreePlanner#initialize(Position origin, Position destination,
	 *      ZonedDateTime etd)
	 */
	@Override
	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {
		this.clearExpendables();

		this.setGoal(new RRTreeWaypoint(destination));
		this.getGoal().setH(0d);

		RRTreeWaypoint start = new RRTreeWaypoint(origin);
		start.setEto(etd);
		start.setG(0d);
		start.setH(this.computeHeuristic(start, this.getGoal()));

		this.setStart(start);
		this.getWaypointList().add(start);
		this.setWaypointNew(start);
	}

	/**
	 * Computes a plan by growing a tree until the goal is reached
	 * 
	 * @see RRTreePlanner#compute()
	 */
	@Override
	protected boolean compute() {
		boolean status = false;
		for (int i = 0; i < this.getMAX_ITER(); i++) {
			RRTreeWaypoint waypointRand = this.sampleTarget(this.getBIAS());
			if (waypointRand != null) {
				status = this.extendToTarget(waypointRand);
				if (status) {
					if (this.checkGoal()) {
						this.getGoal().setG(this.computeCost(getWaypointNew()));
						this.computePath();
						return true;
					}
				}
			}

		}
		return false;
	}

	/**
	 * Improves a plan incrementally.
	 */
	protected void improve() {
		boolean newPlan;
		double costOld = Double.POSITIVE_INFINITY;
		while (!this.isImproved(costOld)) {
			this.clearExpendables();
			this.addVertex(getStart());
			this.setWaypointNew(getStart());

			costOld = this.getGoal().getCost();
			System.out.println("Computing...");
			newPlan = this.compute();

			this.updateWeights();
			if (newPlan) {
				System.out.println("New plan available");
				this.updateCostBound();

				Trajectory trajectory = this.createTrajectory();
				this.revisePlan(trajectory);
			}
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
	 * @see RRTreePlanner#plan(Position, Position, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		this.initialize(origin, destination, etd);

		this.setCostBias(this.getMinimumQuality());
		this.setDistBias(1 - this.getCostBias());
		this.setCostBound(Double.POSITIVE_INFINITY);

		this.improve();
		System.out.println("Improved!!!!!!!!!!!!!! CostBias=" + this.getCostBias() + " DistBias=" + this.getDistBias()
				+ "\n\n\n\n");

		Trajectory trajectory = this.createTrajectory();
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
	 * @see RRTree#plan(Position, Position, List, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {

		// create first plan by using basic RRT
		// Trajectory trajectory = super.plan(origin, destination, waypoints, etd);
		// return trajectory;
		// System.out.println("First trajectory from basic RRT");
		// if no plan could be found, return an empty trajectory
		// if (trajectory.isEmpty())
		// return trajectory;

		// TODO: Review Implementation

		LinkedList<Waypoint> plan = new LinkedList<>();
		Waypoint currentOrigin;
		ZonedDateTime currentEtd;
		Trajectory trajectory;

		// collect intermediate destinations
		ArrayList<Waypoint> destinations = waypoints.stream().map(Waypoint::new)
				.collect(Collectors.toCollection(ArrayList::new));
		destinations.add(new Waypoint(destination));

		Trajectory part;
		LinkedList<Waypoint> partPlan = new LinkedList<>();
		Waypoint currentDestination;
		double costOld;
		boolean newPlan, newPart, allImproved = false;

		// Initialize backup plans with number of intermediate plans needed
		backupCostBounds = new double[destinations.size()];
		for (int i = 0; i < destinations.size(); i++) {
			backupPlans.add(new LinkedList<Waypoint>());
			backupCostBounds[i] = Double.POSITIVE_INFINITY;
		}
		System.out.println("BackupPlans size=" + backupPlans.size());

		System.out.println("setting initial parametes");
		this.setCostBias(this.getMinimumQuality());
		this.setDistBias(1 - this.getCostBias());
		this.setCostBound(Double.POSITIVE_INFINITY);

		// Create new trajectories until it is considered improved
		while (!allImproved) {
			System.out.println("Entering while cycle");
			// reset boolean values
			allImproved = true;
			newPlan = false;

			currentOrigin = new Waypoint(origin);
			currentEtd = etd;

			// plan and concatenate partial trajectories
			// for (Waypoint currentDestination : destinations) {
			for (int i = 0; i < destinations.size(); i++) {
				System.out.println("Entering for cycle");
				currentDestination = destinations.get(i);

				System.out.println("Initializing");
				this.initialize(currentOrigin, currentDestination, currentEtd);
				this.clearExpendables();
				this.addVertex(getStart());
				this.setWaypointNew(getStart());
				
				// Retrieve last plan for this part
				System.out.println("backingUp");
				this.setCostBound(backupCostBounds[i]);
				partPlan = backupPlans.get(i);
				System.out.println("PartPlan size = "+partPlan.size());
				if (partPlan.isEmpty()) {
					System.out.println("This 121212");
					costOld = Double.POSITIVE_INFINITY;
				}
				else {
					System.out.println("Else 121212");
					costOld = partPlan.get(partPlan.size()-1).getCost();
				}
				System.out.println("Updated cost old = " + costOld);

				
				// newPlan must be true when at least one segment has a new plan
				newPart = this.compute();
				if (newPart) {
					System.out.println("\t\tThissssssssss");
					System.out.println("plansize: "+this.getPlan().size());
					backupPlans.set(i, (LinkedList<Waypoint>) this.getPlan().clone());
					this.updateCostBound();
					backupCostBounds[i]=this.getCostBound();
					part = this.createTrajectory();
					System.out.println("This ---- BackUp("+i+") size = "+backupPlans.get(i).size());
				} else { // TODO: review
					System.out.println("Elseeeeeee");
					part = new Trajectory((List<Waypoint>) partPlan.clone());
				}
				newPlan = newPlan || newPart;

				// allImporved is true only when all segments are improved
				allImproved = allImproved && this.isImproved(costOld);

				// append partial trajectory to plan
				if ((!plan.isEmpty()) && (!part.isEmpty()))
					plan.pollLast();
				for (Waypoint waypoint : part.getWaypoints())
					plan.add(waypoint);

				// set current origin from last waypoint in plan to be used in next iteration
				currentOrigin = plan.peekLast();
				currentEtd = currentOrigin.getEto();
			}

			// update weights to be used in next iteration
			this.updateWeights();

			// if the plan was modified
			if (newPlan) {
				System.out.println("New plan");
				// create trajectory from plan
				trajectory = new Trajectory(plan);
				this.revisePlan(trajectory);
			}
		}

		System.out.println("Exiting...");
		trajectory = this.createTrajectory();
		this.revisePlan(trajectory);
		return trajectory;

	}

	/**
	 * Sorts a list of elements by increasing selected cost to a given waypoint
	 * 
	 * @param list the list with the elements to be sorted by selected cost
	 * @param waypoint the waypoint to be considered as reference for the selected
	 *            cost
	 * 
	 * @return a new array list with its elements sorted by selected cost
	 */
	public ArrayList<RRTreeWaypoint> sortSelectedCost(ArrayList<RRTreeWaypoint> list, RRTreeWaypoint waypoint) {

		return list.stream()
				.sorted((w1, w2) -> Double.compare(this.selectCost(waypoint, w1), this.selectCost(waypoint, w2)))
				.collect(Collectors.toCollection(ArrayList::new));

	}
}
