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
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Extension;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreePlanner;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreeWaypoint;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Sampling;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Strategy;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Edge;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes an anytime RRT planner that plans a trajectory of an aircraft in an
 * environment considering a local cost and risk policy. The planner provides
 * various solutions depending on the deliberating time it is given and it
 * continuously improves the cost of each solution by a given factor.
 * 
 * @author Manuel Rosa
 *
 */
public class ARRTreePlanner extends RRTreePlanner implements AnytimePlanner {

	// TODO: How to define this values?
	/** the maximum number of sampling attempts per iteration of uniform sampling */
	private final int MAX_SAMPLE_ATTEMPTS = 50;

	/** the maximum number of neighbors to be considered for expansion */
	private final int MAX_NEIGHBORS = 5;

	/** the sampling technique for the planner */
	private final Sampling SAMPLING;

	/** the initial relative weight of costs to calculate the cost of a waypoint */
	private double initialCostBias = 0d;

	/** the final relative weight of costs to calculate the cost of a waypoint */
	private double finalCostBias = 1d;

	/** the improvement factor for the cost of each new generated solution */
	private double improvementFactor;

	/** the relative weight of distances to calculate the cost of a waypoint */
	private double distBias = 1d;

	/** the relative weight of costs to calculate the cost of a waypoint */
	private double costBias = 0d;

	/** the step used to modify the relative weights of distances and costs */
	// TODO: should be defined proportionally to improvementFactor/initialBias
	private double step = 0.2d; // step =(finalCostBias -initialCostBias)/4

	/** the cost value bounding any new solution to be generated */
	private double costBound = Double.POSITIVE_INFINITY;

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
		// this.online = false;
		// this.positionThreshold = 2d;
		SAMPLING = Sampling.UNIFORM;
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
	 * @param strategy the expanding strategy for the planner
	 * @param extension the extension technique for the planner
	 * @param sampling the sampling technique for the planner
	 * 
	 * @see RRTreePlanner#RRTreePlanner(Aircraft, Environment, double, int, int)
	 */
	public ARRTreePlanner(Aircraft aircraft, Environment environment, double epsilon, int bias, int maxIter,
			Strategy strategy, Extension extension, Sampling sampling) {
		super(aircraft, environment, epsilon, bias, maxIter, strategy, extension);
		SAMPLING = sampling;
	}

	/**
	 * Gets the sampling technique for the planner.
	 * 
	 * @return the sampling technique for the planner
	 */
	public Sampling getSAMPLING() {
		return SAMPLING;
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
		this.step = (this.finalCostBias - this.initialCostBias) / 4d;
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
		this.step = (this.finalCostBias - this.initialCostBias) / 4d;
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
	 * Gets the relative weight of distances to calculate the cost of a waypoint.
	 * 
	 * @return the the relative weight of distances
	 */
	public double getDistBias() {
		return distBias;
	}

	/**
	 * Sets the relative weight of distances to calculate the cost of a waypoint.
	 * 
	 * @param distBias the relative weight of distances to set
	 */
	public void setDistBias(double distBias) {
		this.distBias = distBias;
	}

	/**
	 * Gets the relative weight of costs to calculate the cost of a waypoint.
	 * 
	 * @return the relative weight of costs
	 */
	public double getCostBias() {
		return costBias;
	}

	/**
	 * Sets the relative weight of costs to calculate the cost of a waypoint.
	 * 
	 * @param costBias the relative weight of costs to set
	 */
	public void setCostBias(double costBias) {
		this.costBias = costBias;
	}

	/**
	 * Gets the step used to modify the relative weights of distances and costs.
	 * 
	 * @return the step used to modify the relative weights
	 */
	public double getStep() {
		return step;
	}

	/**
	 * Sets the step used to modify the relative weights of distances and costs.
	 * 
	 * @param step the step used to modify the relative weights to set
	 */
	public void setStep(double step) {
		this.step = step;
	}

	/**
	 * Gets the cost value bounding any new solution to be generated.
	 * 
	 * @return the bounding cost value
	 */
	public double getCostBound() {
		return costBound;
	}

	/**
	 * Sets the cost value bounding any new solution to be generated.
	 * 
	 * @param costBound the bounding cost value to set
	 */
	public void setCostBound(double costBound) {
		this.costBound = costBound;
	}

	/**
	 * Gets the maximum number of sampling attempts per iteration of uniform
	 * sampling.
	 * 
	 * @return the maximum number of sampling attempts
	 */
	protected int getMaxSampleAttempts() {
		return MAX_SAMPLE_ATTEMPTS;
	}

	/**
	 * Gets the maximum number of neighbors to be considered for expansion.
	 * 
	 * @return the maximum number of neighbors
	 */
	protected int getMaxNeighbors() {
		return MAX_NEIGHBORS;
	}

	/**
	 * Selects the cost between two waypoints considering the relative weights of
	 * distance and cost at the moment of the call.
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
	 * Sorts a list of elements by increasing selected cost to a given waypoint.
	 * 
	 * @param list the list with the elements to be sorted by selected cost
	 * @param waypoint the waypoint to be considered as reference for the selected
	 *            cost
	 * 
	 * @return a new array list with its elements sorted by selected cost
	 */
	protected ArrayList<RRTreeWaypoint> sortSelectedCost(ArrayList<RRTreeWaypoint> list, RRTreeWaypoint waypoint) {

		return list.stream()
				.sorted((w1, w2) -> Double.compare(this.selectCost(waypoint, w1), this.selectCost(waypoint, w2)))
				.collect(Collectors.toCollection(ArrayList::new));

	}

	/**
	 * Checks whether or not the current solution can be considered as having the
	 * maximum improvement possible.
	 * 
	 * @return true if improvements reached a maximum, false otherwise
	 */
	protected boolean isImproved(double costOld) {
		return this.isImproved(costOld, getGoal().getCost());
	}

	protected boolean isImproved(double costOld, double costNew) {
		double costDiff = (costOld - costNew) / costOld;
		return costDiff <= getQualityImprovement() && getCostBias() >= getMaximumQuality();
	}

	/**
	 * Updates the cost value bounding the next solution to be generated.
	 */
	protected void updateCostBound() {
		this.setCostBound((1 - improvementFactor) * getGoal().getCost());
	}

	/**
	 * Updates the relative weights of distances and costs with default step.
	 */
	protected void updateWeights() {
		this.updateWeights(getStep());
	}

	/**
	 * Updates the relative weights of distances and costs with a certain value.
	 */
	protected void updateWeights(double step) {
		double costBias = this.getCostBias() + step;
		costBias = costBias > this.getMaximumQuality() ? this.getMaximumQuality() : costBias;
		costBias = costBias < this.getMinimumQuality() ? this.getMinimumQuality() : costBias;

		this.setCostBias(costBias);
		this.setDistBias(1 - costBias);
	}

	/**
	 * Saves the current data in the planner and environment to an anytime tree.
	 * 
	 * @param tree the tree where to save the current data
	 */
	@SuppressWarnings("unchecked")
	protected void saveToTree(ARRTree tree) {
		tree.setWaypointList((ArrayList<RRTreeWaypoint>) ((ArrayList<RRTreeWaypoint>) this.getWaypointList()).clone());
		tree.setEdgeList((ArrayList<Edge>) ((ArrayList<Edge>) this.getEdgeList()).clone());
		tree.setPlan((LinkedList<Waypoint>) this.getPlan().clone());
		tree.setDistBias(this.getDistBias());
		tree.setCostBias(this.getCostBias());
		tree.setCostBound(this.getCostBound());
	}

	/**
	 * Loads the data from an anytime tree to the current planner.
	 * 
	 * @param tree the tree from where to load the data
	 */
	protected void loadFromTree(ARRTree tree) {
		this.setWaypointList(tree.getWaypointList());
		this.setEdgeList(tree.getEdgeList());
		this.setPlan(tree.getPlan());
		this.setDistBias(tree.getDistBias());
		this.setCostBias(tree.getCostBias());
		this.setCostBound(tree.getCostBound());
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
		double heuristic = Double.POSITIVE_INFINITY;
		int rand = new Random().nextInt(100 - 1) + 1;

		if (rand <= bias)
			return this.getGoal();

		switch (this.getSAMPLING()) {
		// Ellipsoidal sampling
		case ELLIPSOIDAL:
			double dist = getEnvironment().getDistance(getStart(), getGoal()) > getEnvironment().getDistance(getCostBound())
					? getEnvironment().getDistance(getStart(), getGoal())
					: getEnvironment().getDistance(getCostBound());
			waypoint = this.sampleEllipsoid(getStart(), getGoal(), dist);
			// Uniform sampling
		case UNIFORM:
		default:
			for (int attempts = 0; heuristic >= getCostBound(); attempts++) {
				if (attempts > MAX_SAMPLE_ATTEMPTS)
					return null;
				waypoint = this.sampleRandom();
				heuristic = computeHeuristic(getStart(), waypoint) + computeHeuristic(waypoint, getGoal());
			}
		}

		return waypoint;
	}

	/**
	 * Creates a new waypoint from a random position sampled from inside an
	 * ellipsoid defined in the environment space.
	 *
	 * @return waypoint the RRTreeWaypoint sampled
	 */
	protected RRTreeWaypoint sampleEllipsoid(Position focusA, Position focusB, double distance) {
		return this.createWaypoint(this.getEnvironment().samplePositionEllipsoid(focusA, focusB, distance));
	}

	/**
	 * Extends the tree in the direction of the target waypoint, choosing to expand
	 * the waypoint from the neighbors in the tree sorted by increasing selected
	 * cost.
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
		super.initialize(origin, destination, etd);

		this.getGoal().setH(0d);
		this.getStart().setH(this.computeHeuristic(getStart(), getGoal()));
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

		Trajectory trajectory = new Trajectory();

		// Anytime variables
		boolean newPlan;
		double costOld = Double.POSITIVE_INFINITY;

		// Auxiliary Trees
		ARRTree treeR = new ARRTree(), treeT = new ARRTree();
		treeT.setCostBias(this.getMinimumQuality());
		treeT.setDistBias(1 - this.getCostBias());
		treeT.setCostBound(Double.POSITIVE_INFINITY);

		do {
			this.clearExpendables();
			this.addVertex(getStart());
			this.setWaypointNew(getStart());

			costOld = this.getGoal().getCost();
			newPlan = this.compute();

			this.saveToTree(treeT);

			if (newPlan) {
				trajectory = this.createTrajectory();
				this.revisePlan(trajectory);

				treeR.clone(treeT);

				this.updateWeights();
				this.updateCostBound();
				treeT.setBiases(this.getDistBias(), this.getCostBias());
				treeT.setCostBound(this.getCostBound());

			} else {
				this.updateWeights();
				treeT.setBiases(this.getDistBias(), this.getCostBias());
			}

		} while ((!isImproved(costOld)));

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

		// Multiple-waypoint-plan variables
		LinkedList<Waypoint> plan = new LinkedList<>();
		Waypoint currentOrigin, currentDestination;
		ZonedDateTime currentEtd;
		Trajectory trajectory;

		// collect intermediate destinations
		ArrayList<Waypoint> destinations = waypoints.stream().map(Waypoint::new)
				.collect(Collectors.toCollection(ArrayList::new));
		destinations.add(new Waypoint(destination));

		// Auxiliary Trees
		ARRTree[] treeR = new ARRTree[destinations.size()], treeT = new ARRTree[destinations.size()];
		for (int i = 0; i < destinations.size(); i++) {
			treeR[i] = new ARRTree();
			treeT[i] = new ARRTree();
			treeT[i].setCostBias(this.getMinimumQuality());
			treeT[i].setDistBias(1 - this.getCostBias());
			treeT[i].setCostBound(Double.POSITIVE_INFINITY);
		}

		// Anytime variables
		Trajectory part;
		boolean newPlan, newPart, allImproved = false;
		double costOld = Double.POSITIVE_INFINITY, costNew = Double.POSITIVE_INFINITY;

		this.setCostBias(this.getMinimumQuality());
		this.setDistBias(1 - this.getCostBias());

		// Create new trajectories until all paths are considered improved
		while (!allImproved) {
			System.out.println("Outer loop");
			// reset boolean values
			allImproved = true;
			newPlan = false;

			currentOrigin = new Waypoint(origin);
			currentEtd = etd;
			plan.clear();

			// plan and concatenate partial trajectories
			for (int i = 0; i < destinations.size(); i++) {
				this.loadFromTree(treeT[i]);
				if (treeR[i].getPlan().isEmpty())
					costOld = Double.POSITIVE_INFINITY;
				else
					costOld = treeR[i].getPlan().peekLast().getCost();

				currentDestination = destinations.get(i);
				this.initialize(currentOrigin, currentDestination, currentEtd);

				newPart = this.compute();

				this.saveToTree(treeT[i]);

				if (newPart) {
					treeR[i].clone(treeT[i]);

					this.updateWeights();
					this.updateCostBound();
					treeT[i].setBiases(this.getDistBias(), this.getCostBias());
					treeT[i].setCostBound(this.getCostBound());

				} else {
					this.updateWeights();
					treeT[i].setBiases(this.getDistBias(), this.getCostBias());
				}

				// newPlan must be true when at least one segment has a new plan
				newPlan = newPlan || newPart;

				// allImporved is true only when all segments are improved
				costNew = treeR[i].getPlan().peekLast().getCost();
				allImproved = allImproved && this.isImproved(costOld, costNew);

				part = this.createTrajectory(treeR[i].getPlan());
				// append partial trajectory to plan
				if ((!plan.isEmpty()) && (!part.isEmpty())) {
					part.pollFirst();
				}
				for (Waypoint waypoint : part.getWaypoints()) {
					plan.add(waypoint);
				}

				// set current origin from last waypoint in plan to be used in next iteration
				currentOrigin = plan.peekLast();
				currentEtd = currentOrigin.getEto();
			}

			// if the plan was modified
			if (newPlan) {
				// create trajectory from plan
				trajectory = this.createTrajectory(plan);
				this.revisePlan(trajectory);
			}
		}

		trajectory = this.createTrajectory(plan);
		this.revisePlan(trajectory);
		return trajectory;

	}

}
