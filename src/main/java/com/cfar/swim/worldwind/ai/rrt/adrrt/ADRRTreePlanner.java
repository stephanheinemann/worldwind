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
package com.cfar.swim.worldwind.ai.rrt.adrrt;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Random;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.ai.AnytimePlanner;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Extension;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreePlanner;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreeWaypoint;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Strategy;
import com.cfar.swim.worldwind.ai.rrt.drrt.DRRTreeWaypoint;
import com.cfar.swim.worldwind.ai.rrt.drrt.DRRTreePlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Edge;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.render.Obstacle;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes an anytime dynamic RRT planner that plans a trajectory of an
 * aircraft in an environment considering a local cost and risk policy. The
 * planner provides an initial plan and then continuously checks if new
 * obstacles are added. When an obstacle affects the current plan, a new one is
 * computed reusing information. The plans provided depend on the deliberating
 * time given and and are continuously improved to reduce the cost of each
 * solution by a given factor
 * 
 * @author Manuel Rosa
 *
 */
public class ADRRTreePlanner extends DRRTreePlanner implements AnytimePlanner {

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
	private double step = 0.2d;

	/** the cost value bounding any new solution to be generated */
	private double costBound = Double.POSITIVE_INFINITY;

	/**
	 * Constructs an anytime dynamic RRT planner for a specified aircraft and
	 * environment using default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see DRRTreePlanner#DRRTreePlanner(Aircraft, Environment)
	 */
	public ADRRTreePlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}

	/**
	 * Constructs an anytime dynamic RRT planner for a specified aircraft and
	 * environment using default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * @param epsilon the maximum distance to extend a waypoint in the tree
	 * @param bias the bias of the sampling algorithm towards goal
	 * @param maxIter the maximum number of sampling iterations
	 * @param strategy the expanding strategy for the planner
	 * @param extension the extension technique for the planner
	 * 
	 * @see DRRTreePlanner#DRRTreePlanner(Aircraft, Environment, double, int, int, Strategy, Extension)
	 */
	public ADRRTreePlanner(Aircraft aircraft, Environment environment, double epsilon, int bias, int maxIter,
			Strategy strategy, Extension extension) {
		super(aircraft, environment, epsilon, bias, maxIter, strategy, extension);
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

	/**
	 * Selects the cost between two waypoints considering the relative weights of
	 * distance and cost at the moment of the call
	 * 
	 * @param waypoint the new waypoint to the tree towards
	 * @param waypointNear the parent waypoint in the tree
	 * 
	 * @return the cost of the new waypoint with specific relative weights
	 */
	protected double selectCost(DRRTreeWaypoint waypoint, DRRTreeWaypoint waypointNear) {
		return this.getDistBias() * this.getEnvironment().getDistance(waypoint, waypointNear)
				+ this.getCostBias() * waypoint.getCost();
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
	protected ArrayList<DRRTreeWaypoint> sortSelectedCost(ArrayList<DRRTreeWaypoint> list, DRRTreeWaypoint waypoint) {
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
		// double costDiff = (costOld - this.getGoal().getCost()) / costOld;
		// return costDiff <= 0.05 && getCostBias() >= getMaximumQuality();
		return false;
	}

	protected boolean isImproved(double costOld, double costNew) {
		double costDiff = (costOld - costNew) / costOld;
		return costDiff <= 0.05 && getCostBias() >= getMaximumQuality();
	}

	/**
	 * Updates the cost value bounding the next solution to be generated
	 */
	protected void updateCostBound() {
		this.setCostBound((1 - improvementFactor) * getGoal().getCost());
	}

	/**
	 * Updates the relative weights of distances and costs.
	 */
	protected void updateWeights() {
		this.updateWeights(step);
	}

	protected void updateWeights(double step) {
		this.setDistBias(distBias - step < 1 - getMaximumQuality() ? 1 - getMaximumQuality() : distBias - step);
		this.setCostBias(costBias + step > getMaximumQuality() ? getMaximumQuality() : costBias + step);
	}

	/**
	 * Tests whether the dynamic changes in the obstacles are significant or not.
	 * 
	 * @return true if changes in obstacles are significant, false otherwise
	 */
	protected boolean areSignificant() {
		// TODO: How to classify changes as being significant
		return true;
	}

	/**
	 * Saves the current data in the planner and environment to an anytime dynamic
	 * tree.
	 * 
	 * @param tree the tree where to save the current data
	 */
	@SuppressWarnings("unchecked")
	protected void saveToTree(ADTree tree) {
		tree.setWaypointList((ArrayList<RRTreeWaypoint>) ((ArrayList<RRTreeWaypoint>) this.getWaypointList()).clone());
		tree.setEdgeList((ArrayList<Edge>) ((ArrayList<Edge>) this.getEdgeList()).clone());
		tree.setPlan((LinkedList<Waypoint>) this.getPlan().clone());
		tree.setDistBias(this.getDistBias());
		tree.setCostBias(this.getCostBias());
		tree.setCostBound(this.getCostBound());
	}

	/**
	 * Loads the data from an anytime dynamic tree to the current planner.
	 * 
	 * @param tree the tree from where to load the data
	 */
	protected void loadFromTree(ADTree tree) {
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
	 * @return waypoint the DRRTreeWaypoint sampled
	 */
	protected DRRTreeWaypoint sampleTarget(double bias) {
		DRRTreeWaypoint waypoint = null;
		int rand = new Random().nextInt(100 - 1) + 1;

		if (rand <= bias)
			return this.getGoal();

		double heuristic = Double.POSITIVE_INFINITY;
		int attempts = 0;
		while (heuristic >= this.getCostBound()) {
			if (attempts > MAX_SAMPLE_ATTEMPTS)
				return null;
			waypoint = (DRRTreeWaypoint) this.sampleRandom();
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
	protected boolean extendToTarget(DRRTreeWaypoint waypointSamp) {
		DRRTreeWaypoint waypointNew;

		ArrayList<DRRTreeWaypoint> neighborsList = (ArrayList<DRRTreeWaypoint>) this.getEnvironment()
				.findNearest(waypointSamp, MAX_NEIGHBORS);

		neighborsList = this.sortSelectedCost(neighborsList, waypointSamp);

		for (DRRTreeWaypoint waypointNear : neighborsList) {
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
			DRRTreeWaypoint waypointRand = this.sampleTarget(this.getBIAS());
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
	 * @see DRRTreePlanner#plan(Position, Position, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		this.initialize(origin, destination, etd);

		this.setCostBias(this.getMinimumQuality());
		this.setDistBias(1 - this.getCostBias());
		this.setCostBound(Double.POSITIVE_INFINITY);

		Trajectory trajectory = new Trajectory();

		// Anytime Dynamic variables
		boolean newPlan;
		double costOld = Double.POSITIVE_INFINITY;
		HashSet<Obstacle> diffObstacles = new HashSet<>();

		// Auxiliary Trees
		ADTree treeR = new ADTree(), treeT = new ADTree();
		
		treeT.setCostBias(this.getMinimumQuality());
		treeT.setDistBias(1 - this.getCostBias());
		treeT.setCostBound(Double.POSITIVE_INFINITY);
		
		while (!this.checkGoal(getStart())) {
			// Anytime behavior
			if (!this.isImproved(costOld)) {
				System.out.println("\n\nImproving...");

				this.loadFromTree(treeT);
				
				this.clearExpendables();
				this.addVertex(getStart());
				this.setWaypointNew(getStart());
				
				System.out.println("Set Costs to tree T: db=" + this.getDistBias() + " cb=" + this.getCostBias() + "Cs="
						+ this.getCostBound());

				costOld = this.getGoal().getCost();
				newPlan = this.compute();
				
				this.saveToTree(treeT);

				if (newPlan) {
					System.out.println("NEW PLAN found!!");
					trajectory = this.createTrajectory();
					this.revisePlan(trajectory);

					treeR.clone(treeT);

					this.updateWeights();
					this.updateCostBound();
					treeT.setBiases(this.getDistBias(), this.getCostBias());
					treeT.setCostBound(this.getCostBound());
					
				} else {
					System.out.println("No new plan...");
					this.updateWeights();
					treeT.setBiases(this.getDistBias(), this.getCostBias());
				}

			} else {
				System.out.println("Improved");
			}

			this.loadFromTree(treeR);

			// Dynamic behavior
			this.updateObstacles();
			if (!this.getDiffObstacles().isEmpty()) {

				// Invalidate Waypoints affected by new obstacles
				System.out.println("Invalidating obstacles");
				for (Obstacle obstacle : diffObstacles) {
					System.out.println("Obstacle in diffObstacle");
					this.invalidateWaypoints(obstacle);
				}

				// If changes were significant adjust weights
				if (this.areSignificant()) {
					System.out.println("Significant changes ocurred");
					// TODO: Define how much should weights be changed
					this.updateWeights(-this.getStep());
					treeT.setBiases(this.getDistBias(), this.getCostBias());
					
					// TODO: Define how to increase cost bound to find solution
					this.setCostBound(10 * getCostBound());
				}

				// Check if the current path is still valid
				if (!this.isPathValid()) {
					System.out.println("Non-valid path, regrowing...");
					this.repair();
					this.updateCostBound();
					treeT.setCostBound(this.getCostBound());
				}
			}

			// Print Plan
			System.out.println("Current Path #" + this.getPlan().size());
			for (Waypoint wpt : this.getPlan())
				System.out.println(wpt + " Cost=" + wpt.getCost());
			
			// Update trajectory reflecting the modified plan
			trajectory = this.createTrajectory();
			this.revisePlan(trajectory);

			//Move to next and save main tree
			System.out.println("Moving to next");
			this.moveToNext();
			this.saveToTree(treeR);
		}

		return trajectory;
	}
}
