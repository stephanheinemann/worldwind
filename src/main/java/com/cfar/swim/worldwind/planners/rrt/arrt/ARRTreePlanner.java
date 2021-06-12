/**
 * Copyright (c) 2021, Stephan Heinemann (UVic Center for Aerospace Research)
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
package com.cfar.swim.worldwind.planners.rrt.arrt;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.function.ToDoubleFunction;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.environments.Edge;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.planners.AnytimePlanner;
import com.cfar.swim.worldwind.planners.rrt.Status;
import com.cfar.swim.worldwind.planners.rrt.brrt.RRTreePlanner;
import com.cfar.swim.worldwind.planners.rrt.brrt.RRTreeWaypoint;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.util.Logging;

/**
 * Realizes an anytime RRT planner that plans a trajectory of an aircraft in an
 * environment considering a local cost and risk policy. The planner provides
 * various solutions depending on the deliberating time it is given and it
 * continuously improves the cost of each solution by a given factor.
 * 
 * @author Manuel Rosa
 * @author Stephan Heinemann
 *
 */
public class ARRTreePlanner extends RRTreePlanner implements AnytimePlanner {

	// least distance (distance bias) to least cost (cost bias)
	// sampling: q_target :- h(q_start, q_target) + h(q_target, q_goal) < cost bound
	// selection: k-nearest: priority order (1 - improvement factor) cost bound,  distance bias -= dd, cost bias += dc
	// expansion: fast attempts versus large sets
	// sample attempts for target could increase with increasing cost bias (function of cost bias)
	
	/** the limit of neighbors to consider for extension  */
	private int neighborLimit = 5; // [1, Integer.MAX_VALUE]
	
	/** the initial relative weight of costs to calculate the cost of a waypoint */
	private double initialCostBias = 0d;
	
	/** the final relative weight of costs to calculate the cost of a waypoint */
	private double finalCostBias = 1d;
	
	/** the improvement factor for the cost of each new generated solution */
	private double improvementFactor = 0.05d;
	
	/** the relative weight of distances to calculate the cost of a waypoint */
	private double distanceBias = 1d;
	
	/** the relative weight of costs to calculate the cost of a waypoint */
	private double costBias = 0d;
	
	/** the step used to modify the relative weights of distances and costs */
	// TODO: separate attribute (improvement factor not sufficient)?
	// TODO: should be defined proportionally to improvementFactor/initialBias
	private double step = 0.2d; // step =(finalCostBias -initialCostBias)/4
	
	/** the cost value bounding any new solution to be generated */
	private double costBound = Double.POSITIVE_INFINITY;
	
	/** the maximum number of sampling attempts per iteration of uniform sampling */
	// TODO: review calculation depending on cost bound
	private int maxSamplingAttempts = 50;
	
	/**
	 * Constructs an ARRT planner for a specified aircraft and environment
	 * using default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see RRTreePlanner#RRTreePlanner(Aircraft, Environment)
	 */
	public ARRTreePlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		// TODO: review default setup
	}
	
	/**
	 * Gets the start ARRT waypoint of this ARRT planner.
	 * 
	 * @return the start ARRT waypoint of this ARRT planner
	 * 
	 * @see RRTreePlanner#getStart()
	 */
	@Override
	public ARRTreeWaypoint getStart() {
		return (ARRTreeWaypoint) super.getStart();
	}
	
	/**
	 * Gets the goal ARRT waypoint of this ARRT planner.
	 * 
	 * @return the goal ARRT waypoint of this ARRT planner
	 * 
	 * @see RRTreePlanner#getGoal()
	 */
	@Override
	public ARRTreeWaypoint getGoal() {
		return (ARRTreeWaypoint) super.getGoal();
	}
	
	/**
	 * Gets the newest ARRT waypoint added to the tree.
	 * 
	 * @return the newest ARRT waypoint added to the tree
	 * 
	 * @see RRTreePlanner#getNewestWaypoint()
	 */
	@Override
	public ARRTreeWaypoint getNewestWaypoint() {
		return (ARRTreeWaypoint) super.getNewestWaypoint();
	}
	
	/**
	 * Creates an ARRT waypoint at a specified position.
	 * 
	 * @param position the position
	 * 
	 * @return the ARRT waypoint at the specified position
	 */
	@Override
	protected ARRTreeWaypoint createWaypoint(Position position) {
		return new ARRTreeWaypoint(position);
	}
	
	/**
	 * Gets the limit of neighbors to consider for extension by this ARRT
	 * planner.
	 * 
	 * @return the limit neighbors to consider for extension by this ARRT
	 *         planner
	 */
	public int getNeighborLimit() {
		return this.neighborLimit;
	}
	
	/**
	 * Sets the limit of neighbors to consider for extension by this ARRT
	 * planner.
	 * 
	 * @param neighborLimit the limit of neighbors to be set
	 * 
	 * @throws IllegalArgumentException if neighbor limit is invalid
	 */
	public void setNeighborLimit(int neighborLimit) {
		if ((0 < neighborLimit) && (Integer.MAX_VALUE >= neighborLimit)) {
			this.neighborLimit = neighborLimit;
		} else {
			throw new IllegalArgumentException("neighbor limit is invalid");
		}
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
		return this.initialCostBias;
	}
	
	/**
	 * Sets the minimum quality (initial cost bias) of this ARRT planner.
	 * 
	 * @param initialCostBias the minimum quality (initial cost bias) of this
	 *                        ARRT planner
	 * 
	 * @throws IllegalArgumentException if the initial cost bias is invalid
	 * 
	 * @see AnytimePlanner#setMinimumQuality(double)
	 */
	@Override
	public void setMinimumQuality(double initialCostBias) {
		if ((0d <= initialCostBias) && (this.getMaximumQuality() >= initialCostBias)) {
			this.initialCostBias = initialCostBias;
			// TODO: review step calculation
			this.step = (this.finalCostBias - this.initialCostBias) / 4d;
		} else {
			throw new IllegalArgumentException("initial cost bias is invalid");
		}
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
		return this.finalCostBias;
	}
	
	/**
	 * Sets the maximum quality (final cost bias) of this ARRT planner.
	 * 
	 * @param finalCostBias the maximum quality (final cost bias) of this ARRT
	 *                      planner
	 * 
	 * @throws IllegalArgumentException if the final inflation is invalid
	 * 
	 * @see AnytimePlanner#setMaximumQuality(double)
	 */
	@Override
	public void setMaximumQuality(double finalCostBias) {
		if ((this.getMinimumQuality() <= finalCostBias) && (1d >= finalCostBias)) {
			this.finalCostBias = finalCostBias;
			// TODO: review step calculation
			this.step = (this.finalCostBias - this.initialCostBias) / 4d;
		} else {
			throw new IllegalArgumentException("final cost bias is invalid");
		}
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
		return this.improvementFactor;
	}
	
	/**
	 * Sets the quality improvement of this ARRT planner.
	 * 
	 * @param improvementFactor the quality improvement of this ARRT planner
	 * 
	 * @throws IllegalArgumentException if the improvement factor is invalid
	 * 
	 * @see AnytimePlanner#setQualityImprovement(double)
	 */
	@Override
	public void setQualityImprovement(double improvementFactor) {
		if ((0d <= improvementFactor) && (1d >= improvementFactor)) {
			this.improvementFactor = improvementFactor;
		} else {
			throw new IllegalArgumentException("improvement factor is invalid");
		}
	}
	
	/**
	 * Gets the distance bias of this ARRT planner.
	 * 
	 * @return the distance bias of this ARRT planner
	 */
	protected double getDistanceBias() {
		return this.distanceBias;
	}
	
	/**
	 * Sets the distance bias of this ARRT planner.
	 * 
	 * @param distanceBias the distance bias to be set
	 */
	protected void setDistanceBias(double distanceBias) {
		this.distanceBias = distanceBias;
	}
	
	/**
	 * Gets the cost bias of this ARRT planner.
	 * 
	 * @return the cost bias of this ARRT planner
	 */
	protected double getCostBias() {
		return this.costBias;
	}
	
	/**
	 * Sets the cost bias of this ARRT planner.
	 * 
	 * @param costBias the cost bias to be set
	 */
	protected void setCostBias(double costBias) {
		this.costBias = costBias;
	}
	
	/**
	 * Gets the step used to modify the distance and cost biases.
	 * 
	 * @return the step used to modify the distance and cost biases
	 */
	protected double getStep() {
		return this.step;
	}
	
	/**
	 * Sets the step used to modify the distance and cost biases.
	 * 
	 * @param step the step used to modify the distance and cost biases
	 */
	protected void setStep(double step) {
		this.step = step;
	}
	
	/**
	 * Gets the cost bound of trajectories planned by this ARRT planner.
	 * 
	 * @return the cost bound of trajectories planned by this ARRT planner
	 */
	protected double getCostBound() {
		return this.costBound;
	}
	
	/**
	 * Sets the cost bound of trajectories planned by this ARRT planner.
	 * 
	 * @param costBound the cost bound of trajectories to be set
	 */
	protected void setCostBound(double costBound) {
		this.costBound = costBound;
	}
	
	/**
	 * Gets the maximum number of sampling attempts per sampling iteration of
	 * this ARRT planner.
	 * 
	 * @return the maximum number of sampling attempts per sampling iteration
	 *         of this ARRT planner
	 */
	protected int getMaxSamplingAttempts() {
		return this.maxSamplingAttempts;
	}
	
	/**
	 * Sets the maximum number of sampling attempts per sampling iteration of
	 * this ARRT planner.
	 * 
	 * @param maxSamplingAttempts the maximum number of sampling attempts to be
	 *                            set
	 */
	protected void setMaxSamplingAttempts(int maxSamplingAttempts) {
		this.maxSamplingAttempts = maxSamplingAttempts;
	}
	
	/**
	 * Computes the estimated remaining cost (heuristic) of a specified source
	 * waypoint to reach a target ARRT waypoint.
	 * 
	 * @param source the source waypoint in globe coordinates
	 * @param target the target waypoint in globe coordinates
	 * 
	 * @return the estimated remaining cost (heuristic)
	 */
	protected double computeHeuristic(
			ARRTreeWaypoint source, ARRTreeWaypoint target) {
		return this.getEnvironment().getNormalizedDistance(source, target);
	}
	
	/**
	 * Computes the composite heuristic of a waypoint as the sum of the simple
	 * heuristic from the start to the waypoint and the simple heuristic from
	 * the waypoint to the goal (heuristic path cost containing the waypoint).
	 * 
	 * @param waypoint the waypoint for which to compute the composite heuristic
	 * 
	 * @return the composite heuristic of the waypoint
	 */
	protected double computeCompositeHeuristic(ARRTreeWaypoint waypoint) {
		return this.computeHeuristic(this.getStart(), waypoint)
		+ this.computeHeuristic(waypoint, this.getGoal());
	}
	
	/**
	 * Computes the selection cost of a source waypoint in relation to a target
	 * waypoint to extend to considering the current distance and cost weights.
	 * 
	 * @param source the source waypoint to extend from
	 * @param target the target waypoint to extend to
	 * 
	 * @return the selection cost of the source waypoint
	 */
	protected double computeSelectionCost(ARRTreeWaypoint source, ARRTreeWaypoint target) {
		return this.getDistanceBias() * this.computeHeuristic(source, target)
				+ this.getCostBias() * source.getCost();
	}
	
	/**
	 * Realizes a cost selector to establish a priority order of source
	 * waypoints with respect to their selection cost for an extension towards
	 * a target waypoint.
	 * 
	 * @author Stephan Heinemann
	 *
	 */
	protected class CostSelector implements ToDoubleFunction<ARRTreeWaypoint> {
		
		ARRTreeWaypoint target;
		
		/**
		 * Constructs a new cost selector for a target waypoint.
		 * 
		 * @param target the target waypoint
		 */
		public CostSelector(ARRTreeWaypoint target) {
			this.target = target;
		}
		
		/**
		 * Applies this cost selector to a source waypoint in order to
		 * establish its priority order for extension to the target waypoint of
		 * this cost selector.
		 * 
		 * @param source the source waypoint to extend from
		 */
		@Override
		public double applyAsDouble(ARRTreeWaypoint source) {
			return computeSelectionCost(source, this.target);
		}
		
	}
	
	/**
	 * Samples a target waypoint from the environment applying the goal bias
	 * and respecting the currently achieved cost bound.
	 * 
	 * @return the target waypoint if it could be sampled
	 */
	protected Optional<ARRTreeWaypoint> sampleTarget() {
		Optional<ARRTreeWaypoint> target = Optional.empty();
		
		ARRTreeWaypoint sample = (ARRTreeWaypoint) this.sampleBiased();
		sample.setH(this.computeHeuristic(sample, this.getGoal()));
		sample.setP(this.computeCompositeHeuristic(sample));
		
		if (sample.equals(this.getGoal())) {
			target = Optional.of(sample);
		} else {
			int sampleAttempt = 0;
			// TODO: increase sampling attempts with lower cost bound
			while ((this.getCostBound() < sample.getP())
					&& (this.getMaxSamplingAttempts() > sampleAttempt)) {
				// TODO: sampling space could be further constrained according to cost bound
				sample = (ARRTreeWaypoint) this.sample();
				sample.setH(this.computeHeuristic(sample, this.getGoal()));
				sample.setP(this.computeCompositeHeuristic(sample));
				sampleAttempt++;
			}
			if (this.getCostBound() >= sample.getP()) {
				target = Optional.of(sample);
			}
		}

		return target;
	}
	
	/**
	 * Extends the tree in the direction of a target waypoint by expanding
	 * neighbors in the tree according to their selection cost priority order.
	 * 
	 * @param target the target waypoint to extend the tree towards
	 * 
	 * @return the status of the extension towards the target waypoint
	 */
	@SuppressWarnings("unchecked")
	protected Status extendToTarget(ARRTreeWaypoint target) {
		Status status = Status.TRAPPED;
		
		// establish priority order according to selection cost
		PriorityQueue<ARRTreeWaypoint> neighbors = new PriorityQueue<ARRTreeWaypoint>(
				Comparator.comparingDouble(new CostSelector(target)));
		neighbors.addAll((Set<ARRTreeWaypoint>)
				this.getEnvironment().findNearest(target, this.getNeighborLimit()));
		
		for (ARRTreeWaypoint neighbor : neighbors) {
			// TODO: several extensions (connect versus extend)?
			Optional<Edge> extension = this.createExtension(neighbor, target);
			
			if (extension.isPresent()) {
				if (this.getNewestWaypoint().getF() < this.getCostBound()) {
					if (this.getNewestWaypoint().equals(target)) {
						return Status.REACHED;
					} else {
						return Status.ADVANCED;
					}
				} else {
					// remove too expensive vertex and associated edge
					this.getEnvironment().removeVertex(this.getNewestWaypoint());
				}
			}
		}
		
		return status;
	}
	
	/**
	 * Creates an extension to the tree by growing from a connected tree
	 * waypoint towards a non-connected sampled waypoint observing capability
	 * constraints if applicable.
	 * 
	 * @param treeWaypoint the tree waypoint to be extended
	 * @param sampledWaypoint the sampled waypoint to be reached
	 * 
	 * @return the extension if it could be created
	 */
	@Override
	protected Optional<Edge> createExtension(
			RRTreeWaypoint treeWaypoint, RRTreeWaypoint sampledWaypoint) {
		Optional<Edge> extension = super.createExtension(treeWaypoint, sampledWaypoint);
		
		if (extension.isPresent()) {
			ARRTreeWaypoint newestWaypoint = this.getNewestWaypoint();
			newestWaypoint.setH(this.computeHeuristic(newestWaypoint, this.getGoal()));
			newestWaypoint.setP(this.computeCompositeHeuristic(newestWaypoint));
		}
		
		return extension;
	}
	
	// TODO: review from here...
	
	/**
	 * Checks whether or not the current solution can be considered as having the
	 * maximum improvement possible.
	 * 
	 * @param costOld the old cost
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
	 * 
	 * @param step the bias shift step
	 */
	// TODO: priority order should be encoded in the key of waypoints making
	// explicit sorting unnecessary
	protected void updateWeights(double step) {
		double costBias = this.getCostBias() + step;
		costBias = costBias > this.getMaximumQuality() ? this.getMaximumQuality() : costBias;
		costBias = costBias < this.getMinimumQuality() ? this.getMinimumQuality() : costBias;

		this.setCostBias(costBias);
		this.setDistanceBias(1 - costBias);
	}
	
	// TODO: review anytime backup options
	// TODO: there is no real tree and backups can be done as private class
	// TODO: the applicable inflation factor and key method should be stored in a ARRTWaypoint
	
	/**
	 * Saves the current data in the planner and environment to an anytime tree.
	 * 
	 * @param tree the tree where to save the current data
	 */
	@SuppressWarnings("unchecked")
	protected void saveToTree(ARRTree tree) {
		//tree.setWaypointList((ArrayList<RRTreeWaypoint>) ((ArrayList<RRTreeWaypoint>) this.getWaypointList()).clone());
		ArrayList<RRTreeWaypoint> waypoints = new ArrayList<RRTreeWaypoint>();
		for (Position position : this.getEnvironment().getVertexIterable()) {
			waypoints.add((RRTreeWaypoint) position);
		}
		tree.setWaypointList(waypoints);
		
		//tree.setEdgeList((ArrayList<Edge>) ((ArrayList<Edge>) this.getEnvironment().getEdgeIterable()()).clone());
		ArrayList<Edge> edges = new ArrayList<Edge>();
		for (Edge edge : this.getEnvironment().getEdgeIterable()) {
			edges.add(edge);
		}
		tree.setEdgeList(edges);
				
		tree.setPlan((LinkedList<Waypoint>) ((LinkedList<Waypoint>) this.getPlan()).clone());
		tree.setDistBias(this.getDistanceBias());
		tree.setCostBias(this.getCostBias());
		tree.setCostBound(this.getCostBound());
	}

	/**
	 * Loads the data from an anytime tree to the current planner.
	 * 
	 * @param tree the tree from where to load the data
	 */
	protected void loadFromTree(ARRTree tree) {
		// TODO: adding vertices explicitly is not required
		
		//this.setWaypointList(tree.getWaypointList());
		for (RRTreeWaypoint waypoint : tree.getWaypointList()) {
			this.getEnvironment().addVertex(waypoint);
		}
		
		//this.setEdgeList(tree.getEdgeList());
		for (Edge edge : tree.getEdgeList()) {
			this.getEnvironment().addEdge(edge);
		}
		
		//this.setPlan(tree.getPlan());
		for (Waypoint waypoint : tree.getPlan()) {
			this.getPlan().add(waypoint);
		}
		
		this.setDistanceBias(tree.getDistBias());
		this.setCostBias(tree.getCostBias());
		this.setCostBound(tree.getCostBound());
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
	 * Computes an ARRT plan applying the current cost and goal biases by
	 * growing a tree until the goal region is reached.
	 * 
	 * @return true if the goal region was reached, false otherwise
	 */
	@Override
	protected boolean compute() {
		this.createSamplingShape();
		
		int iteration = 0;
		while ((!this.isInGoalRegion()) && (this.getMaxIterations() > iteration)) {
			
			// select a target sample
			Optional<ARRTreeWaypoint> target = this.sampleTarget();
			if (target.isPresent()) {
				// TODO: connect to versus extend to target
				Status status = this.extendToTarget(target.get());
				if ((Status.TRAPPED != status) && (this.isInGoalRegion())) {
					// update goal for cost bound calculation
					this.getGoal().setEto(this.getNewestWaypoint().getEto());
					this.getGoal().setCost(this.getNewestWaypoint().getCost());
					// avoid feasibility issues connecting to newest sample only
					this.connectPlan();
				}
			}
			
			iteration++;
		}
		
		this.disposeSamplingShape();
		
		if (!this.isInGoalRegion()) {
			Logging.logger().info(
					"no trajectory found after " + this.getMaxIterations()
					+ " iterations at cost bias " + this.getCostBias());
		}
		
		return this.isInGoalRegion();
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

		// TODO: part of initialize?
		this.setCostBias(this.getMinimumQuality());
		this.setDistanceBias(1 - this.getCostBias());
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
			this.getEnvironment().addVertex(getStart());
			this.setNewestWaypoint(getStart());

			costOld = this.getGoal().getCost();
			newPlan = this.compute();

			this.saveToTree(treeT);

			if (newPlan) {
				trajectory = this.createTrajectory();
				this.revisePlan(trajectory);

				treeR.clone(treeT);

				this.updateWeights();
				this.updateCostBound();
				treeT.setBiases(this.getDistanceBias(), this.getCostBias());
				treeT.setCostBound(this.getCostBound());

			} else {
				this.updateWeights();
				treeT.setBiases(this.getDistanceBias(), this.getCostBias());
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
	 * @see RRTreePlanner#plan(Position, Position, List, ZonedDateTime)
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
		this.setDistanceBias(1 - this.getCostBias());

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
					treeT[i].setBiases(this.getDistanceBias(), this.getCostBias());
					treeT[i].setCostBound(this.getCostBound());

				} else {
					this.updateWeights();
					treeT[i].setBiases(this.getDistanceBias(), this.getCostBias());
				}

				// newPlan must be true when at least one segment has a new plan
				newPlan = newPlan || newPart;

				// allImporved is true only when all segments are improved
				// TODO: NPE observed here (empty plan)
				costNew = treeR[i].getPlan().peekLast().getCost();
				allImproved = allImproved && this.isImproved(costOld, costNew);

				part = this.createTrajectory(treeR[i].getPlan());
				// append partial trajectory to plan
				if ((!plan.isEmpty()) && (!part.isEmpty())) {
					//part.pollFirst();
					part.withoutFirst();
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
