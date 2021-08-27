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
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.function.ToDoubleFunction;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.environments.DirectedEdge;
import com.cfar.swim.worldwind.environments.Edge;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.planners.AbstractPlanner;
import com.cfar.swim.worldwind.planners.AnytimePlanner;
import com.cfar.swim.worldwind.planners.rrt.Status;
import com.cfar.swim.worldwind.planners.rrt.brrt.RRTreePlanner;
import com.cfar.swim.worldwind.planners.rrt.brrt.RRTreeWaypoint;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.planners.rrt.ARRTreeProperties;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.util.Logging;

/**
 * Realizes an anytime RRT planner that plans a trajectory of an aircraft in an
 * environment considering a local cost and risk policy. The planner
 * continuously improves and revises plans according to a desired quality as
 * long as deliberation time is available.
 * 
 * @author Manuel Rosa
 * @author Stephan Heinemann
 *
 */
public class ARRTreePlanner extends RRTreePlanner implements AnytimePlanner {
	
	/** the limit of neighbors to consider for extension by this ARRT planner */
	private int neighborLimit = 5;
	
	/** the initial cost bias of this ARRT planner */
	private double initialCostBias = 0d;
	
	/** the final cost bias of this ARRT planner */
	private double finalCostBias = 1d;
	
	/** the cost improvement of consecutive plans of this ARRT planner */
	private double improvementFactor = 0.05d;
	
	/** the current distance bias (initially high) of this ARRT planner */
	private double distanceBias = 1d;
	
	/** the current cost bias (initially low) of this ARRT planner */
	private double costBias = 0d;
	
	/** the step used by this ARRT planner to shift from distance to cost bias */
	private double step = 0.1d;
	
	/** the current cost bound to be satisfied by plans of this ARRT planner */
	private double costBound = Double.POSITIVE_INFINITY;
	
	/** the current maximum number of sampling attempts of this ARRT planner */
	private int maxSamplingAttempts = 50;
	
	/** the maximum number of risk-policy exceeding probes of this ARRT planner */
	private int maxRiskyProbes = 10;
	
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
	}
	
	/**
	 * Gets the start ARRT waypoint of this ARRT planner.
	 * 
	 * @return the start ARRT waypoint of this ARRT planner
	 */
	@Override
	protected ARRTreeWaypoint getStart() {
		return (ARRTreeWaypoint) super.getStart();
	}
	
	/**
	 * Gets the goal ARRT waypoint of this ARRT planner.
	 * 
	 * @return the goal ARRT waypoint of this ARRT planner
	 */
	@Override
	protected ARRTreeWaypoint getGoal() {
		return (ARRTreeWaypoint) super.getGoal();
	}
	
	/**
	 * Gets the newest ARRT waypoint added to the tree.
	 * 
	 * @return the newest ARRT waypoint added to the tree
	 */
	@Override
	protected ARRTreeWaypoint getNewestWaypoint() {
		return (ARRTreeWaypoint) super.getNewestWaypoint();
	}
	
	/**
	 * Creates an ARRT waypoint at a specified position.
	 * 
	 * @param position the position in globe coordinates
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
			this.step = (this.finalCostBias - this.initialCostBias) / 10d;
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
			this.step = (this.finalCostBias - this.initialCostBias) / 10d;
		} else {
			throw new IllegalArgumentException("final cost bias is invalid");
		}
	}
	
	/**
	 * Determines whether or not this ARRT planner has achieved the maximum
	 * quality. The maximum quality has been achieved if the computed
	 * trajectory does not satisfy the cost bound while the cost bias has
	 * reached its maximum.
	 * 
	 * @return true if this ARRT planner has achieved the maximum quality,
	 *         false otherwise
	 * 
	 * @see AnytimePlanner#hasMaximumQuality()
	 */
	@Override
	public boolean hasMaximumQuality() {
		boolean hasMaximumQuality = false;
		
		if (this.hasGoal()) {
			hasMaximumQuality = (this.getCostBound() < this.getGoal().getCost())
				&& (this.getCostBias() >= this.getMaximumQuality());
		}
		
		return hasMaximumQuality;
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
	 * Gets the step used to shift the distance and cost biases.
	 * 
	 * @return the step used to shift the distance and cost biases
	 */
	protected double getStep() {
		return this.step;
	}
	
	/**
	 * Sets the step used to shift the distance and cost biases.
	 * 
	 * @param step the step used to shift the distance and cost biases
	 */
	protected void setStep(double step) {
		this.step = step;
	}
	
	/**
	 * Updates the cost and distance biases applied by this ARRT planner.
	 * Shifts the bias from distance to cost by a single step.
	 */
	protected void updateBiases() {
		// delta cost bias
		double dq = this.getMaximumQuality() - this.getMinimumQuality();
		// optimal cost fraction
		double fc = this.getStart().getH() / this.getCostBound();
		
		// adjust cost bias to current cost bound
		double minCostBias = fc * dq;
		if (this.getCostBias() < minCostBias) {
			this.setCostBias(minCostBias);
		} else {
			double costBias = this.getCostBias() + this.getStep();
			costBias = (costBias > this.getMaximumQuality()) ? this.getMaximumQuality() : costBias;
			costBias = (costBias < this.getMinimumQuality()) ? this.getMinimumQuality() : costBias;
			this.setCostBias(costBias);
		}
		
		this.setDistanceBias(1d - this.getCostBias());
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
	 * Updates the cost bound of trajectories planned by this ARRT planner.
	 * Tightens (decreases) the cost of the last found trajectory by the
	 * quality improvement factor to determine the updated cost bound.
	 */
	protected void updateCostBound() {
		double costBound = (1d - this.getQualityImprovement())
				* this.getGoal().getCost();
		
		if (this.getStart().getH() > costBound) {
			this.setCostBound(this.getStart().getH());
		} else {
			this.setCostBound(costBound);
		}
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
	 * Gets the maximum number of risky probes of this ARRT planner exceeding
	 * the risk policy before giving up any improvement attempt.
	 * 
	 * @return the maximum number of risky probes of this ARRT planner
	 */
	protected int getMaxRiskyProbes() {
		return this.maxRiskyProbes;
	}
	
	/**
	 * Sets the maximum number of risky probes of this ARRT planner exceeding
	 * the risk policy before giving up any improvement attempt.
	 * 
	 * @param maxRiskyProbes the maximum number of risky probes to be set
	 */
	protected void setMaxRiskyProbes(int maxRiskyProbes) {
		this.maxRiskyProbes = maxRiskyProbes;
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
	protected double computeSelectionCost(
			ARRTreeWaypoint source, ARRTreeWaypoint target) {
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
				// TODO: sampling space could be further constrained
				// according to cost bound
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
		// TODO: other nearest metrics (capabilities)
		neighbors.addAll((Set<ARRTreeWaypoint>) this.getPlanningContinuum()
				.findNearest(target, this.getNeighborLimit()));
		
		for (ARRTreeWaypoint neighbor : neighbors) {
			// TODO: single versus several extensions towards lower cost bound
			Optional<Edge> extension = this.createExtension(neighbor, target);
			
			if (extension.isPresent()) {
				// ensure all extensions satisfy the current cost bound
				ARRTreeWaypoint newest = this.getNewestWaypoint();
				if (newest.getF() <= this.getCostBound()) {
					if (newest.equals(target)) {
						return Status.REACHED;
					} else {
						return Status.ADVANCED;
					}
				} else {
					// remove too expensive vertex and associated edge
					if (newest.hasParent()) {
						newest.getParent().removeChild(newest);
						newest.removeParent();
					}
					this.getPlanningContinuum().removeVertex(newest);
					this.setNewestWaypoint(this.getStart());
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
	
	/**
	 * Initializes this ARRT planner to plan from an origin to a destination at
	 * a specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 */
	@Override
	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {
		super.initialize(origin, destination, etd);
		
		this.getGoal().setH(0d);
		this.getStart().setH(this.computeHeuristic(getStart(), getGoal()));
		
		this.setCostBias(this.getMinimumQuality());
		this.setDistanceBias(1d - this.getCostBias());
		this.setCostBound(Double.POSITIVE_INFINITY);
	}
	
	/**
	 * Computes an ARRT plan applying the current cost and goal biases by
	 * growing a tree until the goal region is reached.
	 * 
	 * @return true if the goal region was reached, false otherwise
	 */
	@Override
	protected boolean compute() {
		if (this.getStart().equals(this.getGoal())) {
			this.connectPlan(this.getStart());
			return true;
		}
		
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
					this.getGoal().setParent(this.getNewestWaypoint().getParent());
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
	 * Improves an ARRT plan incrementally.
	 * 
	 * @param partIndex the index of the plan part to be improved
	 */
	protected void improve(int partIndex) {
		if (!this.hasMaximumQuality()) {
			this.backup(partIndex);
			this.getPlanningContinuum().clearVertices();
			this.getPlanningContinuum().addVertex(this.getStart());
			this.setNewestWaypoint(this.getStart());
			this.getStart().clearChildren();
			this.getGoal().removeParent();
			
			if (this.compute() && !this.getGoal().hasInfiniteCost()) {
				// improved trajectory without exceeding risk policy
				this.revisePlan(this.createTrajectory());
				this.updateCostBound();
			} else {
				// no trajectory or exceeded risk policy
				this.restore(partIndex);
			}
			
			this.updateBiases();
		}
	}
	
	/**
	 * Elaborates an ARRT plan.
	 * 
	 * @param partIndex the index of the plan part to be elaborated
	 */
	protected void elaborate(int partIndex) {
		// do not elaborate an exceeded risk policy solution beyond limit
		int riskyProbes = (this.getGoal().hasInfiniteCost()) ? 1 : 0;
		
		while (!this.hasMaximumQuality() && (this.getMaxRiskyProbes() > riskyProbes)) {
			this.improve(partIndex);
			if (this.getGoal().hasInfiniteCost()) {
				riskyProbes++;
			}
		}
	}
	
	/**
	 * Plans a part of a trajectory. Improves the planned part incrementally
	 * until a maximum quality has been achieved.
	 * 
	 * @param partIndex the index of the part
	 * 
	 * @return the planned part of a trajectory
	 */
	@Override
	protected Trajectory planPart(int partIndex) {
		Trajectory trajectory = super.planPart(partIndex);
		this.revisePlan(trajectory);
		this.elaborate(partIndex);
		return this.createTrajectory();
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
		this.initBackups(1);
		this.initialize(origin, destination, etd);
		Trajectory trajectory = this.planPart(0);
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
	 * @return the planned trajectory from the origin to the destination along
	 *         the waypoints with the estimated time of departure
	 * 
	 * @see RRTreePlanner#plan(Position, Position, List, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
		this.initBackups(waypoints.size() + 1);
		return super.plan(origin, destination, waypoints, etd);
	}
	
	/** the backups of this ARRT planner */
	protected final ArrayList<Backup> backups = new ArrayList<>();
	
	/**
	 * Realizes a backup of this ARRT planner.
	 * 
	 * @author Stephan Heinemann
	 * 
	 */
	protected class Backup {
		
		/** the start waypoint of this ARRT backup */
		public ARRTreeWaypoint start = null;
		
		/** the goal waypoint of this ARRT backup */
		public ARRTreeWaypoint goal = null;
		
		/** the edges of this ARRT backup */
		public Set<DirectedEdge> edges = new HashSet<>();
		
		/** the distance bias of this ARRT backup */
		public double distanceBias = 1d;
		
		/** the cost bias of this ARRT backup */
		public double costBias = 0d;
		
		/** the cost bound of this ARRT backup */
		public double costBound = Double.POSITIVE_INFINITY;
		
		/** the plan waypoints of this ARRT backup */
		public List<Waypoint> plan = new LinkedList<Waypoint>();
		
		/**
		 * Clears this ARRT backup.
		 */
		public void clear() {
			this.start = null;
			this.goal = null;
			this.edges.clear();
			this.plan.clear();
		}
		
		/**
		 * Determines whether or not this ARRT backup is empty.
		 * 
		 * @return true if this ARRT backup is empty, false otherwise
		 */
		public boolean isEmpty() {
			return (null == this.start) && (null == this.goal)
					&& this.edges.isEmpty()
					&& this.plan.isEmpty();
		}
	}
	
	/**
	 * Initializes a number backups for this ARRT planner.
	 * 
	 * @param size the number of backups for this ARRT planner
	 */
	protected void initBackups(int size) {
		this.backups.clear();
		for (int backupIndex = 0; backupIndex < size; backupIndex++) {
			this.backups.add(backupIndex, new Backup());
		}
	}
	
	/**
	 * Determines whether or not a backup of this ARRT planner can be
	 * performed.
	 * 
	 * @param backupIndex the index of the backup
	 * 
	 * @return true if a backup can be performed, false otherwise
	 */
	protected boolean canBackup(int backupIndex) {
		return (-1 < backupIndex) && (backupIndex < this.backups.size());
	}
	
	/**
	 * Determines whether or not this ARRT planner has a backup.
	 * 
	 * @param backupIndex the index of the backup
	 * 
	 * @return true if this ARRT planner has a backup, false otherwise
	 */
	protected boolean hasBackup(int backupIndex) {
		return this.canBackup(backupIndex) && (!this.backups.get(backupIndex).isEmpty());
	}
	
	/**
	 * Backs up this ARRT planner for improvement.
	 * 
	 * @param backupIndex the index of the backup
	 * 
	 * @return true if a backup has been performed, false otherwise
	 */
	protected boolean backup(int backupIndex) {
		boolean backedup = false;
		
		if (this.canBackup(backupIndex)) {
			Backup backup = this.backups.get(backupIndex);
			backup.clear();
			backup.start = (ARRTreeWaypoint) this.getStart().clone();
			backup.goal = (ARRTreeWaypoint) this.getGoal().clone();
			for (Edge edge : this.getPlanningContinuum().getEdges()) {
				backup.edges.add((DirectedEdge) edge);
			}
			backup.distanceBias = this.getDistanceBias();
			backup.costBias = this.getCostBias();
			backup.costBound = this.getCostBound();
			backup.plan.addAll(this.getWaypoints());
			backedup = true;
		}
		
		return backedup;
	}
	
	/**
	 * Restores this ARRT planner for improvement.
	 * 
	 * @param backupIndex the index of the backup
	 * 
	 * @return true if a restore has been performed, false otherwise
	 */
	protected boolean restore(int backupIndex) {
		boolean restored = false;
		
		if (this.hasBackup(backupIndex)) {
			Backup backup = this.backups.get(backupIndex);
			this.clearExpendables();
			this.setStart(backup.start);
			this.setGoal(backup.goal);
			for (Edge edge : backup.edges) {
				this.getPlanningContinuum().addEdge(edge);
			}
			this.setDistanceBias(backup.distanceBias);
			this.setCostBias(backup.costBias);
			this.setCostBound(backup.costBound);
			this.getWaypoints().addAll(backup.plan);
			restored = true;
		}
		
		return restored;
	}
	
	/**
	 * Determines whether or not an ARRT backup has waypoints.
	 * 
	 * @param backupIndex the index of the backup
	 * 
	 * @return true if the ARRT backup has waypoints, false otherwise
	 */
	protected boolean hasWaypoints(int backupIndex) {
		boolean hasWaypoints = false;
		
		if (this.hasBackup(backupIndex)) {
			Backup backup = this.backups.get(backupIndex);
			hasWaypoints = !backup.plan.isEmpty();
		}
		
		return hasWaypoints;
	}
	
	/**
	 * Determines whether or not an ARRT backup index is the last.
	 * 
	 * @param backupIndex the backup index
	 * 
	 * @return if the backup index is the last, false otherwise
	 */
	protected boolean isLastIndex(int backupIndex) {
		return (this.backups.size() - 1) == backupIndex;
	}
	
	/**
	 * Determines whether or not this ARRT planner matches a specification.
	 * 
	 * @param specification the specification to be matched
	 * 
	 * @return true if the this ARRT planner matches the specification,
	 *         false otherwise
	 * 
	 * @see AbstractPlanner#matches(Specification)
	 */
	@Override
	public boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = false;
		
		if ((null != specification) && (specification.getProperties() instanceof ARRTreeProperties)) {
			ARRTreeProperties arrtp = (ARRTreeProperties) specification.getProperties();
			matches = (this.getCostPolicy().equals(arrtp.getCostPolicy()))
					&& (this.getRiskPolicy().equals(arrtp.getRiskPolicy()))
					&& (this.getBias() == arrtp.getBias())
					&& (this.getEpsilon() == arrtp.getEpsilon())
					&& (this.getExtension() == arrtp.getExtension())
					&& (this.getGoalThreshold() == arrtp.getGoalThreshold())
					&& (this.getMaxIterations() == arrtp.getMaxIterations())
					&& (this.getSampling() == arrtp.getSampling())
					&& (this.getStrategy() == arrtp.getStrategy())
					&& (this.getMaximumQuality() == arrtp.getMaximumQuality())
					&& (this.getMinimumQuality() == arrtp.getMaximumQuality())
					&& (this.getNeighborLimit() == arrtp.getNeighborLimit())
					&& (this.getQualityImprovement() == arrtp.getQualityImprovement())
					&& (specification.getId().equals(Specification.PLANNER_ARRT_ID));
		}
		
		return matches;
	}
	
}
