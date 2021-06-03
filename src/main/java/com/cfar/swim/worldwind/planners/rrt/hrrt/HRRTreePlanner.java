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
package com.cfar.swim.worldwind.planners.rrt.hrrt;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Optional;
import java.util.Random;
import java.util.Set;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.environments.Edge;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.planners.AbstractPlanner;
import com.cfar.swim.worldwind.planners.rrt.Status;
import com.cfar.swim.worldwind.planners.rrt.brrt.RRTreePlanner;
import com.cfar.swim.worldwind.planners.rrt.brrt.RRTreeWaypoint;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.planners.rrt.HRRTreeProperties;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.util.Logging;

/**
 * Realizes an heuristic RRT planner that considers the cost of waypoints
 * during sampling and the expanding steps of a basic RRT planner in order to
 * direct the path to a more optimal solution.
 * 
 * @author Manuel Rosa
 * @author Stephan Heinemann
 *
 */
public class HRRTreePlanner extends RRTreePlanner {
	
	/** the heuristic algorithm of this hRRT planner */
	private Heuristic heuristic = Heuristic.BkRRT;
	
	/**
	 * the variant of this hRRT planner affecting its quality and probability
	 * calculations
	 */
	private HRRTreeVariant variant = HRRTreeVariant.ENHANCED;
	
	/**
	 * limit of sampled neighbors to consider by this hRRT planner for a
	 * quality test of a new sample
	 */
	private int neighborLimit = 5; // [1, Integer.MAX_VALUE]
	
	/**
	 * the probability floor value of this hRRT planner to ensure the search is
	 * not overly biased against exploration
	 */
	private double probabilityFloor = 0.1d; // [0, 1]
	
	/** the nearest waypoint in the tree of this hRRT planner */
	private HRRTreeWaypoint waypointNear = null;
	
	/** the estimated total cost of the optimal path from start to goal */
	private double optimalCostEstimate;
	
	/** the maximum cost of all vertices already considered */
	private double maximumCost;
	
	/**
	 * Constructs a heuristic RRT planner for a specified aircraft and
	 * environment using default local cost and risk policies as well as
	 * default hRRT properties.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see RRTreePlanner#RRTreePlanner(Aircraft, Environment)
	 */
	public HRRTreePlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}
	
	/**
	 * Gets the start hRRT waypoint of this hRRT planner.
	 * 
	 * @return the start hRRT waypoint of this hRRT planner
	 * 
	 * @see RRTreePlanner#getStart()
	 */
	@Override
	public HRRTreeWaypoint getStart() {
		return (HRRTreeWaypoint) super.getStart();
	}
	
	/**
	 * Gets the goal hRRT waypoint of this hRRT planner.
	 * 
	 * @return the goal hRRT waypoint of this hRRT planner
	 * 
	 * @see RRTreePlanner#getGoal()
	 */
	@Override
	public HRRTreeWaypoint getGoal() {
		return (HRRTreeWaypoint) super.getGoal();
	}
	
	/**
	 * Creates a hRRT waypoint at a specified position.
	 * 
	 * @param position the position
	 * 
	 * @return the hRRT waypoint at the specified position
	 */
	@Override
	protected HRRTreeWaypoint createWaypoint(Position position) {
		return new HRRTreeWaypoint(position);
	}
	
	/**
	 * Computes the estimated remaining cost (h-value) of a specified source
	 * hRRT waypoint to reach a target hRRT waypoint.
	 * 
	 * @param source the source hRRT waypoint in globe coordinates
	 * @param target the target hRRT waypoint in globe coordinates
	 * 
	 * @return the estimated remaining cost (h-value)
	 */
	protected double computeHeuristic(
			HRRTreeWaypoint source, HRRTreeWaypoint target) {
		return this.getEnvironment().getNormalizedDistance(source, target);
	}
	
	/**
	 * Gets the heuristic algorithm of this hRRT planner.
	 * 
	 * @return the heuristic algorithm of this hRRT planner
	 */
	public Heuristic getHeuristic() {
		return this.heuristic;
	}
	
	/**
	 * Sets the heuristic algorithm of this hRRT planner.
	 * 
	 * @param heuristic the heuristic algorithm to be set
	 */
	public void setHeuristic(Heuristic heuristic) {
		this.heuristic = heuristic;
	}
	
	/**
	 * Gets the applied implementation variant of this hRRT planner.
	 * 
	 * @return the applied implementation variant of this hRRT planner
	 */
	public HRRTreeVariant getVariant() {
		return this.variant;
	}
	
	/**
	 * Sets the applied implementation variant of this hRRT planner.
	 * 
	 * @param variant the implementation variant to be set
	 */
	public void setVariant(HRRTreeVariant variant) {
		this.variant = variant;
	}
	
	/**
	 * Determines whether or not the original implementation variant is being
	 * applied by this hRRT planner.
	 * 
	 * @return true if the original implementation variant is being applied by
	 *         this hRRT planner, false otherwise
	 */
	public boolean isOriginal() {
		return HRRTreeVariant.ORIGINAL == this.getVariant();
	}
	
	/**
	 * Determines whether or not the enhance implementation variant is being
	 * applied by this hRRT planner.
	 * 
	 * @return true if the enhanced implementation variant is being applied by
	 *         this hRRT planner, false otherwise
	 */
	public boolean isEnhanced() {
		return HRRTreeVariant.ENHANCED == this.getVariant();
	}
	
	/**
	 * Gets limit of sampled neighbors to consider for a quality test of a new
	 * sample by this hRRT planner.
	 * 
	 * @return the limit of sampled neighbors to consider for a quality test of
	 *         a new sample by this hRRT planner
	 */
	public int getNeighborLimit() {
		return this.neighborLimit;
	}
	
	/**
	 * Sets the limit of neighbors to consider for a quality test of a new
	 * sample by this hRRT planner.
	 * 
	 * @param neighborLimit the limit of neighbors to be set
	 * 
	 * @throws IllegalArgumentException exception if neighbor limit is invalid
	 */
	public void setNeighborLimit(int neighborLimit) {
		if ((0 < neighborLimit) && (Integer.MAX_VALUE >= neighborLimit)) {
			this.neighborLimit = neighborLimit;
		} else {
			throw new IllegalArgumentException("neighbor limit is invalid");
		}
	}
	
	/**
	 * Gets the probability floor for the acceptance of a new sample by this
	 * hRRT planner.
	 * 
	 * @return the probability floor for the acceptance of a new sample by this
	 *         hRRT planner
	 */
	public double getProbabilityFloor() {
		return this.probabilityFloor;
	}
	
	/**
	 * Sets the probability floor for the acceptance of a new sample by this
	 * hRRT planner.
	 * 
	 * @param probabilityFloor the probability floor to be set
	 * 
	 * @throws IllegalArgumentException if probability floor is invalid
	 */
	public void setProbabilityFloor(double probabilityFloor) {
		if ((0d <= probabilityFloor) && (1d >= probabilityFloor)) {
			this.probabilityFloor = probabilityFloor;
		} else {
			throw new IllegalArgumentException("probability floor is invalid");
		}
	}
	
	/**
	 * Gets the estimated total cost of the optimal path from start to goal.
	 * 
	 * @return the estimated total cost of the optimal path from start to goal
	 */
	protected double getOptimalCostEstimate() {
		return this.optimalCostEstimate;
	}
	
	/**
	 * Sets the estimated total cost of the optimal path from start to goal.
	 * 
	 * @param optimalCostEstimate the estimated total cost of the optimal path
	 *                            from start to goal to be set
	 */
	protected void setOptimalCostEstimate(double optimalCostEstimate) {
		this.optimalCostEstimate = optimalCostEstimate;
	}
	
	/**
	 * Gets the maximum cost of all vertices already considered.
	 * 
	 * @return the maximum cost of all vertices already considered
	 */
	protected double getMaximumCost() {
		return this.maximumCost;
	}
	
	/**
	 * Sets the maximum cost of all vertices already considered.
	 * 
	 * @param maximumCost the maximum cost of all vertices already considered
	 *                    to be set
	 */
	protected void setMaximumCost(double maximumCost) {
		this.maximumCost = maximumCost;
	}
	
	// TODO: review from here...
	
	/**
	 * Gets the nearest RRT waypoint added to the tree.
	 * 
	 * @return the waypointNear the nearest waypoint added to the tree
	 */
	public HRRTreeWaypoint getWaypointNear() {
		return this.waypointNear;
	}

	/**
	 * Sets the nearest RRT waypoint added to the tree.
	 * 
	 * @param waypointNear the nearest waypoint added to the tree
	 */
	public void setWaypointNear(HRRTreeWaypoint waypointNear) {
		this.waypointNear = waypointNear;
	}

	/**
	 * Computes the quality of a certain waypoint given its cost.
	 * 
	 * @param cost the cost (f-value) of the waypoint
	 * 
	 * @return the quality of the waypoint [0,1]
	 */
	protected double computeQuality(double cost) {
		double relativeCost = 0d;
		double diff = cost - optimalCostEstimate;
		double quality;

		if (Math.abs(diff) < 0.00001) // prevent numerical imprecision
			return 1;

		// original implementation of HRRT
		if (this.isOriginal()) {
			relativeCost = (cost - optimalCostEstimate) / (maximumCost - optimalCostEstimate);
			quality = 1 - relativeCost;
		}
		// enhanced implementation of HRRT
		else {
			if (cost > optimalCostEstimate)
				relativeCost = Math.exp(-optimalCostEstimate / (cost - optimalCostEstimate));
			else
				relativeCost = 0d; // limit case of exponential

			quality = 1 - relativeCost;
			quality = (quality > probabilityFloor) ? quality : probabilityFloor;
		}

		return quality;

	}

	/**
	 * Selects the waypoint from the environment space according to quality
	 * parameters.
	 * 
	 * @return the selected waypoint
	 */
	protected HRRTreeWaypoint selectWaypoint() {
		HRRTreeWaypoint sample = null, waypointNear = null;

		Random r = new Random();
		double quality = -1.0, rand = 0;

		while (rand > quality) {
			sample = (HRRTreeWaypoint) super.sampleBiased();
			waypointNear = (HRRTreeWaypoint) this.getEnvironment().findNearest(sample, 1).iterator().next();

			quality = computeQuality(waypointNear.getF());
			if (this.isOriginal())
				quality = (quality < probabilityFloor) ? quality : probabilityFloor; // Minimum(qual,floor)

			rand = r.nextDouble();
		}

		this.waypointNear = waypointNear;
		return sample;
	}

	/**
	 * Selects a good waypoint according to a quality parameter by repeatedly
	 * sampling a random waypoint from the environment space and iteratively testing
	 * the k-nearest waypoints sorted by decreasing quality
	 * 
	 * @param neighborLimit the number of number of neighborLimit to test
	 * 
	 * @return the selected waypoint
	 */
	@SuppressWarnings("unchecked")
	protected HRRTreeWaypoint selectWaypointIK(int neighborLimit) {
		HRRTreeWaypoint sample = null;
		// TODO: priority queue according to configurable compareTo
		ArrayList<HRRTreeWaypoint> neighborsList = new ArrayList<HRRTreeWaypoint>();
		
		Random r = new Random();
		double quality, rand;

		while (true) {
			sample = (HRRTreeWaypoint) super.sampleBiased();
			neighborsList.clear();
			neighborsList.addAll((Set<HRRTreeWaypoint>) this.getEnvironment().findNearest(sample, neighborLimit));
			neighborsList = this.sortByQuality(neighborsList);

			for (HRRTreeWaypoint neighbor : neighborsList) {
				quality = computeQuality(neighbor.getF());
				if (this.isOriginal())
					quality = (quality < probabilityFloor) ? quality : probabilityFloor; // Minimum(qual,floor)

				rand = r.nextDouble();
				if (rand < quality) {
					this.waypointNear = neighbor;
					return sample;
				}
			}
		}

	}

	/**
	 * Selects a good waypoint according to a quality parameter by repeatedly
	 * sampling a random waypoint from the environment space and testing the best of
	 * the k-nearest waypoints
	 * 
	 * @param neighborLimit the number of number of neighborLimit to test
	 * 
	 * @return the selected waypoint
	 */
	@SuppressWarnings("unchecked")
	protected HRRTreeWaypoint selectWaypointBK(int neighborLimit) {
		HRRTreeWaypoint sample = null, waypointNear = null;
		// TODO: priority queue according to configurable compareTo
		ArrayList<HRRTreeWaypoint> neighborsList = new ArrayList<HRRTreeWaypoint>();

		Random r = new Random();
		double quality = -1.0, rand = 0;
		
		while (rand > quality) {
			sample = (HRRTreeWaypoint) super.sampleBiased();
			neighborsList.clear();
			neighborsList.addAll((Set<HRRTreeWaypoint>) this.getEnvironment().findNearest(sample, neighborLimit));
			neighborsList = this.sortByQuality(neighborsList);
			waypointNear = neighborsList.get(0);

			quality = computeQuality(waypointNear.getF());
			if (this.isOriginal())
				quality = (quality < probabilityFloor) ? quality : probabilityFloor; // Minimum(qual,floor)

			rand = r.nextDouble();
		}
		//System.out.println("Cost = "+waypointNear.getG()+" TotalCost = "+waypointNear.getF()+" Quality = "+computeQuality(waypointNear.getF())+"\tWpt = "+waypointNear.toString());

		this.waypointNear = waypointNear;
		return sample;
	}

	/**
	 * Sorts a list of waypoints by decreasing quality.
	 * 
	 * @param waypointList the list to be sorted
	 * 
	 * @return the sorted list of waypoints
	 */
	protected ArrayList<HRRTreeWaypoint> sortByQuality(ArrayList<HRRTreeWaypoint> waypointList) {
		// without variant quality is inversely proportional to total cost of node
		if (this.isOriginal())
			Collections.sort(waypointList, (a, b) -> a.getF() < b.getF() ? -1 : a.getF() == b.getF() ? 0 : 1);
		// with variant quality may not be proportional to f if it was saturated
		else
			Collections.sort(waypointList, (a, b) -> computeQuality(a.getF()) > computeQuality(b.getF()) ? -1
					: computeQuality(a.getF()) == computeQuality(b.getF()) ? 0 : 1);

		return waypointList;
	}

	/**
	 * Connects the tree in the direction of the given waypoint until it is reached
	 * or the tree is trapped. Repeatedly calls extendRRT and returns the status
	 * according to the last result of the extension
	 * 
	 * @param waypoint the waypoint set as the goal for extension
	 * @return status the status resulting from the last extend
	 * 
	 * @see RRTreePlanner#connectRRT(RRTreeWaypoint waypoint)
	 */
	@Override
	protected Status connectRRT(RRTreeWaypoint waypoint) {
		Status status = Status.ADVANCED;

		while (status == Status.ADVANCED && !this.isInGoalRegion(this.getNewestWaypoint())) {
			status = this.extendRRT(waypoint);
			this.setWaypointNear((HRRTreeWaypoint) this.getNewestWaypoint());
		}

		return status;
	}

	/**
	 * Extends the tree in the direction of the given waypoint and returns the
	 * status according to the result of the extension.
	 * 
	 * @param waypoint the waypoint set as the goal for extension
	 * @return status the status resulting from the extension
	 * 
	 * @see RRTreePlanner#extendRRT(RRTreeWaypoint waypoint)
	 */
	@Override
	protected Status extendRRT(RRTreeWaypoint waypoint) {
		Status status;
		RRTreeWaypoint waypointNear = this.getWaypointNear();
		HRRTreeWaypoint waypointNew;

		// Create a new node by extension from near to sampled
		Optional<Edge> extension = this.createExtension(waypointNear, waypoint);
		

		// Set status variable by checking if extension was possible
		if (extension.isPresent()) {
			waypointNew = (HRRTreeWaypoint) this.getNewestWaypoint();
			this.getEnvironment().addEdge(waypointNew.getParent(), waypointNew);

			// TODO: integrate H into overridden create extension
			waypointNew.setH(this.computeHeuristic(waypointNew, this.getGoal()));

			maximumCost = (maximumCost > waypointNew.getF()) ? maximumCost : waypointNew.getF();

			if (waypointNew.getPrecisionPosition().equals(waypoint.getPrecisionPosition())) {
				status = Status.REACHED;
			} else {
				status = Status.ADVANCED;
			}
		} else {
			status = Status.TRAPPED;
		}

		return status;
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

		this.setGoal(this.createWaypoint(destination));
		this.getGoal().setH(0d);

		HRRTreeWaypoint start = this.createWaypoint(origin);
		start.setEto(etd);
		start.setG(0d);
		start.setH(this.computeHeuristic(start, this.getGoal()));

		this.setStart(start);
		this.getEnvironment().addVertex(start);
		this.setNewestWaypoint(start);

		this.setOptimalCostEstimate(start.getF());
		this.setMaximumCost(this.getOptimalCostEstimate());
	}
	
	/**
	 * Computes a hRRT plan by growing a tree until the goal region is reached.
	 * 
	 * @return true if the goal region was reached, false otherwise
	 */
	@Override
	protected boolean compute() {
		this.createSamplingShape();
		
		for (int i = 0; i < this.getMaxIterations(); i++) {
			HRRTreeWaypoint sample = null;
			Status status = Status.TRAPPED;
			
			// select a new sample
			switch (this.getHeuristic()) {
			case BkRRT:
				sample = this.selectWaypointBK(this.getNeighborLimit());
				break;
			case IkRRT:
				sample = this.selectWaypointIK(this.getNeighborLimit());
				break;
			case hRRT:
			default:
				sample = this.selectWaypoint();
			}
			
			// connect to or extend towards sample according to strategy
			switch (this.getStrategy()) {
			case CONNECT:
				status = this.connectRRT(sample);
				break;
			case EXTEND:
			default:
				status = this.extendRRT(sample);
			}
			
			// check goal region
			if ((Status.TRAPPED != status) && this.isInGoalRegion()) {
				this.setGoal(this.getNewestWaypoint());
				this.connectPlan();
				return true;
			}
		}
		
		this.disposeSamplingShape();
		
		Logging.logger().info("no trajectory found after " + this.getMaxIterations() + " iterations");
		return false;
	}
	
	/**
	 * Determines whether or not this hRRT planner matches a specification.
	 * 
	 * @param specification the specification to be matched
	 * 
	 * @return true if the this hRRT planner matches the specification,
	 *         false otherwise
	 * 
	 * @see AbstractPlanner#matches(Specification)
	 */
	@Override
	public boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = false;
		
		if (matches && (specification.getProperties() instanceof HRRTreeProperties)) {
			HRRTreeProperties hrrtp = (HRRTreeProperties) specification.getProperties();
			matches = (this.getCostPolicy().equals(hrrtp.getCostPolicy()))
					&& (this.getRiskPolicy().equals(hrrtp.getRiskPolicy()))
					&& (this.getBias() == hrrtp.getBias())
					&& (this.getEpsilon() == hrrtp.getEpsilon())
					&& (this.getExtension() == hrrtp.getExtension())
					&& (this.getGoalThreshold() == hrrtp.getGoalThreshold())
					&& (this.getMaxIterations() == hrrtp.getMaxIterations())
					&& (this.getSampling() == hrrtp.getSampling())
					&& (this.getStrategy() == hrrtp.getStrategy())
					&& (this.getHeuristic() == hrrtp.getHeuristic())
					&& (this.getVariant() == hrrtp.getVariant())
					&& (this.getNeighborLimit() == hrrtp.getNeighborLimit())
					&& (this.getProbabilityFloor() == hrrtp.getProbabilityFloor())
					&& (specification.getId().equals(Specification.PLANNER_HRRT_ID));
		}
		
		return matches;
	}
	
}
