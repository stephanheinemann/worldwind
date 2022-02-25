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
import java.util.Comparator;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.Random;
import java.util.Set;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.environments.Edge;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.planners.rrt.Status;
import com.cfar.swim.worldwind.planners.rrt.brrt.RRTreePlanner;
import com.cfar.swim.worldwind.planners.rrt.brrt.RRTreeWaypoint;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.planners.rrt.HRRTreeProperties;
import com.cfar.swim.worldwind.util.Identifiable;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.util.Logging;

/**
 * Realizes a heuristic RRT planner that plans a trajectory of an aircraft in
 * an environment considering a local cost and risk policy. The planner
 * considers the estimated cost of waypoints during the sampling and expanding
 * steps of a basic RRT planner in order to direct the path to a more optimal
 * solution.
 * 
 * @author Manuel Rosa
 * @author Stephan Heinemann
 *
 */
public class HRRTreePlanner extends RRTreePlanner {
	
	/** the heuristic algorithm of this hRRT planner */
	private HRRTreeAlgorithm algorithm = HRRTreeAlgorithm.BkRRT;
	
	/** the quality assessment function of this hRRT planner */
	private HRRTreeQuality quality = new HRRTreeQuality();
	
	/** the nearest waypoint to extend from in the tree of this hRRT planner */
	private HRRTreeWaypoint nearestWaypoint = null;
	
	/** the limit of neighbors to consider for extension by this hRRT planner */
	private int neighborLimit = 5; // [1, Integer.MAX_VALUE]
	
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
	 * Gets the identifier of this heuristic RRT planner.
	 * 
	 * @return the identifier of this heuristic RRT planner
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return Specification.PLANNER_HRRT_ID;
	}
	
	/**
	 * Gets the start hRRT waypoint of this hRRT planner.
	 * 
	 * @return the start hRRT waypoint of this hRRT planner
	 */
	@Override
	protected HRRTreeWaypoint getStart() {
		return (HRRTreeWaypoint) super.getStart();
	}
	
	/**
	 * Gets the goal hRRT waypoint of this hRRT planner.
	 * 
	 * @return the goal hRRT waypoint of this hRRT planner
	 */
	@Override
	protected HRRTreeWaypoint getGoal() {
		return (HRRTreeWaypoint) super.getGoal();
	}
	
	/**
	 * Gets the newest hRRT waypoint added to the tree.
	 * 
	 * @return the newest hRRT waypoint added to the tree
	 */
	@Override
	protected HRRTreeWaypoint getNewestWaypoint() {
		return (HRRTreeWaypoint) super.getNewestWaypoint();
	}
	
	/**
	 * Creates a hRRT waypoint at a specified position.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return the hRRT waypoint at the specified position
	 */
	@Override
	protected HRRTreeWaypoint createWaypoint(Position position) {
		HRRTreeWaypoint waypoint = new HRRTreeWaypoint(position);
		
		if (waypoint.equals(this.getStart())) {
			waypoint = this.getStart();
		} else if (waypoint.equals(this.getGoal())) {
			waypoint = this.getGoal();
		}
		
		return waypoint;
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
	public synchronized HRRTreeAlgorithm getAlgorithm() {
		return this.algorithm;
	}
	
	/**
	 * Sets the heuristic algorithm of this hRRT planner.
	 * 
	 * @param algorithm the heuristic algorithm to be set
	 */
	public synchronized void setAlgorithm(HRRTreeAlgorithm algorithm) {
		this.algorithm = algorithm;
	}
	
	/**
	 * Gets the quality assessment function of this hRRT planner.
	 * 
	 * @return the quality assessment function of this hRRT planner
	 */
	protected HRRTreeQuality getQuality() {
		return this.quality;
	}
	
	/**
	 * Sets the quality assessment function of this hRRT planner.
	 * 
	 * @param quality the quality assessment function of this hRRT planner
	 */
	protected void setQuality(HRRTreeQuality quality) {
		this.quality = quality;
	}
	
	/**
	 * Gets the applied quality assessment variant of this hRRT planner.
	 * 
	 * @return the applied quality assessment variant of this hRRT planner
	 */
	public synchronized HRRTreeQualityVariant getVariant() {
		return this.getQuality().getVariant();
	}
	
	/**
	 * Sets the applied quality assessment variant of this hRRT planner.
	 * 
	 * @param variant the applied assessment variant to be set
	 */
	public synchronized void setVariant(HRRTreeQualityVariant variant) {
		this.getQuality().setVariant(variant);
	}
	
	/**
	 * Gets the limit of neighbors to consider for extension by this hRRT
	 * planner.
	 * 
	 * @return the limit neighbors to consider for extension by this hRRT
	 *         planner
	 */
	public synchronized int getNeighborLimit() {
		return this.neighborLimit;
	}
	
	/**
	 * Sets the limit of neighbors to consider for extension by this hRRT
	 * planner.
	 * 
	 * @param neighborLimit the limit of neighbors to be set
	 * 
	 * @throws IllegalArgumentException if neighbor limit is invalid
	 */
	public synchronized void setNeighborLimit(int neighborLimit) {
		if ((0 < neighborLimit) && (Integer.MAX_VALUE >= neighborLimit)) {
			this.neighborLimit = neighborLimit;
		} else {
			throw new IllegalArgumentException("neighbor limit is invalid");
		}
	}
	
	/**
	 * Gets the quality (probability) bound of the quality assessment function
	 * of this hRRT planner.
	 * 
	 * @return the quality (probability) bound of the quality assessment
	 *         function of this hRRT planner
	 */
	public synchronized double getQualityBound() {
		return this.getQuality().getQualityBound();
	}
	
	/**
	 * Sets the quality (probability) bound of the quality assessment function
	 * of this hRRT planner.
	 * 
	 * @param qualityBound the quality (probability) bound to be set
	 */
	public synchronized void setQualityBound(double qualityBound) {
		this.getQuality().setQualityBound(qualityBound);
	}
	
	/**
	 * Gets the nearest (quality) hRRT waypoint to extend from in the tree.
	 * 
	 * @return the nearest (quality) hRRT waypoint to extend from in the tree
	 */
	protected HRRTreeWaypoint getNearestWaypoint() {
		return this.nearestWaypoint;
	}
	
	/**
	 * Sets the nearest (quality) hRRT waypoint to extend from in the tree.
	 * 
	 * @param nearestWaypoint the nearest (quality) hRRT waypoint to be set
	 */
	protected void setNearestWaypoint(HRRTreeWaypoint nearestWaypoint) {
		this.nearestWaypoint = nearestWaypoint;
	}
	
	/**
	 * Selects a sampled waypoint whose nearest neighbor's quality assessment
	 * satisfies a random quality threshold. 
	 * 
	 * @return the sampled waypoint whose nearest neighbor's quality assessment
	 *         satisfies a random quality threshold
	 */
	protected HRRTreeWaypoint selectWaypoint() {
		HRRTreeWaypoint sample = null, nearest = null;
		
		Random random = new Random();
		double quality = 0d, threshold = 1d;
		
		while (threshold > quality) {
			sample = (HRRTreeWaypoint) super.sampleBiased();
			nearest = (HRRTreeWaypoint) this.getPlanningContinuum()
					.findNearest(sample, 1).iterator().next();
			quality = this.getQuality().get(nearest);
			threshold = random.nextDouble();
		}
		
		this.setNearestWaypoint(nearest);
		return sample;
	}
	
	/**
	 * Selects a sampled waypoint for which one of its k-nearest neighbors's
	 * quality assessment satisfies a random quality threshold. The k-nearest
	 * neighbors are considered according to their quality priority order.
	 * 
	 * @return the sampled waypoint for which one of its k-nearest neighbor's
	 *         quality assessment satisfies a random quality threshold
	 */
	@SuppressWarnings("unchecked")
	protected HRRTreeWaypoint selectWaypointIk() {
		HRRTreeWaypoint sample = null;
		PriorityQueue<HRRTreeWaypoint> neighbors =
				new PriorityQueue<HRRTreeWaypoint>(Comparator.comparingDouble(this.getQuality()));
		
		Random random = new Random();
		
		while (true) {
			sample = (HRRTreeWaypoint) super.sampleBiased();
			neighbors.clear();
			neighbors.addAll((Set<HRRTreeWaypoint>) this.getPlanningContinuum()
					.findNearest(sample, this.getNeighborLimit()));
			
			for (HRRTreeWaypoint neighbor : neighbors) {
				double quality = this.getQuality().get(neighbor);
				double threshold = random.nextDouble();
				if (threshold < quality) {
					this.setNearestWaypoint(neighbor);
					return sample;
				}
			}
		}
	}
	
	/**
	 * Selects a sampled waypoint for which the best of its k-nearest
	 * neighbor's quality assessment satisfies a random quality threshold.
	 * 
	 * @return the sampled waypoint for which the best of its k-nearest
	 *         neighbor's quality assessment satisfies a random quality
	 *         threshold
	 */
	@SuppressWarnings("unchecked")
	protected HRRTreeWaypoint selectWaypointBk() {
		HRRTreeWaypoint sample = null, nearest = null;
		PriorityQueue<HRRTreeWaypoint> neighbors =
				new PriorityQueue<HRRTreeWaypoint>(Comparator.comparingDouble(this.getQuality()));
		
		Random random = new Random();
		double quality = 0d, threshold = 1d;
		
		while (threshold > quality) {
			sample = (HRRTreeWaypoint) super.sampleBiased();
			neighbors.clear();
			neighbors.addAll((Set<HRRTreeWaypoint>) this.getPlanningContinuum()
					.findNearest(sample, this.getNeighborLimit()));
			nearest = neighbors.poll();
			quality = this.getQuality().get(nearest);
			threshold = random.nextDouble();
		}
		
		this.setNearestWaypoint(nearest);
		return sample;
	}
	
	/**
	 * Attempts to connect the tree to a given waypoint by repeatedly extending
	 * towards it until it has been reached or the tree branch is trapped.
	 * 
	 * @param waypoint the waypoint to connect the tree to
	 * 
	 * @return the status of the last extension towards the waypoint
	 */
	@Override
	protected Status connectRRT(RRTreeWaypoint waypoint) {
		Status status = Status.ADVANCED;
		
		while ((Status.ADVANCED == status) && (!this.isInGoalRegion(this.getNewestWaypoint()))) {
			status = this.extendRRT(waypoint);
			this.setNearestWaypoint(this.getNewestWaypoint());
		}
		
		return status;
	}
	
	/**
	 * Extends the tree in the direction of a given waypoint.
	 * 
	 * @param waypoint the waypoint to extend the tree towards
	 * 
	 * @return the status of the extension towards the waypoint
	 */
	@Override
	protected Status extendRRT(RRTreeWaypoint waypoint) {
		Status status = Status.TRAPPED;
		
		// create a new edge in the tree by extending from nearest to sampled
		Optional<Edge> extension = this.createExtension(this.getNearestWaypoint(), waypoint);
		
		if (extension.isPresent()) {
			this.getQuality().setMaximumCost(Math.max(
					this.getQuality().getMaximumCost(),
					this.getNewestWaypoint().getF()));
			
			if (this.getNewestWaypoint().equals(waypoint)) {
				status = Status.REACHED;
			} else {
				status = Status.ADVANCED;
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
			this.getNewestWaypoint().setH(this.computeHeuristic(
					this.getNewestWaypoint(), this.getGoal()));
		}
		
		return extension;
	}
	
	/**
	 * Initializes this hRRT planner to plan from an origin to a destination at
	 * a specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 */
	@Override
	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {
		this.clearExpendables();
		
		this.setGoal(this.createWaypoint(destination));
		this.getGoal().setH(0d);
		
		this.setStart(this.createWaypoint(origin));
		this.getStart().setEto(etd);
		this.getStart().setG(0d);
		this.getStart().setH(this.computeHeuristic(this.getStart(), this.getGoal()));
		
		this.getPlanningContinuum().addVertex(this.getStart());
		this.setNewestWaypoint(this.getStart());
		
		this.getQuality().setOptimalCost(this.getStart().getF());
		this.getQuality().setMaximumCost(this.getStart().getF());
	}
	
	/**
	 * Computes a hRRT plan by growing a tree until the goal region is reached.
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
			
			// select a new sample
			HRRTreeWaypoint sample = null;
			switch (this.getAlgorithm()) {
			case BkRRT:
				sample = this.selectWaypointBk();
				break;
			case IkRRT:
				sample = this.selectWaypointIk();
				break;
			case hRRT:
			default:
				sample = this.selectWaypoint();
			}
			
			// connect to or extend towards sample according to strategy
			Status status = Status.TRAPPED;
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
				if (this.getNewestWaypoint().equals(this.getGoal())) {
					this.setGoal(this.getNewestWaypoint());
				} else {
					// TODO: possible feasibility / capability issues
					this.createExtension(this.getNewestWaypoint(), this.getGoal());
				}
				this.connectPlan();
			}
			
			iteration++;
		}
		
		this.disposeSamplingShape();
		
		if (!this.isInGoalRegion()) {
			Logging.logger().info("no trajectory found after " + this.getMaxIterations() + " iterations");
		}
		
		return this.isInGoalRegion();
	}
	
	/**
	 * Determines whether or not this hRRT planner matches a specification.
	 * 
	 * @param specification the specification to be matched
	 * 
	 * @return true if the this hRRT planner matches the specification,
	 *         false otherwise
	 * 
	 * @see RRTreePlanner#matches(Specification)
	 */
	@Override
	public synchronized boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = super.matches(specification);
		
		if (matches && (specification.getProperties() instanceof HRRTreeProperties)) {
			HRRTreeProperties properties = (HRRTreeProperties) specification.getProperties();
			matches = (this.getAlgorithm().equals(properties.getAlgorithm()))
					&& (this.getVariant().equals(properties.getVariant()))
					&& (this.getNeighborLimit() == properties.getNeighborLimit())
					&& (this.getQualityBound() == properties.getQualityBound());
		}
		
		return matches;
	}
	
	/**
	 * Updates this hRRT planner according to a specification.
	 * 
	 * @param specification the specification to be used for the update
	 * 
	 * @return true if this hRRT planner has been updated, false otherwise
	 * 
	 * @see RRTreePlanner#update(Specification)
	 */
	@Override
	public synchronized boolean update(Specification<? extends FactoryProduct> specification) {
		boolean updated = super.update(specification);
		
		if (updated && (specification.getProperties() instanceof HRRTreeProperties)) {
			HRRTreeProperties properties = (HRRTreeProperties) specification.getProperties();
			this.setAlgorithm(properties.getAlgorithm());
			this.setVariant(properties.getVariant());
			this.setNeighborLimit(properties.getNeighborLimit());
			this.setQualityBound(properties.getQualityBound());
		}
		
		return updated;
	}
	
}
