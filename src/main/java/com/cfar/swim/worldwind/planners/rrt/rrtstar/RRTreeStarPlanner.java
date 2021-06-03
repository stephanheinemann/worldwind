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
package com.cfar.swim.worldwind.planners.rrt.rrtstar;

import java.util.Set;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.CapabilitiesException;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.planners.AbstractPlanner;
import com.cfar.swim.worldwind.planners.rrt.Status;
import com.cfar.swim.worldwind.planners.rrt.brrt.RRTreePlanner;
import com.cfar.swim.worldwind.planners.rrt.brrt.RRTreeWaypoint;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.planners.rrt.RRTreeProperties;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.util.Logging;

/**
 * Realizes a RRT* planner that plans a trajectory of an aircraft in an
 * environment considering a local cost and risk policy. The tree is grown from
 * the origin to the goal by connecting a new sample to the best waypoint
 * already in the tree. As new samples are taken, connections in the tree are
 * evaluated and may be replaced by better solutions. As time advances the
 * generated solution trajectory tends to be optimal.
 * 
 * @author Manuel Rosa
 * @author Stephan Heinemann
 *
 */
public class RRTreeStarPlanner extends RRTreePlanner {
	
	/**
	 * Constructs a RRT* planner for a specified aircraft and environment using
	 * default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see RRTreePlanner#RRTreePlanner(Aircraft, Environment)
	 */
	public RRTreeStarPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}
	
	/**
	 * Probes the estimated cost from a source to a target waypoint within the
	 * environment of this RRT* planner.
	 * 
	 * @param source the source waypoint in globe coordinates
	 * @param target the target waypoint in globe coordinates
	 * 
	 * @return the estimated cost from a source to a target waypoint within the
	 *         environment of this RRT* planner
	 */
	protected double probeCost(RRTreeWaypoint source, RRTreeWaypoint target) {
		double cost = source.getCost();
		
		if (this.getEnvironment().collidesTerrain(source, target)) {
			cost = Double.POSITIVE_INFINITY;
		} else {
			cost += this.getEnvironment().getLegCost(
					source, target,
					source.getEto(), target.getEto(),
					this.getCostPolicy(), this.getRiskPolicy());
		}
		
		return cost;
	}
	
	/**
	 * Improves the parent of a target waypoint to become the least costly one
	 * from a set of potential sources in the tree within the maximum extension
	 * distance.
	 * 
	 * @param target the target waypoint
	 * @param sources the potential source waypoints
	 */
	protected void improveParent(Set<RRTreeWaypoint> sources, RRTreeWaypoint target) {
		RRTreeWaypoint parent = target.getParent();
		double minCost = target.getCost();
		
		for (RRTreeWaypoint source : sources) {
			if (!source.equals(parent)) {
				Position position = null;
				switch (this.getExtension()) {
				case FEASIBLE:
					position = this.growFeasiblePosition(source, target);
					break;
				case LINEAR:
				default:
					position = this.growPosition(source, target);
				}
				
				// target can be reached in one step
				if (position.equals(target)) {
					double cost = this.probeCost(source, target);
					if (cost < minCost) {
						try {
							// compute ETO and check extension feasibility
							// according to aircraft capabilities
							this.computeEto(source, target);
							
							// establish improved parent
							this.getEnvironment().removeEdge(parent, target);
							target.setParent(source);
							this.getEnvironment().addEdge(source, target);
							this.computeCost(source, target);
							minCost = target.getCost();
							parent = source;
						} catch (CapabilitiesException ce) {
							Logging.logger().info(ce.getMessage());
						}
					}
				}
			}
		}
	}
	
	/**
	 * Rewires a set of potential target waypoints in the tree within the
	 * maximum extension distance of an improved source as well as their
	 * dependents in case of a cost improvement.
	 * 
	 * @param source the improved source waypoint
	 * @param targets the potential target waypoints
	 */
	protected void rewireTree(RRTreeWaypoint source, Set<RRTreeWaypoint> targets) {
		
		for (RRTreeWaypoint target : targets) {
			if (!target.equals(source.getParent())) {
				Position position = null;
				switch (this.getExtension()) {
				case FEASIBLE:
					position = this.growFeasiblePosition(source, target);
					break;
				case LINEAR:
				default:
					position = this.growPosition(source, target);
				}
				
				// target can be reached in one step
				if (position.equals(target)) {
					double cost = probeCost(source, target);
					if (cost < target.getCost()) {
						try {
							// compute ETO and check extension feasibility
							// according to aircraft capabilities
							this.computeEto(source, target);
							
							// re-wire dependent tree
							this.getEnvironment().removeEdge(target.getParent(), target);
							target.setParent(source);
							this.getEnvironment().addEdge(source, target);
							this.computeCost(source, target);
							this.propagateChanges(target);
						} catch (CapabilitiesException ce) {
							Logging.logger().info(ce.getMessage());
						}
					}
				}
			}
		}
	}
	
	/**
	 * Propagates changes of a source waypoint in the tree to its dependents.
	 * 
	 * @param source the source waypoint with changes to be propagated
	 */
	protected void propagateChanges(RRTreeWaypoint source) {
		for (Position position : this.getEnvironment().getVertexIterable()) {
			RRTreeWaypoint target = (RRTreeWaypoint) position;
			if (target.hasParent() && target.getParent().equals(source)) {
				// TODO: IAE - edge not present?, SOE - termination / cycles?
				/*
				 * NOTE: ETO changes may negatively affect dependent costs.
				 * There is no guarantee for increased overall optimality in
				 * the space-time continuum. Even if propagation continues only
				 * if the entire subtree is improved, it does not guarantee
				 * improved trajectories for consecutive samples at future ETOs.
				 * An improved parent (local improvement) does not necessarily
				 * lead to a global improvement based on different ETOs. A
				 * subtree improvement may lead to a global regression based
				 * on different ETOs. A subtree regression may lead to a global
				 * improvement based o different ETOs. Can this algorithm be
				 * implemented without frequent roll-backs?
				 */
				this.computeEto(source, target);
				this.computeCost(source, target);
				this.propagateChanges(target);
			}
		}
	}
	
	/**
	 * Computes a RRT* plan by growing and improving a tree to the goal region
	 * for a maximum number of iterations.
	 * 
	 * @return true if the goal region was reached, false otherwise
	 */
	@SuppressWarnings("unchecked")
	@Override
	protected boolean compute() {
		this.createSamplingShape();

		for (int i = 0; i < this.getMaxIterations(); i++) {
			// sample a new waypoint
			RRTreeWaypoint sample = this.sampleBiased();
			
			// connect to or extend towards sample according to strategy
			Status status;
			switch (this.getStrategy()) {
			case CONNECT:
				status = this.connectRRT(sample);
				break;
			case EXTEND:
			default:
				status = this.extendRRT(sample);
			}
			
			// improve and check goal region
			if (Status.TRAPPED != status) {
				RRTreeWaypoint endWaypoint = this.getNewestWaypoint();
				Set<RRTreeWaypoint> nearestWaypoints = (Set<RRTreeWaypoint>)
						this.getEnvironment().findNearest(endWaypoint,
								this.getEnvironment().getOptimalNumNearest());
								//this.getEnvironment().getNumVertices());
				this.improveParent(nearestWaypoints, endWaypoint);
				this.rewireTree(endWaypoint, nearestWaypoints);
				
				if (this.isInGoalRegion()
						&& (endWaypoint.getCost() < this.getGoal().getCost())) {
					this.setGoal(endWaypoint);
					this.connectPlan(this.getGoal());
					this.revisePlan(this.createTrajectory());
				}
			}
		}
		
		this.disposeSamplingShape();
		
		// determine if goal region has been reached
		if (!this.getGoal().hasParent()) {
			Logging.logger().info("no trajectory found after " + this.getMaxIterations() + " iterations");
		}
		
		return this.getGoal().hasParent();
	}
	
	/**
	 * Determines whether or not this RRT* planner matches a specification.
	 * 
	 * @param specification the specification to be matched
	 * 
	 * @return true if the this RRT* planner matches the specification,
	 *         false otherwise
	 * 
	 * @see AbstractPlanner#matches(Specification)
	 */
	@Override
	public boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = false;
		
		if ((null != specification) && (specification.getProperties() instanceof RRTreeProperties)) {
			RRTreeProperties rrtp = (RRTreeProperties) specification.getProperties();
			matches = (this.getCostPolicy().equals(rrtp.getCostPolicy()))
					&& (this.getRiskPolicy().equals(rrtp.getRiskPolicy()))
					&& (this.getBias() == rrtp.getBias())
					&& (this.getEpsilon() == rrtp.getEpsilon())
					&& (this.getExtension() == rrtp.getExtension())
					&& (this.getGoalThreshold() == rrtp.getGoalThreshold())
					&& (this.getMaxIterations() == rrtp.getMaxIterations())
					&& (this.getSampling() == rrtp.getSampling())
					&& (this.getStrategy() == rrtp.getStrategy())
					&& (specification.getId().equals(Specification.PLANNER_RRTS_ID));
		}
		
		return matches;
	}
	
}
