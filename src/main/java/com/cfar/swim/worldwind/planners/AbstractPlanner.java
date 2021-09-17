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
package com.cfar.swim.worldwind.planners;

import java.time.Duration;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.planners.AbstractPlannerProperties;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;
import gov.nasa.worldwind.util.Logging;

/**
 * Abstracts a motion planner for an aircraft in an environment using cost
 * and risk policies.
 * 
 * @author Stephan Heinemann
 *
 */
public abstract class AbstractPlanner implements Planner {

	/** the aircraft of this abstract planner */
	private Aircraft aircraft = null;
	
	/** the environment of this abstract planner */
	private Environment environment = null;
	
	/** the cost policy of this abstract planner */
	private CostPolicy costPolicy = CostPolicy.AVERAGE;
	
	/** the risk policy of this abstract planner */
	private RiskPolicy riskPolicy = RiskPolicy.SAFETY;
	
	/** the computed plan of this abstact planner */
	private final LinkedList<Waypoint> plan = new LinkedList<>();
	
	/** the plan revision listeners of this abstract planner */
	private final List<PlanRevisionListener> planRevisionListeners = new LinkedList<>();
	
	/** indicates whether or not revision notifications are enabled */
	private boolean revisionsEnabled = true;
	
	/**
	 * Constructs a motion planner with a specified aircraft and environment.
	 * 
	 * @param aircraft the aircraft for planning
	 * @param environment the environment for planning
	 */
	public AbstractPlanner(Aircraft aircraft, Environment environment) {
		this.aircraft = aircraft;
		this.environment = environment;
	}
	
	/**
	 * Gets the aircraft of this abstract planner.
	 * 
	 * @return the aircraft of this abstract planner
	 * 
	 * @see Planner#getAircraft()
	 */
	@Override
	public Aircraft getAircraft() {
		return this.aircraft;
	}
	
	/**
	 * Sets the aircraft of this abstract planner.
	 * 
	 * @param aircraft the aircraft to be set
	 * 
	 * @see Planner#setAircraft(Aircraft)
	 */
	@Override
	public void setAircraft(Aircraft aircraft) {
		this.aircraft = aircraft;
	}
	
	/**
	 * Gets the environment of this abstract planner.
	 * 
	 * @return the environment of this abstract planner
	 * 
	 * @see Planner#getEnvironment()
	 */
	@Override
	public Environment getEnvironment() {
		return this.environment;
	}
	
	/**
	 * Sets the environment of this abstract planner
	 * 
	 * @param environment the environment to be set
	 * 
	 * @see Planner#setEnvironment(Environment)
	 */
	@Override
	public void setEnvironment(Environment environment) {
		this.environment = environment;
	}
	
	/**
	 * Gets the cost policy of this abstract planner.
	 * 
	 * @return the cost policy of this abstract planner
	 * 
	 * @see Planner#getCostPolicy()
	 */
	@Override
	public synchronized CostPolicy getCostPolicy() {
		return this.costPolicy;
	}
	
	/**
	 * Sets the cost policy of this abstract planner.
	 * 
	 * @param costPolicy the cost policy of this abstract planner
	 * 
	 * @see Planner#setCostPolicy(CostPolicy)
	 */
	@Override
	public synchronized void setCostPolicy(CostPolicy costPolicy) {
		this.costPolicy = costPolicy;
	}
	
	/**
	 * Gets the risk policy of this abstract planner.
	 * 
	 * @return the risk policy of this abstract planner
	 * 
	 * @see Planner#getRiskPolicy()
	 */
	@Override
	public synchronized RiskPolicy getRiskPolicy() {
		return this.riskPolicy;
	}
	
	/**
	 * Sets the risk policy of this abstract planner.
	 * 
	 * @param riskPolicy the risk policy of this abstract planner
	 * 
	 * @see Planner#setRiskPolicy(RiskPolicy)
	 */
	@Override
	public synchronized void setRiskPolicy(RiskPolicy riskPolicy) {
		this.riskPolicy = riskPolicy;
	}
	
	/**
	 * Computes the estimated time over a target waypoint when traveled to from
	 * a source waypoint considering the applicable aircraft capabilities.
	 * 
	 * @param source the source waypoint with known ETO
	 * @param target the target waypoint with unknown ETO
	 */
	protected void computeEto(Waypoint source, Waypoint target) {
		Path leg = new Path(source, target);
		Capabilities capabilities = this.getAircraft().getCapabilities();
		Globe globe = this.getEnvironment().getGlobe();
		target.setEto(capabilities.getEstimatedTime(leg, globe, source.getEto()));
	}
	
	/**
	 * Computes the estimated cost of a target waypoint when traveled to from
	 * a source waypoint considering the operational cost and risk policies.
	 * 
	 * @param source the source waypoint with known cost
	 * @param target the target waypoint with unknown cost
	 */
	protected void computeCost(Waypoint source, Waypoint target) {
		double cost = source.getCost();
		cost += this.getEnvironment().getStepCost(
				source, target,
				source.getEto(), target.getEto(),
				this.getCostPolicy(), this.getRiskPolicy());
		target.setCost(cost);
	}
	
	/**
	 * Clears the waypoints of the current plan of this abstact planner.
	 */
	protected void clearWaypoints() {
		this.plan.clear();
	}
	
	/**
	 * Determines whether or not the current plan of this abstact planner has
	 * waypoints.
	 * 
	 * @return true if the current plan of this abstact planner has waypoints,
	 *         false otherwise
	 */
	protected boolean hasWaypoints() {
		return !this.plan.isEmpty();
	}
	
	/**
	 * Gets the waypoints of the current plan of this abstact planner.
	 * 
	 * @return the waypoints of the current plan of this abstact planner
	 */
	protected LinkedList<Waypoint> getWaypoints() {
		return this.plan;
	}
	
	/**
	 * Creates a trajectory of the computed plan of this abstract planner.
	 * 
	 * @return the trajectory of the computed plan of this abstract planner
	 */
	protected Trajectory createTrajectory() {
		LinkedList<Waypoint> waypoints = new LinkedList<>();
		
		this.getWaypoints().descendingIterator()
			.forEachRemaining(waypoint -> {
				Waypoint current = waypoint.clone();
				if (waypoints.isEmpty()) {
					current.setTtg(Duration.ZERO);
					current.setDtg(0d);
				} else {
					// TODO: consider time and distance to next versus to goal waypoint
					current.setTtg(Duration.between(current.getEto(), waypoints.getFirst().getEto()));
					current.setDtg(this.getEnvironment().getDistance(current, waypoints.getFirst()));
				}
				waypoints.addFirst(current);
			});
		
		return new Trajectory(Collections.unmodifiableList(waypoints));
	}
	
	/**
	 * Adds a plan revision listener to this abstract planner that will be
	 * notified whenever a plan has been revised.
	 * 
	 * @param listener the plan revision listener to be added
	 * 
	 * @see Planner#addPlanRevisionListener(PlanRevisionListener)
	 * @see PlanRevisionListener
	 */
	@Override
	public void addPlanRevisionListener(PlanRevisionListener listener) {
		this.planRevisionListeners.add(listener);
	};
	
	/**
	 * Removes a plan revision listener from this abstract planner.
	 * 
	 * @param listener the plan revision listener to be removed
	 * 
	 * @see Planner#addPlanRevisionListener(PlanRevisionListener)
	 * @see PlanRevisionListener
	 */
	@Override
	public void removePlanRevisionListener(PlanRevisionListener listener) {
		this.planRevisionListeners.remove(listener);
	}
	
	/**
	 * Revises a plan notifying the plan revision listeners of this abstract
	 * planner.
	 * 
	 * @param trajectory the revised trajectory
	 */
	protected void revisePlan(Trajectory trajectory) {
		if (this.revisionsEnabled) {
			Logging.logger().info("revising plan...");
			for (PlanRevisionListener listener : this.planRevisionListeners) {
				listener.revisePlan(trajectory);
			}
		}
	}
	
	/**
	 * Enables plan revision notification.
	 */
	protected void enableRevisions() {
		this.revisionsEnabled = true;
	}
	
	/**
	 * Disables plan revision notification.
	 */
	protected void disableRevisions() {
		this.revisionsEnabled = false;
	}
	
	/**
	 * Indicates whether or not this abstract planner supports a specified
	 * aircraft.
	 * 
	 * @param aircraft the aircraft
	 * 
	 * @return true if the aircraft is non-null, false otherwise
	 * 
	 * @see Planner#supports(Aircraft)
	 */
	@Override
	public boolean supports(Aircraft aircraft) {
		return (null != aircraft);
	}
	
	/**
	 * Indicates whether or not this abstract planner supports a specified
	 * environment.
	 * 
	 * @param environment the environment
	 * 
	 * @return true if the environment is non-null, false otherwise
	 */
	@Override
	public boolean supports(Environment environment) {
		return (null != environment);
	}
	
	
	/**
	 * Indicates whether or not this abstract planner supports specified
	 * waypoints.
	 * 
	 * @param waypoints the waypoints
	 * 
	 * @return true if waypoints are contained in the planner's environment,
	 *         false otherwise
	 */
	@Override
	public boolean supports(List<Position> waypoints) {
		boolean supports = false;
		
		if ((null != this.environment) && (null != waypoints)) {
			supports = true;
			for (Position waypoint : waypoints) {
				if (!this.environment.contains(waypoint)) {
					supports = false;
					break;
				}
			}
		}
		
		return supports;
	}
	
	/**
	 * Determines whether or not this abstract planner matches a specification.
	 * 
	 * @param specification the specification to be matched
	 * 
	 * @return true if the this abstract planner matches the specification,
	 *         false otherwise
	 * 
	 * @see FactoryProduct#matches(Specification)
	 */
	@Override
	public synchronized boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = false;
		
		if ((null != specification)
				&& specification.getId().equals(this.getId())
				&& (specification.getProperties() instanceof AbstractPlannerProperties)) {
			AbstractPlannerProperties properties =
					(AbstractPlannerProperties) specification.getProperties();
			matches = this.getCostPolicy().equals(properties.getCostPolicy())
					&& this.getRiskPolicy().equals(properties.getRiskPolicy());
		}
		
		return matches;
	}
	
	/**
	 * Updates this abstract planner according to a specification.
	 * 
	 * @param specification the specification to be used for the update
	 * 
	 * @return true if this abstract planner has been updated, false otherwise
	 * 
	 * @see FactoryProduct#update(Specification)
	 */
	@Override
	public synchronized boolean update(Specification<? extends FactoryProduct> specification) {
		boolean updated = false;
		
		if ((null != specification)
				&& specification.getId().equals(this.getId())
				&& (specification.getProperties() instanceof AbstractPlannerProperties)
				&& !this.matches(specification)) {
			AbstractPlannerProperties properties =
					(AbstractPlannerProperties) specification.getProperties();
			this.setCostPolicy(properties.getCostPolicy());
			this.setRiskPolicy(properties.getRiskPolicy());
			updated = true;
		}
		
		return updated;
	}
	
}
