/**
 * Copyright (c) 2016, Stephan Heinemann (UVic Center for Aerospace Research)
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
package com.cfar.swim.worldwind.ai;

import java.util.LinkedList;
import java.util.List;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.planning.Trajectory;

import gov.nasa.worldwind.geom.Position;

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
	
	/** the plan revision listeners of this abstract planner */
	private final List<PlanRevisionListener> planRevisionListeners = new LinkedList<>();
	
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
	 * Gets the cost policy of this abstract planner.
	 * 
	 * @return the cost policy of this abstract planner
	 * 
	 * @see Planner#getCostPolicy()
	 */
	@Override
	public CostPolicy getCostPolicy() {
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
	public void setCostPolicy(CostPolicy costPolicy) {
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
	public RiskPolicy getRiskPolicy() {
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
	public void setRiskPolicy(RiskPolicy riskPolicy) {
		this.riskPolicy = riskPolicy;
	}
	
	/**
	 * Gets the list of plan revision listeners of this abstract planner.
	 * 
	 * @return the planRevisionListeners the list of plan revision listeners
	 * 
	 * @see Planner#getPlanRevisionListeners()
	 */
	@Override
	public List<PlanRevisionListener> getPlanRevisionListeners() {
		return planRevisionListeners;
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
		for (PlanRevisionListener listener : this.planRevisionListeners) {
			listener.revisePlan(trajectory);
		}
	}
	
	/**
	 * Revises a plan notifying the plan revision listeners of this abstract
	 * planner.
	 */
	protected void reviseObstacle() {
		for (PlanRevisionListener listener : this.planRevisionListeners) {
			listener.reviseObstacle();
		}
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

}
