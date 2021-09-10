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
package com.cfar.swim.worldwind.managers;

import java.time.Duration;
import java.util.Set;

import com.cfar.swim.worldwind.managing.Features;
import com.cfar.swim.worldwind.managing.PlannerTuning;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.session.Scenario;
import com.cfar.swim.worldwind.session.Session;

/**
 * Describes a autonomic manager for motion planners operating in autonomic
 * contexts aggregated by scenarios of a planner session.
 * 
 * @author Stephan Heinemann
 *
 * @see Session
 * @see Scenario
 */
public interface AutonomicManager extends FactoryProduct {
	
	/**
	 * Gets the cost policy of this autonomic manager.
	 * 
	 * @return the cost policy of this autonomic manager
	 */
	public CostPolicy getCostPolicy();
	
	/**
	 * Sets the cost policy of this autonomic manager.
	 * 
	 * @param costPolicy the cost policy to be set
	 */
	public void setCostPolicy(CostPolicy costPolicy);
	
	/**
	 * Gets the risk policy of this autonomic manager.
	 * 
	 * @return the risk policy of this autonomic manager
	 */
	public RiskPolicy getRiskPolicy();
	
	/**
	 * Sets the risk policy of this autonomic manager.
	 * 
	 * @param riskPolicy the risk policy to be set
	 */
	public void setRiskPolicy(RiskPolicy riskPolicy);
	
	/**
	 * Gets the feature horizon of this autonomic manager.
	 * 
	 * @return the feature horizon of this autonomic manager
	 */
	public Duration getFeatureHorizon();
	
	/**
	 * Sets the feature horizon of this autonomic manager.
	 * 
	 * @param featureHorizon the feature horizon to be set
	 */
	public void setFeatureHorizon(Duration featureHorizon);
	
	/**
	 * Gets the managed scenarios of this autonomic manager.
	 * 
	 * @return the managed scenarios of this autonomic manager
	 */
	public Set<Scenario> getManagedScenarios();
	
	/**
	 * Creates a new planner tuning for this autonomic manager based
	 * on a planner specification and features.
	 * 
	 * @param specification the planner specification
	 * @param features the features
	 * 
	 * @return the created planner tuning
	 * 
	 * @see PlannerTuning#PlannerTuning(Specification, Features)
	 */
	public PlannerTuning createPlannerTuning(
			Specification<Planner> specification, Features features);
	
	/**
	 * Gets the planner tuning of a managed scenario of this autonomic
	 * manager.
	 * 
	 * @param managedScenario the managed scenario
	 * 
	 * @return the planner tuning of the managed scenario, null if the scenario
	 *         is not managed by this autonomic manager
	 */
	public PlannerTuning getPlannerTuning(Scenario managedScenario);
	
	/**
	 * Manages a session identified by a session identifier.
	 * 
	 * @param sessionId the identifier of the session to be managed
	 */
	public void manage(String sessionId);
	
	/**
	 * Manages a specified session.
	 * 
	 * @param session the session to be managed
	 */
	public void manage(Session session);
	
	/**
	 * Abandons the managed session without terminating its planners.
	 */
	public void abandon();
	
	/**
	 * Terminates the managed session.
	 */
	public void terminate();
	
}
