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

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.RiskPolicy;

/**
 * Concretizes a motion planner for an aircraft in an environment using cost
 * and risk policies.
 * 
 * @author Stephan Heinemann
 *
 */
public abstract class AbstractPlanner implements Planner {

	/** the aircraft of this planner */
	private Aircraft aircraft = null;
	
	/** the environment of this planner */
	private Environment environment = null;
	
	/** the cost policy of this planner */
	private CostPolicy costPolicy = CostPolicy.AVERAGE;
	
	/** the risk policy of this planner */
	private RiskPolicy riskPolicy = RiskPolicy.SAFETY;
	
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
	 * @see Planner#getRisPolicy()
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

}
