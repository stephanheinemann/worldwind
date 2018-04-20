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
package com.cfar.swim.worldwind.registries.planners;

import com.cfar.swim.worldwind.ai.rrt.basicrrt.Strategy;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;

/**
 * Realizes the properties bean of a basic RRTree planner.
 * 
 * @author Manuel Rosa
 *
 */
public class RRTreeProperties extends AbstractPlannerProperties {
	
	/** the maximum number of sampling iterations */
	private int maxIter = 3_000;

	/** the expansion strategy for the planner */
	private Strategy strategy = Strategy.EXTEND;

	/** the maximum distance to extend a waypoint in the tree */
	private double epsilon = 250d;

	/** the bias of the sampling algorithm towards goal */
	private int bias = 5;

	/**
	 * Constructs a new basic RRTree planner properties bean.
	 */
	public RRTreeProperties() {
		super();
	}
	
	/**
	 * Constructs a new basic RRTree planner properties bean with specified cost and
	 * risk policy property values.
	 * 
	 * @param costPolicy the cost policy of this basic RRTree planner properties
	 *            bean
	 * @param riskPolicy the risk policy of this basic RRTree planner properties
	 *            bean
	 */
	public RRTreeProperties(CostPolicy costPolicy, RiskPolicy riskPolicy) {
		super(costPolicy, riskPolicy);
	}
	
	/**
	 * Constructs a new basic RRTree planner properties bean with specified cost and
	 * risk policy property values as well as specified maximum number of iterations and expansion strategy
	 * 
	 * @param costPolicy the cost policy of this basic RRTree planner properties bean
	 * @param riskPolicy the risk policy of this basic RRTree planner properties bean
	 * @param strategy the expansion strategy for this planner
	 * @param epsilon the maximum distance to extend a waypoint in the tree
	 * @param bias the bias of the sampling algorithm towards goal
	 * @param maxIter the maximum number of sampling iterations
	 */
	public RRTreeProperties( CostPolicy costPolicy, RiskPolicy riskPolicy,
			Strategy strategy, int maxIter, double epsilon, int bias) {
		super(costPolicy, riskPolicy);
		this.setStrategy(strategy);
		this.setMaxIter(maxIter);
		this.setEpsilon(epsilon);
		this.setBias(bias);
	}

	/**
	 * Gets the expansion strategy for the planner
	 * 
	 * @return the expansion strategy
	 */
	public Strategy getStrategy() {
		return strategy;
	}

	/**
	 * Sets the expansion strategy for the planner
	 * 
	 * @param strategy the expansion strategy to set
	 */
	public void setStrategy(Strategy strategy) {
		this.strategy = strategy;
	}

	/**
	 * Gets the maximum number of iterations for the planner to attempt to connect
	 * to goal
	 * 
	 * @return the maximum number of sampling iterations
	 */
	public int getMaxIter() {
		return maxIter;
	}

	/**
	 * Sets the maximum number of iterations for the planner to attempt to connect
	 * to goal
	 * 
	 * @param maxIter the maximum number of sampling iterations
	 */
	public void setMaxIter(int maxIter) {
		this.maxIter = maxIter;
	}

	/**
	 * Gets the maximum distance to extend a waypoint in the tree
	 * 
	 * @return the maximum distance to extend
	 */
	public double getEpsilon() {
		return epsilon;
	}

	/**
	 * Sets the maximum distance to extend a waypoint in the tree
	 * 
	 * @param epsilon the maximum distance to extend
	 */
	public void setEpsilon(double epsilon) {
		this.epsilon = epsilon;
	}

	/**
	 * Gets the bias of the sampling algorithm towards goal
	 * 
	 * @return the bias of the sampling algorithm
	 */
	public int getBias() {
		return bias;
	}

	/**
	 * Sets the bias of the sampling algorithm towards goal
	 * 
	 * @param bias the bias of the sampling algorithm
	 */
	public void setBias(int bias) {
		this.bias = bias;
	}
	
	

}
