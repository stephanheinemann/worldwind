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

import com.cfar.swim.worldwind.ai.rrt.basicrrt.Sampling;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Strategy;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;

/**
 * Realizes the properties bean of an anytime RRTree planner.
 * 
 * @author Manuel Rosa
 *
 */
public class ARRTreeProperties extends RRTreeProperties implements AnytimePlannerProperties, OnlinePlannerProperties {
	
	/** the sampling technique for the planner */
	private Sampling sampling = Sampling.UNIFORM;
	
	/** the description of this planner properties bean */
	private final static String DESCRIPTION_ARRT = "Anytime RRT: Anytime version of a RRT which computes plans with increasing quality as time allows until a ceratin improvement is reached.";
	
	/** the initial relative weight of costs to calculate the cost of a waypoint */
	private double initialCostBias = 0d;
	
	/** the final relative weight of costs to calculate the cost of a waypoint */
	private double finalCostBias = 1d;

	/** the improvement factor for the cost of each new generated solution */
	private double improvementFactor = 0.05;

	/**
	 * Constructs a new anytime RRTree planner properties bean.
	 */
	public ARRTreeProperties() {
		super();
		this.setDescription(DESCRIPTION_ARRT);
	}
	
	/**
	 * Constructs a new anytime RRTree planner properties bean with specified cost and
	 * risk policy property values as well as specified maximum number of iterations and expansion strategy
	 * 
	 * @param costPolicy the cost policy of this basic RRTree planner properties bean
	 * @param riskPolicy the risk policy of this basic RRTree planner properties bean
	 * @param strategy the expansion strategy for this planner
	 * @param epsilon the maximum distance to extend a waypoint in the tree
	 * @param bias the bias of the sampling algorithm towards goal
	 * @param maxIter the maximum number of sampling iterations
	 */
	public ARRTreeProperties( CostPolicy costPolicy, RiskPolicy riskPolicy,
			Strategy strategy, int maxIter, double epsilon, int bias) {
		super(costPolicy, riskPolicy, strategy, maxIter, epsilon, bias);
		this.setDescription(DESCRIPTION_ARRT);
	}
	
	

	/**
	 * Gets the sampling technique for the planner.
	 * 
	 * @return the sampling technique for the planner
	 */
	public Sampling getSampling() {
		return sampling;
	}

	/**
	 * Sets the sampling technique for the planner.
	 * 
	 * @param sampling the sampling technique for the planner
	 */
	public void setSampling(Sampling sampling) {
		this.sampling = sampling;
	}

	/**
	 * Gets the minimum quality (initial cost bias) of this ARRT planner.
	 * 
	 * @return the minimum quality (initial cost bias) of this ARRT planner
	 * 
	 * @see AnytimePlannerProperties#getMinimumQuality()
	 */
	@Override
	public double getMinimumQuality() {
		return initialCostBias;
	}

	/**
	 * Sets the minimum quality (initial inflation) of this ARRT planner.
	 * 
	 * @param initialCostBias the minimum quality (initial cost bias) of this ARRT
	 *            planner
	 * 
	 * @see AnytimePlannerProperties#setMinimumQuality(double)
	 */
	@Override
	public void setMinimumQuality(double initialCostBias) {
		this.initialCostBias = initialCostBias;
	}

	/**
	 * Gets the maximum quality (final cost bias) of this ARRT planner.
	 * 
	 * @return the maximum quality (final cost bias) of this ARRT planner
	 * 
	 * @see AnytimePlannerProperties#getMaximumQuality()
	 */
	@Override
	public double getMaximumQuality() {
		return finalCostBias;
	}

	/**
	 * Sets the maximum quality (final cost bias) of this ARRT planner.
	 * 
	 * @param finalCostBias the maximum quality (final cost bias) of this ARRT
	 *            planner
	 * 
	 * @throws IllegalArgumentException if the final inflation is invalid
	 * 
	 * @see AnytimePlannerProperties#setMaximumQuality(double)
	 */
	@Override
	public void setMaximumQuality(double finalCostBias) {
		this.finalCostBias = finalCostBias;
	}

	/**
	 * Gets the quality improvement of this ARRT planner.
	 * 
	 * @return the quality improvement of this ARRT planner
	 * 
	 * @see AnytimePlannerProperties#getQualityImprovement()
	 */
	@Override
	public double getQualityImprovement() {
		return improvementFactor;
	}

	/**
	 * Sets the quality improvement of this ARRT planner.
	 * 
	 * @param improvementFactor the quality improvement of this ARRT planner
	 * 
	 * @see AnytimePlannerProperties#setQualityImprovement(double)
	 */
	@Override
	public void setQualityImprovement(double improvementFactor) {
		this.improvementFactor = improvementFactor;
	}
	
	
	/** the state of the online capabilities of the planner */
	private boolean online = true;
	
	/** the distance threshold to consider a position displacement as worthy of a new plan */
	private double positionThreshold = 2d; 
	
	/**
	 * Checks if the online capabilities of the planner mode are active or not.
	 * 
	 * @return true if the planner mode is set to online, false otherwise
	 */
	public boolean isOnline() {
		return online;
	}
	
	/**
	 * Sets the online capabilities of the planner as are active or not.
	 * 
	 * @param online the state of the online capabilities
	 */
	public void setOnline(boolean online) {
		this.online = online;
	}

	/**
	 * Gets the distance threshold to consider a position displacement as worthy of a new plan.
	 * 
	 * @return the distance threshold for each position
	 */
	public double getPositionThreshold() {
		return positionThreshold;
	}

	/**
	 * Sets the distance threshold to consider a position displacement as worthy of a new plan.
	 * 
	 * @param positionThreshold the distance threshold for each position
	 */
	public void setPositionThreshold(double positionThreshold) {
		this.positionThreshold = positionThreshold;
	}
}
