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
 * @author Manuel
 *
 */
public class OARRTreeProperties extends ARRTreeProperties implements AnytimePlannerProperties, OnlinePlannerProperties {
	
	/** the description of this planner properties bean */
	private final static String DESCRIPTION_OARRT = "Online Anytime RRT: Online version with anytime capabilities of the RRT. It repeatedly computes better solutions as long until the connected aircraft is inside the goal region. Requires a datalink connection.";
	
	/** the state of the online capabilities of the planner */
	private boolean online = false;
	
	/** the distance threshold to consider a position displacement as worthy of a new plan */
	private double positionThreshold = 2d; 

	/**
	 * Constructs a new online anytime RRTree planner properties bean.
	 */
	public OARRTreeProperties() {
		super();
		this.setDescription(DESCRIPTION_OARRT);
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
	public OARRTreeProperties( CostPolicy costPolicy, RiskPolicy riskPolicy,
			Strategy strategy, int maxIter, double epsilon, int bias) {
		super(costPolicy, riskPolicy, strategy, maxIter, epsilon, bias);
		this.setDescription(DESCRIPTION_OARRT);
	}
	
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