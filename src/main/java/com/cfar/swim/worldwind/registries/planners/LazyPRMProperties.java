/**
 * Copyright (c) 2018, Henrique Ferreira (UVic Center for Aerospace Research)
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

import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;

/**
 * Realizes the properties bean of a basic PRM planner.
 * 
 * @author Henrique Ferreira
 *
 */
public class LazyPRMProperties extends BasicPRMProperties {

	/**
	 * Constructs a new basic PRM planner properties bean.
	 */
	public LazyPRMProperties() {
		super();
	}

	/**
	 * Constructs a new basic PRM planner properties bean with
	 * specified cost and risk policy property values.
	 * 
	 * @param costPolicy the cost policy of this basic PRM planner properties bean
	 * @param riskPolicy the risk policy of this basic PRM planner properties bean
	 */
	public LazyPRMProperties(CostPolicy costPolicy, RiskPolicy riskPolicy) {
		super(costPolicy, riskPolicy);
	}
	
	/**
	 * Constructs a new basic PRM planner properties bean with specified cost and
	 * risk policy property values as well as specified maximum number of
	 * iterations, maximum number of neighbors and maximum distance.
	 * 
	 * @param costPolicy the cost policy of this basic PRM planner properties
	 *            bean
	 * @param riskPolicy the risk policy of this basic PRM planner properties
	 *            bean
	 * @param maxIter the maximum number of sampling iterations
	 * @param maxNeighbors the maximum number of neighbors a waypoint can have
	 * @param maxDistance the maximum distance between two connected waypoints
	 */
	public LazyPRMProperties( CostPolicy costPolicy, RiskPolicy riskPolicy,
			int maxIter, int maxNeighbors, double maxDistance) {
		super(costPolicy, riskPolicy, maxIter, maxNeighbors, maxDistance);
	}

}