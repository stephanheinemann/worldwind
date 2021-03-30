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
package com.cfar.swim.worldwind.registries.planners;

import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;

/**
 * Realizes the properties bean of a forward A* planner.
 * 
 * @author Stephan Heinemann
 *
 */
public class ForwardAStarProperties extends AbstractPlannerProperties {

	/**
	 * Constructs a new forward A* planner properties bean.
	 */
	public ForwardAStarProperties() {
		super();
	}

	/**
	 * Constructs a new forward A* planner properties bean with
	 * specified cost and risk policy property values.
	 * 
	 * @param costPolicy the cost policy of this forward A* planner properties bean
	 * @param riskPolicy the risk policy of this forward A* planner properties bean
	 */
	public ForwardAStarProperties(CostPolicy costPolicy, RiskPolicy riskPolicy) {
		super(costPolicy, riskPolicy);
	}
	
	/**
	 * Determines whether or not this forward A* planner properties bean equals
	 * another forward A* planner properties bean based on their aggregated
	 * properties.
	 * 
	 * @param o the other forward A* planner properties bean
	 * 
	 * @return true, if the aggregated properties of this forward A* planner
	 *         properties bean equal the aggregated properties of the other
	 *         forward A* planner properties bean, false otherwise
	 * 
	 * @see AbstractPlannerProperties#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = super.equals(o);
		
		if (equals) {
			equals = (this.getClass() == o.getClass());
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this forward A* planner properties bean based on
	 * its aggregated properties.
	 * 
	 * @return the hash code of this forward A* planner properties bean based
	 *         on its aggregated properties
	 * 
	 * @see AbstractPlannerProperties#hashCode()
	 */
	@Override
	public int hashCode() {
		return super.hashCode();
	}

}
