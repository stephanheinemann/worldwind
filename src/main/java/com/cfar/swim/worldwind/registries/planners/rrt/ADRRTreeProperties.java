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
package com.cfar.swim.worldwind.registries.planners.rrt;

import java.util.Objects;

import javax.validation.constraints.DecimalMax;
import javax.validation.constraints.DecimalMin;

import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.registries.planners.DynamicPlannerProperties;

/**
 * Realizes the properties bean of an anytime dynamic RRT planner.
 * 
 * @author Stephan Heinemann
 *
 */
public class ADRRTreeProperties extends ARRTreeProperties implements DynamicPlannerProperties {
	
	/** the significant change threshold of this ADRRT planner properties bean */
	@DecimalMin(value = "0", message = "{property.planner.adrrt.significantChange.min}")
	@DecimalMax(value = "1", message = "{property.planner.adrrt.significantChange.max}")
	private double significantChange = 0.5d;
	
	/**
	 * Constructs a new ADRRT planner properties bean.
	 */
	public ADRRTreeProperties() {
		super();
	}
	
	/**
	 * Constructs a new ADRRT planner properties bean with specified cost
	 * and risk policy property values.
	 * 
	 * @param costPolicy the cost policy of this ADRRT planner properties bean
	 * @param riskPolicy the risk policy of this ADRRT planner properties bean
	 */
	public ADRRTreeProperties(CostPolicy costPolicy, RiskPolicy riskPolicy) {
		super(costPolicy, riskPolicy);
	}
	
	/**
	 * Gets the significant change threshold of this ADRRT planner properties
	 * bean.
	 * 
	 * @return the significant change threshold of this ADRRT planner properties
	 *         bean
	 * 
	 * @see DynamicPlannerProperties#getSignificantChange()
	 */
	@Override
	public double getSignificantChange() {
		return this.significantChange;
	}
	
	/**
	 * Sets the significant change threshold of this ADRRT planner properties
	 * bean.
	 * 
	 * @param significantChange the significant change threshold to be set
	 * 
	 * @see DynamicPlannerProperties#setSignificantChange(double)
	 */
	@Override
	public void setSignificantChange(double significantChange) {
		this.significantChange = significantChange;
	}
	
	/**
	 * Determines whether or not this ADRRT planner properties bean equals
	 * another ADRRT planner properties bean based on their aggregated
	 * properties.
	 * 
	 * @param o the other ADRRT planner properties bean
	 * 
	 * @return true, if the aggregated properties of this ADRRT planner
	 *         properties bean equal the aggregated properties of the other
	 *         ADRRT planner properties bean, false otherwise
	 * 
	 * @see ARRTreeProperties#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = super.equals(o);
		
		if (equals) {
			ADRRTreeProperties adrrtp = (ADRRTreeProperties) o;
			equals = (this.significantChange == adrrtp.significantChange);
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this ADRRT planner properties bean based on its
	 * aggregated properties.
	 * 
	 * @return the hash code of this ADRRT planner properties bean based on its
	 *         aggregated properties
	 * 
	 * @see ARRTreeProperties#hashCode()
	 */
	@Override
	public int hashCode() {
		return Objects.hash(
				super.hashCode(),
				this.significantChange);
	}
	
}
