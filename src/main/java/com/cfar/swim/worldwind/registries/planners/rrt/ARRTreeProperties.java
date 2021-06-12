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
import javax.validation.constraints.Max;
import javax.validation.constraints.Min;

import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.registries.planners.AnytimePlannerProperties;

/**
 * Realizes the properties bean of an ARRT planner.
 * 
 * @author Manuel Rosa
 *
 */
@ARRTreeValidQuality
public class ARRTreeProperties extends RRTreeProperties implements AnytimePlannerProperties {
	
	/** the limit of neighbors to consider for extension by the ARRT planner */
	@Min(value = 1, message = "{property.planner.arrt.neighborLimit.min}")
	@Max(value = Integer.MAX_VALUE, message = "{property.planner.arrt.neighborLimit.max}")
	private int neighborLimit = 5;
	
	/** the initial bias towards cost versus distance */
	private double initialCostBias = 0d;
	
	/** the final bias towards costs versus distance */
	private double finalCostBias = 1d;
	
	/** the improvement factor for the cost of consecutive solutions */
	@DecimalMin(value = "0", message = "{property.planner.arrt.improvementFactor.min}")
	@DecimalMax(value = "1", message = "{property.planner.arrt.improvementFactor.max}")
	private double improvementFactor = 0.05d;
	
	/**
	 * Constructs a new ARRT planner properties bean.
	 */
	public ARRTreeProperties() {
		super();
	}
	
	/**
	 * Constructs a new ARRT planner properties bean with
	 * specified cost and risk policy property values.
	 * 
	 * @param costPolicy the cost policy of this ARRT planner properties bean
	 * @param riskPolicy the risk policy of this ARRT planner properties bean
	 */
	public ARRTreeProperties(CostPolicy costPolicy, RiskPolicy riskPolicy) {
		super(costPolicy, riskPolicy);
	}
	
	/**
	 * Constructs a new ARRT planner properties bean with specified cost and
	 * risk policy property values as well as specified initial, final and
	 * improvement of the cost versus goal bias.
	 * 
	 * @param costPolicy the cost policy of this ARRT planner properties bean
	 * @param riskPolicy the risk policy of this ARRT planner properties bean
	 * @param initialCostBias the initial bias towards cost versus distance
	 * @param finalCostBias the final bias towards cost versus distance
	 * @param improvementFactor the improvement factor for the cost of consecutive solutions
	 */
	public ARRTreeProperties(
			CostPolicy costPolicy, RiskPolicy riskPolicy,
			double initialCostBias,
			double finalCostBias,
			double improvementFactor) {
		super(costPolicy, riskPolicy);
		this.setMinimumQuality(initialCostBias);
		this.setMaximumQuality(finalCostBias);
		this.setQualityImprovement(improvementFactor);
	}
	
	/**
	 * Gets the limit of neighbors of this ARRT planner properties bean.
	 * 
	 * @return the limit of neighbors of this ARRT planner properties bean
	 */
	public int getNeighborLimit() {
		return this.neighborLimit;
	}
	
	/**
	 * Sets the limit of neighbors of this ARRT planner properties bean.
	 * 
	 * @param neighborLimit the limit of neighbors to be set
	 */
	public void setNeighborLimit(int neighborLimit) {
		this.neighborLimit = neighborLimit;
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
		return this.initialCostBias;
	}
	
	/**
	 * Sets the minimum quality (initial cost bias) of this ARRT planner.
	 * 
	 * @param initialCostBias the minimum quality (initial cost bias) of this
	 *                        ARRT planner
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
		return this.finalCostBias;
	}
	
	/**
	 * Sets the maximum quality (final cost bias) of this ARRT planner.
	 * 
	 * @param finalCostBias the maximum quality (final cost bias) of this ARRT
	 *                      planner
	 * 
	 * @see AnytimePlannerProperties#setMaximumQuality(double)
	 */
	@Override
	public void setMaximumQuality(double finalCostBias) {
		this.finalCostBias = finalCostBias;
	}
	
	/**
	 * Gets the quality improvement (factor) of this ARRT planner.
	 * 
	 * @return the quality improvement (factor) of this ARRT planner
	 * 
	 * @see AnytimePlannerProperties#getQualityImprovement()
	 */
	@Override
	public double getQualityImprovement() {
		return this.improvementFactor;
	}
	
	/**
	 * Sets the quality improvement (factor) of this ARRT planner.
	 * 
	 * @param improvementFactor the quality improvement (factor) of this ARRT
	 *                          planner
	 * 
	 * @see AnytimePlannerProperties#setQualityImprovement(double)
	 */
	@Override
	public void setQualityImprovement(double improvementFactor) {
		this.improvementFactor = improvementFactor;
	}
	
	/**
	 * Determines whether or not this ARRT planner properties bean equals
	 * another ARRT planner properties bean based on their aggregated
	 * properties.
	 * 
	 * @param o the other ARRT planner properties bean
	 * 
	 * @return true, if the aggregated properties of this ARRT planner
	 *         properties bean equal the aggregated properties of the other
	 *         ARRT planner properties bean, false otherwise
	 * 
	 * @see RRTreeProperties#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = super.equals(o);
		
		if (equals) {
			ARRTreeProperties arrtp = (ARRTreeProperties) o;
			equals = (this.neighborLimit == arrtp.neighborLimit)
					&& (this.initialCostBias == arrtp.initialCostBias)
					&& (this.finalCostBias == arrtp.finalCostBias)
					&& (this.improvementFactor == arrtp.improvementFactor);
		} else {
			equals = false;
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this ARRT planner properties bean based on its
	 * aggregated properties.
	 * 
	 * @return the hash code of this ARRT planner properties bean based on its
	 *         aggregated properties
	 * 
	 * @see RRTreeProperties#hashCode()
	 */
	@Override
	public int hashCode() {
		return Objects.hash(
				super.hashCode(),
				this.neighborLimit,
				this.initialCostBias,
				this.finalCostBias,
				this.improvementFactor);
	}
	
}
