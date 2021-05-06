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
package com.cfar.swim.worldwind.registries.planners.cgs;

import java.util.Objects;

import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.registries.planners.AnytimePlannerProperties;
import com.cfar.swim.worldwind.registries.planners.DynamicPlannerProperties;

/**
 * Realizes the properties bean of an AD* planner.
 * 
 * @author Stephan Heinemann
 *
 */
public class ADStarProperties extends ARAStarProperties
	implements AnytimePlannerProperties, DynamicPlannerProperties {

	/** the significant change threshold of this AD* planner properties bean */
	private double significantChange;
	
	/**
	 * Constructs a new AD* planner properties bean.
	 */
	public ADStarProperties() {
		super();
		this.setSignificantChange(0.5d);
		
	}
	
	/**
	 * Constructs a new AD* planner properties bean with
	 * specified cost and risk policy property values.
	 * 
	 * @param costPolicy the cost policy of this AD* planner properties bean
	 * @param riskPolicy the risk policy of this AD* planner properties bean
	 */
	public ADStarProperties(CostPolicy costPolicy, RiskPolicy riskPolicy) {
		super(costPolicy, riskPolicy);
		this.setSignificantChange(0.5d);
	}
	
	/**
	 * Constructs a new AD* planner properties bean with specified cost and
	 * risk policy property values as well as specified initial, final and
	 * decrease of heuristic inflation property values.
	 * 
	 * @param costPolicy the cost policy of this AD* planner properties bean
	 * @param riskPolicy the risk policy of this AD* planner properties bean
	 * @param initialInflation the initial inflation of the heuristic function
	 * @param finalInflation the final inflation of the heuristic function
	 * @param deflationAmount the deflation amount of the heuristic function
	 */
	public ADStarProperties(
			CostPolicy costPolicy, RiskPolicy riskPolicy,
			double initialInflation,
			double finalInflation,
			double deflationAmount) {
		super(costPolicy, riskPolicy, initialInflation, finalInflation, deflationAmount);
		this.setSignificantChange(0.5d);
	}
	
	/**
	 * Constructs a new AD* planner properties bean with specified cost and
	 * risk policy property values as well as specified initial, final and
	 * decrease of heuristic inflation property values. The significant change
	 * threshold controls inflation and deflation after dynamic changes.
	 * 
	 * @param costPolicy the cost policy of this AD* planner properties bean
	 * @param riskPolicy the risk policy of this AD* planner properties bean
	 * @param initialInflation the initial inflation of the heuristic function
	 * @param finalInflation the final inflation of the heuristic function
	 * @param deflationAmount the deflation amount of the heuristic function
	 * @param significantChange the significant change threshold
	 */
	public ADStarProperties(
			CostPolicy costPolicy, RiskPolicy riskPolicy,
			double initialInflation,
			double finalInflation,
			double deflationAmount,
			double significantChange) {
		super(costPolicy, riskPolicy, initialInflation, finalInflation, deflationAmount);
		this.setSignificantChange(significantChange);
	}
	
	/**
	 * Gets the significant change threshold of this AD* planner properties
	 * bean.
	 * 
	 * @return the significant change threshold of this AD* planner properties
	 *         bean
	 * 
	 * @see DynamicPlannerProperties#getSignificantChange()
	 */
	@Override
	public double getSignificantChange() {
		return this.significantChange;
	}
	
	/**
	 * Sets the significant change threshold of this AD* planner properties
	 * bean.
	 * 
	 * @param significantChange the significant change threshold to be set
	 * 
	 * @throws IllegalArgumentException if the significant change threshold
	 *         is invalid
	 * 
	 * @see DynamicPlannerProperties#setSignificantChange(double)
	 */
	@Override
	public void setSignificantChange(double significantChange) {
		if ((0d <= significantChange) && (1d >= significantChange)) {
			this.significantChange = significantChange;
		} else {
			throw new IllegalArgumentException("significant change is invalid");
		}
	}
	
	/**
	 * Determines whether or not this AD* planner properties bean equals
	 * another AD* planner properties bean based on their aggregated
	 * properties.
	 * 
	 * @param o the other AD* planner properties bean
	 * 
	 * @return true, if the aggregated properties of this AD* planner
	 *         properties bean equal the aggregated properties of the other
	 *         AD* planner properties bean, false otherwise
	 * 
	 * @see ARAStarProperties#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = super.equals(o);
		
		if (equals) {
			ADStarProperties adsp = (ADStarProperties) o;
			equals = (this.significantChange == adsp.significantChange);
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this AD* planner properties bean based on its
	 * aggregated properties.
	 * 
	 * @return the hash code of this AD* planner properties bean based on its
	 *         aggregated properties
	 * 
	 * @see ARAStarProperties#hashCode()
	 */
	@Override
	public int hashCode() {
		return Objects.hash(
				super.hashCode(),
				this.significantChange);
	}
	
}
