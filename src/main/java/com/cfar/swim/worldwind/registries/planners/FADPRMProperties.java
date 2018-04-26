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
 * @author Henrique Ferreira
 *
 */
public class FADPRMProperties extends AbstractPlannerProperties implements AnytimePlannerProperties{

	public FADPRMProperties() {
		super();
		this.setMinimumQuality(0d);
		this.setMaximumQuality(1d);
		this.setQualityImprovement(0.1);
	}
	
	public FADPRMProperties(CostPolicy costPolicy, RiskPolicy riskPolicy) {
		super(costPolicy, riskPolicy);
		this.setMinimumQuality(0d);
		this.setMaximumQuality(1d);
		this.setQualityImprovement(0.1);
	}
	
	/**
	 * Constructs a new ARA* planner properties bean with specified cost and
	 * risk policy property values as well as specified initial, final and
	 * decrease of heuristic inflation property values.
	 * 
	 * @param costPolicy the cost policy of this ARA* planner properties bean
	 * @param riskPolicy the risk policy of this ARA* planner properties bean
	 * @param initialInflation the initial inflation of the heuristic function
	 * @param finalInflation the final inflation of the heuristic function
	 * @param deflationAmount the deflation amount of the heuristic function
	 */
	public FADPRMProperties(
			CostPolicy costPolicy, RiskPolicy riskPolicy,
			double initialInflation,
			double finalInflation,
			double deflationAmount) {
		super(costPolicy, riskPolicy);
		this.setMinimumQuality(initialInflation);
		this.setMaximumQuality(finalInflation);
		this.setQualityImprovement(deflationAmount);
	}
	
	/** the initial inflation factor applied to the heuristic function */
	private double initialInflation=0d;
	
	/** the final inflation factor applied the heuristic function */
	private double finalInflation=1d;
	
	/** the deflation amount to be applied to the current inflation */
	private double inflationAmount=0.1;
	
	/**
	 * Gets the minimum quality (initial inflation) of this
	 * ARA* properties bean.
	 * 
	 * @return the minimum quality (initial inflation) of this
	 *         ARA* properties bean
	 * 
	 * @see AnytimePlannerProperties#getMinimumQuality()
	 */
	@Override
	public double getMinimumQuality() {
		return this.initialInflation;
	}
	
	/**
	 * Sets the minimum quality (initial inflation) of this
	 * ARA* properties bean.
	 * 
	 * @param initialInflation the minimum quality (initial inflation) of this
	 *                         ARA* properties bean
	 * 
	 * @throws IllegalArgumentException if the initial inflation is invalid
	 * 
	 * @see AnytimePlannerProperties#setMinimumQuality(double)
	 */
	@Override
	public void setMinimumQuality(double initialInflation) {
		if ((0d <= initialInflation) &&
				(initialInflation <= this.finalInflation)) {
			this.initialInflation = initialInflation;
		} else {
			throw new IllegalArgumentException("initial inflation is invalid");
		}
	}
	
	/**
	 * Gets the maximum quality (final inflation) of this ARA* properties bean.
	 * 
	 * @return the maximum quality (final inflation) of this
	 *         ARA* properties bean
	 * 
	 * @see AnytimePlannerProperties#getMaximumQuality()
	 */
	@Override
	public double getMaximumQuality() {
		return this.finalInflation;
	}
	
	/**
	 * Sets the maximum quality (initial inflation) of this
	 * ARA* properties bean.
	 * 
	 * @param finalInflation the maximum quality (final inflation) of this
	 *                       ARA* properties bean
	 * 
	 * @throws IllegalArgumentException if the final inflation is invalid
	 * 
	 * @see AnytimePlannerProperties#setMaximumQuality(double)
	 */
	@Override
	public void setMaximumQuality(double finalInflation) {
		if ((1d >= finalInflation) && (this.initialInflation <= finalInflation)) {
			this.finalInflation = finalInflation;
		} else {
			throw new IllegalArgumentException("final deflation is invalid");
		}
	}
	
	/**
	 * Gets the quality improvement (deflation amount) of this
	 * ARA* properties bean.
	 * 
	 * @return the quality improvement (deflation amount) of this
	 *         ARA* properties bean
	 * 
	 * @see AnytimePlannerProperties#getQualityImprovement()
	 */
	@Override
	public double getQualityImprovement() {
		return this.inflationAmount;
	}

	/**
	 * Sets the quality improvement (deflation amount) of this
	 * ARA* properties bean.
	 * 
	 * @param deflationAmount the quality improvement (deflation amount) of
	 *                        this ARA* properties bean
	 * 
	 * @throws IllegalArgumentException if the deflation amount is invalid
	 * 
	 * @see AnytimePlannerProperties#setQualityImprovement(double)
	 */
	@Override
	public void setQualityImprovement(double inflationAmount) {
		if (0d < inflationAmount) {
			this.inflationAmount = inflationAmount;
		} else {
			throw new IllegalArgumentException("deflation amount is invalid");
		}
	}
}
