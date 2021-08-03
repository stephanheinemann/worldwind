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
package com.cfar.swim.worldwind.registries.managers;

import java.util.Objects;

import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.registries.Properties;

/**
 * Abstracts manager properties applicable to all managers.
 * 
 * @author Stephan Heinemann
 *
 */
public abstract class AbstractManagerProperties implements ManagerProperties {
	
	/** the cost policy of this manager properties bean */
	CostPolicy costPolicy;
	
	/** the risk policy of this manager properties bean */
	RiskPolicy riskPolicy;
	
	/**
	 * Constructs a new manager properties bean using default cost and risk
	 * policy property values.
	 */
	public AbstractManagerProperties() {
		this.costPolicy = CostPolicy.AVERAGE;
		this.riskPolicy = RiskPolicy.SAFETY;
	}
	
	/**
	 * Constructs a new manager properties bean with specified cost and risk
	 * policy property values.
	 * 
	 * @param costPolicy the cost policy of this manager properties bean
	 * @param riskPolicy the risk policy of this manager properties bean
	 */
	public AbstractManagerProperties(CostPolicy costPolicy, RiskPolicy riskPolicy) {
		this.costPolicy = costPolicy;
		this.riskPolicy = riskPolicy;
	}
	
	/**
	 * Gets the cost policy of this manager properties bean.
	 * 
	 * @return the cost policy of this manager properties bean
	 * 
	 * @see ManagerProperties#getCostPolicy()
	 */
	@Override
	public CostPolicy getCostPolicy() {
		return this.costPolicy;
	}
	
	/**
	 * Sets the cost policy of this manager properties bean.
	 * 
	 * @param costPolicy the cost policy to be set
	 * 
	 * @see ManagerProperties#setCostPolicy(CostPolicy)
	 */
	@Override
	public void setCostPolicy(CostPolicy costPolicy) {
		this.costPolicy = costPolicy;
	}
	
	/**
	 * Gets the risk policy of this manager properties bean.
	 * 
	 * @return the risk policy of this manager properties bean
	 * 
	 * @see ManagerProperties#getRiskPolicy()
	 */
	@Override
	public RiskPolicy getRiskPolicy() {
		return this.riskPolicy;
	}
	
	/**
	 * Sets the risk policy of this manager properties bean.
	 * 
	 * @param riskPolicy the risk policy to be set
	 * 
	 * @see ManagerProperties#setRiskPolicy(RiskPolicy)
	 */
	@Override
	public void setRiskPolicy(RiskPolicy riskPolicy) {
		this.riskPolicy = riskPolicy;
	}
	
	/**
	 * Clones this manager properties bean.
	 * 
	 * @return a clone of this manager properties bean
	 * 
	 * @see Properties#clone()
	 */
	@Override
	public AbstractManagerProperties clone() {
		AbstractManagerProperties clone = null;
		try {
			clone = (AbstractManagerProperties) super.clone();
		} catch (CloneNotSupportedException e) {
			e.printStackTrace();
		}
		return clone;
	}
	
	/**
	 * Determines whether or not this abstract manager properties bean equals
	 * another abstract manager properties bean based on their aggregated
	 * properties.
	 * 
	 * @param o the other abstract manager properties bean
	 * 
	 * @return true, if the aggregated properties of this abstract manager
	 *         properties bean equal the aggregated properties of the other
	 *         abstract manager properties bean, false otherwise
	 * 
	 * @see Object#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = false;
		
		if (this == o) {
			equals = true;
		} else if ((null != o) && (this.getClass() == o.getClass())) {
			AbstractManagerProperties amp = (AbstractManagerProperties) o;
			equals = (this.costPolicy.equals(amp.costPolicy))
					&& (this.riskPolicy.equals(amp.riskPolicy));
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this abstract manager properties bean based on its
	 * aggregated properties.
	 * 
	 * @return the hash code of this abstract manager properties bean based on
	 *         its aggregated properties
	 * 
	 * @see Object#hashCode()
	 */
	@Override
	public int hashCode() {
		return Objects.hash(this.costPolicy, this.riskPolicy);
	}
	
	// TODO: managed planner error tolerances
	
}
