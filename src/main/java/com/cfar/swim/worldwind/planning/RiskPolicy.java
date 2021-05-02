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
package com.cfar.swim.worldwind.planning;

/**
 * Enumerates the planning risk policies for taking local risk in an
 * environment featuring risk.
 * 
 * @author Stephan Heinemann
 *
 */
public enum RiskPolicy {
	/**
	 * Avoids all local risk
	 */
	AVOIDANCE(1d),
	/**
	 * Accepts a maximum of 25% local risk
	 */
	PROBE(25d),
	/**
	 * Accepts a maximum of 50% local risk
	 */
	SAFETY(50d),
	/**
	 * Accepts a maximum of 75% local risk
	 */
	EFFECTIVENESS(75d),
	/**
	 * Ignores all local risk
	 */
	IGNORANCE(100d);
	
	/** the threshold cost of this risk policy */
	private double thresholdCost = 0d;
	
	/**
	 * Constructs a new risk policy with a specified threshold cost.
	 * 
	 * @param thresholdCost the threshold cost of this risk policy
	 */
	private RiskPolicy(double thresholdCost) {
		this.thresholdCost = thresholdCost;
	}
	
	/**
	 * Gets the threshold cost of this risk policy.
	 * 
	 * @return the threshold cost of this risk policy
	 */
	public double getThreshholdCost() {
		return this.thresholdCost;
	}
	
	/**
	 * Indicates whether or not a cost satisfies this risk policy.
	 * 
	 * @param cost the cost
	 * 
	 * @return true if the cost satisfies the risk policy, false otherwise
	 */
	public boolean satisfies(double cost) {
		return this.thresholdCost >= cost;
	}
	
}
