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

import com.cfar.swim.worldwind.planners.rrt.hrrt.HRRTreeAlgorithm;
import com.cfar.swim.worldwind.planners.rrt.hrrt.HRRTreeQualityVariant;

/**
 * Realizes the properties bean of a heuristic RRT planner.
 * 
 * @author Manuel Rosa
 *
 */
public class HRRTreeProperties extends RRTreeProperties {
	
	/** the heuristic algorithm applied by the hRRT planner */
	private HRRTreeAlgorithm algorithm = HRRTreeAlgorithm.BkRRT;
	
	/** the quality assessment variant applied by the hRRT planner */
	private HRRTreeQualityVariant variant = HRRTreeQualityVariant.ENHANCED;
	
	/** the limit of neighbors to consider for extension by the hRRT planner */
	@Min(value = 1, message = "{property.planner.hrrt.neighborLimit.min}")
	@Max(value = Integer.MAX_VALUE, message = "{property.planner.hrrt.neighborLimit.max}")
	private int neighborLimit = 5;
	
	/**
	 * the probability floor value of the hRRT planner to ensure the search is
	 * not overly biased against exploration
	 */
	@DecimalMin(value = "0", message = "{property.planner.hrrt.qualityBound.min}")
	@DecimalMax(value = "1", message = "{property.planner.hrrt.qualityBound.max}")
	private double qualityBound = 0.1d;
	
	/**
	 * Constructs a new hRRT planner properties bean.
	 */
	public HRRTreeProperties() {
		super();
	}
	
	/**
	 * Gets the heuristic algorithm of this hRRT planner properties bean.
	 * 
	 * @return the heuristic algorithm of this hRRT planner properties bean
	 */
	public HRRTreeAlgorithm getAlgorithm() {
		return this.algorithm;
	}
	
	/**
	 * Sets the heuristic algorithm of this hRRT planner properties bean.
	 * 
	 * @param algorithm the heuristic algorithm to be set
	 */
	public void setAlgorithm(HRRTreeAlgorithm algorithm) {
		this.algorithm = algorithm;
	}
	
	/**
	 * Gets the quality assessment variant of this hRRT planner properties bean.
	 * 
	 * @return the quality assessment variant of this hRRT planner properties bean
	 */
	public HRRTreeQualityVariant getVariant() {
		return this.variant;
	}
	
	/**
	 * Sets the quality assessment variant of this hRRT planner properties bean.
	 * 
	 * @param variant the quality assessment variant to be set
	 */
	public void setVariant(HRRTreeQualityVariant variant) {
		this.variant = variant;
	}
	
	/**
	 * Gets the limit of neighbors of this hRRT planner properties bean.
	 * 
	 * @return the limit of neighbors of this hRRT planner properties bean
	 */
	public int getNeighborLimit() {
		return neighborLimit;
	}
	
	/**
	 * Sets the limit of neighbors of this hRRT planner properties bean.
	 * 
	 * @param neighborLimit the limit of neighbors to be set
	 */
	public void setNeighborLimit(int neighborLimit) {
		this.neighborLimit = neighborLimit;
	}
	
	/**
	 * Gets the quality (probability) bound of this hRRT planner properties bean.
	 * 
	 * @return the quality (probability) bound of this hRRT planner properties bean
	 */
	public double getQualityBound() {
		return qualityBound;
	}
	
	/**
	 * Sets the quality (probability) bound of this hRRT planner properties bean.
	 * 
	 * @param qualityBound the quality (probability) bound to be set
	 */
	public void setQualityBound(double qualityBound) {
		this.qualityBound = qualityBound;
	}
	
	/**
	 * Determines whether or not this hRRT planner properties bean equals
	 * another hRRT planner properties bean based on their aggregated
	 * properties.
	 * 
	 * @param o the other hRRT planner properties bean
	 * 
	 * @return true, if the aggregated properties of this hRRT planner
	 *         properties bean equal the aggregated properties of the other
	 *         hRRT planner properties bean, false otherwise
	 * 
	 * @see RRTreeProperties#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = super.equals(o);
		
		if (equals) {
			HRRTreeProperties hrrtp = (HRRTreeProperties) o;
			equals = (this.algorithm == hrrtp.algorithm)
					&& (this.variant == hrrtp.variant)
					&& (this.neighborLimit == hrrtp.neighborLimit)
					&& (this.qualityBound == hrrtp.qualityBound);
		} else {
			equals = false;
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this hRRT planner properties bean based on its
	 * aggregated properties.
	 * 
	 * @return the hash code of this hRRT planner properties bean based on its
	 *         aggregated properties
	 * 
	 * @see RRTreeProperties#hashCode()
	 */
	@Override
	public int hashCode() {
		return Objects.hash(
				super.hashCode(),
				this.algorithm,
				this.variant,
				this.neighborLimit,
				this.qualityBound);
	}
	
}
