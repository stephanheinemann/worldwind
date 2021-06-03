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

import com.cfar.swim.worldwind.planners.rrt.hrrt.HRRTreeVariant;
import com.cfar.swim.worldwind.planners.rrt.hrrt.Heuristic;

/**
 * Realizes the properties bean of a heuristic RRT planner.
 * 
 * @author Manuel Rosa
 *
 */
public class HRRTreeProperties extends RRTreeProperties {
	
	/** the heuristic applied by the hRRT planner */
	private Heuristic heuristic = Heuristic.BkRRT;
	
	/** the implementation variant of the hRRT planner */
	private HRRTreeVariant variant = HRRTreeVariant.ENHANCED;
	
	/**
	 * the limit of neighbors to consider as parents for a sample of the hRRT
	 * planner
	 */
	private int neighborLimit = 5; // [1, Integer.MAX_VALUE]
	
	/**
	 * the probability floor value of the hRRT planner to ensure the search is
	 * not overly biased against exploration
	 */
	private double probabilityFloor = 0.1d; // [0, 1]
	
	/**
	 * Constructs a new hRRT planner properties bean.
	 */
	public HRRTreeProperties() {
		super();
	}
	
	/**
	 * Gets the heuristic of this hRRT planner properties bean.
	 * 
	 * @return the heuristic of this hRRT planner properties bean
	 */
	public Heuristic getHeuristic() {
		return this.heuristic;
	}
	
	/**
	 * Sets the heuristic of this hRRT planner properties bean.
	 * 
	 * @param heuristic the heuristic to be set
	 */
	public void setHeuristic(Heuristic heuristic) {
		this.heuristic = heuristic;
	}
	
	/**
	 * Gets the implementation variant of this hRRT planner properties bean.
	 * 
	 * @return the implementation variant of this hRRT planner properties bean
	 */
	public HRRTreeVariant getVariant() {
		return this.variant;
	}
	
	/**
	 * Sets the implementation variant of this hRRT planner properties bean.
	 * 
	 * @param variant the implementation variant to be set
	 */
	public void setVariant(HRRTreeVariant variant) {
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
	 * 
	 * @throws IllegalArgumentException exception if neighbor limit is invalid
	 */
	public void setNeighborLimit(int neighborLimit) {
		if ((0 < neighborLimit) && (Integer.MAX_VALUE >= neighborLimit)) {
			this.neighborLimit = neighborLimit;
		} else {
			throw new IllegalArgumentException("neighbor limit is invalid");
		}
	}
	
	/**
	 * Gets the probability floor of this hRRT planner properties bean.
	 * 
	 * @return the probability floor of this hRRT planner properties bean
	 */
	public double getProbabilityFloor() {
		return probabilityFloor;
	}
	
	/**
	 * Sets the probability floor of this hRRT planner properties bean.
	 * 
	 * @param probabilityFloor the probability floor to be set
	 * 
	 * @throws IllegalArgumentException if probability floor is invalid
	 */
	public void setProbabilityFloor(double probabilityFloor) {
		if ((0d <= probabilityFloor) && (1d >= probabilityFloor)) {
			this.probabilityFloor = probabilityFloor;
		} else {
			throw new IllegalArgumentException("probability floor is invalid");
		}
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
			equals = (this.heuristic == hrrtp.heuristic)
					&& (this.variant == hrrtp.variant)
					&& (this.neighborLimit == hrrtp.neighborLimit)
					&& (this.probabilityFloor == hrrtp.probabilityFloor);
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
				this.heuristic,
				this.variant,
				this.neighborLimit,
				this.probabilityFloor);
	}
	
}
