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
package com.cfar.swim.worldwind.planners.rrt.hrrt;

import java.util.function.ToDoubleFunction;

import com.cfar.swim.worldwind.geom.precision.PrecisionDouble;

/**
 * Realizes a hRRT quality assessment function to select a suitable extension
 * candidate. An enhanced variant of the quality assessment function was
 * introduced by Manuel Rosa and is described with more detail in the
 * associated paper (Improvements for Heuristically Biasing RRT Growth).
 * 
 * @author Manuel Rosa
 * @author Stephan Heinemann
 *
 */
public class HRRTreeQuality implements ToDoubleFunction<HRRTreeWaypoint> {
	
	/** the variant of this hRRT quality assessment function */
	private HRRTreeQualityVariant variant = HRRTreeQualityVariant.ENHANCED;
	
	/** the optimal cost of this hRRT quality assessment function */
	private double optimalCost;
	
	/** the maximum cost of this hRRT quality assessment function */
	private double maximumCost;
	
	/** the quality bound of this hRRT quality assessment function */
	private double qualityBound = 0.1d; // [0, 1]
	
	/**
	 * Gets the applied variant of this hRRT quality assessment function.
	 * 
	 * @return the applied variant of this hRRT quality assessment function
	 */
	public HRRTreeQualityVariant getVariant() {
		return this.variant;
	}
	
	/**
	 * Sets the applied variant of this hRRT quality assessment function.
	 * 
	 * @param variant the applied variant to be set
	 */
	public void setVariant(HRRTreeQualityVariant variant) {
		this.variant = variant;
	}
	
	/**
	 * Determines whether or not the original variant of this hRRT quality
	 * assessment function is being applied.
	 * 
	 * @return true if the original variant of this hRRT quality assessment
	 *         function is being applied, false otherwise
	 */
	public boolean isOriginal() {
		return HRRTreeQualityVariant.ORIGINAL == this.getVariant();
	}
	
	/**
	 * Determines whether or not the enhanced variant of this hRRT quality
	 * assessment function is being applied.
	 * 
	 * @return true if the enhanced variant of this hRRT quality assessment
	 *         function is being applied, false otherwise
	 */
	public boolean isEnhanced() {
		return HRRTreeQualityVariant.ENHANCED == this.getVariant();
	}
	
	/**
	 * Gets the estimated total cost (f-value) of the optimal path from start
	 * to goal (start f-value).
	 * 
	 * @return the estimated total cost (f-value) of the optimal path from
	 *         start to goal (start f-value)
	 */
	public double getOptimalCost() {
		return this.optimalCost;
	}
	
	/**
	 * Sets the estimated total cost (f-value) of the optimal path from start
	 * to goal (start f-value).
	 * 
	 * @param optimalCost the estimated total cost (f-value) of the optimal
	 *                    path from start to goal (start f-value) to be set
	 */
	public void setOptimalCost(double optimalCost) {
		this.optimalCost = optimalCost;
	}
	
	/**
	 * Gets the maximum estimated total cost (f-value) of all vertices already
	 * considered by the hRRT planner.
	 * 
	 * @return the maximum estimated total cost of all vertices already
	 *         considered by the hRRT planner
	 */
	public double getMaximumCost() {
		return this.maximumCost;
	}
	
	/**
	 * Sets the maximum estimated total cost (f-value) of all vertices already
	 * considered by the hRRT planner.
	 * 
	 * @param maximumCost the maximum estimated total cost of all vertices
	 *                    already considered to be set
	 */
	public void setMaximumCost(double maximumCost) {
		this.maximumCost = maximumCost;
	}
	
	/**
	 * Gets the quality bound of this hRRT quality assessment function.
	 * 
	 * @return the quality bound of this hRRT quality assessment function
	 */
	public double getQualityBound() {
		return this.qualityBound;
	}
	
	/**
	 * Sets the quality bound of this hRRT quality assessment function.
	 * 
	 * @param qualityBound the quality bound to be set
	 * 
	 * @throws IllegalArgumentException if the quality bound is invalid
	 */
	public void setQualityBound(double qualityBound) {
		if ((0d <= qualityBound) && (1d >= qualityBound)) {
			this.qualityBound = qualityBound;
		} else {
			throw new IllegalArgumentException("quality bound is invalid");
		}
	}
	
	/**
	 * Applies the quality assessment function to a hRRT waypoint.
	 * 
	 * @param waypoint the waypoint to be assessed
	 * 
	 * @return the inverse quality (priority order) of the assessed waypoint,
	 *         [0: highest priority, 1: lowest priority]
	 */
	@Override
	public double applyAsDouble(HRRTreeWaypoint waypoint) {
		double relativeCost = 0d;
		double diff = waypoint.getF() - this.optimalCost;
		double quality = 0d;
		
		// prevent numerical imprecision
		if (Math.abs(diff) < PrecisionDouble.EPSILON)
			return 0d; // 1d
		
		if (this.isOriginal()) {
			// original implementation of HRRT
			relativeCost = (waypoint.getF() - this.optimalCost)
					/ (this.maximumCost - this.optimalCost);
			
			quality = 1d - relativeCost;
			quality = (quality < this.qualityBound) ? quality : this.qualityBound;
		} else {
			// enhanced implementation of HRRT
			if (waypoint.getF() > this.optimalCost) {
				// exp(-x): higher than optimal cost gives higher relative cost
				// exp(-x): increases substantially beyond 20% cost overhead
				relativeCost = Math.exp(-this.optimalCost
						/ (waypoint.getF() - this.optimalCost));
			} else {
				relativeCost = 0d; // limit case of exponential
			}
			
			quality = 1d - relativeCost;
			quality = (quality > this.qualityBound) ? quality : this.qualityBound;
		}
		
		return 1d - quality;
	}
	
	/**
	 * Gets the quality assessment of a hRRT waypoint.
	 * 
	 * @param waypoint the waypoint to be assessed
	 * 
	 * @return the quality of the assessed waypoint,
	 *         [0: lowest quality, 1: highest quality]
	 */
	public double get(HRRTreeWaypoint waypoint) {
		return  1d - this.applyAsDouble(waypoint);
	}
	
}
