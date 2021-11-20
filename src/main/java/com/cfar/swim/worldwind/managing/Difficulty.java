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
package com.cfar.swim.worldwind.managing;

import com.google.common.collect.Range;

/**
 * Realizes a difficulty that covers obstacles penetration feature ranges.
 *  
 * @author Stephan Heinemann
 *
 */
public class Difficulty extends Criticality {
	
	/** the obstacle scatter range of this difficulty */
	private Range<Double> obstacleScatterRange;
	
	/**
	 * Constructs a new difficulty limiting only POI distance ranges.
	 * 
	 * @param features the features containing the POI distances
	 * 
	 * @see Criticality#Criticality(Features)
	 */
	public Difficulty(Features features) {
		super(features);
	}
	
	/**
	 * Gets the obstacle scatter range of this difficulty.
	 * 
	 * @return the obstacle scatter range of this difficulty
	 */
	public Range<Double> getObstacleScatterRange() {
		return obstacleScatterRange;
	}
	
	/**
	 * Sets the obstacle scatter range of this difficulty.
	 * 
	 * @param obstacleScatterRange the obstacle scatter range to be set
	 */
	public void setObstacleScatterRange(Range<Double> obstacleScatterRange) {
		this.obstacleScatterRange = obstacleScatterRange;
	}
	
	// TODO: direct versus indirect (derived) features
	// TODO: passed POIs are not relevant
	/*
	 * OSR ... Obstacle Scatter Range
	 * 
	 * OSR = FEATURE_POIS_OBSTACLES_VOLUME_AVG / FEATURE_POIS_VOLUME / FEATURE_POIS_OBSTACLES_VOLUME_RATIO
	 *     = FEATURE_POIS_OBSTACLES_VOLUME_AVG_RATIO / FEATURE_POIS_OBSTACLES_VOLUME_SUM_RATIO
	 * 
	 * 				OSR
	 * LOW			(0.8, INF)
	 * MODERATE		(0.6, 0.8]
	 * SUBSTANTIAL	(0.4, 0.6]
	 * SEVERE		(0.2, 0.4]
	 * CRITICAL		(0.0, 0,2]
	 * 
	 * A single large and non-expensive obstacle is indeed less difficult than
	 * a higher number of smaller obstacles. For simple A*, the heuristic will
	 * underestimate significantly in this case leading to a longer search.
	 * However, if the heuristic is inflated by the cost of the obstacle then
	 * a first solution is found quickly and the situation becomes equivalent
	 * to a no-obstacle, non-inflated scenario. In both cases the initial
	 * inflation could be set to the maximum or average obstacle cost by the
	 * heuristic autonomic manager and improved by the SMAC autonomic manager
	 * based on the sensed difficulty.
	 * 
	 * For RRT within a single large and expensive obstacle, new samples will
	 * unlikely permit to improve an existing shorter solution. Therefore, the
	 * distance bias becomes more important than the cost bias which is
	 * equivalent to the high initial inflation of ARA*.
	 * 
	 */
	
	/** the low difficulty obstacle scattered range */
	private static final Range<Double> OSR_LOW = Range.greaterThan(0.8d);
	/** the moderate difficulty obstacle scattered range */
	private static final Range<Double> OSR_MODERATE = Range.openClosed(0.6d, 0.8d);
	/** the substantial difficulty obstacle scattered range */
	private static final Range<Double> OSR_SUBSTANTIAL = Range.openClosed(0.4d, 0.6);
	/** the severe difficulty obstacle scattered range */
	private static final Range<Double> OSR_SEVERE = Range.openClosed(0.2d, 0.4d);
	/** the critical difficulty obstacle scattered range */
	private static final Range<Double> OSR_CRITICAL = Range.openClosed(0.0d, 0.2d);
	
	/**
	 * Creates a low difficulty based on features.
	 * 
	 * @param features the features
	 * 
	 * @return a low difficulty based on features
	 */
	public static Difficulty createLow(Features features) {
		Difficulty low = new Difficulty(features);
		
		low.setObstacleScatterRange(Difficulty.OSR_LOW);
		
		return low;
	}
	
	/**
	 * Creates a moderate difficulty based on features.
	 * 
	 * @param features the features
	 * 
	 * @return a moderate difficulty based on features
	 */
	public static Difficulty createModerate(Features features) {
		Difficulty moderate = new Difficulty(features);
		
		moderate.setObstacleScatterRange(Difficulty.OSR_MODERATE);
		
		return moderate;
	}
	
	/**
	 * Creates a substantial difficulty based on features.
	 * 
	 * @param features the features
	 * 
	 * @return a substantial difficulty based on features
	 */
	public static Difficulty createSubstantial(Features features) {
		Difficulty substantial = new Difficulty(features);
		
		substantial.setObstacleScatterRange(Difficulty.OSR_SUBSTANTIAL);
		
		return substantial;
	}
	
	/**
	 * Creates a severe difficulty based on features.
	 * 
	 * @param features the features
	 * 
	 * @return a severe difficulty based on features
	 */
	public static Difficulty createSevere(Features features) {
		Difficulty severe = new Difficulty(features);
		
		severe.setObstacleScatterRange(Difficulty.OSR_SEVERE);
		
		return severe;
	}
	
	/**
	 * Creates a critical difficulty based on features.
	 * 
	 * @param features the features
	 * 
	 * @return a critical difficulty based on features
	 */
	public static Difficulty createCritical(Features features) {
		Difficulty critical = new Difficulty(features);
		
		critical.setObstacleScatterRange(Difficulty.OSR_CRITICAL);
		
		return critical;
	}
	
	/**
	 * Determines whether or not this difficulty covers given features.
	 * 
	 * @param features the features
	 * 
	 * @return true if this difficulty covers the features, false otherwise
	 * 
	 * @see Criticality#covers(Features)
	 */
	public boolean covers(Features features) {
		boolean covers = super.covers(features);
		
		if (covers) {
			covers = this.getObstacleScatterRange().contains(
					features.get(Features.FEATURE_POIS_OBSTACLES_VOLUME_AVG) /
					features.get(Features.FEATURE_POIS_VOLUME) /
					features.get(Features.FEATURE_POIS_OBSTACLES_VOLUME_RATIO));
		}
		
		return covers;
	}
	
}
