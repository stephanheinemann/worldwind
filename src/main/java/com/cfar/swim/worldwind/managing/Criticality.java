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
 * Abstracts a criticality that covers certain feature ranges.
 * 
 * @author Stephan Heinemann
 *
 */
public abstract class Criticality implements FeatureCategory {
	
	/** the distance tolerance of associable criticalities */
	public static final double DISTANCE_TOLERANCE = 0.2d;
	
	/** the minimum POI distance range of this criticality */
	private Range<Double> poiDistanceMinRange = Range.all();
	
	/** the maximum POI distance range of this criticality */
	private Range<Double> poiDistanceMaxRange = Range.all();
	
	/** the minimum POI vertical distance range of this criticality */
	private Range<Double> poiDistanceVerticalMinRange = Range.all();
	
	/** the maximum POI vertical distance range of this criticality */
	private Range<Double> poiDistanceVerticalMaxRange = Range.all();
	
	/**
	 * Constructs a new criticality limiting only POI distance ranges.
	 * 
	 * @param features the features containing the POI distances
	 */
	public Criticality(Features features) {
		this.poiDistanceMinRange = Criticality.createPoisDistanceMinRange(features);
		this.poiDistanceMaxRange = Criticality.createPoisDistanceMaxRange(features);
		// TODO: vertical distance ranges
	}
	
	/**
	 * Creates a minimum POI distance range for associated criticalities of
	 * given features.
	 * 
	 * @param features the features
	 * 
	 * @return the minimum POI distance range for associated criticalities
	 */
	private static Range<Double> createPoisDistanceMinRange(Features features) {
		double poisDistanceMinTolerance = Criticality.DISTANCE_TOLERANCE
				* features.get(Features.FEATURE_POIS_DISTANCE_MIN);
		Range<Double> poisDistanceMinRange = Range.closed(
				features.get(Features.FEATURE_POIS_DISTANCE_MIN) - poisDistanceMinTolerance,
				features.get(Features.FEATURE_POIS_DISTANCE_MIN) + poisDistanceMinTolerance);
		return poisDistanceMinRange;
	}
	
	/**
	 * Creates a maximum POI distance range for associated criticalities of
	 * given features.
	 * 
	 * @param features the features
	 * 
	 * @return the maximum POI distance range for associated criticalities
	 */
	private static Range<Double> createPoisDistanceMaxRange(Features features) {
		double poisDistanceMaxTolerance = Criticality.DISTANCE_TOLERANCE
				* features.get(Features.FEATURE_POIS_DISTANCE_MAX);
		Range<Double> poisDistanceMaxRange = Range.closed(
				features.get(Features.FEATURE_POIS_DISTANCE_MAX) - poisDistanceMaxTolerance,
				features.get(Features.FEATURE_POIS_DISTANCE_MAX) + poisDistanceMaxTolerance);
		return poisDistanceMaxRange;
	}
	
	/**
	 * Determines whether or not this criticality covers given features.
	 * 
	 * @param features the features
	 * 
	 * @return true if this criticality covers the features, false otherwise
	 * 
	 * @see FeatureCategory#covers(Features)
	 */
	@Override
	public boolean covers(Features features) {
		return this.poiDistanceMinRange.contains(features.get(Features.FEATURE_POIS_DISTANCE_MIN))
				&& this.poiDistanceMaxRange.contains(features.get(Features.FEATURE_POIS_DISTANCE_MAX))
				&& this.poiDistanceVerticalMinRange.contains(features.get(Features.FEATURE_POIS_DISTANCE_VERTICAL_MIN))
				&& this.poiDistanceVerticalMaxRange.contains(features.get(Features.FEATURE_POIS_DISTANCE_VERTICAL_MAX));
	}
	
}
