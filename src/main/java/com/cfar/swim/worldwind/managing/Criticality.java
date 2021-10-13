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

import com.cfar.swim.worldwind.flight.FlightPhase;
import com.google.common.collect.Range;

/**
 * Realizes a criticality level that covers certain feature ranges.
 * 
 * @author Stephan Heinemann
 *
 */
public class Criticality implements FeatureCategory {
	
	// TODO: aircraft safety volume penetration: obstacle criticality range
	// TODO: obstacle penetration is not discriminating (favored spaces)
	// TODO: review minimum and maximum cost values (increased base cost)
	// https://github.com/stephanheinemann/worldwind/issues/61
	// TODO: emergency (hazardous), urgency (critical), normal
	
	/** the distance tolerance of associable features */
	public static final double DISTANCE_TOLERANCE = 0.2d;
	
	/** the minimum POI distance range of this criticality level */
	private final Range<Double> minPoiDistanceRange;
	
	/** the maximum POI distance range of this criticality level */
	private final Range<Double> maxPoiDistanceRange;
	
	/** the minimum POI vertical range of this criticality level */
	private final Range<Double> minPoiVerticalRange;
	
	/** the maximum POI vertical range of this criticality level */
	private final Range<Double> maxPoiVerticalRange;
	
	private Range<Integer> aircraftObstaclesCountRange;
	private Range<Integer> poisObstaclesCountRange;
	private Range<Integer> envObstaclesCountRange;
	
	private Range<Double> aircraftObstaclesSeverityRange;
	private Range<Double> poisObstaclesSeverityRange;
	private Range<Double> envObstaclesSeverityRange;
	
	/** the minimum obstacle distance range of this criticality level */
	private final Range<Double> minObstacleDistanceRange;
	
	/** the maximum obstacle distance range of this criticality level */
	private final Range<Double> maxObstacleDistanceRange;
	
	/** the obstacle count range of this criticality level */
	private Range<Integer> obstacleCountRange;
	
	/** the unsafe obstacle count range of this criticality level */
	private Range<Integer> unsafeObstacleCountRange;
	
	/** the obstacle penetration range of this criticality level */
	private final Range<Double> obstaclePentrationRange;
	
	/** the obstacle severity range of this criticality level */
	private final Range<Double> obstacleSeverityRange;
	
	/**
	 * Constructs a new criticality level based on given feature ranges.
	 * 
	 * @param minPoiDistanceRange the minimum POI distance range
	 * @param maxPoiDistanceRange the maximum POI distance range
	 * @param minPoiVerticalRange the minimum POI vertical range
	 * @param maxPoiVerticalRange the maximum POI vertical range
	 * @param minObstacleDistanceRange the minimum obstacle range
	 * @param maxObstacleDistanceRange the maximum obstacle range
	 * @param obstaclePenetrationRange the obstacle penetration range
	 * @param obstacleSeverityRange the obstacle severity range
	 */
	public Criticality(
			Range<Double> minPoiDistanceRange,
			Range<Double> maxPoiDistanceRange,
			Range<Double> minPoiVerticalRange,
			Range<Double> maxPoiVerticalRange,
			Range<Double> minObstacleDistanceRange,
			Range<Double> maxObstacleDistanceRange,
			Range<Double> obstaclePenetrationRange,
			Range<Double> obstacleSeverityRange) {
		this.minPoiDistanceRange = minPoiDistanceRange;
		this.maxPoiDistanceRange = maxPoiDistanceRange;
		this.minPoiVerticalRange = minPoiVerticalRange;
		this.maxPoiVerticalRange = maxPoiVerticalRange;
		this.minObstacleDistanceRange = minObstacleDistanceRange;
		this.maxObstacleDistanceRange = maxObstacleDistanceRange;
		this.obstaclePentrationRange = obstaclePenetrationRange;
		this.obstacleSeverityRange = obstacleSeverityRange;
		// TODO: extend as required
	}
	
	/**
	 * Constructs a new criticality level based on existing features and given
	 * feature ranges
	 * 
	 * @param features the features
	 * @param obstaclePenetrationRange the obstacle penetration range
	 * @param obstacleSeverityRange the obstacle severity range
	 */
	public Criticality(
			Features features,
			Range<Double> obstaclePenetrationRange,
			Range<Double> obstacleSeverityRange) {
		this.minPoiDistanceRange = Criticality.createPoisDistanceMinRange(features);
		this.maxPoiDistanceRange = Criticality.createPoisDistanceMaxRange(features);
		this.minPoiVerticalRange = Range.all();
		this.maxPoiVerticalRange = Range.all();
		this.minObstacleDistanceRange = Range.all();
		this.maxObstacleDistanceRange = Range.all();
		this.obstaclePentrationRange = obstaclePenetrationRange;
		this.obstacleSeverityRange = obstacleSeverityRange;
		// TODO: extend and restrict as required
	}
	
	/**
	 * Creates a minimum POI distance range for associated criticality levels
	 * of given features.
	 * 
	 * @param features the features
	 * 
	 * @return the minimum POI distance range for associated criticality levels
	 */
	private static Range<Double> createPoisDistanceMinRange(Features features) {
		double poisDistanceMinTolerance = FlightPhase.DISTANCE_TOLERANCE
				* features.get(Features.FEATURE_POIS_DISTANCE_MIN);
		Range<Double> poisDistanceMinRange = Range.closed(
				features.get(Features.FEATURE_POIS_DISTANCE_MIN) - poisDistanceMinTolerance,
				features.get(Features.FEATURE_POIS_DISTANCE_MIN) + poisDistanceMinTolerance);
		return poisDistanceMinRange;
	}
	
	/**
	 * Creates a maximum POI distance range for associated criticality levels
	 * of given features.
	 * 
	 * @param features the features
	 * 
	 * @return the maximum POI distance range for associated criticality levels
	 */
	private static Range<Double> createPoisDistanceMaxRange(Features features) {
		double poisDistanceMaxTolerance = FlightPhase.DISTANCE_TOLERANCE
				* features.get(Features.FEATURE_POIS_DISTANCE_MAX);
		Range<Double> poisDistanceMaxRange = Range.closed(
				features.get(Features.FEATURE_POIS_DISTANCE_MAX) - poisDistanceMaxTolerance,
				features.get(Features.FEATURE_POIS_DISTANCE_MAX) + poisDistanceMaxTolerance);
		return poisDistanceMaxRange;
	}
	
	/**
	 * Determines whether or not this criticality level covers given features.
	 * 
	 * @param features the features
	 * 
	 * @return true if this criticality level covers the features,
	 *         false otherwise
	 * 
	 * @see FeatureCategory#covers(Features)
	 */
	@Override
	public boolean covers(Features features) {
		return this.minPoiDistanceRange.contains(features.get(Features.FEATURE_POIS_DISTANCE_MIN))
				&& this.maxPoiDistanceRange.contains(features.get(Features.FEATURE_POIS_DISTANCE_MAX))
				&& this.minPoiVerticalRange.contains(features.get(Features.FEATURE_POIS_DISTANCE_VERTICAL_MIN))
				&& this.maxPoiVerticalRange.contains(features.get(Features.FEATURE_POIS_DISTANCE_VERTICAL_MAX))
				&& this.minObstacleDistanceRange.contains(features.get(Features.FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_MIN))
				&& this.maxObstacleDistanceRange.contains(features.get(Features.FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_MAX))
				&& this.obstaclePentrationRange.contains(features.get(Features.FEATURE_POIS_OBSTACLES_VOLUME_RATIO))
				&& this.obstacleSeverityRange.contains(features.get(Features.FEATURE_POIS_OBSTACLES_COST_MIN))
				&& this.obstacleSeverityRange.contains(features.get(Features.FEATURE_POIS_OBSTACLES_COST_MAX))
				&& this.obstacleSeverityRange.contains(features.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_POLICIES));
	}
	
	/**
	 * Creates a low criticality level based on features.
	 * 
	 * @param features the features
	 *  
	 * @return a low criticality level based on the features
	 */
	public static Criticality createLow(Features features) {
		// no obstacle penetration inside aircraft safety radius, and
		Range<Integer> aircraftObstaclesCountRange = Range.singleton(0);
		Range<Double> aircraftObstaclesSeverityRange = Range.openClosed(Double.NEGATIVE_INFINITY, 0d);
		// no obstacle penetration inside POI bounding box, and
		Range<Integer> poisObstaclesCountRange = Range.singleton(0);
		Range<Double> poisObstaclesSeverityRange = Range.openClosed(Double.NEGATIVE_INFINITY, 0d);
		// no up to no risk-exceeding obstacle penetration inside environment; or
		Range<Integer> envObstaclesCoundRange = Range.closedOpen(0, Integer.MAX_VALUE);
		Range<Double> envObstacleSeverityRange = Range.openClosed(Double.NEGATIVE_INFINITY, features.get(Features.FEATURE_POLICY_RISK));
		
		Range<Integer> obstacleCountRange = Range.singleton(0);
		Range<Integer> unsafeObstacleCountRange = Range.singleton(0);
		Range<Double> obstaclePenetrationRange = Range.singleton(0d);
		Range<Double> obstacleSeverityRange = Range.singleton(0d);
		return new Criticality(features,
				obstaclePenetrationRange,
				obstacleSeverityRange);
	}
	
	/**
	 * Creates a moderate criticality level based on features.
	 * 
	 * @param features the features
	 * 
	 * @return a moderate criticality level based on the features
	 */
	public static Criticality createModerate(Features features) {
		// no obstacle penetration inside aircraft safety radius, and
		// no risk-exceeding obstacle penetration inside POI bounding box, and
		// no up to no risk-exceeding obstacle penetration inside environment; or
		Range<Double> obstaclePenetrationRange = Range.closedOpen(0.5d, Double.POSITIVE_INFINITY);
		Range<Double> obstacleSeverityRange = Range.closed(0d, Double.POSITIVE_INFINITY);
		return null; //new Criticality(features, obstaclePenetrationRange, obstacleSeverityRange);
	}
	
	/**
	 * Creates a substantial criticality level based on features.
	 * 
	 * @param features the features
	 * 
	 * @return a substantial criticality level based on the features
	 */
	public static Criticality createSubstantial(Features features) {
		// no obstacle penetration inside aircraft safety radius, and
		// no risk-exceeding up to risk-exceeding obstacle penetration inside POI bounding box, and
		// no up to risk-exceeding obstacle penetration inside environment; or
		Range<Double> obstaclePenetrationRange = Range.closedOpen(0.25d, 0.5d);
		Range<Double> obstacleSeverityRange = Range.closed(0d, Double.POSITIVE_INFINITY);
		return null; //new Criticality(features, obstaclePenetrationRange, obstacleSeverityRange);
	}
	
	/**
	 * Creates a severe criticality level based on features.
	 * 
	 * @param features the features
	 * 
	 * @return a severe criticality level based on the features
	 */
	public static Criticality createSevere(Features features) {
		// no risk-exceeding obstacle penetration inside aircraft safety radius, and
		// no up to risk-exceeding obstacle penetration inside POI bounding box, and
		// no up to risk-exceeding obstacle penetration inside environment; or
		Range<Double> obstaclePenetrationRange = Range.closedOpen(0d, 0.25d);
		Range<Double> obstacleSeverityRange = Range.closed(0d, Double.POSITIVE_INFINITY);
		return null; //new Criticality(features, obstaclePenetrationRange, obstacleSeverityRange);
	}
	
	/**
	 * Creates a critical criticality level based on features.
	 * 
	 * @param features the features
	 * 
	 * @return a critical criticality level based on the features
	 */
	public static Criticality createCritical(Features features) {
		// risk-exceeding obstacle penetration inside aircraft safety radius, and
		// no up to risk-exceeding obstacle penetration inside POI bounding box, and
		// no up to risk-exceeding obstacle penetration inside environment; or
		Range<Double> obstaclePenetrationRange = Range.closedOpen(0d, 0.25d);
		Range<Double> obstacleSeverityRange = Range.closed(0d, Double.POSITIVE_INFINITY);
		return null; //new Criticality(features, obstaclePenetrationRange, obstacleSeverityRange);
	}
	
}
