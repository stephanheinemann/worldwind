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
package com.cfar.swim.worldwind.flight;

import com.cfar.swim.worldwind.managing.FeatureCategory;
import com.cfar.swim.worldwind.managing.Features;
import com.google.common.collect.Range;

/**
 * Realizes a flight phase that covers certain feature ranges.
 * 
 * @author Stephan Heinemann
 *
 */
public class FlightPhase implements FeatureCategory {
	
	/** the altitude range of this flight phase */
	private final Range<Double> altitudeRange;
	
	/** the minimum POI distance range of this flight phase */
	private final Range<Double> minPoiDistanceRange;
	
	/** the maximum POI distance range of this flight phase */
	private final Range<Double> maxPoiDistanceRange;
	
	/** the minimum POI vertical range of this flight phase */
	private final Range<Double> minPoiVerticalRange;
	
	/** the maximum POI vertical range of this flight phase */
	private final Range<Double> maxPoiVerticalRange;
	
	/** the minimum obstacle distance range of this flight phase */
	private final Range<Double> minObstacleDistanceRange;
	
	/** the maximum obstacle distance range of this flight phase */
	private final Range<Double> maxObstacleDistanceRange;
	
	/** the obstacle penetration range of this flight phase */
	private final Range<Double> obstaclePentrationRange;
	
	/** the obstacle severity range of this flight phase */
	private final Range<Double> obstacleSeverityRange;
	
	/**
	 * Constructs a new flight phase based on certain feature ranges.
	 * 
	 * @param altitudeRange the altitude range
	 * @param minPoiDistanceRange the minimum POI distance range
	 * @param maxPoiDistanceRange the maximum POI distance range
	 * @param minPoiVerticalRange the minimum POI vertical range
	 * @param maxPoiVerticalRange the maximum POI vertical range
	 * @param minObstacleDistanceRange the minimum obstacle range
	 * @param maxObstacleDistanceRange the maximum obstacle range
	 * @param obstaclePenetrationRange the obstacle penetration range
	 * @param obstacleSeverityRange the obstacle severity range
	 */
	public FlightPhase(
			Range<Double> altitudeRange,
			Range<Double> minPoiDistanceRange,
			Range<Double> maxPoiDistanceRange,
			Range<Double> minPoiVerticalRange,
			Range<Double> maxPoiVerticalRange,
			Range<Double> minObstacleDistanceRange,
			Range<Double> maxObstacleDistanceRange,
			Range<Double> obstaclePenetrationRange,
			Range<Double> obstacleSeverityRange) {
		this.altitudeRange = altitudeRange;
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
	 * Determines whether or not this flight phase covers features.
	 * 
	 * @param features the features
	 * 
	 * @return true if this flight phase covers the features, false otherwise
	 * 
	 * @see FeatureCategory#covers(Features)
	 */
	@Override
	public boolean covers(Features features) {
		// TODO: rather use POI elevations for altitude range (problem ASL versus AGL)
		return /* this.altitudeRange.contains(features.get(Features.FEATURE_AIRCRAFT_ALTITUDE))
				&& */ this.minPoiDistanceRange.contains(features.get(Features.FEATURE_POIS_DISTANCE_MIN))
				&& this.maxPoiDistanceRange.contains(features.get(Features.FEATURE_POIS_DISTANCE_MAX))
				&& this.minPoiVerticalRange.contains(features.get(Features.FEATURE_POIS_DISTANCE_VERTICAL_MIN))
				&& this.maxPoiVerticalRange.contains(features.get(Features.FEATURE_POIS_DISTANCE_VERTICAL_MAX))
				&& this.minObstacleDistanceRange.contains(features.get(Features.FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_MIN))
				&& this.maxObstacleDistanceRange.contains(features.get(Features.FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_MAX))
				&& this.obstaclePentrationRange.contains(features.get(Features.FEATURE_POIS_OBSTACLES_VOLUME_RATIO))
				&& this.obstacleSeverityRange.contains(features.get(Features.FEATURE_POIS_OBSTACLES_COST_MIN))
				&& this.obstacleSeverityRange.contains(features.get(Features.FEATURE_POIS_OBSTACLES_COST_MAX));
	}
	
}
