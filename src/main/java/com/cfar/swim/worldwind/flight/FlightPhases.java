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

import com.cfar.swim.worldwind.managing.Features;
import com.google.common.collect.Range;

/**
 * Provides factory methods for flight phases with certain features
 * representing different levels of criticality.
 * 
 * @author Stephan Heinemann
 *
 */
public class FlightPhases {
	
	// TODO: aircraft safety volume penetration: obstacle criticality range
	// TODO: obstacle penetration is not discriminating (favored spaces)
	// TODO: review minimum and maximum cost values (increased base cost)
	// https://github.com/stephanheinemann/worldwind/issues/61
	// TODO: emergency (hazardous), urgency (critical), normal
	
	/**
	 * Creates a flight phase within a hazardous environment based on existing
	 * features.
	 * 
	 * @param features the features
	 *  
	 * @return a flight phase within a hazardous environment
	 */
	public static FlightPhase createHazardPhase(Features features) {
		Range<Double> obstaclePenetrationRange = Range.closed(0.75d, Double.POSITIVE_INFINITY);
		Range<Double> obstacleSeverityRange = Range.singleton(Double.POSITIVE_INFINITY);
		return new FlightPhase(features, obstaclePenetrationRange, obstacleSeverityRange);
	}
	
	/**
	 * Creates a flight phase within an aerodrome environment based on existing
	 * features.
	 * 
	 * @param features the features
	 * 
	 * @return a flight phase within an aerodrome environment
	 */
	public static FlightPhase createAerodromePhase(Features features) {
		Range<Double> obstaclePenetrationRange = Range.closedOpen(0.5d, Double.POSITIVE_INFINITY);
		Range<Double> obstacleSeverityRange = Range.closed(0d, Double.POSITIVE_INFINITY);
		return new FlightPhase(features, obstaclePenetrationRange, obstacleSeverityRange);
	}
	
	/**
	 * Creates a flight phase within an terminal environment based on existing
	 * features.
	 * 
	 * @param features the features
	 * 
	 * @return a flight phase within an terminal environment
	 */
	public static FlightPhase createTerminalPhase(Features features) {
		Range<Double> obstaclePenetrationRange = Range.closedOpen(0.25d, 0.5d);
		Range<Double> obstacleSeverityRange = Range.closed(0d, Double.POSITIVE_INFINITY);
		return new FlightPhase(features, obstaclePenetrationRange, obstacleSeverityRange);
	}
	
	/**
	 * Creates a flight phase within an enroute environment based on existing
	 * features.
	 * 
	 * @param features the features
	 * 
	 * @return a flight phase within an enroute environment
	 */
	public static FlightPhase createEnroutePhase(Features features) {
		Range<Double> obstaclePenetrationRange = Range.closedOpen(0d, 0.25d);
		Range<Double> obstacleSeverityRange = Range.closed(0d, Double.POSITIVE_INFINITY);
		return new FlightPhase(features, obstaclePenetrationRange, obstacleSeverityRange);
	}
	
}
