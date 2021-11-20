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
 * Realizes a severity that covers obstacle cost feature ranges.
 * 
 * @author Stephan Heinemann
 *
 */
public class Severity extends Criticality {
	
	/** the minimum aircraft safety obstacles cost range of this severity */
	private Range<Double> aircraftSafetyObstaclesCostMinRange = Range.all();
	
	/** the maximum aircraft safety obstacles cost range of this severity */
	private Range<Double> aircraftSafetyObstaclesCostMaxRange = Range.all();
	
	/** the policy aircraft safety obstacles cost range of this severity */
	private Range<Double> aircraftSafetyObstaclesPolicyRange = Range.all();
	
	/** the minimum POI bounding box obstacles cost range of this severity */
	private Range<Double> poiObstaclesCostMinRange = Range.all();
	
	/** the maximum POI bounding box obstacles cost range of this severity */ 
	private Range<Double> poiObstaclesCostMaxRange = Range.all();
	
	/** the policy POI bounding box obstacles cost range of this severity */
	private Range<Double> poiObstaclesCostPolicyRange = Range.all();
	
	/** the minimum environment obstacles cost range of this severity */
	private Range<Double> environmentObstaclesCostMinRange = Range.all();
	
	/** the maximum environment obstacles cost range of this severity */
	private Range<Double> environmentObstaclesCostMaxRange = Range.all();
	
	/** the policy environment obstacles cost range of this severity */
	private Range<Double> environmentObstaclesCostPolicyRange = Range.all();
	
	/**
	 * Constructs a new severity limiting only POI distance ranges.
	 * 
	 * @param features the features containing the POI distances
	 * 
	 * @see Criticality#Criticality(Features)
	 */
	public Severity(Features features) {
		super(features);
	}
	
	/**
	 * Gets the minimum aircraft safety obstacles cost range.
	 * 
	 * @return the minimum aircraft safety obstacles cost range
	 */
	public Range<Double> getAircraftSafetyObstaclesCostMinRange() {
		return this.aircraftSafetyObstaclesCostMinRange;
	}
	
	/**
	 * Sets the minimum aircraft safety obstacles cost range.
	 * 
	 * @param aircraftSafetyObstaclesCostMinRange the minimum aircraft safety obstacles cost range to be set
	 */
	public void setAircraftSafetyObstaclesCostMinRange(Range<Double> aircraftSafetyObstaclesCostMinRange) {
		this.aircraftSafetyObstaclesCostMinRange = aircraftSafetyObstaclesCostMinRange;
	}
	
	/**
	 * Gets the maximum aircraft safety obstacles cost range.
	 * 
	 * @return the maximum aircraft safety obstacles cost range
	 */
	public Range<Double> getAircraftSafetyObstaclesCostMaxRange() {
		return this.aircraftSafetyObstaclesCostMaxRange;
	}
	
	/**
	 * Sets the maximum aircraft safety obstacles cost range.
	 * 
	 * @param aircraftSafetyObstaclesCostMaxRange the maximum aircraft safety obstacles cost range to be set
	 */
	public void setAircraftSafetyObstaclesCostMaxRange(Range<Double> aircraftSafetyObstaclesCostMaxRange) {
		this.aircraftSafetyObstaclesCostMaxRange = aircraftSafetyObstaclesCostMaxRange;
	}
	
	/**
	 * Gets the policy aircraft safety obstacles cost range.
	 * 
	 * @return the policy aircraft safety obstacles cost range
	 */
	public Range<Double> getAircraftSafetyObstaclesPolicyRange() {
		return this.aircraftSafetyObstaclesPolicyRange;
	}
	
	/**
	 * Sets the policy aircraft safety obstacles cost range.
	 * 
	 * @param aircraftSafetyObstaclesPolicyRange the policy aircraft safety obstacles cost range to be set
	 */
	public void setAircraftSafetyObstaclesPolicyRange(Range<Double> aircraftSafetyObstaclesPolicyRange) {
		this.aircraftSafetyObstaclesPolicyRange = aircraftSafetyObstaclesPolicyRange;
	}
	
	/**
	 * Gets the minimum POI bounding box obstacles cost range.
	 * 
	 * @return the minimum POI bounding box obstacles cost range
	 */
	public Range<Double> getPoiObstaclesCostMinRange() {
		return this.poiObstaclesCostMinRange;
	}
	
	/**
	 * Sets the minimum POI bounding box obstacles cost range.
	 * 
	 * @param poiObstaclesCostMinRange the minimum POI bounding box obstacles cost range to be set
	 */
	public void setPoiObstaclesCostMinRange(Range<Double> poiObstaclesCostMinRange) {
		this.poiObstaclesCostMinRange = poiObstaclesCostMinRange;
	}
	
	/**
	 * Gets the maximum POI bounding box obstacles cost range.
	 * 
	 * @return the maximum POI bounding box obstacles cost range
	 */
	public Range<Double> getPoiObstaclesCostMaxRange() {
		return this.poiObstaclesCostMaxRange;
	}
	
	/**
	 * Sets the maximum POI bounding box obstacles cost range.
	 * 
	 * @param poiObstaclesCostMaxRange the maximum POI bounding box obstacles cost range to be set
	 */
	public void setPoiObstaclesCostMaxRange(Range<Double> poiObstaclesCostMaxRange) {
		this.poiObstaclesCostMaxRange = poiObstaclesCostMaxRange;
	}
	
	/**
	 * Gets the policy POI bounding box obstacles cost range.
	 * 
	 * @return the policy POI bounding box obstacles cost range
	 */
	public Range<Double> getPoiObstaclesCostPolicyRange() {
		return poiObstaclesCostPolicyRange;
	}
	
	/**
	 * Sets the policy POI bounding box obstacles cost range.
	 * 
	 * @param poiObstaclesCostPolicyRange the policy POI bounding box obstacles cost range to be set
	 */
	public void setPoiObstaclesCostPolicyRange(Range<Double> poiObstaclesCostPolicyRange) {
		this.poiObstaclesCostPolicyRange = poiObstaclesCostPolicyRange;
	}
	
	/**
	 * Gets the minimum environment obstacles cost range.
	 * 
	 * @return the minimum environment obstacles cost range
	 */
	public Range<Double> getEnvironmentObstaclesCostMinRange() {
		return this.environmentObstaclesCostMinRange;
	}
	
	/**
	 * Sets the minimum environment obstacles cost range.
	 * 
	 * @param environmentObstaclesCostMinRange the minimum environment obstacles cost range to be set
	 */
	public void setEnvironmentObstaclesCostMinRange(Range<Double> environmentObstaclesCostMinRange) {
		this.environmentObstaclesCostMinRange = environmentObstaclesCostMinRange;
	}
	
	/**
	 * Gets the maximum environment obstacles cost range. 
	 * 
	 * @return the maximum environment obstacles cost range
	 */
	public Range<Double> getEnvironmentObstaclesCostMaxRange() {
		return this.environmentObstaclesCostMaxRange;
	}
	
	/**
	 * Sets the maximum environment obstacles cost range
	 * 
	 * @param environmentObstaclesCostMaxRange the maximum environment obstacles cost range to be set
	 */
	public void setEnvironmentObstaclesCostMaxRange(Range<Double> environmentObstaclesCostMaxRange) {
		this.environmentObstaclesCostMaxRange = environmentObstaclesCostMaxRange;
	}
	
	/**
	 * Gets the policy environment obstacles cost range.
	 * 
	 * @return the policy environment obstacles cost range
	 */
	public Range<Double> getEnvironmentObstaclesCostPolicyRange() {
		return this.environmentObstaclesCostPolicyRange;
	}

	/**
	 * Sets the policy environment obstacles cost range.
	 * 
	 * @param environmentObstaclesCostPolicyRange the policy environment obstacles cost range to be set
	 */
	public void setEnvironmentObstaclesCostPolicyRange(Range<Double> environmentObstaclesCostPolicyRange) {
		this.environmentObstaclesCostPolicyRange = environmentObstaclesCostPolicyRange;
	}
	
	// TODO: passed POIs are not relevant
	/* ASS	... Aircraft Safety Sphere
	 * POI	... Points Of Interest Bounding Box
	 * ENV	... Environment
	 * P	... Policy Cost
	 * E	... Exceeded Policy Cost
	 * 
	 * Containment: ASS (= ENV and POI (= ENV
	 * Costs:		ASS <= ENV and POI <= ENV
	 * 
	 * 				ASS		POI		ENV
	 * LOW			<=0		<=0		<=P
	 * MODERATE		<=0		<=P		(0, P]
	 * SUBSTANTIAL	<=P		[0, P]	(0, E]
	 * SEVERE		[0, P]	(0, E]	E
	 * CRITICAL		(0, E]	E		E
	 * FATAL		E		*		*
	 * 
	 * 0, 0, E -> SUB
	 * 0, P, E -> SUB, SEV
	 * 0, E, E -> SEV
	 * P, 0, E -> SUB (aircraft outside POI bounding box)
	 * P, P, E -> SUB, SEV
	 * P, E, E -> SEV, CRT
	 * E, 0, E -> FAT (aircraft outside POI bounding box)
	 * E, P, E -> FAT (aircraft outside POI bounding box)
	 * E, E, E -> CRT, FAT
	 * 
	 * 0, 0, P -> LOW, MOD, SUB
	 * 0, P, P -> MOD, SUB
	 * P, 0, P -> SUB (aircraft outside POI bounding box)
	 * P, P, P -> SUB
	 * 
	 * The severity has an influence on the expansion of A* since higher costs
	 * in less difficult cost areas reduce the effect of the heuristic and may
	 * lead to a longer search if the heuristic is not inflated accordingly.
	 * Similarly it increases the effect of the heuristic and may lead to a
	 * faster search in more difficult high cost areas if the heuristic remains
	 * deflated. For RRT it affects the choice of biases in particular and
	 * sampling distribution. Nearby and expensive obstacles may require a
	 * lower goal bias, more scattered sampling, and higher cost bias. In a low
	 * severity scenario a higher goal bias, more focused sampling, and higher
	 * distance bias may be the better choices.
	 * 
	 * Critical and fatal cases may be solvable if the scenario is difficult as
	 * opposed to simple if the explained approach is taken. A critical and fatal
	 * severity with a simple scenario may be impossible to solve.
	 */
	
	/**
	 * Creates a low severity based on features.
	 * 
	 * @param features the features
	 *  
	 * @return a low severity based on the features
	 */
	public static Severity createLow(Features features) {
		Severity low = new Severity(features);
		
		// no obstacle cost with positive cost inside aircraft safety radius, and
		low.setAircraftSafetyObstaclesCostMaxRange(Range.openClosed(Double.NEGATIVE_INFINITY, 0d));
		// no obstacle cost with positive cost inside POI bounding box, and
		low.setPoiObstaclesCostMaxRange(Range.openClosed(Double.NEGATIVE_INFINITY, 0d));
		// up to no risk-exceeding obstacle cost inside environment
		low.setEnvironmentObstaclesCostPolicyRange(Range.lessThan(Double.POSITIVE_INFINITY));
			
		return low;
	}
	
	/**
	 * Creates a moderate severity based on features.
	 * 
	 * @param features the features
	 * 
	 * @return a moderate severity based on the features
	 */
	public static Severity createModerate(Features features) {
		Severity moderate = new Severity(features);
		
		// no obstacle cost with positive cost inside aircraft safety radius, and
		moderate.setAircraftSafetyObstaclesCostMaxRange(Range.openClosed(Double.NEGATIVE_INFINITY, 0d));
		// up to no risk-exceeding obstacle cost inside POI bounding box, and
		moderate.setPoiObstaclesCostPolicyRange(Range.lessThan(Double.POSITIVE_INFINITY));
		// positive up to no risk-exceeding obstacle cost inside environment
		moderate.setEnvironmentObstaclesCostMinRange(Range.greaterThan(0d));
		moderate.setEnvironmentObstaclesCostPolicyRange(Range.lessThan(Double.POSITIVE_INFINITY));
		
		return moderate;
	}
	
	/**
	 * Creates a substantial severity based on features.
	 * 
	 * @param features the features
	 * 
	 * @return a substantial severity based on the features
	 */
	public static Severity createSubstantial(Features features) {
		Severity substantial = new Severity(features);
		
		// no obstacle cost with positive cost inside aircraft safety radius, and
		substantial.setAircraftSafetyObstaclesPolicyRange(Range.lessThan(Double.POSITIVE_INFINITY));
		
		// positive up to no risk-exceeding obstacle cost inside POI bounding box, and
		substantial.setPoiObstaclesCostMinRange(Range.atLeast(0d));
		substantial.setPoiObstaclesCostPolicyRange(Range.lessThan(Double.POSITIVE_INFINITY));
		
		// positive up to risk-exceeding obstacle cost inside environment; or
		substantial.setEnvironmentObstaclesCostMinRange(Range.greaterThan(0d));
		
		return substantial;
	}
	
	/**
	 * Creates a severe severity based on features.
	 * 
	 * @param features the features
	 * 
	 * @return a severe severity based on the features
	 */
	public static Severity createSevere(Features features) {
		Severity severe = new Severity(features);
		
		// positive up to no risk-exceeding obstacle cost inside aircraft safety radius, and
		severe.setAircraftSafetyObstaclesCostMinRange(Range.atLeast(0d));
		severe.setAircraftSafetyObstaclesPolicyRange(Range.lessThan(Double.POSITIVE_INFINITY));
		
		// positive up to risk-exceeding obstacle cost inside POI bounding box, and
		severe.setPoiObstaclesCostMinRange(Range.greaterThan(0d));
		
		// risk-exceeding obstacle cost inside environment; or
		severe.setEnvironmentObstaclesCostPolicyRange(Range.singleton(Double.POSITIVE_INFINITY));
		
		return severe;
	}
	
	/**
	 * Creates a critical severity based on features.
	 * 
	 * @param features the features
	 * 
	 * @return a critical severity based on the features
	 */
	public static Severity createCritical(Features features) {
		Severity critical = new Severity(features);
		
		// positive up to risk-exceeding obstacle cost inside aircraft safety radius, and
		critical.setAircraftSafetyObstaclesCostMinRange(Range.greaterThan(0d));
		
		// risk-exceeding obstacle cost inside POI bounding box, and
		critical.setPoiObstaclesCostPolicyRange(Range.singleton(Double.POSITIVE_INFINITY));
		
		// risk-exceeding obstacle cost inside environment; or
		critical.setEnvironmentObstaclesCostPolicyRange(Range.singleton(Double.POSITIVE_INFINITY));
		
		return critical;
	}
	
	/**
	 * Creates a fatal severity based on features
	 * 
	 * @param features the features
	 * 
	 * @return a fatal severity based on features
	 */
	public static Severity createFatal(Features features) {
		Severity fatal = new Severity(features);
		
		// risk-exceeding obstacle cost inside aircraft safety radius
		fatal.setAircraftSafetyObstaclesPolicyRange(Range.singleton(Double.POSITIVE_INFINITY));
		
		return fatal;
	}
	
	/**
	 * Determines whether or not this severity covers given features.
	 * 
	 * @param features the features
	 * 
	 * @return true if this severity covers the features, false otherwise
	 * 
	 * @see Criticality#covers(Features)
	 */
	@Override
	public boolean covers(Features features) {
		boolean covers = super.covers(features);
		
		if (covers) {
			covers =  this.getAircraftSafetyObstaclesCostMaxRange().contains(features.get(Features.FEATURE_AIRCRAFT_SAFETY_OBSTACLES_COST_MAX))
				&& this.getAircraftSafetyObstaclesCostMinRange().contains(features.get(Features.FEATURE_AIRCRAFT_SAFETY_OBSTACLES_COST_MIN))
				&& this.getAircraftSafetyObstaclesPolicyRange().contains(features.get(Features.FEATURE_AIRCRAFT_SAFETY_OBSTACLES_COST_POLICIES))
				&& this.getEnvironmentObstaclesCostMaxRange().contains(features.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_MAX))
				&& this.getEnvironmentObstaclesCostMinRange().contains(features.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_MIN))
				&& this.getEnvironmentObstaclesCostPolicyRange().contains(features.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_POLICIES))
				&& this.getPoiObstaclesCostMaxRange().contains(features.get(Features.FEATURE_POIS_OBSTACLES_COST_MAX))
				&& this.getPoiObstaclesCostMinRange().contains(features.get(Features.FEATURE_POIS_OBSTACLES_COST_MIN))
				&& this.getPoiObstaclesCostPolicyRange().contains(features.get(Features.FEATURE_POIS_OBSTACLES_COST_POLICIES));
		}
		
		return covers;
	}
	
}
