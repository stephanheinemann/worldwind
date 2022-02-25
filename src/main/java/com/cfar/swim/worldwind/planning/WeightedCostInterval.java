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

import java.time.ZonedDateTime;
import java.util.Objects;

/**
 * Realizes a weighted cost interval based on a cost interval.
 * 
 * @see CostInterval
 * 
 * @author Stephan Heinemann
 *
 */
public class WeightedCostInterval extends CostInterval {
	
	/** the weight of this weighted cost interval */
	private double weight = 1.0;
	
	/**
	 * Constructs a weighted cost (instant) interval based on the current UTC
	 * time for start and end times with a specified identifier, a cost of
	 * zero and a weight of one.
	 * 
	 * @param id the identifier of this weighted cost interval
	 */
	public WeightedCostInterval(String id) {
		super(id);
	}
	
	/**
	 * Constructs a weighted cost (instant) interval based on a specified time
	 * for start and end times with a specified identifier, a cost of zero and
	 * a weight of one.
	 * 
	 * @param id the identifier of this weighted cost interval
	 * @param time the start and end time of this weighted cost interval
	 */
	public WeightedCostInterval(String id, ZonedDateTime time) {
		super(id, time);
	}
	
	/**
	 * Constructs a weighted cost interval from specified start and end times
	 * with a specified identifier, a cost of zero and a weight of one.
	 * 
	 * @param id the identifier of this weighted cost interval
	 * @param start the start time of this weighted cost interval
	 * @param end the end time of this weighted cost interval
	 */
	public WeightedCostInterval(String id, ZonedDateTime start, ZonedDateTime end) {
		super(id, start, end);
	}
	
	/**
	 * Constructs a weighted cost interval from specified start and end times
	 * with a specified identifier, cost and a weight of one.
	 * 
	 * @param id the identifier of this weighted cost interval
	 * @param start the start time of this weighted cost interval
	 * @param end the end time of this weighted cost interval
	 * @param cost the cost of this weighted cost interval
	 */
	public WeightedCostInterval(String id, ZonedDateTime start, ZonedDateTime end, double cost) {
		super(id, start, end, cost);
	}
	
	/**
	 * Constructs a weighted cost interval from specified start and end times
	 * with a specified identifier, cost and weight.
	 * 
	 * @param id the identifier of this weighted cost interval
	 * @param start the start time of this weighted cost interval
	 * @param end the end time of this weighted cost interval
	 * @param cost the cost of this weighted cost interval
	 * @param weight the weight of this weighted cost interval
	 */
	public WeightedCostInterval(String id, ZonedDateTime start, ZonedDateTime end, double cost, double weight) {
		super(id, start, end, cost);
		this.weight = weight;
	}
	
	/**
	 * Constructs a weighted cost interval from a specified cost interval with
	 * a weight of one.
	 * 
	 * @param costInterval the cost interval of this weighted cost interval
	 */
	public WeightedCostInterval(CostInterval costInterval) {
		super(costInterval.id, costInterval, costInterval.cost);
	}

	/**
	 * Constructs a weighted cost interval from a specified cost interval and
	 * weight.
	 * 
	 * @param costInterval the cost interval of this weighted cost interval
	 * @param weight the weight of this weighted cost interval
	 */
	public WeightedCostInterval(CostInterval costInterval, double weight) {
		super(costInterval.id, costInterval, costInterval.cost);
		this.weight = weight;
	}
	
	/**
	 * Gets the weight of this weighted cost interval.
	 * 
	 * @return the weight of this weighted cost interval
	 */
	public double getWeight() {
		return this.weight;
	}
	
	/**
	 * Sets the weight of this weighted cost interval.
	 * 
	 * @param weight the weight of this weighted cost interval
	 */
	public void setWeight(double weight) {
		this.weight = weight;
	}
	
	/**
	 * Gets the weighted cost of this weighted cost interval.
	 * 
	 * @return the weighted cost of this weighted cost interval
	 */
	public double getWeightedCost() {
		return this.weight * this.cost;
	}
	
	/**
	 * Determines whether or not this weighted cost interval equals another
	 * cost interval based on their identifiers, costs, weights as well as
	 * start and end times.
	 * 
	 * @param o the other weighted cost interval
	 * 
	 * @return true if this weighted cost interval equals the other weighted
	 *         cost interval based on their identifiers, costs, weights as well
	 *         as start and end times
	 * 
	 * @see CostInterval#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = super.equals(o);
		
		if (equals) {
			equals = this.weight == ((WeightedCostInterval) o).weight;
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this weighted cost interval based on its
	 * identifier, cost, weight as well as start and end times.
	 * 
	 * @return the hash code of this weighted cost interval based on its
	 *         identifier, cost, weight as well as start and end times
	 * 
	 * @see CostInterval#hashCode()
	 */
	@Override
	public int hashCode() {
		return Objects.hash(super.hashCode(), this.weight);
	}
	
}
