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

import java.util.HashMap;
import java.util.HashSet;
import java.util.OptionalDouble;
import java.util.Set;

/**
 * Realizes a performance reputation of tunings.
 * 
 * @author Stephan Heinemann
 *
 * @see Tuning
 * @see Performance
 */
public class Reputation extends HashMap<Tuning<?>, Set<Performance>> {
	
	/** the default serial identification of this reputation */
	private static final long serialVersionUID = 1L;
	
	/**
	 * Adds a tuning performance to this reputation.
	 * 
	 * @param tuning the tuning to be added
	 * @param performance the performance of the tuning
	 */
	public void addTuningPerformance(Tuning<?> tuning, Performance performance) {
		if (this.containsKey(tuning)) {
			this.get(tuning).add(performance);
		} else {
			HashSet<Performance> performances = new HashSet<>();
			performances.add(performance);
			this.put(tuning, performances);
		}
	}
	
	/**
	 * Removes the performances of a tuning from this reputation.
	 * 
	 * @param tuning the tuning to be removed
	 */
	public void removeTuningPerformances(Tuning<?> tuning) {
		this.remove(tuning);
	}
	
	/**
	 * Gets the average tuning performance of a tuning in this reputation.
	 * 
	 * @param tuning the tuning
	 * 
	 * @return the average tuning performance, if present
	 */
	public OptionalDouble getAverageTuningPerformance(Tuning<?> tuning) {
		OptionalDouble performance = OptionalDouble.empty();
		
		// TODO: filter for system information
		if (this.containsKey(tuning)) {
			performance = this.get(tuning).stream()
					.map(p -> p.get()).mapToDouble(Double::valueOf).average();
		}
		
		return performance;
	}
	
	/**
	 * Gets the maximum tuning performance of a tuning in this reputation.
	 * 
	 * @param tuning the tuning
	 * 
	 * @return the maximum tuning performance, if present
	 */
	public OptionalDouble getMaximumTuningPerformance(Tuning<?> tuning) {
		OptionalDouble performance = OptionalDouble.empty();
		
		// TODO: filter for system information
		if (this.containsKey(tuning)) {
			performance = this.get(tuning).stream()
					.map(p -> p.get()).mapToDouble(Double::valueOf).max();
		}
		
		return performance;
	}
	
	/**
	 * Gets the minimum tuning performance of a tuning in this reputation.
	 * 
	 * @param tuning the tuning
	 * 
	 * @return the minimum tuning performance, if present
	 */
	public OptionalDouble getMinimumTuningPerformance(Tuning<?> tuning) {
		OptionalDouble performance = OptionalDouble.empty();
		
		// TODO: filter for system information
		if (this.containsKey(tuning)) {
			performance = this.get(tuning).stream()
					.map(p -> p.get()).mapToDouble(Double::valueOf).min();
		}
		
		return performance;
	}
	
	/**
	 * Gets the string representation of this reputation.
	 * 
	 * @return the string representation of this reputation
	 * 
	 * @see Object#toString()
	 */
	@Override
	public String toString() {
		String reputation = "";
		
		for (Tuning<?> tuning : this.keySet()) {
			reputation = reputation.concat(tuning.toString() + ": ");
			for (Performance performance : this.get(tuning)) {
				reputation = reputation.concat(performance.toString() + " ");
			}
			reputation = reputation.trim().concat("\n");
		}
		
		return reputation;
	}
	
	// TODO: reputations need to be visualized
}
