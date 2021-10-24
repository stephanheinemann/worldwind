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

import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.OptionalDouble;
import java.util.Set;
import java.util.TreeSet;

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
	
	/** the current performance context */
	public static final PerformanceContext PERFORMANCE_CONTEXT = new PerformanceContext();
	
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
	 * Determines whether or not this reputation has performances for a tuning.
	 * 
	 * @param tuning the tuning
	 * 
	 * @return true if this reputation has performances for the tuning,
	 *         false otherwise
	 */
	public boolean hasPerformances(Tuning<?> tuning) {
		return this.containsKey(tuning) && !this.get(tuning).isEmpty();
	}
	
	/**
	 * Determines whether or not this reputation has performances for a tuning
	 * given a performance context.
	 * 
	 * @param tuning the tuning
	 * @param context the performance context
	 * 
	 * @return true if this reputation has performances for the tuning in the
	 *         performance context, false otherwise
	 */
	public boolean hasPerformances(Tuning<?> tuning, PerformanceContext context) {
		return this.containsKey(tuning) && this.get(tuning).stream()
				.anyMatch(p -> p.getContext().equals(context));
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
		
		if (this.containsKey(tuning)) {
			performance = this.get(tuning).stream()
					.map(p -> p.get()).mapToDouble(Double::valueOf).average();
		}
		
		return performance;
	}
	
	/**
	 * Gets the average tuning performance of a tuning in this reputation
	 * within a given performance context.
	 * 
	 * @param tuning the tuning
	 * @param context the performance context
	 * 
	 * @return the average tuning performance within the performance context,
	 *         if present
	 */
	public OptionalDouble getAverageTuningPerformance(
			Tuning<?> tuning, PerformanceContext context) {
		OptionalDouble performance = OptionalDouble.empty();
		
		if (this.containsKey(tuning)) {
			performance = this.get(tuning).stream()
					.filter(p -> p.getContext().equals(context))
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
		
		if (this.containsKey(tuning)) {
			performance = this.get(tuning).stream()
					.map(p -> p.get()).mapToDouble(Double::valueOf).max();
		}
		
		return performance;
	}
	
	/**
	 * Gets the maximum tuning performance of a tuning in this reputation
	 * within a given performance context.
	 * 
	 * @param tuning the tuning
	 * @param context the performance context
	 * 
	 * @return the maximum tuning performance within the performance context,
	 *         if present
	 */
	public OptionalDouble getMaximumTuningPerformance(
			Tuning<?> tuning, PerformanceContext context) {
		OptionalDouble performance = OptionalDouble.empty();
		
		if (this.containsKey(tuning)) {
			performance = this.get(tuning).stream()
					.filter(p -> p.getContext().equals(context))
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
		
		if (this.containsKey(tuning)) {
			performance = this.get(tuning).stream()
					.map(p -> p.get()).mapToDouble(Double::valueOf).min();
		}
		
		return performance;
	}
	
	/**
	 * Gets the minimum tuning performance of a tuning in this reputation
	 * within a given performance context.
	 * 
	 * @param tuning the tuning
	 * @param context the performance context
	 * 
	 * @return the minimum tuning performance within the performance context,
	 *         if present
	 */
	public OptionalDouble getMinimumTuningPerformance(
			Tuning<?> tuning, PerformanceContext context) {
		OptionalDouble performance = OptionalDouble.empty();
		
		if (this.containsKey(tuning)) {
			performance = this.get(tuning).stream()
					.filter(p -> p.getContext().equals(context))
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
		String reputation = Reputation.PERFORMANCE_CONTEXT.toString() + "\n\n";
		
		for (Tuning<?> tuning : this.keySet()) {
			reputation = reputation.concat(tuning.toString() + ":\n");
			TreeSet<Performance> performances = new TreeSet<Performance>(
					new Comparator<Performance>() {
						/**
						 * Compares two performances according to their epoch
						 * (primary) and quantity (secondary) keys.
						 * 
						 * @param p1 the first performance
						 * @param p2 the second performance
						 * 
						 * @return -1, 0, or 1, if the first performance is earlier,
						 *         equal, or later than the second performance,
						 *         respectively
						 */
						@Override
						public int compare(Performance p1, Performance p2) {
							int result = p1.getEpoch().compareTo(p2.getEpoch());
							
							if (0 == result) {
								result = p1.getQuantity().compareTo(p2.getQuantity());
							}
							
							return result;
						}
					});
			performances.addAll(this.get(tuning));	
			for (Performance performance : performances) {
				reputation = reputation.concat(performance.toString() + "\n");
			}
			reputation = reputation.trim().concat("\n\n");
		}
		
		return reputation;
	}
	
	// TODO: reputations need to be visualized
}
