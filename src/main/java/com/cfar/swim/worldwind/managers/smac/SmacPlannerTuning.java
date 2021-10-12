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
package com.cfar.swim.worldwind.managers.smac;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.flight.FlightPhase;
import com.cfar.swim.worldwind.managers.heuristic.HeuristicPlannerTuning;
import com.cfar.swim.worldwind.managing.FeatureTuning;
import com.cfar.swim.worldwind.managing.Features;
import com.cfar.swim.worldwind.managing.KnowledgeBase;
import com.cfar.swim.worldwind.managing.PlannerTuning;
import com.cfar.swim.worldwind.managing.Reputation;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.registries.Properties;
import com.cfar.swim.worldwind.registries.Specification;

/**
 * Realizes a SMAC planner tuning based on features.
 * 
 * @author Stephan Heinemann
 *
 * @see HeuristicPlannerTuning
 */
public class SmacPlannerTuning extends HeuristicPlannerTuning {
	
	/** the default serial identification of this SMAC planner tuning */
	private static final long serialVersionUID = 1L;
	
	/** the knowledge base of this SMAC planner tuning */
	private transient final KnowledgeBase knowledgeBase;
	
	/** the flight phases of this SMAC planner tuning */
	private transient final Set<FlightPhase> flightPhases;
	
	/**
	 * Constructs a new SMAC planner tuning based on a planner specification
	 * and features.
	 * 
	 * @param specification the planner specification
	 * @param features the features
	 * 
	 * @throws IllegalArgumentException if the planner specification or
	 *         features are invalid
	 *
	 * @see PlannerTuning#PlannerTuning(Specification, Features)
	 */
	public SmacPlannerTuning(
			Specification<Planner> specification, Features features) {
		super(specification, features);
		this.knowledgeBase = null;
		this.flightPhases = null;
	}
	
	/**
	 * Constructs a new SMAC planner tuning based on a planner specification
	 * and features as well as an existing knowledge base and associable flight
	 * phases.
	 * 
	 * @param specification the planner specification
	 * @param features the features
	 * @param knowledgeBase the existing knowledge base
	 * @param flightPhases the associable flight phases
	 * 
	 * @throws IllegalArgumentException if the planner specification or
	 *         features are invalid
	 *
	 * @see PlannerTuning#PlannerTuning(Specification, Features)
	 */
	public SmacPlannerTuning(
			Specification<Planner> specification, Features features,
			KnowledgeBase knowledgeBase, Set<FlightPhase> flightPhases) {
		super(specification, features);
		this.knowledgeBase = knowledgeBase;
		this.flightPhases = flightPhases;
	}
	
	/**
	 * Tunes the specification of a planner according to features.
	 * 
	 * @param specification the planner specification to be tuned
	 * @param features the features to tune the planner specification for
	 * 
	 * @return the tuned candidate properties for the specification
	 * 
	 * @see FeatureTuning#tune(Specification, Features)
	 */
	@Override
	public List<Properties<Planner>> tune(
			Specification<Planner> specification, Features features) {
		List<Properties<Planner>> properties = new ArrayList<>();
		
		if ((null == this.knowledgeBase) || (null == this.flightPhases)) {
			// revert to heuristic tuning
			properties = super.tune(specification, features);
		} else {
			Reputation reputation = this.knowledgeBase.getReputation();
			List<PlannerTuning> tunings = reputation.keySet().stream()
					.filter(t -> (t instanceof PlannerTuning))
					.map(t -> (PlannerTuning) t)
					.filter(t -> t.getSpecification().getId().equals(specification.getId()))
					.collect(Collectors.toUnmodifiableList());
			Optional<PlannerTuning> tuning = tunings.stream()
					.filter(t -> t.getFeatures().equals(features))
					.findAny();
			
			// TODO: maybe do not use existing tuning to allow for improvements?
			if (tuning.isPresent()) {
				// apply existing tuning
				properties.add(tuning.get().getSpecification().getProperties());
			} else {
				// apply tuning for associated flight phases
				List<FlightPhase> flightPhases = this.flightPhases.stream()
						.filter(fp -> fp.covers(features))
						.collect(Collectors.toUnmodifiableList());
				if (flightPhases.isEmpty()) {
					// revert to heuristic tuning
					properties = super.tune(specification, features);
				} else {
					// associate covered flight phases
					List<PlannerTuning> flightPhaseTunings = new ArrayList<>();
					for (FlightPhase flightPhase : flightPhases) {
						flightPhaseTunings.addAll(tunings.stream()
							.filter(t -> flightPhase.covers(t.getFeatures()))
							.collect(Collectors.toUnmodifiableList()));
					}
					if (flightPhaseTunings.isEmpty()) {
						// revert to heuristic tuning
						properties = super.tune(specification, features);
					} else {
						// apply flight phase tunings
						properties.addAll(flightPhaseTunings.stream()
								.map(t -> t.getSpecification().getProperties())
								.collect(Collectors.toUnmodifiableList()));
					}
				}
			}
		}
		
		return properties;
	}
	
}
