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
package com.cfar.swim.worldwind.managers.heuristic;

import java.util.ArrayList;
import java.util.List;

import com.cfar.swim.worldwind.flight.FlightPhase;
import com.cfar.swim.worldwind.managing.Difficulty;
import com.cfar.swim.worldwind.managing.FeatureTuning;
import com.cfar.swim.worldwind.managing.Features;
import com.cfar.swim.worldwind.managing.PlannerTuning;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.planners.rrt.Extension;
import com.cfar.swim.worldwind.planners.rrt.Sampling;
import com.cfar.swim.worldwind.planners.rrt.Strategy;
import com.cfar.swim.worldwind.registries.Properties;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.planners.cgs.OADStarProperties;
import com.cfar.swim.worldwind.registries.planners.rrt.OADRRTreeProperties;

/**
 * Realizes a heuristic planner tuning based on features.
 * 
 * @author Stephan Heinemann
 *
 * @see PlannerTuning
 */
public class HeuristicPlannerTuning extends PlannerTuning {
	
	/** the default serial identification of this heuristic planner tuning */
	private static final long serialVersionUID = 1L;

	/**
	 * Constructs a new heuristic planner tuning based on a planner
	 * specification and features.
	 * 
	 * @param specification the planner specification
	 * @param features the features
	 * 
	 * @throws IllegalArgumentException if the planner specification or
	 *         features are invalid
	 *
	 * @see PlannerTuning#PlannerTuning(Specification, Features)
	 */
	public HeuristicPlannerTuning(
			Specification<Planner> specification, Features features) {
		super(specification, features);
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
		ArrayList<Properties<Planner>> properties = new ArrayList<>();
		
		if (specification.getId().equals(Specification.PLANNER_MGP_ID)) {
			OADStarProperties candidate = (OADStarProperties)
					specification.getProperties(); //.clone();
			this.tuneManagedGridPlanner(candidate, features);
			properties.add(candidate);
		} else if (specification.getId().equals(Specification.PLANNER_MTP_ID)) {
			OADRRTreeProperties candidate = (OADRRTreeProperties)
					specification.getProperties(); //.clone();
			this.tuneManageTreePlanner(candidate, features);
			properties.add(candidate);
		} // TODO: ManagedRoadmapPlanner
		
		return properties;
	}
	
	/**
	 * Tunes the properties of a managed grid planner according to features.
	 * 
	 * @param properties the properties of the managed grid planner to be tuned
	 * @param features the features to tune the managed grid planner for
	 */
	private void tuneManagedGridPlanner(OADStarProperties properties, Features features) {
		if (0d == features.get(Features.FEATURE_POIS_OBSTACLES_COUNT)) {
			// quality
			properties.setMinimumQuality(features.get(Features.FEATURE_ENVIRONMENT_COST_BASE));
			properties.setMaximumQuality(features.get(Features.FEATURE_ENVIRONMENT_COST_BASE));
			properties.setQualityImprovement(features.get(Features.FEATURE_ENVIRONMENT_COST_BASE));
			properties.setSignificantChange(0.01d);
		} else {
			// quality
			properties.setMinimumQuality(features.get(Features.FEATURE_ENVIRONMENT_COST_BASE)
					+ features.get(Features.FEATURE_POIS_OBSTACLES_COST_AVG));
			
			// reduce search time in less critical situations for multi-part problems
			if (FlightPhase.areCruise(features)) {
				properties.setMaximumQuality(Math.max(1d, 0.75d * properties.getMinimumQuality()));
			} else if (FlightPhase.areTransition(features)) {
				properties.setMaximumQuality(Math.max(1d, 0.50d * properties.getMinimumQuality()));
			} else if (FlightPhase.areTerminal(features)) {
				properties.setMaximumQuality(Math.max(1d, 0.25d * properties.getMinimumQuality()));
			} else {
				properties.setMaximumQuality(1.0d);
			}
			
			double qi = Math.abs(properties.getMinimumQuality() - properties.getMaximumQuality()) / 5d;
			if (0d < qi) {
				properties.setQualityImprovement(qi);
			} else {
				properties.setQualityImprovement(0.1d);
			}
			properties.setSignificantChange(0.5d);
		}
	}
	
	/**
	 * Tunes the properties of a managed tree planner according to features.
	 * 
	 * @param properties the properties of the managed tree planner to be tuned
	 * @param features the features to tune the managed tree planner for
	 */
	private void tuneManageTreePlanner(OADRRTreeProperties properties, Features features) {
		if (0d == features.get(Features.FEATURE_POIS_OBSTACLES_COUNT)) {
			// sampling
			properties.setSampling(Sampling.ELLIPSOIDAL);
			// goal bias
			properties.setBias(100); // ignore terrain and capabilities
			// epsilon - extension distance
			properties.setEpsilon(features.get(Features.FEATURE_POIS_DISTANCE_MAX));
			// extension strategy and technique
			properties.setStrategy(Strategy.CONNECT);
			properties.setExtension(Extension.FEASIBLE);
			// goal threshold
			properties.setGoalThreshold(0d);
			// maximum iterations
			properties.setMaxIterations(1); // ignore terrain and capabilities
			// neighborhood
			properties.setNeighborLimit(1);
			// quality
			properties.setMinimumQuality(0.0d);
			properties.setMaximumQuality(0.0d);
			properties.setQualityImprovement(0.1d);
			properties.setSignificantChange(0.01d);
		} else {
			// sampling
			if (FlightPhase.areCruise(features) || FlightPhase.areTransition(features)) {
				properties.setSampling(Sampling.ELLIPSOIDAL);
			} else {
				properties.setSampling(Sampling.UNIFORM);
			}
			
			// goal bias
			if (FlightPhase.areCruise(features)) {
				properties.setBias(10);
			} else if (FlightPhase.areTransition(features)) {
				properties.setBias(5);
			} else if (FlightPhase.areTerminal(features)) {
				properties.setBias(3);
			} else {
				properties.setBias(1);
			}
			
			// epsilon - extension distance
			properties.setEpsilon(0.05d * features.get(Features.FEATURE_POIS_DISTANCE_MAX));
			
			// goal threshold
			properties.setGoalThreshold(properties.getMaxLandingHorizontalError() / 2d);
			
			// extension strategy and technique
			if (Difficulty.areLow(features) || Difficulty.areModerate(features)) {
				properties.setStrategy(Strategy.CONNECT);
			} else {
				properties.setStrategy(Strategy.EXTEND);
			}
			properties.setExtension(Extension.FEASIBLE);
			
			// maximum iterations
			// reduce search time in less critical situations for multi-part problems
			if (FlightPhase.areCruise(features)) {
				properties.setMaxIterations(500);
			} else if (FlightPhase.areTransition(features)) {
				properties.setMaxIterations(1000);
			} else if (FlightPhase.areTerminal(features)) {
				properties.setMaxIterations(1500);
			} else {
				properties.setMaxIterations(2000);
			}
			
			// neighborhood
			if (Difficulty.areLow(features)) {
				properties.setNeighborLimit(1);
			} else if (Difficulty.areModerate(features)) {
				properties.setNeighborLimit(2);
			} else if (Difficulty.areSubstantial(features)) {
				properties.setNeighborLimit(3);
			} else if (Difficulty.areSevere(features)) {
				properties.setNeighborLimit(4);
			} else {
				properties.setNeighborLimit(5);
			}
			
			// quality
			properties.setMinimumQuality(0d);
			// reduce search time in less critical situations for multi-part problems
			if (FlightPhase.areCruise(features)) {
				properties.setMaximumQuality(0.25d);
			} else if (FlightPhase.areTransition(features)) {
				properties.setMaximumQuality(0.50d);
			} else if (FlightPhase.areTerminal(features)) {
				properties.setMaximumQuality(0.75d);
			} else {
				properties.setMaximumQuality(1.0d);
			}
			
			double qi = Math.abs(properties.getMinimumQuality() - properties.getMaximumQuality()) / 5d;
			if (0d < qi) {
				properties.setQualityImprovement(qi);
			} else {
				properties.setQualityImprovement(0.1d);
			}
			properties.setSignificantChange(0.5d);
		}
	}
	
}
