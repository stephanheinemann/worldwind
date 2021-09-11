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

import java.util.ArrayList;
import java.util.List;

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
	 * @param specification the specification to be tuned.
	 * @param features the features
	 * 
	 * @return the tuned candidate properties for the specification
	 * 
	 * @see FeatureTuning#tune(Specification, Features)
	 */
	@Override
	public List<Properties<Planner>> tune(
			Specification<Planner> specification, Features features) {
		ArrayList<Properties<Planner>> candidates = new ArrayList<>();
		
		if (specification.getId().equals(Specification.PLANNER_OADS_ID)) {
			OADStarProperties candidate = (OADStarProperties)
					specification.getProperties().clone();
			// TODO: add suitable candidates
			candidates.add(candidate);
		} else if (specification.getId().equals(Specification.PLANNER_OADRRT_ID)) {
			OADRRTreeProperties candidate = (OADRRTreeProperties)
					specification.getProperties().clone();
			candidate.getBias();
			
			// TODO: feasibility
			if (0 == (Integer) features.get(Features.FEATURE_POIS_OBSTACLES_COUNT)) {
				candidate.setSampling(Sampling.ELLIPSOIDAL);
				candidate.setBias(50); // possible terrain?
				candidate.setEpsilon((Double) features.get(Features.FEATURE_POIS_DISTANCE_MAX));
				candidate.setStrategy(Strategy.CONNECT);
				candidate.setExtension(Extension.LINEAR); // TODO
				candidate.setGoalThreshold(0);
				candidate.setMaxIterations(50); // TODO
				candidate.setSignificantChange(0.1d); // TODO
				candidate.setNeighborLimit(1);
				candidate.setMinimumQuality(1.0d);
				candidate.setMaximumQuality(1.0d);
				candidate.setQualityImprovement(1.0d);
			} else {
				candidate.setBias(5);
				candidate.setEpsilon(0.05d * (Double) features.get(Features.FEATURE_POIS_DISTANCE_MAX));
				candidate.setGoalThreshold(candidate.getMaxLandingHorizontalError() / 2d);
				candidate.setStrategy(Strategy.EXTEND);
				candidate.setMaxIterations(1500);
				candidate.setNeighborLimit(5);
				candidate.setMinimumQuality(0.1d);
				candidate.setMaximumQuality(1.0d);
				candidate.setQualityImprovement(0.1d);
			}
			/*
			candidate.getExtension();
			candidate.getGoalThreshold();
			candidate.getMaximumQuality();
			candidate.getMaxIterations();
			candidate.getMinimumQuality();
			candidate.getNeighborLimit();
			candidate.getQualityImprovement();
			candidate.getSampling();
			candidate.getSignificantChange();
			*/
			/*
			if (0 == (Integer) features.get(Features.FEATURE_POIS_OBSTACLES_COUNT)) {
				candidate.setStrategy(Strategy.CONNECT);
			} else {
				candidate.setStrategy(Strategy.EXTEND);
			}
			*/
			
			candidates.add(candidate);
		}
		
		return candidates;
	}

}
