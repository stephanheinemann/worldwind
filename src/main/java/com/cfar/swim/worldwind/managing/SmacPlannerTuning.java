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

import java.util.List;

import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.registries.Properties;
import com.cfar.swim.worldwind.registries.Specification;

/**
 * Realizes a SMAC planner tuning based on features.
 * 
 * @author Stephan Heinemann
 *
 * @see PlannerTuning
 */
public class SmacPlannerTuning extends PlannerTuning {
	
	/** the default serial identification of this SMAC planner tuning */
	private static final long serialVersionUID = 1L;
	
	/**
	 * Constructs a new SMAC planner tuning based on a planner
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
	public SmacPlannerTuning(
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
		// TODO: only select configuration based on loaded model (warm-start)
		// TODO Auto-generated method stub
		return null;
	}
	
}
