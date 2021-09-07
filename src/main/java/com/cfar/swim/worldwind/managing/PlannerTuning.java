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
 * Abstracts a feature tuning of a planner.
 * 
 * @author Stephan Heinemann
 *
 * @see FeatureTuning
 */
public abstract class PlannerTuning extends FeatureTuning<Planner> {
	
	/**
	 * Constructs a new planner tuning based on a planner specification and
	 * features.
	 * 
	 * @param specification the planner specification
	 * @param features the features
	 * 
	 * @throws IllegalArgumentException if the planner specification or
	 *         features are invalid
	 * 
	 * @see FeatureTuning#FeatureTuning(Specification, Features)
	 */
	public PlannerTuning(
			Specification<Planner> specification, Features features) {
		super(specification, features);
	}
	
	/**
	 * Tunes the specification of the planner of this tuning.
	 * 
	 * @return the tuned candidate properties for the specification
	 * 
	 * @see Tuning#tune()
	 */
	@Override
	public List<Properties<Planner>> tune() {
		return this.tune(this.getSpecification());
	}
	
	/**
	 * Tunes the specification of a planner according to the features
	 * of this planner tuning.
	 * 
	 * @param specification the specification to be tuned.
	 * 
	 * @return the tuned candidate properties for the specification
	 * 
	 * @see Tuning#tune(Specification)
	 */
	@Override
	public List<Properties<Planner>> tune(
			Specification<Planner> specification) {
		return this.tune(specification, this.getFeatures());
	}
	
	/**
	 * Tunes the specification of a planner according to features.
	 * 
	 * @param specification the specification to be tuned.
	 * 
	 * @return the tuned candidate properties for the specification
	 * 
	 * @see FeatureTuning#tune(Specification, Features)
	 */
	@Override
	public abstract List<Properties<Planner>> tune(
			Specification<Planner> specification, Features features);
	
}
