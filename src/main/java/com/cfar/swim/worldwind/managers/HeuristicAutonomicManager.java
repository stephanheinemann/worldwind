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
package com.cfar.swim.worldwind.managers;

import com.cfar.swim.worldwind.managing.Features;
import com.cfar.swim.worldwind.managing.HeuristicPlannerTuning;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.util.Identifiable;

/**
 * Realizes a heuristic autonomic manager.
 * 
 * @author Stephan Heinemann
 *
 */
public class HeuristicAutonomicManager extends AbstractAutonomicManager {
	
	/**
	 * Gets the identifier of this heuristic autonomic manager.
	 * 
	 * @return the identifier of this heuristic autonomic manager
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return Specification.MANAGER_HEURISTIC_ID;
	}
	
	/**
	 * Creates a new heuristic planner tuning for this heuristic autonomic
	 * manager based on a planner specification and features.
	 * 
	 * @param specification the planner specification
	 * @param features the features
	 * 
	 * @return the created heuristic planner tuning
	 * 
	 * @see AbstractAutonomicManager#createPlannerTuning(Specification, Features)
	 */
	@Override
	public HeuristicPlannerTuning createPlannerTuning(
			Specification<Planner> specification, Features features) {
		return new HeuristicPlannerTuning(specification, features);
	}
	
}
