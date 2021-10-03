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
import com.cfar.swim.worldwind.managing.PlannerTuning;
import com.cfar.swim.worldwind.managing.SmacManagerMode;
import com.cfar.swim.worldwind.managing.SmacPlannerTuning;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.util.Identifiable;

/**
 * Realizes a sequential model-based algorithm configuration autonomic manager.
 *  
 * @author Stephan Heinemann
 *
 */
public class SmacAutonomicManager extends AbstractAutonomicManager implements AutonomicManager {
	
	/** the manager mode of this SMAC autonomic manager */
	private SmacManagerMode managerMode = SmacManagerMode.EXECUTING;
	
	/**
	 * Gets the identifier of this SMAC autonomic manager.
	 * 
	 * @return the identifier of this SMAC autonomic manager
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return Specification.MANAGER_SMAC_ID;
	}
	
	/**
	 * Gets the manager mode of this SMAC autonomic manager.
	 * 
	 * @return the manager mode of this SMAC autonomic manager
	 */
	public SmacManagerMode getManagerMode() {
		return this.managerMode;
	}
	
	/**
	 * Sets the manager mode of this SMAC autonomic manager.
	 * 
	 * @param managerMode the manager mode to be set
	 */
	public void setManagerMode(SmacManagerMode managerMode) {
		this.managerMode = managerMode;
	}
	
	/**
	 * Creates a new SMAC planner tuning for this SMAC autonomic manager based
	 * on a planner specification and features.
	 * 
	 * @param specification the planner specification
	 * @param features the features
	 * 
	 * @return the created SMAC planner tuning
	 * 
	 * @see AbstractAutonomicManager#createPlannerTuning(Specification, Features)
	 */
	@Override
	public PlannerTuning createPlannerTuning(
			Specification<Planner> specification, Features features) {
		return new SmacPlannerTuning(specification, features);
	}
	
	// TODO: override abstract manager methods for SmacManagerMode.TRAINING
	
}
