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
package com.cfar.swim.worldwind.registries.managers;

import java.time.Duration;

import com.cfar.swim.worldwind.managers.AutonomicManager;
import com.cfar.swim.worldwind.managers.HeuristicAutonomicManager;
import com.cfar.swim.worldwind.registries.AbstractFactory;
import com.cfar.swim.worldwind.registries.Specification;

/**
 * Realizes a manager factory to create autonomic managers according to
 * customized manager specifications.
 * 
 * @author Stephan Heinemann
 * 
 * @see AbstractFactory
 * @see Specification
 */
public class ManagerFactory extends AbstractFactory<AutonomicManager> {
	
	/**
	 * Creates a new autonomic manager according to the customized manager
	 * specification of this manager factory.
	 * 
	 * @return the created autonomic manager, or null if no autonomic manager
	 *         could be created
	 * 
	 * @see AbstractFactory#createInstance()
	 */
	@Override
	public AutonomicManager createInstance() {
		AutonomicManager manager = null;
		
		if (this.hasSpecification()) {
			if (this.specification.getId().equals(Specification.MANAGER_HEURISTIC_ID)) {
				HeuristicManagerProperties properties = (HeuristicManagerProperties) this.specification.getProperties();
				manager = new HeuristicAutonomicManager();
				manager.setCostPolicy(properties.getCostPolicy());
				manager.setRiskPolicy(properties.getRiskPolicy());
				manager.setFeatureHorizon(Duration.ofSeconds(Math.round(properties.getFeatureHorizon() * 60d)));
			}
			// TODO: ROAR, SMAC autonomic managers
		}
			
		return manager;
	}

}
