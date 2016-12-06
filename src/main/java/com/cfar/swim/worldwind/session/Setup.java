/**
 * Copyright (c) 2016, Stephan Heinemann (UVic Center for Aerospace Research)
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
package com.cfar.swim.worldwind.session;

import com.cfar.swim.worldwind.ai.Planner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.connections.AircraftConnection;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.registries.Specification;

/**
 * Realizes a setup for a session. A setup aggregates customized
 * specifications to be passed to factories.
 * 
 * @author Stephan Heinemann
 *
 */
public class Setup {
	
	/** the aircraft specification of this setup */
	Specification<Aircraft> aircraftSpecification;
	
	/** the environment specification of this setup */
	Specification<Environment> environmentSpecification;
	
	/** the planner specification of this setup */
	Specification<Planner> plannerSpecification;
	
	/** the aircraft connection specification of this setup */
	Specification<AircraftConnection> aircraftConnectionSpecification;
	// ...
	
	/**
	 * Gets the aircraft specification of this setup.
	 * 
	 * @return the aircraft specification of this setup
	 */
	public Specification<Aircraft> getAircraftSpecification() {
		return this.aircraftSpecification;
	}
	
	/**
	 * Sets the aircraft specification of this setup.
	 * 
	 * @param aircraftSpecification the aircraft specification to be set
	 */
	public void setAircraftSpecification(Specification<Aircraft> aircraftSpecification) {
		this.aircraftSpecification = aircraftSpecification;
	}
	
	/**
	 * Gets the environment specification of this setup.
	 * 
	 * @return the environment specification of this setup
	 */
	public Specification<Environment> getEnvironmentSpecification() {
		return this.environmentSpecification;
	}
	
	/**
	 * Sets the environment specification of this setup
	 * 
	 * @param environmentSpecification the environment specification to be set
	 */
	public void setEnvironmentSpecification(Specification<Environment> environmentSpecification) {
		this.environmentSpecification = environmentSpecification;
	}
	
	/**
	 * Gets the planner specification of this setup.
	 * 
	 * @return the planner specification of this setup
	 */
	public Specification<Planner> getPlannerSpecification() {
		return this.plannerSpecification;
	}
	
	/**
	 * Sets the planner specification of this setup
	 * 
	 * @param plannerSpecification the planner specification to be set
	 */
	public void setPlannerSpecification(Specification<Planner> plannerSpecification) {
		this.plannerSpecification = plannerSpecification;
	}
	
	/**
	 * Gets the aircraft connection specification of this setup.
	 * 
	 * @return the aircraft connection specification of this setup
	 */
	public Specification<AircraftConnection> getAircraftConnectionSpecification() {
		return this.aircraftConnectionSpecification;
	}
	
	/**
	 * Sets the aircraft connection specification of this setup
	 * 
	 * @param aircraftConnectionSpecification the aircraft connection specification to be set
	 */
	public void setAircraftConnectionSpecification(Specification<AircraftConnection> aircraftConnectionSpecification) {
		this.aircraftConnectionSpecification = aircraftConnectionSpecification;
	}
	
}
