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
package com.cfar.swim.worldwind.registries.environments;

import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;

import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.environments.PlanningContinuum;
import com.cfar.swim.worldwind.environments.PlanningGrid;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.geom.Cube;
import com.cfar.swim.worldwind.registries.AbstractFactory;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.session.Scenario;

import gov.nasa.worldwind.geom.Sector;

/**
 * Realizes an environment factory to create environments according to
 * customized environment specifications.
 * 
 * @author Stephan Heinemann
 * 
 * @see AbstractFactory
 * @see Specification
 */
public class EnvironmentFactory extends AbstractFactory<Environment> {
	
	// TODO: Both, environment and planner factory need the active scenario
	// maybe pull up scenario functionality into abstract factory
	
	/** the scenario of this environment factory */
	private Scenario scenario;
	
	/** the active scenario change listener of this environment factory */
	private ActiveScenarioChangeListener ascl = new ActiveScenarioChangeListener();
	
	/**
	 * Constructs a new environment factory with a specified scenario. The
	 * scenario aggregates a globe and sector which shall not be part of the
	 * environment specification.
	 * 
	 * @param scenario the scenario of this environment factory
	 */
	public EnvironmentFactory(Scenario scenario) {
		this.scenario = scenario;
	}
	
	/**
	 * Constructs a new environment factory with a specified scenario to create
	 * registered environments according to a customized environment
	 * specification. The scenario aggregates a globe and sector which shall
	 * not be part of the environment specification.
	 * 
	 * @param specification the environment specification describing the
	 *                      registered environment
	 * @param scenario the scenario of this environment factory
	 * 
	 * @see AbstractFactory
	 */
	public EnvironmentFactory(Specification<Environment> specification, Scenario scenario) {
		super(specification);
		this.scenario = scenario;
	}
	
	/**
	 * Gets the scenario of this environment factory.
	 * 
	 * @return the scenario of this environment factory
	 */
	public Scenario getScenario() {
		return this.scenario;
	}
	
	/**
	 * Sets the scenario of this environment factory.
	 * 
	 * @param scenario the scenario to be set
	 */
	public void setScenario(Scenario scenario) {
		this.scenario = scenario;
	}
	
	/**
	 * Gets the active scenario change listener of this environment factory.
	 * 
	 * @return the active scenario change listener of this environment factory
	 */
	public PropertyChangeListener getActiveScenarioChangeListener() {
		return this.ascl;
	}
	
	/**
	 * Creates a new environment according to the customized environment
	 * specification of this environment factory.
	 * 
	 * @return the created environment
	 * 
	 * @see AbstractFactory#createInstance()
	 */
	@Override
	public Environment createInstance() {
		Environment environment = null;
		
		if (this.hasSpecification()) {
			if (this.specification.getId().equals(Specification.ENVIRONMENT_PLANNING_GRID_ID)) {
				PlanningGridProperties properties = (PlanningGridProperties) this.specification.getProperties();
				gov.nasa.worldwind.geom.Box bb = Sector.computeBoundingBox(this.scenario.getGlobe(), 1d, this.scenario.getSector(), properties.getFloor(), properties.getCeiling());
	            Box envBox = new Box(bb);
	            double side = envBox.getRLength() / properties.getDivision();
	            Cube envCube = new Cube(envBox.getOrigin(), envBox.getUnitAxes(), side);
	            int sCells = (int) Math.ceil(envBox.getSLength() / side);
	            int tCells = (int) Math.ceil(envBox.getTLength() / side);
	            environment = new PlanningGrid(envCube, properties.getDivision(), sCells, tCells);
	            environment.setThreshold(0d);
	            environment.setGlobe(this.scenario.getGlobe());
	            ((PlanningGrid) environment).addStructuralChangeListener(this.scenario);
			} else if (this.specification.getId().equals(Specification.ENVIRONMENT_PLANNING_CONTINUUM_ID)) {
				PlanningContinuumProperties properties = (PlanningContinuumProperties) this.specification.getProperties();
				gov.nasa.worldwind.geom.Box bb = Sector.computeBoundingBox(this.scenario.getGlobe(), 1d, this.scenario.getSector(), properties.getFloor(), properties.getCeiling());
				Box envBox = new Box(bb);
				environment = new PlanningContinuum(envBox);
				environment.setThreshold(0d);
				environment.setGlobe(this.scenario.getGlobe());
				((PlanningContinuum) environment).setResolution(properties.getResolution());
				((PlanningContinuum) environment).addStructuralChangeListener(this.scenario); 
			} else if (this.specification.getId().equals(Specification.ENVIRONMENT_PLANNING_ROADMAP_ID)) {
				// TODO: implement
			}
		}
		
		return environment;
	}

	/**
	 * Realizes an active scenario change listener for this environment factory.
	 * 
	 * @author Stephan Heinemann
	 */
	private class ActiveScenarioChangeListener implements PropertyChangeListener {
		
		/**
		 * Notifies the environment factory about an active scenario change.
		 * 
		 * @param evt the property change event
		 */
		@Override
		public void propertyChange(PropertyChangeEvent evt) {
			if (evt.getNewValue() instanceof Scenario) {
				scenario = (Scenario) evt.getNewValue();
			}
		}
	}
	
}
