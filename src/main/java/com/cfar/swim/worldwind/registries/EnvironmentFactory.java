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
package com.cfar.swim.worldwind.registries;

import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.geom.Cube;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.PlanningContinuum;
import com.cfar.swim.worldwind.planning.PlanningGrid;
import com.cfar.swim.worldwind.planning.PlanningRoadmap;
import com.cfar.swim.worldwind.session.Scenario;

import gov.nasa.worldwind.geom.Sector;

/**
 * Realizes an environment factory to create environments according to
 * customized environment specifications.
 * 
 * @author Stephan Heinemann
 * 
 * @see Factory
 * @see Specification
 */
public class EnvironmentFactory implements Factory<Environment> {
	
	/** the scenario of this environment factory */
	private Scenario scenario;
	
	/**
	 * Constructs a new environment factory with a specified scenario.
	 * The scenario aggregates a globe and sector which shall not be
	 * part of the environment specification.
	 * 
	 * @param scenario the scenario of this environment factory
	 */
	public EnvironmentFactory(Scenario scenario) {
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
	 * Creates a new environment according to a customized environment specification.
	 * 
	 * @param specification the customized environment specification
	 * 
	 * @return the created environment
	 * 
	 * @see Factory#createInstance(Specification)
	 */
	@Override
	public Environment createInstance(Specification<Environment> specification) {
		Environment environment = null;
		
		if (specification.getId().equals(Specification.PLANNING_GRID_ID)) {
			PlanningGridProperties properties = (PlanningGridProperties) specification.getProperties();
			gov.nasa.worldwind.geom.Box bb = Sector.computeBoundingBox(this.scenario.getGlobe(), 1d, this.scenario.getSector(), properties.getFloor(), properties.getCeiling());
            Box envBox = new Box(bb);
            double side = envBox.getRLength() / properties.getDivsion();
            Cube envCube = new Cube(envBox.getOrigin(), envBox.getUnitAxes(), side);
            int sCells = (int) Math.ceil(envBox.getSLength() / side);
            int tCells = (int) Math.ceil(envBox.getTLength() / side);
            environment = new PlanningGrid(envCube, properties.getDivsion(), sCells, tCells);
            environment.setThreshold(0d);
            environment.setGlobe(this.scenario.getGlobe());
		} else if (specification.getId().equals(Specification.PLANNING_ROADMAP_ID)) {
			// TODO: implement
			environment = new PlanningRoadmap();
		} else if (specification.getId().equals(Specification.PLANNING_CONTINUUM_ID)) {
			// TODO: implement
			environment = new PlanningContinuum();
		}
		
		return environment;
	}

}
