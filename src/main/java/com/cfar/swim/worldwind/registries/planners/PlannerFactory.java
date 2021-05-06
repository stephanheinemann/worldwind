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
package com.cfar.swim.worldwind.registries.planners;

import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;

import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.planners.cgs.adstar.ADStarPlanner;
import com.cfar.swim.worldwind.planners.cgs.arastar.ARAStarPlanner;
import com.cfar.swim.worldwind.planners.cgs.astar.ForwardAStarPlanner;
import com.cfar.swim.worldwind.planners.cgs.thetastar.ThetaStarPlanner;
import com.cfar.swim.worldwind.registries.AbstractFactory;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.planners.cgs.ADStarProperties;
import com.cfar.swim.worldwind.registries.planners.cgs.ARAStarProperties;
import com.cfar.swim.worldwind.registries.planners.cgs.ForwardAStarProperties;
import com.cfar.swim.worldwind.registries.planners.cgs.ThetaStarProperties;
import com.cfar.swim.worldwind.session.Scenario;

/**
 * Realizes a planner factory to create planners according to
 * customized planner specifications.
 * 
 * @author Stephan Heinemann
 * 
 * @see AbstractFactory
 * @see Specification
 */
public class PlannerFactory extends AbstractFactory<Planner> {

	// TODO: Both, environment and planner factory need the active scenario
	// maybe pull up scenario functionality into abstract factory
	
	/** the scenario of this planner factory */
	private Scenario scenario;
	
	/** the active scenario change listener of this planner factory */
	private ActiveScenarioChangeListener ascl = new ActiveScenarioChangeListener();
	
	/**
	 * Constructs a new planner factory with a specified scenario. The scenario
	 * aggregates an aircraft and environment which shall not be part of the
	 * planner specification.
	 * 
	 * @param scenario the scenario of this planner factory
	 */
	public PlannerFactory(Scenario scenario) {
		this.scenario = scenario;
	}
	
	/**
	 * Constructs a new planner factory with a specified scenario to create
	 * registered planners according to a customized planner specification.
	 * The scenario aggregates an aircraft and an environment which shall
	 * not be part of the environment specification.
	 * 
	 * @param specification the planner specification describing the
	 *                      registered planner
	 * @param scenario the scenario of this planner factory
	 * 
	 * @see AbstractFactory
	 */
	public PlannerFactory(Specification<Planner> specification, Scenario scenario) {
		super(specification);
		this.scenario = scenario;
	}
	
	/**
	 * Gets the scenario of this planner factory.
	 * 
	 * @return the scenario of this planner factory
	 */
	public Scenario getScenario() {
		return this.scenario;
	}
	
	/**
	 * Sets the scenario of this planner factory.
	 * 
	 * @param scenario the scenario to be set
	 */
	public void setScenario(Scenario scenario) {
		this.scenario = scenario;
	}
	
	/**
	 * Gets the active scenario change listener of this planner factory.
	 * 
	 * @return the active scenario change listener of this planner factory
	 */
	public PropertyChangeListener getActiveScenarioChangeListener() {
		return this.ascl;
	}
	
	/**
	 * Creates a new planner according to the customized planner specification
	 * of this planner factory.
	 * 
	 * @return the created planner
	 * 
	 * @see AbstractFactory#createInstance()
	 */
	@Override
	public Planner createInstance() {
		Planner planner = null;
		
		// TODO: validate scenario for planner creation? (supports)
		if (this.hasSpecification()) {
			if (this.specification.getId().equals(Specification.PLANNER_FAS_ID)) {
				ForwardAStarProperties properties = (ForwardAStarProperties) this.specification.getProperties();
				planner = new ForwardAStarPlanner(this.scenario.getAircraft(), this.scenario.getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
			} else if (this.specification.getId().equals(Specification.PLANNER_TS_ID)) {
				ThetaStarProperties properties = (ThetaStarProperties) this.specification.getProperties();
				planner = new ThetaStarPlanner(this.scenario.getAircraft(), this.scenario.getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
			} else if (this.specification.getId().equals(Specification.PLANNER_ARAS_ID)) {
				ARAStarProperties properties = (ARAStarProperties) this.specification.getProperties();
				planner = new ARAStarPlanner(this.scenario.getAircraft(), this.scenario.getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
				((ARAStarPlanner) planner).setMinimumQuality(properties.getMinimumQuality());
				((ARAStarPlanner) planner).setMaximumQuality(properties.getMaximumQuality());
				((ARAStarPlanner) planner).setQualityImprovement(properties.getQualityImprovement());
			} else if (this.specification.getId().equals(Specification.PLANNER_ADS_ID)) {
				ADStarProperties properties = (ADStarProperties) this.specification.getProperties();
				planner = new ADStarPlanner(this.scenario.getAircraft(), this.scenario.getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
				((ADStarPlanner) planner).setMinimumQuality(properties.getMinimumQuality());
				((ADStarPlanner) planner).setMaximumQuality(properties.getMaximumQuality());
				((ADStarPlanner) planner).setQualityImprovement(properties.getQualityImprovement());
				((ADStarPlanner) planner).setSignificantChange(properties.getSignificantChange());
				((ADStarPlanner) planner).setObstacleManager(this.scenario);
			}
			// TODO: implement more planners
		}
		
		return planner;
	}
	
	/**
	 * Realizes an active scenario change listener for this planner factory.
	 * 
	 * @author Stephan Heinemann
	 */
	private class ActiveScenarioChangeListener implements PropertyChangeListener {
		
		/**
		 * Notifies the planner factory about an active scenario change.
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
