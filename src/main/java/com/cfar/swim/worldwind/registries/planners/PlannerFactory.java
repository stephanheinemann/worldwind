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
import com.cfar.swim.worldwind.planners.rrt.arrt.ARRTreePlanner;
import com.cfar.swim.worldwind.planners.rrt.brrt.RRTreePlanner;
import com.cfar.swim.worldwind.planners.rrt.drrt.DRRTreePlanner;
import com.cfar.swim.worldwind.planners.rrt.hrrt.HRRTreePlanner;
import com.cfar.swim.worldwind.planners.rrt.rrtstar.RRTreeStarPlanner;
import com.cfar.swim.worldwind.registries.AbstractFactory;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.planners.cgs.ADStarProperties;
import com.cfar.swim.worldwind.registries.planners.cgs.ARAStarProperties;
import com.cfar.swim.worldwind.registries.planners.cgs.ForwardAStarProperties;
import com.cfar.swim.worldwind.registries.planners.cgs.ThetaStarProperties;
import com.cfar.swim.worldwind.registries.planners.rrt.ARRTreeProperties;
import com.cfar.swim.worldwind.registries.planners.rrt.DRRTreeProperties;
import com.cfar.swim.worldwind.registries.planners.rrt.HRRTreeProperties;
import com.cfar.swim.worldwind.registries.planners.rrt.RRTreeProperties;
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
			} else if (this.specification.getId().equals(Specification.PLANNER_RRT_ID)) {
				RRTreeProperties properties = (RRTreeProperties) this.specification.getProperties();
				planner = new RRTreePlanner(this.scenario.getAircraft(), this.scenario.getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
				((RRTreePlanner) planner).setSampling(properties.getSampling());
				((RRTreePlanner) planner).setStrategy(properties.getStrategy());
				((RRTreePlanner) planner).setExtension(properties.getExtension());
				((RRTreePlanner) planner).setMaxIterations(properties.getMaxIterations());
				((RRTreePlanner) planner).setEpsilon(properties.getEpsilon());
				((RRTreePlanner) planner).setBias(properties.getBias());
				((RRTreePlanner) planner).setGoalThreshold(properties.getGoalThreshold());
			} else if (this.specification.getId().equals(Specification.PLANNER_RRTS_ID)) {
				RRTreeProperties properties = (RRTreeProperties) this.specification.getProperties();
				planner = new RRTreeStarPlanner(this.scenario.getAircraft(), this.scenario.getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
				((RRTreeStarPlanner) planner).setSampling(properties.getSampling());
				((RRTreeStarPlanner) planner).setStrategy(properties.getStrategy());
				((RRTreeStarPlanner) planner).setExtension(properties.getExtension());
				((RRTreeStarPlanner) planner).setMaxIterations(properties.getMaxIterations());
				((RRTreeStarPlanner) planner).setEpsilon(properties.getEpsilon());
				((RRTreeStarPlanner) planner).setBias(properties.getBias());
				((RRTreeStarPlanner) planner).setGoalThreshold(properties.getGoalThreshold());
			} else if (this.specification.getId().equals(Specification.PLANNER_HRRT_ID)) {
				HRRTreeProperties properties = (HRRTreeProperties) this.specification.getProperties();
				planner = new HRRTreePlanner(this.scenario.getAircraft(), this.scenario.getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
				((HRRTreePlanner) planner).setSampling(properties.getSampling());
				((HRRTreePlanner) planner).setStrategy(properties.getStrategy());
				((HRRTreePlanner) planner).setExtension(properties.getExtension());
				((HRRTreePlanner) planner).setMaxIterations(properties.getMaxIterations());
				((HRRTreePlanner) planner).setEpsilon(properties.getEpsilon());
				((HRRTreePlanner) planner).setBias(properties.getBias());
				((HRRTreePlanner) planner).setGoalThreshold(properties.getGoalThreshold());
				((HRRTreePlanner) planner).setAlgorithm(properties.getAlgorithm());
				((HRRTreePlanner) planner).setVariant(properties.getVariant());
				((HRRTreePlanner) planner).setNeighborLimit(properties.getNeighborLimit());
				((HRRTreePlanner) planner).setQualityBound(properties.getQualityBound());
			} else if (this.specification.getId().equals(Specification.PLANNER_ARRT_ID)) {
				ARRTreeProperties properties = (ARRTreeProperties) this.specification.getProperties();
				planner = new ARRTreePlanner(this.scenario.getAircraft(), this.scenario.getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
				((ARRTreePlanner) planner).setSampling(properties.getSampling());
				((ARRTreePlanner) planner).setStrategy(properties.getStrategy());
				((ARRTreePlanner) planner).setExtension(properties.getExtension());
				((ARRTreePlanner) planner).setMaxIterations(properties.getMaxIterations());
				((ARRTreePlanner) planner).setEpsilon(properties.getEpsilon());
				((ARRTreePlanner) planner).setBias(properties.getBias());
				((ARRTreePlanner) planner).setGoalThreshold(properties.getGoalThreshold());
				((ARRTreePlanner) planner).setNeighborLimit(properties.getNeighborLimit());
				((ARRTreePlanner) planner).setMinimumQuality(properties.getMinimumQuality());
				((ARRTreePlanner) planner).setMaximumQuality(properties.getMaximumQuality());
				((ARRTreePlanner) planner).setQualityImprovement(properties.getQualityImprovement());
			} else if (this.specification.getId().equals(Specification.PLANNER_DRRT_ID)) {
				DRRTreeProperties properties = (DRRTreeProperties) this.specification.getProperties();
				planner = new DRRTreePlanner(this.scenario.getAircraft(), this.scenario.getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
				((DRRTreePlanner) planner).setSampling(properties.getSampling());
				((DRRTreePlanner) planner).setStrategy(properties.getStrategy());
				((DRRTreePlanner) planner).setExtension(properties.getExtension());
				((DRRTreePlanner) planner).setMaxIterations(properties.getMaxIterations());
				((DRRTreePlanner) planner).setEpsilon(properties.getEpsilon());
				((DRRTreePlanner) planner).setBias(properties.getBias());
				((DRRTreePlanner) planner).setGoalThreshold(properties.getGoalThreshold());
				((DRRTreePlanner) planner).setAlgorithm(properties.getAlgorithm());
				((DRRTreePlanner) planner).setVariant(properties.getVariant());
				((DRRTreePlanner) planner).setNeighborLimit(properties.getNeighborLimit());
				((DRRTreePlanner) planner).setQualityBound(properties.getQualityBound());
				((DRRTreePlanner) planner).setSignificantChange(properties.getSignificantChange());
				((DRRTreePlanner) planner).setObstacleManager(this.scenario);
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
