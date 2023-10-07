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

import java.time.Duration;




import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.planners.cgs.adstar.ADStarPlanner;
import com.cfar.swim.worldwind.planners.cgs.arastar.ARAStarPlanner;
import com.cfar.swim.worldwind.planners.cgs.astar.ForwardAStarPlanner;
import com.cfar.swim.worldwind.planners.cgs.oadstar.OADStarPlanner;
import com.cfar.swim.worldwind.planners.cgs.thetastar.ThetaStarPlanner;
import com.cfar.swim.worldwind.planners.managed.ManagedGridPlanner;
import com.cfar.swim.worldwind.planners.managed.ManagedTreePlanner;
import com.cfar.swim.worldwind.planners.rl.dqn.DQNPlanner;
import com.cfar.swim.worldwind.planners.rl.dqn.DQNPlannerNoCosts;
import com.cfar.swim.worldwind.planners.rl.qlearning.QLearningPlanner;
import com.cfar.swim.worldwind.planners.rrt.adrrt.ADRRTreePlanner;
import com.cfar.swim.worldwind.planners.rrt.arrt.ARRTreePlanner;
import com.cfar.swim.worldwind.planners.rrt.brrt.RRTreePlanner;
import com.cfar.swim.worldwind.planners.rrt.drrt.DRRTreePlanner;
import com.cfar.swim.worldwind.planners.rrt.hrrt.HRRTreePlanner;
import com.cfar.swim.worldwind.planners.rrt.oadrrt.OADRRTreePlanner;
import com.cfar.swim.worldwind.planners.rrt.rrtstar.RRTreeStarPlanner;
import com.cfar.swim.worldwind.registries.AbstractFactory;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.planners.cgs.ADStarProperties;
import com.cfar.swim.worldwind.registries.planners.cgs.ARAStarProperties;
import com.cfar.swim.worldwind.registries.planners.cgs.ForwardAStarProperties;
import com.cfar.swim.worldwind.registries.planners.cgs.OADStarProperties;
import com.cfar.swim.worldwind.registries.planners.cgs.ThetaStarProperties;
import com.cfar.swim.worldwind.registries.planners.rrt.ADRRTreeProperties;
import com.cfar.swim.worldwind.registries.planners.rrt.ARRTreeProperties;
import com.cfar.swim.worldwind.registries.planners.rrt.DRRTreeProperties;
import com.cfar.swim.worldwind.registries.planners.rrt.HRRTreeProperties;
import com.cfar.swim.worldwind.registries.planners.rrt.OADRRTreeProperties;
import com.cfar.swim.worldwind.registries.planners.rrt.RRTreeProperties;
import com.cfar.swim.worldwind.registries.planners.rl.QLearningProperties;
import com.cfar.swim.worldwind.registries.planners.rl.DQNProperties;
import com.cfar.swim.worldwind.registries.planners.rl.DQNProperties2;
import com.cfar.swim.worldwind.session.Scenario;
import com.cfar.swim.worldwind.tracks.AircraftTrackError;
import com.cfar.swim.worldwind.tracks.AircraftTrackPointError;

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
	
	/**
	 * Constructs a new planner factory with a specified scenario. The scenario
	 * aggregates an aircraft and environment which shall not be part of the
	 * planner specification.
	 * 
	 * @param scenario the scenario of this planner factory
	 */
	public PlannerFactory(Scenario scenario) {
		this.setScenario(scenario);
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
	public PlannerFactory(
			Specification<Planner> specification, Scenario scenario) {
		super(specification);
		this.setScenario(scenario);
	}
	
	/**
	 * Creates a new planner according to the customized planner specification
	 * of this planner factory.
	 * 
	 * @return the created planner, or null if no planner could be created
	 * 
	 * @see AbstractFactory#createInstance()
	 */
	@Override
	public Planner createInstance() {
		Planner planner = null;
		
		// TODO: validate scenario for planner creation? (supports)
		if (this.hasSpecification()) {
			if (this.getSpecification().getId().equals(Specification.PLANNER_FAS_ID)) {
				ForwardAStarProperties properties = (ForwardAStarProperties) this.getSpecification().getProperties();
				planner = new ForwardAStarPlanner(this.getScenario().getAircraft(), this.getScenario().getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
			} else if (this.getSpecification().getId().equals(Specification.PLANNER_TS_ID)) {
				ThetaStarProperties properties = (ThetaStarProperties) this.getSpecification().getProperties();
				planner = new ThetaStarPlanner(this.getScenario().getAircraft(), this.getScenario().getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
			} else if (this.getSpecification().getId().equals(Specification.PLANNER_ARAS_ID)) {
				ARAStarProperties properties = (ARAStarProperties) this.getSpecification().getProperties();
				planner = new ARAStarPlanner(this.getScenario().getAircraft(), this.getScenario().getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
				((ARAStarPlanner) planner).setMinimumQuality(properties.getMinimumQuality());
				((ARAStarPlanner) planner).setMaximumQuality(properties.getMaximumQuality());
				((ARAStarPlanner) planner).setQualityImprovement(properties.getQualityImprovement());
			} else if (this.getSpecification().getId().equals(Specification.PLANNER_ADS_ID)) {
				ADStarProperties properties = (ADStarProperties) this.getSpecification().getProperties();
				planner = new ADStarPlanner(this.getScenario().getAircraft(), this.getScenario().getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
				((ADStarPlanner) planner).setMinimumQuality(properties.getMinimumQuality());
				((ADStarPlanner) planner).setMaximumQuality(properties.getMaximumQuality());
				((ADStarPlanner) planner).setQualityImprovement(properties.getQualityImprovement());
				((ADStarPlanner) planner).setSignificantChange(properties.getSignificantChange());
				((ADStarPlanner) planner).setObstacleManager(this.getScenario());
			} else if (this.getSpecification().getId().equals(Specification.PLANNER_OADS_ID)) {
				OADStarProperties properties = (OADStarProperties) this.getSpecification().getProperties();
				planner = new OADStarPlanner(this.getScenario().getAircraft(), this.getScenario().getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
				((OADStarPlanner) planner).setMinimumQuality(properties.getMinimumQuality());
				((OADStarPlanner) planner).setMaximumQuality(properties.getMaximumQuality());
				((OADStarPlanner) planner).setQualityImprovement(properties.getQualityImprovement());
				((OADStarPlanner) planner).setSignificantChange(properties.getSignificantChange());
				((OADStarPlanner) planner).setObstacleManager(this.getScenario());
				((OADStarPlanner) planner).setDatalink(this.getScenario().getDatalink());
				((OADStarPlanner) planner).setMinDeliberation(Duration.ofSeconds(properties.getMinDeliberation()));
				((OADStarPlanner) planner).setMaxDeliberation(Duration.ofSeconds(properties.getMaxDeliberation()));
				AircraftTrackError maxTrackError = AircraftTrackError.maxAircraftTrackError();
				maxTrackError.setCrossTrackError(properties.getMaxCrossTrackError());
				maxTrackError.setTimingError(Duration.ofSeconds(properties.getMaxTimingError()));
				((OADStarPlanner) planner).setMaxTrackError(maxTrackError);
				AircraftTrackPointError maxTakeOffError = AircraftTrackPointError.maxAircraftTrackPointError();
				maxTakeOffError.setHorizontalError(properties.getMaxTakeOffHorizontalError());
				maxTakeOffError.setTimingError(Duration.ofSeconds(properties.getMaxTakeOffTimingError()));
				((OADStarPlanner) planner).setMaxTakeOffError(maxTakeOffError);
				AircraftTrackPointError maxLandingError = AircraftTrackPointError.maxAircraftTrackPointError();
				maxLandingError.setHorizontalError(properties.getMaxLandingHorizontalError());
				maxLandingError.setTimingError(Duration.ofSeconds(properties.getMaxLandingTimingError()));
				((OADStarPlanner) planner).setMaxLandingError(maxLandingError);
			} else if (this.getSpecification().getId().equals(Specification.PLANNER_MGP_ID)) {
				OADStarProperties properties = (OADStarProperties) this.getSpecification().getProperties();
				planner = new ManagedGridPlanner(this.getScenario().getAircraft(), this.getScenario().getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
				((ManagedGridPlanner) planner).setMinimumQuality(properties.getMinimumQuality());
				((ManagedGridPlanner) planner).setMaximumQuality(properties.getMaximumQuality());
				((ManagedGridPlanner) planner).setQualityImprovement(properties.getQualityImprovement());
				((ManagedGridPlanner) planner).setSignificantChange(properties.getSignificantChange());
				((ManagedGridPlanner) planner).setObstacleManager(this.getScenario());
				((ManagedGridPlanner) planner).setDatalink(this.getScenario().getDatalink());
				((ManagedGridPlanner) planner).setMinDeliberation(Duration.ofSeconds(properties.getMinDeliberation()));
				((ManagedGridPlanner) planner).setMaxDeliberation(Duration.ofSeconds(properties.getMaxDeliberation()));
				AircraftTrackError maxTrackError = AircraftTrackError.maxAircraftTrackError();
				maxTrackError.setCrossTrackError(properties.getMaxCrossTrackError());
				maxTrackError.setTimingError(Duration.ofSeconds(properties.getMaxTimingError()));
				((ManagedGridPlanner) planner).setMaxTrackError(maxTrackError);
				AircraftTrackPointError maxTakeOffError = AircraftTrackPointError.maxAircraftTrackPointError();
				maxTakeOffError.setHorizontalError(properties.getMaxTakeOffHorizontalError());
				maxTakeOffError.setTimingError(Duration.ofSeconds(properties.getMaxTakeOffTimingError()));
				((ManagedGridPlanner) planner).setMaxTakeOffError(maxTakeOffError);
				AircraftTrackPointError maxLandingError = AircraftTrackPointError.maxAircraftTrackPointError();
				maxLandingError.setHorizontalError(properties.getMaxLandingHorizontalError());
				maxLandingError.setTimingError(Duration.ofSeconds(properties.getMaxLandingTimingError()));
				((ManagedGridPlanner) planner).setMaxLandingError(maxLandingError);
			} else if (this.getSpecification().getId().equals(Specification.PLANNER_RRT_ID)) {
				RRTreeProperties properties = (RRTreeProperties) this.getSpecification().getProperties();
				planner = new RRTreePlanner(this.getScenario().getAircraft(), this.getScenario().getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
				((RRTreePlanner) planner).setSampling(properties.getSampling());
				((RRTreePlanner) planner).setStrategy(properties.getStrategy());
				((RRTreePlanner) planner).setExtension(properties.getExtension());
				((RRTreePlanner) planner).setMaxIterations(properties.getMaxIterations());
				((RRTreePlanner) planner).setEpsilon(properties.getEpsilon());
				((RRTreePlanner) planner).setBias(properties.getBias());
				((RRTreePlanner) planner).setGoalThreshold(properties.getGoalThreshold());
			} else if (this.getSpecification().getId().equals(Specification.PLANNER_RRTS_ID)) {
				RRTreeProperties properties = (RRTreeProperties) this.getSpecification().getProperties();
				planner = new RRTreeStarPlanner(this.getScenario().getAircraft(), this.getScenario().getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
				((RRTreeStarPlanner) planner).setSampling(properties.getSampling());
				((RRTreeStarPlanner) planner).setStrategy(properties.getStrategy());
				((RRTreeStarPlanner) planner).setExtension(properties.getExtension());
				((RRTreeStarPlanner) planner).setMaxIterations(properties.getMaxIterations());
				((RRTreeStarPlanner) planner).setEpsilon(properties.getEpsilon());
				((RRTreeStarPlanner) planner).setBias(properties.getBias());
				((RRTreeStarPlanner) planner).setGoalThreshold(properties.getGoalThreshold());
			} else if (this.getSpecification().getId().equals(Specification.PLANNER_HRRT_ID)) {
				HRRTreeProperties properties = (HRRTreeProperties) this.getSpecification().getProperties();
				planner = new HRRTreePlanner(this.getScenario().getAircraft(), this.getScenario().getEnvironment());
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
			} else if (this.getSpecification().getId().equals(Specification.PLANNER_ARRT_ID)) {
				ARRTreeProperties properties = (ARRTreeProperties) this.getSpecification().getProperties();
				planner = new ARRTreePlanner(this.getScenario().getAircraft(), this.getScenario().getEnvironment());
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
			} else if (this.getSpecification().getId().equals(Specification.PLANNER_DRRT_ID)) {
				DRRTreeProperties properties = (DRRTreeProperties) this.getSpecification().getProperties();
				planner = new DRRTreePlanner(this.getScenario().getAircraft(), this.getScenario().getEnvironment());
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
				((DRRTreePlanner) planner).setObstacleManager(this.getScenario());
			} else if (this.getSpecification().getId().equals(Specification.PLANNER_ADRRT_ID)) {
				ADRRTreeProperties properties = (ADRRTreeProperties) this.getSpecification().getProperties();
				planner = new ADRRTreePlanner(this.getScenario().getAircraft(), this.getScenario().getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
				((ADRRTreePlanner) planner).setSampling(properties.getSampling());
				((ADRRTreePlanner) planner).setStrategy(properties.getStrategy());
				((ADRRTreePlanner) planner).setExtension(properties.getExtension());
				((ADRRTreePlanner) planner).setMaxIterations(properties.getMaxIterations());
				((ADRRTreePlanner) planner).setEpsilon(properties.getEpsilon());
				((ADRRTreePlanner) planner).setBias(properties.getBias());
				((ADRRTreePlanner) planner).setGoalThreshold(properties.getGoalThreshold());
				((ADRRTreePlanner) planner).setNeighborLimit(properties.getNeighborLimit());
				((ADRRTreePlanner) planner).setMinimumQuality(properties.getMinimumQuality());
				((ADRRTreePlanner) planner).setMaximumQuality(properties.getMaximumQuality());
				((ADRRTreePlanner) planner).setQualityImprovement(properties.getQualityImprovement());
				((ADRRTreePlanner) planner).setSignificantChange(properties.getSignificantChange());
				((ADRRTreePlanner) planner).setObstacleManager(this.getScenario());
			} else if (this.getSpecification().getId().equals(Specification.PLANNER_OADRRT_ID)) {
				OADRRTreeProperties properties = (OADRRTreeProperties) this.getSpecification().getProperties();
				planner = new OADRRTreePlanner(this.getScenario().getAircraft(), this.getScenario().getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
				((OADRRTreePlanner) planner).setSampling(properties.getSampling());
				((OADRRTreePlanner) planner).setStrategy(properties.getStrategy());
				((OADRRTreePlanner) planner).setExtension(properties.getExtension());
				((OADRRTreePlanner) planner).setMaxIterations(properties.getMaxIterations());
				((OADRRTreePlanner) planner).setEpsilon(properties.getEpsilon());
				((OADRRTreePlanner) planner).setBias(properties.getBias());
				((OADRRTreePlanner) planner).setGoalThreshold(properties.getGoalThreshold());
				((OADRRTreePlanner) planner).setNeighborLimit(properties.getNeighborLimit());
				((OADRRTreePlanner) planner).setMinimumQuality(properties.getMinimumQuality());
				((OADRRTreePlanner) planner).setMaximumQuality(properties.getMaximumQuality());
				((OADRRTreePlanner) planner).setQualityImprovement(properties.getQualityImprovement());
				((OADRRTreePlanner) planner).setSignificantChange(properties.getSignificantChange());
				((OADRRTreePlanner) planner).setObstacleManager(this.getScenario());
				((OADRRTreePlanner) planner).setDatalink(this.getScenario().getDatalink());
				((OADRRTreePlanner) planner).setMinDeliberation(Duration.ofSeconds(properties.getMinDeliberation()));
				((OADRRTreePlanner) planner).setMaxDeliberation(Duration.ofSeconds(properties.getMaxDeliberation()));
				AircraftTrackError maxTrackError = AircraftTrackError.maxAircraftTrackError();
				maxTrackError.setCrossTrackError(properties.getMaxCrossTrackError());
				maxTrackError.setTimingError(Duration.ofSeconds(properties.getMaxTimingError()));
				((OADRRTreePlanner) planner).setMaxTrackError(maxTrackError);
				AircraftTrackPointError maxTakeOffError = AircraftTrackPointError.maxAircraftTrackPointError();
				maxTakeOffError.setHorizontalError(properties.getMaxTakeOffHorizontalError());
				maxTakeOffError.setTimingError(Duration.ofSeconds(properties.getMaxTakeOffTimingError()));
				((OADRRTreePlanner) planner).setMaxTakeOffError(maxTakeOffError);
				AircraftTrackPointError maxLandingError = AircraftTrackPointError.maxAircraftTrackPointError();
				maxLandingError.setHorizontalError(properties.getMaxLandingHorizontalError());
				maxLandingError.setTimingError(Duration.ofSeconds(properties.getMaxLandingTimingError()));
				((OADRRTreePlanner) planner).setMaxLandingError(maxLandingError);
			} else if (this.getSpecification().getId().equals(Specification.PLANNER_QLP_ID)) {
				QLearningProperties properties = (QLearningProperties) this.getSpecification().getProperties();
				planner = new QLearningPlanner(this.getScenario().getAircraft(), this.getScenario().getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
			} else if (this.getSpecification().getId().equals(Specification.PLANNER_DQN_ID)) {
				DQNProperties properties = (DQNProperties) this.getSpecification().getProperties();
				planner = new DQNPlanner(this.getScenario().getAircraft(), this.getScenario().getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
			} else if (this.getSpecification().getId().equals(Specification.PLANNER_DQN_NOCOSTS_ID)) {
				DQNProperties2 properties = (DQNProperties2) this.getSpecification().getProperties();
				planner = new DQNPlannerNoCosts(this.getScenario().getAircraft(), this.getScenario().getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
			} else if (this.getSpecification().getId().equals(Specification.PLANNER_MTP_ID)) {
				OADRRTreeProperties properties = (OADRRTreeProperties) this.getSpecification().getProperties();
				planner = new ManagedTreePlanner(this.getScenario().getAircraft(), this.getScenario().getEnvironment());
				planner.setCostPolicy(properties.getCostPolicy());
				planner.setRiskPolicy(properties.getRiskPolicy());
				((ManagedTreePlanner) planner).setSampling(properties.getSampling());
				((ManagedTreePlanner) planner).setStrategy(properties.getStrategy());
				((ManagedTreePlanner) planner).setExtension(properties.getExtension());
				((ManagedTreePlanner) planner).setMaxIterations(properties.getMaxIterations());
				((ManagedTreePlanner) planner).setEpsilon(properties.getEpsilon());
				((ManagedTreePlanner) planner).setBias(properties.getBias());
				((ManagedTreePlanner) planner).setGoalThreshold(properties.getGoalThreshold());
				((ManagedTreePlanner) planner).setNeighborLimit(properties.getNeighborLimit());
				((ManagedTreePlanner) planner).setMinimumQuality(properties.getMinimumQuality());
				((ManagedTreePlanner) planner).setMaximumQuality(properties.getMaximumQuality());
				((ManagedTreePlanner) planner).setQualityImprovement(properties.getQualityImprovement());
				((ManagedTreePlanner) planner).setSignificantChange(properties.getSignificantChange());
				((ManagedTreePlanner) planner).setObstacleManager(this.getScenario());
				((ManagedTreePlanner) planner).setDatalink(this.getScenario().getDatalink());
				((ManagedTreePlanner) planner).setMinDeliberation(Duration.ofSeconds(properties.getMinDeliberation()));
				((ManagedTreePlanner) planner).setMaxDeliberation(Duration.ofSeconds(properties.getMaxDeliberation()));
				AircraftTrackError maxTrackError = AircraftTrackError.maxAircraftTrackError();
				maxTrackError.setCrossTrackError(properties.getMaxCrossTrackError());
				maxTrackError.setTimingError(Duration.ofSeconds(properties.getMaxTimingError()));
				((ManagedTreePlanner) planner).setMaxTrackError(maxTrackError);
				AircraftTrackPointError maxTakeOffError = AircraftTrackPointError.maxAircraftTrackPointError();
				maxTakeOffError.setHorizontalError(properties.getMaxTakeOffHorizontalError());
				maxTakeOffError.setTimingError(Duration.ofSeconds(properties.getMaxTakeOffTimingError()));
				((ManagedTreePlanner) planner).setMaxTakeOffError(maxTakeOffError);
				AircraftTrackPointError maxLandingError = AircraftTrackPointError.maxAircraftTrackPointError();
				maxLandingError.setHorizontalError(properties.getMaxLandingHorizontalError());
				maxLandingError.setTimingError(Duration.ofSeconds(properties.getMaxLandingTimingError()));
				((ManagedTreePlanner) planner).setMaxLandingError(maxLandingError);
			}
			// TODO: implement more planners
		}
		
		return planner;
	}
	
}
