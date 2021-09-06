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

import java.time.Duration;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.managing.Features;
import com.cfar.swim.worldwind.planners.PlanRevisionListener;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.aircraft.AircraftFactory;
import com.cfar.swim.worldwind.registries.environments.EnvironmentFactory;
import com.cfar.swim.worldwind.registries.planners.PlannerFactory;
import com.cfar.swim.worldwind.registries.planners.PlannerProperties;
import com.cfar.swim.worldwind.render.annotations.DepictionAnnotation;
import com.cfar.swim.worldwind.session.Scenario;
import com.cfar.swim.worldwind.session.Session;
import com.cfar.swim.worldwind.session.SessionManager;
import com.cfar.swim.worldwind.util.Depiction;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.BasicShapeAttributes;
import gov.nasa.worldwind.render.Material;
import gov.nasa.worldwind.symbology.milstd2525.MilStd2525GraphicFactory;

/**
 * Abstracts an autonomic manager for motion planners operating in autonomic
 * contexts aggregated by scenarios of a planner session.
 * 
 * @author Stephan Heinemann
 *
 * @see Session
 * @see Scenario
 */
public abstract class AbstractAutonomicManager implements AutonomicManager {
	
	/** the cost policy of this abstract autonomic manager */
	CostPolicy costPolicy = CostPolicy.AVERAGE;
	
	/** the risk policy of this abstract autonomic manger */
	RiskPolicy riskPolicy = RiskPolicy.SAFETY;
	
	/** the feature horizon of this abstract autonomic manager */
	private Duration featureHorizon = Features.FEATURE_HORIZON;
	
	/** the managed scenarios and their features of this abstract autonomic manager */
	private Map<Scenario, Features> managedScenarios = new HashMap<>();
	
	/**
	 * Gets the cost policy of this abstract autonomic manager.
	 * 
	 * @return the cost policy of this abstract autonomic manger
	 * 
	 * @see AutonomicManager#getCostPolicy()
	 */
	@Override
	public CostPolicy getCostPolicy() {
		return this.costPolicy;
	}
	
	/**
	 * Sets the cost policy of this abstract autonomic manager.
	 * 
	 * @param costPolicy the cost policy to be set
	 * 
	 * @see AutonomicManager#setCostPolicy(CostPolicy)
	 */
	@Override
	public void setCostPolicy(CostPolicy costPolicy) {
		this.costPolicy = costPolicy;
	}
	
	/**
	 * Gets the risk policy of this abstract autonomic manager.
	 * 
	 * @return the risk policy of this abstract autonomic manager
	 * 
	 * @see AutonomicManager#getRiskPolicy()
	 */
	@Override
	public RiskPolicy getRiskPolicy() {
		return this.riskPolicy;
	}
	
	/**
	 * Sets the risk policy of this abstract autonomic manager.
	 * 
	 * @param riskPolicy the risk policy to be set
	 * 
	 * @see AutonomicManager#setRiskPolicy(RiskPolicy)
	 */
	@Override
	public void setRiskPolicy(RiskPolicy riskPolicy) {
		this.riskPolicy = riskPolicy;
	}
	
	/**
	 * Gets the feature horizon of this abstract autonomic manager.
	 * 
	 * @return the feature horizon of this abstract autonomic manager
	 * 
	 * @see AutonomicManager#getFeatureHorizon()
	 */
	@Override
	public Duration getFeatureHorizon() {
		return this.featureHorizon;
	}
	
	/**
	 * Sets the feature horizon of this abstract autonomic manager.
	 * 
	 * @param featureHorizon the feature horizon to be set
	 * 
	 * @see AutonomicManager#setFeatureHorizon(Duration)
	 */
	@Override
	public void setFeatureHorizon(Duration featureHorizon) {
		this.featureHorizon = featureHorizon;
	}
	
	/**
	 * Gets the managed scenarios of this abstract autonomic manager.
	 * 
	 * @return the managed scenarios of this abstract autonomic manager
	 * 
	 * @see AutonomicManager#getManagedScenarios()
	 */
	@Override
	public Set<Scenario> getManagedScenarios() {
		return Collections.unmodifiableSet(this.managedScenarios.keySet());
	}
	
	/**
	 * Gets the features of a scenario managed by this abstract autonomic
	 * manager.
	 * 
	 * @param managedScenario the managed scenario
	 * 
	 * @return the features of the managed scenario, null if the scenario is
	 *         not managed by this abstract autonomic manager
	 * 
	 * @see AutonomicManager#getFeatures(Scenario)
	 */
	@Override
	public Features getFeatures(Scenario managedScenario) {
		return managedScenarios.get(managedScenario);
	}
	
	/**
	 * Initializes this abstract autonomic manager for a managed session.
	 * 
	 * @param managedSession the managed session
	 */
	protected void initialize(Session managedSession) {
		Scenario sourceScenario = managedSession.getActiveScenario();
		Set<Specification<Planner>> plannerSpecs =
				managedSession.getManagedPlannerSpecifications();
		Set<Specification<Environment>> envSpecs =
				managedSession.getEnvironmentSpecifications();
		
		for (Specification<Planner> plannerSpec : plannerSpecs) {
			for (Specification<Environment> envSpec : envSpecs) {
				String managedScenarioId = plannerSpec.getId() + " / " + envSpec.getId();
				Scenario managedScenario = new Scenario(managedScenarioId);
				
				managedScenario.setTime(sourceScenario.getTime());
				
				if (sourceScenario.hasAircraft()) {
					AircraftFactory aircraftFactory =
							new AircraftFactory(managedSession.getSetup().getAircraftSpecification());
					managedScenario.setAircraft(aircraftFactory.createInstance());
					managedScenario.getAircraft().setCostInterval(sourceScenario.getAircraft().getCostInterval());
				} // TODO: else
				
				managedScenario.setSector(sourceScenario.getSector());
				List<Position> pois = new ArrayList<Position>();
				pois.addAll(sourceScenario.getWaypoints());
				for (Waypoint poi : sourceScenario.getWaypoints()) {
					managedScenario.addWaypoint(poi);
				}
				
				EnvironmentFactory envFactory = new EnvironmentFactory(managedScenario);
				envFactory.setSpecification(envSpec);
				Environment environment = envFactory.createInstance();
				
				if (null != environment) {
					managedScenario.setEnvironment(environment);
					
					PlannerFactory plannerFactory = new PlannerFactory(managedScenario);
					((PlannerProperties) plannerSpec.getProperties()).setCostPolicy(this.costPolicy);
					((PlannerProperties) plannerSpec.getProperties()).setRiskPolicy(this.riskPolicy);
					plannerFactory.setSpecification(plannerSpec);
					Planner planner = plannerFactory.createInstance();
					
					if (null != planner) {
						managedScenario.setPlanner(planner);
						
						if (planner.supports(managedScenario.getAircraft())
								&& planner.supports(managedScenario.getEnvironment())
								&& planner.supports(pois) && (1 < pois.size())) {
							
							managedScenario.moveAircraft(pois.get(0));
							managedScenario.submitAddObstacles(sourceScenario.getObstacles());
							managedScenario.commitObstacleChange();
							// TODO: datalink and SWIM
							// TODO: features and knowledge base
							
							planner.addPlanRevisionListener(new PlanRevisionListener() {
								@Override
								public void revisePlan(Trajectory trajectory) {
									styleTrajectory(trajectory);
									managedScenario.setTrajectory(trajectory);
									Thread.yield();
								}
							});
							
							Features features = new Features(managedScenario, this.featureHorizon);
							this.managedScenarios.put(managedScenario, features);							
							managedSession.addScenario(managedScenario);
						}
					}
				}
			}
		}
	}
	
	protected void cleanup(Session managedSession) {
		for (Scenario managedScenario : this.managedScenarios.keySet()) {
			managedSession.removeScenario(managedScenario);
		}
	}
	
	@Override
	public void manage(String sessionId) {
		Session session = SessionManager.getInstance().getSession(sessionId);
		this.manage(session);
	}
	
	
	public void manage(Session session) {
		if (null != session) {
			this.initialize(session);
			this.run(session);
			//this.cleanup(session);
		}
	}
	
	public abstract void run(Session managedSession);
	
	// TODO: managed session versus training session
	
	@Override
	public void abandon() {}
	
	@Override
	public void terminate() {}
	
	
	private final MilStd2525GraphicFactory symbolFactory = new MilStd2525GraphicFactory();
	
	private void styleTrajectory(Trajectory trajectory) {
		trajectory.setVisible(true);
		trajectory.setShowPositions(true);
		trajectory.setDrawVerticals(true);
		trajectory.setAttributes(new BasicShapeAttributes());
		trajectory.getAttributes().setOutlineMaterial(Material.MAGENTA);
		trajectory.getAttributes().setOutlineWidth(5d);
		trajectory.getAttributes().setOutlineOpacity(0.5d);
		for (Waypoint waypoint : trajectory.getWaypoints()) {
			Depiction depiction = new Depiction(symbolFactory.createPoint(Waypoint.SICD_NAV_WAYPOINT_ROUTE, waypoint, null));
			depiction.setAnnotation(new DepictionAnnotation(waypoint.getEto().toString(), waypoint));
			depiction.setVisible(true);
			waypoint.setDepiction(depiction);
		}
	}
	

	@Override
	public boolean matches(Specification<? extends FactoryProduct> specification) {
		// TODO Auto-generated method stub
		return false;
	}
	
}
