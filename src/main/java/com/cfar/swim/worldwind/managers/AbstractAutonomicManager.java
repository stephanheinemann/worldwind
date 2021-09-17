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
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.managing.Features;
import com.cfar.swim.worldwind.managing.PlannerTuning;
import com.cfar.swim.worldwind.managing.TrajectoryQuality;
import com.cfar.swim.worldwind.planners.DynamicObstacleListener;
import com.cfar.swim.worldwind.planners.LifelongPlanner;
import com.cfar.swim.worldwind.planners.OnlinePlanner;
import com.cfar.swim.worldwind.planners.PlanRevisionListener;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Properties;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.aircraft.AircraftFactory;
import com.cfar.swim.worldwind.registries.environments.EnvironmentFactory;
import com.cfar.swim.worldwind.registries.environments.EnvironmentProperties;
import com.cfar.swim.worldwind.registries.managers.AbstractManagerProperties;
import com.cfar.swim.worldwind.registries.planners.PlannerFactory;
import com.cfar.swim.worldwind.registries.planners.PlannerProperties;
import com.cfar.swim.worldwind.render.annotations.DepictionAnnotation;
import com.cfar.swim.worldwind.session.ObstacleManager;
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
public abstract class AbstractAutonomicManager implements AutonomicManager, DynamicObstacleListener {
	
	/** the cost policy of this abstract autonomic manager */
	private CostPolicy costPolicy = CostPolicy.AVERAGE;
	
	/** the risk policy of this abstract autonomic manger */
	private RiskPolicy riskPolicy = RiskPolicy.SAFETY;
	
	/** the feature horizon of this abstract autonomic manager */
	private Duration featureHorizon = Features.FEATURE_HORIZON;
	
	/** the source scenario of this abstract autonomic manager */
	private Scenario sourceScenario = null;
	
	/** the obstacle manager of this abstract autonomic manager */
	private ObstacleManager obstacleManager = null;
	
	/** the managed scenarios and their planner tunings of this abstract autonomic manager */
	private Map<Scenario, PlannerTuning> managedScenarios = new HashMap<>();
	
	/** the executor of this abstract autonomic manager */
	private ExecutorService executor = null;
	
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
	 * Gets the source scenario of this autonomic manager.
	 * 
	 * @return the source scenario of this autonomic manager
	 */
	protected Scenario getSourceScenario() {
		return this.sourceScenario;
	}
	
	/**
	 * Sets the source scenario of this autonomic manager.
	 * 
	 * @param sourceScenario the source scenario to be set
	 */
	protected void setSourceScenario(Scenario sourceScenario) {
		this.sourceScenario = sourceScenario;
	}
	
	/**
	 * Gets the managed scenarios of this abstract autonomic manager.
	 * 
	 * @return the managed scenarios of this abstract autonomic manager
	 */
	protected Set<Scenario> getManagedScenarios() {
		return Collections.unmodifiableSet(this.managedScenarios.keySet());
	}
	
	/**
	 * Determines whether or not this abstract autonomic manager has managed
	 * scenarios.
	 * 
	 * @return true if this abstract autonomic manager has managed scenarios,
	 *         false otherwise
	 */
	protected boolean hasManagedScenarios() {
		return (0 < this.getManagedScenarios().size());
	}
	
	/**
	 * Creates a new planner tuning for this abstract autonomic manager based
	 * on a planner specification and features.
	 * 
	 * @param specification the planner specification
	 * @param features the features
	 * 
	 * @return the created planner tuning
	 * 
	 * @see AutonomicManager#createPlannerTuning(Specification, Features)
	 */
	@Override
	public abstract PlannerTuning createPlannerTuning(
			Specification<Planner> specification, Features features);
	
	/**
	 * Gets the planner tuning of a managed scenario of this abstract autonomic
	 * manager.
	 * 
	 * @param managedScenario the managed scenario
	 * 
	 * @return the planner tuning of the managed scenario, null if the scenario
	 *         is not managed by this abstract autonomic manager
	 * 
	 * @see AutonomicManager#getPlannerTuning(Scenario)
	 */
	@Override
	public PlannerTuning getPlannerTuning(Scenario managedScenario) {
		return this.managedScenarios.get(managedScenario);
	}
	
	/**
	 * Initializes this abstract autonomic manager for a managed session.
	 * 
	 * @param managedSession the managed session
	 */
	protected void initialize(Session managedSession) {
		this.setSourceScenario(managedSession.getActiveScenario());
		Set<Specification<Planner>> plannerSpecs =
				managedSession.getManagedPlannerSpecifications();
		Set<Specification<Environment>> envSpecs =
				managedSession.getEnvironmentSpecifications();
		EnvironmentProperties sourceEnvProperties = (EnvironmentProperties)
				managedSession.getSetup().getEnvironmentSpecification().getProperties();
		
		// create managed scenarios
		for (Specification<Planner> plannerSpec : plannerSpecs) {
			for (Specification<Environment> envSpec : envSpecs) {
				// managed scenario
				String managedScenarioId = plannerSpec.getId() + " / " + envSpec.getId();
				Scenario managedScenario = new Scenario(managedScenarioId);
				
				// managed scenario time
				managedScenario.setTime(this.getSourceScenario().getTime());
				// managed scenario cost threshold
				managedScenario.setThreshold(this.getSourceScenario().getThreshold());
				// managed scenario globe
				managedScenario.setGlobe(this.getSourceScenario().getGlobe());
				
				// managed scenario aircraft
				if (sourceScenario.hasAircraft()) {
					AircraftFactory aircraftFactory = new AircraftFactory(
							managedSession.getSetup().getAircraftSpecification());
					managedScenario.setAircraft(aircraftFactory.createInstance());
					managedScenario.getAircraft().setCostInterval(
							this.getSourceScenario().getAircraft().getCostInterval());
				}
				
				// managed scenario sector / environment boundary
				managedScenario.setSector(this.getSourceScenario().getSector());
				// managed scenario POIs
				List<Position> pois = new ArrayList<Position>();
				pois.addAll(this.getSourceScenario().getWaypoints());
				for (Waypoint poi : this.getSourceScenario().getWaypoints()) {
					managedScenario.addWaypoint(poi);
				}
				
				// managed scenario environment
				EnvironmentFactory envFactory = new EnvironmentFactory(managedScenario);
				// apply common source scenario environment boundaries
				((EnvironmentProperties) envSpec.getProperties()).setCeiling(
						sourceEnvProperties.getCeiling());
				((EnvironmentProperties) envSpec.getProperties()).setFloor(
						sourceEnvProperties.getFloor());
				envFactory.setSpecification(envSpec);
				Environment environment = envFactory.createInstance();
				
				if (null != environment) {
					managedScenario.setEnvironment(environment);
					// managed scenario planner
					PlannerFactory plannerFactory = new PlannerFactory(managedScenario);
					((PlannerProperties) plannerSpec.getProperties()).setCostPolicy(this.getCostPolicy());
					((PlannerProperties) plannerSpec.getProperties()).setRiskPolicy(this.getRiskPolicy());
					plannerFactory.setSpecification(plannerSpec);
					Planner planner = plannerFactory.createInstance();
					
					if ((null != planner) && (planner instanceof LifelongPlanner)) {
						managedScenario.setPlanner(planner);
						
						// confirm environment and planner compatibility
						if (planner.supports(managedScenario.getAircraft())
								&& planner.supports(managedScenario.getEnvironment())
								&& planner.supports(pois) && (1 < pois.size())) {
							
							// managed scenario aircraft position
							managedScenario.moveAircraft(pois.get(0));
							// managed scenario obstacles
							managedScenario.submitAddObstacles(this.getSourceScenario().getObstacles());
							//managedScenario.submitAddObstacles(this.getSourceScenario().getEmbeddedObstacles());
							managedScenario.commitObstacleChange();
							
							if (planner instanceof OnlinePlanner) {
								OnlinePlanner onlinePlanner = (OnlinePlanner) planner;
								onlinePlanner.setStandby(true);
								onlinePlanner.setDatalink(this.getSourceScenario().getDatalink());
							}
							
							// TODO: datalink and SWIM
							// TODO: features and knowledge base
							
							planner.addPlanRevisionListener(new PlanRevisionListener() {
								@Override
								public void revisePlan(Trajectory trajectory) {
									styleTrajectory(trajectory);
									managedScenario.setTrajectory(trajectory);
									// TODO: update source scenario with best trajectory
									// TODO: measure performance and update reputation
									Trajectory sourceTrajectory = getSourceScenario().getTrajectory();
									
									List<Waypoint> spois = sourceTrajectory.getPois();
									List<Waypoint> rpois = trajectory.getPois();
									int poiIndex = Math.min(spois.size(), rpois.size()) - 1;
									System.out.println("common POI indices = " + poiIndex);
									
									Trajectory st = sourceTrajectory.getSubTrajectory(poiIndex);
									TrajectoryQuality stq = new TrajectoryQuality(st);
									Trajectory rt = trajectory.getSubTrajectory(poiIndex);
									TrajectoryQuality rtq = new TrajectoryQuality(rt);
									
									// TODO: comparison of trajectories can only be based on
									// same start and end POIs using same normalizer
									
									// TODO: multiple parts / uploading versus showing
									// consider non-empty, improved, or longer revised trajectory
									if (!trajectory.isEmpty() && (sourceTrajectory.isEmpty()
											|| (0 > stq.compareTo(rtq))
											|| ((0 == stq.compareTo(rtq)) && (rpois.size() >= poiIndex)))) {
										getSourceScenario().setTrajectory(trajectory);
									}
									Thread.yield();
								}
							});
							
							// managed scenario features and initial tuning
							// TODO: environment versus scenario obstacles
							Features features = new Features(managedScenario, this.getFeatureHorizon());
							PlannerTuning tuning = this.createPlannerTuning(plannerSpec, features);
							List<Properties<Planner>> candidates = tuning.tune();
							// TODO: default candidate
							tuning.getSpecification().setProperties(candidates.get(0));
							planner.update(tuning.getSpecification());
							
							// add elaborated managed scenario
							this.managedScenarios.put(managedScenario, tuning);
							managedSession.addScenario(managedScenario);
						}
					}
				}
			}
			
			if (this.hasManagedScenarios()) {
				// execute managed planners in parallel
				this.executor = Executors.newWorkStealingPool(this.getManagedScenarios().size());
				// inject all source scenario environment changes into managed scenarios
				this.getSourceScenario().clearTrajectory();
				this.setObstacleManager(this.getSourceScenario());
				this.getSourceScenario().setDynamicObstacleListener(this);
			}
		}
	}
	
	protected void run(Session managedSession) {
		ArrayList<Callable<Trajectory>> managedPlanners = new ArrayList<>();
		
		for (Scenario managedScenario : this.getManagedScenarios()) {
			System.out.println(managedScenario.getId());
			System.out.println(this.getPlannerTuning(managedScenario).getFeatures());
		
			Planner managedPlanner = managedScenario.getPlanner();
			List<Position> pois = new ArrayList<Position>();
			pois.addAll(managedScenario.getWaypoints());
			Position origin = pois.remove(0);
			Position destination = pois.remove(pois.size() - 1);
			
			// tuning procedure
			/*
			List<Properties<Planner>> candidates = getPlannerTuning(managedScenario).tune();
			getPlannerTuning(managedScenario).getSpecification().setProperties(candidates.get(0));
			managedPlanner.update(getPlannerTuning(managedScenario).getSpecification());
			*/
			
			if (pois.isEmpty()) {
				managedPlanners.add(new Callable<Trajectory>() {
					@Override
					public Trajectory call() throws Exception {
						System.out.println("running planner " + managedPlanner.getId());
						return managedPlanner.plan(origin, destination, managedScenario.getTime());
					}});
			} else {
				managedPlanners.add(new Callable<Trajectory>() {
					@Override
					public Trajectory call() throws Exception {
						System.out.println("running planner " + managedPlanner.getId());
						return managedPlanner.plan(origin, destination, pois, managedScenario.getTime());
					}});
			}
		}
		
		try {
			if (!managedPlanners.isEmpty()) {
				this.executor.invokeAll(managedPlanners);
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * Cleans up this abstract autonomic manager and removes all its managed
	 * scenarios.
	 * 
	 * @param managedSession the managed session of this abstract autonomic
	 *                       manager
	 */
	protected void cleanup(Session managedSession) {
		if (this.hasManagedScenarios()) {
			this.getSourceScenario().resetDynamicObstacleListener();
			this.setObstacleManager(null);
			for (Scenario managedScenario : this.managedScenarios.keySet()) {
				managedSession.removeScenario(managedScenario);
			}
			this.managedScenarios.clear();
			this.executor.shutdown();
			this.getSourceScenario().clearTrajectory();
		}
	}
	
	// TODO: managed session versus training session
	
	/**
	 * Manages a session identified by a session identifier.
	 * 
	 * @param sessionId the identifier of the session to be managed
	 * 
	 * @see AutonomicManager#manage(String)
	 */
	@Override
	public void manage(String sessionId) {
		Session session = SessionManager.getInstance().getSession(sessionId);
		this.manage(session);
	}
	
	/**
	 * Manages a specified session.
	 * 
	 * @param session the session to be managed
	 * 
	 * @see AutonomicManager#manage(Session)
	 */
	@Override
	public void manage(Session session) {
		if (null != session) {
			this.initialize(session);
			this.run(session);
			this.cleanup(session);
		}
	}
	
	/**
	 * Terminates the managed session of this abstract autonomic manager.
	 * 
	 * @see AutonomicManager#terminate()
	 */
	@Override
	public synchronized void terminate() {
		for (Scenario managedScenario : this.managedScenarios.keySet()) {
			((LifelongPlanner) managedScenario.getPlanner()).terminate();
		}
	}
	
	/**
	 * Determines whether or not this abstract autonomic manager is listening.
	 * 
	 * @return true if this abstract autonomic manager is listening,
	 *         false otherwise
	 * 
	 * @see DynamicObstacleListener#isListening()
	 */
	@Override
	public synchronized boolean isListening() {
		return true;
	}
	
	/**
	 * Notifies this abstract autonomic manager about a pending obstacle change.
	 * 
	 * @see DynamicObstacleListener#notifyPendingObstacleChange()
	 */
	@Override
	public synchronized void notifyPendingObstacleChange() {
		if (this.hasObstacleManager()
				&& !this.getObstacleManager().commitObstacleChange().isEmpty()) {
			this.getSourceScenario().clearTrajectory();
			
			// update all managed scenarios and planner tunings
			for (Scenario managedScenario : getManagedScenarios()) {
				PlannerTuning tuning = getPlannerTuning(managedScenario);
				// TODO: environment obstacles only
				tuning.getFeatures().extractFeatures(this.getSourceScenario());
				System.out.println(tuning.getFeatures());
				List<Properties<Planner>> candidates = tuning.tune();
				// TODO: default candidate
				tuning.getSpecification().setProperties(candidates.get(0));
				managedScenario.getPlanner().update(tuning.getSpecification());
				managedScenario.submitReplaceObstacles(
						getSourceScenario().getObstacles());
				//managedScenario.submitReplaceObstacles(
				//		getSourceScenario().getEmbeddedObstacles());
			}
		}
	}
	
	/**
	 * Gets the obstacle manager of this abstract autonomic manager.
	 * 
	 * @return the obstacle manager of this abstract autonomic manager
	 * 
	 * @see DynamicObstacleListener#getObstacleManager()
	 */
	@Override
	public synchronized ObstacleManager getObstacleManager() {
		return this.obstacleManager;
	}
	
	/**
	 * Sets the obstacle manager of this abstract autonomic manager.
	 * 
	 * @param obstacleManager the obstacle manager to be set
	 * 
	 * @see DynamicObstacleListener#setObstacleManager(ObstacleManager)
	 */
	@Override
	public synchronized void setObstacleManager(ObstacleManager obstacleManager) {
		this.obstacleManager = obstacleManager;
	}
	
	/**
	 * Determines whether or not this abstract autonomic manager has an
	 * obstacle manager.
	 * 
	 * @return true if this abstract autonomic manager has an obstacle manager,
	 *         false otherwise
	 * 
	 * @see DynamicObstacleListener#hasObstacleManager()
	 */
	@Override
	public synchronized boolean hasObstacleManager() {
		return (null != this.obstacleManager);
	}
	
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
	
	/**
	 * Determines whether or not this abstract autonomic manager matches a
	 * specification.
	 * 
	 * @param specification the specification to be matched
	 * 
	 * @return true if the this abstract autonomic manager matches the
	 *         specification, false otherwise
	 * 
	 * @see FactoryProduct#matches(Specification)
	 */
	@Override
	public boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = false;
		
		if ((null != specification)
				&& specification.getId().equals(this.getId())
				&& (specification.getProperties() instanceof AbstractManagerProperties)) {
			AbstractManagerProperties properties = (AbstractManagerProperties) specification.getProperties();
			matches = this.getCostPolicy().equals(properties.getCostPolicy())
					&& this.getRiskPolicy().equals(properties.getRiskPolicy())
					&& this.getFeatureHorizon().equals(Duration.ofSeconds(
							Math.round(properties.getFeatureHorizon() * 60d)));
		}
	
		return matches;
	}
	
	/**
	 * Updates this abstract autonomic manager according to a specification.
	 * 
	 * @param specification the specification to be used for the update
	 * 
	 * @return true if this autonomic manager has been updated, false otherwise
	 * 
	 * @see FactoryProduct#update(Specification)
	 */
	@Override
	public boolean update(Specification<? extends FactoryProduct> specification) {
		boolean updated = false;
		
		if ((null != specification)
				&& specification.getId().equals(this.getId())
				&& (specification.getProperties() instanceof AbstractManagerProperties)
				&& !this.matches(specification)) {
			AbstractManagerProperties properties = (AbstractManagerProperties) specification.getProperties();
			this.setCostPolicy(properties.getCostPolicy());
			this.setRiskPolicy(properties.getRiskPolicy());
			this.setFeatureHorizon(Duration.ofSeconds(
					Math.round(properties.getFeatureHorizon() * 60d)));
			updated = true;
		}
		
		return updated;
	}
	
}
