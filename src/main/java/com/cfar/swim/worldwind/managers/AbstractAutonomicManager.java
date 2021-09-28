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
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import com.cfar.swim.worldwind.connections.Communication;
import com.cfar.swim.worldwind.connections.Datalink;
import com.cfar.swim.worldwind.connections.DatalinkCommunicator;
import com.cfar.swim.worldwind.connections.DatalinkTracker;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.managing.Features;
import com.cfar.swim.worldwind.managing.PlannerTuning;
import com.cfar.swim.worldwind.managing.TrajectoryQuality;
import com.cfar.swim.worldwind.planners.DynamicObstacleListener;
import com.cfar.swim.worldwind.planners.PlanRevisionListener;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.planners.managed.ManagedGoals;
import com.cfar.swim.worldwind.planners.managed.ManagedPlanner;
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
import com.cfar.swim.worldwind.tracks.AircraftTrackError;
import com.cfar.swim.worldwind.tracks.AircraftTrackPointError;
import com.cfar.swim.worldwind.util.Depiction;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.BasicShapeAttributes;
import gov.nasa.worldwind.render.Material;
import gov.nasa.worldwind.symbology.milstd2525.MilStd2525GraphicFactory;
import gov.nasa.worldwind.util.Logging;

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
	private CostPolicy costPolicy = CostPolicy.AVERAGE;
	
	/** the risk policy of this abstract autonomic manger */
	private RiskPolicy riskPolicy = RiskPolicy.SAFETY;
	
	/** the feature horizon of this abstract autonomic manager */
	private Duration featureHorizon = Features.FEATURE_HORIZON;
	
	/** the source scenario of this abstract autonomic manager */
	private Scenario sourceScenario = null;
	
	/** the managed scenarios and their planner tunings of this abstract autonomic manager */
	private Map<Scenario, PlannerTuning> managedScenarios = new HashMap<>();
	
	/** the active planner of this abstract autonomic manager */
	private ManagedPlanner activePlanner = null;
	
	/** the obstacle manager of this abstract autonomic manager */
	private ObstacleManager obstacleManager = null;
	
	/** the datalink take-off communication of this abstract autonomic manager */
	private Communication<Datalink> takeOff = null;
	
	/** the datalink landing communication of this abstract autonomic manager */
	private Communication<Datalink> landing = null;
	
	/** the datalink unplanned landing communication of this abstract autonomic manager */
	private Communication<Datalink> unplannedLanding = null;
	
	/** the establish datalink communication of this abstract autonomic manager */
	private Communication<Datalink> establishDatalink = null;
	
	/** the maximum acceptable aircraft take-off error of this abstract autonomic manager */
	private AircraftTrackPointError maxTakeOffError = AircraftTrackPointError.ZERO;
	
	/** the maximum acceptable aircraft landing error of this abstract autonomic manager */
	private AircraftTrackPointError maxLandingError = AircraftTrackPointError.ZERO;
	
	/** the maximum acceptable aircraft track error of this abstract autonomic manager */
	private AircraftTrackError maxTrackError = AircraftTrackError.ZERO;
	
	/** the tuner executor of this abstract autonomic manager */
	private final ExecutorService tunerExecutor = Executors.newSingleThreadExecutor();
	
	/** the planner executor of this abstract autonomic manager */
	private ExecutorService plannerExecutor = null;
	
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
	 * Gets the source scenario of this abstract autonomic manager.
	 * 
	 * @return the source scenario of this abstract autonomic manager
	 * 
	 * @see AutonomicManager#getSourceScenario()
	 */
	@Override
	public Scenario getSourceScenario() {
		return this.sourceScenario;
	}
	
	/**
	 * Sets the source scenario of this abstract autonomic manager.
	 * 
	 * @param sourceScenario the source scenario to be set
	 * 
	 * @see AutonomicManager#setSourceScenario(Scenario)
	 */
	@Override
	public void setSourceScenario(Scenario sourceScenario) {
		this.sourceScenario = sourceScenario;
	}
	
	/**
	 * Determines whether or not this abstract autonomic manager has a source
	 * scenario.
	 * 
	 * @return true if this abstract autonomic manager has a source scenario,
	 *         false otherwise
	 * 
	 * @see AutonomicManager#hasSourceScenario()
	 */
	@Override
	public boolean hasSourceScenario() {
		return (null != this.sourceScenario);
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
	 * Gets the active planner of this abstract autonomic manager.
	 * 
	 * @return the active planner of this abstract autonomic manager
	 */
	protected ManagedPlanner getActivePlanner() {
		return this.activePlanner;
	}
	
	/**
	 * Sets the active planner of this abstract autonomic manager.
	 * 
	 * @param activePlanner the active planner to be set
	 */
	public void setActivePlanner(ManagedPlanner activePlanner) {
		this.activePlanner = activePlanner;
	}
	
	/**
	 * Determines whether or not this abstract autonomic manager has an active
	 * planner.
	 * 
	 * @return true if this abstract autonomic manager has an active planner,
	 *         false otherwise
	 */
	public boolean hasActivePlanner() {
		return (null != this.activePlanner);
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
	 * Gets the datalink of this abstract autonomic manager.
	 * 
	 * @return the datalink of this abstract autonomic manager
	 * 
	 * @see DatalinkCommunicator#getDatalink()
	 */
	@Override
	public Datalink getDatalink() {
		return this.getSourceScenario().getDatalink();
	}
	
	/**
	 * Sets the datalink of this abstract autonomic manager.
	 * 
	 * @param datalink the datalink to be set
	 * 
	 * @see DatalinkCommunicator#setDatalink(Datalink)
	 */
	@Override
	public void setDatalink(Datalink datalink) {
		this.getSourceScenario().setDatalink(datalink);
	}
	
	/**
	 * Determines whether or not this abstract autonomic manager has a
	 * datalink.
	 * 
	 * @return true if this abstract autonomic manager has a datalink,
	 *         false otherwise
	 * 
	 * @see DatalinkCommunicator#hasDatalink()
	 */
	@Override
	public boolean hasDatalink() {
		return this.hasSourceScenario();
	}
	
	/**
	 * Gets the datalink take-off communication of this abstract autonomic
	 * manager.
	 * 
	 * @return the datalink take-off communication of this abstract autonomic
	 *         manager
	 * 
	 * @see DatalinkCommunicator#getTakeOff()
	 */
	@Override
	public Communication<Datalink> getTakeOff() {
		return this.takeOff;
	}
	
	/**
	 * Sets the datalink take-off communication of this abstract autonomic
	 * manager.
	 * 
	 * @param takeOff the datalink take-off communication to be set
	 *
	 * @see DatalinkCommunicator#setTakeOff(Communication)
	 */
	@Override
	public void setTakeOff(Communication<Datalink> takeOff) {
		this.takeOff = takeOff;
	}
	
	/**
	 * Determines whether or not this abstract autonomic manager has a datalink
	 * take-off communication.
	 * 
	 * @return true if this abstract autonomic manager has a datalink take-off
	 *         communication, false otherwise
	 *
	 * @see DatalinkCommunicator#hasTakeOff()
	 */
	@Override
	public boolean hasTakeOff() {
		return (null != this.takeOff);
	}
	
	/**
	 * Gets the datalink landing communication of this abstract autonomic
	 * manager.
	 * 
	 * @return the datalink landing communication of this abstract autonomic
	 *         manager
	 *
	 * @see DatalinkCommunicator#getLanding()
	 */
	@Override
	public Communication<Datalink> getLanding() {
		return this.landing;
	}
	
	/**
	 * Sets the datalink landing communication of this abstract autonomic
	 * manager.
	 * 
	 * @param landing the datalink landing communication to be set
	 *
	 * @see DatalinkCommunicator#setLanding(Communication)
	 */
	@Override
	public void setLanding(Communication<Datalink> landing) {
		this.landing = landing;
	}
	
	/**
	 * Determines whether or not this abstract autonomic manager has a datalink
	 * landing communication.
	 * 
	 * @return true if this abstract autonomic manager has a datalink landing
	 *         communication, false otherwise
	 *
	 * @see DatalinkCommunicator#hasLanding() 
	 */
	@Override
	public boolean hasLanding() {
		return (null != this.landing);
	}
	
	/**
	 * Gets the datalink unplanned landing communication of this abstract
	 * autonomic manager.
	 * 
	 * @return the datalink unplanned landing communication of this abstract
	 *         autonomic manager
	 *
	 * @see DatalinkCommunicator#getUnplannedLanding()
	 */
	@Override
	public Communication<Datalink> getUnplannedLanding() {
		return this.unplannedLanding;
	}
	
	/**
	 * Sets the datalink unplanned landing communication of this abstract
	 * autonomic manager.
	 * 
	 * @param unplannedLanding the datalink unplanned landing communication to
	 *                         be set
	 *
	 * @see DatalinkCommunicator#setUnplannedLanding(Communication)
	 */
	@Override
	public void setUnplannedLanding(Communication<Datalink> unplannedLanding) {
		this.unplannedLanding = unplannedLanding;
	}
	
	/**
	 * Determines whether or not this abstract autonomic manager has a datalink
	 * unplanned landing communication.
	 * 
	 * @return true if this abstract autonomic manager has a datalink unplanned
	 *         landing communication, false otherwise
	 *
	 * @see DatalinkCommunicator#hasUnplannedLanding()
	 */
	@Override
	public boolean hasUnplannedLanding() {
		return (null != this.unplannedLanding);
	}
	
	/**
	 * Gets the establish datalink communication of this abstract autonomic
	 * manager.
	 * 
	 * @return the establish datalink communication of this abstract autonomic
	 *         manager
	 *
	 * @see DatalinkCommunicator#getEstablishDataLink()
	 */
	@Override
	public Communication<Datalink> getEstablishDataLink() {
		return this.establishDatalink;
	}
	
	/**
	 * Sets the establish datalink communication of this abstract autonomic
	 * manager.
	 * 
	 * @param establishDatalink the establish datalink communication to be set
	 *
	 * @see DatalinkCommunicator#setEstablishDatalink(Communication)
	 */
	@Override
	public void setEstablishDatalink(Communication<Datalink> establishDatalink) {
		this.establishDatalink = establishDatalink;
	}
	
	/**
	 * Determines whether or not this abstract autonomic manager has an
	 * establish datalink communication.
	 * 
	 * @return true if this abstract autonomic manager has an establish
	 *         datalink communication, false otherwise
	 *
	 * @see DatalinkCommunicator#hasEstablishDatalink()
	 */
	@Override
	public boolean hasEstablishDatalink() {
		return (null != this.establishDatalink);
	}
	
	/**
	 * Gets the maximum acceptable take-off error of this abstract autonomic
	 * manager to consider the aircraft within the take-off regime.
	 * 
	 * @return the maximum acceptable take-off error of this abstract autonomic
	 *         manager to consider the aircraft within the take-off regime
	 *
	 * @see DatalinkTracker#getMaxTakeOffError()
	 */
	@Override
	public AircraftTrackPointError getMaxTakeOffError() {
		return this.maxTakeOffError;
	}
	
	/**
	 * Sets the maximum acceptable take-off error of this abstract autonomic
	 * manager to consider the aircraft within the take-off regime.
	 * 
	 * @param maxTakeOffError the maximum acceptable take-off error to be set
	 *
	 * @throws IllegalArgumentException if the maximum take-off error is null
	 * 
	 * @see DatalinkTracker#setMaxTakeOffError(AircraftTrackPointError)
	 */
	@Override
	public void setMaxTakeOffError(AircraftTrackPointError maxTakeOffError) {
		if (null == maxTakeOffError) {
			throw new IllegalArgumentException();
		}
		this.maxTakeOffError = maxTakeOffError;
	}
	
	/**
	 * Determines whether or not the aircraft of this abstract autonomic
	 * manager is on track.
	 * 
	 * @return true if the aircraft of this abstract autonomic manager is on
	 *         track, false otherwise
	 * 
	 * @see DatalinkTracker#isOnTrack()
	 */
	@Override
	public boolean isOnTrack() {
		boolean isOnTrack = false;
		
		if (this.hasActivePlanner()) {
			isOnTrack = this.getActivePlanner().isOnTrack();
		}
		
		return isOnTrack;
	}
	
	/**
	 * Determines whether or not the aircraft of this abstract autonomic
	 * manager is within the take-off zone.
	 * 
	 * @return true if the aircraft of this abstract autonomic manager is
	 *         within the take-off zone, false otherwise
	 *
	 * @see DatalinkTracker#isInTakeOffZone()
	 */
	@Override
	public boolean isInTakeOffZone() {
		boolean isInTakeOffZone = false;
		
		if (this.hasActivePlanner()) {
			isInTakeOffZone = this.getActivePlanner().isInTakeOffZone();
		}
		
		return isInTakeOffZone;
	}
	
	/**
	 * Determines whether or not the aircraft of this abstract autonomic
	 * manager is within the take-off window.
	 * 
	 * @return true if the aircraft of this abstract autonomic manager is
	 *         within the take-off window, false otherwise
	 *
	 * @see DatalinkTracker#isInTakeOffWindow()
	 */
	@Override
	public boolean isInTakeOffWindow() {
		boolean isInTakeOffWindow = false;
		
		if (this.hasActivePlanner()) {
			isInTakeOffWindow = this.getActivePlanner().isInTakeOffWindow();
		}
		
		return isInTakeOffWindow;
	}
	
	/**
	 * Gets the maximum acceptable landing error of this abstract autonomic
	 * manager to consider the aircraft within the landing regime.
	 * 
	 * @return the maximum acceptable landing error of this abstract autonomic
	 *         manager to consider the aircraft within the landing regime
	 *
	 * @see DatalinkTracker#getMaxLandingError()
	 */
	@Override
	public AircraftTrackPointError getMaxLandingError() {
		return this.maxLandingError;
	}
	
	/**
	 * Sets the maximum acceptable landing error of this abstract autonomic
	 * manager to consider the aircraft within the landing regime.
	 * 
	 * @param maxLandingError the maximum acceptable landing error to be set
	 *
	 * @throws IllegalArgumentException if the maximum landing error is null
	 *
	 * @see DatalinkTracker#setMaxLandingError(AircraftTrackPointError)
	 */
	@Override
	public void setMaxLandingError(AircraftTrackPointError maxLandingError) {
		if (null == maxLandingError) {
			throw new IllegalArgumentException();
		}
		this.maxLandingError = maxLandingError;
	}
	
	/**
	 * Determines whether or not the aircraft of this abstract autonomic
	 * manager is within the landing zone.
	 * 
	 * @return true if the aircraft of this abstract autonomic manager is
	 *         within the landing zone, false otherwise
	 *
	 * @see DatalinkTracker#isInLandingZone()
	 */
	@Override
	public boolean isInLandingZone() {
		boolean isInLandingZone = false;
		
		if (this.hasActivePlanner()) {
			isInLandingZone = this.getActivePlanner().isInLandingZone();
		}
		
		return isInLandingZone;
	}
	
	/**
	 * Determines whether or not the aircraft of this abstract autonomic
	 * manager is within the landing window.
	 * 
	 * @return true if the aircraft of this abstract autonomic manager is
	 *         within the landing window, false otherwise
	 *
	 * @see DatalinkTracker#isInLandingWindow()
	 */
	@Override
	public boolean isInLandingWindow() {
		boolean isInLandingWindow = false;
		
		if (this.hasActivePlanner()) {
			isInLandingWindow = this.getActivePlanner().isInLandingWindow();
		}
		
		return isInLandingWindow;
	}
	
	/**
	 * Gets the maximum acceptable track error of this abstract autonomic
	 * manager to consider the aircraft on track.
	 * 
	 * @return the maximum acceptable track error of this abstract autonomic
	 *         manager to consider the aircraft on track
	 * 
	 * @see DatalinkTracker#getMaxTrackError()
	 */
	@Override
	public AircraftTrackError getMaxTrackError() {
		return this.maxTrackError;
	}
	
	/**
	 * Sets the maximum acceptable track error of this abstract autonomic
	 * manager to consider the aircraft on track.
	 * 
	 * @param maxTrackError the maximum acceptable track error to be set
	 * 
	 * @throws IllegalArgumentException if the maximum track error is null
	 * 
	 * @see DatalinkTracker#setMaxTrackError(AircraftTrackError)
	 */
	@Override
	public void setMaxTrackError(AircraftTrackError maxTrackError) {
		if (null == maxTrackError) {
			throw new IllegalArgumentException();
		}
		this.maxTrackError = maxTrackError;
	}
	
	/**
	 * Initializes this abstract autonomic manager for a managed session.
	 * 
	 * @param managedSession the managed session
	 */
	protected void initialize(Session managedSession) {
		if (!this.hasSourceScenario()) {
			this.setSourceScenario(managedSession.getActiveScenario());
		}
		
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
					
					if ((null != planner) && (planner instanceof ManagedPlanner)) {
						ManagedPlanner managedPlanner = (ManagedPlanner) planner;
						managedScenario.setPlanner(managedPlanner);
						
						// confirm environment and planner compatibility
						if (managedPlanner.supports(managedScenario.getAircraft())
								&& managedPlanner.supports(managedScenario.getEnvironment())
								&& managedPlanner.supports(pois) && (1 < pois.size())) {
							
							// managed scenario aircraft position
							managedScenario.moveAircraft(pois.get(0));
							// managed scenario obstacles
							managedScenario.submitAddObstacles(this.getSourceScenario().getObstacles());
							//managedScenario.submitAddObstacles(this.getSourceScenario().getEmbeddedObstacles());
							managedScenario.commitObstacleChange();
							
							// managed planner online properties
							managedPlanner.setStandby(true);
							managedPlanner.setDatalink(this.getSourceScenario().getDatalink());
							managedPlanner.setTakeOff(this.getTakeOff());
							managedPlanner.setLanding(this.getLanding());
							managedPlanner.setUnplannedLanding(this.getUnplannedLanding());
							managedPlanner.setEstablishDatalink(this.getEstablishDataLink());
							managedPlanner.setMaxLandingError(this.getMaxLandingError());
							managedPlanner.setMaxTakeOffError(this.getMaxTakeOffError());
							managedPlanner.setMaxTrackError(this.getMaxTrackError());
							
							managedPlanner.addPlanRevisionListener(new PlanRevisionListener() {
								@Override
								public void revisePlan(Trajectory trajectory) {
									evaluatePlanner(managedScenario, managedPlanner, trajectory);
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
							managedPlanner.update(tuning.getSpecification());
							
							// add elaborated managed scenario
							this.managedScenarios.put(managedScenario, tuning);
							managedSession.addScenario(managedScenario);
						}
					}
				}
			}
		}
		
		if (this.hasManagedScenarios()) {
			// execute managed planners in parallel
			this.setActivePlanner((ManagedPlanner)
					this.getManagedScenarios().stream().findFirst().get().getPlanner());
			this.getActivePlanner().setStandby(false);
			this.plannerExecutor = Executors.newWorkStealingPool(this.getManagedScenarios().size());
			// inject all source scenario environment changes into managed scenarios
			this.getSourceScenario().clearTrajectory();
			this.setObstacleManager(this.getSourceScenario());
			this.getSourceScenario().setDynamicObstacleListener(this);
		}
	}
	
	/**
	 * Runs this abstract autonomic manager for a managed session.
	 * 
	 * @param managedSession the managed session
	 */
	protected void run(Session managedSession) {
		ArrayList<ManagedPlanner> managedPlanners = new ArrayList<>();
		
		for (Scenario managedScenario : this.getManagedScenarios()) {
			System.out.println(managedScenario.getId());
			System.out.println(this.getPlannerTuning(managedScenario).getFeatures());
		
			ManagedPlanner managedPlanner = (ManagedPlanner) managedScenario.getPlanner();
			ManagedGoals managedGoals = new ManagedGoals();
			
			List<Position> pois = new ArrayList<Position>();
			pois.addAll(managedScenario.getWaypoints());
			managedGoals.setOrigin(pois.remove(0));
			managedGoals.setDestination(pois.remove(pois.size() - 1));
			managedGoals.addAllPois(pois);
			managedGoals.setEtd(managedScenario.getTime());
			
			managedPlanner.setGoals(managedGoals);
			managedPlanners.add(managedPlanner);
		}
		
		try {
			if (!managedPlanners.isEmpty()) {
				this.plannerExecutor.invokeAll(managedPlanners);
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * Cleans up this abstract autonomic manager and removes all its managed
	 * scenarios from its managed session.
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
			this.plannerExecutor.shutdown();
			this.setActivePlanner(null);
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
			((ManagedPlanner) managedScenario.getPlanner()).terminate();
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
	
	// TODO: include replanning callback?
	
	/**
	 * Notifies this abstract autonomic manager about a pending obstacle change
	 * and invokes a tuning and joining procedure as required.
	 * 
	 * @see DynamicObstacleListener#notifyPendingObstacleChange()
	 */
	@Override
	public synchronized void notifyPendingObstacleChange() {
		this.tunerExecutor.execute(new Runnable() {
			@Override
			public void run() {
				if (hasObstacleManager()
						&& !getObstacleManager().commitObstacleChange().isEmpty()) {
					getSourceScenario().clearTrajectory();
					
					// update all managed scenarios and planner tunings
					for (Scenario managedScenario : getManagedScenarios()) {
						PlannerTuning tuning = getPlannerTuning(managedScenario);
						// TODO: environment obstacles only
						tuning.getFeatures().extractFeatures(getSourceScenario());
						System.out.println(tuning.getFeatures());
						List<Properties<Planner>> candidates = tuning.tune();
						// TODO: default candidate
						tuning.getSpecification().setProperties(candidates.get(0));
						
						ManagedPlanner managedPlanner = (ManagedPlanner) managedScenario.getPlanner();
						managedPlanner.update(tuning.getSpecification());
						if (getActivePlanner() != managedPlanner) {
							// join active planner in trajectory competition
							managedPlanner.join(getActivePlanner());
						}
						
						managedScenario.submitReplaceObstacles(
								getSourceScenario().getObstacles());
						//managedScenario.submitReplaceObstacles(
						//		getSourceScenario().getEmbeddedObstacles());
					}
				}
			}
		});
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
	
	/**
	 * Evaluates a managed planner's trajectory and selects an active planner
	 * accordingly.
	 * 
	 * @param scenario the managed scenario of the planner
	 * @param planner the managed planner
	 * @param trajectory the trajectory computed by the managed planner
	 */
	private synchronized void evaluatePlanner(
			Scenario scenario, ManagedPlanner planner, Trajectory trajectory) {
		styleTrajectory(trajectory);
		scenario.setTrajectory(trajectory);
		// TODO: measure performance and update reputation
		// TODO: features and knowledge base
		Trajectory sourceTrajectory = getSourceScenario().getTrajectory();
		
		
		// comparison of trajectories can only be based on the same start
		// and end POIs using the same environment normalizer
		List<Waypoint> spois = sourceTrajectory.getPois();
		List<Waypoint> rpois = trajectory.getPois();
		int poiIndex = Math.min(spois.size(), rpois.size()) - 1;
		System.out.println("common POI indices = " + poiIndex);
		
		Trajectory st = sourceTrajectory.getSubTrajectory(poiIndex);
		TrajectoryQuality stq = new TrajectoryQuality(st);
		Trajectory rt = trajectory.getSubTrajectory(poiIndex);
		TrajectoryQuality rtq = new TrajectoryQuality(rt);
		
		boolean commonStart = true;
		if (!st.isEmpty() && !rt.isEmpty()) {
			commonStart = st.getFirstWaypoint().equals(rt.getFirstWaypoint());
		}
		
		// TODO: multiple parts / uploading versus showing
		// consider non-empty, improved, or longer revised trajectory
		if (!trajectory.isEmpty() && commonStart && (sourceTrajectory.isEmpty()
				|| (0 > stq.compareTo(rtq))
				|| ((0 == stq.compareTo(rtq)) && (rpois.size() >= poiIndex)))) {
			getSourceScenario().setTrajectory(trajectory);
			
			// select improved planner if necessary
			if (getActivePlanner() !=  planner) {
				getActivePlanner().setStandby(true);
				setActivePlanner(planner);
				getActivePlanner().setStandby(false);
				getActivePlanner().getMissionLoader().revisePlan(trajectory);
				Logging.logger().info("new active planner is " + getActivePlanner().getId());
			}
		}
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
