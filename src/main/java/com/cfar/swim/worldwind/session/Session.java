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
package com.cfar.swim.worldwind.session;

import java.beans.PropertyChangeListener;
import java.beans.PropertyChangeSupport;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Optional;
import java.util.Set;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.connections.Datalink;
import com.cfar.swim.worldwind.connections.SwimConnection;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.managers.AutonomicManager;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.registries.Factory;
import com.cfar.swim.worldwind.registries.Registry;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.aircraft.A320Properties;
import com.cfar.swim.worldwind.registries.aircraft.AircraftFactory;
import com.cfar.swim.worldwind.registries.aircraft.H135Properties;
import com.cfar.swim.worldwind.registries.aircraft.IrisProperties;
import com.cfar.swim.worldwind.registries.connections.DatalinkFactory;
import com.cfar.swim.worldwind.registries.connections.DronekitDatalinkProperties;
import com.cfar.swim.worldwind.registries.connections.LiveSwimConnectionProperties;
import com.cfar.swim.worldwind.registries.connections.SimulatedDatalinkProperties;
import com.cfar.swim.worldwind.registries.connections.SimulatedSwimConnectionProperties;
import com.cfar.swim.worldwind.registries.connections.SwimConnectionFactory;
import com.cfar.swim.worldwind.registries.environments.EnvironmentFactory;
import com.cfar.swim.worldwind.registries.environments.PlanningContinuumProperties;
import com.cfar.swim.worldwind.registries.environments.PlanningGridProperties;
import com.cfar.swim.worldwind.registries.environments.PlanningRoadmapProperties;
import com.cfar.swim.worldwind.registries.managers.HeuristicManagerProperties;
import com.cfar.swim.worldwind.registries.managers.ManagerFactory;
import com.cfar.swim.worldwind.registries.managers.SmacManagerProperties;
import com.cfar.swim.worldwind.registries.planners.PlannerFactory;
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
import com.cfar.swim.worldwind.util.Identifiable;

import gov.nasa.worldwind.Configuration;
import gov.nasa.worldwind.avlist.AVKey;

/**
 * Realizes a planning session that aggregates planning scenarios,
 * registries and a setup.
 * 
 * @author Stephan Heinemann
 *
 */
public class Session implements Identifiable {
	
	// TODO: a managed scenario can be enabled/disabled externally
	
	// TODO: if a session is used concurrently (AM, UI monitoring),
	// then this should be a protected monitor
	
	/** the default session identifier */
	public static final String DEFAULT_SESSION_ID = "Default Session";
	
	/** the property change support of this session */
	private final PropertyChangeSupport pcs = new PropertyChangeSupport(this);
	
	/** the identifier of this session */
	private final String id;
	
	/** the scenarios of this session */
	private final Set<Scenario> scenarios = new LinkedHashSet<Scenario>();
	
	/** the default scenario of this session */
	private final Scenario defaultScenario = new Scenario();
	
	/** the active scenario of this session */
	private Scenario activeScenario = this.defaultScenario;
	
	/** the aircraft registry of this session */
	private Registry<Aircraft> aircraftRegistry = new Registry<>();
	
	/** the aircraft factory of this session */
	private AircraftFactory aircraftFactory = new AircraftFactory();
	
	/** the environment registry of this session */
	private Registry<Environment> environmentRegistry = new Registry<>();
	
	/** the environment factory of this session */
	private EnvironmentFactory environmentFactory = new EnvironmentFactory(this.getActiveScenario());
	
	/** the planner registry of this session */
	private Registry<Planner> plannerRegistry = new Registry<>();
	
	/** the managed planner registry of this session */
	private Registry<Planner> managedPlannerRegistry = new Registry<>();
	
	/** the planner factory of this session */
	private PlannerFactory plannerFactory = new PlannerFactory(this.getActiveScenario());
	
	/** the datalink registry of this session */
	private Registry<Datalink> datalinkRegistry = new Registry<>();
	
	/** the datalink factory of this session */
	private DatalinkFactory datalinkFactory = new DatalinkFactory();
	
	/** the SWIM connection registry of this session */
	private Registry<SwimConnection> swimConnectionRegistry = new Registry<>();
	
	/** the SWIM connection factory of this session */
	private SwimConnectionFactory swimConnectionFactory = new SwimConnectionFactory();
	
	/** the manager registry of this session */
	private Registry<AutonomicManager> managerRegistry = new Registry<>();
	
	/** the manager factory of this session */
	private ManagerFactory managerFactory = new ManagerFactory(this.getActiveScenario());
	
	/** the setup of this session */
	private Setup setup;
	
	/** the autonomic manager of this session */
	private AutonomicManager manager = null;
	
	/**
	 * Constructs and initializes a default session.
	 */
	public Session() {
		this(Session.DEFAULT_SESSION_ID);
	}
	
	/**
	 * Constructs and initializes a session with a specified session identifier.
	 * 
	 * @param id the session identifier
	 */
	public Session(String id) {
		this.id = id;
		this.init();
	}
	
	/**
	 * Gets the identifier of this session.
	 * 
	 * @return the identifier of this session
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return this.id;
	}
	
	/**
	 * Initializes this session.
	 */
	public void init() {
		Configuration.setValue(
    			AVKey.MIL_STD_2525_ICON_RETRIEVER_PATH,
    			this.getClass().getClassLoader().getResource("milstd2525"));
		
		this.clearScenarios();
		
		// aircraft
		this.aircraftRegistry.clearSpecifications();
		this.aircraftRegistry.addSpecification(new Specification<Aircraft>(Specification.AIRCRAFT_IRIS_ID, Specification.AIRCRAFT_IRIS_DESCRIPTION, new IrisProperties()));
		this.aircraftRegistry.addSpecification(new Specification<Aircraft>(Specification.AIRCRAFT_A320_ID, Specification.AIRCRAFT_A320_DESCRIPTION, new A320Properties()));
		this.aircraftRegistry.addSpecification(new Specification<Aircraft>(Specification.AIRCRAFT_H135_ID, Specification.AIRCRAFT_H135_DESCRIPTION, new H135Properties()));
		
		// environments
		this.environmentRegistry.clearSpecifications();
		this.environmentRegistry.addSpecification(new Specification<Environment>(Specification.ENVIRONMENT_PLANNING_GRID_ID, Specification.ENVIRONMENT_PLANNING_GRID_DESCRIPTION, new PlanningGridProperties()));
		this.environmentRegistry.addSpecification(new Specification<Environment>(Specification.ENVIRONMENT_PLANNING_ROADMAP_ID, new PlanningRoadmapProperties()));
		this.environmentRegistry.addSpecification(new Specification<Environment>(Specification.ENVIRONMENT_PLANNING_CONTINUUM_ID, Specification.ENVIRONMENT_PLANNING_CONTINUUM_DESCRIPTION, new PlanningContinuumProperties()));
		this.addActiveScenarioChangeListener(this.environmentFactory.getActiveScenarioChangeListener());
		
		// planners
		this.plannerRegistry.clearSpecifications();
		this.plannerRegistry.addSpecification(new Specification<Planner>(Specification.PLANNER_FAS_ID, Specification.PLANNER_FAS_DESCRIPTION, new ForwardAStarProperties()));
		this.plannerRegistry.addSpecification(new Specification<Planner>(Specification.PLANNER_TS_ID, Specification.PLANNER_TS_DESCRIPTION, new ThetaStarProperties()));
		this.plannerRegistry.addSpecification(new Specification<Planner>(Specification.PLANNER_ARAS_ID, Specification.PLANNER_ARAS_DESCRIPTION, new ARAStarProperties()));
		this.plannerRegistry.addSpecification(new Specification<Planner>(Specification.PLANNER_ADS_ID, Specification.PLANNER_ADS_DESCRIPTION, new ADStarProperties()));
		this.plannerRegistry.addSpecification(new Specification<Planner>(Specification.PLANNER_OADS_ID, Specification.PLANNER_OADS_DESCRIPTION, new OADStarProperties()));
		this.plannerRegistry.addSpecification(new Specification<Planner>(Specification.PLANNER_RRT_ID, Specification.PLANNER_RRT_DESCRIPTION, new RRTreeProperties()));
		this.plannerRegistry.addSpecification(new Specification<Planner>(Specification.PLANNER_RRTS_ID, Specification.PLANNER_RRTS_DESCRIPTION, new RRTreeProperties()));
		this.plannerRegistry.addSpecification(new Specification<Planner>(Specification.PLANNER_HRRT_ID, Specification.PLANNER_HRRT_DESCRIPTION, new HRRTreeProperties()));
		this.plannerRegistry.addSpecification(new Specification<Planner>(Specification.PLANNER_ARRT_ID, Specification.PLANNER_ARRT_DESCRIPTION, new ARRTreeProperties()));
		this.plannerRegistry.addSpecification(new Specification<Planner>(Specification.PLANNER_DRRT_ID, Specification.PLANNER_DRRT_DESCRIPTION, new DRRTreeProperties()));
		this.plannerRegistry.addSpecification(new Specification<Planner>(Specification.PLANNER_ADRRT_ID, Specification.PLANNER_ADRRT_DESCRIPTION, new ADRRTreeProperties()));
		this.plannerRegistry.addSpecification(new Specification<Planner>(Specification.PLANNER_OADRRT_ID, Specification.PLANNER_OADRRT_DESCRIPTION, new OADRRTreeProperties()));
		this.addActiveScenarioChangeListener(this.plannerFactory.getActiveScenarioChangeListener());
		
		// managed planners
		this.managedPlannerRegistry.clearSpecifications();
		this.managedPlannerRegistry.addSpecification(new Specification<Planner>(Specification.PLANNER_MGP_ID, this.plannerRegistry.getSpecification(Specification.PLANNER_OADS_ID).getProperties()));
		this.managedPlannerRegistry.addSpecification(new Specification<Planner>(Specification.PLANNER_MTP_ID, this.plannerRegistry.getSpecification(Specification.PLANNER_OADRRT_ID).getProperties()));
		
		// datalinks
		this.datalinkRegistry.clearSpecifications();
		this.datalinkRegistry.addSpecification(new Specification<Datalink>(Specification.CONNECTION_DATALINK_DRONEKIT_ID, Specification.CONNECTION_DATALINK_DRONEKIT_DESCRIPTION, new DronekitDatalinkProperties()));
		this.datalinkRegistry.addSpecification(new Specification<Datalink>(Specification.CONNECTION_DATALINK_SIMULATED_ID, Specification.CONNECTION_DATALINK_SIMULATED_DESCRIPTION, new SimulatedDatalinkProperties()));
		
		// SWIM connections
		this.swimConnectionRegistry.clearSpecifications();
		this.swimConnectionRegistry.addSpecification(new Specification<SwimConnection>(Specification.CONNECTION_SWIM_LIVE_ID, Specification.CONNECTION_SWIM_LIVE_DESCRIPTION, new LiveSwimConnectionProperties()));
		this.swimConnectionRegistry.addSpecification(new Specification<SwimConnection>(Specification.CONNECTION_SWIM_SIMULATED_ID, Specification.CONNECTION_SWIM_SIMULATED_DESCRIPTION, new SimulatedSwimConnectionProperties()));
		
		// autonomic managers
		this.managerRegistry.clearSpecifications();
		this.managerRegistry.addSpecification(new Specification<AutonomicManager>(Specification.MANAGER_HEURISTIC_ID, Specification.MANAGER_HEURISTIC_DESCRIPTION, new HeuristicManagerProperties()));
		this.managerRegistry.addSpecification(new Specification<AutonomicManager>(Specification.MANAGER_SMAC_ID, Specification.MANAGER_SMAC_DESCRIPTION, new SmacManagerProperties()));
		this.addActiveScenarioChangeListener(this.managerFactory.getActiveScenarioChangeListener());
		
		// modifications on setup shall always be reflected in the registries
		this.setup = new Setup();
		this.setup.setAircraftSpecification(this.aircraftRegistry.getSpecification(Specification.AIRCRAFT_IRIS_ID));
		this.setup.setEnvironmentSpecification(this.environmentRegistry.getSpecification(Specification.ENVIRONMENT_PLANNING_GRID_ID));
		this.setup.setPlannerSpecification(this.plannerRegistry.getSpecification(Specification.PLANNER_FAS_ID));
		this.setup.setDatalinkSpecification(this.datalinkRegistry.getSpecification(Specification.CONNECTION_DATALINK_SIMULATED_ID));
		this.setup.setSwimConnectionSpecification(this.swimConnectionRegistry.getSpecification(Specification.CONNECTION_SWIM_SIMULATED_ID));
		this.setup.setManagerSpecification(this.managerRegistry.getSpecification(Specification.MANAGER_HEURISTIC_ID));
	}
	
	/**
	 * Adds a property change listener to this session.
	 * 
	 * @param listener the property change listener to be added
	 */
	public void addPropertyChangeListener(PropertyChangeListener listener) {
		this.pcs.addPropertyChangeListener(listener);
	}
	
	/**
	 * Removes a property change listener from this session.
	 * 
	 * @param listener the property change listener to be removed
	 */
	public void removePropertyChangeListener(PropertyChangeListener listener) {
		this.pcs.removePropertyChangeListener(listener);
	}
	
	/**
	 * Adds a scenarios change listener to this session.
	 * 
	 * @param listener the scenarios change listener to be added
	 */
	public void addScenariosChangeListener(PropertyChangeListener listener) {
		this.pcs.addPropertyChangeListener("scenarios", listener);
	}
	
	/**
	 * Adds an active scenario change listener to this session.
	 * 
	 * @param listener the active scenario change listener to be added
	 */
	public void addActiveScenarioChangeListener(PropertyChangeListener listener) {
		this.pcs.addPropertyChangeListener("activeScenario", listener);
	}
	
	/**
	 * Gets a scenario with a specified identifier from this session.
	 *  
	 * @param id the scenario identifier
	 * 
	 * @return the identified scenario if present, null otherwise
	 */
	public Scenario getScenario(String id) {
		Scenario scenario = null;
		Optional<Scenario> optScenario = this.scenarios.stream().filter(s -> s.getId().equals(id)).findFirst();
		
		if (optScenario.isPresent()) {
			scenario = optScenario.get();
		}
		
		return scenario;
	}
	
	/**
	 * Removes a scenario with a specified identifier from this session
	 * if present. The default scenario cannot be removed. If the removed
	 * scenario was enabled, the default scenario will be enabled.
	 * 
	 * @param id the scenario identifier of the scenario to be removed
	 */
	public void removeScenario(String id) {
		if (!id.equals(Scenario.DEFAULT_SCENARIO_ID)) {
			Optional<Scenario> optScenario = this.scenarios.stream().filter(s -> s.getId().equals(id)).findFirst();
			
			if (optScenario.isPresent()) {
				this.removeScenario(optScenario.get());
			}
		}
	}
	
	/**
	 * Sets the active scenario with a specified identifier of this session
	 * if present. The previously enabled scenario will be disabled.
	 * 
	 * @param id the scenario identifier of the scenario to be disabled
	 */
	public void setActiveScenario(String id) {
		Optional<Scenario> optScenario = this.scenarios.stream().filter(s -> s.getId().equals(id)).findFirst();
		
		if (optScenario.isPresent()) {
			this.setActiveScenario(optScenario.get());
		}
	}
	
	/**
	 * Gets the default scenario of this session.
	 * 
	 * @return the default scenario of this session
	 */
	public Scenario getDefaultScenario() {
		return this.defaultScenario;
	}
	
	/**
	 * Gets the active scenario of this session.
	 * 
	 * @return the active scenario of this session
	 */
	public Scenario getActiveScenario() {
		return this.activeScenario;
	}
	
	/**
	 * Sets the active scenario of this session. The scenario is added if not present.
	 * 
	 * @param scenario the scenario to be activated
	 */
	public void setActiveScenario(Scenario scenario) {
		this.addScenario(scenario);
		this.activeScenario.disable();
		this.activeScenario = this.getScenario(scenario.getId());
		this.activeScenario.enable();
		this.pcs.firePropertyChange("activeScenario", null, this.activeScenario);
	}
	
	/**
	 * Adds a scenario to this session if not present. Disables the scenario if
	 * added.
	 * 
	 * @param scenario the scenario to be added
	 */
	public void addScenario(Scenario scenario) {
		if (this.scenarios.add(scenario)) {
			scenario.disable();
			this.pcs.firePropertyChange("scenarios", null, (Iterable<Scenario>) this.scenarios);
		}
	}
	
	/**
	 * Removes a scenario from this session if present. The default scenario
	 * cannot be removed. If the active scenario is removed, the default
	 * scenario becomes the active scenario.
	 * 
	 * @param scenario the scenario to be removed
	 */
	public void removeScenario(Scenario scenario) {
		if (!scenario.equals(this.defaultScenario)) {
			if (this.scenarios.remove(scenario)) {
				if (this.activeScenario.equals(scenario)) {
					this.activeScenario.disable();
					this.activeScenario = this.defaultScenario;
					this.activeScenario.enable();
					this.pcs.firePropertyChange("activeScenario", null, this.activeScenario);
				}
				this.pcs.firePropertyChange("scenarios", null, (Iterable<Scenario>) this.scenarios);
			}
		}
	}
	
	/**
	 * Removes all scenarios of this session but the default scenario.
	 */
	public void clearScenarios() {
		this.scenarios.clear();
		this.scenarios.add(this.defaultScenario);
		this.activeScenario.disable();
		this.activeScenario = this.defaultScenario;
		this.activeScenario.enable();
		this.pcs.firePropertyChange("activeScenario", null, this.activeScenario);
		this.pcs.firePropertyChange("scenarios", null, (Iterable<Scenario>) this.scenarios);
	}
	
	/**
	 * Gets all scenarios of this session.
	 * 
	 * @return all scenarios of this session
	 */
	public Set<Scenario> getScenarios() {
		return Collections.unmodifiableSet(this.scenarios);
	}
	
	/**
	 * Gets the aircraft specifications of this session.
	 * 
	 * @return the aircraft specifications of this session
	 */
	public Set<Specification<Aircraft>> getAircraftSpecifications() {
		return this.aircraftRegistry.getSpecifications();
	}
	
	/**
	 * Gets an identified aircraft specification from this session.
	 * 
	 * @param id the aircraft specification identifier
	 * 
	 * @return the identified aircraft specification, or null otherwise
	 */
	public Specification<Aircraft> getAircraftSpecification(String id) {
		Specification<Aircraft> aircraftSpec = null;
		Optional<Specification<Aircraft>> optSpec =
				this.aircraftRegistry.getSpecifications()
				.stream()
				.filter(s -> s.getId().equals(id))
				.findFirst();
		
		if (optSpec.isPresent()) {
			aircraftSpec = optSpec.get();
		}
		
		return aircraftSpec;
	}
	
	/**
	 * Gets the aircraft factory of this session.
	 * 
	 * @return the aircraft factory of this session
	 */
	public Factory<Aircraft> getAircraftFactory() {
		return this.aircraftFactory;
	}
	
	/**
	 * Gets the environment specifications of this session.
	 * 
	 * @return the environment specifications of this session
	 */
	public Set<Specification<Environment>> getEnvironmentSpecifications() {
		return this.environmentRegistry.getSpecifications();
	}
	
	/**
	 * Gets an identified environment specification from this session.
	 * 
	 * @param id the environment specification identifier
	 * 
	 * @return the identified environment specification, or null otherwise
	 */
	public Specification<Environment> getEnvironmentSpecification(String id) {
		Specification<Environment> envSpec = null;
		Optional<Specification<Environment>> optSpec =
				this.environmentRegistry.getSpecifications()
				.stream()
				.filter(s -> s.getId().equals(id))
				.findFirst();
		
		if (optSpec.isPresent()) {
			envSpec = optSpec.get();
		}
		
		return envSpec;
	}
	
	/**
	 * Gets the environment factory of this session.
	 * 
	 * @return the environment factory of this session
	 */
	public Factory<Environment> getEnvironmentFactory() {
		return this.environmentFactory;
	}
	
	/**
	 * Gets the planner specifications of this session.
	 * 
	 * @return the planner specifications of this session
	 */
	public Set<Specification<Planner>> getPlannerSpecifications() {
		return this.plannerRegistry.getSpecifications();
	}
	
	/**
	 * Gets an identified planner specification from this session.
	 * 
	 * @param id the planner specification identifier
	 * 
	 * @return the identified planner specification, or null otherwise
	 */
	public Specification<Planner> getPlannerSpecification(String id) {
		Specification<Planner> plannerSpec = null;
		Optional<Specification<Planner>> optSpec =
				this.plannerRegistry.getSpecifications()
				.stream()
				.filter(s -> s.getId().equals(id))
				.findFirst();
		
		if (optSpec.isPresent()) {
			plannerSpec = optSpec.get();
		}
		
		return plannerSpec;
	}
	
	/**
	 * Gets the managed planner specifications of this session.
	 * 
	 * @return the managed planner specifications of this session
	 */
	public Set<Specification<Planner>> getManagedPlannerSpecifications() {
		return this.managedPlannerRegistry.getSpecifications();
	}
	
	/**
	 * Gets an identified managed planner specification from this session.
	 * 
	 * @param id the managed planner specification identifier
	 * 
	 * @return the identified managed planner specification, or null otherwise
	 */
	public Specification<Planner> getManagedPlannerSpecification(String id) {
		Specification<Planner> plannerSpec = null;
		Optional<Specification<Planner>> optSpec =
				this.managedPlannerRegistry.getSpecifications()
				.stream()
				.filter(s -> s.getId().equals(id))
				.findFirst();
		
		if (optSpec.isPresent()) {
			plannerSpec = optSpec.get();
		}
		
		return plannerSpec;
	}
	
	/**
	 * Gets the planner factory of this session.
	 * 
	 * @return the planner factory of this session
	 */
	public Factory<Planner> getPlannerFactory() {
		return this.plannerFactory;
	}
	
	/**
	 * Gets the datalink specifications of this session.
	 * 
	 * @return the datalink specifications of this session
	 */
	public Set<Specification<Datalink>> getDatalinkSpecifications() {
		return this.datalinkRegistry.getSpecifications();
	}
	
	/**
	 * Gets an identified datalink specification from this session.
	 * 
	 * @param id the datalink specification identifier
	 * 
	 * @return the identified datalink specification, or null otherwise
	 */
	public Specification<Datalink> getDatalinkSpecification(String id) {
		Specification<Datalink> datalinkSpec = null;
		Optional<Specification<Datalink>> optSpec =
				this.datalinkRegistry.getSpecifications()
				.stream()
				.filter(s -> s.getId().equals(id))
				.findFirst();
		
		if (optSpec.isPresent()) {
			datalinkSpec = optSpec.get();
		}
		
		return datalinkSpec;
	}
	
	/**
	 * Gets the datalink factory of this session.
	 * 
	 * @return the datalink factory of this session
	 */
	public Factory<Datalink> getDatalinkFactory() {
		return this.datalinkFactory;
	}
	
	/**
	 * Gets the SWIM connection specifications of this session.
	 * 
	 * @return the SWIM connection specifications of this session
	 */
	public Set<Specification<SwimConnection>> getSwimConnectionSpecifications() {
		return this.swimConnectionRegistry.getSpecifications();
	}
	
	/**
	 * Gets an identified SWIM connection specification from this session.
	 * 
	 * @param id the SWiM connection specification identifier
	 * 
	 * @return the identified SWIM connection specification, or null otherwise
	 */
	public Specification<SwimConnection> getSwimConnectionSpecification(String id) {
		Specification<SwimConnection> swimConnectionSpec = null;
		Optional<Specification<SwimConnection>> optSpec =
				this.swimConnectionRegistry.getSpecifications()
				.stream()
				.filter(s -> s.getId().equals(id))
				.findFirst();
		
		if (optSpec.isPresent()) {
			swimConnectionSpec = optSpec.get();
		}
		
		return swimConnectionSpec;
	}
	
	/**
	 * Gets the SWIM connection factory of this session.
	 * 
	 * @return the SWIM connection factory of this session
	 */
	public Factory<SwimConnection> getSwimConnectionFactory() {
		return this.swimConnectionFactory;
	}
	
	/**
	 * Gets the manager specifications of this session.
	 * 
	 * @return the manager specifications of this session
	 */
	public Set<Specification<AutonomicManager>> getManagerSpecifications() {
		return this.managerRegistry.getSpecifications();
	}
	
	/**
	 * Gets an identified manager specification from this session.
	 * 
	 * @param id the manager specification identifier
	 * 
	 * @return the identified manager specification, or null otherwise
	 */
	public Specification<AutonomicManager> getManagerSpecification(String id) {
		Specification<AutonomicManager> managerSpec = null;
		Optional<Specification<AutonomicManager>> optSpec =
				this.managerRegistry.getSpecifications()
				.stream()
				.filter(s -> s.getId().equals(id))
				.findFirst();
		
		if (optSpec.isPresent()) {
			managerSpec = optSpec.get();
		}
		
		return managerSpec;
	}
	
	/**
	 * Gets the manager factory of this session.
	 * 
	 * @return the manager factory of this session
	 */
	public Factory<AutonomicManager> getManagerFactory() {
		return this.managerFactory;
	}
	
	/**
	 * Gets the setup of this session.
	 * 
	 * @return the setup of this session.
	 */
	public Setup getSetup() {
		return this.setup;
	}
	
	/**
	 * Sets the setup of this session.
	 * 
	 * @param setup the setup to be set
	 */
	public void setSetup(Setup setup) {
		this.setup = setup;
	}
	
	/**
	 * Gets the autonomic manager of this session.
	 * 
	 * @return the autonomic manager of this session
	 */
	public AutonomicManager getManager() {
		return this.manager;
	}
	
	/**
	 * Sets the autonomic manager of this session.
	 * 
	 * @param manager the autonomic manager of this session
	 */
	public void setManager(AutonomicManager manager) {
		this.manager = manager;
	}
	
	/**
	 * Determines whether or not this session has an autonomic manager.
	 * 
	 * @return true if this session has an autonomic manager, false otherwise
	 */
	public boolean hasManager() {
		return (null != this.manager);
	}
	
	/**
	 * Determines whether or not this session equals another session based on
	 * their identifiers.
	 * 
	 * @param o the other session
	 * 
	 * @return true, if the identifier of this session equals the
	 *         identifier of the other session, false otherwise
	 * 
	 * @see Object#equals(Object)
	 */
	@Override
	public final boolean equals(Object o) {
		boolean equals = false;
		
		if (this == o) {
			equals = true;
		} else if ((null != o) && (o instanceof Session)) {
			equals = this.id.equals(((Session) o).id);
		}
	
		return equals;
	}
	
	/**
	 * Gets the hash code of this session based on its identifier.
	 * 
	 * @return the hash code of this session based on its identifier
	 * 
	 * @see Object#hashCode()
	 */
	@Override
	public final int hashCode() {
		return this.id.hashCode();
	}
	
}
