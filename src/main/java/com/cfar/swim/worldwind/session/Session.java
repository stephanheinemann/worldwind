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

import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

import com.cfar.swim.worldwind.util.Identifiable;

import gov.nasa.worldwind.Configuration;
import gov.nasa.worldwind.avlist.AVKey;

/**
 * Realizes a planning session that aggregates planning scenarios.
 * 
 * @author Stephan Heinemann
 *
 */
public class Session implements Identifiable {
	
	/** the default session identifier */
	public static final String DEFAULT_SESSION_ID = "Session.Default.Identifier";
	
	/** the identifier of this session */
	private String id;
	
	/** the scenarios of this session */
	private Set<Scenario> scenarios = new HashSet<Scenario>();
	
	/** the default scenario of this session */
	private final Scenario defaultScenario = new Scenario();
	
	/** the active scenario of this session */
	private Scenario activeScenario;
	
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
	 * Sets the identifier of this session.
	 * 
	 * @param id the identifier of this session
	 * 
	 * @see Identifiable#setId(String)
	 */
	@Override
	public void setId(String id) {
		this.id = id;
	}
	
	/**
	 * Initializes this session.
	 */
	public void init() {
		this.scenarios.clear();
		this.scenarios.add(this.defaultScenario);
		this.activeScenario = this.defaultScenario;
		
		Configuration.setValue(
    			AVKey.MIL_STD_2525_ICON_RETRIEVER_PATH,
    			this.getClass().getClassLoader().getResource("milstd2525"));
		// TODO: initialize registries (aircraft, environments, planners...)
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
	 * if present.
	 * 
	 * @param id the scenario identifier
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
	 * Activates a scenario with a specified identifier of this session
	 * if present.
	 * 
	 * @param id the scenario identifier
	 */
	public void setActiveScenario(String id) {
		Optional<Scenario> optScenario = this.scenarios.stream().filter(s -> s.getId().equals(id)).findFirst();
		
		if (optScenario.isPresent()) {
			this.activeScenario = optScenario.get();
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
	 * Sets the active scenario of this session.
	 * The scenario is added if not present.
	 * 
	 * @param scenario the scenario to be activated
	 */
	public void setActiveScenario(Scenario scenario) {
		if (!this.scenarios.contains(scenario)) {
			this.scenarios.add(scenario);
		}
		this.activeScenario = scenario;
	}
	
	/**
	 * Adds a scenario to this session if not present.
	 * 
	 * @param scenario the scenario to be added
	 */
	public void addScenario(Scenario scenario) {
		this.scenarios.add(scenario);
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
			this.scenarios.remove(scenario);
			if (scenario.equals(this.activeScenario)) {
				this.activeScenario = this.defaultScenario;
			}
		}
	}
	
	/**
	 * Indicates whether or not this session equals another session based on
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
	public boolean equals(Object o) {
		boolean equals = false;
		
		if (o instanceof Session) {
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
	public int hashCode() {
		return this.id.hashCode();
	}
}
