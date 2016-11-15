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
import java.util.Set;

import com.cfar.swim.worldwind.util.Identifiable;

import gov.nasa.worldwind.Configuration;
import gov.nasa.worldwind.avlist.AVKey;

public class Session implements Identifiable {
	
	// TODO: equals, hashCode (id)
	
	public static final String DEFAULT_SESSION_ID = "default";
	
	private String id;
	
	private Set<Scenario> scenarios = new HashSet<Scenario>();
	
	public Session() {
		this(Session.DEFAULT_SESSION_ID);
	}
	
	public Session(String id) {
		this.id = id;
		this.scenarios.add(new Scenario());
		this.init();
	}
	
	@Override
	public String getId() {
		return this.id;
	}
	
	@Override
	public void setId(String id) {
		this.id = id;
	}
	
	private void init() {
		Configuration.setValue(
    			AVKey.MIL_STD_2525_ICON_RETRIEVER_PATH,
    			this.getClass().getClassLoader().getResource("milstd2525"));
		// TODO: initialize registries
	}
	
	public Scenario getDefaultScenario() {
		return this.scenarios.stream().filter(s -> s.getId().equals(Scenario.DEFAULT_SCENARIO_ID)).findFirst().get();
	}
	
	public Scenario getScenario(String id) {
		return this.scenarios.stream().filter(s -> s.getId().equals(id)).findFirst().get();
	}
	
	public void addScenario(Scenario scenario) {
		this.scenarios.add(scenario);
	}

}
