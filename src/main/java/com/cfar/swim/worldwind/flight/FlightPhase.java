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
package com.cfar.swim.worldwind.flight;

import java.util.HashSet;
import java.util.Set;

import com.cfar.swim.worldwind.managing.Criticality;
import com.cfar.swim.worldwind.managing.Difficulty;
import com.cfar.swim.worldwind.managing.Features;
import com.cfar.swim.worldwind.managing.Severity;

/**
 * Realizes a flight phase that covers certain difficulties and severities.
 * 
 * @author Stephan Heinemann
 *
 * @see Difficulty
 * @see Severity
 * @see Criticality
 */
public class FlightPhase extends Criticality {
	
	/** the possible difficulties of this flight phase */
	private final Set<Difficulty> difficulties = new HashSet<>();
	
	/** the possible severities of this flight phase */
	private final Set<Severity> severities = new HashSet<>();
	
	/**
	 * Constructs a new flight phase limiting only POI distance ranges.
	 * 
	 * @param features the features containing the POI distances
	 * 
	 * @see Criticality#Criticality(Features)
	 */
	public FlightPhase(Features features) {
		super(features);
	}
	
	/*				Difficulties	Severities
	 * Cruise		*				LOW, MOD
	 * Transition	LOW, MOD, SUB	SUB, SEV
	 * Terminal		SEV, CRT        SUB, SEV
	 * Urgency		*				CRT
	 * Emergency	*				FAT
	 */
	
	/**
	 * Creates a cruise flight phase based on features.
	 * 
	 * @param features the features
	 * 
	 * @return a cruise flight phase based on features
	 */
	public static FlightPhase createCruise(Features features) {
		FlightPhase cruise = new FlightPhase(features);
		
		cruise.difficulties.add(Difficulty.createLow(features));
		cruise.difficulties.add(Difficulty.createModerate(features));
		cruise.difficulties.add(Difficulty.createSubstantial(features));
		cruise.difficulties.add(Difficulty.createSevere(features));
		cruise.difficulties.add(Difficulty.createCritical(features));
		cruise.severities.add(Severity.createLow(features));
		cruise.severities.add(Severity.createModerate(features));
		
		return cruise;
	}
	
	/**
	 * Creates a transition (departure, arrival) flight phase based on
	 * features.
	 * 
	 * @param features the features
	 * 
	 * @return a transition flight phase based on features
	 */
	public static FlightPhase createTransition(Features features) {
		FlightPhase transition = new FlightPhase(features);
		
		transition.difficulties.add(Difficulty.createLow(features));
		transition.difficulties.add(Difficulty.createModerate(features));
		transition.difficulties.add(Difficulty.createSubstantial(features));
		transition.severities.add(Severity.createSubstantial(features));
		transition.severities.add(Severity.createSevere(features));
		
		return transition;
	}
	
	/**
	 * Creates a terminal (aerodrome) flight phase based on
	 * features.
	 * 
	 * @param features the features
	 * 
	 * @return a terminal flight phase based on features
	 */
	public static FlightPhase createTerminal(Features features) {
		FlightPhase terminal = new FlightPhase(features);
		
		terminal.difficulties.add(Difficulty.createSevere(features));
		terminal.difficulties.add(Difficulty.createCritical(features));
		terminal.severities.add(Severity.createSubstantial(features));
		terminal.severities.add(Severity.createSevere(features));
		
		return terminal;
	}
	
	/**
	 * Creates an urgency flight phase based on features.
	 * 
	 * @param features the features
	 * 
	 * @return an urgency flight phase based on features
	 */
	public static FlightPhase createUrgency(Features features) {
		FlightPhase urgency = new FlightPhase(features);
		
		urgency.difficulties.add(Difficulty.createLow(features));
		urgency.difficulties.add(Difficulty.createModerate(features));
		urgency.difficulties.add(Difficulty.createSubstantial(features));
		urgency.difficulties.add(Difficulty.createSevere(features));
		urgency.difficulties.add(Difficulty.createCritical(features));
		urgency.severities.add(Severity.createCritical(features));
		urgency.severities.add(Severity.createSevere(features));
		
		return urgency;
	}
	
	/**
	 * Creates an emergency flight phase based on features.
	 * 
	 * @param features the features
	 * 
	 * @return an emergency flight phase based on features
	 */
	public static FlightPhase createEmergency(Features features) {
		FlightPhase emergency = new FlightPhase(features);
		
		emergency.difficulties.add(Difficulty.createLow(features));
		emergency.difficulties.add(Difficulty.createModerate(features));
		emergency.difficulties.add(Difficulty.createSubstantial(features));
		emergency.difficulties.add(Difficulty.createSevere(features));
		emergency.difficulties.add(Difficulty.createCritical(features));
		emergency.severities.add(Severity.createFatal(features));
		
		return emergency;
	}
	
	/**
	 * Determines whether or not this flight phase covers given features.
	 * 
	 * @param features the features
	 * 
	 * @return true if this flight phase covers the features, false otherwise
	 * 
	 * @see Criticality#covers(Features)
	 */
	public boolean covers(Features features) {
		boolean covers = super.covers(features);
		
		if (covers) {
			covers = (0 < this.severities.stream()
					.filter(s -> s.covers(features)).count())
					&& (0 < this.difficulties.stream()
					.filter(d -> d.covers(features)).count());
		}
		
		return covers;
	}
	
}
