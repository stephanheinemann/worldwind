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
package com.cfar.swim.worldwind.ai;

import java.time.ZonedDateTime;
import java.util.List;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.RiskPolicy;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.Path;

public interface Planner {

	public Aircraft getAircraft();
	public Environment getEnvironment();
	
	public CostPolicy getCostPolicy();
	public void setCostPolicy(CostPolicy costPolicy);
	
	public RiskPolicy getRiskPolicy();
	public void setRiskPolicy(RiskPolicy riskPolicy);
	
	public Path plan(Position origin, Position destination, ZonedDateTime eto);
	public Path plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime eto);

	// TODO: think about applicable heuristics (e.g., Euclidian distance)
	// TODO: minimum ground clearances, altitude restrictions
	// TODO: include capabilities (e.g., velocities, rates) and obtain cost at time
	// TODO: construct grid from globe and positions
	// TODO: think about feasibility and limited deliberation time
	// TODO: think about policies - risk, reward tradeoffs
	// TODO: think about interaction (airspace or obstacle could provide CPDLC interface)
	// TODO: changing altitudes may require CPDLC clearances depending on airspace
	// TODO: computed paths can be associated with symbols (e.g. minimum risk path)

	// TODO: think about required time constraints for waypoints (4D positions)
	// TODO: higher costs might be acceptable to meet timing constraints
}
