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
package com.cfar.swim.worldwind.planners.cgs.thetastar;

import java.time.ZonedDateTime;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.planners.cgs.astar.AStarWaypoint;
import com.cfar.swim.worldwind.planners.cgs.astar.ForwardAStarPlanner;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.util.Identifiable;

import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;


/**
 * Realizes a Theta* planner that plans a trajectory of an aircraft
 * in an environment considering a local cost and risk policy. Being an
 * any-angle planner, the planner removes some restrictions imposed by the
 * discrete environment by pulling strings and avoiding unnecessary heading
 * changes.
 * 
 * @author Stephan Heinemann
 *
 */
public class ThetaStarPlanner extends ForwardAStarPlanner {

	/**
	 * Constructs a basic Theta* planner for a specified aircraft and
	 * environment using default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 */
	public ThetaStarPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}
	
	/**
	 * Gets the identifier of this Theta* planner.
	 * 
	 * @return the identifier of this Theta* planner
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return Specification.PLANNER_TS_ID;
	}
	
	/**
	 * Computes the estimated cost of a specified target waypoint when reached
	 * via a specified source waypoint. A straight connection between the
	 * parent of the specified source waypoint and the specified target
	 * waypoint is also considered to create an any-angle path.
	 * 
	 * @param source the source waypoint in globe coordinates
	 * @param target the target waypoint in globe coordinates
	 */
	@Override
	protected void computeCost(AStarWaypoint source, AStarWaypoint target) {
		Path leg = new Path(source, target);
		Capabilities capabilities = this.getAircraft().getCapabilities();
		Globe globe = this.getEnvironment().getGlobe();
		// TODO: catch CapabilitiesException (incapable) and exit
		ZonedDateTime end = capabilities.getEstimatedTime(leg, globe, source.getEto());
		
		AStarWaypoint parent = null;
		double straightCost = Double.POSITIVE_INFINITY;
		double straightTargetG = Double.POSITIVE_INFINITY;
		ZonedDateTime straightEnd = null;
		
		// attempt to plan a straight leg from source's parent to target
		if (null != source.getParent()) {
			parent = source.getParent();
			Path straightLeg = new Path(parent, target);
			// TODO: catch CapabilitiesException (incapable) and exit
			straightEnd = capabilities.getEstimatedTime(straightLeg, globe, parent.getEto());
			
			straightCost = this.getEnvironment().getLegCost(
				parent, target,
				parent.getEto(), straightEnd,
				this.getCostPolicy(), this.getRiskPolicy());
			straightTargetG = parent.getG() + straightCost;
		}
			
		double cost = this.getEnvironment().getStepCost(
				source, target,
				source.getEto(), end,
				this.getCostPolicy(), this.getRiskPolicy());
		double targetG = source.getG() + cost;
		
		// consider expansion cost and break ties with travel time
		boolean straight = (straightTargetG < targetG);
		boolean improvedStraightCost = (straightTargetG < target.getG());
		boolean equalStraightCost = (straightTargetG == target.getG());
		boolean improvedStraightTime = (target.hasEto() && straightEnd.isBefore(target.getEto()));
		boolean improvedCost = (targetG < target.getG());
		boolean equalCost = (targetG == target.getG());
		boolean improvedTime = (target.hasEto() && end.isBefore(target.getEto()));
		
		// take the better of the two trajectories
		if (straight && (improvedStraightCost || (equalStraightCost && improvedStraightTime))) {
			target.setParent(parent);
			target.setG(straightTargetG);
			target.setEto(straightEnd);
		} else if (improvedCost || (equalCost && improvedTime)) {
			target.setParent(source);
			target.setG(targetG);
			target.setEto(end);
		}
	}
	
}
