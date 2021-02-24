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
package com.cfar.swim.worldwind.ai.adstar;

import java.time.ZonedDateTime;
import java.util.Optional;
import java.util.Set;

import com.cfar.swim.worldwind.ai.DynamicPlanner;
import com.cfar.swim.worldwind.ai.arastar.ARAStarPlanner;
import com.cfar.swim.worldwind.ai.arastar.ARAStarWaypoint;
import com.cfar.swim.worldwind.ai.astar.AStarWaypoint;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.planning.Environment;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;

/**
 * Realizes an Anytime Dynamic A* planner (AD*) that plans a trajectory of
 * an aircraft in an environment considering a local cost and risk policy.
 * 
 * @author Stephan Heinemann
 *
 */
public class ADStarPlanner extends ARAStarPlanner implements DynamicPlanner {

	/**
	 * Constructs an AD* planner for a specified aircraft and environment
	 * using default local cost and risk policies without an initial inflation
	 * applied to the heuristic.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see ARAStarPlanner#ARAStarPlanner(Aircraft, Environment)
	 */
	public ADStarPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}
	
	/**
	 * Creates an AD* waypoint at a specified position.
	 * 
	 * @param position the position
	 * 
	 * @return the AD* waypoint at the specified position
	 */
	@Override
	protected ADStarWaypoint createWaypoint(Position position) {
		ADStarWaypoint adswp = null;
		
		// only create new waypoints if necessary
		if (position.equals(this.getStart())) {
			adswp = this.getStart();
		} else if (position.equals(this.getGoal())) {
			adswp = this.getGoal();
		} else {
			adswp = new ADStarWaypoint(position);
		}
		
		// avoid duplicating discovered waypoints
		Optional<? extends ARAStarWaypoint> existing = super.findExisting(adswp);
		if (existing.isPresent()) {
			adswp = (ADStarWaypoint) existing.get();
		}
		
		// set current inflation factor
		adswp.setEpsilon(this.getInflation());
		return adswp;
	}
	
	/**
	 * Gets the start AD* waypoint of this AD* planner.
	 * 
	 * @return the start AD* waypoint of this AD* planner
	 * 
	 * @see ARAStarPlanner#getStart()
	 */
	@Override
	protected ADStarWaypoint getStart() {
		return (ADStarWaypoint) super.getStart();
	}
	
	/**
	 * Gets the goal AD* waypoint of this AD* planner.
	 * 
	 * @return the goal AD* waypoint of this AD* planner
	 * 
	 * @see ARAStarPlanner#getGoal()
	 */
	@Override
	protected ADStarWaypoint getGoal() {
		return (ADStarWaypoint) super.getGoal();
	}
	
	/**
	 * Determines whether or not AD* waypoints can be expanded.
	 * 
	 * @return true if AD* waypoints can be expanded, false otherwise
	 * 
	 * @see ARAStarPlanner#canExpand()
	 */
	@Override
	protected boolean canExpand() {
		return super.canExpand() || (this.getGoal().isUnderConsistent());
	}
	
	/**
	 * Polls an AD* waypoint from the expandable AD* waypoints.
	 * 
	 * @return the polled AD* waypoint from the expandable AD* waypoints
	 *         if any, null otherwise
	 * 
	 * @see ARAStarWaypoint#pollExpandable()
	 */
	@Override
	protected ADStarWaypoint pollExpandable() {
		return (ADStarWaypoint) super.pollExpandable();
	}
	
	/**
	 * Finds an expandable AD* waypoint.
	 * 
	 * @param waypoint the expandable AD* waypoint to be found
	 * 
	 * @return the found expandable AD* waypoint if any
	 * 
	 * @see ARAStarPlanner#findExpandable(AStarWaypoint)
	 */
	@SuppressWarnings("unchecked")
	@Override
	protected Optional<? extends ADStarWaypoint>
		findExpandable(AStarWaypoint waypoint) {
		return (Optional<ADStarWaypoint>) super.findExpandable(waypoint);
	}
	
	/**
	 * Finds an expanded AD* waypoint.
	 * 
	 * @param waypoint the expanded AD* waypoint to be found
	 * 
	 * @return the found expanded AD* waypoint, if any
	 * 
	 * @see ARAStarPlanner#findExpanded(ARAStarWaypoint)
	 */
	@SuppressWarnings("unchecked")
	@Override
	protected Optional<? extends ADStarWaypoint>
		findExpanded(AStarWaypoint waypoint) {
		return (Optional<ADStarWaypoint>) super.findExpanded(waypoint);
	}
	
	/**
	 * Expands an AD* waypoint towards its neighbors in the environment.
	 * 
	 * @param waypoint the AD* waypoint to be expanded
	 * 
	 * @return the neighbors of the expanded AD* waypoint
	 * 
	 * @see ARAStarPlanner#expand(AStarWaypoint)
	 */
	@SuppressWarnings("unchecked")
	@Override
	protected Set<? extends ADStarWaypoint> expand(AStarWaypoint waypoint) {
		// super.expand implicitly adds the expanded waypoint to the expanded set
		Set<ADStarWaypoint> neighbors = (Set<ADStarWaypoint>) super.expand(waypoint);
		ADStarWaypoint adsw = (ADStarWaypoint) waypoint;
		
		if (adsw.isOverConsistent()) {
			// establish consistency
			adsw.makeConsistent();
		} else {
			// eliminate under-consistency
			adsw.setV(Double.POSITIVE_INFINITY);
			this.removeExpanded(adsw);
			this.updateSets(adsw);
		}
		
		return neighbors;
	}
	
	/**
	 * Updates the planner waypoint sets for an updated AD* waypoint.
	 * 
	 * @param waypoint the updated AD* waypoint
	 * 
	 * @see ARAStarPlanner#updateSets(AStarWaypoint)
	 */
	@Override
	protected void updateSets(AStarWaypoint waypoint) {
		if (((ADStarWaypoint) waypoint).isConsistent()) {
			// consistent waypoints can be removed from sets
			if (this.isExpandable(waypoint)) {
				this.removeExpandable(waypoint);
			} else if (this.isInconsistent((ADStarWaypoint) waypoint)) {
				this.removeInconsistent((ADStarWaypoint) waypoint);
			}
		} else {
			// inconsistent waypoints need to be considered for expansion
			if (!this.isExpanded(waypoint)) {
				// priority queue requires re-insertion for key evaluation
				if (!this.removeExpandable(waypoint)) {
					waypoint.setH(this.getEnvironment()
							.getNormalizedDistance(waypoint, this.getGoal()));
				}
				this.addExpandable(waypoint);
			} else if (!this.isInconsistent((ADStarWaypoint) waypoint)) {
				this.addInconsistent((ADStarWaypoint) waypoint);
			}
		}
	}
	
	/**
	 * Repairs the estimated cost of a specified target AD* waypoint when
	 * reached via a specified source AD* waypoint.
	 * 
	 * @param source the source AD* waypoint in globe coordinates
	 * @param target the target AD* waypoint in globe coordinates
	 */
	protected void repairCost(ADStarWaypoint source, ADStarWaypoint target) {
		Path leg = new Path(source, target);
		Capabilities capabilities = this.getAircraft().getCapabilities();
		Globe globe = this.getEnvironment().getGlobe();
		// TODO: catch IllegalArgumentException (incapable) and exit
		ZonedDateTime end = capabilities.getEstimatedTime(leg, globe, source.getEto());
		
		double cost = this.getEnvironment().getStepCost(
				source, target,
				source.getEto(), end,
				this.getCostPolicy(), this.getRiskPolicy());
		
		if ((source.getV() + cost) < target.getG()) {
			target.setParent(source);
			target.setG(source.getV() + cost);
			target.setEto(end);
		}
	}
	
	/**
	 * Repairs the estimated cost of a specified target AD* waypoint.
	 * 
	 * @param target the target AD* waypoint in globe coordinates
	 */
	protected void repair(ADStarWaypoint target) {
		target.setG(Double.POSITIVE_INFINITY);
		
		for (AStarWaypoint source : target.getParents()) {
			this.repairCost((ADStarWaypoint) source, target);
		}
	}
	
	/**
	 * Computes a plan.
	 * 
	 * @see ARAStarPlanner#compute()
	 */
	@Override
	protected void compute() {
		while (this.canExpand()) {
			ADStarWaypoint source = this.pollExpandable();
			
			Set<? extends ADStarWaypoint> neighbors = this.expand(source);
			
			// source is always inconsistent as per expandable invariant
			if (source.isOverConsistent()) {
				// propagate over-consistency
				for (ADStarWaypoint target : neighbors) {
					this.updateWaypoint(source, target);
				}
			} else {
				// propagate under-consistency
				for (ADStarWaypoint target : neighbors) {
					// find target that depends on under-consistent source
					if (target.getParent().equals(source)) {
						this.repair(target);
						this.updateSets(target);
					}
				}
			}
		}
		
		this.connectPlan(this.getGoal());
	}
	
}
