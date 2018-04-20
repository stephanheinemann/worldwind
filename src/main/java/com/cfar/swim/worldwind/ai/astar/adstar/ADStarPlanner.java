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
package com.cfar.swim.worldwind.ai.astar.adstar;

import java.util.Optional;
import java.util.Set;

import com.cfar.swim.worldwind.ai.astar.arastar.ARAStarPlanner;
import com.cfar.swim.worldwind.ai.astar.arastar.ARAStarWaypoint;
import com.cfar.swim.worldwind.ai.astar.astar.AStarWaypoint;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Environment;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes an Anytime Dynamic A* planner (AD*) that plans a trajectory of
 * an aircraft in an environment considering a local cost and risk policy.
 * 
 * @author Stephan Heinemann
 *
 */
public class ADStarPlanner extends ARAStarPlanner {

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
		return new ADStarWaypoint(position);
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
	 * @see ARAStarPlanner#pollExpandable()
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
	 * Finds the dependent target of an expanded AD* source waypoint.
	 * The source waypoint is the parent of the target waypoint.
	 *  
	 * @param source the source AD* waypoint
	 * @param target the dependent target AD* waypoint to be found
	 * @return the found dependent target AD* waypoint, if any
	 */
	@SuppressWarnings("unchecked")
	@Override
	protected Optional<? extends ADStarWaypoint>
		findDependent(AStarWaypoint source, AStarWaypoint target) {
		return (Optional<ADStarWaypoint>) super.findDependent(source, target);
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
	
	@Override
	protected void updateSets(AStarWaypoint waypoint) {
		// TODO: finding the expandable is actually not required since no
		// updates to it are necessary, isExpandable should be sufficient
		// check precisely which waypoints should be retained and moved
		// between the sets!
		Optional<? extends ADStarWaypoint> expandable =
				this.findExpandable(waypoint);
		
		if (((ADStarWaypoint) waypoint).isConsistent()) {
			if (expandable.isPresent()) {
				this.removeExpandable(expandable.get());
			} else if (this.isInconsistent((ADStarWaypoint) waypoint)) {
				this.removeInconsistent((ADStarWaypoint) waypoint);
			}
		} else {
			if (!this.isExpanded(waypoint)) {
				if (expandable.isPresent()) {
					this.removeExpandable(waypoint);
					this.addExpandable(waypoint);
				} else {
					this.addExpandable(waypoint);
				}
			} else if (this.isInconsistent((ADStarWaypoint) waypoint)) {
				this.addInconsistent((ADStarWaypoint) waypoint);
			}
		}
	}
	
	protected void repairCost(ADStarWaypoint waypoint) {
		// TODO: Fig. 18 (lines 25/26)
		// TODO: find neighbors including start if in start region
		// TODO: identify minimum v-cost neighbor if any (expandable, expanded)
		// TODO: repair g-cost based on minimum v-cost and update parent
		// TODO: maybe visited predecessors and successors should be stored...
		
		Set<Position> neighbors = this.getDiscreteEnvironment().getNeighbors(waypoint);
		
		// add start to the start region
		if (this.isInStartRegion(waypoint)) {
			neighbors.add(this.getStart());
		}
		
		// add goal to the goal region
		if (this.isInGoalRegion(waypoint)) {
			neighbors.add(this.getGoal());
		}
		
		for (Position neighbor : neighbors) {
			ADStarWaypoint adsw = this.createWaypoint(neighbor);
			Optional<? extends ADStarWaypoint> expandable =
					this.findExpandable(adsw);
		}
		
		// TODO: only incons gets backed up, parents have to be considered too
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
			
			if (source.equals(this.getGoal())) {
				this.connectPlan(source);
				return;
			}
			
			Set<? extends ADStarWaypoint> neighbors = this.expand(source);
			
			if (source.isOverConsistent()) {
				// propagate over-consistency
				for (ADStarWaypoint target : neighbors) {
					this.updateWaypoint(source, target);
				}
			} else {
				// propagate under-consistency
				for (ADStarWaypoint target : neighbors) {
					// find target that depends on under-consistent source
					Optional<? extends ADStarWaypoint> dependent =
							this.findDependent(source, target);
					
					if (dependent.isPresent()) {
						this.repairCost(dependent.get());
						this.updateSets(dependent.get());
					}
				}
			}
		}
	}
	
}
