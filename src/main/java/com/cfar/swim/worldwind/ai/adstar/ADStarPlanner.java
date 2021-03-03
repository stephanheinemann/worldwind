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
import java.util.List;
import java.util.Optional;
import java.util.Set;

import com.cfar.swim.worldwind.ai.DynamicPlanner;
import com.cfar.swim.worldwind.ai.arastar.ARAStarPlanner;
import com.cfar.swim.worldwind.ai.arastar.ARAStarWaypoint;
import com.cfar.swim.worldwind.ai.astar.AStarWaypoint;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.Trajectory;

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

	/** indicates whether or or not this AD* planner has terminated */
	volatile private boolean terminated = false;
	
	/** indicates whether or or not this AD* planner has a significant dynamic change */
	private boolean hasSignificantChange = false;
	
	/** the significant change threshold of this AD* planner */
	private double significantChange = 0.25d;
	
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
		
		/*
		 * Avoid visiting existing positions although a 4D waypoint may very
		 * well revisit an existing position to avoid higher costs. The
		 * computed trajectories shall however be space-optimal but not
		 * necessarily time-optimal. To mitigate this issue, departure slots,
		 * or more generally, waypoint slots could be considered and take into
		 * account the aircraft capabilities appropriately (endurance). This
		 * could realize the concept of holding or loitering.
		 * 
		 * https://github.com/stephanheinemann/worldwind/issues/24
		 */
		
		// only create new waypoints if necessary
		if (this.hasStart() && this.getStart().equals(position)) {
			adswp = this.getStart();
		} else if (this.hasGoal() && this.getGoal().equals(position)) {
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
		return (Set<ADStarWaypoint>) super.expand(waypoint);
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
		
		// consider expansion cost and break ties with travel time
		boolean improvedCost = (source.getV() + cost) < target.getG();
		boolean equalCost = (source.getV() + cost) == target.getG();
		boolean improvedTime = (target.hasEto() && end.isBefore(target.getEto()));
		
		if (improvedCost || (equalCost && improvedTime)) {
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
	 * Computes an AD* plan.
	 * 
	 * @see ARAStarPlanner#compute()
	 */
	@Override
	protected void compute() {
		System.out.println("compute");
		while (this.canExpand()) {
			ADStarWaypoint source = this.pollExpandable();
			
			System.out.println("expanding");
			Set<? extends ADStarWaypoint> neighbors = this.expand(source);
			
			System.out.println(source.getDesignator() + ": G = " + source.getG() + " V = " + source.getV());
			
			// source is always inconsistent as per expandable invariant
			if (source.isOverConsistent()) {
				System.out.println("over-consistent");
				// establish consistency
				source.makeConsistent();
				// propagate over-consistency
				for (ADStarWaypoint target : neighbors) {
					this.updateWaypoint(source, target);
				}
			} else {
				System.out.println("under-consistent");
				// eliminate under-consistency
				source.setV(Double.POSITIVE_INFINITY);
				// expand implicitly adds the expanded waypoint to the expanded set
				this.removeExpanded(source);
				this.updateSets(source);
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
	
	/**
	 * Terminates this AD* planner.
	 * 
	 * @see DynamicPlanner#terminate()
	 */
	@Override
	public void terminate() {
		this.terminated = true;
	}
	
	/**
	 * Indicates whether or not this AD* planner has terminated.
	 * 
	 * @return true if this AD* planner has terminated, false otherwise
	 * 
	 * @see DynamicPlanner#hasTerminated()
	 */
	@Override
	public boolean hasTerminated() {
		return this.terminated;
	}
	
	/**
	 * Repairs an AD* plan in case of dynamic changes.
	 */
	protected void repair() {
		// TODO: determine and repair affected legs and target waypoints
		ADStarWaypoint target = this.getStart();
		if (!this.getStart().equals(target)) {
			this.repair(target);
			this.updateSets(target);
		}
	}
	
	/**
	 * Sets the significant change threshold of this AD* planner.
	 * 
	 * @param significantChange the signficant change threshold of this AD*
	 *                          planner
	 */
	public void setSignificantChange(double significantChange) {
		this.significantChange = significantChange;
	}
	
	/**
	 * Indicates whether or or not this AD* planner has a significant dynamic
	 * change.
	 * 
	 * @return true if this AD* planner has a significant dynamic change,
	 *         false otherwise
	 *
	 * @see DynamicPlanner#hasSignificantChange()
	 */
	@Override
	public boolean hasSignificantChange() {
		return this.hasSignificantChange;
	}
	
	/**
	 * Improves an AD* plan incrementally.
	 */
	@Override
	protected void improve(int partIndex) {
		this.backup(partIndex);

		// determine significant change
		if (this.hasSignificantChange()) {
			// TODO: implement significant change strategy
			// increase e or re-plan from scratch
		} else if (!this.isDeflated()) {
			this.deflate();
		}
		
		this.restore(partIndex);
		
		this.compute();
		this.revisePlan(this.createTrajectory());
	}
	
	/**
	 * Elaborates an AD* plan
	 * 
	 */
	@Override
	protected void elaborate(int partIndex) {
		while (!this.isDeflated() && !this.hasTerminated()) {
			this.repair();
			this.improve(partIndex);
		}
	}
	
	@Override
	protected Trajectory planPart(Position origin, Position destination, ZonedDateTime etd, int partIndex) {
		// TODO: if index affected by change (and all subsequent indices)
		// if (this.isAffected) {
		return super.planPart(origin, destination, etd, partIndex);
		//}
	}
	
	/**
	 * Plans a trajectory from an origin to a destination at a specified
	 * estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @return the planned trajectory from the origin to the destination with
	 *         the estimated time of departure
	 * 
	 * @see ARAStarPlanner#plan(Position, Position, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		this.initBackups(1);
		Trajectory trajectory = new Trajectory();
		
		while (!this.hasTerminated()) {
			trajectory = this.planPart(origin, destination, etd, 0);
			// TODO: wait for changes here
			/*
			try {
				this.wait();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			*/
			this.terminate();
		}
		
		this.revisePlan(trajectory);
		return trajectory;
	}
	
	/**
	 * Plans a trajectory from an origin to a destination along waypoints at a
	 * specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param waypoints the waypoints in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @return the planned trajectory from the origin to the destination along
	 *         the waypoints with the estimated time of departure
	 * 
	 * @see ARAStarPlanner#plan(Position, Position, List, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
		Trajectory trajectory = new Trajectory();
		
		while (!this.hasTerminated()) {
			trajectory = super.plan(origin, destination, waypoints, etd);
			// TODO: wait for changes here (iterating all parts or just from the affected)
			/*
			try {
				this.wait();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			*/
			this.terminate();
		}
		
		return trajectory;
	}
	
	protected void suspend() {}
	
	protected void resume() {}
	
}
