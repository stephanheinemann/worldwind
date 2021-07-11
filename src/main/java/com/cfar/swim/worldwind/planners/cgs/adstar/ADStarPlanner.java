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
package com.cfar.swim.worldwind.planners.cgs.adstar;

import java.time.ZonedDateTime;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.environments.DynamicEnvironment;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.planners.AbstractPlanner;
import com.cfar.swim.worldwind.planners.DynamicObstacleListener;
import com.cfar.swim.worldwind.planners.DynamicPlanner;
import com.cfar.swim.worldwind.planners.LifelongPlanner;
import com.cfar.swim.worldwind.planners.cgs.arastar.ARAStarPlanner;
import com.cfar.swim.worldwind.planners.cgs.astar.AStarWaypoint;
import com.cfar.swim.worldwind.planning.TimeInterval;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.planners.cgs.ADStarProperties;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.session.ObstacleManager;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;

/**
 * Realizes an Anytime Dynamic A* planner (AD*) that plans a trajectory of
 * an aircraft in an environment considering a local cost and risk policy.
 * The planner can cope with partial environment knowledge efficiently
 * improving, repairing and revising plans accordingly.
 * 
 * @author Stephan Heinemann
 *
 */
public class ADStarPlanner extends ARAStarPlanner
implements DynamicPlanner, LifelongPlanner {
	
	/** indicates whether or not this AD* planner has terminated */
	private boolean terminated = false;
	
	/** indicates whether or not this AD* planning is listening */
	private boolean isListening = false;
	
	/** the obstacle manager of this AD* planner */
	private ObstacleManager obstacleManager = null;
	
	/** the dynamic obstacles of an obstacle commitment by this AD* planner */
	private Set<Obstacle> dynamicObstacles = new HashSet<>();
		
	/** the significant change threshold of this AD* planner */
	private double significantChange = 0.5d;
	
	/** the actual change of this AD* planner */
	private double change = 0d;
	
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
		Optional<? extends AStarWaypoint> visitedWaypoint = this.findVisited(adswp);
		if (visitedWaypoint.isPresent()) {
			adswp = (ADStarWaypoint) visitedWaypoint.get();
		} else {
			this.addVisited(adswp);
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
	 * @see ARAStarPlanner#findExpanded(AStarWaypoint)
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
	 * Clears the dynamic obstacles of this AD* planner.
	 */
	protected void clearDynamicObstacles() {
		this.dynamicObstacles.clear();
	}
	
	/**
	 * Adds dynamic obstacles to this AD* planner.
	 * 
	 * @param dyamicObstacles the dynamic obstacles to be added
	 */
	protected void addDynamicObstacles(Collection<Obstacle> dyamicObstacles) {
		this.dynamicObstacles.addAll(dyamicObstacles);
	}
	
	/**
	 * Gets the dynamic obstacles of this AD* planner.
	 * 
	 * @return the dynamic obstacles of this AD* planner
	 */
	protected Iterable<Obstacle> getDynamicObstacles() {
		return this.dynamicObstacles;
	}
	
	/**
	 * Determines whether or not this AD* planner has dynamic obstacles.
	 * 
	 * @return true if this AD* planner has dynamic obstacles, false otherwise
	 */
	protected boolean hasDynamicObstacles() {
		return !this.dynamicObstacles.isEmpty();
	}
	
	/**
	 * Initializes this AD* planner to plan from an origin to a destination at
	 * a specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @see ARAStarPlanner#initialize(Position, Position, ZonedDateTime)
	 */
	@Override
	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {
		super.initialize(origin, destination, etd);
		this.clearDynamicObstacles();
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
	 * Finds the waypoints that are affected by the dynamic obstacles of this
	 * AD* planner.
	 * 
	 * @return the waypoints that are affected by the dynamic obstacles of this
	 *         AD* planner
	 */
	protected Set<ADStarWaypoint> findAffectedWaypoints() {
		Set<ADStarWaypoint> affectedWaypoints = new HashSet<>();
		
		for (Obstacle obstacle : this.getDynamicObstacles()) {
			affectedWaypoints.addAll(
					((DynamicEnvironment) this.getEnvironment())
					.getAffectedWaypointPositions(obstacle).stream()
					.map(position -> this.createWaypoint(position))
					.filter(waypoint -> waypoint.hasEto() && waypoint.hasParent())
					.filter(waypoint -> obstacle.getCostInterval().intersects(
							new TimeInterval(
									waypoint.getParent().getEto(),
									waypoint.getEto())))
					.collect(Collectors.toSet()));
		}
		this.clearDynamicObstacles();
		
		return affectedWaypoints;
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
		// TODO: catch CapabilitiesException (incapable) and exit
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
	 * Determines whether or not an AD* plan needs a potential repair.
	 * 
	 * @return true if the AD* plan needs a potential repair, false otherwise
	 */
	protected boolean needsRepair() {
		if (this.hasObstacleManager() && this.obstacleManager.hasObstacleChange()) {
			Set<Obstacle> dynamicObstacles = this.obstacleManager.commitObstacleChange();
			this.addDynamicObstacles(dynamicObstacles);
			this.shareDynamicObstacles(dynamicObstacles);
		}
		
		return this.hasDynamicObstacles();
	}
	
	/**
	 * Repairs an AD* plan in case of dynamic changes.
	 * 
	 * @param partIndex the index of the part to be repaired
	 */
	protected void repair(int partIndex) {
		if (this.needsRepair()) {
			ADStarWaypoint partStart = (ADStarWaypoint) this.getStart().clone();
			
			// repair previous parts before current part
			if (this.hasDynamicObstacles(partIndex - 1)) {
				this.backup(partIndex);
				double partInflation = this.getInflation();
				this.setInflation(this.getMaximumQuality());
				this.restore(partIndex -1);
				this.planPart(partIndex - 1);
				// TODO: what if no plan could be found
				partStart.setEto(this.getGoal().getEto());
				partStart.setParent(this.getGoal().getParent());
				this.backup(partIndex - 1);
				this.setInflation(partInflation);
				this.restore(partIndex);
			}
			
			// TODO: consider departure slots to avoid planning from scratch
			// TODO: consider early arrivals with holding / loitering
			if (this.getStart().getEto().equals(partStart.getEto())) {
				// connect current to previous part
				if (partStart.hasParent()) {
					this.getStart().setParent(partStart.getParent());
				}
				
				// repair affected waypoints for current part
				for (ADStarWaypoint target : this.findAffectedWaypoints()) {
					if (!this.getStart().equals(target)) {
						this.repair(target);
						this.updateSets(target);
					}
				}
				
				// force improvement in case of dynamic removals
				//this.setInflation(this.getMaximumQuality() + this.getQualityImprovement());
				
			} else {
				// plan current part from scratch if start ETO has changed
				this.initialize(this.getStart(), this.getGoal(), partStart.getEto());
				if (partStart.hasParent()) {
					this.getStart().setParent(partStart.getParent());
				}
				super.planPart(partIndex);
			}
		}
	}
	
	/**
	 * Computes an AD* plan.
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
				// establish consistency
				source.makeConsistent();
				// propagate over-consistency
				for (ADStarWaypoint target : neighbors) {
					this.updateWaypoint(source, target);
				}
			} else {
				// eliminate under-consistency
				source.makeUndetermined();
				// expanding implicitly adds the expanded waypoint to the expanded set
				this.removeExpanded(source);
				this.updateSets(source);
				// propagate under-consistency
				for (ADStarWaypoint target : neighbors) {
					// find targets that depend on under-consistent source
					if (target.hasParent() && target.getParent().equals(source)) {
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
	 * @see LifelongPlanner#terminate()
	 */
	@Override
	public synchronized void terminate() {
		this.terminated = true;
		this.notifyAll();
	}
	
	/**
	 * Recycles this AD* planner.
	 * 
	 * @see LifelongPlanner#recycle()
	 */
	@Override
	public synchronized void recycle() {
		this.terminated = false;
	}
	
	/**
	 * Indicates whether or not this AD* planner has terminated.
	 * 
	 * @return true if this AD* planner has terminated, false otherwise
	 * 
	 * @see LifelongPlanner#hasTerminated()
	 */
	@Override
	public synchronized boolean hasTerminated() {
		return this.terminated;
	}
	
	/**
	 * Gets the significant change threshold of this AD* planner.
	 * 
	 * @return the significant change threshold of this AD* planner.
	 * 
	 * @see DynamicPlanner#getSignificantChange()
	 */
	@Override
	public double getSignificantChange() {
		return this.significantChange;
	}
	
	/**
	 * Sets the significant change threshold of this AD* planner.
	 * 
	 * @param significantChange the significant change threshold to be set
	 * 
	 * @see DynamicPlanner#setSignificantChange(double)
	 * 
	 * @throws IllegalArgumentException if significant change threshold is
	 *                                  invalid
	 */
	@Override
	public void setSignificantChange(double significantChange) {
		if ((0d <= significantChange) && (1d >= significantChange)) {
			this.significantChange = significantChange;
		} else {
			throw new IllegalArgumentException("significant change is invalid");
		}
	}
	
	/**
	 * Determines whether or or not this AD* planner has a significant dynamic
	 * change.
	 * 
	 * @return true if this AD* planner has a significant dynamic change,
	 *         false otherwise
	 */
	protected boolean hasSignificantChange() {
		boolean hasSignificantChange = false;
		// TODO: examine different determination policies including
		// (1) consistent, under-consistent, over-consistent waypoints
		// (2) inconsistencies close to start versus close to goal (ETOs)
		// (3) even if the current trajectory is not directly affected,
		//     the surrounding environment could allow for improvements
		
		// the amount of waypoints in the current plan
		int wc = this.getWaypoints().size();
		// the amount of inconsistent waypoints in the current plan
		long ic = this.getWaypoints().stream()
			.filter(waypoint -> !((ADStarWaypoint) waypoint).isConsistent())
			.count();
		// the ratio of inconsistent to all waypoints in the current plan
		if (0 != wc) {
			this.change = ((double) ic) / wc;
			hasSignificantChange = (this.change >= this.significantChange);
		}
		
		return hasSignificantChange;
	}
	
	/**
	 * Increases the current inflation depending on the actual dynamic change.
	 */
	protected void inflate() {
		// TODO: consider different significant change handling strategies
		// increase epsilon or re-plan from scratch
		double qualityRange = this.getMinimumQuality() - this.getMaximumQuality();
		double qualityAdjustment = this.getMaximumQuality() + (this.change * qualityRange);
		
		if (this.getInflation() < qualityAdjustment) {
			this.setInflation(qualityAdjustment);
		}
	}
	
	/**
	 * Improves an AD* plan incrementally.
	 * 
	 * @param partIndex the index of the plan to be improved
	 */
	@Override
	protected void improve(int partIndex) {
		this.backup(partIndex);

		// determine significant change
		if (this.hasSignificantChange()) {
			// inflate to lower quality or re-plan from scratch
			this.inflate();
		} else if (!this.isDeflated()) {
			// deflate to higher quality and continue improving
			this.deflate();
		}
		
		this.restore(partIndex);
		
		this.compute();
		this.revisePlan(this.createTrajectory());
	}
	
	/**
	 * Elaborates an AD* plan.
	 * 
	 * @param partIndex the index of the plan to be elaborated
	 */
	@Override
	protected void elaborate(int partIndex) {
		// proceed to next part only if fully deflated and not in need of repair
		while ((!this.hasMaximumQuality() || this.needsRepair()) && !this.hasTerminated()) {
			this.repair(partIndex);
			this.improve(partIndex);
		}
		// always backup at least once for potential repair later
		this.backup(partIndex);
	}
	
	/**
	 * Plans a part of a trajectory. Repairs and improves the planned part
	 * incrementally if required until a maximum quality has been achieved.
	 * 
	 * @param partIndex the index of the part
	 * 
	 * @return the planned part of a trajectory
	 */ 
	@Override
	protected Trajectory planPart(int partIndex) {
		if (this.hasBackup(partIndex)) {
			// repair and improve existing plan after dynamic changes
			this.elaborate(partIndex);
			return this.createTrajectory();
		} else {
			// plan from scratch
			return super.planPart(partIndex);
		}
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
		Trajectory trajectory = new Trajectory();
		this.initBackups(1);
		this.initialize(origin, destination, etd);
		
		this.setListening(true);
		while (!this.hasTerminated()) {
			trajectory = this.planPart(0);
			this.revisePlan(trajectory);
			// wait for termination or dynamic changes
			this.suspend();
		}
		this.setListening(false);
		
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
		this.initBackups(waypoints.size() + 1);
		
		this.setListening(true);
		while (!this.hasTerminated()) {
			if (this.hasBackup(waypoints.size())) {
				this.elaborate(waypoints.size());
				trajectory = this.createTrajectory();
			} else {
				trajectory = super.plan(origin, destination, waypoints, etd);
			}
			// wait for termination or dynamic changes
			this.suspend();
		}
		this.setListening(false);
		
		return trajectory;
	}
	
	/**
	 * Suspends this AD* planner until termination or context changes occur.
	 */
	protected synchronized void suspend() {
		try {
			this.wait();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * Sets whether or not this AD* planner is listening.
	 * 
	 * @param isListening the listening state to be set
	 */
	protected synchronized void setListening(boolean isListening) {
		this.isListening = isListening;
	}
	
	/**
	 * Determines whether or not this AD* planner is listening.
	 * 
	 * @return true if this AD* planning is listening,
	 *         false otherwise
	 *
	 * @see DynamicObstacleListener#isListening()
	 */
	@Override
	public synchronized boolean isListening() {
		return this.isListening;
	}
	
	/**
	 * Notifies this AD* planner about a pending obstacle change.
	 *
	 * @see DynamicObstacleListener#notifyPendingObstacleChange()
	 */
	@Override
	public synchronized void notifyPendingObstacleChange() {
		this.notifyAll();
	}

	/**
	 * Sets the obstacle manager of this AD* planner.
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
	 * Determines whether or not this AD* planner has an obstacle manager.
	 * 
	 * @return true if this AD* planner has an obstacle manager,
	 *         false otherwise
	 * 
	 * @see DynamicObstacleListener#hasObstacleManager()
	 */
	@Override
	public synchronized boolean hasObstacleManager() {
		return (null != this.obstacleManager);
	}
	
	/**
	 * Realizes a backup of this AD* planner.
	 * 
	 * @author Stephan Heinemann
	 * 
	 * @see ARAStarPlanner.Backup
	 */
	protected class Backup extends ARAStarPlanner.Backup {
		
		/** the dynamic obstacles of this AD* backup */
		public Set<Obstacle> dynamicObstacles = new HashSet<>();
		
		/**
		 * Clears this AD* backup.
		 * 
		 * @see ARAStarPlanner.Backup#clear()
		 */
		@Override
		public void clear() {
			super.clear();
			this.dynamicObstacles.clear();
		}
		
		/**
		 * Determines whether or not this AD* backup is empty.
		 * 
		 * @return true if this AD* backup is empty, false otherwise
		 * 
		 * @see ARAStarPlanner.Backup#isEmpty()
		 */
		@Override
		public boolean isEmpty() {
			return super.isEmpty() && this.dynamicObstacles.isEmpty();
		}
	}
	
	/**
	 * Initializes a number backups for this AD* planner.
	 * 
	 * @param size the number of backups for this AD* planner
	 */
	@Override
	protected void initBackups(int size) {
		this.backups.clear();
		for (int backupIndex = 0; backupIndex < size; backupIndex++) {
			this.backups.add(backupIndex, new Backup());
		}
	}
	
	/**
	 * Backs up this AD* planner for incremental improvement and dynamic repair.
	 * 
	 * @param backupIndex the index of the backup
	 * 
	 * @return true if a backup has been performed, false otherwise
	 */
	@Override
	protected boolean backup(int backupIndex) {
		boolean backedup = super.backup(backupIndex);
		
		if (backedup) {
			Backup backup = (Backup) this.backups.get(backupIndex);
			backedup = backup.dynamicObstacles.addAll(this.dynamicObstacles);
		}
		
		return backedup;
	}
	
	/**
	 * Restores this AD* planner for incremental improvement and dynamic repair.
	 * 
	 * @param backupIndex the index of the backup
	 * 
	 * @return true if a restore has been performed, false otherwise
	 */
	@Override
	protected boolean restore(int backupIndex) {
		boolean restored = super.restore(backupIndex);
		
		if (restored) {
			Backup backup = (Backup) this.backups.get(backupIndex);
			this.clearDynamicObstacles();
			this.addDynamicObstacles(backup.dynamicObstacles);
			restored = true;
		}
		
		return restored;
	}
	
	/**
	 * Shares dynamic obstacles among all existing AD* backups for dynamic repair.
	 * 
	 * @param dynamicObstacles the dynamic obstacles to be shared
	 */
	protected void shareDynamicObstacles(Set<Obstacle> dynamicObstacles) {
		for (int backupIndex = 0; backupIndex < backups.size(); backupIndex++) {
			if (this.hasBackup(backupIndex)) {
				Backup backup = (Backup) this.backups.get(backupIndex);
				backup.dynamicObstacles.addAll(dynamicObstacles);
			}
		}
	}
	
	/**
	 * Determines whether or not an AD* backup has dynamic obstacles.
	 * 
	 * @param backupIndex the index of the backup
	 * 
	 * @return true if the AD* backup has dynamic obstacles, false otherwise
	 */
	protected boolean hasDynamicObstacles(int backupIndex) {
		boolean hasDynamicObstacles = false;
		
		if (this.hasBackup(backupIndex)) {
			Backup backup = (Backup) this.backups.get(backupIndex);
			hasDynamicObstacles = !backup.dynamicObstacles.isEmpty();
		}
		
		return hasDynamicObstacles;
	}
	
	/**
	 * Determines whether or not this AD* planner matches a specification.
	 * 
	 * @param specification the specification to be matched
	 * 
	 * @return true if the this AD* planner matches the specification,
	 *         false otherwise
	 * 
	 * @see AbstractPlanner#matches(Specification)
	 */
	@Override
	public boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = false;
		
		if ((null != specification) && (specification.getProperties() instanceof ADStarProperties)) {
			ADStarProperties adsp = (ADStarProperties) specification.getProperties();
			matches = (this.getCostPolicy().equals(adsp.getCostPolicy()))
					&& (this.getRiskPolicy().equals(adsp.getRiskPolicy()))
					&& (this.getMinimumQuality() == adsp.getMinimumQuality())
					&& (this.getMaximumQuality() == adsp.getMaximumQuality())
					&& (this.getQualityImprovement() == adsp.getQualityImprovement()
					&& (this.getSignificantChange() == adsp.getSignificantChange())
					&& (specification.getId().equals(Specification.PLANNER_ADS_ID)));
		}
		
		return matches;
	}
	
}
