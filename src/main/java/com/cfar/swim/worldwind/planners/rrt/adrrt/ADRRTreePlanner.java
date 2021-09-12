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
package com.cfar.swim.worldwind.planners.rrt.adrrt;

import java.time.ZonedDateTime;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.planners.DynamicObstacleListener;
import com.cfar.swim.worldwind.planners.DynamicPlanner;
import com.cfar.swim.worldwind.planners.LifelongPlanner;
import com.cfar.swim.worldwind.planners.rrt.arrt.ARRTreePlanner;
import com.cfar.swim.worldwind.planners.rrt.brrt.RRTreeWaypoint;
import com.cfar.swim.worldwind.planning.TimeInterval;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.planners.rrt.ADRRTreeProperties;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.session.ObstacleManager;
import com.cfar.swim.worldwind.util.Identifiable;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes an anytime dynamic RRT planner that plans a trajectory of an
 * aircraft in an environment considering a local cost and risk policy. The
 * planner can cope with partial environment knowledge efficiently repairing,
 * improving and revising plans according to a desired quality as long as
 * deliberation time is available.
 * 
 * @author Stephan Heinemann
 *
 */
public class ADRRTreePlanner extends ARRTreePlanner
implements DynamicPlanner, LifelongPlanner {
	
	/** indicates whether or not this ADRRT planner has terminated */
	private boolean terminated = false;
	
	/** indicates whether or not this ADRRT planning is listening */
	private boolean isListening = false;
	
	/** the obstacle manager of this ADRRT planner */
	private ObstacleManager obstacleManager = null;
	
	/** the dynamic obstacles of an obstacle commitment by this ADRRT planner */
	private Set<Obstacle> dynamicObstacles = new HashSet<>();
	
	/** the significant change threshold of this ADRRT planner */
	private double significantChange = 0.5d;
	
	/** the actual change of this ADRRT planner */
	private double change = 0d;
	
	/**
	 * Constructs an anytime dynamic RRT planner for a specified aircraft and
	 * environment using default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see ARRTreePlanner#ARRTreePlanner(Aircraft, Environment)
	 */
	public ADRRTreePlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}
	
	/**
	 * Gets the identifier of this ADRRT planner.
	 * 
	 * @return the identifier of this ADRRT planner
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return Specification.PLANNER_ADRRT_ID;
	}
	
	/**
	 * Gets the start ADRRT waypoint of this ADRRT planner.
	 * 
	 * @return the start ADRRT waypoint of this ADRRT planner
	 */
	@Override
	protected ADRRTreeWaypoint getStart() {
		return (ADRRTreeWaypoint) super.getStart();
	}
	
	/**
	 * Gets the goal ADRRT waypoint of this ADRRT planner.
	 * 
	 * @return the goal ADRRT waypoint of this ADRRT planner
	 */
	@Override
	protected ADRRTreeWaypoint getGoal() {
		return (ADRRTreeWaypoint) super.getGoal();
	}
	
	/**
	 * Gets the newest ADRRT waypoint added to the tree.
	 * 
	 * @return the newest ADRRT waypoint added to the tree
	 */
	@Override
	protected ADRRTreeWaypoint getNewestWaypoint() {
		return (ADRRTreeWaypoint) super.getNewestWaypoint();
	}
	
	/**
	 * Creates an ADRRT waypoint at a specified position.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return the ADRRT waypoint at the specified position
	 */
	@Override
	protected ADRRTreeWaypoint createWaypoint(Position position) {
		return new ADRRTreeWaypoint(position);
	}
	
	/**
	 * Clears the dynamic obstacles of this ADRRT planner.
	 */
	protected void clearDynamicObstacles() {
		this.dynamicObstacles.clear();
	}
	
	/**
	 * Adds dynamic obstacles to this ADRRT planner.
	 * 
	 * @param dyamicObstacles the dynamic obstacles to be added
	 */
	protected void addDynamicObstacles(Collection<Obstacle> dyamicObstacles) {
		this.dynamicObstacles.addAll(dyamicObstacles);
	}
	
	/**
	 * Gets the dynamic obstacles of this ADRRT planner.
	 * 
	 * @return the dynamic obstacles of this ADRRT planner
	 */
	protected Iterable<Obstacle> getDynamicObstacles() {
		return this.dynamicObstacles;
	}
	
	/**
	 * Determines whether or not this ADRRT planner has dynamic obstacles.
	 * 
	 * @return true if this ADRRT planner has dynamic obstacles, false otherwise
	 */
	protected boolean hasDynamicObstacles() {
		return !this.dynamicObstacles.isEmpty();
	}
	
	/**
	 * Initializes this ADRRT planner to plan from an origin to a destination at
	 * a specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @see ARRTreePlanner#initialize(Position, Position, ZonedDateTime)
	 */
	@Override
	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {
		super.initialize(origin, destination, etd);
		this.clearDynamicObstacles();
	}
	
	/**
	 * Trims the tree generated by this ADRRT planner from an invalid root
	 * and its dependent branches retaining only its valid portions.
	 * 
	 * @param root the invalid root of a branch to be trimmed
	 */
	protected void trim(ADRRTreeWaypoint root) {
		if (this.getStart().equals(root)) {
			// purge the entire tree
			this.clearExpendables();
			this.getStart().clearChildren();
			this.getPlanningContinuum().addVertex(this.getStart());
		} else {
			root.setValid(false);
			
			// trim all sub-branches of the invalid vertex
			if (root.hasChildren()) {
				for (RRTreeWaypoint child : root.getChidren()) {
					this.trim((ADRRTreeWaypoint) child);
				}
				root.clearChildren();
			}
			// remove the invalid vertex
			root.removeParent();
			this.getPlanningContinuum().removeVertex(root);
		}
	}
	
	/**
	 * Trims the tree generated by this ADRRT planner from any invalid vertices
	 * and their dependent branches retaining only its valid portions.
	 */
	protected void trim() {
		// trim all affected tree branches
		for (Obstacle obstacle : this.getDynamicObstacles()) {
			this.getPlanningContinuum().findAffectedEdges(obstacle).stream()
			.filter(edge -> obstacle.getCostInterval()
					.intersects(new TimeInterval(
						((ADRRTreeWaypoint) edge.getFirstPosition()).getEto(),
						((ADRRTreeWaypoint) edge.getSecondPosition()).getEto())))
			.map(edge -> (ADRRTreeWaypoint) edge.getSecondPosition())
			.forEachOrdered(waypoint -> this.trim(waypoint));
		}
		
		// handle a significant change
		if (this.hasSignificantChange()) {
			this.handleSignificantChange();
		}
		
		this.setNewestWaypoint(this.getStart());
	}
	
	/**
	 * Determines whether or not the plan of this ADRRT planner is valid.
	 * 
	 * @return true if the plan of this ADRRT planner is valid, false otherwise
	 */
	protected boolean hasValidPlan() {
		return this.hasWaypoints()
				&& (0 == this.getWaypoints().stream()
				.filter(waypoint -> ((ADRRTreeWaypoint) waypoint).isInvalid())
				.count());
	}
	
	/**
	 * Determines whether or not an ADRRT plan needs a potential repair.
	 * 
	 * @return true if the ADRRT plan needs a potential repair, false otherwise
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
	 * Repairs an ADRRT plan in case of dynamic changes.
	 * 
	 * @param partIndex the index of the part to be repaired
	 * 
	 * @return false if a repair is not possible due to incomplete previous
	 *         parts, true otherwise
	 */
	protected boolean repair(int partIndex) {
		boolean repaired = true;
		
		if (this.needsRepair()) {
			ADRRTreeWaypoint partStart = (ADRRTreeWaypoint) this.getStart().clone();
			
			// repair previous parts before current part
			if (this.hasDynamicObstacles(partIndex - 1)) {
				this.backup(partIndex);
				this.restore(partIndex -1);
				this.planPart(partIndex - 1);
				partStart.setCost(this.getGoal().getCost());
				partStart.setEto(this.getGoal().getEto());
				partStart.setParent(this.getGoal().getParent());
				this.backup(partIndex - 1);
				this.restore(partIndex);
			}
			
			if (partStart.hasInfiniteCost()) {
				// no previous part without exceeded risk policy
				this.clearDynamicObstacles();
				this.clearWaypoints();
				this.getGoal().setInfiniteCost();
				repaired = false;
			} else if (this.getStart().getEto().equals(partStart.getEto())) {
				// connect current to previous part
				// TODO: consider departure slots to avoid planning from scratch
				// TODO: consider early arrivals with holding / loitering
				// TODO: feasibility issues connecting to goal region
				if (partStart.hasParent()) {
					this.getStart().setParent(partStart.getParent());
				}
				
				// trim and re-grow tree if plan is affected
				this.trim();
				this.clearDynamicObstacles();
				// force improvement in case of dynamic removals
				this.setCostBound(this.getGoal().getCost());
				if (!this.hasValidPlan()) {
					// an invalid plan has an infinite cost bound
					this.getGoal().setInfiniteCost();
					this.setCostBound(this.getGoal().getCost());
					if (this.compute() && !this.getGoal().hasInfiniteCost()) {
						// trajectory without exceeding risk policy
						this.revisePlan(this.createTrajectory());
						this.updateCostBound();
					} else {
						// no trajectory or exceeded risk policy
						this.clearWaypoints();
					}
				}	
			} else {
				// plan current part from scratch if start ETO has changed
				this.initialize(this.getStart(), this.getGoal(), partStart.getEto());
				if (partStart.hasParent()) {
					this.getStart().setParent(partStart.getParent());
				}
				super.planPart(partIndex);
			}
		}
		
		return repaired;
	}
	
	/**
	 * Elaborates an ADRRT plan.
	 * 
	 * @param partIndex the index of the plan to be elaborated
	 */
	protected void elaborate(int partIndex) {
		// do not elaborate an exceeded risk policy solution beyond limit
		int riskyProbes = (this.getGoal().hasInfiniteCost()) ? 1 : 0;
		
		// proceed to next part only if fully improved and not in need of repair
		while (((!this.hasMaximumQuality() && (this.getMaxRiskyProbes() > riskyProbes))
				|| this.needsRepair()) && !this.hasTerminated()) {
			if (!this.repair(partIndex)) break;
			this.improve(partIndex);
			if (this.getGoal().hasInfiniteCost()) {
				riskyProbes++;
			} else {
				riskyProbes = 0;
			}
			Thread.yield();
		}
		// backup after elaboration
		this.backup(partIndex);
	}
	
	/**
	 * Plans a part of a trajectory. Repairs the planned part if required.
	 * 
	 * @param partIndex the index of the part
	 * 
	 * @return the planned part of a trajectory
	 */ 
	@Override
	protected Trajectory planPart(int partIndex) {
		Trajectory trajectory = null;
		
		if (this.hasBackup(partIndex)) {
			// repair an existing plan after dynamic changes
			this.elaborate(partIndex);
			trajectory = this.createTrajectory();
		} else {
			// plan from scratch
			trajectory = super.planPart(partIndex);
		}
		
		// always backup at least once for potential repair later
		this.backup(partIndex);
		return trajectory;
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
	 * @see ARRTreePlanner#plan(Position, Position, ZonedDateTime)
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
	 * @see ARRTreePlanner#plan(Position, Position, List, ZonedDateTime)
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
				this.revisePlan(trajectory);
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
	 * Suspends this ADRRT planner until termination or context changes occur.
	 */
	protected synchronized void suspend() {
		try {
			this.wait();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * Sets whether or not this ADRRT planner is listening.
	 * 
	 * @param isListening the listening state to be set
	 */
	protected synchronized void setListening(boolean isListening) {
		this.isListening = isListening;
	}
	
	/**
	 * Determines whether or not this ADRRT planner is listening.
	 * 
	 * @return true if this ADRRT planning is listening,
	 *         false otherwise
	 *
	 * @see DynamicObstacleListener#isListening()
	 */
	@Override
	public synchronized boolean isListening() {
		return this.isListening;
	}
	
	/**
	 * Notifies this ADRRT planner about a pending obstacle change.
	 *
	 * @see DynamicObstacleListener#notifyPendingObstacleChange()
	 */
	@Override
	public synchronized void notifyPendingObstacleChange() {
		this.notifyAll();
	}
	
	/**
	 * Sets the obstacle manager of this ADRRT planner.
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
	 * Determines whether or not this ADRRT planner has an obstacle manager.
	 * 
	 * @return true if this ADRRT planner has an obstacle manager,
	 *         false otherwise
	 * 
	 * @see DynamicObstacleListener#hasObstacleManager()
	 */
	@Override
	public synchronized boolean hasObstacleManager() {
		return (null != this.obstacleManager);
	}
	
	/**
	 * Terminates this ADRRT planner.
	 * 
	 * @see LifelongPlanner#terminate()
	 */
	@Override
	public synchronized void terminate() {
		this.terminated = true;
		this.notifyAll();
	}
	
	/**
	 * Recycles this ADRRT planner.
	 * 
	 * @see LifelongPlanner#recycle()
	 */
	@Override
	public synchronized void recycle() {
		this.terminated = false;
	}
	
	/**
	 * Indicates whether or not this ADRRT planner has terminated.
	 * 
	 * @return true if this ADRRT planner has terminated, false otherwise
	 * 
	 * @see LifelongPlanner#hasTerminated()
	 */
	@Override
	public synchronized boolean hasTerminated() {
		return this.terminated;
	}
	
	/**
	 * Gets the significant change threshold of this ADRRT planner.
	 * 
	 * @return the significant change threshold of this ADRRT planner.
	 * 
	 * @see DynamicPlanner#getSignificantChange()
	 */
	@Override
	public synchronized double getSignificantChange() {
		return this.significantChange;
	}
	
	/**
	 * Sets the significant change threshold of this ADRRT planner.
	 * 
	 * @param significantChange the significant change threshold to be set
	 * 
	 * @see DynamicPlanner#setSignificantChange(double)
	 * 
	 * @throws IllegalArgumentException if significant change threshold is
	 *                                  invalid
	 */
	@Override
	public synchronized void setSignificantChange(double significantChange) {
		if ((0d <= significantChange) && (1d >= significantChange)) {
			this.significantChange = significantChange;
		} else {
			throw new IllegalArgumentException("significant change is invalid");
		}
	}
	
	/**
	 * Determines whether or or not this ADRRT planner has a significant
	 * dynamic change.
	 * 
	 * @return true if this ADRRT planner has a significant dynamic change,
	 *         false otherwise
	 */
	protected synchronized boolean hasSignificantChange() {
		boolean hasSignificantChange = false;
		// TODO: examine different determination policies including
		// (1) invalid close to start versus close to goal (ETOs)
		// (2) even if the current trajectory is not directly affected,
		//     the surrounding environment could allow for improvements
		
		// the amount of waypoints in the current plan
		int wc = this.getWaypoints().size();
		// the amount of invalid waypoints in the current plan
		long ic = this.getWaypoints().stream()
			.filter(waypoint -> ((ADRRTreeWaypoint) waypoint).isInvalid())
			.count();
		// the ratio of invalid to all waypoints in the current plan
		if (0 != wc) {
			this.change = ((double) ic) / wc;
			hasSignificantChange = (this.change >= this.significantChange);
		}
		
		return hasSignificantChange;
	}
	
	/**
	 * Handles a significant change for this ADRRT planner.
	 */
	protected void handleSignificantChange() {
		// TODO: purge entire tree or adjust distance and cost biases 
		// this.trim(this.getStart());
		this.setDistanceBias(1d);
		this.setCostBias(0d);
	}
	
	/**
	 * Realizes a backup of this ADRRT planner.
	 * 
	 * @author Stephan Heinemann
	 * 
	 */
	protected class Backup extends ARRTreePlanner.Backup {
		
		/** the dynamic obstacles of this ADRRT backup */
		public Set<Obstacle> dynamicObstacles = new HashSet<>();
		
		/**
		 * Clears this ADRRT backup.
		 * 
		 * @see ARRTreePlanner.Backup#clear()
		 */
		@Override
		public void clear() {
			super.clear();
			this.dynamicObstacles.clear();
		}
		
		/**
		 * Determines whether or not this ADRRT backup is empty.
		 * 
		 * @return true if this ADRRT backup is empty, false otherwise
		 * 
		 * @see ARRTreePlanner.Backup#isEmpty()
		 */
		@Override
		public boolean isEmpty() {
			return super.isEmpty() && this.dynamicObstacles.isEmpty();
		}
	}
	
	/**
	 * Initializes a number backups for this ADRRT planner.
	 * 
	 * @param size the number of backups for this ADRRT planner
	 */
	@Override
	protected void initBackups(int size) {
		this.backups.clear();
		for (int backupIndex = 0; backupIndex < size; backupIndex++) {
			this.backups.add(backupIndex, new Backup());
		}
	}
	
	/**
	 * Backs up this ADRRT planner for dynamic repair.
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
	 * Restores this ADRRT planner for dynamic repair.
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
	 * Shares dynamic obstacles among all existing ADRRT backups for dynamic repair.
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
	 * Determines whether or not an ADRRT backup has dynamic obstacles.
	 * 
	 * @param backupIndex the index of the backup
	 * 
	 * @return true if the ADRRT backup has dynamic obstacles, false otherwise
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
	 * Determines whether or not this ADRRT planner matches a specification.
	 * 
	 * @param specification the specification to be matched
	 * 
	 * @return true if the this ADRRT planner matches the specification,
	 *         false otherwise
	 * 
	 * @see ARRTreePlanner#matches(Specification)
	 */
	@Override
	public synchronized boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = super.matches(specification);
		
		if (matches && (specification.getProperties() instanceof ADRRTreeProperties)) {
			ADRRTreeProperties properties = (ADRRTreeProperties) specification.getProperties();
			matches = (this.getSignificantChange() == properties.getSignificantChange());
		}
		
		return matches;
	}
	
	/**
	 * Updates this ADRRT planner according to a specification.
	 * 
	 * @param specification the specification to be used for the update
	 * 
	 * @return true if this ADRRT planner has been updated, false otherwise
	 * 
	 * @see ARRTreePlanner#update(Specification)
	 */
	@Override
	public synchronized boolean update(Specification<? extends FactoryProduct> specification) {
		boolean updated = super.update(specification);
		
		if (updated && (specification.getProperties() instanceof ADRRTreeProperties)) {
			ADRRTreeProperties properties = (ADRRTreeProperties) specification.getProperties();
			this.setSignificantChange(properties.getSignificantChange());
		}
		
		return updated;
	}
	
}
