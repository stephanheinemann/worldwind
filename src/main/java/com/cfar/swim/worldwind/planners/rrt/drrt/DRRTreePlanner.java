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
package com.cfar.swim.worldwind.planners.rrt.drrt;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.environments.DirectedEdge;
import com.cfar.swim.worldwind.environments.Edge;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.planners.AbstractPlanner;
import com.cfar.swim.worldwind.planners.DynamicObstacleListener;
import com.cfar.swim.worldwind.planners.DynamicPlanner;
import com.cfar.swim.worldwind.planners.LifelongPlanner;
import com.cfar.swim.worldwind.planners.rrt.brrt.RRTreeWaypoint;
import com.cfar.swim.worldwind.planners.rrt.hrrt.HRRTreePlanner;
import com.cfar.swim.worldwind.planning.TimeInterval;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.planners.rrt.DRRTreeProperties;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.session.ObstacleManager;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes a dynamic RRT planner that plans a trajectory of an aircraft in an
 * environment considering a local cost and risk policy. The planner can cope
 * with partial environment knowledge efficiently repairing and revising plans
 * accordingly.
 * 
 * @author Manuel Rosa
 * @author Stephan Heinemann
 *
 */
public class DRRTreePlanner extends HRRTreePlanner
implements DynamicPlanner, LifelongPlanner {
	
	/** indicates whether or or not this DRRT planner has terminated */
	private boolean terminated = false;
	
	/** the obstacle manager of this DRRT planner */
	private ObstacleManager obstacleManager = null;
	
	/** the dynamic obstacles of an obstacle commitment by this DRRT planner */
	private Set<Obstacle> dynamicObstacles = new HashSet<>();
	
	/** the significant change threshold of this DRRT planner */
	private double significantChange = 0.5d;
	
	/** the actual change of this DRRT planner */
	private double change = 0d;
	
	/**
	 * Constructs a dynamic RRT planner for a specified aircraft and environment
	 * using default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see HRRTreePlanner#HRRTreePlanner(Aircraft, Environment)
	 */
	public DRRTreePlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}
	
	/**
	 * Gets the start DRRT waypoint of this DRRT planner.
	 * 
	 * @return the start DRRT waypoint of this DRRT planner
	 */
	@Override
	protected DRRTreeWaypoint getStart() {
		return (DRRTreeWaypoint) super.getStart();
	}

	/**
	 * Gets the goal DRRT waypoint of this DRRT planner.
	 * 
	 * @return the goal DRRT waypoint of this DRRT planner
	 */
	@Override
	protected DRRTreeWaypoint getGoal() {
		return (DRRTreeWaypoint) super.getGoal();
	}
	
	/**
	 * Gets the newest DRRT waypoint added to the tree.
	 * 
	 * @return the newest DRRT waypoint added to the tree
	 */
	@Override
	protected DRRTreeWaypoint getNewestWaypoint() {
		return (DRRTreeWaypoint) super.getNewestWaypoint();
	}
	
	/**
	 * Creates a DRRT waypoint at a specified position.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return the DRRT waypoint at the specified position
	 */
	@Override
	protected DRRTreeWaypoint createWaypoint(Position position) {
		return new DRRTreeWaypoint(position);
	}
	
	/**
	 * Clears the dynamic obstacles of this DRRT planner.
	 */
	protected void clearDynamicObstacles() {
		this.dynamicObstacles.clear();
	}
	
	/**
	 * Adds dynamic obstacles to this DRRT planner.
	 * 
	 * @param dyamicObstacles the dynamic obstacles to be added
	 */
	protected void addDynamicObstacles(Collection<Obstacle> dyamicObstacles) {
		this.dynamicObstacles.addAll(dyamicObstacles);
	}
	
	/**
	 * Gets the dynamic obstacles of this DRRT planner.
	 * 
	 * @return the dynamic obstacles of this DRRT planner
	 */
	protected Iterable<Obstacle> getDynamicObstacles() {
		return this.dynamicObstacles;
	}
	
	/**
	 * Determines whether or not this DRRT planner has dynamic obstacles.
	 * 
	 * @return true if this DRRT planner has dynamic obstacles, false otherwise
	 */
	protected boolean hasDynamicObstacles() {
		return !this.dynamicObstacles.isEmpty();
	}
	
	/**
	 * Initializes this DRRT planner to plan from an origin to a destination at
	 * a specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @see HRRTreePlanner#initialize(Position, Position, ZonedDateTime)
	 */
	@Override
	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {
		super.initialize(origin, destination, etd);
		this.clearDynamicObstacles();
	}
	
	/**
	 * Updates the quality (maximum cost) of this DRRT planner in accordance
	 * with the dependent branches of a root.
	 * 
	 * @param root the root of dependent branches
	 */
	protected void updateQuality(DRRTreeWaypoint root) {
		if (root.hasChildren()) {
			for (RRTreeWaypoint child : root.getChidren()) {
				this.updateQuality((DRRTreeWaypoint) child);
			}
		} else {
			this.getQuality().setMaximumCost(Math.max(
				this.getQuality().getMaximumCost(),
				root.getF()));
		}
	}
	
	/**
	 * Updates the quality (maximum cost) of this DRRT planner.
	 */
	protected void updateQuality() {
		this.getQuality().setMaximumCost(this.getStart().getF());
		this.updateQuality(this.getStart());
	}
	
	/**
	 * Trims the generated tree by this DRRT planner from an invalid root
	 * and its dependent branches retaining only its valid portions.
	 * 
	 * @param root the invalid root of a branch to be trimmed
	 */
	protected void trim(DRRTreeWaypoint root) {
		if (this.getStart().equals(root)) {
			// purge the entire tree
			this.clearExpendables();
			this.getStart().clearChildren();
			this.getEnvironment().addVertex(this.getStart());
		} else {
			root.setValid(false);
			
			// trim all sub-branches of the invalid vertex
			if (root.hasChildren()) {
				for (RRTreeWaypoint child : root.getChidren()) {
					this.trim((DRRTreeWaypoint) child);
				}
				root.clearChildren();
			}
			// remove the invalid vertex
			if (root.hasParent()) {
				root.setParent(null);
				this.getEnvironment().removeVertex(root);
			}
		}
	}
	
	/**
	 * Trims the tree generated by this DRRT planner from any invalid vertices
	 * and their dependent branches retaining only its valid portions.
	 */
	protected void trim() {
		// trim all affected tree branches
		for (Obstacle obstacle : this.getDynamicObstacles()) {
			this.getEnvironment().findAffectedEdges(obstacle).stream()
			.filter(edge -> obstacle.getCostInterval()
					.intersects(new TimeInterval(
						((DRRTreeWaypoint) edge.getFirstPosition()).getEto(),
						((DRRTreeWaypoint) edge.getSecondPosition()).getEto())))
			.map(edge -> (DRRTreeWaypoint) edge.getSecondPosition())
			.forEachOrdered(waypoint -> this.trim(waypoint));
		}
		
		// handle a significant change
		if (this.hasSignificantChange()) {
			this.handleSignificantChange();
		}
		
		this.clearDynamicObstacles();
		this.updateQuality();
		this.setNewestWaypoint(this.getStart());
	}
	
	/**
	 * Determines whether or not the plan of this DRRT planner is valid.
	 * 
	 * @return true if the plan of this DRRT planner is valid, false otherwise
	 */
	protected boolean hasValidPlan() {
		return this.hasWaypoints()
				&& (0 == this.getWaypoints().stream()
				.filter(waypoint -> ((DRRTreeWaypoint) waypoint).isInvalid())
				.count());
	}
	
	/**
	 * Determines whether or not a DRRT plan needs a potential repair.
	 * 
	 * @return true if the DRRT plan needs a potential repair, false otherwise
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
	 * Repairs a DRRT plan in case of dynamic changes.
	 * 
	 * @param partIndex the index of the part to be repaired
	 */
	protected void repair(int partIndex) {
		if (this.needsRepair()) {
			DRRTreeWaypoint partStart = (DRRTreeWaypoint) this.getStart().clone();
			
			// repair previous parts before current part
			if (this.hasDynamicObstacles(partIndex - 1)) {
				this.backup(partIndex);
				this.restore(partIndex -1);
				this.planPart(partIndex - 1);
				// TODO: what if no plan could be found
				partStart.setEto(this.getGoal().getEto());
				partStart.setParent(this.getGoal().getParent());
				this.backup(partIndex - 1);
				this.restore(partIndex);
			}
			
			// TODO: consider departure slots to avoid planning from scratch
			// TODO: consider early arrivals with holding / loitering
			if (this.getStart().getEto().equals(partStart.getEto())) {
				// connect current to previous part
				// TODO: feasibility issues connecting to goal region
				if (partStart.hasParent()) {
					this.getStart().setParent(partStart.getParent());
				}
				
				// trim and re-grow tree if plan is affected
				this.trim();
				if (!this.hasValidPlan()) {
					this.compute();
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
	}
	
	/**
	 * Elaborates a DRRT plan.
	 * 
	 * @param partIndex the index of the plan to be elaborated
	 */
	protected void elaborate(int partIndex) {
		// proceed to next part only if not in need of repair
		while (this.needsRepair() && !this.hasTerminated()) {
			this.repair(partIndex);
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
	 * @see HRRTreePlanner#plan(Position, Position, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		Trajectory trajectory = new Trajectory();
		this.initBackups(1);
		this.initialize(origin, destination, etd);
		
		while (!this.hasTerminated()) {
			trajectory = this.planPart(0);
			this.revisePlan(trajectory);
			// wait for termination or dynamic changes
			this.suspend();
		}
		
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
	 * @see HRRTreePlanner#plan(Position, Position, List, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
		Trajectory trajectory = new Trajectory();
		this.initBackups(waypoints.size() + 1);
		
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
		
		return trajectory;
	}
	
	/**
	 * Suspends this DRRT planner until termination or context changes occur.
	 */
	protected synchronized void suspend() {
		try {
			this.wait();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * Notifies this DRRT planner about a pending obstacle change.
	 *
	 * @see DynamicObstacleListener#notifyPendingObstacleChange()
	 */
	@Override
	public synchronized void notifyPendingObstacleChange() {
		this.notifyAll();
	}
	
	/**
	 * Sets the obstacle manager of this DRRT planner.
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
	 * Determines whether or not this DRRT planner has an obstacle manager.
	 * 
	 * @return true if this DRRT planner has an obstacle manager,
	 *         false otherwise
	 * 
	 * @see DynamicObstacleListener#hasObstacleManager()
	 */
	@Override
	public synchronized boolean hasObstacleManager() {
		return (null != this.obstacleManager);
	}
	
	/**
	 * Terminates this DRRT planner.
	 * 
	 * @see LifelongPlanner#terminate()
	 */
	@Override
	public synchronized void terminate() {
		this.terminated = true;
		this.notifyAll();
	}
	
	/**
	 * Recycles this DRRT planner.
	 * 
	 * @see LifelongPlanner#recycle()
	 */
	@Override
	public synchronized void recycle() {
		this.terminated = false;
	}
	
	/**
	 * Indicates whether or not this DRRT planner has terminated.
	 * 
	 * @return true if this DRRT planner has terminated, false otherwise
	 * 
	 * @see LifelongPlanner#hasTerminated()
	 */
	@Override
	public synchronized boolean hasTerminated() {
		return this.terminated;
	}
	
	/**
	 * Gets the significant change threshold of this DRRT planner.
	 * 
	 * @return the significant change threshold of this DRRT planner.
	 * 
	 * @see DynamicPlanner#getSignificantChange()
	 */
	@Override
	public double getSignificantChange() {
		return this.significantChange;
	}
	
	/**
	 * Sets the significant change threshold of this DRRT planner.
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
	 * Determines whether or or not this DRRT planner has a significant dynamic
	 * change.
	 * 
	 * @return true if this DRRT planner has a significant dynamic change,
	 *         false otherwise
	 */
	protected boolean hasSignificantChange() {
		boolean hasSignificantChange = false;
		// TODO: examine different determination policies including
		// (1) invalid close to start versus close to goal (ETOs)
		// (2) even if the current trajectory is not directly affected,
		//     the surrounding environment could allow for improvements
		
		// the amount of waypoints in the current plan
		int wc = this.getWaypoints().size();
		// the amount of invalid waypoints in the current plan
		long ic = this.getWaypoints().stream()
			.filter(waypoint -> ((DRRTreeWaypoint) waypoint).isInvalid())
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
		// purge entire tree
		this.trim(this.getStart());
	}
	
	/** the backups of this DRRT planner */
	protected final ArrayList<Backup> backups = new ArrayList<>();
	
	/**
	 * Realizes a backup of this DRRT planner.
	 * 
	 * @author Stephan Heinemann
	 * 
	 */
	protected class Backup {
		
		/** the start waypoint of this DRRT backup */
		public DRRTreeWaypoint start = null;
		
		/** the goal waypoint of this DRRT backup */
		public DRRTreeWaypoint goal = null;
		
		/** the edges of this DRRT backup */
		public Set<DirectedEdge> edges = new HashSet<>();
		
		/** the dynamic obstacles of this DRRT backup */
		public Set<Obstacle> dynamicObstacles = new HashSet<>();
		
		/** the plan waypoints of this DRRT backup */
		public List<Waypoint> plan = new LinkedList<Waypoint>();
		
		/**
		 * Clears this DRRT backup.
		 */
		public void clear() {
			this.start = null;
			this.goal = null;
			this.edges.clear();
			this.dynamicObstacles.clear();
			this.plan.clear();
		}
		
		/**
		 * Determines whether or not this DRRT backup is empty.
		 * 
		 * @return true if this DRRT backup is empty, false otherwise
		 */
		public boolean isEmpty() {
			return (null == this.start) && (null == this.goal)
					&& this.edges.isEmpty()
					&& this.dynamicObstacles.isEmpty()
					&& this.plan.isEmpty();
		}
	}
	
	/**
	 * Initializes a number backups for this DRRT planner.
	 * 
	 * @param size the number of backups for this DRRT planner
	 */
	protected void initBackups(int size) {
		this.backups.clear();
		for (int backupIndex = 0; backupIndex < size; backupIndex++) {
			this.backups.add(backupIndex, new Backup());
		}
	}
	
	/**
	 * Determines whether or not a backup of this DRRT planner can be performed.
	 * 
	 * @param backupIndex the index of the backup
	 * 
	 * @return true if a backup can be performed, false otherwise
	 */
	protected boolean canBackup(int backupIndex) {
		return (-1 < backupIndex) && (backupIndex < this.backups.size());
	}
	
	/**
	 * Determines whether or not this DRRT planner has a backup.
	 * 
	 * @param backupIndex the index of the backup
	 * 
	 * @return true if this DRRT planner has a backup, false otherwise
	 */
	protected boolean hasBackup(int backupIndex) {
		return this.canBackup(backupIndex) && (!this.backups.get(backupIndex).isEmpty());
	}
	
	/**
	 * Backs up this DRRT planner for dynamic repair.
	 * 
	 * @param backupIndex the index of the backup
	 * 
	 * @return true if a backup has been performed, false otherwise
	 */
	protected boolean backup(int backupIndex) {
		boolean backedup = false;
		
		if (this.canBackup(backupIndex)) {
			Backup backup = this.backups.get(backupIndex);
			backup.clear();
			backup.start = this.getStart();
			backup.goal = this.getGoal();
			for (Edge edge : this.getEnvironment().getEdges()) {
				backup.edges.add((DirectedEdge) edge);
			}
			backup.dynamicObstacles.addAll(this.dynamicObstacles);
			backup.plan.addAll(this.getWaypoints());
			backedup = true;
		}
		
		return backedup;
	}
	
	/**
	 * Restores this DRRT planner for dynamic repair.
	 * 
	 * @param backupIndex the index of the backup
	 * 
	 * @return true if a restore has been performed, false otherwise
	 */
	protected boolean restore(int backupIndex) {
		boolean restored = false;
		
		if (this.hasBackup(backupIndex)) {
			Backup backup = this.backups.get(backupIndex);
			this.clearExpendables();
			this.setStart(backup.start);
			this.getQuality().setOptimalCost(this.getStart().getF());
			this.setGoal(backup.goal);
			double maxCost = this.getStart().getF();
			for (Edge edge : backup.edges) {
				this.getEnvironment().addEdge(edge);
				maxCost = Math.max(maxCost,
						((DRRTreeWaypoint) edge.getSecondPosition()).getF());
			}
			this.getQuality().setMaximumCost(maxCost);
			this.clearDynamicObstacles();
			this.addDynamicObstacles(backup.dynamicObstacles);
			this.addAllWaypoints(backup.plan);
			restored = true;
		}
		
		return restored;
	}
	
	/**
	 * Shares dynamic obstacles among all existing DRRT backups for dynamic repair.
	 * 
	 * @param dynamicObstacles the dynamic obstacles to be shared
	 */
	protected void shareDynamicObstacles(Set<Obstacle> dynamicObstacles) {
		for (int backupIndex = 0; backupIndex < backups.size(); backupIndex++) {
			if (this.hasBackup(backupIndex)) {
				Backup backup = this.backups.get(backupIndex);
				backup.dynamicObstacles.addAll(dynamicObstacles);
			}
		}
	}
	
	/**
	 * Determines whether or not a DRRT backup has dynamic obstacles.
	 * 
	 * @param backupIndex the index of the backup
	 * 
	 * @return true if the DRRT backup has dynamic obstacles, false otherwise
	 */
	protected boolean hasDynamicObstacles(int backupIndex) {
		boolean hasDynamicObstacles = false;
		
		if (this.hasBackup(backupIndex)) {
			Backup backup = this.backups.get(backupIndex);
			hasDynamicObstacles = !backup.dynamicObstacles.isEmpty();
		}
		
		return hasDynamicObstacles;
	}
	
	/**
	 * Determines whether or not this DRRT planner matches a specification.
	 * 
	 * @param specification the specification to be matched
	 * 
	 * @return true if the this DRRT planner matches the specification,
	 *         false otherwise
	 * 
	 * @see AbstractPlanner#matches(Specification)
	 */
	@Override
	public boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = false;
		
		if ((null != specification) && (specification.getProperties() instanceof DRRTreeProperties)) {
			DRRTreeProperties drrtp = (DRRTreeProperties) specification.getProperties();
			matches = (this.getCostPolicy().equals(drrtp.getCostPolicy()))
					&& (this.getRiskPolicy().equals(drrtp.getRiskPolicy()))
					&& (this.getBias() == drrtp.getBias())
					&& (this.getEpsilon() == drrtp.getEpsilon())
					&& (this.getExtension() == drrtp.getExtension())
					&& (this.getGoalThreshold() == drrtp.getGoalThreshold())
					&& (this.getMaxIterations() == drrtp.getMaxIterations())
					&& (this.getSampling() == drrtp.getSampling())
					&& (this.getStrategy() == drrtp.getStrategy())
					&& (this.getAlgorithm() == drrtp.getAlgorithm())
					&& (this.getVariant() == drrtp.getVariant())
					&& (this.getNeighborLimit() == drrtp.getNeighborLimit())
					&& (this.getQualityBound() == drrtp.getQualityBound())
					&& (this.getSignificantChange() == drrtp.getSignificantChange())
					&& (specification.getId().equals(Specification.PLANNER_DRRT_ID));
		}
		
		return matches;
	}
	
}
