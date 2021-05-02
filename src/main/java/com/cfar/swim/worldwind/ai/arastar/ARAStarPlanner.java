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
package com.cfar.swim.worldwind.ai.arastar;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import com.cfar.swim.worldwind.ai.AnytimePlanner;
import com.cfar.swim.worldwind.ai.astar.AStarWaypoint;
import com.cfar.swim.worldwind.ai.astar.ForwardAStarPlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.geom.precision.PrecisionPosition;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.planners.ARAStarProperties;

import gov.nasa.worldwind.geom.Position;


/**
 * Realizes an Anytime Replanning A* planner (ARA*) that plans a trajectory of
 * an aircraft in an environment considering a local cost and risk policy.
 * 
 * @author Stephan Heinemann
 *
 */
public class ARAStarPlanner extends ForwardAStarPlanner implements AnytimePlanner {

	/** the set of inconsistent already expanded waypoints */
	protected Set<ARAStarWaypoint> incons = new HashSet<>();
	
	/** the initial inflation factor applied to the heuristic function */
	private double initialInflation;
	
	/** the final inflation factor applied the heuristic function */
	private double finalInflation;
	
	/** the current inflation factor applied to the heuristic function */
	private double inflation;
	
	/** the deflation amount to be applied to the current inflation */
	private double deflationAmount;
	
	/**
	 * Constructs an ARA* planner for a specified aircraft and environment
	 * using default local cost and risk policies without an initial inflation 
	 * applied to the heuristic.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see ForwardAStarPlanner#ForwardAStarPlanner(Aircraft, Environment)
	 */
	public ARAStarPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		this.setMinimumQuality(1d);
		this.setMaximumQuality(1d);
		this.setQualityImprovement(1d);
		this.setInflation(1d);
	}
	
	/**
	 * Gets the minimum quality (initial inflation) of this ARA* planner.
	 * 
	 * @return the minimum quality (initial inflation) of this ARA* planner
	 * 
	 * @see AnytimePlanner#getMinimumQuality()
	 */
	@Override
	public double getMinimumQuality() {
		return this.initialInflation;
	}
	
	/**
	 * Sets the minimum quality (initial inflation) of this ARA* planner.
	 * 
	 * @param initialInflation the minimum quality (initial inflation) of this
	 *                         ARA* planner
	 * 
	 * @throws IllegalArgumentException if the initial inflation is invalid
	 * 
	 * @see AnytimePlanner#setMinimumQuality(double)
	 */
	@Override
	public void setMinimumQuality(double initialInflation) {
		if ((1d <= initialInflation) &&
				(initialInflation >= this.finalInflation)) {
			this.initialInflation = initialInflation;
		} else {
			throw new IllegalArgumentException("initial inflation is invalid");
		}
	}
	
	/**
	 * Gets the maximum quality (final inflation) of this ARA* planner.
	 * 
	 * @return the maximum quality (final inflation) of this ARA* planner
	 * 
	 * @see AnytimePlanner#getMaximumQuality()
	 */
	@Override
	public double getMaximumQuality() {
		return this.finalInflation;
	}
	
	/**
	 * Sets the maximum quality (initial inflation) of this ARA* planner.
	 * 
	 * @param finalInflation the maximum quality (final inflation) of this
	 *                       ARA* planner
	 * 
	 * @throws IllegalArgumentException if the final inflation is invalid
	 * 
	 * @see AnytimePlanner#setMaximumQuality(double)
	 */
	@Override
	public void setMaximumQuality(double finalInflation) {
		if ((1d <= finalInflation) && (this.initialInflation >= finalInflation)) {
			this.finalInflation = finalInflation;
		} else {
			throw new IllegalArgumentException("final deflation is invalid");
		}
	}
	
	/**
	 * Gets the quality improvement (deflation amount) of this ARA* planner.
	 * 
	 * @return the quality improvement (deflation amount) of this ARA* planner
	 * 
	 * @see AnytimePlanner#getQualityImprovement()
	 */
	@Override
	public double getQualityImprovement() {
		return this.deflationAmount;
	}
	
	/**
	 * Sets the quality improvement (deflation amount) of this ARA* planner.
	 * 
	 * @param deflationAmount the quality improvement (deflation amount) of
	 *                        this ARA* planner
	 * 
	 * @throws IllegalArgumentException if the deflation amount is invalid
	 * 
	 * @see AnytimePlanner#setQualityImprovement(double)
	 */
	@Override
	public void setQualityImprovement(double deflationAmount) {
		if (0d < deflationAmount) {
			this.deflationAmount = deflationAmount;
		} else {
			throw new IllegalArgumentException("deflation amount is invalid");
		}
	}
	
	/**
	 * Decreases the current inflation by the deflation amount, or otherwise to
	 * the final deflation.
	 */
	protected void deflate() {
		if (this.finalInflation < (this.inflation - this.deflationAmount)) {
			this.inflation -= this.deflationAmount;
		} else {
			this.inflation = this.finalInflation;
		}
	}
	
	/**
	 * Determines whether or not the final deflation has been reached.
	 * 
	 * @return true if the final deflation has been reached, false otherwise
	 */
	protected boolean isDeflated() {
		return (this.inflation == this.finalInflation);
	}
	
	/**
	 * Gets the current inflation of this ARA* planner.
	 * 
	 * @return the current inflation of this ARA* planner
	 */
	protected double getInflation() {
		return this.inflation;
	}
	
	/**
	 * Sets the current inflation of this ARA* planner.
	 * 
	 * @param inflation the current inflation of this ARA* planner
	 */
	protected void setInflation(double inflation) {
		this.inflation = inflation;
	}
	
	/**
	 * Creates an ARA* waypoint at a specified position.
	 * 
	 * @param position the position
	 * 
	 * @return the ARA* waypoint at the specified position
	 */
	@Override
	protected ARAStarWaypoint createWaypoint(Position position) {
		ARAStarWaypoint araswp = null;
		
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
			araswp = this.getStart();
		} else if (this.hasGoal() && this.getGoal().equals(position)) {
			araswp = this.getGoal();
		} else {
			araswp = new ARAStarWaypoint(position);
		}
		
		// avoid duplicating discovered waypoints
		Optional<? extends AStarWaypoint> visitedWaypoint = this.findVisited(araswp);
		if (visitedWaypoint.isPresent()) {
			araswp = (ARAStarWaypoint) visitedWaypoint.get();
		} else {
			this.addVisited(araswp);
		}

		// set current inflation factor
		araswp.setEpsilon(this.getInflation());
		return araswp;
	}
	
	/**
	 * Gets the start ARA* waypoint of this ARA* planner.
	 * 
	 * @return the start ARA* waypoint of this ARA* planner
	 * 
	 * @see ForwardAStarPlanner#getStart()
	 */
	@Override
	protected ARAStarWaypoint getStart() {
		return (ARAStarWaypoint) super.getStart();
	}
	
	/**
	 * Gets the goal ARA* waypoint of this ARA* planner.
	 * 
	 * @return the goal ARA* waypoint of this ARA* planner
	 * 
	 * @see ForwardAStarPlanner#getGoal()
	 */
	@Override
	protected ARAStarWaypoint getGoal() {
		return (ARAStarWaypoint) super.getGoal();
	}
	
	/**
	 * Determines whether or not ARA* waypoints can be expanded.
	 * 
	 * @return true if ARA* waypoints can be expanded, false otherwise
	 * 
	 * @see ForwardAStarPlanner#canExpand()
	 */
	@Override
	protected boolean canExpand() {
		// allow for equal keys to propagate under-consistency to goal
		// and achieve potential improvements in terms of ETO
		return super.canExpand() &&
				(this.getGoal().getKey() >= this.peekExpandable().getKey());
	}
	
	/**
	 * Peeks an ARA* waypoint from the expandable ARA* waypoints.
	 * 
	 * @return the peeked ARA* waypoint from the expandable ARA* waypoints
	 *         if any, null otherwise
	 * 
	 * @see ForwardAStarPlanner#peekExpandable()
	 */
	@Override
	protected ARAStarWaypoint peekExpandable() {
		return (ARAStarWaypoint) super.peekExpandable();
	}
	
	/**
	 * Polls an ARA* waypoint from the expandable ARA* waypoints.
	 * 
	 * @return the polled ARA* waypoint from the expandable ARA* waypoints
	 *         if any, null otherwise
	 * 
	 * @see ForwardAStarPlanner#pollExpandable()
	 */
	@Override
	protected ARAStarWaypoint pollExpandable() {
		return (ARAStarWaypoint) super.pollExpandable();
	}
	
	/**
	 * Finds an expandable ARA* waypoint.
	 * 
	 * @param waypoint the expandable ARA* waypoint to be found
	 * 
	 * @return the found expandable ARA* waypoint if any
	 * 
	 * @see ForwardAStarPlanner#findExpandable(AStarWaypoint)
	 */
	@SuppressWarnings("unchecked")
	@Override
	protected Optional<? extends ARAStarWaypoint>
		findExpandable(AStarWaypoint waypoint) {
		return (Optional<ARAStarWaypoint>) super.findExpandable(waypoint);
	}
	
	/**
	 * Finds an expanded ARA* waypoint.
	 * 
	 * @param waypoint the expanded ARA* waypoint to be found
	 * 
	 * @return the found expanded ARA* waypoint, if any
	 * 
	 * @see ForwardAStarPlanner#findExpanded(AStarWaypoint)
	 */
	@SuppressWarnings("unchecked")
	@Override
	protected Optional<? extends ARAStarWaypoint>
		findExpanded(AStarWaypoint waypoint) {
		return (Optional<ARAStarWaypoint>) super.findExpanded(waypoint);
	}
	
	/**
	 * Expands an ARA* waypoint towards its neighbors in the environment.
	 * 
	 * @param waypoint the ARA* waypoint to be expanded
	 * 
	 * @return the neighbors of the expanded ARA* waypoint
	 * 
	 * @see ForwardAStarPlanner#expand(AStarWaypoint)
	 */
	@SuppressWarnings("unchecked")
	@Override
	protected Set<? extends ARAStarWaypoint> expand(AStarWaypoint waypoint) {
		return (Set<ARAStarWaypoint>) super.expand(waypoint);
	}
	
	/**
	 * Adds an ARA* waypoint to the inconsistent ARA* waypoints.
	 * 
	 * @param waypoint the ARA* waypoint
	 * 
	 * @return true if the ARA* waypoint has been added to the inconsistent
	 *         ARA* waypoints, false otherwise
	 */
	protected boolean addInconsistent(ARAStarWaypoint waypoint) {
		return this.incons.add(waypoint);
	}
	
	/**
	 * Removes an ARA* waypoint from the inconsistent ARA* waypoints.
	 * 
	 * @param waypoint the ARA* waypoint
	 * 
	 * @return true if the ARA* waypoint has been removed from the inconsistent
	 *         ARA* waypoints, false otherwise
	 */
	protected boolean removeInconsistent(ARAStarWaypoint waypoint) {
		return this.incons.remove(waypoint);
	}
	
	/**
	 * Clears the inconsistent ARA* waypoints.
	 */
	protected void clearInconsistent() {
		this.incons.clear();
	}
	
	/**
	 * Determines whether or not an ARA* waypoint has been made inconsistent.
	 * 
	 * @param waypoint the ARA* waypoint
	 * 
	 * @return true if the ARA* waypoint has been made inconsistent,
	 *         false otherwise
	 */
	protected boolean isInconsistent(ARAStarWaypoint waypoint) {
		return this.incons.contains(waypoint);
	}
	
	/**
	 * Finds an inconsistent ARA* waypoint.
	 * 
	 * @param waypoint the inconsistent ARA* waypoint to be found
	 * 
	 * @return the found inconsistent ARA* waypoint, if any
	 */
	protected Optional<? extends ARAStarWaypoint>
		findInconsistent(ARAStarWaypoint waypoint) {
		
		return this.incons.stream()
				.filter(w -> w.equals(waypoint))
				.findFirst();
	}
	
	/**
	 * Finds an existing ARA* waypoint.
	 * 
	 * @param waypoint the existing ARA* waypoint to be found
	 * 
	 * @return the found existing ARA* waypoint, if any
	 */
	@SuppressWarnings("unchecked")
	@Override
	protected Optional<? extends ARAStarWaypoint>
		findExisting(AStarWaypoint waypoint) {
		
		Optional<? extends ARAStarWaypoint> existing = (Optional<? extends ARAStarWaypoint>) super.findExisting(waypoint);
		
		if (existing.isEmpty()) {
			existing = this.findInconsistent((ARAStarWaypoint) waypoint);
		}
		
		return existing;
	}
	
	/**
	 * Initializes this ARA* planner to plan from an origin to a destination at
	 * a specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @see ForwardAStarPlanner#initialize(Position, Position, ZonedDateTime)
	 */
	@Override
	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {
		super.initialize(origin, destination, etd);
		this.clearInconsistent();
		this.setInflation(this.getMinimumQuality());
		this.getGoal().setEpsilon(this.getInflation());
		this.getStart().setEpsilon(this.getInflation());
	}
	
	/**
	 * Updates the planner waypoint sets for an updated ARA* waypoint.
	 * 
	 * @param waypoint the updated ARA* waypoint
	 * 
	 * @see ForwardAStarPlanner#updateSets(AStarWaypoint)
	 */
	@Override
	protected void updateSets(AStarWaypoint waypoint) {
		if (this.isExpanded(waypoint)) {
			this.addInconsistent((ARAStarWaypoint) waypoint);
		} else {
			super.updateSets(waypoint);
		}
	}
	
	/**
	 * Computes an ARA* plan.
	 * 
	 * @see ForwardAStarPlanner#compute()
	 */
	@Override
	protected void compute() {
		while (this.canExpand()) {
			ARAStarWaypoint source = this.pollExpandable();
			
			Set<? extends ARAStarWaypoint> neighbors = this.expand(source);
			
			for (ARAStarWaypoint target : neighbors) {
				this.updateWaypoint(source, target);
			}
		}
		
		this.connectPlan(this.getGoal());
	}
	
	/**
	 * Improves an ARA* plan incrementally.
	 * 
	 * @param partIndex the index of the plan part to be improved
	 * 
	 */
	protected void improve(int partIndex) {
		this.backup(partIndex);
		// TODO: potentially deflate below actual current sub-optimality
		// bound for more aggressive optimization 
		this.deflate();
		this.restore(partIndex);
		
		this.compute();
		this.revisePlan(this.createTrajectory());
	}
	
	/**
	 * Elaborates an ARA* plan.
	 * 
	 * @param partIndex the index of the plan part to be elaborated
	 */
	protected void elaborate(int partIndex) {
		while (!this.isDeflated()) {
			this.improve(partIndex);
		}
	}
	
	/**
	 * Plans a part of a trajectory from an origin to a destination at a
	 * specified estimated time of departure. If origin is the goal of the
	 * current plan, then the resulting plan will be the trajectory from
	 * the start of the current plan to the specified destination. Improves
	 * the computed trajectory incrementally.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 * @param partIndex the index of the part
	 * 
	 * @return the planned trajectory from origin at the estimated time of
	 *         departure or the start leading to origin in the current plan to
	 *         the destination
	 * 
	 * @see ForwardAStarPlanner#planPart(Position, Position, ZonedDateTime, int)
	 */
	@Override
	protected Trajectory planPart(Position origin, Position destination, ZonedDateTime etd, int partIndex) {
		super.planPart(origin, destination, etd, partIndex);
		this.elaborate(partIndex);
		return this.createTrajectory();
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
	 * @see ForwardAStarPlanner#plan(Position, Position, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		this.initBackups(1);
		this.initialize(origin, destination, etd);
		Trajectory trajectory = this.planPart(origin, destination, etd, 0);
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
	 * @see ForwardAStarPlanner#plan(Position, Position, List, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
		this.initBackups(waypoints.size() + 1);
		return super.plan(origin, destination, waypoints, etd);
	}
	
	/** the backups of this ARA* planner */
	protected final ArrayList<Backup> backups = new ArrayList<>();
	
	/**
	 * Realizes a backup of this ARA* planner.
	 * 
	 * @author Stephan Heinemann
	 */
	protected class Backup {
		/** the start waypoint of this ARA* backup */
		public ARAStarWaypoint start = null;
		
		/** the goal waypoint of this ARA* backup */
		public ARAStarWaypoint goal = null;
		
		/** the start region of this ARA* backup */
		public Set<PrecisionPosition> startRegion = new HashSet<>();
		
		/** the goal region of this ARA* backup */
		public Set<PrecisionPosition> goalRegion = new HashSet<>();
		
		/** the visited waypoints of this ARA* backup */
		public Set<AStarWaypoint> visited = new HashSet<>();
		
		/** the open waypoints of this ARA* backup */
		public Set<AStarWaypoint> open = new HashSet<>();
		
		/** the inconsistent waypoints of this ARA* backup */
		public Set<ARAStarWaypoint> incons = new HashSet<>();
		
		/** the closed waypoints of this ARA* backup */
		public Set<AStarWaypoint> closed = new HashSet<>();
		
		/** the plan waypoints of this ARA* backup */
		public List<AStarWaypoint> plan = new LinkedList<>(); 
		
		/**
		 * Clears this ARA* backup.
		 */
		public void clear() {
			this.start = null;
			this.goal = null;
			this.startRegion.clear();
			this.goalRegion.clear();
			this.visited.clear();
			this.open.clear();
			this.incons.clear();
			this.closed.clear();
			this.plan.clear();
		}
		
		/**
		 * Determines whether or not this ARA* backup is empty.
		 * 
		 * @return true if this ARA* backup is empty, false otherwise
		 */
		public boolean isEmpty() {
			return (null == this.start) && (null == this.goal)
					&& this.startRegion.isEmpty()
					&& this.goalRegion.isEmpty()
					&& this.visited.isEmpty()
					&& this.open.isEmpty()
					&& this.incons.isEmpty()
					&& this.closed.isEmpty()
					&& this.plan.isEmpty();
		}
	}
	
	/**
	 * Initializes a number backups for this ARA* planner.
	 * 
	 * @param size the number of backups for this ARA* planner
	 */
	protected void initBackups(int size) {
		this.backups.clear();
		for (int backupIndex = 0; backupIndex < size; backupIndex++) {
			this.backups.add(backupIndex, new Backup());
		}
	}
	
	/**
	 * Determines whether or not a backup of this ARA* planner can be performed.
	 * 
	 * @param backupIndex the index of the backup
	 * 
	 * @return true if a backup can be performed, false otherwise
	 */
	protected boolean canBackup(int backupIndex) {
		return (-1 < backupIndex) && (backupIndex < this.backups.size());
	}
	
	/**
	 * Determines whether or not this ARA* planner has a backup.
	 * 
	 * @param backupIndex the index of the backup
	 * 
	 * @return true if this ARA* planner has a backup, false otherwise
	 */
	protected boolean hasBackup(int backupIndex) {
		return this.canBackup(backupIndex) && (!this.backups.get(backupIndex).isEmpty());
	}
	
	/**
	 * Backs up this ARA* planner for incremental improvement.
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
			backup.startRegion.addAll(this.getStartRegion());
			backup.goalRegion.addAll(this.getGoalRegion());
			backup.visited.addAll(this.visited);
			backup.open.addAll(this.open);
			backup.incons.addAll(this.incons);
			backup.closed.addAll(this.closed);
			backup.plan.addAll(this.plan);
			backedup = true;
		}
		return backedup;
	}
	
	/**
	 * Restores this ARA* planner for incremental improvement.
	 * 
	 * @param backupIndex the index of the backup
	 * 
	 * @return true if a restore has been performed, false otherwise
	 */
	protected boolean restore(int backupIndex) {
		boolean restored = false;
		if (this.hasBackup(backupIndex)) {
			Backup backup = this.backups.get(backupIndex);
			// restore start and goal waypoints
			this.setStart(backup.start);
			this.getStart().setEpsilon(this.getInflation());
			this.setGoal(backup.goal);
			this.getGoal().setEpsilon(this.getInflation());
			
			// restore start and goal regions
			this.getStartRegion().clear();
			this.getStartRegion().addAll(backup.startRegion);
			this.getGoalRegion().clear();
			this.getGoalRegion().addAll(backup.goalRegion);
			
			// restore visited waypoints
			this.clearVisited();
			for (AStarWaypoint waypoint : backup.visited) {
				((ARAStarWaypoint) waypoint).setEpsilon(this.getInflation());
				this.addVisited(waypoint);
			}
			
			// restore expandable waypoints
			this.clearExpandables();
			for (AStarWaypoint waypoint : backup.open) {
				((ARAStarWaypoint) waypoint).setEpsilon(this.getInflation());
				// TODO: potentially prune open to optimize by removing
				// waypoints with non-inflated waypoint.getF() > goal.getG()
				this.addExpandable(waypoint);
			}
			// merge inconsistent with expandable waypoints for improvement
			for (ARAStarWaypoint waypoint : backup.incons) {
				waypoint.setEpsilon(this.getInflation());
				this.addExpandable(waypoint);
			}
			
			// clear inconsistent and expandable waypoints
			this.clearInconsistent();
			this.clearExpanded();
			
			this.clearWaypoints();
			this.plan.addAll(backup.plan);
			
			restored = true;
		}
		return restored;
	}
	
	/**
	 * Determines whether or not this ARA* planner matches a specification.
	 * 
	 * @param specification the specification to be matched
	 * 
	 * @return true if the this ARA* planner matches the specification,
	 *         false otherwise
	 * 
	 * @see FactoryProduct#matches(Specification)
	 */
	@Override
	public boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = false;
		
		if ((null != specification) && (specification.getProperties() instanceof ARAStarProperties)) {
			ARAStarProperties fasp = (ARAStarProperties) specification.getProperties();
			matches = (this.getCostPolicy().equals(fasp.getCostPolicy()))
					&& (this.getRiskPolicy().equals(fasp.getRiskPolicy()))
					&& (this.getMinimumQuality() == fasp.getMinimumQuality())
					&& (this.getMaximumQuality() == fasp.getMaximumQuality())
					&& (this.getQualityImprovement() == fasp.getQualityImprovement()
					&& (specification.getId().equals(Specification.PLANNER_ARAS_ID)));
		}
		
		return matches;
	}
	
}
