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
package com.cfar.swim.worldwind.planners.managed;

import java.util.concurrent.Callable;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.planners.cgs.oadstar.OADStarPlanner;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.util.Identifiable;

import gov.nasa.worldwind.util.Logging;

/**
 * Realizes a managed grid planner based on an online anytime dynamic A*
 * planner.
 * 
 * @author Stephan Heinemann
 *
 */
public class ManagedGridPlanner extends OADStarPlanner implements ManagedPlanner {
	
	/** the goals of this managed grid planner */
	private ManagedGoals goals = null;
	
	/** the standby status of this managed grid planner */
	private boolean isStandby = false;
	
	/** the associate planner of this managed grid planner to join */
	private ManagedPlanner associate = null;
	
	/**
	 * Constructs a managed grid planner for a specified aircraft and
	 * environment.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 *
	 * @see OADStarPlanner#OADStarPlanner(Aircraft, Environment)
	 */
	public ManagedGridPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}
	
	/**
	 * Constructs a managed grid planner for a specified aircraft and
	 * environment to plan for specified goals.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * @param goals the goals
	 */
	public ManagedGridPlanner(
			Aircraft aircraft, Environment environment,
			ManagedGoals goals) {
		super(aircraft, environment);
		this.goals = goals;
	}
	
	/**
	 * Gets the identifier of this managed grid planner.
	 * 
	 * @return the identifier of this managed grid planner
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return Specification.PLANNER_MGP_ID;
	}
	
	/**
	 * Sets this managed grid planner to standby or active.
	 * 
	 * @param isStandby true if standby, false if active
	 * 
	 * @see ManagedPlanner#setStandby(boolean)
	 */
	@Override
	public synchronized void setStandby(boolean isStandby) {
		if (this.isStandby != isStandby) {
			if (isStandby) {
				this.getMissionLoader().mute();
			} else {
				this.getMissionLoader().unmute();
			}
			this.isStandby = isStandby;
		}
	}
	
	/**
	 * Determines whether or not this managed grid planner is standing by.
	 * 
	 * @return true if this managed grid planner is standing by,
	 *         false if active
	 * 
	 * @see ManagedPlanner#isStandby()
	 */
	@Override
	public synchronized boolean isStandby() {
		return this.isStandby;
	}
	
	/**
	 * Determines whether or not the aircraft of this managed grid planner is
	 * on track.
	 * 
	 * @return true if the aircraft of this managed grid planner is on track,
	 *         false otherwise
	 * 
	 * @see OADStarPlanner#isOnTrack()
	 */
	@Override
	public synchronized boolean isOnTrack() {
		boolean isOnTrack = true;
		
		if (!this.isStandby()) {
			isOnTrack = super.isOnTrack();
		}
		
		return isOnTrack;
	}
	
	/**
	 * Progresses the plan of this managed grid planner by rooting its
	 * generated graph at the next mission position.
	 * 
	 * @param partIndex the index of the part to be progressed
	 */
	@Override
	protected synchronized void progress(int partIndex) {
		if (!this.isStandby()) {
			super.progress(partIndex);
		}
	}
	
	/**
	 * Performs a take-off via the datalink communication of this managed grid
	 * planner.
	 */
	@Override
	protected synchronized void performTakeOff() {
		if (!this.isStandby()) {
			super.performTakeOff();
		}
	}
	
	/**
	 * Performs a landing via the datalink communication of this managed grid
	 * planner.
	 */
	@Override
	protected synchronized void performLanding() {
		if (!this.isStandby()) {
			super.performLanding();
		}
	}
	
	/**
	 * Performs an unplanned landing via the datalink communication of this
	 * managed grid planner.
	 */
	@Override
	protected synchronized void performUnplannedLanding() {
		if (!this.isStandby()) {
			super.performUnplannedLanding();
		}
	}
	
	/**
	 * Establishes the datalink communication of this managed tree planner.
	 */
	@Override
	protected synchronized void establishDatalink() {
		if (!this.isStandby()) {
			super.establishDatalink();
		}
	}
	
	/**
	 * Gets the goals of this managed grid planner.
	 * 
	 * @return the goals of this managed grid planner
	 * 
	 * @see ManagedPlanner#getGoals()
	 */
	@Override
	public ManagedGoals getGoals() {
		return this.goals;
	}
	
	/**
	 * Sets the goals of this managed grid planner.
	 * 
	 * @param goals the goals to be set
	 * 
	 * @see ManagedPlanner#setGoals(ManagedGoals)
	 */
	@Override
	public void setGoals(ManagedGoals goals) {
		this.goals = goals;
	}
	
	/**
	 * Determines whether or not this managed grid planner has goals.
	 * 
	 * @return true if this managed grid planner has goals, false otherwise
	 * 
	 * @see ManagedPlanner#hasGoals()
	 */
	@Override
	public boolean hasGoals() {
		return (null != this.goals);
	}
	
	/**
	 * Gets the associate planner of this managed grid planner.
	 * 
	 * @return the associate planner of this managed grid planner
	 */
	protected synchronized ManagedPlanner getAssociate() {
		return this.associate;
	}
	
	/**
	 * Sets the associate planner of this managed grid planner
	 * 
	 * @param associate the associate planner to be set
	 */
	protected synchronized void setAssociate(ManagedPlanner associate) {
		this.associate = associate;
	}
	
	/**
	 * Determines whether or not this managed grid planner has an associate
	 * planner.
	 * 
	 * @return true if this managed grid planner has an associate planner,
	 *         false otherwise
	 */
	protected synchronized boolean hasAssociate() {
		return (null != this.associate);
	}
	
	/**
	 * Joins a managed planner at its next waypoint.
	 * 
	 * @param associate the managed planner to be joined
	 * 
	 * @see ManagedPlanner#join(ManagedPlanner)
	 */
	@Override
	public synchronized void join(ManagedPlanner associate) {
		if (this.isStandby()) {
			this.setAssociate(associate);
		}
	}
	
	/**
	 * Joins the associate of this managed grid planner.
	 */
	protected synchronized void joinAssociate() {
		if (this.isStandby() && this.hasAssociate()
				&& this.getAssociate().hasNextWaypoint()) {
			Waypoint nextWaypoint = this.getAssociate().getNextWaypoint();
			int activePart = this.getAssociate().getActivePart();
			
			// clear all previous parts
			for (int part = 0; part < activePart; part++) {
				this.backups.get(part).clear();
			}
			
			// join active part
			this.restore(activePart);
			this.setStart(this.createWaypoint(nextWaypoint));
			this.getStart().setCost(nextWaypoint.getCost());
			this.getStart().setEto(nextWaypoint.getEto());
			this.backup(activePart);
			
			// re-plan from joined starting point
			Logging.logger().info(this.getId()
					+ " joins active part " + activePart
					+ " at " + this.getStart()
					+ " of " + this.getAssociate().getId());
			this.restore(this.backups.size() - 1);
			this.replan(this.backups.size() - 1);
			this.setAssociate(null);
		}
	}
	
	/**
	 * Suspends this managed grid planner until termination, context changes,
	 * off-track situations, or join requests occur.
	 */
	@Override
	protected synchronized void suspend() {
		try {
			// wait for termination, dynamic changes, off-track situations, or joins
			while (!this.hasTerminated() && !this.needsRepair() && this.isOnTrack()) {
				this.progress(this.backups.size() - 1);
				this.wait();
				this.joinAssociate();
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * Calls this managed grid planner to plan a trajectory for its goals.
	 * 
	 * @throws IllegalStateException if this managed grid planner has no goals
	 * 
	 * @see Callable#call()
	 */
	@Override
	public Trajectory call() throws IllegalStateException {
		Trajectory trajectory;
		
		if (this.hasGoals()) {
			if (this.getGoals().getPois().isEmpty()) {
				trajectory = this.plan(
						this.getGoals().getOrigin(),
						this.getGoals().getDestination(),
						this.getGoals().getEtd());
			} else {
				trajectory = this.plan(
						this.getGoals().getOrigin(),
						this.getGoals().getDestination(),
						this.getGoals().getPois(),
						this.getGoals().getEtd());
			}
		} else {
			throw new IllegalStateException();
		}
		
		return trajectory;
	}
	
}
