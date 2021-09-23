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
import com.cfar.swim.worldwind.planners.rrt.oadrrt.OADRRTreePlanner;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.util.Identifiable;

/**
 * Realizes a managed tree planner based on an online anytime dynamic RRT
 * planner.
 * 
 * @author Stephan Heinemann
 *
 */
public class ManagedTreePlanner extends OADRRTreePlanner implements ManagedPlanner {
	
	/** the goals of this managed tree planner */
	private ManagedGoals goals = null;
	
	/** the standby status of this managed tree planner */
	private boolean isStandby = false;
	
	/** the associate planner of this managed grid planner to join */
	private ManagedPlanner associate = null;
	
	/**
	 * Constructs a managed tree planner for a specified aircraft and
	 * environment.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 *
	 * @see OADRRTreePlanner#OADRRTreePlanner(Aircraft, Environment)
	 */
	public ManagedTreePlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}
	
	/**
	 * Constructs a managed tree planner for a specified aircraft and
	 * environment to plan for specified goals.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * @param goals the goals
	 */
	public ManagedTreePlanner(
			Aircraft aircraft, Environment environment,
			ManagedGoals goals) {
		super(aircraft, environment);
		this.goals = goals;
	}
	
	/**
	 * Gets the identifier of this managed tree planner.
	 * 
	 * @return the identifier of this managed tree planner
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return Specification.PLANNER_MTP_ID;
	}
	
	/**
	 * Sets this managed tree planner to standby or active.
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
	 * Determines whether or not this managed tree planner is standing by.
	 * 
	 * @return true if this managed tree planner is standing by,
	 *         false if active
	 * 
	 * @see ManagedPlanner#isStandby()
	 */
	@Override
	public synchronized boolean isStandby() {
		return this.isStandby;
	}
	
	/**
	 * Determines whether or not the aircraft of this managed tree planner is
	 * on track.
	 * 
	 * @return true if the aircraft of this managed tree planner is on track,
	 *         false otherwise
	 * 
	 * @see OADRRTreePlanner#isOnTrack()
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
	 * Performs a take-off via the datalink communication of this managed tree
	 * planner.
	 */
	@Override
	protected synchronized void performTakeOff() {
		if (!this.isStandby()) {
			super.performTakeOff();
		}
	}
	
	/**
	 * Performs a landing via the datalink communication of this managed tree
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
	 * managed tree planner.
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
	 * Gets the goals of this managed tree planner.
	 * 
	 * @return the goals of this managed tree planner
	 * 
	 * @see ManagedPlanner#getGoals()
	 */
	@Override
	public ManagedGoals getGoals() {
		return this.goals;
	}
	
	/**
	 * Sets the goals of this managed tree planner.
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
	 * Determines whether or not this managed tree planner has goals.
	 * 
	 * @return true if this managed tree planner has goals, false otherwise
	 * 
	 * @see ManagedPlanner#hasGoals()
	 */
	@Override
	public boolean hasGoals() {
		return (null != this.goals);
	}
	
	/**
	 * Gets the associate planner of this managed tree planner.
	 * 
	 * @return the associate planner of this managed tree planner
	 */
	protected synchronized ManagedPlanner getAssociate() {
		return this.associate;
	}
	
	/**
	 * Sets the associate planner of this managed tree planner
	 * 
	 * @param associate the associate planner to be set
	 */
	protected synchronized void setAssociate(ManagedPlanner associate) {
		this.associate = associate;
	}
	
	/**
	 * Determines whether or not this managed tree planner has an associate
	 * planner.
	 * 
	 * @return true if this managed tree planner has an associate planner,
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
			this.notifyAll();
		}
	}
	
	/**
	 * Joins the associate of this managed tree planner.
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
			this.restore(backups.size() - 1);
			this.replan(backups.size() - 1);
			this.setAssociate(null);
		}
	}
	
	/**
	 * Suspends this managed tree planner until termination, context changes,
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
	 * Calls this managed tree planner to plan a trajectory for its goals.
	 * 
	 * @throws IllegalStateException if this managed tree planner has no goals
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
