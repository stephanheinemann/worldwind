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
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.util.Identifiable;

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
	public void setStandby(boolean isStandby) {
		if (this.isStandby != isStandby) {
			if (isStandby) {
				this.removePlanRevisionListener(this.getMissionLoader());
			} else {
				this.addPlanRevisionListener(this.getMissionLoader());
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
	public boolean isStandby() {
		return this.isStandby;
	}
	
	/**
	 * Performs a take-off via the datalink communication of this managed grid
	 * planner.
	 */
	@Override
	protected void performTakeOff() {
		if (!this.isStandby()) {
			super.performTakeOff();
		}
	}
	
	/**
	 * Performs a landing via the datalink communication of this managed grid
	 * planner.
	 */
	@Override
	protected void performLanding() {
		if (!this.isStandby) {
			super.performLanding();
		}
	}
	
	/**
	 * Performs an unplanned landing via the datalink communication of this
	 * managed grid planner.
	 */
	@Override
	protected void performUnplannedLanding() {
		if (!this.isStandby()) {
			super.performUnplannedLanding();
		}
	}
	
	/**
	 * Establishes the datalink communication of this managed tree planner.
	 */
	@Override
	protected void establishDatalink() {
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
