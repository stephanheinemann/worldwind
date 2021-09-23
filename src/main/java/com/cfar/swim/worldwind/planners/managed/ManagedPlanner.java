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

import com.cfar.swim.worldwind.planners.AnytimePlanner;
import com.cfar.swim.worldwind.planners.DynamicPlanner;
import com.cfar.swim.worldwind.planners.LifelongPlanner;
import com.cfar.swim.worldwind.planners.OnlinePlanner;
import com.cfar.swim.worldwind.planning.Trajectory;

/**
 * Describes a managed planner that can be employed by an autonomic manager.
 * 
 * @author Stephan Heinemann
 *
 */
public interface ManagedPlanner extends
OnlinePlanner, DynamicPlanner, LifelongPlanner, AnytimePlanner,
Callable<Trajectory> {
	
	/**
	 * Sets this managed planner to standby or active.
	 * 
	 * @param isStandby true if standby, false if active
	 */
	public void setStandby(boolean isStandby);
	
	/**
	 * Determines whether or not this managed planner is standing by.
	 * 
	 * @return true if this managed planner is standing by, false if active
	 */
	public boolean isStandby();
	
	/**
	 * Gets the goals of this managed planner.
	 * 
	 * @return the goals of this managed planner
	 */
	public ManagedGoals getGoals();
	
	/**
	 * Sets the goals of this managed planner.
	 * 
	 * @param goals the goals to be set
	 */
	public void setGoals(ManagedGoals goals);
	
	/**
	 * Determines whether or not this managed planner has goals.
	 * 
	 * @return true if this managed planner has goals, false otherwise
	 */
	public boolean hasGoals();
	
	/**
	 * Joins a managed planner at its next waypoint.
	 * 
	 * @param associate the managed planner to be joined
	 */
	public void join(ManagedPlanner associate);
	
}
