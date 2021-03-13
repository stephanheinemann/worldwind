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
package com.cfar.swim.worldwind.ai;

/**
 * Describes a dynamic planner which repairs changing costs while planning.
 * A dynamic planner only revises invalid parts of the plan being computed
 * and avoids the revision of the entire plan as much as possible.
 * 
 * @author Stephan Heinemann
 * 
 */
public interface DynamicPlanner extends Planner, DynamicCostListener {

	// TODO: merge with and check for PRM/RTT interface
	
	/**
	 * Terminates this dynamic planner.
	 */
	public void terminate();
	
	/**
	 * Recycles this dynamic planner.
	 */
	public void recycle();
	
	/**
	 * Indicates whether or not this dynamic planner has terminated.
	 * 
	 * @return true if this dynamic planner has terminated, false otherwise
	 */
	public boolean hasTerminated();
	
	/**
	 * Sets the significant change threshold of this dynamic planner.
	 * 
	 * @param significantChange the signficant change threshold of this dynamic
	 *                          planner
	 */
	public void setSignificantChange(double significantChange);
	
	/**
	 * Indicates whether or or not this dynamic planner has a significant
	 * dynamic change.
	 * 
	 * @return true if this dynamic planner has a significant dynamic change,
	 *         false otherwise
	 */
	public boolean hasSignificantChange();
	
	// repairCost(Waypoint)
	
	// significant cost = affected legs / current legs (e.g. accept 0.25)
	// changes close to start versus goal (resulting repair)
	
	// setSignficantCostChange
	// isSignificantCostChange
	// notifyCostChange (implements Cost/Obstacle ChangeListener, registers at DynamicEnvironment)
	// Environment <- StaticEnvironment, DynamicEnvironment ?
	// DynamicEnvironment.registerDynamicCostListener(DynamicCostListener dcl)
}
