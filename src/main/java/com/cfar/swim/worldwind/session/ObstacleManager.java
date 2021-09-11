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
package com.cfar.swim.worldwind.session;

import java.util.Set;

import com.cfar.swim.worldwind.render.Obstacle;

/**
 * Describes an obstacle manager responsible for requesting and committing
 * obstacle changes.
 * 
 * @author Stephan Heinemann
 *
 */
public interface ObstacleManager {
	
	/**
	 * Submits an obstacle addition to this obstacle manager. The obstacle
	 * addition has to be committed before being effective.
	 * 
	 * @param obstacles the obstacles to be added to this obstacle manager
	 */
	public void submitAddObstacles(Set<Obstacle> obstacles);
	
	/**
	 * Submits an obstacle removal to this obstacle manager. The obstacle
	 * removal has to be committed before being effective.
	 * 
	 * @param obstacles the obstacles to be removed from this obstacle manager
	 */
	public void submitRemoveObstacles(Set<Obstacle> obstacles);
	
	/**
	 * Submits an obstacle replacement to this obstacle manager. The obstacle
	 * replacement has to be committed before being effective.
	 * 
	 * @param obstacles the obstacles to be replaced in this obstacle manager
	 */
	public void submitReplaceObstacles(Set<Obstacle> obstacles);
	
	/**
	 * Submits an obstacle clearance to this obstacle manager. The obstacle
	 * clearance has to be committed before being effective.
	 */
	public void submitClearObstacles();
	
	/**
	 * Submits an obstacle enabling to this obstacle manager. The obstacle
	 * enabling has to be committed before being effective.
	 *
	 * @param obstacles the obstacles to be enabled by this obstacle manager
	 */
	public void submitEnableObstacles(Set<Obstacle> obstacles);
	
	/**
	 * Submits an obstacle disabling to this obstacle manager. The obstacle
	 * disabling has to be committed before being effective.
	 * 
	 * @param obstacles the obstacles to be disables by this obstacle manager
	 */
	public void submitDisableObstacles(Set<Obstacle> obstacles);
	
	/**
	 * Commits an obstacle change (addition, removal, enabling, disabling) to
	 * this obstacle manager.
	 * 
	 * @return the obstacles that were changed
	 */
	public Set<Obstacle> commitObstacleChange();
	
	/**
	 * Retracts an obstacle change (addition, removal, enabling, disabling)
	 * from this obstacle manager.
	 */
	public void retractObstacleChange();
	
	/**
	 * Determines whether or not this obstacle manager has an obstacle change.
	 * 
	 * @return true if this obstacle manager has an obstacle change,
	 *         false otherwise
	 */
	public boolean hasObstacleChange();
	
}
