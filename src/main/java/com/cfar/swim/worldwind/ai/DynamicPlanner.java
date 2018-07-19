/**
 * Copyright (c) 2018, Henrique Ferreira (UVic Center for Aerospace Research)
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

import java.util.Set;

import com.cfar.swim.worldwind.render.Obstacle;

/**
 * Describes a dynamic planner which issues plans computed in dynamic
 * environments. The dynamic planner continuously revises the plan considering
 * new obstacles that have been added to the environment since the last plan was
 * computed.
 * 
 * @author Henrique Ferreira
 * @author Manuel Rosa
 */
public interface DynamicPlanner extends Planner {

	/**
	 * Gets the set of different obstacles containing the difference between the
	 * previous obstacle set and the current one.
	 * 
	 * @return the set of different obstacles containing the difference between the
	 *         previous obstacle set and the current one
	 */
	public Set<Obstacle> getDiffObstacles();

	/**
	 * Sets the set of different obstacles containing the difference between the
	 * previous obstacle set and the current one.
	 * 
	 * @param diffObstacles the set of different obstacles containing the difference
	 *            between the previous obstacle set and the current one
	 */
	public void setDiffObstacles(Set<Obstacle> diffObstacles);

	/**
	 * Updates the set of different obstacles in the planner by saving the old
	 * obstacles set, calling a function to allow the introduction of new obstacles
	 * and computing the difference between the two.
	 */
	public void updateObstacles();

	/**
	 * Checks whether or not the changes made to the environment are significant to
	 * take further actions.
	 * 
	 * @return true if the changes are significant, false otherwise
	 */
	public boolean isSignificantChange();

	/**
	 * Repairs the plan and its environment in order to reflect the changes made to
	 * the set of obstacles.
	 * 
	 * @param diffObstacles the set of obstacles to consider
	 */
	public void repair(Set<Obstacle> diffObstacles);

}
