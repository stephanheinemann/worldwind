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
package com.cfar.swim.worldwind.planners;

import com.cfar.swim.worldwind.planning.Trajectory;

import gov.nasa.worldwind.util.Logging;

/**
 * Realizes a mission loader to upload revised trajectories via the
 * datalink of an online planner.
 * 
 * @author Stephan Heinemann
 *
 * @see PlanRevisionListener
 * @see OnlinePlanner
 */
public class MissionLoader implements PlanRevisionListener {
	
	/** the online planner of this mission loader */
	private final OnlinePlanner planner;
	
	/**
	 * Constructs a new mission loader for an online planner.
	 * 
	 * @param planner the online planner
	 * 
	 * @throws IllegalArgumentException if the online planner is null
	 */
	public MissionLoader(OnlinePlanner planner) {
		if (null == planner) {
			throw new IllegalArgumentException("invalid planner");
		}
		this.planner = planner;
	}
	
	/**
	 * Uploads a revised trajectory via the datalink of the online planner of
	 * this mission loader.
	 * 
	 * @param trajectory the revised trajectory to be uploaded
	 * 
	 * @see PlanRevisionListener#revisePlan(Trajectory)
	 */
	@Override
	public void revisePlan(Trajectory trajectory) {
		if (this.planner.hasDatalink() && this.planner.getDatalink().isConnected()) {
			
			// warn if mission is obsolete
			if (this.planner.hasNextWaypoint() &&
					!this.planner.getNextWaypoint().getPrecisionPosition().getOriginal()
					.equals(this.planner.getDatalink().getNextMissionPosition())) {
				Logging.logger().warning(this.planner.getId()
						+ ": obsolete next mission position...");
			}
			
			// do not upload an empty trajectory
			if (!trajectory.isEmpty()) {
				Logging.logger().info(this.planner.getId()
						+ ": uploading mission: " + trajectory);
				this.planner.getDatalink().uploadMission(trajectory);
				// confirm the consistent upload
				if (!this.planner.getDatalink().hasMission(trajectory, false)) {
					Logging.logger().severe(this.planner.getId()
							+ ": uploaded mission is not consistent...");
				}
			} else {
				Logging.logger().warning(this.planner.getId()
						+ ": not uploading an empty trajectory...");
			}
		}
	}
	
}
