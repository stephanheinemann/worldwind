/**
 * Copyright (c) 2018, Manuel Rosa (UVic Center for Aerospace Research)
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
package com.cfar.swim.worldwind.ai.continuum;

import java.util.List;

import gov.nasa.worldwind.geom.Position;

/**
 * Describes a sampler working on a continuous environment for a sampling based
 * motion planner
 * 
 * @author Manuel Rosa
 * @author Henrique Ferreira
 *
 */
public interface Sampler {

	/**
	 * Samples a random position from a continuous space defined in the current
	 * environment
	 * 
	 * @return position in global coordinates inside the environment
	 */
	Position sampleRandomPosition();

	
	/**
	 * Creates a SampledWaypoint embedding the CostInterval tree for respective
	 * position
	 * 
	 * @param position the position for the new waypoint
	 * 
	 * @return the SampledWaypoint with position and CostInterval
	 */
	SampledWaypoint createSampledWaypoint(Position position);
	
	/**
	 * Checks if a given position is in conflict with untraversable obstacles in
	 * the environment
	 * 
	 * @param waypoint the waypoint in global coordinates
	 * 
	 * @return boolean value true if there is a conflict
	 */
	boolean checkConflict(Position position);

	/**
	 * Checks if a straight leg between the waypoints is in conflict with
	 * untraversable obstacles in the environment
	 * 
	 * @param waypoint1 the first waypoint in global coordinates
	 * @param waypoint2 the second waypoint in global coordinates
	 * 
	 * @return boolean value true if there is a conflict
	 */
	// TODO: Check whether the creation of an edge in planning is beneficial
	boolean checkConflict(Position position1, Position position2);

	/**
	 * Finds the k-nearest waypoints to the given position considering the problems
	 * metric
	 * 
	 * @param position the position in global coordinates
	 * @param num number of waypoints to return
	 * 
	 * @return list of k-nearest waypoints sorted by increasing distance
	 */
	public List<? extends Position> findNearest(Position position, List<? extends Position> list, int num);
	
	/**
	 * Sorts a list of elements by increasing distance to a given position
	 * 
	 * @param position the position in global coordinates
	 * @param list the list of elements to be sorted
	 * 
	 * @return list of elements sorted by increasing distance to the position
	 */
	public List<? extends Position> sortNearest(Position position, List<? extends Position> list);

}
