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
package com.cfar.swim.worldwind.planning;

import java.util.Set;

import gov.nasa.worldwind.geom.Position;

/**
 * TODO : Describe
 * 
 * 
 * @author Henrique Ferreira
 *
 */
public interface DiscreteEnvironment extends Environment{
//TODO: Review whether or not cost intervals methods should be defined here

	/**
	 * Indicates whether or not a position is a waypoint in this discrete environment.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return true if the position is a waypoint in this discrete environment,
	 *         false otherwise
	 */
	public boolean isWaypoint(Position position);
	
	/**
	 * Gets the adjacent waypoints of a position in this discrete environment.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return the adjacent waypoints of the position in this
	 *         discrete environment, or the waypoint position itself
	 */
	public Set<Position> getAdjacentWaypoints(Position position);
	
	/**
	 * Indicates whether or not a position is adjacent to a waypoint in this
	 * discrete environment.
	 * 
	 * @param position the position in globe coordinates
	 * @param waypoint the waypoint in globe coordinates
	 * 
	 * @return true if the position is adjacent to the waypoint in this
	 *         discrete environment, false otherwise
	 */
	public boolean isAdjacentWaypoint(Position position, Position waypoint);
		
	/**
	 * Gets the neighbors of a position in this discrete environment. 
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return the neighbors of the position in this discrete environment
	 */
	public Set<Position> getNeighbors(Position position);

}
