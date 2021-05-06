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
package com.cfar.swim.worldwind.planners.cgs.arastar;

import com.cfar.swim.worldwind.planners.cgs.astar.AStarWaypoint;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes an ARA* waypoint of a trajectory featuring estimates for costs and
 * time. ARA* waypoints are used by ARA* planners. 
 * 
 * @author Stephan Heinemann
 *
 */
public class ARAStarWaypoint extends AStarWaypoint {
	
	/** the weight of the estimated remaining cost (epsilon) of this ARA* waypoint */
	private double epsilon;
	
	/**
	 * Constructs an ARA* waypoint at a specified position.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @see AStarWaypoint#AStarWaypoint(Position)
	 */
	public ARAStarWaypoint(Position position) {
		super(position);
		this.setEpsilon(1d);
	}
	
	/**
	 * Constructs an ARA* waypoint at a specified position with a specified
	 * weight of the estimated remaining cost (epsilon).
	 * 
	 * @param position the position in globe coordinates
	 * @param epsilon the weight of the estimated remaining cost (epsilon)
	 * 
	 * @see AStarWaypoint#AStarWaypoint(Position)
	 */
	public ARAStarWaypoint(Position position, double epsilon) {
		super(position);
		this.setEpsilon(epsilon);
	}
	
	/**
	 * Gets the parent ARA* waypoint of this ARA* waypoint.
	 * 
	 * @return the parent ARA* waypoint of this ARA* waypoint
	 * 
	 * @see AStarWaypoint#getParent()
	 */
	@Override
	public ARAStarWaypoint getParent() {
		return (ARAStarWaypoint) super.getParent();
	}
	
	/**
	 * Gets the weight of the estimated remaining cost (epsilon) of this ARA*
	 * waypoint.
	 * 
	 * @return the weight of the estimated remaining cost (epsilon) of this
	 *         ARA* waypoint
	 */
	public double getEpsilon() {
		return this.epsilon;
	}
	
	/**
	 * Sets the weight of the estimated remaining cost (epsilon) of this ARA*
	 * waypoint.
	 * 
	 * @param epsilon the weight of the estimated remaining cost (epsilon) of
	 *                this ARA* waypoint
	 */
	public void setEpsilon(double epsilon) {
		if (1d > epsilon) {
			throw new IllegalArgumentException("epsilon is less than 1");
		}
		this.epsilon = epsilon;
	}
	
	/**
	 * Gets the priority key for the expansion of this ARA* waypoint.
	 * The heuristic of the estimated total cost is inflated (weighted) by
	 * epsilon.
	 * 
	 * @return the priority key for the expansion of this ARA* waypoint
	 * 
	 * @see #getEpsilon()
	 * @see #setEpsilon(double)
	 */
	@Override
	public double getKey() {
		return this.getG() + (this.getEpsilon() * this.getH());
	}

}
