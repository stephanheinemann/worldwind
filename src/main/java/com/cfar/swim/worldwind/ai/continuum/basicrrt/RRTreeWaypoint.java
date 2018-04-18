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
package com.cfar.swim.worldwind.ai.continuum.basicrrt;

import com.cfar.swim.worldwind.ai.continuum.SampledWaypoint;

import gov.nasa.worldwind.geom.Position;

/**
 * @author Manuel Rosa
 *
 */
public class RRTreeWaypoint extends SampledWaypoint {
	// TODO: Possible inclusion of aircraft heading to define actions possible
	// for extension

	/** the parent RRT waypoint of this RRT waypoint in a trajectory */
	private RRTreeWaypoint parent = null;

	/** the cost (g-value) to get to this this waypoint */
	private double g = Double.POSITIVE_INFINITY;

	/** the estimated remaining cost (h-value) of this waypoint */
	private double h = Double.POSITIVE_INFINITY;

	/**
	 * Constructs an RRT waypoint at a specified position.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @see Waypoint#Waypoint(Position)
	 */
	public RRTreeWaypoint(Position position) {
		super(position);
	}

	public RRTreeWaypoint(Position position, RRTreeWaypoint parent) {
		super(position);
		this.parent = parent;
	}

	/**
	 * Gets the parent RRT waypoint of this RRT waypoint.
	 * 
	 * @return the parent RRT waypoint of this RRT waypoint
	 */
	public RRTreeWaypoint getParent() {
		return parent;
	}

	/**
	 * Sets the parent RRT waypoint of this RRT waypoint.
	 * 
	 * @param parent the parent RRT waypoint of this RRT waypoint
	 */
	public void setParent(RRTreeWaypoint parent) {
		this.parent = parent;
	}

	/**
	 * Gets the cost (g-value) to get to this this waypoint.
	 * 
	 * @return the cost (g-value) to get to this this waypoint
	 */
	public double getG() {
		return g;
	}

	/**
	 * Gets the cost (g-value) to get to this this waypoint.
	 * 
	 * @param g the cost (g-value) to get to this this waypoint
	 */
	public void setG(double g) {
		this.g = g;
	}

	/**
	 * Gets the estimated remaining cost (h-value) of this waypoint.
	 * 
	 * @return the estimated remaining cost (h-value) of this waypoint
	 */
	public double getH() {
		return h;
	}

	/**
	 * Sets the estimated remaining cost (h-value) of this waypoint.
	 * 
	 * @param h the estimated remaining cost (h-value) of this waypoint
	 */
	public void setH(double h) {
		this.h = h;
	}

	/**
	 * Gets the total cost (f-value) of this waypoint from start to goal by adding the actual cost to this
	 * point from start and the estimated cost to goal.
	 * 
	 * @return the total cost (f-value) of this waypoint
	 */
	public double getF() {
		return g + h;
	}
	
    public String toString() {
    	return String.format("( %.6f*, %.6f*, %3.1fm ) g=%.2f h=%.2f f=%.2f",
    			this.latitude.degrees, this.longitude.degrees, this.elevation,
    			this.getG(), this.getH(), this.getF());
    }

}
