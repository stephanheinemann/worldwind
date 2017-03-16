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
package com.cfar.swim.worldwind.ai.arastar;

import com.cfar.swim.worldwind.ai.astar.AStarWaypoint;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes an ARA* waypoint of a trajectory featuring estimates for costs and
 * time. ARA* waypoints are used by ARA* planners. 
 * 
 * @author Stephan Heinemann
 *
 */
public class ARAStarWaypoint extends AStarWaypoint {
	
	// TODO: v-values and consistency is only relevant for AD*
	/** the estimated previous expansion cost (v-value) of this ARA* waypoint */
	private double v;
	
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
		this.setV(Double.POSITIVE_INFINITY);
		this.setEpsilon(1d);
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
	 * Gets the estimated previous expansion cost (v-value) of this ARA*
	 * waypoint.
	 * 
	 * @return the estimated previous expansion (v-value) cost of this ARA*
	 *         waypoint
	 */
	public double getV() {
		return this.v;
	}
	
	/**
	 * Sets the estimated previous expansion cost (v-value) of this ARA*
	 * waypoint.
	 * 
	 * @param v the estimated previous expansion cost (v-value) of this ARA*
	 *          waypoint
	 */
	public void setV(double v) {
		if (0d > v) {
			throw new IllegalArgumentException("v is less than 0");
		}
		this.v = v;
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
	 * Gets the estimated total cost (f-value) of this ARA* waypoint.
	 * The heuristic of the estimated total cost is inflated (weighted) by
	 * epsilon.
	 * 
	 * @return the estimated total cost (f-value) of this ARA* waypoint
	 * 
	 * @see #getEpsilon()
	 * @see #setEpsilon(double)
	 */
	@Override
	public double getF() {
		return this.getG() + (this.getEpsilon() * this.getH());
	}
	
	/**
	 * Indicates whether or not this ARA* waypoint is consistent, that is, if
	 * its current estimated cost (g-value) equals the estimated cost of its
	 * last expansion (v-value).
	 * 
	 * @return true if this ARA* waypoint is consistent, false otherwise
	 */
	public boolean isConsistent() {
		return this.getG() == this.getV();
	}
	
	/**
	 * Indicates whether or not this ARA* waypoint is over-consistent, that is,
	 * if its current estimated cost (g-value) is less than the estimated cost
	 * of its last expansion (v-value). The expansion of an over-consistent
	 * ARA* waypoint may potentially lead to an improved solution (propagation
	 * of over-consistency).
	 * 
	 * @return true if this ARA* waypoint is over-consistent, false otherwise
	 */
	public boolean isOverConsistent() {
		return this.getG() < this.getV();
	}
	
	/**
	 * Indicates whether or not this ARA* waypoint is under-consistent, that
	 * is, if its current estimated cost (g-value) is greater than the
	 * estimated cost of its last expansion (v-value). An under-consistent
	 * state indicates an increase in costs and requires an invalidation of
	 * dependent costs (propagation of under-consistency).
	 * 
	 * @return true if this ARA* waypoint is under-consistent, false otherwise
	 */
	public boolean isUnderConsistent() {
		return this.getG() > this.getV();
	}
	
	/**
	 * Makes this ARA* waypoint consistent.
	 * 
	 * @see #isConsistent()
	 */
	public void makeConsistent() {
		this.setV(this.getG());
	}

}
