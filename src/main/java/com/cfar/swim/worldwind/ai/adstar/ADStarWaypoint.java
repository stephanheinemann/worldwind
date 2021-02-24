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
package com.cfar.swim.worldwind.ai.adstar;

import com.cfar.swim.worldwind.ai.arastar.ARAStarWaypoint;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes an AD* waypoint of a trajectory featuring estimates for costs and
 * time. AD* waypoints are used by AD* planners. 
 * 
 * @author Stephan Heinemann
 *
 */
public class ADStarWaypoint extends ARAStarWaypoint {
	
	/** the estimated previous expansion cost (v-value) of this AD* waypoint */
	private double v;
	
	/**
	 * Constructs an AD* waypoint at a specified position.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @see ARAWaypoint#AStarWaypoint(Position)
	 */
	public ADStarWaypoint(Position position) {
		super(position);
		this.setV(Double.POSITIVE_INFINITY);
	}
	
	/**
	 * Constructs an AD* waypoint at a specified position with a specified
	 * weight of the estimated remaining cost (epsilon).
	 * 
	 * @param position the position in globe coordinates
	 * @param epsilon the weight of the estimated remaining cost (epsilon)
	 * 
	 * @see ARAStarWaypoint#ARAStarWaypoint(Position, double)
	 */
	public ADStarWaypoint(Position position, double epsilon) {
		super(position, epsilon);
		this.setV(Double.POSITIVE_INFINITY);
	}
	
	/**
	 * Gets the parent AD* waypoint of this AD* waypoint.
	 * 
	 * @return the parent AD* waypoint of this AD* waypoint
	 * 
	 * @see ARAStarWaypoint#getParent()
	 */
	@Override
	public ADStarWaypoint getParent() {
		return (ADStarWaypoint) super.getParent();
	}
	
	/**
	 * Gets the estimated previous expansion cost (v-value) of this AD*
	 * waypoint.
	 * 
	 * @return the estimated previous expansion (v-value) cost of this AD*
	 *         waypoint
	 */
	public double getV() {
		return this.v;
	}
	
	/**
	 * Sets the estimated previous expansion cost (v-value) of this AD*
	 * waypoint.
	 * 
	 * @param v the estimated previous expansion cost (v-value) of this AD*
	 *          waypoint
	 */
	public void setV(double v) {
		if (0d > v) {
			throw new IllegalArgumentException("v is less than 0");
		}
		this.v = v;
	}
	
	/**
	 * Indicates whether or not this AD* waypoint is consistent, that is, if
	 * its current estimated cost (g-value) equals the estimated cost of its
	 * last expansion (v-value).
	 * 
	 * @return true if this AD* waypoint is consistent, false otherwise
	 */
	public boolean isConsistent() {
		return this.getG() == this.getV();
	}
	
	/**
	 * Indicates whether or not this AD* waypoint is over-consistent, that is,
	 * if its current estimated cost (g-value) is less than the estimated cost
	 * of its last expansion (v-value). The expansion of an over-consistent
	 * AD* waypoint may potentially lead to an improved solution (propagation
	 * of over-consistency).
	 * 
	 * @return true if this AD* waypoint is over-consistent, false otherwise
	 */
	public boolean isOverConsistent() {
		return this.getG() < this.getV();
	}
	
	/**
	 * Indicates whether or not this AD* waypoint is under-consistent, that
	 * is, if its current estimated cost (g-value) is greater than the
	 * estimated cost of its last expansion (v-value). An under-consistent
	 * state indicates an increase in costs and requires an invalidation of
	 * dependent costs (propagation of under-consistency).
	 * 
	 * @return true if this AD* waypoint is under-consistent, false otherwise
	 */
	public boolean isUnderConsistent() {
		return this.getG() > this.getV();
	}
	
	/**
	 * Makes this AD* waypoint consistent.
	 * 
	 * @see #isConsistent()
	 */
	public void makeConsistent() {
		this.setV(this.getG());
	}
	
	/**
	 * Gets the priority key for the expansion of this AD* waypoint.
	 * The heuristic of the estimated total cost is inflated (weighted) by
	 * epsilon only if this AD* waypoint is consistent or over-consistent.
	 * 
	 * @return the priority key for the expansion of this AD* waypoint
	 * 
	 * @see #getEpsilon()
	 * @see #setEpsilon(double)
	 * @see #isConsistent()
	 * @see #isOverConsistent()
	 * @see #isUnderConsistent()
	 */
	@Override
	public double getKey() {
		double key = Double.POSITIVE_INFINITY;
		
		if (!this.isUnderConsistent()) {
			key = super.getKey(); 
		} else {
			key = this.getV() + this.getH();
		}
		
		return key;
	}

	/**
	 * Compares this AD* waypoint to another waypoint based on their priority
	 * keys for the expansion. If the priority keys for the expansion of both
	 * AD* waypoints are equal, then ties are broken in favor of higher
	 * estimated current costs (g-values) or under-consistent costs (v-values).
	 * If the other waypoint is not an AD* waypoint, then the natural order of
	 * general waypoints applies.
	 * 
	 * @param waypoint the other waypoint
	 * 
	 * @return -1, 0, 1, if this AD* waypoint is less than, equal, or greater,
	 *         respectively, than the other waypoint based on their priority
	 *         keys for the expansion
	 * 
	 * @see ARAStarWaypoint#compareTo(Waypoint)
	 */
	@Override
	public int compareTo(Waypoint waypoint) {
		int compareTo = 0;
		
		if (waypoint instanceof ADStarWaypoint) {
			ADStarWaypoint adsw = (ADStarWaypoint) waypoint;
			compareTo = Double.valueOf(this.getKey()).compareTo(adsw.getKey());
			if (0 == compareTo) {
				if (!this.isUnderConsistent()) {
					// break ties in favor of higher G-values
					compareTo = Double.valueOf(adsw.getG()).compareTo(this.getG());
				} else {
					// break ties in favor of higher V-values
					compareTo = Double.valueOf(adsw.getV()).compareTo(this.getV());
				}
			}
		} else {
			compareTo = super.compareTo(waypoint);
		}
		
		return compareTo;
	}
	
}
