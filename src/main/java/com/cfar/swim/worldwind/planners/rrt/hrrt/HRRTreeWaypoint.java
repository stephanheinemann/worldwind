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
package com.cfar.swim.worldwind.planners.rrt.hrrt;

import com.cfar.swim.worldwind.planners.rrt.brrt.RRTreeWaypoint;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes a hRRT waypoint of a trajectory planned by hRRT planners.
 * 
 * @author Stephan Heinemann
 *
 */
public class HRRTreeWaypoint extends RRTreeWaypoint {
	
	/** the estimated remaining cost (h-value) of this hRRT waypoint */
	private double h;
	
	/**
	 * Constructs a hRRT waypoint at a specified position.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @see RRTreeWaypoint#RRTreeWaypoint(Position)
	 */
	public HRRTreeWaypoint(Position position) {
		super(position);
		this.setG(Double.POSITIVE_INFINITY);
		this.setH(Double.POSITIVE_INFINITY);
	}
	
	/**
	 * Gets the estimated current cost (g-value) of this hRRT waypoint.
	 * 
	 * @return the estimated current cost (g-value) of this hRRT waypoint
	 */
	public double getG() {
		return super.getCost();
	}
	
	/**
	 * Sets the estimated current cost (g-value) of this hRRT waypoint.
	 * 
	 * @param g the estimated current cost (g-value) of this hRRT waypoint
	 * 
	 * @throws IllegalArgumentException if g is less than 0
	 */
	public void setG(double g) {
		if (0d > g) {
			throw new IllegalArgumentException("g is less than 0");
		}
		super.setCost(g);
	}
	
	/**
	 * Gets the estimated remaining cost (h-value) of this hRRT waypoint.
	 * 
	 * @return the estimated remaining cost (h-value) of this hRRT waypoint
	 */
	public double getH() {
		return this.h;
	}
	
	/**
	 * Sets the estimated remaining cost (h-value) of this hRRT waypoint.
	 * 
	 * @param h the estimated remaining cost (h-value) of this hRRT waypoint
	 * 
	 * @throws IllegalArgumentException if h is less than 0
	 */
	public void setH(double h) {
		if (0d > h) {
			throw new IllegalArgumentException("h is less than 0");
		}
		this.h = h;
	}
	
	/**
	 * Gets the estimated total cost (f-value) of this hRRT waypoint.
	 * 
	 * @return the estimated total cost (f-value) of this hRRT waypoint
	 */
	public double getF() {
		return this.getG() + this.getH();
	}
	
	/**
	 * Gets the priority key for the expansion of this hRRT waypoint.
	 * 
	 * @return the priority key for the expansion of this hRRT waypoint
	 */
	public double getKey() {
		return this.getF();
	}
	
	/**
	 * Compares this hRRT waypoint to another waypoint based on their priority
	 * keys for the expansion. If the priority keys for the expansion of both
	 * hRRT waypoints are equal, then ties are broken in favor of higher
	 * estimated current costs (g-values) and earlier ETOs. If the other
	 * waypoint is not an hRRT waypoint, then the natural order of general
	 * waypoints applies.
	 * 
	 * @param waypoint the other waypoint
	 * 
	 * @return less than 0, 0, greater than 0, if this hRRT waypoint is less
	 *         than, equal, or greater, respectively, than the other waypoint
	 *         based on their priority keys for the expansion
	 * 
	 * @see Waypoint#compareTo(Waypoint)
	 */
	@Override
	public int compareTo(Waypoint waypoint) {
		int compareTo = 0;
		
		if (waypoint instanceof HRRTreeWaypoint) {
			HRRTreeWaypoint asw = (HRRTreeWaypoint) waypoint;
			compareTo = Double.valueOf(this.getKey()).compareTo(asw.getKey());
			if (0 == compareTo) {
				// break ties in favor of higher G-values
				compareTo = Double.valueOf(asw.getG()).compareTo(this.getG());
				if ((0 == compareTo) && (this.hasEto())) {
					// break ties in favor of lower ETOs
					compareTo = this.getEto().compareTo(asw.getEto());
				}
			}
		} else {
			compareTo = super.compareTo(waypoint);
		}
		
		return compareTo;
	}

}
