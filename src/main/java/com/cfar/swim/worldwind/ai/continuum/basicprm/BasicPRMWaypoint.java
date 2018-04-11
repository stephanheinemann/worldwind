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
package com.cfar.swim.worldwind.ai.continuum.basicprm;

import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;

/**
 * @author Henrique Ferreira
 *
 */
public class BasicPRMWaypoint extends Waypoint {
	
	
	/** the component to which the waypoint is connected to*/
	private int component;
	
	/**
	 * Constructs an BasicPRM waypoint at a specified position.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @see Waypoint#Waypoint(Position)
	 */
	public BasicPRMWaypoint(Position position) {
		super(position);
		this.setG(Double.POSITIVE_INFINITY);
	}
	
	/**
	 * Gets the estimated current cost (g-value) of this BasicPRM waypoint.
	 * 
	 * @return the estimated current cost (g-value) of this BasicPRM waypoint
	 */
	public double getG() {
		return super.getCost();
	}
	
	/**
	 * Sets the estimated current cost (g-value) of this BasicPRM waypoint.
	 * 
	 * @param g the estimated current cost (g-value) of this BasicPRM waypoint
	 */
	public void setG(double g) {
		if (0d > g) {
			throw new IllegalArgumentException("g is less than 0");
		}
		super.setCost(g);
	}
	
	public int getComponent() {
		return component;
	}
	
	public void setComponent(int component) {
		this.component = component;
	}
	
	/**
	 * Clones this BasicPRM waypoint without its parent and depiction.
	 * 
	 * @return the clone of this BasicPRM waypoint without its parent and depiction
	 * 
	 * @see Object#clone()
	 */
	@Override
	public BasicPRMWaypoint clone() {
		BasicPRMWaypoint waypoint = (BasicPRMWaypoint) super.clone();
		return waypoint;
	}
	
	/**
	 * Compares this BasicPRM waypoint to another waypoint based on their estimated
	 * total costs (f-values). If the estimated total costs of both A*
	 * waypoints is equal, then ties are broken in favor of higher estimated
	 * current costs (g-values). If the other waypoint is not an BasicPRM waypoint,
	 * then the natural order of general waypoints applies.
	 * 
	 * @param waypoint the other waypoint
	 * 
	 * @return -1, 0, 1, if this BasicPRM waypoint is less than, equal, or greater,
	 *         respectively, than the other waypoint based on their total
	 *         estimated cost
	 * 
	 * @see Waypoint#compareTo(Waypoint)
	 */
	@SuppressWarnings("deprecation")
	@Override
	public int compareTo(Waypoint waypoint) {
		int compareTo = 0;
		
		if (waypoint instanceof BasicPRMWaypoint) {
			BasicPRMWaypoint bprmw = (BasicPRMWaypoint) waypoint;
			compareTo = new Double(this.getG()).compareTo(bprmw.getG());

		} else {
			compareTo = super.compareTo(waypoint);
		}
		
		return compareTo;
	}

	public boolean checkSameComponent(BasicPRMWaypoint neighbor) {
		if(this.component == neighbor.component)
			return true;
		else
			return false;
	}
	
	@Override
	public int hashCode() {
		return super.hashCode();
	}
}
