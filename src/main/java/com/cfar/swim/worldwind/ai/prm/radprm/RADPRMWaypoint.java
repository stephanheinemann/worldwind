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
package com.cfar.swim.worldwind.ai.prm.radprm;

import java.util.Set;

import com.cfar.swim.worldwind.ai.prm.fadprm.FADPRMWaypoint;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes a RADPRM waypoint of a trajectory that extends a FADPRM waypoint by
 * including an utility attribute. This attribute reflects the usefulness of the
 * waypoint in the roadmap.
 * 
 * @author Henrique Ferreira
 *
 */
public class RADPRMWaypoint extends FADPRMWaypoint {

	/** the utility value of this waypoint*/
	private double utility = 0;

	/**
	 * Constructs a RADPRM waypoint at a specified position.
	 * 
	 * @param position the position in globe coordinates
	 */
	public RADPRMWaypoint(Position position) {
		super(position);
	}

	/**
	 * Gets the parent RADPRM waypoint of this RADPRM waypoint.
	 * 
	 * @return the parent FADPRM waypoint of this RADPRM waypoint
	 */
	public RADPRMWaypoint getParent() {
		return (RADPRMWaypoint) super.getParent();
	}
	
	/**
	 * Gets the Set of neighbors of this RADPRM waypoint.
	 * 
	 * @return the neighbors the Set of neighbors of this waypoint
	 */
	@SuppressWarnings("unchecked")
	public Set<? extends RADPRMWaypoint> getNeighbors() {
		return (Set<RADPRMWaypoint>) super.getNeighbors();
	}
	
	/**
	 * Gets the utility value of this waypoint.
	 * 
	 * @return the utility the utility value of this waypoint
	 */
	public double getUtility() {
		return utility;
	}

	/**
	 * Sets the utility value of this waypoint.
	 * 
	 * @param utility the utility value to set
	 */
	public void setUtility(double utility) {
		this.utility = utility;
	}
}
