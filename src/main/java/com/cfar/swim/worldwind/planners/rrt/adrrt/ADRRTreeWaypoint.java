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
package com.cfar.swim.worldwind.planners.rrt.adrrt;

import com.cfar.swim.worldwind.planners.rrt.arrt.ARRTreeWaypoint;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes an ADRRT waypoint of a trajectory planned by ADRRT planners.
 * 
 * @author Stephan Heinemann
 *
 */
public class ADRRTreeWaypoint extends ARRTreeWaypoint {
	
	/** determines whether or not this ADRRT waypoint is valid */
	private boolean isValid = true;
	
	/**
	 * Constructs an ADRRT waypoint at a specified position.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @see ARRTreeWaypoint#ARRTreeWaypoint(Position)
	 */
	public ADRRTreeWaypoint(Position position) {
		super(position);
	}
	
	/**
	 * Gets the parent ADRRT waypoint of this ADRRT waypoint.
	 * 
	 * @return the parent ADRRT waypoint of this ADRRT waypoint
	 * 
	 * @see ARRTreeWaypoint#getParent()
	 */
	@Override
	public ADRRTreeWaypoint getParent() {
		return (ADRRTreeWaypoint) super.getParent();
	}
	
	/**
	 * Determines whether or not this ADRRT waypoint is valid.
	 * 
	 * @return true if this ADRRT waypoint is valid, false otherwise
	 */
	public boolean isValid() {
		return this.isValid;
	}
	
	/**
	 * Determines whether or not this ADRRT waypoint is invalid.
	 * 
	 * @return true if this ADRRT waypoint is invalid, false otherwise
	 */
	public boolean isInvalid() {
		return !this.isValid;
	}
	
	/**
	 * Sets whether or not this ADRRT waypoint is valid.
	 * 
	 * @param isValid the validity of this ADRRT waypoint to be set
	 */
	public void setValid(boolean isValid) {
		this.isValid = isValid;
	}
	
}
