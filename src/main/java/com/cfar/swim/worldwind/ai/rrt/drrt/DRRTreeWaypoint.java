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
package com.cfar.swim.worldwind.ai.rrt.drrt;

import java.util.ArrayList;

import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreeWaypoint;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes an DRRT Waypoint by extending the basic RRT waypoint to include a
 * validity field. 
 * 
 * @author Manuel Rosa
 *
 */
public class DRRTreeWaypoint extends RRTreeWaypoint {
	
	/** the validity of this waypoint in the current environment */
	private boolean validity;

	/**
	 * Constructs an DRRT waypoint at a specified position.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @see RRTreeWaypoint#RRTreeWaypoint(Position)
	 */
	public DRRTreeWaypoint(Position position) {
		super(position);
		validity = true;
	}
	
	/**
	 * Gets the parent DRRT waypoint of this DRRT waypoint.
	 * 
	 * @return the parent DRRT waypoint of this DRRT waypoint
	 */
	@Override
	public DRRTreeWaypoint getParent() {
		return (DRRTreeWaypoint) super.getParent();
	}

	/**
	 * Gets the validity of this waypoint in the current environment.
	 * 
	 * @return the validity of this waypoint
	 */
	public boolean isValid() {
		return validity;
	}

	/**
	 * Sets the validity of this waypoint in the current environment.
	 * 
	 * @param validity the validity of this waypoint to set
	 */
	public void setValidity(boolean validity) {
		this.validity = validity;
	}
	
	/**
	 * Gets the predecessors of this DRRT waypoint.
	 * 
	 * @return the list of predecessors of this DRRT waypoint
	 */
	public ArrayList<DRRTreeWaypoint> getPredecessors(){
		ArrayList<DRRTreeWaypoint> predecessors = new ArrayList<DRRTreeWaypoint>();
		
		if(this.getParent()!=null) {
			predecessors.add(this.getParent());
			predecessors.addAll(this.getParent().getPredecessors());
		}
		
		return predecessors; 
			
	}

	/**
	 * Creates a string with the content of all relevant variables of this DRRT
	 * waypoint
	 * 
	 * @return the string with the content of this RRT waypoint
	 */
	public String getInfo() {
		return String.format("( %.6f*, %.6f*, %3.1fm ) g=%.2f h=%.2f f=%.2f hasParent?%b isValid?%b", this.latitude.degrees,
				this.longitude.degrees, this.elevation, this.getG(), this.getH(), this.getF(), this.getParent(), this.isValid());
	}

}
