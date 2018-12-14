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
package com.cfar.swim.worldwind.ai.prm.rigidprm;

import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes a Rigid PRM waypoint of a trajectory by extending the basic
 * waypoint. Rigid PRM waypoints are the nodes of the graph computed by Rigid
 * PRM planners.
 * 
 * @author Henrique Ferreira
 *
 */
public class RigidPRMWaypoint extends Waypoint {

	/** the validity of this waypoint in the current environment */
	private boolean validity;

	/** the number of neighbors that are connected to this waypoint */
	private int neighbors = 0;
	
	/** the number of times the local planner failed to connect this waypoint to another */
	private int connectionFailures = 0;

	/** the number of the component which this waypoint belongs to */
	private int component = 0;

	/**
	 * @param position
	 */
	public RigidPRMWaypoint(Position position) {
		super(position);
		// TODO Auto-generated constructor stub
	}

	/**
	 * @return the validity
	 */
	public boolean isValidity() {
		return validity;
	}

	/**
	 * @param validity the validity to set
	 */
	public void setValidity(boolean validity) {
		this.validity = validity;
	}

	/**
	 * @return the neighbors
	 */
	public int getNeighbors() {
		return neighbors;
	}

	/**
	 * @param neighbors the neighbors to set
	 */
	public void setNeighbors(int neighbors) {
		this.neighbors = neighbors;
	}

	/**
	 * @param neighbors the neighbors to set
	 */
	public void incrementNeighbors() {
		this.setNeighbors(this.neighbors+1);
	}
	
	/**
	 * @return the connectionFailures
	 */
	public int getConnectionFailures() {
		return connectionFailures;
	}

	/**
	 * @param connectionFailures the connectionFailures to set
	 */
	public void setConnectionFailures(int connectionFailures) {
		this.connectionFailures = connectionFailures;
	}
	
	/**
	 * 
	 */
	public void incrementConnectionFailures() {
		this.setConnectionFailures(this.connectionFailures+1);
	}
	/**
	 * @return the component
	 */
	public int getComponent() {
		return component;
	}

	/**
	 * @param component the component to set
	 */
	public void setComponent(int component) {
		this.component = component;
	}

}
