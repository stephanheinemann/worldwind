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
package com.cfar.swim.worldwind.planners.rrt.brrt;

import java.util.HashSet;
import java.util.Set;

import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes a RRT waypoint of a trajectory planned by RRT planners.
 * 
 * @author Manuel Rosa
 * @author Stephan Heinemann
 *
 */
public class RRTreeWaypoint extends Waypoint {

	/** the parent RRT waypoint of this RRT waypoint in a trajectory */
	private RRTreeWaypoint parent = null;
	
	/** the children RRT waypoints of this RRT waypoint in a tree */
	private Set<RRTreeWaypoint> children = new HashSet<RRTreeWaypoint>();

	/**
	 * Constructs a RRT waypoint at a specified position.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @see Waypoint#Waypoint(Position)
	 */
	public RRTreeWaypoint(Position position) {
		super(position);
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
	 * Removes the parent RRT waypoint of this RRT waypoint.
	 */
	public void removeParent() {
		this.parent = null;
	}
	
	/**
	 * Determines whether or not this RRT waypoint has a parent.
	 * 
	 * @return true if this RRT waypoint has a parent, false otherwise
	 */
	public boolean hasParent() {
		return (null != this.parent);
	}
	
	/**
	 * Adds a child to this RRT waypoint.
	 * 
	 * @param child the child to be added to this RRT waypoint
	 */
	public void addChild(RRTreeWaypoint child) {
		this.children.add(child);
	}
	
	/**
	 * Removes a child from this RRT waypoint.
	 * 
	 * @param child the child to be removed from this RRT waypoint
	 */
	public void removeChild(RRTreeWaypoint child) {
		this.children.remove(child);
	}
	
	/**
	 * Clears all children of this RRT waypoint.
	 */
	public void clearChildren() {
		this.children.clear();
	}
	
	/**
	 * Determines whether or not this RRT waypoint has children.
	 * 
	 * @return true if this RRT waypoint has children, false otherwise
	 */
	public boolean hasChildren() {
		return (0 < this.children.size());
	}
	
	/**
	 * Gets the children of this RRT waypoint.
	 * 
	 * @return the children of this RRT waypoint.
	 */
	public Iterable<RRTreeWaypoint> getChidren() {
		return this.children;
	}
	
}
