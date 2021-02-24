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
package com.cfar.swim.worldwind.ai.astar;

import java.util.HashSet;
import java.util.Set;

import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes an A* waypoint of a trajectory featuring estimates for costs and
 * time. A* waypoints are used by A* planners. 
 * 
 * @author Stephan Heinemann
 *
 */
public class AStarWaypoint extends Waypoint {
	
	/** the parent A* waypoint of this A* waypoint in a trajectory */
	private AStarWaypoint parent = null;
	
	/** all parents of this A* waypoint in an environment */
	private Set<AStarWaypoint> parents = new HashSet<>();
	
	/** the estimated remaining cost (h-value) of this A* waypoint */
	private double h;
	
	/**
	 * Constructs an A* waypoint at a specified position.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @see Waypoint#Waypoint(Position)
	 */
	public AStarWaypoint(Position position) {
		super(position);
		this.setG(Double.POSITIVE_INFINITY);
		this.setH(Double.POSITIVE_INFINITY);
	}
	
	/**
	 * Gets the parent A* waypoint of this A* waypoint.
	 * 
	 * @return the parent A* waypoint of this A* waypoint
	 */
	public AStarWaypoint getParent() {
		return parent;
	}
	
	/**
	 * Sets the parent A* waypoint and updates all parents of this A* waypoint.
	 * 
	 * @param parent the parent A* waypoint of this A* waypoint
	 */
	public void setParent(AStarWaypoint parent) {
		this.parent = parent;
		this.parents.add(parent);
	}
	
	/**
	 * Resets all parent A* waypoints of this A* waypoint.
	 */
	public void resetParents() {
		this.parent = null;
		this.parents = new HashSet<>();
	}
	
	/**
	 * Gets all parent A* waypoints of this A* waypoint
	 * 
	 * @return all parent A* waypoints of this A* waypoint
	 */
	public Set<AStarWaypoint> getParents() {
		Set<AStarWaypoint> parents = new HashSet<>();
		parents.addAll(this.parents);
		return parents;
	}
	
	/**
	 * Gets the estimated current cost (g-value) of this A* waypoint.
	 * 
	 * @return the estimated current cost (g-value) of this A* waypoint
	 */
	public double getG() {
		return super.getCost();
	}
	
	/**
	 * Sets the estimated current cost (g-value) of this A* waypoint.
	 * 
	 * @param g the estimated current cost (g-value) of this A* waypoint
	 */
	public void setG(double g) {
		if (0d > g) {
			throw new IllegalArgumentException("g is less than 0");
		}
		super.setCost(g);
	}
	
	/**
	 * Gets the estimated remaining cost (h-value) of this A* waypoint.
	 * 
	 * @return the estimated remaining cost (h-value) of this A* waypoint
	 */
	public double getH() {
		return this.h;
	}
	
	/**
	 * Sets the estimated remaining cost (h-value) of this A* waypoint.
	 * 
	 * @param h the estimated remaining cost (h-value) of this A* waypoint
	 */
	public void setH(double h) {
		if (0d > h) {
			throw new IllegalArgumentException("h is less than 0");
		}
		this.h = h;
	}
	
	/**
	 * Gets the estimated total cost (f-value) of this A* waypoint.
	 * 
	 * @return the estimated total cost (f-value) of this A* waypoint
	 */
	public double getF() {
		return this.getG() + this.getH();
	}
	
	/**
	 * Gets the priority key for the expansion of this A* waypoint.
	 * 
	 * @return the priority key for the expansion of this A* waypoint
	 */
	public double getKey() {
		return this.getF();
	}
	
	/**
	 * Clones this A* waypoint without its parents and depiction.
	 * 
	 * @return the clone of this A* waypoint without its parents and depiction
	 * 
	 * @see Object#clone()
	 */
	@Override
	public AStarWaypoint clone() {
		AStarWaypoint waypoint = (AStarWaypoint) super.clone();
		waypoint.resetParents();
		return waypoint;
	}
	
	/**
	 * Compares this A* waypoint to another waypoint based on their priority
	 * keys for the expansion. If the priority keys for the expansion of both
	 * A* waypoints are equal, then ties are broken in favor of higher
	 * estimated current costs (g-values). If the other waypoint is not an A*
	 * waypoint, then the natural order of general waypoints applies.
	 * 
	 * @param waypoint the other waypoint
	 * 
	 * @return -1, 0, 1, if this A* waypoint is less than, equal, or greater,
	 *         respectively, than the other waypoint based on their priority
	 *         keys for the expansion
	 * 
	 * @see Waypoint#compareTo(Waypoint)
	 */
	@Override
	public int compareTo(Waypoint waypoint) {
		int compareTo = 0;
		
		if (waypoint instanceof AStarWaypoint) {
			AStarWaypoint asw = (AStarWaypoint) waypoint;
			compareTo = Double.valueOf(this.getKey()).compareTo(asw.getKey());
			if (0 == compareTo) {
				// break ties in favor of higher G-values
				compareTo = Double.valueOf(asw.getG()).compareTo(this.getG());
			}
		} else {
			compareTo = super.compareTo(waypoint);
		}
		
		return compareTo;
	}

}
