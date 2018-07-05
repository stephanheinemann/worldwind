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
package com.cfar.swim.worldwind.ai.prm.faprm;

import java.util.HashSet;
import java.util.Set;

import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes a FAPRM waypoint of a trajectory featuring estimates for costs, a
 * list of neighbors, the parameter beta and density.
 * 
 * @author Henrique Ferreira
 *
 */
public class FAPRMWaypoint extends Waypoint {

	/** the distance between this FAPRM waypoint and the goal waypoint */
	private double distanceToGoal;

	/** the neighbors of this FAPRM waypoint (connectable waypoints) */
	private Set<FAPRMWaypoint> neighbors = new HashSet<>();

	/** the number of waypoints within a small distance of this waypoint */
	private int density;

	/** the parameter beta that weights the importance of density and f-value */
	private double beta;

	/** the parent FAPRM waypoint of this FAPRM waypoint in a trajectory */
	private FAPRMWaypoint parent = null;

	/**
	 * Constructs a FAPRM waypoint at a specified position.
	 * 
	 * @param position the position in globe coordinates
	 */
	public FAPRMWaypoint(Position position) {
		super(position);
		this.setCost(Double.POSITIVE_INFINITY);
		this.setDistanceToGoal(Double.POSITIVE_INFINITY);
	}

	/**
	 * Gets the normalized distance to the goal waypoint.
	 * 
	 * @return the distanceToGoal the distance to the goal waypoint
	 */
	public double getDistanceToGoal() {
		return distanceToGoal;
	}

	/**
	 * Sets the normalized distance to the goal waypoint.
	 * 
	 * @param distanceToGoal the distanceToGoal to set
	 */
	public void setDistanceToGoal(double distanceToGoal) {
		this.distanceToGoal = distanceToGoal;
	}

	/**
	 * Gets the Set of neighbors of this FAPRM waypoint.
	 * 
	 * @return the neighbors the Set of neighbors of this waypoint
	 */
	public Set<? extends FAPRMWaypoint> getNeighbors() {
		return this.neighbors;
	}

	/**
	 * Sets the Set of neighbors of this FAPRM waypoint.
	 * 
	 * @param neighbors the Set of neighbors to set
	 */
	public void setNeighbors(Set<FAPRMWaypoint> neighbors) {
		this.neighbors.addAll(neighbors);
	}

	/**
	 * Adds one waypoint to the Set of neighbors of this FAPRM waypoint
	 * 
	 * @param waypoint the waypoint to add to the Set of neighbors
	 */
	public void addNeighbor(FAPRMWaypoint waypoint) {
		this.neighbors.add(waypoint);
	}

	/**
	 * Gets the density of this FAPRM waypoint.
	 * 
	 * @return the density of this FAPRM waypoint.
	 */
	public int getDensity() {
		return density;
	}

	/**
	 * Sets the density of this FAPRM waypoint.
	 * 
	 * @param density the density to set
	 */
	public void setDensity(int density) {
		this.density = density;
	}

	/**
	 * Increments the density of this FAPRM waypoint.
	 */
	public void incrementDensity() {
		this.setDensity(this.getDensity() + 1);
	}

	/**
	 * Gets the parameter beta of this FAPRM waypoint.
	 * 
	 * @return the beta the parameter beta
	 */
	public double getBeta() {
		return beta;
	}

	/**
	 * Sets the parameter beta of this FAPRM waypoint.
	 * 
	 * @param beta the parameter beta to set
	 */
	public void setBeta(double beta) {
		this.beta = beta;
	}

	/**
	 * Gets the parent FAPRM waypoint of this FAPRM waypoint.
	 * 
	 * @return the parent FAPRM waypoint of this FAPRM waypoint
	 */
	public FAPRMWaypoint getParent() {
		return parent;
	}

	/**
	 * Sets the parent FAPRM waypoint of this FAPRM waypoint.
	 * 
	 * @param parent the parent FAPRM waypoint of this FAPRM waypoint
	 */
	public void setParent(FAPRMWaypoint parent) {
		this.parent = parent;
	}

	/**
	 * Gets the estimated current cost (g-value) of this FAPRM waypoint.
	 * 
	 * @return the estimated current cost (g-value) of this FAPRM waypoint
	 */
	public double getG() {
		return 1 / (1 + super.getCost());
	}

	/**
	 * Gets the estimated total cost (f-value) of this FAPRM waypoint.
	 * 
	 * @return the estimated total cost (f-value) of this FAPRM waypoint
	 */
	public double getF() {
		return 1 / (1 + (this.getCost() + this.getDistanceToGoal()));
	}

	/**
	 * Gets the first component of the key used to sort the open priority queue of
	 * the FADPRM planner.
	 * 
	 * @return the value of the first component of the key
	 */
	public double getKey() {
		return (1 - beta) / density + beta * this.getF();
	}

	/**
	 * Compares this FAPRM waypoint to another waypoint based on their keys. If the
	 * first component is equal, then ties are broken in favor of higher current
	 * costs (g-values). If the other waypoint is not an FAPRM waypoint, then the
	 * natural order of general waypoints applies.
	 * 
	 * @param waypoint the other waypoint
	 * 
	 * @return 1, 0, -1, if this FAPRM waypoint is less than, equal, or greater,
	 *         respectively, than the other waypoint based on their keys
	 * 
	 * @see Waypoint#compareTo(Waypoint)
	 */
	@SuppressWarnings("deprecation")
	@Override
	public int compareTo(Waypoint waypoint) {
		// TODO: changed 2nd component of the key to G
		int compareTo = 0;

		if (waypoint instanceof FAPRMWaypoint) {
			FAPRMWaypoint fadprmw = (FAPRMWaypoint) waypoint;
			compareTo = new Double(this.getKey()).compareTo(fadprmw.getKey());
			if (0 == compareTo) {
				// break ties in favor of higher G-values
				compareTo = new Double(fadprmw.getG()).compareTo(this.getG());
			}
		} else {
			compareTo = super.compareTo(waypoint);
		}

		return -compareTo;
	}

	/**
	 * Clones this FAPRM waypoint without its parent and depiction.
	 * 
	 * @return the clone of this FAPRM waypoint without its parent and depiction
	 * 
	 * @see Object#clone()
	 */
	@Override
	public FAPRMWaypoint clone() {
		FAPRMWaypoint waypoint = (FAPRMWaypoint) super.clone();
		waypoint.setParent(null);
		return waypoint;
	}
}