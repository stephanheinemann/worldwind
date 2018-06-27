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
package com.cfar.swim.worldwind.ai.prm.fadprm;

import java.util.HashSet;
import java.util.Set;

import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes a FADPRM waypoint of a trajectory featuring estimates for costs, a
 * list of successors, the density of the waypoint and the parameter beta.
 * FADPRM waypoints are used by FADPRM planners.
 * 
 * @author Henrique Ferreira
 *
 */
public class FADPRMWaypoint extends Waypoint{

	/** the distance/cost between this FADPRM waypoint and the goal waypoint */
	private double cTGoal;

	/** the path desirability of this FADPRM waypoint */
	private double pathDD;

	/** the number of waypoints within a small distance of this waypoint */
	private int density;

	/** the number of the last search that generated this waypoint */
	private int search;

	/** the parameter beta that weights the importance of density and f-value */
	private double beta;

	/** the parameter lambda that weights path desirability and cost */
	private double lambda;
	
	/** the parameter lambda that weights path desirability and cost */
	private double desirability;
	
	/**
	 * @return the desirability
	 */
	public double getDesirability() {
		return desirability;
	}

	/**
	 * @param desirability the desirability to set
	 */
	public void setDesirability(double desirability) {
		this.desirability = desirability;
	}

	/** the parent FADPRM waypoint of this FADPRM waypoint in a trajectory */
	private FADPRMWaypoint pai = null;
	
	/** the estimated remaining cost (h-value) of this A* waypoint */
	private double h;
	
	/** the estimated remaining cost (h-value) of this A* waypoint */
	private double g;
	
	/** the visited neighbors of this A* waypoint in an environment */
	private Set<FADPRMWaypoint> neighbors = new HashSet<>();
	
	/**
	 * Gets the parent FADPRM Waypoint of this FADPRM waypoint.
	 * 
	 * @return the parent FADPRM waypoint of this FADPRM waypoint
	 */
	public FADPRMWaypoint getPai() {
		return pai;
	}

	/**
	 * Sets the parent FADPRM waypoint of this FADPRM waypoint.
	 * 
	 * @param parent the parent FADPRM waypoint of this FADPRM waypoint
	 */
	public void setPai(FADPRMWaypoint pai) {
		this.pai = pai;
}
	/**
	 * Constructs a FADPRM waypoint at a specified position.
	 * 
	 * @param position the position in globe coordinates
	 */
	public FADPRMWaypoint(Position position) {
		super(position);
		this.setCost(Double.POSITIVE_INFINITY);
		this.setCTGoal(Double.POSITIVE_INFINITY);
		this.setPathDD(0);
		lambda = 0.5;
	}

	/**
	 * Gets the path desirability of this FADPRM waypoint.
	 * 
	 * @return the pathDD the path desirability of this FADPRM waypoint.
	 */
	public double getPathDD() {
		return pathDD;
	}

	/**
	 * Sets the path desirability of this FADPRM waypoint.
	 * 
	 * @param pathDD the path desirability to set
	 */
	public void setPathDD(double pathDD) {
		this.pathDD = pathDD;
	}

	/**
	 * Gets the parameter lambda of this FADPRM waypoint.
	 * 
	 * @return the lambda the parameter lambda of this FADPRM waypoint
	 */
	public double getLambda() {
		return lambda;
	}

	/**
	 * Sets the parameter lambda of this FADPRM waypoint.
	 * 
	 * @param lambda the parameter lambda to set
	 */
	public void setLambda(double lambda) {
		this.lambda = lambda;
	}

	/**
	 * Gets the estimated current cost (g-value) of this FADPRM waypoint.
	 * 
	 * @return the estimated current cost (g-value) of this FADPRM waypoint
	 */
	public double getG() {
		return pathDD/(1+lambda*super.getCost());
	}

	/**
	 * Sets the estimated remaining cost (h-value) of this A* waypoint.
	 * 
	 * @param h the estimated remaining cost (h-value) of this A* waypoint
	 */
	public void setG(double g) {
		if (0d > g) {
			throw new IllegalArgumentException("h is less than 0");
		}
		this.g = g;
	}
	
	/**
	 * Gets the estimated remaining cost (h-value) of this FADPRM waypoint.
	 * 
	 * @return the estimated remaining cost (h-value) of this FADPRM waypoint
	 */
	public double getH() {
		return pathDD/(1+lambda*cTGoal);
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
	 * Gets the distance to the goal waypoint.
	 * 
	 * @return the dtGoal the distance to the goal waypoint
	 */
	public double getCTGoal() {
		return cTGoal;
	}

	/**
	 * Sets the distance to the goal waypoint.
	 * 
	 * @param dtGoal the dtGoal to set
	 */
	public void setCTGoal(double cTGoal) {
		this.cTGoal = cTGoal;
	}

	/**
	 * Gets the set of neighbors of this FADPRM waypoint.
	 * 
	 * @return the neighbors the set of neighbors of this waypoint
	 */
	public Set<FADPRMWaypoint> getNeighbors() {
		return this.neighbors;
	}

	
	public void setNeighbors(Set<FADPRMWaypoint> neighbors) {
		this.neighbors.addAll(neighbors);
	}
	/**
	 * Adds one waypoint to the set of successors of this FADPRM waypoint
	 * 
	 * @param waypoint the waypoint to add to the set of successors
	 */
	public void addSuccessor(FADPRMWaypoint waypoint) {
		this.neighbors.add(waypoint);
	}

	/**
	 * Gets the density of this FADPRM waypoint.
	 * 
	 * @return the density of this FADPRM waypoint.
	 */
	public int getDensity() {
		return density;
	}

	/**
	 * Sets the density of this FADPRM waypoint.
	 * 
	 * @param density the density to set
	 */
	public void setDensity(int density) {
		this.density = density;
	}

	/**
	 * Increments the density of this FADPRM waypoint.
	 */
	public void incrementDensity() {
		this.setDensity(this.getDensity() + 1);
	}

	/**
	 * Gets the number of the last search that generated this FADPRM waypoint.
	 * 
	 * @return the search the number of the last search
	 */
	public int getSearch() {
		return search;
	}

	/**
	 * Sets the number of the last search that generated this FADPRM waypoint.
	 * 
	 * @param search the search to set
	 */
	public void setSearch(int search) {
		this.search = search;
	}

	/**
	 * Gets the parameter beta of this FADPRM waypoint.
	 * 
	 * @return the beta the parameter beta
	 */
	public double getBeta() {
		return beta;
	}

	/**
	 * Sets the parameter beta of this FADPRM waypoint.
	 * 
	 * @param beta the parameter beta to set
	 */
	public void setBeta(double beta) {
		this.beta = beta;
	}


	/**
	 * Gets the estimated total cost (f-value) of this FADPRM waypoint.
	 * 
	 * @return the estimated total cost (f-value) of this FADPRM waypoint
	 */
	public double getF() {
		return pathDD/(1+lambda*(this.getCost()+this.getCTGoal()));
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
	 * Compares this FADPRM waypoint to another waypoint based on their keys. If the
	 * first component is equal, then ties are broken in favor of higher estimated
	 * costs to goal (h-values). If the other waypoint is not an FADPRM waypoint,
	 * then the natural order of general waypoints applies.
	 * 
	 * @param waypoint the other waypoint
	 * 
	 * @return 1, 0, -1, if this FADPRM waypoint is less than, equal, or greater,
	 *         respectively, than the other waypoint based on their keys
	 * 
	 * @see Waypoint#compareTo(Waypoint)
	 */
	@SuppressWarnings("deprecation")
	@Override
	public int compareTo(Waypoint waypoint) {
		int compareTo = 0;

		if (waypoint instanceof FADPRMWaypoint) {
			FADPRMWaypoint fadprmw = (FADPRMWaypoint) waypoint;
			compareTo = new Double(this.getKey()).compareTo(fadprmw.getKey());
			if (0 == compareTo) {
				// break ties in favor of higher H-values
				compareTo = new Double(fadprmw.getH()).compareTo(this.getH());
			}
		} else {
			compareTo = super.compareTo(waypoint);
		}

		return -compareTo;
	}

	/**
	 * Clones this FADPRM waypoint without its parent and depiction.
	 * 
	 * @return the clone of this FADPRM waypoint without its parent and depiction
	 * 
	 * @see Object#clone()
	 */
	@Override
	public FADPRMWaypoint clone() {
		FADPRMWaypoint waypoint = (FADPRMWaypoint) super.clone();
		waypoint.setPai(null);
		return waypoint;
	}
}
