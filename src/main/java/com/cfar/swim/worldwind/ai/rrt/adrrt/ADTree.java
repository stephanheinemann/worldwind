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
package com.cfar.swim.worldwind.ai.rrt.adrrt;

import java.util.ArrayList;
import java.util.LinkedList;

import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreeWaypoint;
import com.cfar.swim.worldwind.planning.Edge;
import com.cfar.swim.worldwind.planning.Waypoint;

/**
 * Realizes an anytime dynamic tree, it represents a structure only to hold
 * information
 * 
 * @author Manuel Rosa
 *
 */
public class ADTree {

	/** the list of already sampled waypoints */
	private ArrayList<RRTreeWaypoint> waypointList = null;

	/** the list of edges in this tree */
	private ArrayList<Edge> edgeList = null;

	/** the last computed plan */
	private LinkedList<Waypoint> plan = null;

	/** the relative weight of distances to calculate the cost of a waypoint */
	private double distBias;

	/** the relative weight of costs to calculate the cost of a waypoint */
	private double costBias;

	/** the cost value bounding any new solution to be generated */
	private double costBound;

	/**
	 * Empty constructor for default initialization of an anytime dynamic tree.
	 */
	public ADTree() {
		waypointList = new ArrayList<RRTreeWaypoint>();
		edgeList = new ArrayList<Edge>();
		plan = new LinkedList<Waypoint>();
		distBias = 1d;
		costBias = 0d;
		costBound = Double.POSITIVE_INFINITY;
	}

	/**
	 * Full constructor for complete initialization of an anytime dynamic tree.
	 * 
	 * @param waypointList
	 * @param edgeList
	 * @param plan
	 * @param distBias
	 * @param costBias
	 * @param costBound
	 */
	public ADTree(ArrayList<RRTreeWaypoint> waypointList, ArrayList<Edge> edgeList, LinkedList<Waypoint> plan,
			double distBias, double costBias, double costBound) {
		this.waypointList = waypointList;
		this.edgeList = edgeList;
		this.plan = plan;
		this.distBias = distBias;
		this.costBias = costBias;
		this.costBound = costBound;
	}

	/**
	 * @return the waypointList
	 */
	public ArrayList<RRTreeWaypoint> getWaypointList() {
		return waypointList;
	}

	/**
	 * @param waypointList the waypointList to set
	 */
	public void setWaypointList(ArrayList<RRTreeWaypoint> waypointList) {
		this.waypointList = waypointList;
	}

	/**
	 * @return the edgeList
	 */
	public ArrayList<Edge> getEdgeList() {
		return edgeList;
	}

	/**
	 * @param edgeList the edgeList to set
	 */
	public void setEdgeList(ArrayList<Edge> edgeList) {
		this.edgeList = edgeList;
	}

	/**
	 * @return the plan
	 */
	public LinkedList<Waypoint> getPlan() {
		return plan;
	}

	/**
	 * @param plan the plan to set
	 */
	public void setPlan(LinkedList<Waypoint> plan) {
		this.plan = plan;
	}

	/**
	 * @return the distBias
	 */
	public double getDistBias() {
		return distBias;
	}

	/**
	 * @param distBias the distBias to set
	 */
	public void setDistBias(double distBias) {
		this.distBias = distBias;
	}

	/**
	 * @return the costBias
	 */
	public double getCostBias() {
		return costBias;
	}

	/**
	 * @param costBias the costBias to set
	 */
	public void setCostBias(double costBias) {
		this.costBias = costBias;
	}
	
	/**
	 * 
	 * @param distBias
	 * @param costBias
	 */
	public void setBiases(double distBias, double costBias) {
		this.distBias = distBias;
		this.costBias = costBias;
	}

	/**
	 * @return the costBound
	 */
	public double getCostBound() {
		return costBound;
	}

	/**
	 * @param costBound the costBound to set
	 */
	public void setCostBound(double costBound) {
		this.costBound = costBound;
	}

	/**
	 * Clones the information from another ADTree to this one.
	 * 
	 * @param other the other ADTree
	 */
	@SuppressWarnings("unchecked")
	public void clone(ADTree other) {
		this.waypointList = (ArrayList<RRTreeWaypoint>) other.getWaypointList().clone();
		this.edgeList = (ArrayList<Edge>) other.getEdgeList().clone();
		this.plan = (LinkedList<Waypoint>) other.getPlan().clone();
		this.distBias = other.getDistBias();
		this.costBias = other.getCostBias();
	}
}
