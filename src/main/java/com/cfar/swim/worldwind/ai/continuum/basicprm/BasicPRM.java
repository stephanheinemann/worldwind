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

import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import com.cfar.swim.worldwind.ai.Planner;
import com.cfar.swim.worldwind.ai.continuum.AbstractSampler;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.planning.Edge;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.PlanningContinuum;
import com.cfar.swim.worldwind.planning.PlanningGrid;
import com.cfar.swim.worldwind.planning.PlanningRoadmap;
import com.cfar.swim.worldwind.planning.Trajectory;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;
/**
 * @author Henrique Ferreira
 *
 */
public class BasicPRM extends AbstractSampler{

	public static final int MAX_NEIGHBORS = 30;

	public static final int MAX_DIST = 30;

	public static final int MAX_SAMPLED_WAYPOINTS = 2000;
	

	private List<BasicPRMWaypoint> waypointList = null;
	
	private List<Edge> edgeList = null;
	
	/** the last computed plan */
	private final LinkedList<BasicPRMWaypoint> plan = new LinkedList<>();

	/** the start waypoint */
	public BasicPRMWaypoint start = null;

	/** the goal waypoint */
	public BasicPRMWaypoint goal = null;

	public BasicPRM(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);	
	}

	/**
	 * Gets the start BasicPRMWaypoint of this BasicPRM planner.
	 * 
	 * @return the start BasicPRMWaypoint of this BasicPRM planner
	 */
	protected BasicPRMWaypoint getStart() {
		return this.start;
	}

	/**
	 * Sets the start BasicPRMWaypoint of this BasicPRM planner.
	 * 
	 * @param start the start BasicPRMWaypoint of this BasicPRM planner
	 */
	protected void setStart(BasicPRMWaypoint start) {
		this.start = start;
	}

	/**
	 * Gets the goal BasicPRMWaypoint of this forward BasicPRM planner.
	 * 
	 * @return the goal BasicPRMWaypoint of this forward BasicPRM planner
	 */
	protected BasicPRMWaypoint getGoal() {
		return this.goal;
	}

	/**
	 * Sets the goal BasicPRMWaypoint of this forward BasicPRM planner.
	 * 
	 * @param goal the goal BasicPRMWaypoint of this forward BasicPRM planner
	 */
	protected void setGoal(BasicPRMWaypoint goal) {		
		this.goal = goal;
	}

	/**
	 * Creates an BasicPRMWaypoint at a specified position.
	 * 
	 * @param position the position
	 * 
	 * @return the BasicPRMWaypoint at the specified position
	 */
	protected BasicPRMWaypoint createWaypoint() {
		BasicPRMWaypoint waypoint = new BasicPRMWaypoint(null);
		while(!this.checkConflict(waypoint)) {
			waypoint = new BasicPRMWaypoint(null);
		}
		return waypoint;
	}

	public void construct() {
		
		int num=0;

		/*Sample BasicPRMWaypoints, collision check*/
		while(num< MAX_SAMPLED_WAYPOINTS) {

			BasicPRMWaypoint waypoint = this.createWaypoint(this.sampleRandomPosition());

			if(!this.checkConflict(waypoint)) {
				this.waypointList.add(waypoint);
				this.connectWaypoint(waypoint);
				num++;
			}
		}

	}

	public void connectWaypoint(BasicPRMWaypoint waypoint) {

		int numConnectedNeighbor=0;

		this.waypointList= this.sortNearest(waypoint);

		for (BasicPRMWaypoint neighbor : this.waypointList) {
			//TODO: create methods for the distance and connected neighbors if statements
			if(super.getEnvironment().getDistance(neighbor, waypoint) < MAX_DIST && numConnectedNeighbor < MAX_NEIGHBORS) {
				if(!this.checkConflict(neighbor, waypoint)) {
					numConnectedNeighbor++;
					this.createEdge(waypoint, neighbor);
				}
			}
		}
	}
	
	public void createEdge (BasicPRMWaypoint wp, BasicPRMWaypoint nb) {
		Edge newedge = new Edge(wp, nb);
		this.edgeList.add(newedge);
	}

	public List<BasicPRMWaypoint> sortNearest(BasicPRMWaypoint waypoint) {

		// sorts the list by increasing distance to waypoint
		Collections.sort(this.waypointList,
				(a, b) -> super.getEnvironment().getDistance(waypoint,
						a) < super.getEnvironment().getDistance(waypoint, b)
				? -1
						: super.getEnvironment().getDistance(waypoint,
								a) == super.getEnvironment()
								.getDistance(waypoint, b) ? 0
										: 1);
		return waypointList;
	}

	/**
	 * Creates an BasicPRMWaypoint at a specified position.
	 * 
	 * @param position the position
	 * 
	 * @return the BasicPRMWaypoint at the specified position
	 */
	protected BasicPRMWaypoint createWaypoint(Position position) {
		return new BasicPRMWaypoint(position);
	}
	
	/**
	 * Clears the waypoints of the current plan.
	 */
	protected void clearPlan() {
		this.plan.clear();
	}
	
	/**
	 * Initializes the planner to plan from an origin to a destination at a
	 * specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 */
	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {

		this.waypointList.clear();
		this.edgeList.clear();
		this.clearPlan();
		
		this.setStart(this.createWaypoint(origin));
		this.getStart().setG(0);
		this.getStart().setEto(etd);
		
		this.setGoal(this.createWaypoint(destination));
	}
	
	/**
	 * Connects a plan of specified BasicPRMWaypoints.
	 * 
	 * @param waypoint the last BasicPRMWaypoint of a computed plan
	 */
	protected void connectPlan(BasicPRMWaypoint waypoint) {
		this.clearPlan();
		
		while ((null != waypoint)) {
			this.addFirstWaypoint(waypoint.clone());
			waypoint = waypoint.getParent();
			if (null != waypoint) {
				// TODO: this is not good enough and environment airdata intervals are required
				waypoint.setTtg(Duration.between(waypoint.getEto(), this.getFirstWaypoint().getEto()));
				waypoint.setDtg(this.getEnvironment().getDistance(waypoint, this.getFirstWaypoint()));
			}
		}
	}
	
	
	/**
	 * Computes the estimated cost of a specified target BasicPRM waypoint when
	 * reached via a specified source BasicPRM waypoint.
	 * 
	 * @param source the source BasicPRM waypoint in globe coordinates
	 * @param target the target BasicPRM waypoint in globe coordinates
	 */
	protected void computeCost(BasicPRMWaypoint source, BasicPRMWaypoint target) {
		Path leg = new Path(source, target);
		Capabilities capabilities = this.getAircraft().getCapabilities();
		Globe globe = this.getEnvironment().getGlobe();
		// TODO: catch IllegalArgumentException (incapable) and exit
		ZonedDateTime end = capabilities.getEstimatedTime(leg, globe, source.getEto());
		
		double cost = this.getEnvironment().getStepCost(
				source, target,
				source.getEto(), end,
				this.getCostPolicy(), this.getRiskPolicy());
		
		if ((source.getG() + cost) < target.getG()) {
			target.setParent(source);
			target.setG(source.getG() + cost);
			target.setEto(end);
		}
	}
	/**
	 * Gets the first BasicPRM waypoint of the current plan.
	 * 
	 * @return the first BasicPRM waypoint of the current plan
	 */
	protected BasicPRMWaypoint getFirstWaypoint() {
		return this.plan.getFirst();
	}
	
	/**
	 * Adds a first BasicPRM waypoint to the current plan.
	 * 
	 * @param waypoint the first BasicPRM waypoint to be added to the current plan
	 */
	protected void addFirstWaypoint(BasicPRMWaypoint waypoint) {
		this.plan.addFirst(waypoint);
	}

	/**
	 * Plans a trajectory from an origin to a destination at a specified
	 * estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @return the planned trajectory from the origin to the destination with
	 *         the estimated time of departure
	 * 
	 * @see Planner#plan(Position, Position, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		this.initialize(origin, destination, etd);
		this.construct();
		return null;
	}

	/**
	 * Indicates whether or not this forward A* planner supports a specified
	 * environment.
	 * 
	 * @param environment the environment
	 * 
	 * @return true if the environment is a planning grid or roadmap,
	 *         false otherwise
	 *         
	 * @see PlanningGrid
	 * @see PlanningRoadmap
	 */
	@Override
	public boolean supports(Environment environment) {
		boolean supports = super.supports(environment);
		
		if (supports) {
			supports = (environment instanceof PlanningContinuum);
		}
		
		return supports;
	}

	/**
	 * @param origin
	 * @param destination
	 * @param waypoints
	 * @param etd
	 * @return
	
	 * @see com.cfar.swim.worldwind.ai.Planner#plan(gov.nasa.worldwind.geom.Position, gov.nasa.worldwind.geom.Position, java.util.List, java.time.ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
		// TODO Auto-generated method stub
		return null;
	}
}

