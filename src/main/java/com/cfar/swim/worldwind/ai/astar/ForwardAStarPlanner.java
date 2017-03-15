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

import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.ai.AbstractPlanner;
import com.cfar.swim.worldwind.ai.Planner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.geom.precision.PrecisionPosition;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.PlanningGrid;
import com.cfar.swim.worldwind.planning.PlanningRoadmap;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;

/**
 * Realizes a basic forward A* planner that plans a trajectory of an aircraft
 * in an environment considering a local cost and risk policy.
 * 
 * @author Stephan Heinemann
 *
 */
public class ForwardAStarPlanner extends AbstractPlanner {
	
	/** the priority queue of expandable A* waypoints */
	protected PriorityQueue<AStarWaypoint> open = new PriorityQueue<>();
	
	/** the set of expanded A* waypoints */
	protected Set<AStarWaypoint> closed = new HashSet<>();
	
	/** the start A* waypoint */
	private AStarWaypoint start = null;
	
	/** the goal A* waypoint */
	private AStarWaypoint goal = null;
	
	/** the goal region */
	private Set<PrecisionPosition> goalRegion;
	
	/** the last computed plan */
	private final LinkedList<AStarWaypoint> plan = new LinkedList<>();
	
	/**
	 * Constructs a basic forward A* planner for a specified aircraft and
	 * environment using default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 */
	public ForwardAStarPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}
	
	/**
	 * Creates an A* waypoint at a specified position.
	 * 
	 * @param position the position
	 * 
	 * @return the A* waypoint at the specified position
	 */
	protected AStarWaypoint createWaypoint(Position position) {
		return new AStarWaypoint(position);
	}
	
	/**
	 * Gets the start A* waypoint of this forward A* planner.
	 * 
	 * @return the start A* waypoint of this forward A* planner
	 */
	protected AStarWaypoint getStart() {
		return this.start;
	}
	
	/**
	 * Sets the start A* waypoint of this forward A* planner.
	 * 
	 * @param start the start waypoint of this forward A* planner
	 */
	protected void setStart(AStarWaypoint start) {
		this.start = start;
	}
	
	/**
	 * Gets the goal A* waypoint of this forward A* planner.
	 * 
	 * @return the goal A* waypoint of this forward A* planner
	 */
	protected AStarWaypoint getGoal() {
		return this.goal;
	}
	
	/**
	 * Sets the goal A* waypoint of this forward A* planner.
	 * 
	 * @param goal the goal A* waypoint of this forward A* planner
	 */
	protected void setGoal(AStarWaypoint goal) {
		this.goal = goal;
	}
	
	/**
	 * Sets the goal region of this forward A* planner.
	 * 
	 * @param goalRegion the goal region of this forward A* planner
	 */
	protected void setGoalRegion(Set<PrecisionPosition> goalRegion) {
		this.goalRegion = goalRegion;
	}
	
	protected boolean isInGoalRegion(Position position) {
		return this.goalRegion.contains(position);
	}
	
	protected void addExpandable(AStarWaypoint waypoint) {
		this.open.add(waypoint);
	}
	
	protected void removeExpandable(AStarWaypoint waypoint) {
		this.open.remove(waypoint);
	}
	
	protected void clearExpandables() {
		this.open.clear();
	}
	
	protected boolean isExpandable(AStarWaypoint waypoint) {
		return this.open.contains(waypoint);
	}
	
	protected boolean canExpand() {
		return !(this.open.isEmpty());
	}
	
	protected AStarWaypoint peekExpandable() {
		return this.open.peek();
	}
	
	protected AStarWaypoint pollExpandable() {
		return this.open.poll();
	}
	
	protected AStarWaypoint findExpandable(AStarWaypoint waypoint) {
		return this.open.stream()
				.filter(s -> s.equals(waypoint))
				.findFirst().get();
	}
	
	protected void addExpanded(AStarWaypoint waypoint) {
		this.closed.add(waypoint);
	}
	
	protected void clearExpanded() {
		this.closed.clear();
	}
	
	/**
	 * Determines if an A* waypoint has been expanded.
	 * 
	 * @param waypoint the A* waypoint
	 * 
	 * @return true if the A* waypoint has been expanded, false otherwise
	 */
	protected boolean isExpanded(AStarWaypoint waypoint) {
		return this.closed.contains(waypoint);
	}
	
	/**
	 * Finds an expanded A* waypoint.
	 * 
	 * @param waypoint the A* waypoint to be found
	 * 
	 * @return the found expanded A* waypoint
	 */
	protected AStarWaypoint findExpanded(AStarWaypoint waypoint) {
		return this.closed.stream()
				.filter(s -> s.equals(waypoint))
				.findFirst().get();
	}
	
	/**
	 * Expands an A* waypoint towards its neighbors in the environment.
	 * 
	 * @param waypoint the A* waypoint to be expanded
	 * 
	 * @return the neighbors of the expanded A* waypoint
	 */
	protected Set<? extends AStarWaypoint> expand(AStarWaypoint waypoint) {
		Set<Position> neighbors = this.getEnvironment().getNeighbors(waypoint);
		// if a start has no neighbors, then it is not a waypoint in the
		// environment and its adjacent waypoints have to be determined for
		// initial expansion
		if (neighbors.isEmpty()) {
			neighbors = this.getEnvironment().getAdjacentWaypoints(waypoint);
		}
		// expand a goal region position towards the goal
		if (this.isInGoalRegion(waypoint.getPrecisionPosition())) {
			neighbors.add(this.getGoal());
		}
		
		this.addExpanded(waypoint);
		
		return neighbors.stream()
				.map(n -> this.createWaypoint(n))
				.collect(Collectors.toSet());
	}
	
	/**
	 * Connects a plan of specified A* waypoints.
	 * 
	 * @param waypoint the last A* waypoint of a computed plan
	 */
	protected void connectPlan(AStarWaypoint waypoint) {
		this.plan.clear();
		
		while ((null != waypoint)) {
			this.plan.addFirst(waypoint.clone());
			waypoint = waypoint.getParent();
			if (null != waypoint) {
				// TODO: this is not good enough and environment airdata intervals are required
				waypoint.setTtg(Duration.between(waypoint.getEto(), this.plan.getFirst().getEto()));
				waypoint.setDtg(this.getEnvironment().getDistance(waypoint, this.plan.getFirst()));
			}
		}
		
		this.revisePlan(this.createTrajectory());
	}
	
	/**
	 * Creates a trajectory of the computed plan.
	 * 
	 * @return the trajectory of the computed plan
	 */
	@SuppressWarnings("unchecked")
	protected Trajectory createTrajectory() {
		return new Trajectory((List<Waypoint>) this.plan.clone());
	}
	
	/**
	 * Updates the estimated cost of a specified target A* waypoint when
	 * reached via a specified source A* waypoint.
	 * 
	 * @param source the source A* waypoint in globe coordinates
	 * @param target the target A* waypoint in globe coordinates
	 */
	protected void updateWaypoint(AStarWaypoint source, AStarWaypoint target) {
		double gOld = target.getG();
		this.computeCost(source, target);
		if (target.getG() < gOld) {
			if (!this.isExpandable(target)) {
				target.setH(this.getEnvironment().getNormalizedDistance(target, this.getGoal()));
				this.addExpandable(target);
			} else {
				// priority queue requires re-insertion of modified object
				this.removeExpandable(target);
				this.addExpandable(target);
			}
		}
	}
	
	/**
	 * Computes the estimated cost of a specified target A* waypoint when
	 * reached via a specified source A* waypoint.
	 * 
	 * @param source the source A* waypoint in globe coordinates
	 * @param target the target A* waypoint in globe coordinates
	 */
	protected void computeCost(AStarWaypoint source, AStarWaypoint target) {
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
	 * Initializes the planner to plan from an origin to a destination at a
	 * specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 */
	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {
		this.clearExpandables();
		this.clearExpanded();
		this.plan.clear();
		
		this.setStart(this.createWaypoint(origin));
		this.getStart().setG(0);
		this.getStart().setH(this.getEnvironment().getNormalizedDistance(origin, destination));
		this.getStart().setEto(etd);
		
		this.setGoal(this.createWaypoint(destination));
		this.getGoal().setH(0);
		// if a goal is not a waypoint in the environment, then its
		// goal region has to be determined for the final expansion
		this.setGoalRegion(this.getEnvironment().getAdjacentWaypoints(destination)
				.stream()
				.map(PrecisionPosition::new)
				.collect(Collectors.toSet()));
		
		this.addExpandable(this.getStart());
	}
	
	/**
	 * Computes a plan.
	 */
	protected void compute() {
		while (this.canExpand()) {
			AStarWaypoint source = this.pollExpandable();
			
			if (source.equals(this.getGoal())) {
				this.connectPlan(source);
				return;
			}
			
			Set<? extends AStarWaypoint> neighbors = this.expand(source);
			
			for (AStarWaypoint target : neighbors) {
				if (!this.isExpanded(target)) {
					if (this.isExpandable(target)) {
						AStarWaypoint visited = this.findExpandable(target);
						this.updateWaypoint(source, visited);
					} else {
						this.updateWaypoint(source, target);
					}
				}
			}
		}
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
		this.compute();
		return this.createTrajectory();
	}
	
	/**
	 * Plans a trajectory from an origin to a destination along waypoints at a
	 * specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param waypoints the waypoints in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @return the planned trajectory from the origin to the destination along
	 *         the waypoints with the estimated time of departure
	 * 
	 * @see Planner#plan(Position, Position, List, ZonedDateTime)
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Trajectory plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
		LinkedList<AStarWaypoint> plan = new LinkedList<AStarWaypoint>();
		Waypoint currentOrigin = new Waypoint(origin);
		ZonedDateTime currentEtd = etd;
		
		ArrayList<Waypoint> destinations = waypoints
				.stream().map(Waypoint::new).collect(Collectors.toCollection(ArrayList::new));
		destinations.add(new Waypoint(destination));
		Iterator<Waypoint> destinationsIterator = destinations.iterator();
		
		while (destinationsIterator.hasNext()) {
			Waypoint currentDestination = destinationsIterator.next();
			
			if (!(currentOrigin.equals(currentDestination))) {
				Trajectory part = this.plan(currentOrigin, currentDestination, currentEtd);
				
				for (AStarWaypoint waypoint : (List<AStarWaypoint>) part.getWaypoints()) {
					if ((!plan.isEmpty()) &&  (null == waypoint.getParent())) {
						plan.pollLast();
						waypoint.setParent(plan.peekLast());
					}
					plan.add(waypoint);
				}
				
				if (plan.peekLast().equals(currentOrigin)) {
					// if no plan could be found, return an empty trajectory
					this.plan.clear();
					return new Trajectory();
				} else {
					currentOrigin = plan.peekLast();
					currentEtd = currentOrigin.getEto();
				}
			}
		}
		
		this.plan.clear();
		//this.plan.addAll(plan);
		return new Trajectory(plan/*this.plan.clone()*/);
	}

	/**
	 * Post-processes a previously expanded waypoint upon its own expansion.
	 * This method is intended to be overridden by lazy evaluation algorithms.
	 * 
	 * @param waypoint the expanded waypoint
	 */
	protected void setWaypoint(Waypoint waypoint) {}
	
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
			supports = (environment instanceof PlanningGrid) ||
					(environment instanceof PlanningRoadmap);
		}
		
		return supports;
	}
	
}
