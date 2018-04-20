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
package com.cfar.swim.worldwind.ai.astar.astar;

import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.ai.AbstractPlanner;
import com.cfar.swim.worldwind.ai.Planner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.geom.precision.PrecisionPosition;
import com.cfar.swim.worldwind.planning.DiscreteEnvironment;
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
	
	/** the start region */
	private Set<PrecisionPosition> startRegion;
	
	/** the goal region */
	private Set<PrecisionPosition> goalRegion;
	
	/** the last computed plan */
	private final LinkedList<AStarWaypoint> plan = new LinkedList<>();
	
	// Manuel and Henrique (fixing errors)
	/** The environment casted to a discrete environment */
	private DiscreteEnvironment discreteEnvironment = null;
	
	/**
	 * @return the discreteEnvironment
	 */
	public DiscreteEnvironment getDiscreteEnvironment() {
		return discreteEnvironment;
	}
	
	/**
	 * @param discreteEnvironment the discreteEnvironment to set
	 */
	public void setDiscreteEnvironment(DiscreteEnvironment discreteEnvironment) {
		this.discreteEnvironment = discreteEnvironment;
	}
	

	/**
	 * Constructs a basic forward A* planner for a specified aircraft and
	 * environment using default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see AbstractPlanner#AbstractPlanner(Aircraft, Environment)
	 */
	public ForwardAStarPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		this.discreteEnvironment = (DiscreteEnvironment) this.getEnvironment();
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
	 * Sets the start region of this forward A* planner.
	 * 
	 * @param startRegion the start region of this forward A* planner
	 */
	protected void setStartRegion(Set<PrecisionPosition> startRegion) {
		this.startRegion = startRegion;
	}
	
	/**
	 * Indicates whether or not a position is within the start region.
	 * 
	 * @param position the position
	 * 
	 * @return true if the position is within the start region, false otherwise
	 */
	protected boolean isInStartRegion(Position position) {
		return this.startRegion.contains(position);
	}
	
	/**
	 * Sets the goal region of this forward A* planner.
	 * 
	 * @param goalRegion the goal region of this forward A* planner
	 */
	protected void setGoalRegion(Set<PrecisionPosition> goalRegion) {
		this.goalRegion = goalRegion;
	}
	
	/**
	 * Indicates whether or not a position is within the goal region.
	 * 
	 * @param position the position
	 * 
	 * @return true if the position is within the goal region, false otherwise
	 */
	protected boolean isInGoalRegion(Position position) {
		return this.goalRegion.contains(position);
	}
	
	/**
	 * Adds an A* waypoint to the expandable A* waypoints.
	 * 
	 * @param waypoint the A* waypoint
	 * 
	 * @return true if the A* waypoint has been added to the expandable
	 *         A* waypoints, false otherwise
	 */
	protected boolean addExpandable(AStarWaypoint waypoint) {
		return this.open.add(waypoint);
	}
	
	/**
	 * Removes an A* waypoint from the expandable A* waypoints.
	 * 
	 * @param waypoint the A* waypoint
	 * 
	 * @return true if the A* waypoint has been removed from the expandable
	 *         A* waypoints, false otherwise
	 */
	protected boolean removeExpandable(AStarWaypoint waypoint) {
		return this.open.remove(waypoint);
	}
	
	/**
	 * Clears the expandable A* waypoints.
	 */
	protected void clearExpandables() {
		this.open.clear();
	}
	
	/**
	 * Determines whether or not an A* waypoint is expandable.
	 * 
	 * @param waypoint the A* waypoint
	 * 
	 * @return true if the A* waypoint is expandable, false otherwise
	 */
	protected boolean isExpandable(AStarWaypoint waypoint) {
		return this.open.contains(waypoint);
	}
	
	/**
	 * Determines whether or not A* waypoints can be expanded.
	 * 
	 * @return true if A* waypoints can be expanded, false otherwise
	 */
	protected boolean canExpand() {
		return !(this.open.isEmpty());
	}
	
	/**
	 * Peeks an A* waypoint from the expandable A* waypoints.
	 * 
	 * @return the peeked A* waypoint from the expandable A* waypoints if any,
	 *         null otherwise
	 */
	protected AStarWaypoint peekExpandable() {
		return this.open.peek();
	}
	
	/**
	 * Polls an A* waypoint from the expandable A* waypoints.
	 * 
	 * @return the polled A* waypoint from the expandable A* waypoints if any,
	 *         null otherwise
	 */
	protected AStarWaypoint pollExpandable() {
		return this.open.poll();
	}
	
	/**
	 * Finds an expandable A* waypoint.
	 * 
	 * @param waypoint the expandable A* waypoint to be found
	 * 
	 * @return the found expandable A* waypoint if any
	 */
	protected Optional<? extends AStarWaypoint>
		findExpandable(AStarWaypoint waypoint) {
		
		return this.open.stream()
				.filter(w -> w.equals(waypoint))
				.findFirst();
	}
	
	/**
	 * Adds an A* waypoint to the expanded A* waypoints.
	 * 
	 * @param waypoint the A* waypoint
	 * 
	 * @return true if the A* waypoint has been added to the expanded
	 *         A* waypoints, false otherwise
	 */
	protected boolean addExpanded(AStarWaypoint waypoint) {
		return this.closed.add(waypoint);
	}
	
	/**
	 * Removes an A* waypoint from the expanded A* waypoints.
	 * 
	 * @param waypoint the A* waypoint
	 * 
	 * @return true if the A* waypoint has been removed from the expanded
	 *         A* waypoints, false otherwise
	 */
	protected boolean removeExpanded(AStarWaypoint waypoint) {
		return this.closed.remove(waypoint);
	}
	
	/**
	 * Clears the expanded A* waypoints.
	 */
	protected void clearExpanded() {
		this.closed.clear();
	}
	
	/**
	 * Determines whether or not an A* waypoint has been expanded.
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
	 * @param waypoint the expanded A* waypoint to be found
	 * 
	 * @return the found expanded A* waypoint, if any
	 */
	protected Optional<? extends AStarWaypoint>
		findExpanded(AStarWaypoint waypoint) {
		
		return this.closed.stream()
				.filter(w -> w.equals(waypoint))
				.findFirst();
	}
	
	/**
	 * Finds the dependent target of an expanded A* source waypoint.
	 * The source waypoint is the parent of the target waypoint.
	 *  
	 * @param source the source A* waypoint
	 * @param target the dependent target A* waypoint to be found
	 * @return the found dependent target A* waypoint, if any
	 */
	protected Optional<? extends AStarWaypoint>
		findDependent(AStarWaypoint source, AStarWaypoint target) {
		Optional<? extends AStarWaypoint> dependent = Optional.empty();
		
		Optional<? extends AStarWaypoint> expanded =
				this.findExpanded(target);
		
		if (expanded.isPresent() && expanded.get().getParent() == source) {
			dependent = expanded;
		} else {
			Optional<? extends AStarWaypoint> expandable =
					this.findExpandable(target);
			if (expandable.isPresent() &&
					expandable.get().getParent() == source) {
				dependent = expandable;
			}
		}
		
		return dependent;
	}
	
	/**
	 * Expands an A* waypoint towards its neighbors in the environment.
	 * 
	 * @param waypoint the A* waypoint to be expanded
	 * 
	 * @return the neighbors of the expanded A* waypoint
	 */
	protected Set<? extends AStarWaypoint> expand(AStarWaypoint waypoint) {
		
	    Set<Position> neighbors = this.getDiscreteEnvironment().getNeighbors(waypoint);

	    //TODO: code below is not applicable to PlanningRoadmap. suggestion
		// (if this.getDiscreteEnvironment() instanceof PlanningGrid).....
	    
	    if(this.getDiscreteEnvironment() instanceof PlanningGrid) {
			// if a start has no neighbors, then it is not a waypoint in the
			// environment and its adjacent waypoints have to be determined for
			// initial expansion
			if (neighbors.isEmpty()) {
				neighbors = this.getDiscreteEnvironment().getAdjacentWaypoints(waypoint);
			}
			
			// expand start region position towards the start
			if (this.isInStartRegion(waypoint.getPrecisionPosition())) {
				neighbors.add(this.getStart());
			}
			
//			// expand a goal region position towards the goal
			if (this.isInGoalRegion(waypoint.getPrecisionPosition())) {
				neighbors.add(this.getGoal());
			}
	    }
	
		this.addExpanded(waypoint);
		
		return neighbors.stream()
				.map(n -> this.createWaypoint(n))
				.collect(Collectors.toSet());
	}
	
	/**
	 * Gets the first A* waypoint of the current plan.
	 * 
	 * @return the first A* waypoint of the current plan
	 */
	protected AStarWaypoint getFirstWaypoint() {
		return this.plan.getFirst();
	}
	
	/**
	 * Adds a first A* waypoint to the current plan.
	 * 
	 * @param waypoint the first A* waypoint to be added to the current plan
	 */
	protected void addFirstWaypoint(AStarWaypoint waypoint) {
		this.plan.addFirst(waypoint);
	}
	
	/**
	 * Clears the waypoints of the current plan.
	 */
	protected void clearWaypoints() {
		this.plan.clear();
	}
	
	/**
	 * Connects a plan of specified A* waypoints.
	 * 
	 * @param waypoint the last A* waypoint of a computed plan
	 */
	protected void connectPlan(AStarWaypoint waypoint) {
		this.clearWaypoints();
		
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
	 * Creates a trajectory of the computed plan.
	 * 
	 * @return the trajectory of the computed plan
	 */
	@SuppressWarnings("unchecked")
	protected Trajectory createTrajectory() {
		return new Trajectory((List<Waypoint>) this.plan.clone());
	}
	
	/**
	 * Updates the open set for an updated A* waypoint.
	 * 
	 * @param waypoint the updated A* waypoint
	 */
	protected void updateSets(AStarWaypoint waypoint) {
		// priority queue requires re-insertion of contained modified object
		if (!this.removeExpandable(waypoint)) {
			waypoint.setH(this.getEnvironment()
					.getNormalizedDistance(waypoint, this.getGoal()));
		}
		this.addExpandable(waypoint);
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
			this.updateSets(target);
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
		this.clearWaypoints();
		this.setStart(this.createWaypoint(origin));
		this.getStart().setG(0);
		this.getStart().setH(this.getEnvironment().getNormalizedDistance(origin, destination));

		this.getStart().setEto(etd);

		this.setGoal(this.createWaypoint(destination));
		this.getGoal().setH(0);
		
		// TODO: code below is not applicable to PlanningRoadmap. suggestion
		if (this.getDiscreteEnvironment() instanceof PlanningGrid) {
			// the adjacent waypoints to the origin
			this.setStartRegion(this.getDiscreteEnvironment().getAdjacentWaypoints(origin)
					.stream()
					.map(PrecisionPosition::new)
					.collect(Collectors.toSet()));
			
			// the adjacent waypoints to the destination
			this.setGoalRegion(this.getDiscreteEnvironment().getAdjacentWaypoints(destination)
					.stream()
					.map(PrecisionPosition::new)
					.collect(Collectors.toSet()));
		}

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
					Optional<? extends AStarWaypoint> visited =
							this.findExpandable(target);
					if (visited.isPresent()) {
						this.updateWaypoint(source, visited.get());
					} else {
						this.updateWaypoint(source, target);
					}
				}
			}
		}
		System.out.println("Open list is empty");
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
		Trajectory trajectory = this.createTrajectory();
		this.revisePlan(trajectory);
		return trajectory;
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
	@Override
	public Trajectory plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
		LinkedList<Waypoint> plan = new LinkedList<>();
		Waypoint currentOrigin = new Waypoint(origin);
		ZonedDateTime currentEtd = etd;
		
		// collect intermediate destinations
		ArrayList<Waypoint> destinations = waypoints.stream()
				.map(Waypoint::new)
				.collect(Collectors.toCollection(ArrayList::new));
		destinations.add(new Waypoint(destination));
		
		// plan and concatenate partial trajectories
		for (Waypoint currentDestination : destinations) {
			if (!(currentOrigin.equals(currentDestination))) {
				// plan partial trajectory
				this.initialize(currentOrigin, currentDestination, currentEtd);
				this.compute();
				Trajectory part = this.createTrajectory();
				
				// append partial trajectory to plan
				if ((!plan.isEmpty()) &&  (!part.isEmpty())) {
					plan.pollLast();
				}
				
				for (Waypoint waypoint : part.getWaypoints()) {
					plan.add(waypoint);
				}
				
				if (plan.peekLast().equals(currentOrigin)) {
					// if no plan could be found, return an empty trajectory
					Trajectory trajectory = new Trajectory();
					this.revisePlan(trajectory);
					return trajectory;
				} else {
					currentOrigin = plan.peekLast();
					currentEtd = currentOrigin.getEto();
				}
			}
		}
		
		Trajectory trajectory = new Trajectory(plan);
		this.revisePlan(trajectory);
		return trajectory;
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
			supports = (environment instanceof PlanningGrid) ||
					(environment instanceof PlanningRoadmap);
		}
		
		return supports;
	}
	
}
