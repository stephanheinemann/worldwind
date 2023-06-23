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
package com.cfar.swim.worldwind.planners.cgs.astar;

import java.awt.Color;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.environments.PlanningGrid;
import com.cfar.swim.worldwind.environments.PlanningRoadmap;
import com.cfar.swim.worldwind.geom.precision.PrecisionPosition;
import com.cfar.swim.worldwind.planners.AbstractPlanner;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.planners.rl.Plot;
import com.cfar.swim.worldwind.planners.rl.QLine;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.util.Identifiable;

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
	
	/** the set of already visited A* waypoints */
	protected Set<AStarWaypoint> visited = new HashSet<>();
	
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
	}
	
	/**
	 * Gets the identifier of this forward A* planner.
	 * 
	 * @return the identifier of this forward A* planner
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return Specification.PLANNER_FAS_ID;
	}
	
	/**
	 * Creates an A* waypoint at a specified position.
	 * 
	 * @param position the position
	 * 
	 * @return the A* waypoint at the specified position
	 */
	protected AStarWaypoint createWaypoint(Position position) {
		AStarWaypoint aswp = null;
		
		/*
		 * Avoid visiting existing positions although a 4D waypoint may very
		 * well revisit an existing position to avoid higher costs. The
		 * computed trajectories shall however be space-optimal but not
		 * necessarily time-optimal. To mitigate this issue, departure slots,
		 * or more generally, waypoint slots could be considered and take into
		 * account the aircraft capabilities appropriately (endurance). This
		 * could realize the concept of holding or loitering.
		 * 
		 * https://github.com/stephanheinemann/worldwind/issues/24
		 */
		
		// only create new waypoints if necessary
		aswp = new AStarWaypoint(position);
		if (this.hasStart() && this.getStart().equals(aswp)) {
			aswp = this.getStart();
		} else if (this.hasGoal() && this.getGoal().equals(aswp)) {
			aswp = this.getGoal();
		}
		
		// avoid duplicating discovered waypoints
		Optional<? extends AStarWaypoint> visitedWaypoint = this.findVisited(aswp);
		if (visitedWaypoint.isPresent()) {
			aswp = visitedWaypoint.get();
		} else {
			this.addVisited(aswp);
		}
		
		return aswp;
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
	 * @param start the start A* waypoint of this forward A* planner
	 */
	protected void setStart(AStarWaypoint start) {
		this.start = start;
	}
	
	/**
	 * Determines whether or not this forward A* planner has a
	 * start waypoint.
	 * 
	 * @return true if this forward A* planner has a start waypoint,
	 *         false otherwise
	 */
	protected boolean hasStart() {
		return (null != this.start);
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
	 * Determines whether or not this forward A* planner has a
	 * goal waypoint.
	 * 
	 * @return true if this forward A* planner has a goal waypoint,
	 *         false otherwise
	 */
	protected boolean hasGoal() {
		return (null != this.goal);
	}
	
	/**
	 * Gets the start region of this forward A* planner.
	 * 
	 * @return the start region of this forward A* planner
	 */
	protected Set<PrecisionPosition> getStartRegion() {
		return this.startRegion;
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
	 * Determines whether or not a position is within the start region.
	 * 
	 * @param position the position
	 * 
	 * @return true if the position is within the start region, false otherwise
	 */
	protected boolean isInStartRegion(Position position) {
		return this.startRegion.contains(position);
	}
	
	/**
	 * Gets the goal region of this forward A* planner.
	 * 
	 * @return the goal region of this forward A* planner
	 */
	protected Set<PrecisionPosition> getGoalRegion() {
		return this.goalRegion;
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
	 * Determines whether or not a position is within the goal region.
	 * 
	 * @param position the position
	 * 
	 * @return true if the position is within the goal region, false otherwise
	 */
	protected boolean isInGoalRegion(Position position) {
		return this.goalRegion.contains(position);
	}
	
	/**
	 * Adds an A* waypoint to the visited set of this forward A* planner.
	 * 
	 * @param waypoint the A* waypoint to be added
	 * 
	 * @return true if the A* waypoint was added to the visited set of this
	 *         forward A* planner, false otherwise
	 */
	protected boolean addVisited(AStarWaypoint waypoint) {
		return this.visited.add(waypoint);
	}
	
	/**
	 * Removes an A* waypoint to the visited set of this forward A* planner.
	 * 
	 * @param waypoint the A* waypoint to be removed
	 * 
	 * @return true if the A* waypoint was removed from the visited set of this
	 *         forward A* planner, false otherwise
	 */
	protected boolean removeVisited(AStarWaypoint waypoint) {
		return this.visited.remove(waypoint);
	}
	
	/**
	 * Clears the visited set of this forward A* planner.
	 */
	protected void clearVisited() {
		this.visited.clear();
	}
	
	/**
	 * Determines whether or not the visited set of this forward A* planner
	 * contains an A* waypoint.
	 * 
	 * @param waypoint the A* waypoint to be tested
	 * 
	 * @return true if the A* waypoint is contained in the visited set of this
	 *         forward A* planner, false otherwise
	 */
	protected boolean containsVisited(AStarWaypoint waypoint) {
		return this.visited.contains(waypoint);
	}
	
	/**
	 * Finds an A* waypoint in the visited set of this forward A* planner, if
	 * present.
	 * 
	 * @param waypoint the A* waypoint to be found
	 * 
	 * @return the found A* waypoint, if present 
	 */
	protected Optional<? extends AStarWaypoint> findVisited(AStarWaypoint waypoint) {
		return this.visited.stream()
				.filter(visitedWaypoint -> visitedWaypoint.equals(waypoint))
				.findFirst();
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
	 * Finds an existing A* waypoint.
	 * 
	 * @param waypoint the existing A* waypoint to be found
	 * 
	 * @return the found existing A* waypoint, if any
	 */
	protected Optional<? extends AStarWaypoint>
		findExisting(AStarWaypoint waypoint) {
		
		Optional<? extends AStarWaypoint> existing = Optional.empty();
		
		Optional<? extends AStarWaypoint> expandable = this.findExpandable(waypoint);
		if (expandable.isPresent()) {
			existing = expandable;
		} else {
			Optional<? extends AStarWaypoint> expanded = this.findExpanded(waypoint);
			if (expanded.isPresent()) {
				existing = expanded;
			}
		}
		
		return existing;
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
			neighbors = this.getEnvironment().getAdjacentWaypointPositions(waypoint);
		}
		
		// create neighborhood of waypoints with precision positions
		Set<AStarWaypoint> neighborhood =
				neighbors.stream()
				.map(n -> this.createWaypoint(n)).collect(Collectors.toSet());
		
		// replace start and goal in neighborhood to avoid duplication
		if (neighborhood.remove(this.getStart())) {
			neighborhood.add(this.getStart());
		}
		if (neighborhood.remove(this.getGoal())) {
			neighborhood.add(this.getGoal());
		}

		// expand start region position towards the start
		if ((!waypoint.equals(this.getStart())) &&
				this.isInStartRegion(waypoint.getPrecisionPosition())) {
			// TODO: possible feasibility / capability issues
			neighborhood.add(this.getStart());
		}

		// expand a goal region position towards the goal
		if ((!waypoint.equals(this.getGoal())) &&
				this.isInGoalRegion(waypoint.getPrecisionPosition())) {
			// TODO: possible feasibility / capability issues
			neighborhood.add(this.getGoal());
		}

		this.addExpanded(waypoint);
		
		return neighborhood;
	}
	
	/**
	 * Connects a plan from a specified A* goal waypoint.
	 * 
	 * @param waypoint the last A* waypoint of a computed plan
	 */
	protected void connectPlan(AStarWaypoint waypoint) {
		this.clearWaypoints();
		// only connect plan from reached goal featuring ETO
		// do not accept exceeded risk policy solutions
		while ((null != waypoint) && waypoint.hasEto() && !waypoint.hasInfiniteCost()) {
			this.getWaypoints().addFirst(waypoint);
			waypoint = waypoint.getParent();
		}
	}
	
	/**
	 * Updates the planner waypoint sets for an updated A* waypoint.
	 * 
	 * @param waypoint the updated A* waypoint
	 */
	protected void updateSets(AStarWaypoint waypoint) {
		// priority queue requires re-insertion for key evaluation
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
		double cost = target.getG();
		ZonedDateTime eto = target.getEto();
		this.computeCost(source, target);
		// update improved costs and break ties with ETOs
		if ((target.getG() < cost) ||
				((target.getG() == cost)
						&& (null != eto) && eto.isAfter(target.getEto()))) {
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
		// TODO: align with AbstractPlanner (computeEto, computeCost)
		Path leg = new Path(source, target);
		Capabilities capabilities = this.getAircraft().getCapabilities();
		Globe globe = this.getEnvironment().getGlobe();
		// TODO: catch CapabilitiesException (incapable) and exit
		ZonedDateTime end = capabilities.getEstimatedTime(leg, globe, source.getEto());
		
		double cost = this.getEnvironment().getStepCost(
				source, target,
				source.getEto(), end,
				this.getCostPolicy(), this.getRiskPolicy());
		
		// consider expansion cost and break ties with travel time
		boolean improvedCost = (source.getG() + cost) < target.getG();
		boolean equalCost = (source.getG() + cost) == target.getG();
		boolean improvedTime = (target.hasEto() && end.isBefore(target.getEto()));
		
		// exceeded risk and parent needs to be known for dynamic repairs
		boolean exceededRisk = (target.hasInfiniteCost())
				&& (Double.POSITIVE_INFINITY == cost);
		
		if (improvedCost || (equalCost && improvedTime) || exceededRisk) {
			target.setParent(source);
			target.setG(source.getG() + cost);
			target.setEto(end);
		}
	}
	
	/**
	 * Initializes this A* planner to plan from an origin to a destination at a
	 * specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 */
	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {
		this.setStart(null);
		this.setGoal(null);
		this.clearVisited();
		this.clearExpandables();
		this.clearExpanded();
		this.clearWaypoints();
		
		this.setStart(this.createWaypoint(origin));
		this.getStart().setG(0);
		this.getStart().setH(this.getEnvironment().getNormalizedDistance(origin, destination));
		this.getStart().setEto(etd);
		this.getStart().setPoi(true);
		
		this.setGoal(this.createWaypoint(destination));
		this.getGoal().setH(0);
		this.getGoal().setPoi(true);
		
		// the adjacent waypoints to the origin
		this.setStartRegion(this.getEnvironment().getAdjacentWaypointPositions(origin)
				.stream()
				.map(PrecisionPosition::new)
				.collect(Collectors.toSet()));
		
		// the adjacent waypoints to the destination
		this.setGoalRegion(this.getEnvironment().getAdjacentWaypointPositions(destination)
				.stream()
				.map(PrecisionPosition::new)
				.collect(Collectors.toSet()));
		
		this.addExpandable(this.getStart());
	}
	
	/**
	 * Computes an A* plan.
	 */
	protected void compute() {
		if (this.getStart().equals(this.getGoal())) {
			this.connectPlan(this.getStart());
			return;
		}
		
		while (this.canExpand()) {
			AStarWaypoint source = this.pollExpandable();
			
			if (source.equals(this.getGoal())) {
				this.connectPlan(source);
				return;
			}
			
			Set<? extends AStarWaypoint> neighbors = this.expand(source);
			
			for (AStarWaypoint target : neighbors) {
				if (!this.isExpanded(target)) {
					this.updateWaypoint(source, target);
				}
			}
		}
	}
	
	/**
	 * Plans a part of a trajectory.
	 * 
	 * @param partIndex the index of the part
	 * 
	 * @return the planned part of a trajectory
	 * 
	 */
	protected Trajectory planPart(int partIndex) {
		this.compute();
		return this.createTrajectory();
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
		Trajectory trajectory = this.planPart(0);
		this.revisePlan(trajectory);
		// Starts plot
		Plot plot = new Plot("Hey", 88.1470000, 88.153000, 0.000001, 44.900000, 45.1000000, 0.00001);
		plot.setPointSize(6);
		plot.setPointShape(Plot.CIRCLE);
		plot.setBackground(Color.white);
		// Prints all discovered points in cyan
		plot.setColor(Color.cyan);
		for (AStarWaypoint entry : visited) {
			plot.addPoint(entry.latitude.degrees, entry.longitude.degrees, "");
			System.out.println("Cost of point " + entry.getDesignator() + ": " + entry.getCost());
		}
		// Prints final plan in black
		plot.setColor(Color.black);
		plot.setConnected(true);
		for (int i=0; i<this.getPlan().size(); i++) {
			plot.addPoint(this.getPlan().get(i).latitude.degrees, this.getPlan().get(i).longitude.degrees, "");
		}
		plot.setConnected(false);
		// Prints start in red
		plot.setColor(Color.red);
		plot.addPoint(origin.latitude.degrees, origin.longitude.degrees, "");
		// Prints goal in green
		plot.setColor(Color.green);
		plot.addPoint(destination.latitude.degrees, destination.longitude.degrees, "");
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
		AStarWaypoint currentOrigin = new AStarWaypoint(origin);
		ZonedDateTime currentEtd = etd;
		
		// collect intermediate destinations
		ArrayList<AStarWaypoint> destinations = waypoints.stream()
				.map(AStarWaypoint::new)
				.collect(Collectors.toCollection(ArrayList::new));
		destinations.add(new AStarWaypoint(destination));
		
		// plan and concatenate partial trajectories
		for (int partIndex = 0; partIndex < destinations.size(); partIndex++) {
			AStarWaypoint currentDestination = destinations.get(partIndex);
			if (!(currentOrigin.equals(currentDestination))) {
				
				/* 
				 * Each part of a multi-part plan has to be computed completely
				 * in order to finalize the ETO of the goal waypoint which
				 * becomes the start waypoint of the next part and the basis
				 * for any subsequent plan revisions. A possible repair of one
				 * part requires the re-computation of all subsequent parts.
				 * This cost-greedy solution does not necessarily result in
				 * optimality with respect to the overall cost of the computed
				 * multi-part plan.
				 * 
				 * https://github.com/stephanheinemann/worldwind/issues/24
				 */
				
				// plan partial trajectory
				this.initialize(currentOrigin, currentDestination, currentEtd);
				
				// connect part to previous part via goal parent for trajectory
				// generation, previous goal and start need to be separate
				// objects for AD* repairs
				if (currentOrigin.hasParent()) {
					this.getStart().setParent(currentOrigin.getParent());
					this.getStart().setCost(currentOrigin.getCost());
				}
				this.planPart(partIndex);
				
				if ((!this.hasWaypoints())
						|| (!this.getWaypoints().getLast().equals(currentDestination))) {
					// if no plan could be found, return an empty trajectory
					Trajectory empty = new Trajectory();
					this.revisePlan(empty);
					return empty;
				} else {
					// revise growing trajectory for each part
					this.revisePlan(this.createTrajectory());
					currentOrigin = this.getGoal();
					currentEtd = currentOrigin.getEto();
				}
			}
		}
		
		return this.createTrajectory();
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
