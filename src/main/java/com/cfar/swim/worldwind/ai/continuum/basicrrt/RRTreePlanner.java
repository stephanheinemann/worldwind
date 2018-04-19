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
package com.cfar.swim.worldwind.ai.continuum.basicrrt;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.ai.AbstractPlanner;
import com.cfar.swim.worldwind.ai.Planner;
import com.cfar.swim.worldwind.ai.continuum.AbstractSampler;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.PlanningContinuum;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;

/**
 * Realizes a basic RRT planner that plans a trajectory of an aircraft in an
 * environment considering a local cost and risk policy. The origin of the tree
 * is the departure position and it is grown until the goal is reached.
 * 
 * @author Manuel Rosa
 *
 */
public class RRTreePlanner extends AbstractSampler {

	/** the radius of the sphere defining the goal region */
	private static final double GOAL_THRESHOLD = 5d; // meters?

	/** the maximum number of sampling iterations */
	private final int MAX_ITER;

	/** the maximum distance to extend a waypoint in the tree */
	private final double EPSILON;

	/** the bias of the sampling algorithm towards goal */
	private final int BIAS;

	// ---------- VARIABLES ----------

	/** the start RRT waypoint */
	private RRTreeWaypoint start = null;

	/** the goal RRT waypoint */
	private RRTreeWaypoint goal = null;

	/** the newest waypoint in the tree */
	private RRTreeWaypoint waypointNew = null;

	/** the last computed plan */
	private final LinkedList<RRTreeWaypoint> plan = new LinkedList<>();

	/** the expanding strategy for the planner */
	private Strategy strategy = Strategy.EXTEND;

	// ---------- CONSTRUCTORS ----------

	/**
	 * Constructs a basic RRT planner for a specified aircraft and environment using
	 * default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see AbstractPlanner#AbstractPlanner(Aircraft, Environment)
	 */
	public RRTreePlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		EPSILON = 250d;
		BIAS = 5;
		MAX_ITER = 3_000;
	}

	/**
	 * Constructs a basic RRT planner for a specified aircraft and environment using
	 * default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * @param epsilon the maximum distance to extend a waypoint in the tree
	 * @param bias the bias of the sampling algorithm towards goal
	 * @param maxIter the maximum number of sampling iterations
	 * 
	 * @see AbstractPlanner#AbstractPlanner(Aircraft, Environment)
	 */
	public RRTreePlanner(Aircraft aircraft, Environment environment, double epsilon, int bias, int maxIter) {
		super(aircraft, environment);
		EPSILON = epsilon;
		BIAS = bias;
		MAX_ITER = maxIter;
	}

	// ---------- Setters and Getters ----------
	
//	/**
//	 * Gets the list of already sampled waypoints
//	 * 
//	 * @return the waypointList
//	 * 
//	 * @see AbstractSampler#getWaypointList
//	 */
//	@SuppressWarnings("unchecked")
//	@Override
//	public List<RRTreeWaypoint> getWaypointList() {
//		return (List<RRTreeWaypoint>) super.getWaypointList();
//	}
//
//	/**
//	 * Sets the list of waypoints previously sampled
//	 * 
//	 * @param waypointList the waypointList to set
//	 * 
//	 * @see AbstractSampler#setWaypointList
//	 */
//	@SuppressWarnings("unchecked")
//	@Override
//	public void setWaypointList(List<? extends SampledWaypoint> waypointList) {
//		super.setWaypointList((List<RRTreeWaypoint>) waypointList);
//	}

	/**
	 * Gets the start RRT waypoint of this RRT planner.
	 * 
	 * @return the start RRT waypoint of this RRT planner
	 */
	public RRTreeWaypoint getStart() {
		return start;
	}

	/**
	 * Sets the start RRT waypoint of this RRT planner.
	 * 
	 * @param start the start waypoint of this RRT planner
	 */
	public void setStart(RRTreeWaypoint start) {
		this.start = start;
	}

	/**
	 * Gets the goal RRT waypoint of this RRT planner.
	 * 
	 * @return the goal RRT waypoint of this RRT planner
	 */
	public RRTreeWaypoint getGoal() {
		return goal;
	}

	/**
	 * Sets the goal RRT waypoint of this RRT planner.
	 * 
	 * @param goal the goal waypoint of this RRT planner
	 */
	public void setGoal(RRTreeWaypoint goal) {
		this.goal = goal;
	}

	/**
	 * Gets the newest RRT waypoint added to the tree.
	 * 
	 * @return the waypointNew the newest waypoint added to the tree
	 */
	public RRTreeWaypoint getWaypointNew() {
		return waypointNew;
	}

	/**
	 * Sets the newest RRT waypoint added to the tree.
	 * 
	 * @param waypointNew the newest waypoint added to the tree
	 */
	public void setWaypointNew(RRTreeWaypoint waypointNew) {
		this.waypointNew = waypointNew;
	}

	/**
	 * Gets the strategy for expansion of this planner
	 * 
	 * @return the strategy for expansion step
	 */
	public Strategy getStrategy() {
		return strategy;
	}

	/**
	 * Sets the strategy for expansion of this planner
	 * 
	 * @param strategy the strategy for expansion step
	 */
	public void setStrategy(Strategy strategy) {
		this.strategy = strategy;
	}
	
	/**
	 * Gets the maximum number of iterations for the planner to attempt to connect
	 * to goal
	 * 
	 * @return the maximum number of sampling iterations
	 */
	public int getMAX_ITER() {
		return MAX_ITER;
	}
	
	/**
	 * Gets the maximum distance to extend a waypoint in the tree
	 * 
	 * @return the maximum distance to extend
	 */
	public double getEPSILON() {
		return EPSILON;
	}
	
	/**
	 * Gets the bias of the sampling algorithm towards goal
	 * 
	 * @return the bias of the sampling algorithm
	 */
	public int getBIAS() {
		return BIAS;
	}
	
	// ---------- PROTECTED METHODS ----------
	
	/**
	 * Clears the waypoints in the waypoint list and in the plan
	 */
	protected void clearWaypoints() {
		this.getWaypointList().clear();
		this.plan.clear();
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
	 * Creates a new waypoint from a random position sampled from a uniformed
	 * distribution over the environment space
	 * 
	 * @return waypoint the RRTreeWaypoint sampled
	 */
	protected RRTreeWaypoint sampleRandom() {
		return new RRTreeWaypoint(super.sampleRandomPosition());
	}

	/**
	 * Samples a new waypoint from the environment with a given bias to the goal
	 * 
	 * @param bias the percentage value of bias to sample the goal
	 * 
	 * @return waypoint the RRTreeWaypoint sampled
	 */
	protected RRTreeWaypoint sampleBiased(int bias) {
		RRTreeWaypoint waypoint;
		// Is there any problem in using the random package?
		int rand = new Random().nextInt(100 - 1) + 1;

		waypoint = (rand <= bias) ? this.getGoal() : this.sampleRandom();

		return waypoint;
	}

	/**
	 * Connects the tree in the direction of the given waypoint until it is reached
	 * or the tree is trapped. Repeatedly calls extendRRT and returns the status
	 * according to the last result of the extension
	 * 
	 * @param waypoint the waypoint set as the goal for extension
	 * @return status the status resulting from the last extend
	 */
	protected Status connectRRT(RRTreeWaypoint waypoint) {
		Status status = Status.ADVANCED;

		while (status == Status.ADVANCED && !this.checkGoal(this.getWaypointNew())) {
			status = this.extendRRT(waypoint);
		}

		return status;
	}

	/**
	 * Extends the tree in the direction of the given waypoint and returns the
	 * status according to the result of the extension
	 * 
	 * @param waypoint the waypoint set as the goal for extension
	 * @return status the status resulting from the extend
	 */
	protected Status extendRRT(RRTreeWaypoint waypoint) {
		Status status;
		boolean success;
		RRTreeWaypoint waypointNear, waypointNew;

		// Finds the node in the tree closest to the sampled position
		waypointNear = (RRTreeWaypoint) this.findNearest(waypoint, 1).get(0);
		// Create a new node by extension from near to sampled
		success = this.newWaypoint(waypoint, waypointNear);

		// Set status variable by checking if extension was possible
		if (success) {
			waypointNew = this.getWaypointNew();
			this.getWaypointList().add(waypointNew);
			if (waypointNew.getPrecisionPosition().equals(waypoint.getPrecisionPosition())) {
				status = Status.REACHED;
			} else {
				status = Status.ADVANCED;
			}
		} else {
			status = Status.TRAPPED;
		}

		return status;
	}

	/**
	 * Creates a new waypoint by extending a sampled waypoint in the direction of
	 * the nearest waypoint following certain restrictions
	 * 
	 * @param waypoint the waypoint to be reached
	 * @param waypointNear the waypoint to be extended
	 * 
	 * @return true if it was possible to extend in the desired direction (false
	 *         otherwise)
	 */
	protected boolean newWaypoint(RRTreeWaypoint waypoint, RRTreeWaypoint waypointNear) {
		boolean success = true;

		// Extend nearest waypoint in the direction of waypoint
		Position positionNew = this.growPosition(waypoint, waypointNear);
		RRTreeWaypoint waypointNew = new RRTreeWaypoint(positionNew, waypointNear);
		
		waypointNew.setEto(this.computeTime(waypointNear, waypointNew)); // TODO: Review
		waypointNew.setAto(waypointNew.getEto());
		this.setWaypointNew(waypointNew);

		// Check if the new waypoint is in conflict with the environment
		if (super.checkConflict(waypointNew)) {
			success = false;
		}
		// Check if the edge between waypoints is in conflict
		else if (super.checkConflict(waypointNear, waypointNew)) {
			success = false;
		}

		return success;
	}

	/**
	 * Grows the tree in the direction of the waypoint from the nearest waypoint
	 * 
	 * @param position the goal position to which the tree is grown
	 * @param positionNear the source position from which the tree is grown
	 * 
	 * @return the new position resulting from the controlled growth of the tree
	 */
	protected Position growPosition(Position position, Position positionNear) {
		// TODO: Incorporate aircraft capabilities
		Position positionNew;

		Vec4 point1 = super.getEnvironment().getGlobe().computePointFromPosition(positionNear);
		Vec4 point2 = super.getEnvironment().getGlobe().computePointFromPosition(position);

		double dist = point1.distanceTo3(point2);

		if (dist < EPSILON) {
			positionNew = position;
		} else {
			double x, y, z, dx, dy, dz, dist2, epsilon2, ddz;
			dx = point2.x - point1.x;
			dy = point2.y - point1.y;
			dz = point2.z - point1.z;

			ddz = dz / dist * EPSILON;
			epsilon2 = Math.sqrt(EPSILON * EPSILON - ddz * ddz);
			dist2 = point1.distanceTo2(point2);

			x = point1.x + dx / dist2 * epsilon2;
			y = point1.y + dy / dist2 * epsilon2;
			z = point1.z + ddz;

			Vec4 pointNew = new Vec4(x, y, z);
			positionNew = super.getEnvironment().getGlobe().computePositionFromPoint(pointNew);
		}

		return positionNew;
	}

	/**
	 * Computes the estimated time over(eto) an waypoint using the eto of the
	 * previous waypoint and the aircraft capabilities
	 * 
	 * @param source the previous waypoint with known eto
	 * @param target the following waypoint with eto to be calculated
	 * 
	 * @return the eto for the following waypoint
	 */
	protected ZonedDateTime computeTime(RRTreeWaypoint source, RRTreeWaypoint target) {
		Path leg = new Path(source, target);
		Capabilities capabilities = this.getAircraft().getCapabilities();
		Globe globe = this.getEnvironment().getGlobe();

		ZonedDateTime end = capabilities.getEstimatedTime(leg, globe, source.getEto());
		return end;
	}

	/**
	 * Check whether the waypoint is the goal or within the goal region
	 * 
	 * @param waypoint the RRTree Waypoint to be compared (default=this.waypointNew)
	 * 
	 * @return true if the new waypoint is within the goal region
	 */
	protected boolean checkGoal() {
		return this.checkGoal(waypointNew);
	}

	protected boolean checkGoal(RRTreeWaypoint waypoint) {
		return super.getContinuumEnvironment().getDistance(waypoint, this.getGoal()) < GOAL_THRESHOLD;
	}

	/**
	 * Computes the path from the current waypoint to the starting waypoint
	 * 
	 * @param waypoint the last RRTree Waypoint in the path
	 *            (default=this.waypointNew)
	 */
	protected void computePath() {
		this.computePath(waypointNew);
	}

	protected void computePath(RRTreeWaypoint waypoint) {
		plan.addFirst(waypoint);
		while (waypoint.getParent() != null) {
			waypoint = waypoint.getParent();
			plan.addFirst(waypoint);
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
		this.clearWaypoints();
		
		this.setStart(new RRTreeWaypoint(origin));
		this.getStart().setEto(etd);
		this.getWaypointList().add(start);
		this.setWaypointNew(start);

		this.setGoal(new RRTreeWaypoint(destination));

	}

	/**
	 * Computes a plan by growing a tree until the goal is reached
	 */
	protected void compute() {
		boolean status = false;
		for (int i = 0; i < MAX_ITER; i++) {
			RRTreeWaypoint waypointRand = this.sampleBiased(BIAS);
			switch (this.getStrategy()) {
			case CONNECT:
				status = this.connectRRT(waypointRand) != Status.TRAPPED;
				break;
			case EXTEND:
			default:
				status = this.extendRRT(waypointRand) != Status.TRAPPED;
				break;
			}
			if (status) {
				if (this.checkGoal()) {
					this.computePath();
					return;
				}
			}
		}
		// TODO: Review what happens when no path is found
		System.out.println("No path found after "+MAX_ITER+" iterations.");
		return;
	}

	// ---------- PUBLIC METHODS ----------

	/**
	 * Plans a trajectory from an origin to a destination at a specified estimated
	 * time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @return the planned trajectory from the origin to the destination with the
	 *         estimated time of departure
	 * 
	 * @see Planner#plan(Position, Position, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		this.initialize(origin, destination, etd);
		this.compute();
		Trajectory trajectory = this.createTrajectory();
		this.revisePlan(trajectory); // Not sure what this does
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
	 * @return the planned trajectory from the origin to the destination along the
	 *         waypoints with the estimated time of departure
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
	 * Indicates whether or not this RRT planner supports a specified environment.
	 * 
	 * @param environment the environment
	 * 
	 * @return true if the environment is a planning continuum, false otherwise
	 * 
	 * @see PlanningContinuum
	 */
	@Override
	public boolean supports(Environment environment) {
		boolean supports = super.supports(environment);

		if (supports) {
			supports = (environment instanceof PlanningContinuum);
		}

		return supports;
	}

}
