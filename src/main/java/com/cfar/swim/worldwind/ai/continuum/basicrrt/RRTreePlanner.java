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

import com.cfar.swim.worldwind.ai.continuum.AbstractSampler;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.PlanningContinuum;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;

/**
 * Realizes a basic RRT planner that plans a trajectory of an aircraft in an
 * environment considering a local cost and risk policy. The origin of the tree
 * is the departure position and it is grown until the goal is reached.
 * 
 * @author Manuel Rosa
 *
 */
public class RRTreePlanner extends AbstractSampler {

	/** the maximum number of sampling iterations */
	static final int MAX_ITER = 3_000;

	/** the maximum distance to extend a waypoint in the tree */
	static final double EPSILON = 50d; //TODO: Units?

	static final double GOAL_THRESHOLD = 5d;

	// ---------- VARIABLES ----------

	/** the start RRT waypoint */
	private RRTreeWaypoint start = null;

	/** the goal RRT waypoint */
	private RRTreeWaypoint goal = null;

	/** the newest waypoint in the tree */
	private RRTreeWaypoint waypointNew = null;

	/** the set of sampled RRT waypoints */
	protected List<RRTreeWaypoint> tree = new ArrayList<RRTreeWaypoint>();

	/** the last computed plan */
	private final LinkedList<RRTreeWaypoint> plan = new LinkedList<>();

	// ---------- CONSTRUCTORS ----------

	public RRTreePlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		System.out.println("I am heree!!!!");
	}

	// ---------- Setters and Getters ----------

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

	// ---------- PROTECTED METHODS ----------

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
				System.out.println("Finding nearest waypoint...");
		waypointNear = (RRTreeWaypoint) this.findNearest(waypoint,this.tree, 1).get(0);
		// Create a new node by extension from near to sampled
				System.out.println("Creating new waypoint...");
		success = this.newWaypoint(waypoint, waypointNear);

		// Set status variable by checking if extension was possible
		if (success) {
					System.out.println("No conflict with terrain, adding waypoint to tree and setting status...");
			waypointNew = this.getWaypointNew();
			tree.add(waypointNew);
			if (waypointNew == waypointNear) {
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
				System.out.println("Growing new position...");
		Position positionNew = this.growWaypoint(waypoint, waypointNear);
				System.out.println("Creating new waypoint from grown position..."+positionNew);
		RRTreeWaypoint waypointNew = new RRTreeWaypoint(positionNew, waypointNear);
				System.out.println("Setting new waypoint time..."); //TODO: Review
//		double dtime = this.getEnvironment().getDistance(waypointNear, waypointNew)*this.getAircraft().getCapabilities().getCruiseSpeed()
		waypointNew.setEto(waypointNear.getEto());
				System.out.println("Setting new waypoint...");
		this.setWaypointNew(waypointNew);

				System.out.println("Checking conflict...");
		// Check if the new waypoint is in conflict with the environment
		if (super.checkConflict(waypointNew)) {
					System.out.println("The new waypoint is in conflict with the terrain...");
			success = false;
		}
		// Check if the edge between waypoints is in conflict
		else if (super.checkConflict(waypointNear, waypointNew)) {
					System.out.println("The edge between waypoints is in conflict with the terrain...");
			success = false;
		}

				System.out.println("Returning success value...");
		return success;
	}

	/**
	 * Grows the tree in the direction of the waypoint from the branch ending at
	 * 
	 * @param waypoint
	 * @param waypointNear
	 * @return
	 */
	protected Position growWaypoint(Position position, Position positionNear) {
		// TODO: Incorporate aircraft capabilities
		Position positionNew;

				System.out.println("Computing points from position...");
		Vec4 point1 = super.getEnvironment().getGlobe().computePointFromPosition(positionNear);
		Vec4 point2 = super.getEnvironment().getGlobe().computePointFromPosition(position);
		
		double dist  = point1.distanceTo3(point2);
		
		if (dist < EPSILON) {
					System.out.println("Distance smaller than epsilon");
			positionNew = position;
		}
		else {
					System.out.println("Distance bigger than epsilon");
			double x, y, z, dx, dy, dz, dist2, epsilon2, ddz;
			dx = point2.x - point1.x;
			dy = point2.y - point1.y;
			dz = point2.z - point1.z;
			System.out.println("dx = "+dx+" dy = "+dy+" dz = "+dz);
			
			ddz = dz/dist * EPSILON;
			epsilon2 = Math.sqrt(EPSILON*EPSILON - ddz*ddz);
			dist2 = point1.distanceTo2(point2);
			
			x = point1.x + dx/dist2 * epsilon2;
			y = point1.y + dy/dist2 * epsilon2;
			z = point1.z + ddz;
			
			Vec4 pointNew = new Vec4(x,y,z); 
					System.out.println("New point calculated");
			positionNew = super.getEnvironment().getGlobe().computePositionFromPoint(pointNew);
					System.out.println("New position calculated... "+positionNew);
		}

		return positionNew;
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
	 * @param waypoint the last RRTree Waypoint in the path (default=this.waypointNew)
	 */
	protected void computePath() {
		this.computePath(waypointNew);
	}
	protected void computePath(RRTreeWaypoint waypoint) {
		plan.addFirst(waypoint);
		System.out.println(waypoint);
		while (waypoint.getParent() != null) {
			waypoint = waypoint.getParent();
			System.out.println(waypoint);
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

		this.setStart(new RRTreeWaypoint(origin));
		this.getStart().setEto(etd);
		this.tree.add(start);
		
		this.setGoal(new RRTreeWaypoint(destination));

	}

	/**
	 * Computes a plan by growing a tree until the goal is reached
	 */
	protected void compute() {
		for (int i = 0; i < MAX_ITER; i++) {
					System.out.println("Iteration #"+i);
			RRTreeWaypoint waypointRand = this.sampleBiased(5);
					System.out.println("Iteration #"+i+" Waypoint sampled");
			if (this.extendRRT(waypointRand) != Status.TRAPPED) {
						System.out.println("Iteration #"+i+" Not trapped");
				if (this.checkGoal()) {
							System.out.println("Iteration #"+i+" Inside goal region");
					this.computePath();
							System.out.println("Iteration #"+i+" Path computed");
					return;
				}
			}
		}
	}

	// ---------- PUBLIC METHODS ----------

	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
					System.out.println("Initializing...");
		this.initialize(origin, destination, etd);
					System.out.println("Computing...");
		this.compute();
					System.out.println("Creating trajectory...");
		Trajectory trajectory = this.createTrajectory();
					System.out.println("Trajectory null? " + trajectory.getLength() + "revising...");
		this.revisePlan(trajectory); //Not sure what this does
					System.out.println("Retunring...");
		return trajectory;
	}

	@Override
	public Trajectory plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
		// TODO Implement a planning strategy to allow for intermidiate goals

		// Multiple calls to plan (origin, destination, etd) creating an entire
		// tree between multiple goals
		return this.plan(origin, destination, etd);
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
