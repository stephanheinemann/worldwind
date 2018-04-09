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
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import java.util.Set;

import com.cfar.swim.worldwind.ai.continuum.AbstractSampler;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;

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
    static final int MAX_ITER = 1000_000;

    /** the maximum distance to extend a waypoint in the tree */
    static final int EPSILON = 10;

    // ---------- VARIABLES ----------

    /** the start RRT waypoint */
    private RRTreeWaypoint start = null;

    /** the goal RRT waypoint */
    private RRTreeWaypoint goal = null;

    /** the newest waypoint in the tree */
    private RRTreeWaypoint waypointNew = null;

    /** the set of sampled RRT waypoints */
    protected Set<RRTreeWaypoint> tree = new HashSet<>();

    /** the last computed plan */
    private final LinkedList<RRTreeWaypoint> plan = new LinkedList<>();

    // ---------- CONSTRUCTORS ----------

    public RRTreePlanner(Aircraft aircraft, Environment environment) {
	super(aircraft, environment);
	// TODO Define how the constructor for the planner should be implemented
    }

    /**
     * Creates an RRT waypoint at a specified position.
     * 
     * @param position the position
     * 
     * @return the RRT waypoint at the specified position
     */
    protected RRTreeWaypoint createWaypoint(Position position) {
	return new RRTreeWaypoint(position);
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
	// TODO: How does this function work?
	return new Trajectory((List<Waypoint>) this.plan.clone());
    }

    /**
     * Samples a new waypoint sampled from a uniformed distribution over the
     * environment space
     * 
     * @return waypoint the RRTreeWaypoint sampled
     */
    protected RRTreeWaypoint sampleRandom() {
	// TODO: How to implement a random sampling of the environment

	return null;
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
	// TODO: Is there any problem in using the random package?
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
	// TODO: Find nearest must return a node and not a position
	waypointNear = new RRTreeWaypoint(super.findNearest(waypoint,1).get(0));

	// Create a new node by extension from near to sampled
	success = this.newWaypoint(waypoint, waypointNear);

	// Set status variable by checking if extension was possible
	if (success) {
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
     * Creates a new waypoint by extending a sampled waypoint in the direction
     * of the nearest waypoint following certain restrictions
     * 
     * @param waypoint the waypoint to be reached
     * @param waypointNear the waypoint to be extended
     * 
     * @return true if it was possible to extend in the desired direction (false
     *         otherwise)
     */
    protected boolean newWaypoint(RRTreeWaypoint waypoint,
	    RRTreeWaypoint waypointNear) {
	boolean success;
	RRTreeWaypoint waypointNew;

	// Extend nearest waypoint in the direction of waypoint
	waypointNew = this.growWaypoint(waypoint, waypointNear);
	this.setWaypointNew(waypointNew);

	// Check if the new waypoint is in conflict with the environment
	// TODO: [ENVIRONMENT] How to check if the new waypoint is in conflict
	success = true;

	return success;
    }
    
    
    /**
     * Grows the tree in the direction of the waypoint from the branch ending at  
     * 
     * @param waypoint
     * @param waypointNear
     * @return
     */
    protected RRTreeWaypoint growWaypoint(RRTreeWaypoint waypoint,
	    RRTreeWaypoint waypointNear) {
	RRTreeWaypoint waypointNew;
	
	// TODO: How to extend a waypoint in a certain direction
	waypointNew = new RRTreeWaypoint(waypoint);
	
	return waypointNew;
    }

    /**
     * Check whether the new waypoint is the goal or within the goal region
     * 
     * @return true if the new waypoint is within the goal region
     */
    protected boolean checkGoal() {
	// TODO: Define how to compare if the new waypoint is the goal/in the
	// goal region
	return false;
    }

    /**
     * Computes the path from the current waypoint to the starting waypoint
     */
    protected void computePath() {
	// TODO: compute the path from the new waypoint to the start by
	// repeatedly calling the parent of each node
    }

    /**
     * Initializes the planner to plan from an origin to a destination at a
     * specified estimated time of departure.
     * 
     * @param origin the origin in globe coordinates
     * @param destination the destination in globe coordinates
     * @param etd the estimated time of departure
     */
    protected void initialize(Position origin, Position destination,
	    ZonedDateTime etd) {

	this.setStart(this.createWaypoint(origin));
	this.setGoal(this.createWaypoint(destination));

    }

    /**
     * Computes a plan by growing a tree until the goal is reached
     */
    protected void compute() {
	for (int i = 0; i < MAX_ITER; i++) {
	    RRTreeWaypoint waypointRand = this.sampleBiased(5);
	    if (this.extendRRT(waypointRand) != Status.TRAPPED) {
		if (this.checkGoal()) {
		    this.computePath();
		    return;
		}
	    }
	}
    }

    // ---------- PUBLIC METHODS ----------

    @Override
    public Trajectory plan(Position origin, Position destination,
	    ZonedDateTime etd) {
	// TODO Implement the planning strategy according to this algorithm

	// this.initialize(origin, destination, etd);
	// this.compute();
	// Trajectory trajectory = this.createTrajectory();
	// this.revisePlan(trajectory);
	// return trajectory;

	this.initialize(origin, destination, etd);
	this.compute();
	Trajectory trajectory = this.createTrajectory();
	// this.revisePlan(trajectory);
	// return trajectory;

	return trajectory;
    }

    @Override
    public Trajectory plan(Position origin, Position destination,
	    List<Position> waypoints, ZonedDateTime etd) {
	// TODO Implement a planning strategy to allow for intermidiate goals

	// Multiple calls to plan (origin, destination, etd) creating an entire
	// tree between multiple goals
	return null;
    }

}
