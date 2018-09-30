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
package com.cfar.swim.worldwind.ai.rrt.oarrt;

import java.time.ZonedDateTime;
import java.util.List;

import com.cfar.swim.worldwind.ai.AnytimePlanner;
import com.cfar.swim.worldwind.ai.OnlinePlanner;
import com.cfar.swim.worldwind.ai.rrt.arrt.ARRTree;
import com.cfar.swim.worldwind.ai.rrt.arrt.ARRTreePlanner;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Extension;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreeWaypoint;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Sampling;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Strategy;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.connections.Datalink;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes an online anytime RRT planner that plans a trajectory of an aircraft
 * in an environment considering a local cost and risk policy. The planner
 * provides various solutions depending on the deliberating time it is given and
 * it continuously improves the cost of each solution by a given factor.
 * Planning and executing of the plan are interleaved and for every iterartion
 * the start position is updated to reflect the new position of the agent.
 * 
 * @author Manuel
 *
 */
public class OARRTreePlanner extends ARRTreePlanner implements OnlinePlanner, AnytimePlanner {

	private static final int DELAY_FACTOR = 3;

	/** the state of the online capabilities of the planner mode (active or not) */
	private final boolean online = true;

	/** the dist threshold to consider position displacement worthy of new plan */
	private final double positionThreshold;

	/** the datalink connection of this planner */
	private Datalink datalink;

	/** the current position of the aircraft */
	private Waypoint aircraftTimedPosition;

	/**
	 * Constructs an anytime RRT planner for a specified aircraft and environment
	 * using default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see ARRTreePlanner#ARRTreePlanner(Aircraft, Environment)
	 */
	public OARRTreePlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		this.positionThreshold = 2d;
	}

	/**
	 * Constructs an anytime RRT planner for a specified aircraft and environment
	 * using default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * @param epsilon the maximum distance to extend a waypoint in the tree
	 * @param bias the bias of the sampling algorithm towards goal
	 * @param maxIter the maximum number of sampling iterations
	 * @param strategy the expanding strategy for the planner
	 * @param extension the extension technique for the planner
	 * @param sampling the sampling technique for the planner
	 * @param positionThreshold the treshold to consider position displacements
	 * 
	 * @see ARRTreePlanner#ARRTreePlanner(Aircraft, Environment, double, int, int,
	 *      Strategy, Extension, Sampling)
	 */
	public OARRTreePlanner(Aircraft aircraft, Environment environment, double epsilon, int bias,
			int maxIter,
			Strategy strategy, Extension extension, Sampling sampling, double positionThreshold) {
		super(aircraft, environment, epsilon, bias, maxIter, strategy, extension, sampling);
		this.positionThreshold = positionThreshold;
	}

	/**
	 * Checks if the online capabilities of the planner mode are active or not.
	 * 
	 * @return true if the planner mode is set to online, false otherwise
	 */
	public boolean isOnline() {
		return online;
	}

	/**
	 * Gets the distance threshold to consider a position displacement as worthy of
	 * a new plan.
	 * 
	 * @return the distance threshold for each position
	 */
	public double getPositionThreshold() {
		return positionThreshold;
	}

	/**
	 * Gets the datalink connection of this planner.
	 * 
	 * @return the datalink
	 */
	public Datalink getDatalink() {
		return datalink;
	}

	/**
	 * Sets the datalink connection of this planner.
	 * 
	 * @param datalink the detailing to set
	 */
	public void setDatalink(Datalink datalink) {
		this.datalink = datalink;
	}

	/**
	 * Gets the current position of the aircraft.
	 * 
	 * @return the current position of the aircraft
	 */
	public Waypoint getAircraftTimedPosition() {
		return aircraftTimedPosition;
	}

	/**
	 * Sets the current position of the aircraft.
	 * 
	 * @param aircraftPosition the current position of the aircraft
	 */
	public void setAircraftTimedPosition(Waypoint aircraftTimedPosition) {
		this.aircraftTimedPosition = aircraftTimedPosition;
	}

	/**
	 * Check if the distance between the two positions is significant to consider
	 * the current position as a different one.
	 * 
	 * @param previous the previous position to consider
	 * @param current the current position to consider
	 * 
	 * @return true if the movement between the two positions is significant, false
	 *         otherwise
	 */
	public boolean isSignificantMovement(Position previous, Position current) {
		return this.getEnvironment().getDistance(previous, current) > this.getPositionThreshold();
	}

	/**
	 * Checks if the current position of the aircraft is inside the goal region.
	 * 
	 * @return true if aircraft is inside the goal region, false otherwise
	 */
	public boolean isInsideGoalRegion() {
		return this.checkGoal(getAircraftTimedPosition());
	}

	/**
	 * Updates the current position of the aircraft in the planner by reading its
	 * actual position from an external source.
	 */
	public void updateAircraftTimedPosition() {
		this.setAircraftTimedPosition(getDatalink().getAircraftTimedPosition());
	}

	/**
	 * Finds the closest waypoint to a reference position from a list of waypoints.
	 * Waypoints are compared based on their Eto and the Ato of the position, the
	 * closest is the last one which should already have been passed.
	 * 
	 * @param position the reference position
	 * @param waypointList the waypoint list to be checked
	 * 
	 * @return the waypoint from the list closest to the position
	 */
	public RRTreeWaypoint findClosestWaypointTime(Waypoint waypoint, List<Waypoint> waypointList) {
		RRTreeWaypoint closest = null;
		for (Waypoint wpt : waypointList) {
			closest = (RRTreeWaypoint) wpt;
			if (wpt.getEto().compareTo(waypoint.getAto()) == 1)
				break;
		}
		return closest;
	}

	/**
	 * Finds the closest waypoint to a reference position from a list of waypoints.
	 * Waypoints are compared based on their distance to the reference position.
	 * 
	 * @param position the reference position
	 * @param waypointList the waypoint list to be checked
	 * 
	 * @return the waypoint from the list closest to the position
	 */
	public RRTreeWaypoint findClosestWaypointDist(Waypoint waypoint, List<Waypoint> waypointList) {
		RRTreeWaypoint closest = (RRTreeWaypoint) waypointList.stream().sorted((p1, p2) -> Double
				.compare(this.getEnvironment().getNormalizedDistance(p1, waypoint),
						this.getEnvironment().getNormalizedDistance(p2, waypoint)))
				.findFirst().get();
		return closest;
	}

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
	 * @see ARRTreePlanner#plan(Position, Position, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		this.initialize(origin, destination, etd);

		this.setCostBias(this.getMinimumQuality());
		this.setDistBias(1 - this.getCostBias());
		this.setCostBound(Double.POSITIVE_INFINITY);

		Trajectory trajectory = new Trajectory();

		// Anytime variables
		boolean newPlan;
		double costOld = Double.POSITIVE_INFINITY;

		// Auxiliary Trees
		ARRTree treeR = new ARRTree(), treeT = new ARRTree();
		treeT.setCostBias(this.getMinimumQuality());
		treeT.setDistBias(1 - this.getCostBias());
		treeT.setCostBound(Double.POSITIVE_INFINITY);

		do {
			// Anytime
			if (!this.isImproved(costOld)) {
				this.clearExpendables();
				this.addVertex(getStart());
				this.setWaypointNew(getStart());

				costOld = this.getGoal().getCost();
				newPlan = this.compute();

				this.saveToTree(treeT);

				if (newPlan) {
					trajectory = this.createTrajectory();
					this.revisePlan(trajectory);

					treeR.clone(treeT);

					this.updateWeights();
					this.updateCostBound();
					treeT.setBiases(this.getDistBias(), this.getCostBias());
					treeT.setCostBound(this.getCostBound());

				} else {
					this.updateWeights();
					treeT.setBiases(this.getDistBias(), this.getCostBias());
				}
			}
			// Online
			if (isOnline()) {
				this.updateAircraftTimedPosition();
				int index = getDatalink().getNextWaypointIndex();
				int next = index + DELAY_FACTOR;

				if (index >= 0 && next < this.getPlan().size()) {
					// Check if the aircraft moved significantly
					if (isSignificantMovement(getStart(), getAircraftTimedPosition())) {
						// Update start waypoint
						this.setStart((RRTreeWaypoint) getPlan().get(next));
						this.getStart().setH(computeHeuristic(getStart(), getGoal()));
						RRTreeWaypoint waypoint = findClosestWaypointDist(getAircraftTimedPosition(), getPlan());
						// Check if the displacement is significant for a new plan
						if (isSignificantMovement(waypoint, getAircraftTimedPosition())) {
							this.initialize(getPlan().get(next), destination, getAircraftTimedPosition().getAto());
							// Reset anytime parameters
							this.setCostBias(this.getMinimumQuality());
							this.setDistBias(1 - this.getCostBias());
							this.setCostBound(Double.POSITIVE_INFINITY);
							costOld = Double.POSITIVE_INFINITY;
						}
					}
				}
			}
		} while ((!isOnline() && !isImproved(costOld)) || (isOnline() && !isInsideGoalRegion()));

		return trajectory;
	}

}
