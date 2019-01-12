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
package com.cfar.swim.worldwind.ai.prm.ofadprm;

import java.time.ZonedDateTime;

import com.cfar.swim.worldwind.ai.OnlinePlanner;
import com.cfar.swim.worldwind.ai.prm.fadprm.FADPRMPlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.connections.Datalink;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.google.common.collect.Iterables;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes an online flexible anytime dynamic PRM planner that plans a
 * trajectory of an aircraft in an environment considering a local cost and risk
 * policy. The planner provides various solutions depending on the deliberating
 * time it is given and it continuously improves the cost of each solution by a
 * given factor. OFADPRM deals with dynamic insertion of SWIM obstacles during
 * planning and execution. Planning and executing of the plan are interleaved
 * and for every iteration the start position is updated to reflect the new
 * position of the agent.
 * 
 * @author Henrique Ferreira
 *
 */
public class OFADPRMPlanner extends FADPRMPlanner implements OnlinePlanner {

	private static final int DELAY_FACTOR = 3;

	/** the state of the online capabilities of the planner mode (active or not) */
	private boolean onlineStatus;

	/** the radius of the sphere defining the goal region */
	private double goalThreshold;
	
	/**
	 * the distance threshold to consider a position displacement as worthy of a new
	 * plan
	 */
	private double positionThreshold;

	/** the datalink connection of this planner */
	private Datalink datalink;

	/** the current position of the aircraft */
	private Waypoint aircraftTimedPosition;

	/**
	 * Constructs an online flexible anytime dynamic PRM planner for a specified
	 * aircraft and environment using default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see FADPRMPlanner#FADPRMPlanner(Aircraft, Environment)
	 */
	public OFADPRMPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		this.positionThreshold = 2d;
		goalThreshold = 5d;
	}

	/**
	 * Checks if the online capabilities of the planner mode are active or not.
	 * 
	 * @return true if the planner mode is set to online, false otherwise
	 */
	public boolean isOnline() {
		return onlineStatus;
	}

	/**
	 * Sets the online status of the planner.
	 * 
	 * @param onlineStatus the online status of this planner
	 */
	public void setOnlineStatus(boolean onlineStatus) {
		this.onlineStatus = onlineStatus;
	}
	
	/**
	 * Gets the distance defining the goal region.
	 * 
	 * @return the distance defining the goal region
	 */
	public double getGoalThreshold() {
		return goalThreshold;
	}

	/**
	 * Sets the distance defining the goal region.
	 * 
	 * @param goalThreshold the distance defining the goal region
	 */
	public void setGoalThreshold(double goalThreshold) {
		this.goalThreshold = goalThreshold;
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
	 * @param datalink the datalink to set
	 */
	public void setDatalink(Datalink datalink) {
		this.datalink = datalink;
	}

	/**
	 * Sets the distance threshold to consider a position displacement as worthy of
	 * a new plan.
	 * 
	 * @param positionThreshold the distance threshold for each position
	 */
	public void setPositionThreshold(double positionThreshold) {
		this.positionThreshold = positionThreshold;
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
		return this.getEnvironment().getDistance(this.getAircraftTimedPosition(), this.getGoal()) < this
				.getGoalThreshold();
	}

	/**
	 * Updates the current position of the aircraft in the planner by reading its
	 * actual position from an external source.
	 */
	public void updateAircraftTimedPosition() {
		this.setAircraftTimedPosition(getDatalink().getAircraftTimedPosition());
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
	 * @see FADPRMPlanner#plan(Position, Position, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		// if planner is invoked again with the same environment, lists are cleared
		this.getWaypointList().clear();
		this.getEdgeList().clear();

		this.initialize(origin, destination, etd);
		Trajectory trajectory = this.findFeasiblePath();
		this.revisePlan(trajectory);
		do {
			// // Online
			// if (isOnline()) {
			// System.out.println("Online planning!!!");
			// this.updateAircraftTimedPosition();
			// System.out.println("Position updated!!");
			//
			// int index = getDatalink().getNextWaypointIndex();
			// System.out.println("Next Waypoint index = " + index);
			// if (index >= 0 && index + 3 < this.plan.size()) {
			// System.out.println("Current plan -----------------------");
			// for (Waypoint waypoint : this.plan)
			// System.out.println(waypoint);
			// System.out.println("------------------------------------");
			// System.out.println("In plan this is ... " + this.plan.get(index));
			// this.updateStart(plan.get(index+3));
			// } else
			// System.out.println("Negative indexxxx (or maximum) " + index);
			// }
			// Anytime
			if (!this.isInflated()) {
				// System.out.println("Anytime");
				// System.out.println("Improving");
				trajectory = this.improve();
				double size = 0, waypoints = 0, cost = 0d;
				double sizeT = 0, waypointsT = 0, costT = 0d;
				if (trajectory.isEmpty()) {
					System.out.println("No feasible solution was found");
					size = 0;
					waypoints = 0;
					cost = 0;
				} else {
					size = Iterables.size(trajectory.getPositions());
					waypoints = this.getWaypointList().size();
					cost = this.getEnvironment().getDistance(trajectory.getCost());
				}
				System.out.println(String.format("%.1f, %.1f, %.4f, %.1f", waypoints, size, cost, this.getInflation()));
				this.revisePlan(trajectory);
				// if (getDatalink().isConnected()) {
				// this.getDatalink().uploadFlightPath(trajectory);
				// System.out.println("UPLOADDDDDDDDDDDDDDDDDDDDDDDDDDDDD");
				// }
			}

		} while ((!isOnline() && !isInflated()) || (isOnline() && !isInsideGoalRegion()));
		System.out.println("Improved");

		return trajectory;
	}
}
