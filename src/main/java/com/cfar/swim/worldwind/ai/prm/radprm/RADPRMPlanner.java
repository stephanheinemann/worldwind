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
package com.cfar.swim.worldwind.ai.prm.radprm;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.ai.AbstractPlanner;
import com.cfar.swim.worldwind.ai.AnytimePlanner;
import com.cfar.swim.worldwind.ai.OnlinePlanner;
import com.cfar.swim.worldwind.ai.Planner;
import com.cfar.swim.worldwind.ai.prm.fadprm.FADPRMPlanner;
import com.cfar.swim.worldwind.ai.prm.fadprm.FADPRMWaypoint;
import com.cfar.swim.worldwind.ai.prm.faprm.FAPRMPlanner;
import com.cfar.swim.worldwind.ai.prm.faprm.FAPRMWaypoint;
import com.cfar.swim.worldwind.ai.prm.rigidprm.CollisionDelay;
import com.cfar.swim.worldwind.ai.prm.rigidprm.QueryMode;
import com.cfar.swim.worldwind.ai.prm.rigidprm.RigidPRMWaypoint;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.planning.Edge;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.render.Obstacle;
import com.google.common.collect.Iterables;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;

/**
 * @author Henrique Ferreira
 *
 */
public class RADPRMPlanner extends FADPRMPlanner {

	/** the fraction of waypoints that should be removed */
	private double repairFraction;

	/**
	 * Constructs a FADPRM planner for a specified aircraft and environment using
	 * default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see AbstractPlanner#AbstractPlanner(Aircraft, Environment)
	 */
	public RADPRMPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}

	/**
	 * Gets the list of already sampled FADPRM waypoints
	 * 
	 * @return the list of waypoints
	 */
	@SuppressWarnings("unchecked")
	public List<? extends RADPRMWaypoint> getWaypointList() {
		return (List<RADPRMWaypoint>) this.getEnvironment().getWaypointList();
	}

	/**
	 * Polls an FADPRM waypoint from the expandable FADPRM waypoints.
	 * 
	 * @return the polled FADPRM waypoint from the expandable FADPRM waypoints if
	 *         any, null otherwise
	 */
	protected RADPRMWaypoint pollExpandable() {
		return (RADPRMWaypoint) super.pollExpandable();
	}

	/**
	 * Gets the start FAPRM waypoint of this FADPRM planner.
	 * 
	 * @return the start FAPRM waypoint of this FADPRM planner
	 */
	protected RADPRMWaypoint getStart() {
		return (RADPRMWaypoint) super.getStart();
	}

	/**
	 * Gets the goal FAPRM waypoint of this FADPRM planner.
	 * 
	 * @return the goal FAPRM waypoint of this FADPRM planner
	 */
	protected RADPRMWaypoint getGoal() {
		return (RADPRMWaypoint) super.getGoal();
	}

	/**
	 * Gets the repair fraction of this RADPRM planner.
	 * 
	 * @return the repairFraction of this RADPRM planner
	 */
	public double getRepairFraction() {
		return repairFraction;
	}

	/**
	 * Sets the repair fraction of this RADPRM planner.
	 * 
	 * @param repairFraction the repairFraction to set
	 */
	public void setRepairFraction(double repairFraction) {
		this.repairFraction = repairFraction;
	}

	/**
	 * Creates a waypoint at a specified position, initializes its search value to
	 * 0, its beta to the current inflation, sets the waypoint desirability and
	 * lambda. Finally, adds it to the waypoint list.
	 * 
	 * @param position the position in global coordinates
	 * @return the new created waypoint
	 */
	@SuppressWarnings("unchecked")
	protected RADPRMWaypoint createWaypoint(Position position) {
		RADPRMWaypoint newWaypoint = new RADPRMWaypoint(position);
		newWaypoint.setEto(this.getEnvironment().getTime());
		newWaypoint.setSearch(0);
		newWaypoint.setBeta(this.getInflation());
		newWaypoint.setDensity(0);
		((List<RADPRMWaypoint>) this.getWaypointList()).add(newWaypoint);
		return newWaypoint;
	}

	/**
	 * Creates a trajectory of the computed plan.
	 * 
	 * @return the trajectory of the computed plan
	 */
	@SuppressWarnings("unchecked")
	protected Trajectory createTrajectory() {
		return new Trajectory((List<RADPRMWaypoint>) this.plan.clone());
	}

	/**
	 * Initializes the planner to plan from an origin to a destination at a
	 * specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 */
	@Override
	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {
		this.backupIndex++;
		if (this.improving) {
			this.restore();
		} else {
			this.setInflation(this.getMinimumQuality());
			this.setCounter(1);
			this.pathCost.add(0d);
		}
		this.clearExpandables();
		this.clearExpanded();
		this.clearWaypoints();
		this.setStart(this.createWaypoint(origin));

		this.setGoal(this.createWaypoint(destination));

		// if MULTIPLE query mode is active, then reinitialize cost for all waypoints
		for (FADPRMWaypoint waypoint : this.getWaypointList()) {
			waypoint.setCost(Double.POSITIVE_INFINITY);
			waypoint.setH(this.getEnvironment().getNormalizedDistance(waypoint, this.getGoal()));
			waypoint.setEto(this.getEnvironment().getTime());
			waypoint.setBeta(this.getInflation());
			waypoint.setSearch(0);
		}
		
		this.updateWaypoint(this.getGoal());
		this.updateWaypoint(this.getStart());

		this.getStart().setCost(0d);
		this.getStart().setH(this.getEnvironment().getNormalizedDistance(this.getStart(), this.getGoal()));
		this.getStart().setEto(etd);

		this.getGoal().setH(0d);

		this.addExpandable(this.getStart());

	}

	/**
	 * Repairs the roadmap. First, computes the utility for every node and then
	 * removes the ones with the lowest value. The fraction from the whole roadmap
	 * that is removed equals the repairFraction attribute.
	 */
	public void repairRoadmap() {
		List<RADPRMWaypoint> waypoints = new ArrayList<RADPRMWaypoint>();

		for (RADPRMWaypoint waypoint : this.getWaypointList()) {
			int numberNeighbors = waypoint.getNeighbors().size();
			waypoint.setUtility(1.0 / (1.0 + numberNeighbors));
		}

		int toRepair = (int) Math.round(this.getWaypointList().size() * repairFraction);

		waypoints = this.getWaypointList().stream().sorted(Comparator.comparing(a -> a.getUtility()))
				.collect(Collectors.toList());

		for(int i = 0 ; i < toRepair ; i++) {
			RADPRMWaypoint toRemove = waypoints.get(0);
			for (RADPRMWaypoint neighbor : toRemove.getNeighbors()) {
				neighbor.getNeighbors().remove(toRemove);
				int numberNeighbors = neighbor.getNeighbors().size();
				neighbor.setUtility(1.0 / (1.0 + numberNeighbors));
			}
			// remove all edges that contain the waypoint
			this.getEdgeList().removeIf(s -> s.contains(toRemove));
			
			// remove waypoint
			this.getWaypointList().remove(toRemove);
			
			// resort the list
			waypoints = this.getWaypointList().stream().sorted(Comparator.comparing(a -> a.getUtility()))
					.collect(Collectors.toList());
		}
		// for (RADPRMWaypoint wpt : this.getWaypointList()) {
		// System.out.printf("%.3f ", wpt.getUtility());
		// }
		// System.out.println(" agr os mais baixoss");
		// for (RADPRMWaypoint wpt : waypoints) {
		// System.out.printf("%.3f ", wpt.getUtility());
		// }
		// for(RADPRMWaypoint wpt : waypoints) {
		// System.out.println(wpt.getUtility());
		// }

		return;
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
	 * @see Planner#plan(Position, Position, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		// if planner is invoked again with the same environment, and query mode is
		// single, then lists are cleared
		if (this.getMode() == QueryMode.SINGLE) {
			this.getWaypointList().clear();
			this.getEdgeList().clear();
		}
		System.out.println("waypoint list before " + this.getWaypointList().size() + " edge list before "
				+ this.getEdgeList().size());
		this.repairRoadmap();
		System.out.println("waypoint list after " + this.getWaypointList().size() + " edge list after "
				+ this.getEdgeList().size());
		this.initialize(origin, destination, etd);
		this.setCostBound(Double.POSITIVE_INFINITY);
		Trajectory trajectory = this.findFeasiblePath();
		this.revisePlan(trajectory);
		this.printData(trajectory);
		do {
			// Anytime
			if (!this.isInflated()) {
				this.updateCostBound();
				trajectory = this.improve();
				this.printData(trajectory);
				this.revisePlan(trajectory);

			}

		} while (!isInflated());

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
		// if planner is invoked again with the same environment, lists are cleared
		this.getWaypointList().clear();
		this.getEdgeList().clear();

		this.initBackups(waypoints.size() + 1);
		Trajectory trajectory = this.oneTimePlan(origin, destination, waypoints, etd);
		this.improving = true;
		while (!this.isInflated()) {
			this.inflate();
			this.backupIndex = -1;
			trajectory = this.oneTimePlan(origin, destination, waypoints, etd);
		}
		this.improving = false;

		return trajectory;
	}

	public void printData(Trajectory trajectory) {
		double size = 0, waypoints = 0, cost = 0d;
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
		return;
	}
}
