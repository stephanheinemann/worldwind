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
package com.cfar.swim.worldwind.ai.rrt.rrtstar;

import java.util.List;

import com.cfar.swim.worldwind.ai.rrt.basicrrt.Extension;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreePlanner;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreeWaypoint;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Strategy;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Environment;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes a RRT* planner that plans a trajectory of an aircraft in an
 * environment considering a local cost and risk policy. The tree is grown from
 * the origin to the goal by connecting a random sampled RRTreeWaypoint to the
 * best RRTreeWaypoint already in the tree. As new points are sampled
 * connections in the tree are evaluated and may be replaced by cheaper
 * solutions. As time advances the produce solution tend to be optimal.
 * 
 * @author Manuel Rosa
 *
 */
public class RRTreeStarPlanner extends RRTreePlanner {

	/**
	 * Constructs a RRT* planner for a specified aircraft and environment using
	 * default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see RRTreePlanner#RRTreePlanner(Aircraft, Environment)
	 */
	public RRTreeStarPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}

	/**
	 * Constructs a RRT* planner for a specified aircraft and environment using
	 * default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * @param epsilon the maximum distance to extend a waypoint in the tree
	 * @param bias the bias of the sampling algorithm towards goal
	 * @param maxIter the maximum number of sampling iterations
	 * @param strategy the expanding strategy for the planner
	 * @param extension the extension technique for the planner
	 * 
	 * @see RRTreePlanner#RRTreePlanner(Aircraft, Environment, double, int, int,
	 *      Strategy, Extension)
	 */
	public RRTreeStarPlanner(Aircraft aircraft, Environment environment, double epsilon, int bias, int maxIter,
			Strategy strategy, Extension extension) {
		super(aircraft, environment, epsilon, bias, maxIter, strategy, extension);
	}

	/**
	 * Calculates the optimal number of neighbors to be considered for each sampled
	 * waypoint as a function of the number of waypoints already present in the
	 * tree.
	 * 
	 * @return the optimal number of neighbors to be considered
	 */
	protected int getOptimalkNear() {
		int n = this.getWaypointList().size();
		double kNear = 2 * Math.E * Math.log(n);

		return (int) Math.round(kNear);
	}
	
	/**
	 * Creates a new waypoint from a random position sampled from a uniformed
	 * distribution over the environment space
	 * 
	 * @return waypoint the RRTreeWaypoint sampled
	 */
	protected RRTreeWaypoint sampleEllipsoidal() {
		return this.createWaypoint(this.getEnvironment().samplePositionEllipsoide(getStart(), getGoal(), getGoal().getCost()));
	}

	/**
	 * Chooses the least expensive waypoint from a list of near neighbors as parent
	 * for the new waypoint.
	 * 
	 * @param waypointNew the waypoint for which a parent is desired
	 * @param nearWaypointList the list of neighbors to be considered as possible
	 *            parents
	 * 
	 * @return the waypoint better suited to be the parent
	 */
	protected RRTreeWaypoint chooseParent(RRTreeWaypoint waypointNew, List<RRTreeWaypoint> nearWaypointList) {
		RRTreeWaypoint parent = waypointNew.getParent();
		double costMin = this.computeCost(waypointNew), cost;

		for (RRTreeWaypoint waypoint : nearWaypointList) {
			if (waypoint.equals(parent))
				continue;

			Position positionNew = null;
			switch (this.getEXTENSION()) {
			case FEASIBLE:
				positionNew = this.growPositionFeasible(waypointNew, waypoint);
			case LINEAR:
			default:
				positionNew = this.growPosition(waypointNew, waypoint);
			}
			if (positionNew != null && positionNew.equals(waypointNew)) {
				cost = this.computeCost(waypointNew, waypoint);
				if (cost <= costMin) {
					costMin = cost;
					this.removeEdge(waypointNew, parent);
					parent = waypoint;
					this.addEdge(waypointNew, parent);
				}
			}
		}

		return parent;
	}

	/**
	 * Rewires the waypoints already present in the tree if there is a cheaper
	 * feasible path starting from the last sampled waypoint.
	 * 
	 * @param waypointNew the last waypoint added to the tree
	 * @param nearWaypointList the list of waypoints to be considered for rewiring
	 */
	protected void rewireTree(RRTreeWaypoint waypointNew, List<RRTreeWaypoint> nearWaypointList) {

		for (RRTreeWaypoint nearWaypoint : nearWaypointList) {
			if (nearWaypoint.equals(waypointNew.getParent()))
				continue;

			Position positionNew = null;
			switch (this.getEXTENSION()) {
			case FEASIBLE:
				positionNew = this.growPositionFeasible(nearWaypoint, waypointNew);
			case LINEAR:
			default:
				positionNew = this.growPosition(nearWaypoint, waypointNew);
			}

			if (positionNew != null && positionNew.equals(nearWaypoint)) {
				double cost = computeCost(nearWaypoint, waypointNew);
				if (cost <= nearWaypoint.getCost()) {
					nearWaypoint.setCost(cost);
					this.removeEdge(nearWaypoint);
					nearWaypoint.setParent(waypointNew);
					this.addEdge(nearWaypoint);
					this.propagateChanges(nearWaypoint);
				}
			}
		}
	}

	/**
	 * Propagates the effect of the changes occurred to a particular waypoint to all
	 * its successors.
	 * 
	 * @param waypointChanged the waypoint whose changes shall be propagated
	 */
	protected void propagateChanges(RRTreeWaypoint waypointChanged) {
		for (RRTreeWaypoint waypoint : getWaypointList()) {
			if (waypoint.getParent() != null)
				if (waypoint.getParent().equals(waypointChanged)) {
					waypoint.setCost(computeCost(waypoint));
					waypoint.setEto(computeTime(waypointChanged, waypoint));
					this.propagateChanges(waypoint);
				}
		}
	}

	/**
	 * Computes a plan by growing a tree until the maximum number of iterations is
	 * reached. Every time a better plan is computed, the revision listener is
	 * called.
	 * 
	 * @return true if the goal was reached, false otherwise
	 */
	@SuppressWarnings("unchecked")
	@Override
	protected boolean compute() {
		boolean success = false;
		double oldCost = this.getGoal().getCost();

		for (int i = 0; i < getMAX_ITER(); i++) {
//			RRTreeWaypoint waypointRand = this.sampleBiased(getBIAS());
			RRTreeWaypoint waypointRand = this.sampleEllipsoidal();
			RRTreeWaypoint waypointNear = (RRTreeWaypoint) this.getEnvironment().findNearest(waypointRand, 1).get(0);

			success = this.newWaypoint(waypointRand, waypointNear);
			// local variable pointing to global (changes affect global)
			RRTreeWaypoint waypointNew = this.getWaypointNew();

			if (success) {
				this.addVertex(waypointNew);
				this.addEdge(waypointNew);

				List<RRTreeWaypoint> nearWaypointList = (List<RRTreeWaypoint>) this.getEnvironment()
						.findNearest(waypointNew, getOptimalkNear());
				RRTreeWaypoint parent = this.chooseParent(waypointNew, nearWaypointList);
				waypointNew.setParent(parent);

				waypointNew.setCost(computeCost(waypointNew, parent));

				this.rewireTree(waypointNew, nearWaypointList);

				if (this.checkGoal())
					if (waypointNew.getCost() < getGoal().getCost())
						this.setGoal(waypointNew);

				if (oldCost > this.getGoal().getCost()) {
					oldCost = this.getGoal().getCost();
					this.computePath(getGoal());
					this.revisePlan(createTrajectory());
				}
			}
		}

		// Check if the goal was reached using the value of its parent
		if (getGoal().getParent() != null)
			return true;
		else
			return false;
	}

}
