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
package com.cfar.swim.worldwind.ai.rrt.hrrt;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;

import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreePlanner;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreeWaypoint;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Status;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Environment;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes an heuristic RRT planner that considers the cost of waypoints in the
 * sampling and in the expanding steps of a basic RRT planner in order to direct
 * the path to a more optimal solution
 * 
 * @author Manuel Rosa
 *
 */
public class HRRTreePlanner extends RRTreePlanner {

	/** floor value to ensure the search is not overly biased against exploration */
	private final double PROB_FLOOR; // Value in [0,1] 0=basic RRT
	
	/** number of neighbors to consider to test a sampled waypoint  */
	private final int NEIGHBORS;

	/** the nearest waypoint in the tree */
	private RRTreeWaypoint waypointNear = null;

	/** the estimated total cost of the optimal path from start to goal */
	private double costOpt;

	/** the maximum cost of all nodes already considered */
	private double costMax;

	/** the heuristic algorithm for the planner */
	private Heuristic heuristic = Heuristic.hRRT;

	/**
	 * Constructs a basic RRT planner for a specified aircraft and environment using
	 * default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see RRTreePlanner#RRTreePlanner(Aircraft, Environment)
	 */
	public HRRTreePlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		PROB_FLOOR = 0.9;
		NEIGHBORS = 5;
	}

	/**
	 * Constructs an heuristic RRT planner for a specified aircraft and environment
	 * using default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * @param epsilon the maximum distance to extend a waypoint in the tree
	 * @param bias the bias of the sampling algorithm towards goal
	 * @param maxIter the maximum number of sampling iterations
	 * 
	 * @see RRTreePlanner#RRTreePlanner(Aircraft, Environment, double, int, int)
	 */
	public HRRTreePlanner(Aircraft aircraft, Environment environment, double epsilon, int bias, int maxIter,
			double prob, int neighbors) {
		super(aircraft, environment, epsilon, bias, maxIter);
		PROB_FLOOR = prob;
		NEIGHBORS = neighbors;
	}

	/**
	 * Gets the nearest RRT waypoint added to the tree.
	 * 
	 * @return the waypointNear the nearest waypoint added to the tree
	 */
	public RRTreeWaypoint getWaypointNear() {
		return waypointNear;
	}

	/**
	 * Sets the nearest RRT waypoint added to the tree.
	 * 
	 * @param waypointNear the nearest waypoint added to the tree
	 */
	public void setWaypointNear(RRTreeWaypoint waypointNear) {
		this.waypointNear = waypointNear;
	}

	/**
	 * Gets the heuristic algorithm of this planner.
	 * 
	 * @return the heuristic algorithm
	 */
	public Heuristic getHeuristic() {
		return heuristic;
	}

	/**
	 * Sets the heuristic algorithm of this planner.
	 * 
	 * @param heuristic the heuristic algorithm
	 */
	public void setHeuristic(Heuristic heuristic) {
		this.heuristic = heuristic;
	}

	/**
	 * Gets the probability floor for the acceptance of a sampled waypoint
	 * 
	 * @return the probability floor
	 */
	public double getPROB_FLOOR() {
		return PROB_FLOOR;
	}
	
	/**
	 * Gets number of neighbors to consider to test a sampled waypoint
	 * 
	 * @return the number of neighbors
	 */
	public int getNEIGHBORS() {
		return NEIGHBORS;
	}

	/**
	 * Selects the waypoint from the environment space according to quality
	 * parameters.
	 * 
	 * @return the selected waypoint
	 */
	protected RRTreeWaypoint selectWaypoint() {
		RRTreeWaypoint waypointRand = null, waypointNear = null;

		Random r = new Random();
		double quality = -1.0, rand = 0;

		while (rand > quality) {
			waypointRand = super.sampleBiased(super.getBIAS());
			waypointNear = (RRTreeWaypoint) this.findNearest(waypointRand, 1).get(0);

			quality = 1 - (waypointNear.getF() - costOpt) / (costMax - costOpt);
			quality = (quality < PROB_FLOOR) ? quality : PROB_FLOOR;

			rand = r.nextDouble();
		}

		this.waypointNear = waypointNear;
		return waypointRand;
	}

	/**
	 * Selects a good waypoint according to a quality parameter by repeatedly
	 * sampling a random waypoint from the environment space and iteratively testing
	 * the k-nearest waypoints sorted by decreasing quality
	 * 
	 * @param neighbors the number of number of neighbors to test
	 * 
	 * @return the selected waypoint
	 */
	@SuppressWarnings("unchecked")
	protected RRTreeWaypoint selectWaypointIK(int neighbors) {
		RRTreeWaypoint waypointRand = null;
		ArrayList<RRTreeWaypoint> neighborsList = new ArrayList<RRTreeWaypoint>();

		Random r = new Random();
		double quality, rand;

		while (true) {
			waypointRand = super.sampleBiased(super.getBIAS());
			neighborsList = (ArrayList<RRTreeWaypoint>) this.findNearest(waypointRand, neighbors);
			neighborsList = this.sortByQuality(neighborsList);

			for (RRTreeWaypoint neighbor : neighborsList) {
				quality = 1 - (neighbor.getF() - costOpt) / (costMax - costOpt);
				quality = (quality < PROB_FLOOR) ? quality : PROB_FLOOR;
				rand = r.nextDouble();
				if (rand < quality) {
					this.waypointNear = neighbor;
					return waypointRand;
				}
			}
		}

	}

	/**
	 * Selects a good waypoint according to a quality parameter by repeatedly
	 * sampling a random waypoint from the environment space and testing the best of
	 * the k-nearest waypoints
	 * 
	 * @param neighbors the number of number of neighbors to test
	 * 
	 * @return the selected waypoint
	 */
	@SuppressWarnings("unchecked")
	protected RRTreeWaypoint selectWaypointBK(int neighbors) {
		RRTreeWaypoint waypointRand = null, waypointNear = null;
		ArrayList<RRTreeWaypoint> neighborsList = new ArrayList<RRTreeWaypoint>();

		Random r = new Random();
		double quality = -1.0, rand = 0;

		while (rand > quality) {
			waypointRand = super.sampleBiased(super.getBIAS());
			neighborsList = (ArrayList<RRTreeWaypoint>) this.findNearest(waypointRand, neighbors);
			neighborsList = this.sortByQuality(neighborsList);
			waypointNear = neighborsList.get(0);

			quality = 1 - (waypointNear.getF() - costOpt) / (costMax - costOpt);
			quality = (quality < PROB_FLOOR) ? quality : PROB_FLOOR;

			rand = r.nextDouble();
		}
		
		this.waypointNear = waypointNear;
		return waypointRand;
	}

	/**
	 * Sorts a list of waypoints by decreasing quality (i.e. by increasing f-value)
	 * 
	 * @param waypointList the list to be sorted
	 * 
	 * @return the sorted list of waypoints
	 */
	protected ArrayList<RRTreeWaypoint> sortByQuality(ArrayList<RRTreeWaypoint> waypointList) {
		Collections.sort(waypointList, (a, b) -> a.getF() < b.getF() ? -1 : a.getF() == b.getF() ? 0 : 1);
		return waypointList;
	}
	
	/**
	 * Connects the tree in the direction of the given waypoint until it is reached
	 * or the tree is trapped. Repeatedly calls extendRRT and returns the status
	 * according to the last result of the extension
	 * 
	 * @param waypoint the waypoint set as the goal for extension
	 * @return status the status resulting from the last extend
	 * 
	 * @see RRTreePlanner#connectRRT(RRTreeWaypoint waypoint)
	 */
	@Override
	protected Status connectRRT(RRTreeWaypoint waypoint) {
		Status status = Status.ADVANCED;

		while (status == Status.ADVANCED && !this.checkGoal(this.getWaypointNew())) {
			status = this.extendRRT(waypoint);
		}

		return status;
	}

	/**
	 * Extends the tree in the direction of the given waypoint and returns the
	 * status according to the result of the extension.
	 * 
	 * @param waypoint the waypoint set as the goal for extension
	 * @return status the status resulting from the extension
	 * 
	 * @see RRTreePlanner#extendRRT(RRTreeWaypoint waypoint)
	 */
	@Override
	protected Status extendRRT(RRTreeWaypoint waypoint) {
		Status status;
		RRTreeWaypoint waypointNear =this.getWaypointNear() ,waypointNew;

		// Create a new node by extension from near to sampled
		boolean success = this.newWaypoint(waypoint, waypointNear);

		// Set status variable by checking if extension was possible
		if (success) {
			waypointNew = this.getWaypointNew();
			this.getWaypointList().add(waypointNew);
			waypointNew.setCostIntervals(this.getContinuumEnvironment().embedIntervalTree(waypointNew));
			waypointNew.updateCost();
			waypointNew.setG(this.computeCost(waypointNew));
			waypointNew.setH(this.computeHeuristic(waypointNew, super.getGoal()));
			this.setWaypointNew(waypointNew); // Needed??

			costMax = (costMax > waypointNew.getF()) ? costMax : waypointNew.getF();

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
	 * Computes the estimated cost of a specified RRT waypoint.
	 * 
	 * @param waypoint the specified RRT waypoint in globe coordinates
	 * 
	 * @return g the estimated cost (g-value)
	 */
	protected double computeCost(RRTreeWaypoint waypoint) {
		double g = waypoint.getParent().getG();
		g += this.getContinuumEnvironment().getStepCost(waypoint.getParent(), waypoint,
				this.getCostPolicy(), this.getRiskPolicy());

		return g;
	}

	/**
	 * Computes the estimated remaining cost (h-value) of a specified source RRT
	 * waypoint to reach the target RRT waypoint.
	 * 
	 * @param source the source RRT waypoint in globe coordinates
	 * @param target the target RRT waypoint in globe coordinates
	 * 
	 * @return the estimated remaining cost (h-value)
	 */
	protected double computeHeuristic(RRTreeWaypoint source, RRTreeWaypoint target) {
		return this.getEnvironment().getNormalizedDistance(source, target);
	}

	/**
	 * Initializes the planner to plan from an origin to a destination at a
	 * specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @see RRTreePlanner#initialize(Position origin, Position destination,
	 *      ZonedDateTime etd)
	 */
	@Override
	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {
		this.clearWaypoints();

		this.setGoal(new RRTreeWaypoint(destination));
		this.getGoal().setH(0d);
		
		RRTreeWaypoint start = new RRTreeWaypoint(origin);
		start.setEto(etd); start.setAto(etd);
		start.setCostIntervals(this.getContinuumEnvironment().embedIntervalTree(start));
		start.updateCost();
		start.setG(0d);
		start.setH(this.computeHeuristic(start, this.getGoal()));

		this.setStart(start);
		this.getWaypointList().add(start);
		this.setWaypointNew(start);

		this.costOpt = start.getF();
		this.costMax = 0;// this.costOpt;
	}

	/**
	 * Computes a plan by growing a tree until the goal is reached.
	 * 
	 * @see RRTreePlanner#compute()
	 */
	@Override
	protected void compute() {
		for (int i = 0; i < super.getMAX_ITER(); i++) {
			RRTreeWaypoint waypointRand;
			boolean status;

			switch (this.getHeuristic()) {
			case BkRRT:
				waypointRand = this.selectWaypointBK(this.getNEIGHBORS());
				break;
			case IkRRT:
				waypointRand = this.selectWaypointIK(this.getNEIGHBORS());
				break;
			case hRRT:
			default:
				waypointRand = this.selectWaypoint();
			}

			switch (super.getStrategy()) {
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
		System.out.println("No path found after " + super.getMAX_ITER() + " iterations.");
	}

}
