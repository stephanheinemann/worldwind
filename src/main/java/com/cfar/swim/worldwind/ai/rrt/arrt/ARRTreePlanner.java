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
package com.cfar.swim.worldwind.ai.rrt.arrt;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.ai.AnytimePlanner;
import com.cfar.swim.worldwind.ai.OnlinePlanner;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreePlanner;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.RRTreeWaypoint;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Position;

/**
 * Realizes an anytime RRT planner that plans a trajectory of an aircraft in an
 * environment considering a local cost and risk policy. The planner provides
 * various solutions depending on the deliberating time it is given and it
 * continuously improves the cost of each solution by a given factor
 * 
 * @author Manuel Rosa
 *
 */
public class ARRTreePlanner extends RRTreePlanner implements AnytimePlanner, OnlinePlanner {

	// TODO: How to define this values?
	private static final int MAX_SAMPLE_ATTEMPTS = 50;
	private static final int MAX_NEIGHBORS = 5;

	/** the initial relative weight of costs to calculate the cost of a waypoint */
	private double initialCostBias; // Change to final?

	/** the final relative weight of costs to calculate the cost of a waypoint */
	private double finalCostBias; // Change to final?

	/** the improvement factor for the cost of each new generated solution */
	private double improvementFactor; // Change to final?

	/** the relative weight of distances to calculate the cost of a waypoint */
	private double distBias = 1d;

	/** the relative weight of costs to calculate the cost of a waypoint */
	private double costBias = 0d;

	/** the step used to modify the relative weights of distances and costs */
	// TODO: might be interesting to define proportionally to improvementFactor
	private double step = 0.1d;

	/** the cost value bounding any new solution to be generated */
	private double costBound = Double.POSITIVE_INFINITY;

	/**
	 * Constructs an anytime RRT planner for a specified aircraft and environment
	 * using default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see RRTreePlanner#RRTreePlanner(Aircraft, Environment)
	 */
	public ARRTreePlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		this.online = false;
		this.positionThreshold = 2d;
		this.updateStep = 5d;
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
	 * 
	 * @see RRTreePlanner#RRTreePlanner(Aircraft, Environment, double, int, int)
	 */
	public ARRTreePlanner(Aircraft aircraft, Environment environment, double epsilon, int bias, int maxIter,
			boolean online, double positionThreshold, double updateStep) {
		super(aircraft, environment, epsilon, bias, maxIter);
		this.online = online;
		this.positionThreshold = positionThreshold;
		this.updateStep = updateStep;
	}

	/**
	 * Gets the minimum quality (initial cost bias) of this ARRT planner.
	 * 
	 * @return the minimum quality (initial cost bias) of this ARRT planner
	 * 
	 * @see AnytimePlanner#getMinimumQuality()
	 */
	@Override
	public double getMinimumQuality() {
		return initialCostBias;
	}

	/**
	 * Sets the minimum quality (initial inflation) of this ARRT planner.
	 * 
	 * @param initialCostBias the minimum quality (initial cost bias) of this ARRT
	 *            planner
	 * 
	 * @see AnytimePlanner#setMinimumQuality(double)
	 */
	@Override
	public void setMinimumQuality(double initialCostBias) {
		this.initialCostBias = initialCostBias;
	}

	/**
	 * Gets the maximum quality (final cost bias) of this ARRT planner.
	 * 
	 * @return the maximum quality (final cost bias) of this ARRT planner
	 * 
	 * @see AnytimePlanner#getMaximumQuality()
	 */
	@Override
	public double getMaximumQuality() {
		return finalCostBias;
	}

	/**
	 * Sets the maximum quality (final cost bias) of this ARRT planner.
	 * 
	 * @param finalCostBias the maximum quality (final cost bias) of this ARRT
	 *            planner
	 * 
	 * @throws IllegalArgumentException if the final inflation is invalid
	 * 
	 * @see AnytimePlanner#setMaximumQuality(double)
	 */
	@Override
	public void setMaximumQuality(double finalCostBias) {
		this.finalCostBias = finalCostBias;
	}

	/**
	 * Gets the quality improvement of this ARRT planner.
	 * 
	 * @return the quality improvement of this ARRT planner
	 * 
	 * @see AnytimePlanner#getQualityImprovement()
	 */
	@Override
	public double getQualityImprovement() {
		return improvementFactor;
	}

	/**
	 * Sets the quality improvement of this ARRT planner.
	 * 
	 * @param improvementFactor the quality improvement of this ARRT planner
	 * 
	 * @see AnytimePlanner#setQualityImprovement(double)
	 */
	@Override
	public void setQualityImprovement(double improvementFactor) {
		this.improvementFactor = improvementFactor;
	}

	/**
	 * Gets the relative weight of distances to calculate the cost of a waypoint
	 * 
	 * @return the the relative weight of distances
	 */
	public double getDistBias() {
		return distBias;
	}

	/**
	 * Sets the relative weight of distances to calculate the cost of a waypoint
	 * 
	 * @param distBias the relative weight of distances to set
	 */
	public void setDistBias(double distBias) {
		this.distBias = distBias;
	}

	/**
	 * Gets the relative weight of costs to calculate the cost of a waypoint
	 * 
	 * @return the relative weight of costs
	 */
	public double getCostBias() {
		return costBias;
	}

	/**
	 * Sets the relative weight of costs to calculate the cost of a waypoint
	 * 
	 * @param costBias the relative weight of costs to set
	 */
	public void setCostBias(double costBias) {
		this.costBias = costBias;
	}

	/**
	 * Gets the step used to modify the relative weights of distances and costs
	 * 
	 * @return the step used to modify the relative weights
	 */
	public double getStep() {
		return step;
	}

	/**
	 * Sets the step used to modify the relative weights of distances and costs
	 * 
	 * @param step the step used to modify the relative weights to set
	 */
	public void setStep(double step) {
		this.step = step;
	}

	/**
	 * Gets the cost value bounding any new solution to be generated
	 * 
	 * @return the bounding cost value
	 */
	public double getCostBound() {
		return costBound;
	}

	/**
	 * Sets the cost value bounding any new solution to be generated
	 * 
	 * @param costBound the bounding cost value to set
	 */
	public void setCostBound(double costBound) {
		this.costBound = costBound;
	}

	// TODO Remove???? Not used...
	@SuppressWarnings("unchecked")
	protected Trajectory partialTrajectory(Trajectory trajectory, Waypoint origin, Waypoint destination) {
		if (trajectory.isEmpty())
			return trajectory;

		LinkedList<Waypoint> partialPlan = new LinkedList<>();
		boolean contained = false;

		for (Waypoint waypoint : trajectory.getWaypoints()) {
			if (!contained)
				contained = waypoint == origin;
			if (contained) {
				partialPlan.add(waypoint);
				if (waypoint == destination)
					break;
			}
		}

		return new Trajectory((List<Waypoint>) partialPlan.clone());
	}

	/**
	 * Selects the cost between two waypoints considering the relative weights of
	 * distance and cost at the moment of the call
	 * 
	 * @param waypoint the new waypoint to the tree towards
	 * @param waypointNear the parent waypoint in the tree
	 * 
	 * @return the cost of the new waypoint with specific relative weights
	 */
	protected double selectCost(RRTreeWaypoint waypoint, RRTreeWaypoint waypointNear) {
		return this.getDistBias() * this.getEnvironment().getDistance(waypoint, waypointNear)
				+ this.getCostBias() * waypoint.getCost();
	}

	/**
	 * Sorts a list of elements by increasing selected cost to a given waypoint
	 * 
	 * @param list the list with the elements to be sorted by selected cost
	 * @param waypoint the waypoint to be considered as reference for the selected
	 *            cost
	 * 
	 * @return a new array list with its elements sorted by selected cost
	 */
	protected ArrayList<RRTreeWaypoint> sortSelectedCost(ArrayList<RRTreeWaypoint> list, RRTreeWaypoint waypoint) {

		return list.stream()
				.sorted((w1, w2) -> Double.compare(this.selectCost(waypoint, w1), this.selectCost(waypoint, w2)))
				.collect(Collectors.toCollection(ArrayList::new));

	}

	/**
	 * Checks whether or not the current solution can be considered as having the
	 * maximum improvement possible.
	 * 
	 * @return true if improvements reached a maximum, false otherwise
	 */
	protected boolean isImproved(double costOld) {
		double costDiff = (costOld - this.getGoal().getCost()) / costOld;
		return costDiff <= 0.05 && getCostBias() >= getMaximumQuality();
	}

	protected boolean isImproved(double costOld, double costNew) {
		double costDiff = (costOld - costNew) / costOld;
		return costDiff <= 0.05 && getCostBias() >= getMaximumQuality();
	}

	/**
	 * Updates the cost value bounding the next solution to be generated
	 */
	protected void updateCostBound() {
		this.setCostBound((1 - improvementFactor) * getGoal().getCost());
	}

	/**
	 * Updates the relative weights of distances and costs.
	 */
	protected void updateWeights() {
		this.setDistBias(distBias - step < 1 - getMaximumQuality() ? 1 - getMaximumQuality() : distBias - step);
		this.setCostBias(costBias + step > getMaximumQuality() ? getMaximumQuality() : costBias + step);
	}

	/**
	 * Samples a new waypoint from the environment with a given bias to the goal and
	 * respecting the cost bound value defined for the desired solution.
	 * 
	 * @param bias the percentage value of bias to sample the goal
	 * 
	 * @return waypoint the RRTreeWaypoint sampled
	 */
	protected RRTreeWaypoint sampleTarget(double bias) {
		RRTreeWaypoint waypoint = null;
		int rand = new Random().nextInt(100 - 1) + 1;

		if (rand <= bias) {
			return this.getGoal();
		}

		// Most points sampled from environment will not be inside heuristic region
		// TODO: Should another sampling method be used? Heuristically based sampling?
		double heuristic = Double.POSITIVE_INFINITY;
		int attempts = 0;
		while (heuristic >= this.getCostBound()) {
			if (attempts > MAX_SAMPLE_ATTEMPTS)
				return null;
			waypoint = this.sampleRandom();
			heuristic = this.computeHeuristic(getStart(), waypoint) + this.computeHeuristic(waypoint, getGoal());
			attempts++;
		}

		return waypoint;
	}

	/**
	 * Creates a new waypoint from a random position sampled from inside an
	 * ellipsoid defined in the environment space.
	 *
	 * @return waypoint the RRTreeWaypoint sampled
	 */
	protected RRTreeWaypoint sampleEllipsoid(Position focusA, Position focusB, double distance) {
		return this.createWaypoint(this.getEnvironment().samplePositionEllipsoide(focusA, focusB, distance));
	}

	/**
	 * Extends the tree in the direction of the target waypoint, choosing to expand
	 * the waypoint from the neighbors in the tree sorted by increasing selected
	 * cost
	 * 
	 * @param waypointSamp the waypoint set as the goal for extension
	 * 
	 * @return true if an extension was successful and false otherwise
	 */
	@SuppressWarnings("unchecked")
	protected boolean extendToTarget(RRTreeWaypoint waypointSamp) {
		RRTreeWaypoint waypointNew;

		ArrayList<RRTreeWaypoint> neighborsList = (ArrayList<RRTreeWaypoint>) this.getEnvironment()
				.findNearest(waypointSamp, MAX_NEIGHBORS);

		neighborsList = this.sortSelectedCost(neighborsList, waypointSamp);

		for (RRTreeWaypoint waypointNear : neighborsList) {
			// TODO: Review alternative extension strategies
			if (this.newWaypoint(waypointSamp, waypointNear)) {
				waypointNew = this.getWaypointNew();
				this.addEdge(waypointNew);

				waypointNew.setG(computeCost(waypointNew));
				waypointNew.setH(computeHeuristic(waypointNew, getGoal()));

				if (waypointNew.getF() < this.getCostBound()) {
					this.addVertex(waypointNew);
					this.setWaypointNew(waypointNew);
					return true;
				} else {
					// Edge should only stay in tree if previous test is passed
					this.removeEdge(waypointNew);
				}
			}
		}

		return false;
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
		super.initialize(origin, destination, etd);

		this.getGoal().setH(0d);
		this.getStart().setH(this.computeHeuristic(getStart(), getGoal()));
	}

	/**
	 * Computes a plan by growing a tree until the goal is reached
	 * 
	 * @see RRTreePlanner#compute()
	 */
	@Override
	protected boolean compute() {
		boolean status = false;
		for (int i = 0; i < this.getMAX_ITER(); i++) {
			RRTreeWaypoint waypointRand = this.sampleTarget(this.getBIAS());
			if (waypointRand != null) {
				status = this.extendToTarget(waypointRand);
				if (status) {
					if (this.checkGoal()) {
						this.getGoal().setG(this.computeCost(getWaypointNew()));
						this.computePath();
						return true;
					}
				}
			}

		}
		return false;
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
	 * @see RRTreePlanner#plan(Position, Position, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		this.initialize(origin, destination, etd);

		this.setCostBias(this.getMinimumQuality());
		this.setDistBias(1 - this.getCostBias());
		this.setCostBound(Double.POSITIVE_INFINITY);

		Trajectory trajectory = new Trajectory();
		boolean newPlan;
		double costOld = Double.POSITIVE_INFINITY;
		do {
				// Anytime
				if(!this.isImproved(costOld)) {
					System.out.println("Anytime planning!!!");
					this.clearExpendables();
					this.addVertex(getStart());
					this.setWaypointNew(getStart());

					costOld = this.getGoal().getCost();
					newPlan = this.compute();

					this.updateWeights();
					if (newPlan) {
						this.updateCostBound();
						trajectory = this.createTrajectory();
						this.revisePlan(trajectory);
					}
				}
				// Online
				if (isOnline()) {
					System.out.println("Online planning!!!");
					this.updateAircraftTimedPosition();
					System.out.println("Position updated!!");
					
					// Check if the aircraft moved significantly
					if (isSignificantMovement(getStart(), getAircraftTimedPosition())) {
						System.out.println("The aircraft moved significantly from previous start!!!");
						RRTreeWaypoint waypoint = this.findClosestWaypoint(getAircraftTimedPosition(), this.getPlan());
						System.out.println("Real:  "+getAircraftTimedPosition()+"\t"+getAircraftTimedPosition().getAto());
						System.out.println("Close: "+waypoint+"\t"+waypoint.getEto());
						
						// Check if the displacement is worthy of a new plan or just an improvement
						// TODO use interpolated position from edge instead of extreme waypoint
						if(isSignificantMovement(waypoint, getAircraftTimedPosition())) {
							System.out.println("The aircraft is too far from the closest waypoint for small adjust!!!");
							this.initialize(getAircraftTimedPosition(), destination, getAircraftTimedPosition().getAto());
							
							this.setCostBias(this.getMinimumQuality());
							this.setDistBias(1 - this.getCostBias());
							this.setCostBound(Double.POSITIVE_INFINITY);
							costOld = Double.POSITIVE_INFINITY;
						}
						else {
							System.out.println("The plan is still good, I got this!");
							waypoint.setG(0d);
//							waypoint.setParent(null);
							this.setStart(waypoint);
							this.getStart().setH(this.computeHeuristic(getStart(), getGoal()));
						}
						
					}
					else {
						System.out.println("I think the aircraft is stopped... Am I right?");
					}
				}
			
		} while (!this.isImproved(costOld) || (isOnline() && !isInsideGoalRegion()));
		System.out.println("Improved");

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
	 * @see RRTree#plan(Position, Position, List, ZonedDateTime)
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Trajectory plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {

		LinkedList<Waypoint> plan = new LinkedList<>();
		Waypoint currentOrigin;
		ZonedDateTime currentEtd;
		Trajectory trajectory;

		// collect intermediate destinations
		ArrayList<Waypoint> destinations = waypoints.stream().map(Waypoint::new)
				.collect(Collectors.toCollection(ArrayList::new));
		destinations.add(new Waypoint(destination));

		// initialize auxiliary variables
		ArrayList<LinkedList<Waypoint>> planList = new ArrayList<>(destinations.size());
		double[] boundCosts = new double[destinations.size()];
		double[] newCosts = new double[destinations.size()];
		double[] oldCosts = new double[destinations.size()];
		for (int i = 0; i < destinations.size(); i++) {
			planList.add(new LinkedList<Waypoint>());
			boundCosts[i] = Double.POSITIVE_INFINITY;
			newCosts[i] = Double.POSITIVE_INFINITY;
			oldCosts[i] = Double.POSITIVE_INFINITY;
		}

		Trajectory part;
		Waypoint currentDestination;
		boolean newPlan, newPart, allImproved = false;

		this.setCostBias(this.getMinimumQuality());
		this.setDistBias(1 - this.getCostBias());

		// Create new trajectories until all paths are considered improved
		while (!allImproved) {
			// reset boolean values
			allImproved = true;
			newPlan = false;

			currentOrigin = new Waypoint(origin);
			currentEtd = etd;
			plan.clear();

			// plan and concatenate partial trajectories
			for (int i = 0; i < destinations.size(); i++) {
				currentDestination = destinations.get(i);
				this.setCostBound(boundCosts[i]);

				this.initialize(currentOrigin, currentDestination, currentEtd);

				newPart = this.compute();

				if (newPart) {
					part = this.createTrajectory();
					planList.set(i, (LinkedList<Waypoint>) this.getPlan().clone());
					this.updateCostBound();
					boundCosts[i] = this.getCostBound();
					newCosts[i] = this.getGoal().getCost();
				} else {
					part = new Trajectory((List<Waypoint>) planList.get(i).clone());
				}

				// newPlan must be true when at least one segment has a new plan
				newPlan = newPlan || newPart;
				// allImporved is true only when all segments are improved
				allImproved = allImproved && this.isImproved(oldCosts[i], newCosts[i]);
				System.out.println("newPart? " + newPart + " isImproved? " + this.isImproved(oldCosts[i], newCosts[i])
						+ "\tOld: " + oldCosts[i] + " New: " + newCosts[i]);

				oldCosts[i] = newCosts[i];

				// append partial trajectory to plan
				if ((!plan.isEmpty()) && (!part.isEmpty())) {
					plan.pollLast();
				}
				for (Waypoint waypoint : part.getWaypoints()) {
					plan.add(waypoint);
				}

				// set current origin from last waypoint in plan to be used in next iteration
				currentOrigin = plan.peekLast();
				currentEtd = currentOrigin.getEto();
			}

			// update weights to be used in next iteration
			this.updateWeights();

			// if the plan was modified
			if (newPlan) {
				// create trajectory from plan
				trajectory = this.createTrajectory(plan);
				this.revisePlan(trajectory);
			}
		}

		System.out.println("Improved...");
		trajectory = this.createTrajectory(plan);
		this.revisePlan(trajectory);
		return trajectory;

	}
	
	// Online Planner
	
	/** the state of the online capabilities of the planner mode (active or not) */
	private final boolean online;
	
	/** the time step to update the current position of the aircraft */
	private final double updateStep;
	
	/** the distance threshold to consider a position displacement as worthy of a new plan */
	private final double positionThreshold; 
	
	/** the current position of the aircraft */
	private Waypoint aircraftTimedPosition;

	/**
	 * Checks if the online capabilities of the planner mode are active or not.
	 * 
	 * @return true if the planner mode is set to online, false otherwise
	 */
	public boolean isOnline() {
		return online;
	}
	
	/**
	 * Gets the  time step to update the current position of the aircraft.
	 * 
	 * @return the time step to update the current position
	 */
	public double getUpdateStep() {
		return updateStep;
	}

	/**
	 * Gets the distance threshold to consider a position displacement as worthy of a new plan.
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
		return this.checkGoal(getAircraftTimedPosition());
	}
	
	/**
	 * Updates the current position of the aircraft in the planner by reading its
	 * actual position from an external source.
	 */
	public void updateAircraftTimedPosition() {
		this.setAircraftTimedPosition(reviseAircraftTimedPosition());
	}
	
	/**
	 * Finds the closest waypoint to a reference position from a list of waypoints.
	 * 
	 * @param position the reference position
	 * @param waypointList the waypoint list to be checked
	 * 
	 * @return the waypoint from the list closest to the position 
	 */
	public RRTreeWaypoint findClosestWaypoint(Waypoint waypoint, List<Waypoint> waypointList) {
		RRTreeWaypoint closest = null;
		for (Waypoint wpt: waypointList) {
			closest = (RRTreeWaypoint) wpt;
			if(wpt.getEto().compareTo(waypoint.getAto())==1)
				break;
		}
		return closest;
	}

}
