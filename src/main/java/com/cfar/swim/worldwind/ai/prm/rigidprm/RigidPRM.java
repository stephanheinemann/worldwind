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
package com.cfar.swim.worldwind.ai.prm.rigidprm;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.ai.AbstractPlanner;
import com.cfar.swim.worldwind.ai.AnytimePlanner;
import com.cfar.swim.worldwind.ai.PlanRevisionListener;
import com.cfar.swim.worldwind.ai.Planner;
import com.cfar.swim.worldwind.ai.astar.arastar.ARAStarPlanner;
import com.cfar.swim.worldwind.ai.astar.astar.ForwardAStarPlanner;
import com.cfar.swim.worldwind.ai.rrt.basicrrt.Sampling;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.planning.Edge;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.SamplingEnvironment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.google.common.collect.Iterables;

import gov.nasa.worldwind.geom.Line;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;

/**
 * Realizes a rigid PRM planner that constructs a roadmap by sampling points in
 * a continuous environment and plans a trajectory of an aircraft in an
 * environment considering a local cost and risk policy. The path is found using
 * an A* based algorithm.
 * 
 * @author Henrique Ferreira
 *
 */
public class RigidPRM extends AbstractPlanner implements AnytimePlanner {

	/** the maximum number of sampling iterations in the construction step */
	protected int maxIterConstruction;

	/** the maximum number of sampling iterations in the enhancement step */
	protected int maxIterEnhancement;

	/**
	 * the permission to connect RigidPRMWaypoints in the same connected component
	 */
	protected boolean sameComponent;

	/** the maximum number of neighbors a RigidPRMWaypoint can be connected to */
	protected int maxNeighbors;

	/** the maximum distance between two neighboring RigidPRMWaypoints */
	protected double maxDistance;

	/** the selection of the variable maximum neighbors expression of PRM* */
	protected boolean optimalMaxNeighbors;

	/** the selection of the variable maximum distance expression of PRM* */
	protected boolean optimalMaxDistance;

	/** the sampling strategy to be applied in the construction step */
	protected Sampling samplingStrategy;

	/** the enhancement technique employed in the enhancement step */
	protected EnhancementMode enhancement;

	/** the type of collision delay technique for RigidPRMWaypoints and edges */
	protected CollisionDelay delayCollision;

	/** the planner used to find a path in a previously populated roadmap */
	protected QueryPlanner planner;

	/** the query mode of this PRM planner */
	protected QueryMode mode;

	/** the initial inflation factor applied to the heuristic function */
	private double initialInflation;

	/** the final inflation factor applied the heuristic function */
	private double finalInflation;

	/** the deflation amount to be applied to the current inflation */
	private double deflationAmount;

	/** the k optimal parameter of PRM* for maximum neighbors */
	private double kprmstar;

	/** the gamma optimal parameter of PRM* for maximum distance */
	private double gammaprmstar;

	/**
	 * Constructs a rigid PRM planner for a specified aircraft and environment using
	 * default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see AbstractPlanner#AbstractPlanner(Aircraft, Environment)
	 */
	public RigidPRM(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		maxIterConstruction = 1000;
		maxNeighbors = 30;
		maxDistance = 200d;
	}

	/**
	 * Gets the maximum number of sampling iterations in the construction step.
	 * 
	 * @return the maxIterConstruction the maximum number of sampling iterations in
	 *         the construction step
	 */
	public int getMaxIterConstruction() {
		return maxIterConstruction;
	}

	/**
	 * Sets the maximum number of sampling iterations in the construction step.
	 * 
	 * @param maxIterConstruction the maximum number of sampling iterations in the
	 *            construction step to set
	 */
	public void setMaxIterConstruction(int maxIter) {
		this.maxIterConstruction = maxIter;
	}

	/**
	 * Gets the maximum number of sampling iterations in the enhancement step.
	 * 
	 * @return the maxIterEnhancement the maximum number of sampling iterations in
	 *         the enhancement step
	 */
	public int getMaxIterEnhancement() {
		return maxIterEnhancement;
	}

	/**
	 * Sets the maximum number of sampling iterations in the enhancement step.
	 * 
	 * @param maxIterEnhancement the maximum number of sampling iterations in the
	 *            enhancement step to set
	 */
	public void setMaxIterEnhancement(int maxIterEnhancement) {
		this.maxIterEnhancement = maxIterEnhancement;
	}

	/**
	 * Checks if the permission to connect RigidPRMWaypoints in the same connected
	 * component is active.
	 * 
	 * @return true if it allows to connect RigidPRMWaypoints in the same connected
	 *         component, false otherwise
	 */
	public boolean isSameComponent() {
		return sameComponent;
	}

	/**
	 * Sets the permission to connect RigidPRMWaypoints in the same connected
	 * component.
	 * 
	 * @param sameComponent the permission level to set
	 */
	public void setSameComponent(boolean sameComponent) {
		this.sameComponent = sameComponent;
	}

	/**
	 * Gets the maximum number of neighbors a RigidPRMWaypoint can have.
	 * 
	 * @return the maxNeighbors the maximum number of neighbors a RigidPRMWaypoint
	 *         can have
	 */
	public int getMaxNeighbors() {
		return maxNeighbors;
	}

	/**
	 * Sets the maximum number of neighbors a RigidPRMWaypoint can have.
	 * 
	 * @param maxNeighbors the maximum number of neighbors a RigidPRMWaypoint can
	 *            have
	 */
	public void setMaxNeighbors(int maxNeighbors) {
		this.maxNeighbors = maxNeighbors;
	}

	/**
	 * Gets the maximum distance between two connected RigidPRMWaypoints.
	 * 
	 * @return the maxDistance the maximum distance between two connected
	 *         RigidPRMWaypoints
	 */
	public double getMaxDistance() {
		return maxDistance;
	}

	/**
	 * Sets the maximum distance between two connected RigidPRMWaypoints.
	 * 
	 * @param maxDistance the maximum distance between two connected
	 *            RigidPRMWaypoints to set
	 */
	public void setMaxDistance(double maxDistance) {
		this.maxDistance = maxDistance;
	}

	/**
	 * Checks if the optimal expression for maximum neighbors is active or not.
	 * 
	 * @return true if the expression is active, false otherwise
	 */
	public boolean isOptimalMaxNeighbors() {
		return optimalMaxNeighbors;
	}

	/**
	 * Sets the status for the optimal expression for maximum neighbors.
	 * 
	 * @param optimalMaxNeighbors the status of the optimal expression for maximum
	 *            neighbors to set
	 */
	public void setOptimalMaxNeighbors(boolean optimalMaxNeighbors) {
		this.optimalMaxNeighbors = optimalMaxNeighbors;
	}

	/**
	 * Checks if the optimal expression for maximum distance is active or not.
	 * 
	 * @return true if the expression is active, false otherwise
	 */
	public boolean isOptimalMaxDistance() {
		return optimalMaxDistance;
	}

	/**
	 * Sets the status for the optimal expression for maximum distance.
	 * 
	 * @param optimalMaxDistance the status of the optimal expression for maximum
	 *            neighbors to set
	 */
	public void setOptimalMaxDistance(boolean optimalMaxDistance) {
		this.optimalMaxDistance = optimalMaxDistance;
	}

	/**
	 * Gets the sampling strategy.
	 * 
	 * @return the samplingStrategy used to create waypoints
	 */
	public Sampling getSamplingStrategy() {
		return samplingStrategy;
	}

	/**
	 * Sets the sampling strategy.
	 * 
	 * @param samplingStrategy the samplingStrategy to set
	 */
	public void setSamplingStrategy(Sampling samplingStrategy) {
		this.samplingStrategy = samplingStrategy;
	}

	/**
	 * Gets the enhancement mode or weight function.
	 * 
	 * @return the enhancement mode or weight function
	 */
	public EnhancementMode getEnhancement() {
		return enhancement;
	}

	/**
	 * Sets the enhancement mode or weight function.
	 * 
	 * @param enhancement the enhancement mode or weight function to set
	 */
	public void setEnhancement(EnhancementMode enhancement) {
		this.enhancement = enhancement;
	}

	/**
	 * Gets the collision delay method.
	 * 
	 * @return the delayCollision the collision delay method.
	 */
	public CollisionDelay getDelayCollision() {
		return delayCollision;
	}

	/**
	 * Sets the collision delay method.
	 * 
	 * @param delayCollision the collision delay method to set
	 */
	public void setDelayCollision(CollisionDelay delayCollision) {
		this.delayCollision = delayCollision;
	}

	/**
	 * Gets the minimum quality (initial inflation) of the query ARA* planner.
	 * 
	 * @return the minimum quality (initial inflation) of the query ARA* planner.
	 * 
	 * @see AnytimePlanner#getMinimumQuality()
	 */
	@Override
	public double getMinimumQuality() {
		return this.initialInflation;
	}

	/**
	 * Sets the minimum quality (initial inflation) of the query ARA* planner.
	 * 
	 * @param initialInflation the minimum quality (initial inflation) of the query
	 *            ARA* planner
	 * 
	 * 
	 * @throws IllegalArgumentException if the initial inflation is invalid
	 * 
	 * @see AnytimePlanner#setMinimumQuality(double)
	 */
	@Override
	public void setMinimumQuality(double initialInflation) {
		if ((1d <= initialInflation) && (initialInflation >= this.finalInflation)) {
			this.initialInflation = initialInflation;
		} else {
			throw new IllegalArgumentException("initial inflation is invalid");
		}
	}

	/**
	 * Gets the maximum quality (final inflation) of the query ARA* planner.
	 * 
	 * @return the maximum quality (final inflation) of the query ARA* planner.
	 * 
	 * @see AnytimePlanner#getMaximumQuality()
	 */
	@Override
	public double getMaximumQuality() {
		return this.finalInflation;
	}

	/**
	 * Sets the maximum quality (initial inflation) of the query ARA* planner.
	 * 
	 * @param finalInflation the maximum quality (final inflation) of the query ARA*
	 *            planner
	 * 
	 * @throws IllegalArgumentException if the final inflation is invalid
	 * 
	 * @see AnytimePlanner#setMaximumQuality(double)
	 */
	@Override
	public void setMaximumQuality(double finalInflation) {
		if ((1d <= finalInflation) && (this.initialInflation >= finalInflation)) {
			this.finalInflation = finalInflation;
		} else {
			throw new IllegalArgumentException("final deflation is invalid");
		}
	}

	/**
	 * Gets the quality improvement (deflation amount) of the query ARA* planner.
	 * 
	 * @return the quality improvement (deflation amount) of the query ARA* planner
	 * 
	 * @see AnytimePlanner#getQualityImprovement()
	 */
	@Override
	public double getQualityImprovement() {
		return this.deflationAmount;
	}

	/**
	 * Sets the quality improvement (deflation amount) of the query ARA* planner.
	 * 
	 * @param deflationAmount the quality improvement (deflation amount) of the
	 *            query ARA* planner
	 * 
	 * @throws IllegalArgumentException if the deflation amount is invalid
	 * 
	 * @see AnytimePlanner#setQualityImprovement(double)
	 */
	@Override
	public void setQualityImprovement(double deflationAmount) {
		if (0d < deflationAmount) {
			this.deflationAmount = deflationAmount;
		} else {
			throw new IllegalArgumentException("deflation amount is invalid");
		}
	}

	/**
	 * Gets the kPRM parameter of PRM* for the maximum number of neighbors.
	 * 
	 * @return the kprmstar the value of the PRM* parameter for the maximum number
	 *         of neighbors
	 */
	public double getKprmstar() {
		return kprmstar;
	}

	/**
	 * Sets the kPRM parameter of PRM* for the maximum number of neighbors.
	 * 
	 * @param kprmstar the value of the PRM* parameter for the maximum number of
	 *            neighbors to set
	 */
	public void setKprmstar(double kprmstar) {
		this.kprmstar = kprmstar;
	}

	/**
	 * Gets the gammaPRM parameter of PRM* for the maximum distance.
	 * 
	 * @return the gammaprmstar the value of the PRM* parameter for the maximum
	 *         distance
	 */
	public double getGammaprmstar() {
		return gammaprmstar;
	}

	/**
	 * Sets the gammaPRM parameter of PRM* for the maximum distance.
	 * 
	 * @param gammaprmstar the value of the PRM* parameter for the maximum distance
	 *            to set
	 */
	public void setGammaprmstar(double gammaprmstar) {
		this.gammaprmstar = gammaprmstar;
	}

	/**
	 * Gets the sampling environment of this planner.
	 * 
	 * @return the sampling environment
	 */
	public SamplingEnvironment getEnvironment() {
		return (SamplingEnvironment) super.getEnvironment();
	}

	/**
	 * Gets the list of already sampled RigidPRMWaypoints
	 * 
	 * @return the list of RigidPRMWaypoints
	 */
	@SuppressWarnings("unchecked")
	public List<RigidPRMWaypoint> getWaypointList() {
		return (List<RigidPRMWaypoint>) this.getEnvironment().getWaypointList();
	}

	/**
	 * Sets the list of RigidPRMWaypoints previously sampled
	 * 
	 * @param RigidPRMWaypointList the list of RigidPRMWaypoints to set
	 * 
	 */
	@SuppressWarnings("unchecked")
	public void setWaypointList(List<? extends Waypoint> RigidPRMWaypointList) {
		this.getEnvironment().setWaypointList((List<Waypoint>) RigidPRMWaypointList);
	}

	/**
	 * Gets the list of already created edges
	 * 
	 * @return the list of edges
	 */
	@SuppressWarnings("unchecked")
	public List<Edge> getEdgeList() {
		return (List<Edge>) this.getEnvironment().getEdgeList();
	}

	/**
	 * Sets the list of edges.
	 * 
	 * @param edgeList the list of edges to set
	 * 
	 */
	public void setEdgeList(List<Edge> edgeList) {
		this.getEnvironment().setEdgeList(edgeList);
	}

	/**
	 * Gets the query planner of this planner.
	 * 
	 * @return the planner used to find a path in this environment.
	 */
	public QueryPlanner getPlanner() {
		return planner;
	}

	/**
	 * Sets the query planner of this planner.
	 * 
	 * @param planner the planner to set
	 */
	public void setPlanner(QueryPlanner planner) {
		this.planner = planner;
	}

	/**
	 * Gets the query mode of this planner.
	 * 
	 * @return the mode the query mode
	 */
	public QueryMode getMode() {
		return mode;
	}

	/**
	 * Sets the query mode of this planner.
	 * 
	 * @param mode the mode to set
	 */
	public void setMode(QueryMode mode) {
		this.mode = mode;
	}

	/**
	 * Creates a RigidPRMWaypoint at a specified position.
	 * 
	 * @param position the position
	 * 
	 * @return the RigidPRMWaypoint at the specified position
	 */
	protected RigidPRMWaypoint createWaypoint(Position position, int component) {
		RigidPRMWaypoint newWaypoint = new RigidPRMWaypoint(position);
		newWaypoint.setEto(this.getEnvironment().getTime());
		newWaypoint.setComponent(component);
		return newWaypoint;
	}

	/**
	 * Computes the expressions for optimal connections from PRM*. Also, if the user
	 * inputs for maximum number of neighbors or distance is 0, then it is
	 * understood as no limitation for connection. Therefore, those variables are
	 * set to infinity.
	 */
	protected void setConnectionStrategies() {
		double freespace, unitball;

		if (this.maxNeighbors == 0) {
			this.maxNeighbors = Integer.MAX_VALUE;
		} else if (this.isOptimalMaxNeighbors()) {
			this.setKprmstar(Math.E * (1.0 + (1.0 / 3.0)));
		}

		if (this.maxDistance == 0) {
			this.maxDistance = Double.MAX_VALUE;
		} else if (this.isOptimalMaxDistance()) {
			freespace = this.getEnvironment().computeFreeVolume();
			unitball = (4.0 * Math.PI * Math.pow(1.0, 3.0)) / 3.0;
			this.setGammaprmstar(
					2.0 * Math.pow(1.0 + (1.0 / 3.0), (1.0 / 3.0)) * Math.pow(freespace / unitball, (1.0 / 3.0)));
		}
	}

	/**
	 * Updates the component number stored in all waypoints, to reflect which
	 * waypoints are in the same connected component.
	 * 
	 * @param waypoint the sampled waypoint
	 * @param neighbor the neighbor waypoint
	 */
	protected void updateComponents(RigidPRMWaypoint waypoint, RigidPRMWaypoint neighbor) {
		int component = waypoint.getComponent();
		for (RigidPRMWaypoint wp : this.getWaypointList()) {
			if (wp.getComponent() == component) {
				wp.setComponent(neighbor.getComponent());
			}
		}
	}

	/**
	 * Connects a given RigidPRMWaypoint to another RigidPRMWaypoints already
	 * sampled, which are closer than a maximum distance. The maximum number of
	 * neighbors a RigidPRMWaypoint can be connected to is another connection
	 * constraint. Checks if the two RigidPRMWaypoints are connectable and if there
	 * is a conflict with terrain obstacles.
	 * 
	 * @param waypoint the RigidPRMWaypoint to be connected
	 */
	protected void connectWaypoint(RigidPRMWaypoint waypoint) {
		int numConnectedNeighbor = 0;

		if (this.isOptimalMaxNeighbors()) {
			this.maxNeighbors = (int) Math.ceil(this.getKprmstar() * Math.log(this.getWaypointList().size()));
		}
		if (this.isOptimalMaxDistance()) {
			int n = this.getWaypointList().size();
			this.maxDistance = this.getGammaprmstar() * Math.pow(Math.log(n) / n, (1.0 / 3.0));
		}

		this.getEnvironment().sortNearest(waypoint);

		for (RigidPRMWaypoint neighbor : this.getWaypointList()) {
			if (this.areConnectable(waypoint, neighbor, numConnectedNeighbor)) {
				if (this.getDelayCollision().equals(CollisionDelay.NONE)) {
					if (!this.getEnvironment().checkConflict(neighbor, waypoint, getAircraft())) {
						numConnectedNeighbor++;
						this.createEdge(waypoint, neighbor);
					} else {
						neighbor.incrementConnectionFailures();
						waypoint.incrementConnectionFailures();
					}
				} else {
					numConnectedNeighbor++;
					this.createEdge(waypoint, neighbor);
				}

			}
		}

	}

	/**
	 * Indicates whether or not two RigidPRMWaypoints are connectable.
	 * 
	 * @param waypoint the RigidPRMWaypoint in globe coordinates
	 * @param neighbor the neighbor in globe coordinates
	 * @param num the number of connected neighbors to the RigidPRMWaypoint
	 * 
	 * @return true if the two RigidPRMWaypoints are connectable, false otherwise
	 */
	protected boolean areConnectable(RigidPRMWaypoint waypoint, RigidPRMWaypoint neighbor, int num) {
		boolean connectable = false;

		Capabilities capabilities = this.getAircraft().getCapabilities();

		if (!this.isSameComponent()) {
			if (neighbor.getComponent() == waypoint.getComponent()) {
				return false;
			}
		}
		if (super.getEnvironment().getDistance(neighbor, waypoint) < maxDistance && num < maxNeighbors) {
			if (capabilities.isFeasible(waypoint, neighbor, this.getEnvironment().getGlobe())
					|| capabilities.isFeasible(neighbor, waypoint, this.getEnvironment().getGlobe())) {
				connectable = true;
			}
		}

		return connectable;
	}

	/**
	 * Creates an edge between a source RigidPRMWaypoint and a target
	 * RigidPRMWaypoint, and adds it to the edge list
	 * 
	 * @param source the source RigidPRMWaypoint
	 * @param target the target RigidPRMWaypoint
	 */
	protected void createEdge(RigidPRMWaypoint source, RigidPRMWaypoint target) {

		Vec4 sourcePoint = this.getEnvironment().getGlobe().computePointFromPosition(source);
		Vec4 targetPoint = this.getEnvironment().getGlobe().computePointFromPosition(target);

		if (sourcePoint.equals(targetPoint))
			return;

		Edge edgeNew = new Edge(source, target, Line.fromSegment(sourcePoint, targetPoint));
		edgeNew.setCostIntervals(this.getEnvironment().embedIntervalTree(edgeNew.getLine()));
		this.getEdgeList().add(edgeNew);
		source.incrementNeighbors();
		target.incrementNeighbors();
		if (!this.isSameComponent()) {
			this.updateComponents(source, target);
		}
	}

	/**
	 * Initializes the planner clearing the RigidPRMWaypoint and edge lists.
	 */
	protected void initialize() {
		this.getWaypointList().clear();
		this.getEdgeList().clear();
	}

	/**
	 * Creates the roadmap by sampling positions from a continuous environment.
	 * First, checks if the RigidPRMWaypoint position has conflicts with terrain.
	 * Then the RigidPRMWaypoint is added to the RigidPRMWaypoint list. After that,
	 * tries to connect this RigidPRMWaypoint to others already sampled.
	 */
	protected void construct() {
		int num = 0;

		this.setConnectionStrategies();

		RigidPRMWaypoint waypoint;
		while (num < maxIterConstruction) {
			switch (this.samplingStrategy) {
			case UNIFORM:
				waypoint = this.createWaypoint(this.getEnvironment().sampleRandomPosition(), num);
				break;
			case GAUSSIAN:
				waypoint = this.createWaypoint(this.getEnvironment().sampleRandomGaussianPosition(), num);
				while (!this.getEnvironment().contains(waypoint)) {
					waypoint = this.createWaypoint(this.getEnvironment().sampleRandomGaussianPosition(), num);
				}
				break;
			default:
				waypoint = this.createWaypoint(this.getEnvironment().sampleRandomPosition(), num);
				break;
			}

			if (this.getDelayCollision().equals(CollisionDelay.FULL)) {
				this.getWaypointList().add(waypoint);
				this.connectWaypoint(waypoint);
				num++;
			} else {
				if (!this.getEnvironment().checkConflict(waypoint, getAircraft())) {
					this.getWaypointList().add(waypoint);
					this.connectWaypoint(waypoint);
					num++;
				}
			}
		}

	}

	/**
	 * Extends the roadmap to incorporate the origin and destination positions.
	 * 
	 * @param origin the origin position in globe coordinates
	 * @param destination the destination position in globe coordinates
	 */
	protected void extendsConstruction(Position origin, Position destination) {
		int totalIter = this.getWaypointList().size()+1;
		RigidPRMWaypoint start = this.createWaypoint(origin, totalIter);
		RigidPRMWaypoint goal = this.createWaypoint(destination, totalIter + 1);

		this.setConnectionStrategies();
		
		if (!this.getEnvironment().checkConflict(start, getAircraft())) {
			this.getWaypointList().add(start);
			this.connectWaypoint(start);
		} else {
			// TODO: execute proper error or exception
			System.out.println("start invalid");
		}

		if (!this.getEnvironment().checkConflict(goal, getAircraft())) {
			this.getWaypointList().add(goal);
			this.connectWaypoint(goal);
		} else {
			// TODO: execute proper error or exception
			System.out.println("goal invalid");
		}
	}

	/**
	 * Extends the roadmap to incorporate the origin, intermediate and destination
	 * positions.
	 * 
	 * @param origin the origin position in globe coordinates
	 * @param destination the destination position in globe coordinates
	 * @param waypoints the list of intermediate positions in globe coordinates
	 */
	protected void extendsConstruction(Position origin, Position destination, List<Position> waypoints) {
		int totalIter = this.maxIterConstruction + this.maxIterEnhancement;
		RigidPRMWaypoint start = this.createWaypoint(origin, totalIter);
		RigidPRMWaypoint goal = this.createWaypoint(destination, totalIter + 1);

		this.setConnectionStrategies();
		
		// TODO: add start and goal to the waypoints list and execute one single cycle
		if (!this.getEnvironment().checkConflict(start, getAircraft())) {
			this.getWaypointList().add(start);
			this.connectWaypoint(start);
		} else {
			// TODO: execute proper error or exception
			System.out.println("start invalid");
		}

		if (!this.getEnvironment().checkConflict(goal, getAircraft())) {
			this.getWaypointList().add(goal);
			this.connectWaypoint(goal);
		} else {
			// TODO: execute proper error or exception
			System.out.println("goal invalid");
		}

		for (Position pos : waypoints) {
			int i = 2;
			RigidPRMWaypoint RigidPRMWaypoint = this.createWaypoint(pos, totalIter + i);

			if (!this.getEnvironment().checkConflict(RigidPRMWaypoint, getAircraft())) {
				this.getWaypointList().add(RigidPRMWaypoint);
				this.connectWaypoint(RigidPRMWaypoint);
			} else {
				// TODO: execute proper error or exception
				System.out.println("wp invalid");
			}
			i++;
		}
	}

	/**
	 * Realizes the post construction step of a rigid PRM planner. The main goal is
	 * to improve the roadmap connectivity and to uniformize the distribution of
	 * RigidPRMWaypoints. It selects the RigidPRMWaypoints with fewer neighbors and
	 * creates a new RigidPRMWaypoint around each of them.
	 */
	public void enhance() {

		List<RigidPRMWaypoint> waypoints = new ArrayList<RigidPRMWaypoint>();
		switch (this.enhancement) {
		case NONE:
			return;
		case NEIGHBOR:
			waypoints = this.getWaypointList().stream().sorted(Comparator.comparing(a -> a.getNeighbors()))
					.limit(maxIterEnhancement).collect(Collectors.toList());
			break;
		case LOCALPLANNER:
			waypoints = this.getWaypointList().stream()
					.sorted(Comparator.comparingInt(RigidPRMWaypoint::getConnectionFailures).reversed())
					.limit(maxIterEnhancement).collect(Collectors.toList());
			break;
		}

		int num = 0;

		while (num < maxIterEnhancement) {
			RigidPRMWaypoint waypoint = waypoints.get(num);
			double halfDistance = this.maxDistance / (Math.sqrt(3));
			RigidPRMWaypoint newWaypoint = this.createWaypoint(this.samplePosition(waypoint, halfDistance),
					this.maxIterConstruction + num);

			if (this.getDelayCollision().equals(CollisionDelay.FULL)) {
				this.getWaypointList().add(newWaypoint);
				this.connectWaypoint(newWaypoint);
				num++;
			} else {
				while (this.getEnvironment().checkConflict(newWaypoint, getAircraft())) {
					halfDistance = halfDistance / 2;
					newWaypoint = this.createWaypoint(this.samplePosition(waypoint, halfDistance),
							this.maxIterConstruction + num);
				}
				this.getWaypointList().add(newWaypoint);
				this.connectWaypoint(newWaypoint);
				num++;
			}
		}
	}

	/**
	 * Samples a position from a continuous space (box) defined around a given
	 * position with specified edge length.
	 * 
	 * @param position the position around which the new position is going to be
	 *            sampled
	 * @param halfEdgeLength the half edge length of this box
	 * 
	 * @return the sampled position in globe coordinates
	 */
	public Position samplePosition(Position position, double halfEdgeLength) {
		Vec4 point = this.getEnvironment().getGlobe().computePointFromPosition(position);
		List<Vec4> corners = new ArrayList<Vec4>();

		corners.add(point.add3(-halfEdgeLength, +halfEdgeLength, -halfEdgeLength));
		corners.add(point.add3(+halfEdgeLength, +halfEdgeLength, -halfEdgeLength));
		corners.add(point.add3(+halfEdgeLength, -halfEdgeLength, -halfEdgeLength));
		corners.add(point.add3(+halfEdgeLength, -halfEdgeLength, +halfEdgeLength));
		corners.add(point.add3(-halfEdgeLength, -halfEdgeLength, +halfEdgeLength));
		corners.add(point.add3(-halfEdgeLength, +halfEdgeLength, +halfEdgeLength));

		SamplingEnvironment env = new SamplingEnvironment(
				new Box(gov.nasa.worldwind.geom.Box.computeBoundingBox(corners)));
		env.setGlobe(this.getEnvironment().getGlobe());
		Position pos = env.sampleRandomPosition();
		return pos;
	}

	/**
	 * Sets the plan revision listeners for a query planner. The query planner
	 * listener invokes the listeners of this rigid PRM planner.
	 * 
	 * @param planner the query planner of this planner
	 */
	public void setRevisionListeners(Planner planner) {
		List<PlanRevisionListener> planRevisionListeners = this.getPlanRevisionListeners();
		planner.addPlanRevisionListener(new PlanRevisionListener() {
			@Override
			public void revisePlan(Trajectory trajectory) {
				for (PlanRevisionListener listener : planRevisionListeners) {
					listener.revisePlan(trajectory);
				}
			}

			@Override
			public void reviseObstacle() {
				for (PlanRevisionListener listener : planRevisionListeners) {
					listener.reviseObstacle();
				}
			}

			@Override
			public Position reviseAircraftPosition() {
				Position position = null;
				for (PlanRevisionListener listener : planRevisionListeners) {
					position = listener.reviseAircraftPosition();
				}
				return position;
			}

			@Override
			public Waypoint reviseAircraftTimedPosition() {
				Waypoint waypoint = null;
				for (PlanRevisionListener listener : planRevisionListeners) {
					waypoint = listener.reviseAircraftTimedPosition();
				}
				return waypoint;
			}

			@Override
			public boolean reviseDatalinkPlan() {
				boolean datalinkConnected = false;
				for (PlanRevisionListener listener : planRevisionListeners) {
					datalinkConnected = listener.reviseDatalinkPlan();
				}
				return datalinkConnected;
			}
		});
	}

	/**
	 * Corrects a trajectory, checking if any of its waypoints or edges is in
	 * conflict with terrain obstacles.
	 * 
	 * @param trajectory the planned trajectory
	 * 
	 * @return true if this trajectory is feasible, false otherwise
	 */
	public boolean correctTrajectoryFull(ForwardAStarPlanner aStar, Trajectory trajectory) {
		if (trajectory == null)
			return false;

		int index;
		for (int i = 0; i < trajectory.getLength(); i++) {
			// if i is even then select a waypoint in the natural order starting from start
			// if i is odd then select a waypoint in the inverse order starting from goal
			index = i / 2;
			if ((i % 2) == 1) {
				index = trajectory.getLength() - 1 - index;
			}
			Waypoint waypoint = Iterables.get(trajectory.getWaypoints(), index);
			Waypoint waypointBefore = null;
			if (index > 0)
				waypointBefore = Iterables.get(trajectory.getWaypoints(), index - 1);
			else
				waypointBefore = Iterables.get(trajectory.getWaypoints(), index);
			RigidPRMWaypoint rigid = (RigidPRMWaypoint) this.getEnvironment().getWaypoint(waypoint).get();
			if (!rigid.isValid()) {
				if (this.getEnvironment().checkConflict(waypoint, getAircraft())) {
					this.getWaypointList().removeIf(s -> s.equals(waypoint));
					this.getEdgeList()
							.removeIf(s -> s.getPosition1().equals(waypoint) || s.getPosition2().equals(waypoint));
					aStar.correctWaypoint(waypoint, waypointBefore);
					return false;
				} else {
					rigid.setValid(true);
				}
			}
		}

		for (int i = 0; i < trajectory.getLength() - 1; i++) {
			index = i / 2;
			if ((i % 2) == 1) {
				index = trajectory.getLength() - 2 - index;
			}
			Waypoint waypointBefore = Iterables.get(trajectory.getWaypoints(), index);
			Waypoint waypoint = Iterables.get(trajectory.getWaypoints(), index + 1);
			Edge edge = this.getEnvironment().getEdge(waypoint, waypointBefore).get();
			if (!edge.isValid()) {
				if (this.getEnvironment().checkConflict(waypointBefore, waypoint, getAircraft())) {
					this.getEdgeList().remove(new Edge(waypointBefore, waypoint));
					aStar.correctWaypoint(waypoint, waypointBefore);
					return false;
				} else {
					edge.setValid(true);
				}

			}
		}
		return true;
	}

	/**
	 * Corrects a trajectory, checking if any of its edges is in conflict with
	 * terrain obstacles.
	 * 
	 * @param trajectory the planned trajectory
	 * 
	 * @return true if this trajectory is feasible, false otherwise
	 */
	public boolean correctTrajectoryHalf(ForwardAStarPlanner aStar, Trajectory trajectory) {
		if (trajectory == null)
			return false;

		int index;
		for (int i = 0; i < trajectory.getLength() - 1; i++) {
			index = i / 2;
			if ((i % 2) == 1) {
				index = trajectory.getLength() - 2 - index;
			}
			Waypoint waypointBefore = Iterables.get(trajectory.getWaypoints(), index);
			Waypoint waypoint = Iterables.get(trajectory.getWaypoints(), index + 1);
			Edge edge = this.getEnvironment().getEdge(waypoint, waypointBefore).get();
			if (!edge.isValid()) {
				if (this.getEnvironment().checkConflict(waypointBefore, waypoint, getAircraft())) {
					this.getEdgeList().remove(new Edge(waypointBefore, waypoint));
					aStar.correctWaypoint(waypoint, waypointBefore);
					return false;
				} else {
					edge.setValid(true);
				}

			}
		}
		return true;
	}

	/**
	 * Invokes a query planner to find a trajectory from an origin to a destination
	 * at a specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 * @param planner the planner used to find a path in this populated environment
	 * 
	 * @return the planned trajectory from the origin to the destination with the
	 *         estimated time of departure
	 */
	public Trajectory findPath(Position origin, Position destination, ZonedDateTime etd, QueryPlanner planner) {
		Trajectory trajectory = null;
		switch (planner) {
		case FAS:
			ForwardAStarPlanner aStar = new ForwardAStarPlanner(this.getAircraft(), this.getEnvironment());
			aStar.setCostPolicy(this.getCostPolicy());
			aStar.setRiskPolicy(this.getRiskPolicy());
			this.setRevisionListeners(aStar);
			trajectory = aStar.plan(origin, destination, etd);
			switch (delayCollision) {
			case NONE:
				break;
			case HALF:
				while (!this.correctTrajectoryHalf(aStar, trajectory)) {
					trajectory = aStar.continueComputing();
					if (trajectory.isEmpty()) {
						break;
					}
				}
				break;
			case FULL:
				while (!this.correctTrajectoryFull(aStar, trajectory)) {
					trajectory = aStar.continueComputing();
					if (trajectory.isEmpty()) {
						break;
					}
				}
				break;
			}
			break;
		// TODO: review ARA implementation
		case ARA:
			ARAStarPlanner araStar = new ARAStarPlanner(this.getAircraft(), this.getEnvironment());
			araStar.setCostPolicy(this.getCostPolicy());
			araStar.setRiskPolicy(this.getRiskPolicy());
			araStar.setMinimumQuality(this.getMinimumQuality());
			araStar.setMaximumQuality(this.getMaximumQuality());
			araStar.setQualityImprovement(this.getQualityImprovement());
			this.setRevisionListeners(araStar);
			trajectory = araStar.plan(origin, destination, etd);
			while (!this.correctTrajectoryHalf(araStar, trajectory)) {
				trajectory = araStar.continueComputing();
			}
			break;
		}
		this.revisePlan(trajectory);
		return trajectory;
	}

	/**
	 * Invokes a query planner to find a trajectory from an origin to a destination
	 * along RigidPRMWaypoints at a specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 * @param planner the planner used to find a path in this populated environment
	 * 
	 * @return the planned trajectory from the origin to the destination with the
	 *         estimated time of departure
	 */

	public Trajectory findPath(Position origin, Position destination, ZonedDateTime etd, List<Position> waypoints,
			QueryPlanner planner) {
		Trajectory trajectory = null;

		// TODO: review multiple waypoints
		switch (planner) {
		case FAS:
			ForwardAStarPlanner aStar = new ForwardAStarPlanner(this.getAircraft(), this.getEnvironment());
			aStar.setCostPolicy(this.getCostPolicy());
			aStar.setRiskPolicy(this.getRiskPolicy());
			this.setRevisionListeners(aStar);
			trajectory = aStar.plan(origin, destination, waypoints, etd);
			while (!this.correctTrajectoryHalf(aStar, trajectory)) {
				trajectory = aStar.plan(origin, destination, waypoints, etd);
			}
			break;
		case ARA:
			ARAStarPlanner araStar = new ARAStarPlanner(this.getAircraft(), this.getEnvironment());
			araStar.setCostPolicy(this.getCostPolicy());
			araStar.setRiskPolicy(this.getRiskPolicy());
			araStar.setMinimumQuality(this.getMinimumQuality());
			araStar.setMaximumQuality(this.getMaximumQuality());
			araStar.setQualityImprovement(this.getQualityImprovement());
			this.setRevisionListeners(araStar);
			trajectory = araStar.plan(origin, destination, waypoints, etd);
			while (!this.correctTrajectoryHalf(araStar, trajectory)) {
				trajectory = araStar.plan(origin, destination, waypoints, etd);
			}
			break;
		}
		return trajectory;
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
		if (this.getEnvironment().getEdgeList().isEmpty()) {
			this.setMode(QueryMode.SINGLE);
		}

		if (this.getMode() == QueryMode.SINGLE) {
			this.initialize();
			this.construct();
			this.enhance();
		}
		this.extendsConstruction(origin, destination);
		Trajectory trajectory = this.findPath(origin, destination, etd, this.planner);
		return trajectory;
	}

	/**
	 * Plans a trajectory from an origin to a destination along RigidPRMWaypoints at
	 * a specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param RigidPRMWaypoints the RigidPRMWaypoints in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @return the planned trajectory from the origin to the destination along the
	 *         RigidPRMWaypoints with the estimated time of departure
	 * 
	 * @see Planner#plan(Position, Position, List, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, List<Position> RigidPRMWaypoints, ZonedDateTime etd) {
		if (this.getEnvironment().getEdgeList().isEmpty()) {
			this.setMode(QueryMode.SINGLE);
		}

		if (this.getMode() == QueryMode.SINGLE) {
			this.initialize();
			this.construct();
			this.extendsConstruction(origin, destination, RigidPRMWaypoints);
			this.enhance();
		} else if (this.getMode() == QueryMode.MULTIPLE) {
			this.extendsConstruction(origin, destination, RigidPRMWaypoints);
		}

		Trajectory trajectory = this.findPath(origin, destination, etd, RigidPRMWaypoints, this.planner);
		return trajectory;
	}

	/**
	 * Indicates whether or not this rigid PRM planner supports a specified
	 * environment.
	 * 
	 * @param environment the environment
	 * 
	 * @return true if the environment is a sampling environment, false otherwise
	 * 
	 * @see SamplingEnvironment
	 */
	@Override
	public boolean supports(Environment environment) {
		boolean supports = super.supports(environment);

		if (supports) {
			supports = (environment instanceof SamplingEnvironment);
		}

		return supports;
	}

	/**
	 * Indicates whether or not this PRM planner supports specified
	 * RigidPRMWaypoints.
	 * 
	 * @param RigidPRMWaypoints the RigidPRMWaypoints
	 * 
	 * @return true if RigidPRMWaypoints are contained in the planner's environment,
	 *         false otherwise
	 */
	@Override
	public boolean supports(List<Position> RigidPRMWaypoints) {
		boolean supports = false;

		supports = super.supports(RigidPRMWaypoints);

		for (Position RigidPRMWaypoint : RigidPRMWaypoints) {
			if (this.getEnvironment().checkConflict(RigidPRMWaypoint, getAircraft()))
				supports = false;
		}

		return supports;
	}
}
