/**
 * Copyright (c) 2021, Stephan Heinemann (UVic Center for Aerospace Research)
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
package com.cfar.swim.worldwind.planners.rrt.brrt;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.CapabilitiesException;
import com.cfar.swim.worldwind.environments.DirectedEdge;
import com.cfar.swim.worldwind.environments.Edge;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.environments.PlanningContinuum;
import com.cfar.swim.worldwind.planners.AbstractPlanner;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.planners.rrt.Extension;
import com.cfar.swim.worldwind.planners.rrt.Sampling;
import com.cfar.swim.worldwind.planners.rrt.Status;
import com.cfar.swim.worldwind.planners.rrt.Strategy;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.planners.rrt.RRTreeProperties;
import com.cfar.swim.worldwind.util.Identifiable;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Plane;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Quaternion;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.util.Logging;

/**
 * Realizes a basic RRT planner that plans a trajectory of an aircraft in an
 * environment considering a local cost and risk policy. The origin of the tree
 * is the departure position and it is grown until the goal region is reached.
 * 
 * @author Manuel Rosa
 * @author Stephan Heinemann
 */
public class RRTreePlanner extends AbstractPlanner {
	
	/** the sampling distribution of this RRT planner */
	private Sampling sampling = Sampling.UNIFORM;
	
	/** the expansion strategy of this RRT planner */
	private Strategy strategy = Strategy.EXTEND;
	
	/** the extension technique of this RRT planner */
	private Extension extension = Extension.LINEAR;
	
	/** the maximum number of sampling iterations of this RRT planner */
	private int maxIterations = 3_000; // [1, Integer.MAX_VALUE]
	
	/** the maximum extension distance to a waypoint in the tree of this RRT planner */
	private double epsilon = this.getEnvironment().getDiameter() / 20d; // (0, Double.MAX_VALUE]
	
	/** the sampling bias towards goal of this RRT planner */
	private int bias = 5; // [0, 100]
	
	/** the radius of the sphere defining the goal region of this RRT planner */
	private double goalThreshold = 5d; // (0, Double.MAX_Value]
	
	/** the start RRT waypoint of this RRT planner */
	private RRTreeWaypoint start = null;
	
	/** the goal RRT waypoint of this RRT planner */
	private RRTreeWaypoint goal = null;
	
	/** the newest waypoint in the tree of this RRT planner */
	private RRTreeWaypoint newestWaypoint = null;
	
	/**
	 * Constructs a basic RRT planner for a specified aircraft and environment using
	 * default local cost and risk policies as well as default RRT properties.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see AbstractPlanner#AbstractPlanner(Aircraft, Environment)
	 */
	public RRTreePlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
	}
	
	/**
	 * Gets the identifier of this RRT planner.
	 * 
	 * @return the identifier of this RRT planner
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return Specification.PLANNER_RRT_ID;
	}
	
	/**
	 * Gets the planning continuum of this RRT planner.
	 * 
	 * @return the planning continuum of this RRT planner
	 * 
	 * @see AbstractPlanner#getEnvironment()
	 */
	public PlanningContinuum getPlanningContinuum() {
		return (PlanningContinuum) super.getEnvironment();
	}
	
	/**
	 * Creates a RRT waypoint at a specified position.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return the RRT waypoint at the specified position
	 */
	protected RRTreeWaypoint createWaypoint(Position position) {
		RRTreeWaypoint waypoint = new RRTreeWaypoint(position);
		
		if (waypoint.equals(this.getStart())) {
			waypoint = this.getStart();
		} else if (waypoint.equals(this.getGoal())) {
			waypoint = this.getGoal();
		}
		
		return waypoint;
	}
	
	/**
	 * Gets the start RRT waypoint of this RRT planner.
	 * 
	 * @return the start RRT waypoint of this RRT planner
	 */
	protected RRTreeWaypoint getStart() {
		return this.start;
	}

	/**
	 * Sets the start RRT waypoint of this RRT planner.
	 * 
	 * @param start the start RRT waypoint of this RRT planner
	 */
	protected void setStart(RRTreeWaypoint start) {
		this.start = start;
	}
	
	/**
	 * Determines whether or not this RRT planner has a start waypoint.
	 * 
	 * @return true if this RRT planner has a start waypoint, false otherwise
	 */
	protected boolean hasStart() {
		return (null != this.start);
	}

	/**
	 * Gets the goal RRT waypoint of this RRT planner.
	 * 
	 * @return the goal RRT waypoint of this RRT planner
	 */
	protected RRTreeWaypoint getGoal() {
		return this.goal;
	}

	/**
	 * Sets the goal RRT waypoint of this RRT planner.
	 * 
	 * @param goal the goal RRT waypoint of this RRT planner
	 */
	protected void setGoal(RRTreeWaypoint goal) {
		this.goal = goal;
	}
	
	/**
	 * Determines whether or not this RRT planner has a goal waypoint.
	 * 
	 * @return true if this RRT planner has a goal waypoint, false otherwise
	 */
	protected boolean hasGoal() {
		return (null != this.goal);
	}
	
	/**
	 * Gets the newest RRT waypoint added to the tree.
	 * 
	 * @return the newest RRT waypoint added to the tree
	 */
	protected RRTreeWaypoint getNewestWaypoint() {
		return this.newestWaypoint;
	}
	
	/**
	 * Sets the newest RRT waypoint added to the tree.
	 * 
	 * @param newestWaypoint the newest RRT waypoint added to the tree
	 */
	protected void setNewestWaypoint(RRTreeWaypoint newestWaypoint) {
		this.newestWaypoint = newestWaypoint;
	}
	
	/**
	 * Gets the sampling distribution of this RRT planner.
	 * 
	 * @return the sampling distribution of this RRT planner
	 */
	public synchronized Sampling getSampling() {
		return this.sampling;
	}
	
	/**
	 * Sets the sampling distribution of this RRT planner.
	 * 
	 * @param sampling the sampling distribution to be set
	 */
	public synchronized void setSampling(Sampling sampling) {
		this.sampling = sampling;
	}
	
	/**
	 * Gets the expansion strategy of this RRT planner.
	 * 
	 * @return the expansion strategy of this RRT planner
	 */
	public synchronized Strategy getStrategy() {
		return this.strategy;
	}
	
	/**
	 * Sets the expansion strategy of this RRT planner.
	 * 
	 * @param strategy the expansion strategy to be set
	 */
	public synchronized void setStrategy(Strategy strategy) {
		this.strategy = strategy;
	}
	
	/**
	 * Gets the extension technique of this RRT planner.
	 * 
	 * @return the extension technique of this RRT planner
	 */
	public synchronized Extension getExtension() {
		return this.extension;
	}
	
	/**
	 * Sets the extension technique of this RRT planner.
	 * 
	 * @param extension the extension technique to be set
	 */
	public synchronized void setExtension(Extension extension) {
		this.extension = extension;
	}
	
	/**
	 * Gets the maximum number of sampling iterations towards the goal of this
	 * RRT planner.
	 * 
	 * @return the maximum number of sampling iterations of this RRT planner
	 */
	public synchronized int getMaxIterations() {
		return this.maxIterations;
	}
	
	/**
	 * Sets the maximum number of sampling iterations towards the goal of this
	 * RRT planner.
	 * 
	 * @param maxIterations the maximum number of sampling iterations to be set
	 * 
	 * @throws IllegalArgumentException if maximum iterations is invalid
	 */
	public synchronized void setMaxIterations(int maxIterations) {
		if ((0 < maxIterations) && (Integer.MAX_VALUE >= maxIterations)) {
			this.maxIterations = maxIterations;
		} else {
			throw new IllegalArgumentException("maximum iterations is invalid");
		}
	}
	
	/**
	 * Gets the maximum extension distance to a waypoint in the tree of this
	 * RRT planner.
	 * 
	 * @return the maximum extension distance of this RRT planner
	 */
	public synchronized double getEpsilon() {
		return this.epsilon;
	}
	
	/**
	 * Sets the maximum extension distance to a waypoint in the tree of this
	 * RRT planner.
	 * 
	 * @param epsilon the maximum extension distance to be set
	 * 
	 * @throws IllegalArgumentException if epsilon is invalid
	 */
	public synchronized void setEpsilon(double epsilon) {
		if ((1d < epsilon) && (Double.MAX_VALUE >= epsilon)) {
			this.epsilon = epsilon;
		} else {
			throw new IllegalArgumentException("epsilon is invalid");
		}
	}

	/**
	 * Gets the sampling bias towards the goal of this RRT planner.
	 * 
	 * @return the sampling bias towards the goal of this RRT planner
	 */
	public synchronized int getBias() {
		return this.bias;
	}
	
	/**
	 * Sets the sampling bias towards the goal of this RRT planner.
	 * 
	 * @param bias the sampling bias to be set
	 * 
	 * @throws IllegalArgumentException if bias is invalid
	 */
	public synchronized void setBias(int bias) {
		if ((0 <= bias) && (100 >= bias)) {
			this.bias = bias;
		} else {
			throw new IllegalArgumentException("bias is invalid");
		}
	}
	
	/**
	 * Gets the the radius of the sphere defining the goal region of this RRT
	 * planner.
	 * 
	 * @return the radius of the sphere defining the goal region of this RRT
	 *         planner
	 */
	public synchronized double getGoalThreshold() {
		return this.goalThreshold;
	}

	/**
	 * Sets the radius of the sphere defining the goal region of this RRT
	 * planner.
	 * 
	 * @param goalThreshold the radius of the sphere defining the goal region
	 *                      to be set
	 * 
	 * @throws IllegalArgumentException if goal threshold is invalid
	 */
	public synchronized void setGoalThreshold(double goalThreshold) {
		if ((0d <= goalThreshold) && (Double.MAX_VALUE >= goalThreshold)) {
			this.goalThreshold = goalThreshold;
		} else {
			throw new IllegalArgumentException("goal threshold is invalid");
		}
	}
	
	/**
	 * Clears the planning data structures of this RRT planner.
	 */
	protected void clearExpendables() {
		this.setStart(null);
		this.setGoal(null);
		this.getPlanningContinuum().clearVertices();
		this.clearWaypoints();
	}
	
	/**
	 * Connects a plan from the newest sampled RRT waypoint backwards to the
	 * starting RRT waypoint.
	 */
	protected void connectPlan() {
		this.connectPlan(this.getNewestWaypoint());
	}
	
	/**
	 * Connects a plan from a RRT waypoint backwards to the starting RRT
	 * waypoint.
	 * 
	 * @param waypoint the RRT waypoint of a computed plan
	 */
	protected void connectPlan(RRTreeWaypoint waypoint) {
		this.clearWaypoints();
		/*
		 * Only connect a plan from the reached goal featuring an ETO without
		 * accepting solutions that exceed the applicable risk policy. Since
		 * basic RRT does not consider costs during the expansion, it is less
		 * likely to find a solution in an environment with obstacles that
		 * exceed the risk policy in a single shot.
		 */
		while ((null != waypoint) && waypoint.hasEto() && !waypoint.hasInfiniteCost()) {
			this.getWaypoints().addFirst(waypoint);
			waypoint = waypoint.getParent();
		}
	}
	
	/**
	 * Samples a waypoint from within the environment using the sampling
	 * distribution of this RRT planner.
	 * 
	 * @return the sampled waypoint
	 */
	protected RRTreeWaypoint sample() {
		RRTreeWaypoint sample = null;
		
		switch (this.getSampling()) {
		case ELLIPSOIDAL:
			sample = this.sampleEllipsoid();
			break;
		case GAUSSIAN:
			sample = this.sampleGaussian();
			break;
		case UNIFORM:
		default:
			sample = this.sampleUniform();
		}
		
		return sample;
	}
	
	/**
	 * Samples a biased waypoint from within the environment using the bias
	 * of this RRT planner towards the goal.
	 * 
	 * @return the sampled waypoint from within the environment using the bias
	 *         towards the goal
	 */
	protected RRTreeWaypoint sampleBiased() {
		int random = new Random().nextInt(100 - 1) + 1;
		return (random <= this.getBias()) ? this.getGoal() : this.sample();
	}
	
	/**
	 * Samples a random waypoint from within the environment using a uniform
	 * distribution.
	 * 
	 * @return the sampled waypoint from within the environment using the
	 *         uniform distribution
	 */
	protected RRTreeWaypoint sampleUniform() {
		return this.createWaypoint(
				this.getPlanningContinuum().sampleRandomUniformPosition());
	}
	
	/**
	 * Samples a random waypoint from within the environment using a Gaussian
	 * (normal) distribution.
	 * 
	 * @return the sampled waypoint from within the environment using the
	 *         Gaussian (normal) distribution
	 */
	protected RRTreeWaypoint sampleGaussian() {
		return this.createWaypoint(
				this.getPlanningContinuum().sampleRandomGaussianPosition());
	}
	
	/**
	 * Samples a random position from within the intersection of the
	 * environment and an ellipsoid defined by the start and goal as its foci.
	 * 
	 * @return the sampled waypoint from within the intersection of the
	 *         environment and the ellipsoid
	 */
	protected RRTreeWaypoint sampleEllipsoid() {
		return this.createWaypoint(
				this.getPlanningContinuum().sampleRandomEllipsoidPosition(
						this.start, this.goal));
	}
	
	/**
	 * Creates a sampling shape in the environment of this RRT planner.
	 */
	protected void createSamplingShape() {
		switch(this.getSampling()) {
		case ELLIPSOIDAL:
			this.getPlanningContinuum().setEllipsoidShape(this.getStart(), this.getGoal());
			break;
		case GAUSSIAN:
		case UNIFORM:
		default:
			this.getPlanningContinuum().setBoxShape();
		}
	}
	
	/**
	 * Disposes a sampling shape in the environment of this RRT planner.
	 */
	protected void disposeSamplingShape() {
		this.getPlanningContinuum().setSamplingShape(null);
	}
	
	/**
	 * Attempts to connect the tree to a given waypoint by repeatedly extending
	 * towards it until it has been reached or the tree branch is trapped.
	 * 
	 * @param waypoint the waypoint to connect the tree to
	 * 
	 * @return the status of the last extension towards the waypoint
	 */
	protected Status connectRRT(RRTreeWaypoint waypoint) {
		Status status = Status.ADVANCED;
		
		// attempt to connect new sample to tree while not trapped
		while ((Status.ADVANCED == status) && !(this.isInGoalRegion())) {
			status = this.extendRRT(waypoint);
		}
		
		return status;
	}
	
	/**
	 * Extends the tree in the direction of a given waypoint.
	 * 
	 * @param waypoint the waypoint to extend the tree towards
	 * 
	 * @return the status of the extension towards the waypoint
	 */
	protected Status extendRRT(RRTreeWaypoint waypoint) {
		Status status = Status.TRAPPED;
		
		// find the waypoint in the tree nearest to the sampled one
		RRTreeWaypoint nearestWaypoint = (RRTreeWaypoint)
				this.getPlanningContinuum().findNearest(waypoint, 1)
				.iterator().next();
		
		// TODO: consider performance metrics instead
		//this.getPlanningContinuum().findNearest(waypoint, 1, metric);
		
		// create a new edge in the tree by extending from nearest to sampled
		Optional<Edge> extension = this.createExtension(nearestWaypoint, waypoint);
		
		if (extension.isPresent()) {
			if (extension.get().getSecondPosition().equals(waypoint)) {
				status = Status.REACHED;
			} else {
				status = Status.ADVANCED;
			}
		}
		
		return status;
	}
	
	/**
	 * Creates an extension to the tree by growing from a connected tree
	 * waypoint towards a non-connected sampled waypoint observing capability
	 * constraints if applicable.
	 * 
	 * @param treeWaypoint the tree waypoint to be extended
	 * @param sampledWaypoint the sampled waypoint to be reached
	 * 
	 * @return the extension if it could be created
	 */
	protected Optional<Edge> createExtension(
			RRTreeWaypoint treeWaypoint, RRTreeWaypoint sampledWaypoint) {
		Optional<Edge> extension = Optional.empty();
		
		// extend tree according to extension technique
		Position position = null;
		switch (this.getExtension()) {
		case FEASIBLE:
			position = this.growFeasiblePosition(treeWaypoint, sampledWaypoint);
			break;
		case LINEAR:
		default:
			position = this.growPosition(treeWaypoint, sampledWaypoint);
		}
		
		RRTreeWaypoint endWaypoint = this.createWaypoint(position);
		
		// enforce tree structure by avoiding cycles
		if (this.getPlanningContinuum().findVertex(endWaypoint).isEmpty()) {
			try {
				// check extension feasibility according to aircraft capabilities
				this.computeEto(treeWaypoint, endWaypoint);
				
				// TODO: integrate NASA terrain and jBullet for conflict checks
				// TODO: review if dedicated terrain (man-made) obstacles are required
				// TODO: consider safe altitude and distance (except take-off and landing?)
				// TODO: consider waypoint actions (take-off, land, hold) and delays
				// check extension and edge conflict with environment (terrain)
				if (this.getStart().equals(treeWaypoint)
						|| this.isInGoalRegion(endWaypoint)
						|| !(this.getEnvironment().collidesTerrain(
								treeWaypoint, endWaypoint))) {
					treeWaypoint.addChild(endWaypoint);
					endWaypoint.setParent(treeWaypoint);
					DirectedEdge edge = new DirectedEdge(
							this.getEnvironment(), treeWaypoint, endWaypoint);
					this.getPlanningContinuum().addEdge(edge);
					extension = Optional.of(edge);
					this.computeCost(treeWaypoint, endWaypoint);
					this.setNewestWaypoint(endWaypoint);
				}
			} catch (CapabilitiesException ce) {
				Logging.logger().info(ce.getMessage());
			}
		}
		
		return extension;
	}
	
	/**
	 * Grows the tree by one position from a source within the tree towards a
	 * destination position applying a linear extension technique.
	 * 
	 * @param source the source position from which the tree is grown
	 * @param destination the goal position to which the tree is grown
	 * 
	 * @return the extension position
	 */
	protected synchronized Position growPosition(Position source, Position destination) {
		Position position = null;
		
		Vec4 point1 = this.getEnvironment().getGlobe().computePointFromPosition(source);
		Vec4 point2 = this.getEnvironment().getGlobe().computePointFromPosition(destination);
		
		double distance = point1.distanceTo3(point2);
		
		if (distance <= this.getEpsilon()) {
			// distance is within maximum distance epsilon
			position = destination;
		} else {
			// shorten distance to maximum distance epsilon
			Vec4 dp = point2.subtract3(point1);
			Vec4 edp = dp.normalize3().multiply3(this.getEpsilon());
			Vec4 point = point1.add3(edp);
			position = super.getEnvironment().getGlobe().computePositionFromPoint(point);
		}

		return position;
	}
	
	/**
	 * Grows the tree by one position from a source within the tree towards a
	 * destination position applying a feasible extension technique considering
	 * aircraft capabilities.
	 * 
	 * @param source the source position from which the tree is grown
	 * @param destination the goal position to which the tree is grown
	 * 
	 * @return the extension position
	 */
	protected synchronized Position growFeasiblePosition(Position source, Position destination) {
		Position position = null;
		
		Aircraft aircraft = this.getAircraft();
		Globe globe = this.getEnvironment().getGlobe();
		
		Vec4 point1 = globe.computePointFromPosition(source);
		Vec4 point2 = globe.computePointFromPosition(destination);
		Vec4 normal = globe.computeSurfaceNormalAtPoint(point1);
		Vec4 climb = point2.subtract3(point1);
		double distance = climb.getLength3();
		
		// climb or descent angle
		Angle angle = Angle.POS90.subtract(normal.angleBetween3(climb));
		
		// climb angle at maximum rate of climb (not maximum angle)
		Angle maxClimbAngle = Angle.fromRadians(Math.asin(
				aircraft.getCapabilities().getMaximumRateOfClimb() /
				aircraft.getCapabilities().getMaximumRateOfClimbSpeed()))
				.multiply(1).subtractDegrees(1d);
		
		// descent angle at maximum rate of descent (not maximum angle)
		Angle maxDescentAngle = Angle.fromRadians(Math.asin(
				aircraft.getCapabilities().getMaximumRateOfDescent() /
				aircraft.getCapabilities().getMaximumRateOfDescentSpeed()))
				.multiply(-1).addDegrees(1d);
		
		// restrict climb or descent angle according to aircraft capabilities
		if ((0 < angle.compareTo(maxClimbAngle)) ||
				(0 > angle.compareTo(maxDescentAngle))) {
			// rotate climb or descent vector to perform shuttle climb or descent
			Angle rotationAngle = Angle.POS90.subtract(maxClimbAngle);
			Plane rotationPlane = Plane.fromPoints(point1, point2, point1.add3(normal));
			Vec4 rotationAxis = rotationPlane.getNormal();
			Quaternion rotation = Quaternion.fromAxisAngle(rotationAngle, rotationAxis);
			distance = (this.getEpsilon() < distance) ? this.getEpsilon() : distance;
			climb = normal.transformBy3(rotation).normalize3().multiply3(distance);
			position = globe.computePositionFromPoint(point1.add3(climb));
		} else {
			// climb or descent angle is within limits
			position = this.growPosition(source, destination);
		}
		
		return position;
	}
	
	/**
	 * Determines whether or not the newest sampled waypoint is located within
	 * the goal region.
	 * 
	 * @return true if the new newest sampled waypoint is located within the
	 *         goal region, false otherwise
	 */
	protected boolean isInGoalRegion() {
		return this.isInGoalRegion(this.getNewestWaypoint());
	}
	
	/**
	 * Determines whether or not a position is located within the goal region.
	 * 
	 * @param position the position in global coordinates
	 * 
	 * @return true if the position is located within the goal region,
	 *         false otherwise
	 */
	protected boolean isInGoalRegion(Position position) {
		return this.getGoalThreshold() >= this.getEnvironment()
				.getDistance(position, this.getGoal());
	}
	
	/**
	 * Initializes this RRT planner to plan from an origin to a destination at
	 * a specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 */
	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {
		this.clearExpendables();
		
		this.setStart(this.createWaypoint(origin));
		this.getStart().setEto(etd);
		this.getStart().setCost(0d);
		this.getStart().setPoi(true);
		this.getPlanningContinuum().addVertex(this.getStart());
		this.setNewestWaypoint(this.getStart());
		
		this.setGoal(this.createWaypoint(destination));
		this.getGoal().setPoi(true);
	}
	
	/**
	 * Computes a RRT plan by growing a tree until the goal region is reached.
	 * 
	 * @return true if the goal region was reached, false otherwise
	 */
	protected boolean compute() {
		if (this.getStart().equals(this.getGoal())) {
			this.connectPlan(this.getStart());
			return true;
		}
		
		this.createSamplingShape();
		
		int iteration = 0;
		while ((!this.isInGoalRegion()) && (this.getMaxIterations() > iteration)) {
			// sample a new waypoint
			RRTreeWaypoint sample = this.sampleBiased();
			
			// connect to or extend towards sample according to strategy
			Status status = Status.TRAPPED;
			switch (this.getStrategy()) {
			case CONNECT:
				status = this.connectRRT(sample);
				break;
			case EXTEND:
			default:
				status = this.extendRRT(sample);
			}
			
			// check goal region
			if ((Status.TRAPPED != status) && this.isInGoalRegion()) {
				if (this.getNewestWaypoint().equals(this.getGoal())) {
					this.setGoal(this.getNewestWaypoint());
				} else {
					// TODO: possible feasibility / capability issues
					this.createExtension(this.getNewestWaypoint(), this.getGoal());
				}
				this.connectPlan();
			}
		
			iteration++;
		}
		
		this.disposeSamplingShape();
		
		if (!this.isInGoalRegion()) {
			Logging.logger().info("no trajectory found after " + this.getMaxIterations() + " iterations");
		}
		
		return this.isInGoalRegion();
	}
	
	/**
	 * Plans a part of a trajectory.
	 * 
	 * @param partIndex the index of the part
	 * 
	 * @return the planned part of a trajectory
	 */
	protected Trajectory planPart(int partIndex) {
		this.compute();
		return this.createTrajectory();
	}
	
	/**
	 * Plans a trajectory from an origin to a destination at a specified
	 * estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @return the planned trajectory from the origin to the destination with
	 *         the estimated time of departure
	 * 
	 * @see Planner#plan(Position, Position, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		this.initialize(origin, destination, etd);
		Trajectory trajectory = this.planPart(0);
		this.revisePlan(trajectory);
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
	 * @return the planned trajectory from the the origin to the destination
	 *         along the waypoints with the estimated time of departure
	 * 
	 * @see Planner#plan(Position, Position, List, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
		RRTreeWaypoint currentOrigin = this.createWaypoint(origin);
		ZonedDateTime currentEtd = etd;
		
		// collect intermediate destinations
		ArrayList<RRTreeWaypoint> destinations = waypoints.stream()
				.map(w -> this.createWaypoint(w))
				.collect(Collectors.toCollection(ArrayList::new));
		destinations.add(this.createWaypoint(destination));
		
		// plan and concatenate partial trajectories
		for (int partIndex = 0; partIndex < destinations.size(); partIndex++) {
			RRTreeWaypoint currentDestination = destinations.get(partIndex);
			if (!(currentOrigin.equals(currentDestination))) {
				
				/* 
				 * Each part of a multi-part plan has to be computed completely
				 * in order to finalize the ETO of the goal waypoint which
				 * becomes the start waypoint of the next part and the basis
				 * for any subsequent plan revisions. A possible repair of one
				 * part requires the re-computation of all subsequent parts.
				 * This cost-greedy solution does not necessarily result in
				 * optimality with respect to the overall cost of the computed
				 * multi-part plan.
				 * 
				 * https://github.com/stephanheinemann/worldwind/issues/24
				 */
				
				// plan partial trajectory
				this.initialize(currentOrigin, currentDestination, currentEtd);
				
				// connect part to previous part via goal parent for trajectory
				// generation
				if (currentOrigin.hasParent()) {
					this.getStart().setParent(currentOrigin.getParent());
					this.getStart().setCost(currentOrigin.getCost());
				}
				this.planPart(partIndex);
				
				if (this.hasWaypoints()) {
					// revise growing trajectory for each part
					this.revisePlan(this.createTrajectory());
					currentOrigin = this.getGoal();
					currentEtd = currentOrigin.getEto();
				} else {
					// if no plan could be found, return an empty trajectory
					Trajectory empty = new Trajectory();
					this.revisePlan(empty);
					return empty;
				}
			}
		}
		
		return this.createTrajectory();
	}
	
	/**
	 * Determines whether or not this RRT planner supports a specified
	 * environment.
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
	
	/**
	 * Determines whether or not this RRT planner matches a specification.
	 * 
	 * @param specification the specification to be matched
	 * 
	 * @return true if the this RRT planner matches the specification,
	 *         false otherwise
	 * 
	 * @see AbstractPlanner#matches(Specification)
	 */
	@Override
	public synchronized boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = super.matches(specification);
		
		if (matches && (specification.getProperties() instanceof RRTreeProperties)) {
			RRTreeProperties properties = (RRTreeProperties) specification.getProperties();
			matches = (this.getBias() == properties.getBias())
					&& (this.getEpsilon() == properties.getEpsilon())
					&& (this.getExtension() == properties.getExtension())
					&& (this.getGoalThreshold() == properties.getGoalThreshold())
					&& (this.getMaxIterations() == properties.getMaxIterations())
					&& (this.getSampling() == properties.getSampling())
					&& (this.getStrategy() == properties.getStrategy());
		}
		
		return matches;
	}
	
	/**
	 * Updates this RRT planner according to a specification.
	 * 
	 * @param specification the specification to be used for the update
	 * 
	 * @return true if this RRT planner has been updated, false otherwise
	 * 
	 * @see AbstractPlanner#update(Specification)
	 */
	@Override
	public synchronized boolean update(Specification<? extends FactoryProduct> specification) {
		boolean updated = super.update(specification);
		
		if (updated && (specification.getProperties() instanceof RRTreeProperties)) {
			RRTreeProperties properties = (RRTreeProperties) specification.getProperties();
			this.setBias(properties.getBias());
			this.setEpsilon(properties.getEpsilon());
			this.setExtension(properties.getExtension());
			this.setGoalThreshold(properties.getGoalThreshold());
			this.setMaxIterations(properties.getMaxIterations());
			this.setSampling(properties.getSampling());
			this.setStrategy(properties.getStrategy());
		}
		
		return updated;
	}
	
}
