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

import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
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
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.planners.rrt.RRTreeProperties;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Plane;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Quaternion;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;
import gov.nasa.worldwind.util.Logging;

/**
 * Realizes a basic RRT planner that plans a trajectory of an aircraft in an
 * environment considering a local cost and risk policy. The origin of the tree
 * is the departure position and it is grown until the goal is reached.
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
	private int maxIterations = 3_000;
	
	/** the maximum extension distance to a waypoint in the tree of this RRT planner */
	private double epsilon = this.getEnvironment().getDiameter() / 20d;
	
	/** the sampling bias towards goal of this RRT planner */
	private int bias = 5;
	
	/** the radius of the sphere defining the goal region of this RRT planner */
	private double goalThreshold = 5d;
	
	/** the start RRT waypoint of this RRT planner */
	private RRTreeWaypoint start = null;
	
	/** the goal RRT waypoint of this RRT planner */
	private RRTreeWaypoint goal = null;
	
	/** the newest waypoint in the tree of this RRT planner */
	private RRTreeWaypoint newestWaypoint = null;
	
	/** the last computed plan of this RRT planner */
	private final LinkedList<Waypoint> plan = new LinkedList<>();
	
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
	 * Gets the planning continuum of this RRT planner.
	 * 
	 * @return the planning continuum of this RRT planner
	 * 
	 * @see AbstractPlanner#getEnvironment()
	 */
	@Override
	public PlanningContinuum getEnvironment() {
		return (PlanningContinuum) super.getEnvironment();
	}
	
	/**
	 * Creates a RRT waypoint at a specified position.
	 * 
	 * @param position the position
	 * 
	 * @return the RRT waypoint at the specified position
	 */
	protected RRTreeWaypoint createWaypoint(Position position) {
		return new RRTreeWaypoint(position);
	}
	
	/**
	 * Gets the start RRT waypoint of this RRT planner.
	 * 
	 * @return the start RRT waypoint of this RRT planner
	 */
	public RRTreeWaypoint getStart() {
		return this.start;
	}

	/**
	 * Sets the start RRT waypoint of this RRT planner.
	 * 
	 * @param start the start RRT waypoint of this RRT planner
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
		return this.goal;
	}

	/**
	 * Sets the goal RRT waypoint of this RRT planner.
	 * 
	 * @param goal the goal RRT waypoint of this RRT planner
	 */
	public void setGoal(RRTreeWaypoint goal) {
		this.goal = goal;
	}
	
	/**
	 * Gets the newest RRT waypoint added to the tree.
	 * 
	 * @return the newest RRT waypoint added to the tree
	 */
	public RRTreeWaypoint getNewestWaypoint() {
		return this.newestWaypoint;
	}
	
	/**
	 * Sets the newest RRT waypoint added to the tree.
	 * 
	 * @param newestWaypoint the newest RRT waypoint added to the tree
	 */
	public void setNewestWaypoint(RRTreeWaypoint newestWaypoint) {
		this.newestWaypoint = newestWaypoint;
	}
	
	/**
	 * Gets the sampling distribution of this RRT planner.
	 * 
	 * @return the sampling distribution of this RRT planner
	 */
	public Sampling getSampling() {
		return this.sampling;
	}
	
	/**
	 * Sets the sampling distribution of this RRT planner.
	 * 
	 * @param sampling the sampling distribution to be set
	 */
	public void setSampling(Sampling sampling) {
		this.sampling = sampling;
	}
	
	/**
	 * Gets the expansion strategy of this RRT planner.
	 * 
	 * @return the expansion strategy of this RRT planner
	 */
	public Strategy getStrategy() {
		return this.strategy;
	}
	
	/**
	 * Sets the expansion strategy of this RRT planner.
	 * 
	 * @param strategy the expansion strategy to be set
	 */
	public void setStrategy(Strategy strategy) {
		this.strategy = strategy;
	}
	
	/**
	 * Gets the extension technique of this RRT planner.
	 * 
	 * @return the extension technique of this RRT planner
	 */
	public Extension getExtension() {
		return this.extension;
	}
	
	/**
	 * Sets the extension technique of this RRT planner.
	 * 
	 * @param extension the extension technique to be set
	 */
	public void setExtension(Extension extension) {
		this.extension = extension;
	}
	
	/**
	 * Gets the maximum number of sampling iterations towards the goal of this
	 * RRT planner.
	 * 
	 * @return the maximum number of sampling iterations of this RRT planner
	 */
	public int getMaxIterations() {
		return this.maxIterations;
	}
	
	/**
	 * Sets the maximum number of sampling iterations towards the goal of this
	 * RRT planner.
	 * 
	 * @param maxIterations the maximum number of sampling iterations to be set
	 */
	public void setMaxIterations(int maxIterations) {
		this.maxIterations = maxIterations;
	}
	
	/**
	 * Gets the maximum extension distance to a waypoint in the tree of this
	 * RRT planner.
	 * 
	 * @return the maximum extension distance of this RRT planner
	 */
	public double getEpsilon() {
		return this.epsilon;
	}
	
	/**
	 * Sets the maximum extension distance to a waypoint in the tree of this
	 * RRT planner.
	 * 
	 * @param epsilon the maximum extension distance to be set
	 */
	public void setEpsilon(double epsilon) {
		this.epsilon = epsilon;
	}

	/**
	 * Gets the sampling bias towards the goal of this RRT planner.
	 * 
	 * @return the sampling bias towards the goal of this RRT planner
	 */
	public int getBias() {
		return this.bias;
	}
	
	/**
	 * Sets the sampling bias towards the goal of this RRT planner.
	 * 
	 * @param bias the sampling bias to be set
	 */
	public void setBias(int bias) {
		this.bias = bias;
	}
	
	/**
	 * Gets the the radius of the sphere defining the goal region of this RRT
	 * planner.
	 * 
	 * @return the radius of the sphere defining the goal region of this RRT
	 *         planner
	 */
	public double getGoalThreshold() {
		return this.goalThreshold;
	}

	/**
	 * Sets the radius of the sphere defining the goal region of this RRT
	 * planner.
	 * 
	 * @param goalThreshold the radius of the sphere defining the goal region
	 *                      to be set
	 */
	public void setGoalThreshold(double goalThreshold) {
		this.goalThreshold = goalThreshold;
	}
	
	/**
	 * Gets the current plan produced by this RRT planner
	 *
	 * @return the current plan produced by this RRT planner
	 */
	protected List<Waypoint> getPlan() {
		return this.plan;
	}
	
	/**
	 * Clears the planning data structures of this RRT planner.
	 */
	protected void clearExpendables() {
		this.getEnvironment().clearVertices();
		this.getPlan().clear();
	}
	
	/**
	 * Connects a plan from the newest sampled RRT waypoint backwards to the
	 * starting RRT waypoint.
	 */
	protected void connectPlan() {
		this.connectPlan(getNewestWaypoint());
	}
	
	/**
	 * Connects a plan from a RRT waypoint backwards to the starting RRT
	 * waypoint.
	 * 
	 * @param waypoint the RRT waypoint of a computed plan
	 */
	protected void connectPlan(RRTreeWaypoint waypoint) {
		this.plan.clear();
		waypoint.setTtg(Duration.ZERO);
		waypoint.setDtg(0d);
		this.plan.addFirst(waypoint.clone());
		while (waypoint.hasParent()) {
			waypoint = waypoint.getParent();
			waypoint.setTtg(Duration.between(waypoint.getEto(), plan.getFirst().getEto()));
			waypoint.setDtg(this.getEnvironment().getDistance(waypoint, plan.getFirst()));
			this.plan.addFirst(waypoint.clone());
		}
	}
	
	/**
	 * Creates a trajectory of the computed plan of this RRT planner.
	 * 
	 * @return the trajectory of the computed plan of this RRT planner
	 */
	@SuppressWarnings("unchecked")
	protected Trajectory createTrajectory() {
		return new Trajectory(
				Collections.unmodifiableList((List<Waypoint>) this.plan.clone()));
	}
	
	/**
	 * Creates a trajectory from a particular plan.
	 * 
	 * @param plan the plan from which the trajectory is created
	 * 
	 * @return the trajectory of the plan
	 */
	@SuppressWarnings("unchecked")
	protected Trajectory createTrajectory(LinkedList<Waypoint> plan) {
		return new Trajectory(
				Collections.unmodifiableList((List<Waypoint>) plan.clone()));
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
		return (random <= this.bias) ? this.getGoal() : this.sample();
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
				this.getEnvironment().sampleRandomUniformPosition());
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
				this.getEnvironment().sampleRandomGaussianPosition());
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
				this.getEnvironment().sampleRandomEllipsoidPosition(
						this.start, this.goal));
	}
	
	/**
	 * Creates a sampling shape in the environment of this RRT planner.
	 */
	protected void createSamplingShape() {
		switch(this.getSampling()) {
		case ELLIPSOIDAL:
			this.getEnvironment().setEllipsoidShape(this.getStart(), this.getGoal());
			break;
		case GAUSSIAN:
		case UNIFORM:
		default:
			this.getEnvironment().setBoxShape();
		}
	}
	
	/**
	 * Disposes a sampling shape in the environment of this RRT planner.
	 */
	protected void disposeSamplingShape() {
		this.getEnvironment().setSamplingShape(null);
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
		RRTreeWaypoint nearestWaypoint = (RRTreeWaypoint) this.getEnvironment()
				.findNearest(waypoint, 1).iterator().next();
		
		// TODO: consider performance metrics instead
		//this.getEnvironment().findNearest(waypoint, 1, metric);
		
		// create a new edge in the tree by extending from nearest to sampled
		Optional<Edge> extension = this.createExtension(nearestWaypoint, waypoint);
		
		if (extension.isPresent()) {
			if (extension.get().getSecondPosition().equals(waypoint)) {
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
		endWaypoint.setParent(treeWaypoint);
		
		try {
			// check extension feasibility according to aircraft capabilities
			this.computeEto(treeWaypoint, endWaypoint);
			
			// TODO: integrate NASA terrain and jBullet for conflict checks
			// TODO: review if dedicated terrain (man-made) obstacles are required
			// TODO: consider safe altitude and distance (except take-off and landing?)
			// TODO: consider waypoint actions (take-off, land, hold) and delays
			// check extension and edge conflict with environment (terrain)
			if (this.getStart().equals(treeWaypoint) || this.isInGoalRegion(endWaypoint) ||
					!(this.getEnvironment().collidesTerrain(treeWaypoint, endWaypoint))) {
				Edge edge = new Edge(this.getEnvironment(), treeWaypoint, endWaypoint);
				this.getEnvironment().addEdge(edge);
				extension = Optional.of(edge);
				this.computeCost(treeWaypoint, endWaypoint);
				this.setNewestWaypoint(endWaypoint);
			}
			
		} catch (IllegalArgumentException e) {
			e.printStackTrace();
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
	protected Position growPosition(Position source, Position destination) {
		Position position = null;
		
		Vec4 point1 = this.getEnvironment().getGlobe().computePointFromPosition(source);
		Vec4 point2 = this.getEnvironment().getGlobe().computePointFromPosition(destination);
		
		double distance = point1.distanceTo3(point2);
		
		if (distance <= this.epsilon) {
			// distance is within maximum distance epsilon
			position = destination;
		} else {
			// shorten distance to maximum distance epsilon
			Vec4 dp = point2.subtract3(point1);
			Vec4 edp = dp.normalize3().multiply3(this.epsilon);
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
	protected Position growFeasiblePosition(Position source, Position destination) {
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
			distance = (this.epsilon < distance) ? this.epsilon : distance;
			climb = normal.transformBy3(rotation).normalize3().multiply3(distance);
			position = globe.computePositionFromPoint(point1.add3(climb));
		} else {
			// climb or descent angle is within limits
			position = this.growPosition(source, destination);
		}
		
		return position;
	}
	
	/**
	 * Computes the estimated time over a target waypoint when traveled to from
	 * a source waypoint considering the applicable aircraft capabilities.
	 * 
	 * @param source the source waypoint with known ETO
	 * @param target the target waypoint with unknown ETO
	 */
	protected void computeEto(RRTreeWaypoint source, RRTreeWaypoint target) {
		Path leg = new Path(source, target);
		Capabilities capabilities = this.getAircraft().getCapabilities();
		Globe globe = this.getEnvironment().getGlobe();
		
		target.setEto(capabilities.getEstimatedTime(leg, globe, source.getEto()));
	}
	
	/**
	 * Computes the estimated cost of a target waypoint when traveled to from
	 * a source waypoint considering the operational cost and risk policies.
	 * 
	 * @param source the source waypoint with known cost
	 * @param target the target waypoint with unknown cost
	 */
	protected void computeCost(RRTreeWaypoint source, RRTreeWaypoint target) {
		double cost = source.getCost();
		cost += this.getEnvironment().getStepCost(
				source, target,
				source.getEto(), target.getEto(),
				this.getCostPolicy(), this.getRiskPolicy());
		target.setCost(cost);
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
		this.getEnvironment().addVertex(this.getStart());
		this.setNewestWaypoint(this.getStart());

		this.setGoal(this.createWaypoint(destination));
	}
	
	/**
	 * Computes a RRT plan by growing a tree until the goal region is reached.
	 * 
	 * @return true if the goal region was reached, false otherwise
	 */
	protected boolean compute() {
		this.createSamplingShape();
		
		for (int i = 0; i < this.maxIterations; i++) {
			// sample a new waypoint
			RRTreeWaypoint sample = this.sampleBiased();
			
			// connect to or extend towards sample according to strategy
			boolean notTrapped = false;
			switch (this.getStrategy()) {
			case CONNECT:
				notTrapped = (Status.TRAPPED != this.connectRRT(sample));
				break;
			case EXTEND:
			default:
				notTrapped = (Status.TRAPPED != this.extendRRT(sample));
				break;
			}
			
			if (notTrapped) {
				if (this.isInGoalRegion()) {
					this.setGoal(this.getNewestWaypoint());
					this.connectPlan();
					return true;
				}
			}
		}
		
		this.disposeSamplingShape();
		
		Logging.logger().info("no trajectory found after " + this.getMaxIterations() + " iterations");
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
	 * @see Planner#plan(Position, Position, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		// TODO: consecutive planning attempts may lead to empty trajectories (bug)
		this.initialize(origin, destination, etd);
		this.compute();
		Trajectory trajectory = this.createTrajectory();
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
	 * @return the planned trajectory from the origin to the destination along the
	 *         waypoints with the estimated time of departure
	 * 
	 * @see Planner#plan(Position, Position, List, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
		LinkedList<Waypoint> plan = new LinkedList<>();
		Waypoint currentOrigin = new Waypoint(origin);
		ZonedDateTime currentEtd = etd;
		
		// collect intermediate destinations
		ArrayList<Waypoint> destinations = waypoints.stream().map(Waypoint::new)
				.collect(Collectors.toCollection(ArrayList::new));
		destinations.add(new Waypoint(destination));
		
		// plan and concatenate partial trajectories
		for (Waypoint currentDestination : destinations) {
			if (!(currentOrigin.equals(currentDestination))) {
				// plan partial trajectory
				this.initialize(currentOrigin, currentDestination, currentEtd);
				this.compute();
				Trajectory part = this.createTrajectory();
				
				// append partial trajectory to plan
				if ((!plan.isEmpty()) && (!part.isEmpty())) {	
					// TODO: review (connection within goal region?)
					part = part.withoutFirst();
				}
				
				for (Waypoint waypoint : part.getWaypoints()) {
					plan.add(waypoint);
				}
				
				if (plan.isEmpty() || plan.peekLast().equals(currentOrigin)) {
					// if no plan could be found, return an empty trajectory
					Trajectory trajectory = new Trajectory();
					this.revisePlan(trajectory);
					return trajectory;
				} else {
					currentOrigin = plan.peekLast();
					currentEtd = currentOrigin.getEto();
				}
			}
		}
		
		Trajectory trajectory = this.createTrajectory(plan);
		this.revisePlan(trajectory);
		return trajectory;
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
	public boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = false;
		
		if ((null != specification) && (specification.getProperties() instanceof RRTreeProperties)) {
			RRTreeProperties rrtp = (RRTreeProperties) specification.getProperties();
			matches = (this.getCostPolicy().equals(rrtp.getCostPolicy()))
					&& (this.getRiskPolicy().equals(rrtp.getRiskPolicy()))
					&& (this.getBias() == rrtp.getBias())
					&& (this.getEpsilon() == rrtp.getEpsilon())
					&& (this.getExtension() == rrtp.getExtension())
					&& (this.getGoalThreshold() == rrtp.getGoalThreshold())
					&& (this.getMaxIterations() == rrtp.getMaxIterations())
					&& (this.getSampling() == rrtp.getSampling())
					&& (this.getStrategy() == rrtp.getStrategy())
					&& (specification.getId().equals(Specification.PLANNER_RRT_ID));
		}
		
		return matches;
	}
	
}
