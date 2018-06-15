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
package com.cfar.swim.worldwind.ai.prm.fadprm;

import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.ai.AbstractPlanner;
import com.cfar.swim.worldwind.ai.AnytimePlanner;
import com.cfar.swim.worldwind.ai.Planner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.planning.DesirabilityZone;
import com.cfar.swim.worldwind.planning.Edge;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.SamplingEnvironment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.render.Obstacle;
import com.google.common.collect.Iterables;

import gov.nasa.worldwind.geom.Line;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;

/**
 * Realizes a Flexible Anytime Dynamic Probabilistic Roadmap planner (FADPRM)
 * that plans a trajectory of an aircraft in an environment considering a local
 * cost and risk policy.
 * 
 * @author Henrique Ferreira
 *
 */
public class FADPRMPlanner extends AbstractPlanner implements AnytimePlanner {

	private ArrayList<DesirabilityZone> desirabilityZones = new ArrayList<DesirabilityZone>();

	/**
	 * @return the desirabilityEnvironments
	 */
	public ArrayList<DesirabilityZone> getDesirabilityZones() {
		return desirabilityZones;
	}

	/**
	 * @param desirabilityEnvironments the desirabilityEnvironments to set
	 */
	public void setDesirabilityZones(ArrayList<DesirabilityZone> desirabilityZones) {
		this.desirabilityZones = desirabilityZones;
	}
	
	/** the maximum number of neighbors a waypoint can be connected to */
	private final int MAX_NEIGHBORS;

	/** the maximum distance between two neighboring waypoints */
	private final double MAX_DIST;

	/** the bias of the sampling algorithm towards goal */
	private final int BIAS;

	/** the priority queue of expandable FADPRM waypoints */
	protected PriorityQueue<FADPRMWaypoint> open = new PriorityQueue<>();

	/** the set of expanded FADPRM waypoints */
	protected Set<FADPRMWaypoint> closed = new HashSet<>();

	/** the start FADPRM waypoint */
	private FADPRMWaypoint start = null;

	/** the goal FADPRM waypoint */
	private FADPRMWaypoint goal = null;

	/** the last computed plan */
	private final LinkedList<FADPRMWaypoint> plan = new LinkedList<>();

	/** the initial beta factor used to sort the open list */
	private double initialBeta;

	/** the final beta factor used to sort the open list */
	private double finalBeta;

	/** the current beta factor used to sort the open list */
	private double beta;

	/** the increase amount to be applied to the current beta */
	private double stepBeta;

	 /** the number of times the planner was called */
	 private ArrayList<Integer> counters = new ArrayList<>();
	
	 /** the cost of the best path for each iteration of the algorithm */
	 private ArrayList<ArrayList<Double>> pathCosts = new ArrayList<>();

	/** the number of times the planner was called */
	private int counter = 0;

	/** the cost of the best path for each iteration of the algorithm */
	private ArrayList<Double> pathCost = new ArrayList<>();

	/**
	 * Constructs a FADPRM planner for a specified aircraft and environment using
	 * default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see AbstractPlanner#AbstractPlanner(Aircraft, Environment)
	 */
	public FADPRMPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		MAX_NEIGHBORS = 15;
		MAX_DIST = 400d;
		BIAS = 5;
	}

	/**
	 * Constructs a FAD PRM planner for a specified aircraft and environment, using
	 * default local cost and risk policies. Also, this planner is constructed
	 * considering a maximum number of neighbors (of a single Waypoint) and a
	 * maximum distance between two connected neighbors.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * @param maxNeighbors the maximum number of neighbors
	 * @param maxDist the maximum distance between two connected neighbors
	 * 
	 * @see AbstractPlanner#AbstractPlanner(Aircraft, Environment)
	 */
	public FADPRMPlanner(Aircraft aircraft, Environment environment, int maxNeighbors, double maxDist, int bias) {
		super(aircraft, environment);
		MAX_NEIGHBORS = maxNeighbors;
		MAX_DIST = maxDist;
		BIAS = bias;
	}

	/**
	 * Gets the sampling environment of this planner
	 * 
	 * @return the sampling environment
	 */
	public SamplingEnvironment getEnvironment() {
		return (SamplingEnvironment) super.getEnvironment();
	}

	/**
	 * Gets the list of already sampled waypoints
	 * 
	 * @return the list of waypoints
	 */
	@SuppressWarnings("unchecked")
	public List<FADPRMWaypoint> getWaypointList() {
		return (List<FADPRMWaypoint>) this.getEnvironment().getWaypointList();
	}

	/**
	 * Sets the list of waypoints previously sampled
	 * 
	 * @param waypointList the list of waypoints to set
	 * 
	 */
	@SuppressWarnings("unchecked")
	public void setWaypointList(List<? extends Waypoint> waypointList) {
		this.getEnvironment().setWaypointList((List<Waypoint>) waypointList);
	}

	/**
	 * Gets the list of already sampled edges
	 * 
	 * @return the list of edges
	 */
	public List<Edge> getEdgeList() {
		return this.getEnvironment().getEdgeList();
	}

	/**
	 * Sets the list of edges previously sampled
	 * 
	 * @param edgetList the list of edges to set
	 * 
	 */
	public void setEdgeList(List<Edge> edgeList) {
		this.getEnvironment().setEdgeList(edgeList);
	}

	/**
	 * Gets the start FADPRM waypoint of this FADPRM planner.
	 * 
	 * @return the start FADPRM waypoint of this FADPRM planner
	 */
	protected FADPRMWaypoint getStart() {
		return this.start;
	}

	/**
	 * Sets the start FADPRM waypoint of this FADPRM planner.
	 * 
	 * @param start the start waypoint of this FADPRM planner
	 */
	protected void setStart(FADPRMWaypoint start) {
		this.start = start;
	}

	/**
	 * Gets the goal FADPRM waypoint of this FADPRM planner.
	 * 
	 * @return the goal FADPRM waypoint of this FADPRM planner
	 */
	protected FADPRMWaypoint getGoal() {
		return this.goal;
	}

	/**
	 * Sets the goal FADPRM waypoint of this FADPRM planner.
	 * 
	 * @param goal the goal FADPRM waypoint of this FADPRM planner
	 */
	protected void setGoal(FADPRMWaypoint goal) {
		this.goal = goal;
	}

	/**
	 * Adds an FADPRM waypoint to the expandable FADPRM waypoints.
	 * 
	 * @param waypoint the FADPRM waypoint
	 * 
	 * @return true if the FADPRM waypoint has been added to the expandable FADPRM
	 *         waypoints, false otherwise
	 */
	protected boolean addExpandable(FADPRMWaypoint waypoint) {
		return this.open.add(waypoint);
	}

	/**
	 * Removes an FADPRM waypoint from the expandable FADPRM waypoints.
	 * 
	 * @param waypoint the FADPRM waypoint
	 * 
	 * @return true if the FADPRM waypoint has been removed from the expandable
	 *         FADPRM waypoints, false otherwise
	 */
	protected boolean removeExpandable(FADPRMWaypoint waypoint) {
		return this.open.remove(waypoint);
	}

	/**
	 * Clears the expandable FADPRM waypoints.
	 */
	protected void clearExpandables() {
		this.open.clear();
	}

	/**
	 * Determines whether or not an FADPRM waypoint is expandable.
	 * 
	 * @param waypoint the FADPRM waypoint
	 * 
	 * @return true if the FADPRM waypoint is expandable, false otherwise
	 */
	protected boolean isExpandable(FADPRMWaypoint waypoint) {
		return this.open.contains(waypoint);
	}

	/**
	 * Determines whether or not FADPRM waypoints can be expanded.
	 * 
	 * @return true if FADPRM waypoints can be expanded, false otherwise
	 */
	protected boolean canExpand() {
		return !(this.open.isEmpty());
	}

	/**
	 * Polls an FADPRM waypoint from the expandable FADPRM waypoints.
	 * 
	 * @return the polled FADPRM waypoint from the expandable FADPRM waypoints if
	 *         any, null otherwise
	 */
	protected FADPRMWaypoint pollExpandable() {
		return this.open.poll();
	}

	/**
	 * Adds an FADPRM waypoint to the expanded FADPRM waypoints.
	 * 
	 * @param waypoint the FADPRM waypoint
	 * 
	 * @return true if the FADPRM waypoint has been added to the expanded FADPRM
	 *         waypoints, false otherwise
	 */
	protected boolean addExpanded(FADPRMWaypoint waypoint) {
		return this.closed.add(waypoint);
	}

	/**
	 * Clears the expanded FADPRM waypoints.
	 */
	protected void clearExpanded() {
		this.closed.clear();
	}

	/**
	 * Determines whether or not an FADPRM waypoint has been expanded.
	 * 
	 * @param waypoint the FADPRM waypoint
	 * 
	 * @return true if the FADPRM waypoint has been expanded, false otherwise
	 */
	protected boolean isExpanded(FADPRMWaypoint waypoint) {
		return this.closed.contains(waypoint);
	}

	/**
	 * Gets the first FADPRM waypoint of the current plan.
	 * 
	 * @return the first FADPRM waypoint of the current plan
	 */
	protected FADPRMWaypoint getFirstWaypoint() {
		return this.plan.getFirst();
	}

	/**
	 * Adds a first FADPRM waypoint to the current plan.
	 * 
	 * @param waypoint the first FADPRM waypoint to be added to the current plan
	 */
	protected void addFirstWaypoint(FADPRMWaypoint waypoint) {
		this.plan.addFirst(waypoint);
	}

	/**
	 * Clears the waypoints of the current plan.
	 */
	protected void clearWaypoints() {
		this.plan.clear();
	}

	/**
	 * Gets the minimum quality (initial beta) of this FADPRM planner.
	 * 
	 * @return the minimum quality (initial beta) of this FADPRM planner
	 * 
	 * @see AnytimePlanner#getMinimumQuality()
	 */
	@Override
	public double getMinimumQuality() {
		return this.initialBeta;
	}

	/**
	 * Sets the minimum quality (initial beta) of this FADPRM planner.
	 * 
	 * @param initialBeta the minimum quality (initial beta) of this FADPRM planner
	 * 
	 * @throws IllegalArgumentException if the initial beta is invalid
	 * 
	 * @see AnytimePlanner#setMinimumQuality(double)
	 */
	@Override
	public void setMinimumQuality(double initialBeta) {
		if ((0d <= initialBeta) && (initialBeta <= this.finalBeta)) {
			this.initialBeta = initialBeta;
		} else {
			throw new IllegalArgumentException("initial inflation is invalid");
		}
	}

	/**
	 * Gets the maximum quality (final beta) of this FADPRM planner.
	 * 
	 * @return the maximum quality (final beta) of this FADPRM planner
	 * 
	 * @see AnytimePlanner#getMaximumQuality()
	 */
	@Override
	public double getMaximumQuality() {
		return this.finalBeta;
	}

	/**
	 * Sets the maximum quality (final beta) of this FADPRM planner.
	 * 
	 * @param finalBeta the maximum quality (final beta) of this FADPRM planner
	 * 
	 * @throws IllegalArgumentException if the final inflation is invalid
	 * 
	 * @see AnytimePlanner#setMaximumQuality(double)
	 */
	@Override
	public void setMaximumQuality(double finalBeta) {
		if ((1d >= finalBeta) && (this.initialBeta <= finalBeta)) {
			this.finalBeta = finalBeta;
		} else {
			throw new IllegalArgumentException("final deflation is invalid");
		}
	}

	/**
	 * Gets the quality improvement (stepBeta) of this FADPRM planner.
	 * 
	 * @return the quality improvement (stepBeta) of this FADPRM planner
	 * 
	 * @see AnytimePlanner#getQualityImprovement()
	 */
	@Override
	public double getQualityImprovement() {
		return this.stepBeta;
	}

	/**
	 * Sets the quality improvement (stepBeta) of this FADPRM planner.
	 * 
	 * @param deflationAmount the quality improvement (stepBeta) of this FADPRM
	 *            planner
	 * 
	 * @throws IllegalArgumentException if the deflation amount is invalid
	 * 
	 * @see AnytimePlanner#setQualityImprovement(double)
	 */
	@Override
	public void setQualityImprovement(double stepBeta) {
		if (0d < stepBeta) {
			this.stepBeta = stepBeta;
		} else {
			throw new IllegalArgumentException("deflation amount is invalid");
		}
	}

	/**
	 * Increases the current beta by stepBeta, or otherwise to the final beta.
	 */
	protected void inflate() {
		if (this.finalBeta > (this.beta + this.stepBeta)) {
			this.beta += this.stepBeta;
		} else {
			this.beta = this.finalBeta;
		}
	}

	/**
	 * Determines whether or not the final beta has been reached.
	 * 
	 * @return true if the final beta has been reached, false otherwise
	 */
	protected boolean isInflated() {
		return (this.beta == this.finalBeta);
	}

	/**
	 * Gets the current beta of this FADPRM planner.
	 * 
	 * @return the current beta of this FADPRM planner
	 */
	protected double getInflation() {
		return this.beta;
	}

	/**
	 * Sets the current beta of this FADPRM planner.
	 * 
	 * @param beta the current beta of this FADPRM planner
	 */
	protected void setInflation(double beta) {
		this.beta = beta;
	}

	/**
	 * Gets the counter value of this FADPRM planner.
	 * 
	 * @param planIndex the index of the part of the multi-part plan
	 * 
	 * @return the counter the number of times the planner was called
	 */
	public int getCounter() {
		return counter;
	}

	/**
	 * Sets the counter value of this FADPRM planner.
	 * 
	 * @param counter the counter to set
	 */
	public void setCounter(int counter) {
		this.counter = counter;
	}

	/**
	 * Increments the counter value by 1.
	 */
	public void incrementCounter() {
		this.setCounter(this.getCounter() + 1);
	}
	/**
	 * Stores the path cost for a particular search.
	 */
	protected void addPathCost() {
		this.pathCost.add(this.getGoal().getCost());
	}

	/**
	 * Connects a plan of specified FADPRM waypoints.
	 * 
	 * @param waypoint the last FADPRM waypoint of a computed plan
	 */
	protected void connectPlan(FADPRMWaypoint waypoint) {
		this.clearWaypoints();
		// setting goal dtg and ttg
		waypoint.setDtg(0d);
		waypoint.setTtg(Duration.ZERO);

		while (!this.getStart().equals(waypoint)) {
//			System.out.println("connectinpath");
			this.addFirstWaypoint(waypoint.clone());
			if(waypoint.getPai()==null) {
				System.out.println("NULL");
			}
			waypoint = waypoint.getPai();
//			System.out.println("getting aprent");

			if (null != waypoint) {
				waypoint.setTtg(Duration.between(waypoint.getEto(), this.getFirstWaypoint().getEto()));
				waypoint.setDtg(this.getEnvironment().getDistance(waypoint, this.getFirstWaypoint()));
			}
		}
		this.addFirstWaypoint(waypoint.clone());
	}

	/**
	 * Samples a new waypoint from the environment with a given bias to the goal
	 * 
	 * @param bias the percentage value of bias to sample the goal
	 * 
	 * @return waypoint the FADPRMWaypoint sampled
	 */
	protected Position sampleBiased(int bias, FADPRMWaypoint waypoint) {
		Position position;
		int rand = new Random().nextInt(100 - 1) + 1;

		position = (rand <= bias) ? this.sampleGoal(waypoint) : this.samplePosition(waypoint);

		while (this.getEnvironment().checkConflict(position)
				|| !this.getAircraft().getCapabilities().isFeasible(waypoint, position,
						this.getEnvironment().getGlobe())
				|| !this.getEnvironment().contains(position)) {
			rand = new Random().nextInt(100 - 1) + 1;
			position = (rand <= bias) ? this.sampleGoal(waypoint) : this.samplePosition(waypoint);
		}

		return position;
	}

	/**
	 * Samples a new position in the direction of the goal, at a specified maximum
	 * distance away from the source position.
	 * 
	 * @param position the source position from which the tree is grown
	 * 
	 * @return the new position resulting from the controlled growth of the tree
	 */
	protected Position sampleGoal(Position position) {
		Position positionNew;
		Vec4 point1 = super.getEnvironment().getGlobe().computePointFromPosition(position);
		Vec4 point2 = super.getEnvironment().getGlobe().computePointFromPosition(this.getGoal());

		double dist = point1.distanceTo3(point2);

		if (dist < this.MAX_DIST) {
			positionNew = this.getGoal();
		} else {
			double x, y, z, dx, dy, dz, dist2, epsilon2, ddz;
			dx = point2.x - point1.x;
			dy = point2.y - point1.y;
			dz = point2.z - point1.z;

			ddz = dz / dist * this.MAX_DIST;
			epsilon2 = Math.sqrt(this.MAX_DIST * this.MAX_DIST - ddz * ddz);
			dist2 = point1.distanceTo2(point2);

			x = point1.x + dx / dist2 * epsilon2;
			y = point1.y + dy / dist2 * epsilon2;
			z = point1.z + ddz;

			Vec4 pointNew = new Vec4(x, y, z);
			positionNew = super.getEnvironment().getGlobe().computePositionFromPoint(pointNew);
		}
		return positionNew;
	}

	/**
	 * Samples a position from a continuous space around a given position
	 * 
	 * @param position the position around which the new position is going to be
	 *            sampled
	 * 
	 * @return the sampled position in globe coordinates
	 */

	public Position samplePosition(Position position) {
		Vec4 point = this.getEnvironment().getGlobe().computePointFromPosition(position);
		List<Vec4> corners = new ArrayList<Vec4>();
		double halfDistance = this.MAX_DIST / Math.sqrt(3);

		corners.add(point.add3(-halfDistance, +halfDistance, -halfDistance));
		corners.add(point.add3(+halfDistance, +halfDistance, -halfDistance));
		corners.add(point.add3(+halfDistance, -halfDistance, -halfDistance));
		corners.add(point.add3(+halfDistance, -halfDistance, +halfDistance));
		corners.add(point.add3(-halfDistance, -halfDistance, +halfDistance));
		corners.add(point.add3(-halfDistance, +halfDistance, +halfDistance));

		SamplingEnvironment env = new SamplingEnvironment(
				new Box(gov.nasa.worldwind.geom.Box.computeBoundingBox(corners)));
		env.setGlobe(this.getEnvironment().getGlobe());
		Position pos = env.sampleRandomPosition();
		return pos;
	}

	/**
	 * Indicates whether or not two waypoints are connectable.
	 * 
	 * @param waypoint the waypoint in globe coordinates
	 * @param neighbor the neighbor in globe coordinates
	 * @param num the number of connected neighbors to the waypoint
	 * 
	 * @return true if the two waypoints are connectable, false otherwise
	 */
	protected boolean areConnectable(Waypoint waypoint, Waypoint neighbor, int num) {
		boolean connectable = false;

		Capabilities capabilities = this.getAircraft().getCapabilities();

		if (super.getEnvironment().getDistance(neighbor, waypoint) < MAX_DIST && num < MAX_NEIGHBORS) {
			if (capabilities.isFeasible(waypoint, neighbor, this.getEnvironment().getGlobe())
					|| capabilities.isFeasible(neighbor, waypoint, this.getEnvironment().getGlobe())) {
				connectable = true;
			}
		}

		return connectable;
	}

	/**
	 * Expands an FADPRM waypoint towards its neighbors in the environment.
	 * 
	 * @param waypoint the FADPRM waypoint to be expanded
	 * 
	 * @return the new sampled waypoint
	 */
	protected FADPRMWaypoint expand(FADPRMWaypoint waypoint) {

		Position newPosition = this.sampleBiased(BIAS, waypoint);
		FADPRMWaypoint newWaypoint = this.createWaypoint(newPosition);

		this.createEdge(waypoint, newWaypoint);
		this.computeCost(waypoint, newWaypoint);
		if(newWaypoint.getPai()==null) {
			System.out.println("ESTAMLLL");
		}

		int numConnectedNeighbor = 1;
		this.getEnvironment().sortNearest(newWaypoint);

		for (FADPRMWaypoint neighbor : this.getWaypointList()) {
			if (neighbor.equals(newWaypoint))
				continue;
			if (this.areConnectable(neighbor, newWaypoint, numConnectedNeighbor)) {
				numConnectedNeighbor++;
				this.createEdge(neighbor, newWaypoint);
				neighbor.addSuccessor(newWaypoint);
				newWaypoint.addSuccessor(neighbor);
			}
		}
		double distance = this.getEnvironment().getNormalizedDistance(newWaypoint, this.getGoal());
		newWaypoint.setCTGoal(distance);
		newWaypoint.setH(newWaypoint.getPathDD() / (1 + newWaypoint.getLambda() * newWaypoint.getCTGoal()));
		waypoint.addSuccessor(newWaypoint);
		newWaypoint.addSuccessor(waypoint);

		return newWaypoint;
	}

	/**
	 * Creates a waypoint at a specified position, initializes its search value to
	 * 0, its beta to the current inflation and adds it to the waypoint list.
	 * 
	 * @param position the position in global coordinates
	 * @return the new created waypoint
	 */
	protected FADPRMWaypoint createWaypoint(Position position) {
		FADPRMWaypoint newWaypoint = new FADPRMWaypoint(position);
		newWaypoint.setEto(this.getEnvironment().getTime());
		newWaypoint.setSearch(0);
		newWaypoint.setBeta(this.getInflation());
		this.getWaypointList().add(newWaypoint);
		return newWaypoint;
	}

	/**
	 * Creates an edge between a source and a target waypoint, and adds it to the
	 * edge list
	 * 
	 * @param source the source FADPRM waypoint
	 * @param target the target FADPRM waypoint
	 */
	protected void createEdge(FADPRMWaypoint source, FADPRMWaypoint target) {
		Vec4 sourcePoint = this.getEnvironment().getGlobe().computePointFromPosition(source);
		Vec4 targetPoint = this.getEnvironment().getGlobe().computePointFromPosition(target);

		Edge edgeNew = new Edge(source, target, new Line(sourcePoint, targetPoint));
		edgeNew.setCostIntervals(this.getEnvironment().embedIntervalTree(edgeNew.getLine()));
		this.getEdgeList().add(edgeNew);
	}

	/**
	 * Computes the estimated cost of a specified target FADPRM waypoint when
	 * reached via a specified source FADPRM waypoint.
	 * 
	 * @param source the source FADPRM waypoint in globe coordinates
	 * @param target the target FADPRM waypoint in globe coordinates
	 */
	protected double computeCost(FADPRMWaypoint source, FADPRMWaypoint target) {

		Path leg = new Path(source, target);
		Capabilities capabilities = this.getAircraft().getCapabilities();
		Globe globe = this.getEnvironment().getGlobe();
		ZonedDateTime end = capabilities.getEstimatedTime(leg, globe, source.getEto());
/*
		double distance = this.getEnvironment().getNormalizedDistance(source, target);
		double pathDD = 1 / (2 * this.getEnvironment().getStepCost(source, target, source.getEto(), end,
				this.getCostPolicy(), this.getRiskPolicy()) / distance);*/
		double cost =this.getEnvironment().getStepCost(source, target, source.getEto(), end,
				this.getCostPolicy(), this.getRiskPolicy());
		double desirability =0;
		int i=0;
		for(DesirabilityZone zone : this.getDesirabilityZones()) {
			if(zone.intersects(new Line(this.getEnvironment().getGlobe().computePointFromPosition(source), this.getEnvironment().getGlobe().computePointFromPosition(target)))) {
				desirability = desirability + zone.getDesirability();
//				System.out.println("intersects ddzone");
				i++;
			}
		}
		if(i==0) {
			desirability=0.5;
		}
		else {
			desirability = desirability/i;
		}
		//pathDD = pathDD <= source.getPathDD() ? pathDD : source.getPathDD();
		double pathDD= desirability;
		if (pathDD / (1 + target.getLambda() * (source.getCost() + cost)) > target.getG()) {
			target.setPathDD(pathDD);
			target.setCost(source.getCost() + cost);
			target.setEto(end);
			target.setPai(source);
		}
		return (pathDD) / (1 + target.getLambda() * (source.getCost() + cost)) - source.getG();

	}

	/**
	 * Creates a trajectory of the computed plan.
	 * 
	 * @return the trajectory of the computed plan
	 */
	@SuppressWarnings("unchecked")
	protected Trajectory createTrajectory() {
		return new Trajectory((List<FADPRMWaypoint>) this.plan.clone());
	}

	/**
	 * Gets a set of obstacles that were either removed or added, since the last
	 * time the obstacles were checked.
	 * 
	 * @return the set of different obstacles
	 */
	@SuppressWarnings("unchecked")
	public HashSet<Obstacle> getNewObstacles() {
		HashSet<Obstacle> diffObstacles = new HashSet<Obstacle>();
		HashSet<Obstacle> beforeObstacles = (HashSet<Obstacle>) this.getEnvironment().getObstacles().clone();
		this.reviseObstacle();
		HashSet<Obstacle> afterObstacles = this.getEnvironment().getObstacles();
		for (Obstacle obstacle : beforeObstacles) {
			if (!afterObstacles.contains(obstacle)) {
				diffObstacles.add(obstacle);
			}
		}
		for (Obstacle obstacle : afterObstacles) {
			if (!diffObstacles.contains(obstacle)) {
				diffObstacles.add(obstacle);
			}
		}
		return diffObstacles;
	}

	/**
	 * Gets the affected edges by a set of obstacles and adds the cost interval to
	 * the edges' interval trees.
	 * 
	 * @param diffObstacles the set of different obstacles
	 * 
	 * @return the set of affected edges
	 */
	public Set<Edge> getAffectedEdges(HashSet<Obstacle> diffObstacles) {
		Set<Edge> affectedEdges = new HashSet<Edge>();
		for (Obstacle obstacle : diffObstacles) {
			for (Edge edge : this.getEnvironment().getEdgeList()) {
				if (obstacle.getExtent(this.getEnvironment().getGlobe()).intersects(edge.getLine())) {
					edge.setCostIntervals(this.getEnvironment().embedIntervalTree(edge.getLine()));
					affectedEdges.add(edge);
				}

			}
		}
		return affectedEdges;
	}

	/**
	 * Gets the affected waypoints contained on a set of affected edges.
	 * 
	 * @param affectedEdges the set of affected edges by a set of obstacles
	 * 
	 * @return the set of affected waypoints
	 */
	public Set<FADPRMWaypoint> getAffectedWaypoints(Set<Edge> affectedEdges) {
		Set<FADPRMWaypoint> conflictWaypoints = new HashSet<FADPRMWaypoint>();
		for (Edge edge : affectedEdges) {
			for (FADPRMWaypoint auxWaypoint : this.getWaypointList()) {
				if (auxWaypoint.equals(edge.getPosition1())) {
					conflictWaypoints.add(auxWaypoint);
				}
				if (auxWaypoint.equals(edge.getPosition2())) {
					conflictWaypoints.add(auxWaypoint);
				}
			}
		}
		return conflictWaypoints;
	}

	/**
	 * Updates the costs of all affected edges by the obstacles' changes.
	 * 
	 * @param diffObstacles the set of different obstacles
	 */
	public void updateEdges(HashSet<Obstacle> diffObstacles) {
		Set<Edge> affectedEdges = this.getAffectedEdges(diffObstacles);

		if (!affectedEdges.isEmpty()) {
			this.setInflation(0d);
			this.getGoal().setCost(0d);
			Set<FADPRMWaypoint> affectedWaypoints = this.getAffectedWaypoints(affectedEdges);
			this.correctListsW(affectedWaypoints);
			for (FADPRMWaypoint waypoint : this.getWaypointList()) {
				waypoint.getNeighbors().removeAll(affectedWaypoints);
				if (affectedWaypoints.contains(waypoint.getPai())) {
					waypoint.setCost(0d);
				}
			}
		}
		return;
	}

	/**
	 * Improves a plan incrementally.
	 */
	protected Trajectory improve(Position origin, Position destination, ZonedDateTime etd) {
		Trajectory trajectory = null;
		this.inflate();
		for (FADPRMWaypoint waypoint : this.getWaypointList()) {
			waypoint.setBeta(this.getInflation());
		}
		if (this.isInflated())
			System.out.println("max imprv");
		this.clearExpanded();
		this.clearExpandables();
		this.getStart().setBeta(this.getInflation());
		this.addExpandable(this.getStart());
		trajectory = this.compute(origin, destination, etd);
		this.revisePlan(trajectory);
		return trajectory;
	}

	/** indicates whether or not a multi-part plan is being improved */
	private boolean improving = false;

	/** the index of the backed up expandable waypoints */
	private int backupIndex = -1;

	/** the backed up expandable waypoints */
	private final ArrayList<ArrayList<FADPRMWaypoint>> backups = new ArrayList<>();

	/**
	 * Initializes a number of open backup priority queues.
	 * 
	 * @param size the number of open backup priority queues
	 */
	protected void initBackups(int size) {
		this.backups.clear();
		this.pathCosts.clear();
		this.counters.clear();
		
		for (int waypointIndex = 0; waypointIndex < size; waypointIndex++) {
			this.backups.add(waypointIndex, new ArrayList<FADPRMWaypoint>());
			this.pathCosts.add(waypointIndex, new ArrayList<Double>());
			this.pathCosts.get(waypointIndex).add(0d);
			this.counters.add(waypointIndex, 1);
		}
		this.backupIndex = -1;
	}

	/**
	 * Determines whether or not a backup can be performed.
	 * 
	 * @return true if a backup can be performed, false otherwise
	 */
	protected boolean canBackup() {
		return (-1 < this.backupIndex) && (this.backupIndex < this.backups.size());
	}

	/**
	 * Determines whether or not a backup is available.
	 * 
	 * @return true if a backup is available, false otherwise
	 */
	protected boolean hasBackup() {
		return this.canBackup() && (!this.backups.get(this.backupIndex).isEmpty());
	}

	/**
	 * Backs up the expandable FADPRM waypoints for improvement.
	 * 
	 * @return true if a backup has been performed, false otherwise
	 */
	protected boolean backup() {
		boolean backedup = false;
		if (this.canBackup()) {
			this.backups.get(this.backupIndex).clear();
			// TODO: make clones
			for(FADPRMWaypoint waypoint : this.getWaypointList()) {
				FADPRMWaypoint newWaypoint = waypoint.clone();
//				newWaypoint.setPai(waypoint.getPai());
				this.backups.get(this.backupIndex).add(newWaypoint);
			}

			for (FADPRMWaypoint wpt : this.backups.get(this.backupIndex)) {
				FADPRMWaypoint wptMirror = this.getWaypointList().stream().filter(s -> s.equals(wpt)).findFirst().get();
				FADPRMWaypoint parentMirror = wptMirror.getPai();
				FADPRMWaypoint parent;
				if(parentMirror ==null) {
					parent = null;
				}
				else {
					parent = this.backups.get(this.backupIndex).stream().filter(s -> s.equals(parentMirror)).findFirst().get();
				}
				wpt.setPai(parent);
			}

			this.pathCosts.get(this.backupIndex).addAll(this.pathCost);
			this.counters.set(this.backupIndex, this.counter);
			backedup = true;
			this.getWaypointList().clear();
		}
		return backedup;
	}

	/**
	 * Restores expandable FADPRM waypoints for improvement.
	 * 
	 * @return true if a restoral has been performed, false otherwise
	 */
	protected boolean restore() {
		boolean restored = false;
		if (this.hasBackup()) {
			for (FADPRMWaypoint waypoint : this.backups.get(this.backupIndex)) {
				waypoint.setBeta(this.getInflation());
			}
			this.setWaypointList(this.backups.get(this.backupIndex));
			restored = true;
		}
		this.pathCost = this.pathCosts.get(this.backupIndex);
		this.counter = this.counters.get(this.backupIndex);
		return restored;
	}

	/**
	 * Updates a given waypoint, improving its heuristic value and updating its
	 * counter value.
	 * 
	 * @param waypoint the waypoint to be updated
	 */
	protected void updateWaypoint(FADPRMWaypoint waypoint) {
		if (waypoint.getSearch() != 0 && waypoint.getSearch() != this.getCounter()) {
			if (waypoint.getCost() + waypoint.getCTGoal() < pathCost.get(waypoint.getSearch())) {
				waypoint.setCTGoal(pathCost.get(waypoint.getSearch()) - waypoint.getCost());
				waypoint.setH(waypoint.getPathDD() / (1 + waypoint.getLambda() * waypoint.getCTGoal()));
				waypoint.setCost(0);
			}
		} else if (waypoint.getSearch() == 0) {
			waypoint.setCost(0);
		}
		waypoint.setSearch(counter);
	}

	/**
	 * Updates all successors of a given waypoint.
	 * 
	 * @param source the predecessor of all waypoints to be updated
	 */
	public void updateSuccessors(FADPRMWaypoint source) {

		for (FADPRMWaypoint waypoint : source.getNeighbors()) {
			if (waypoint.equals(this.getGoal())) {
				continue;
			}
			if (!this.isExpanded(waypoint)) {
				this.updateWaypoint(waypoint);
				for (FADPRMWaypoint wptaux : waypoint.getNeighbors()) {
					if (wptaux.equals(this.getGoal()))
						continue;
					this.computeCost(wptaux, waypoint);
				}
				this.removeExpandable(waypoint);
				this.addExpandable(waypoint);
			}
		}
	}

	/**
	 * Updates the density of all waypoints in the waypoint list, by checking the
	 * number of waypoints that lie within a certain distance.
	 * 
	 * @param waypoint the new sampled waypoint
	 */
	@SuppressWarnings("unchecked")
	public void updateDensity(FADPRMWaypoint waypoint) {
		int density = 0;
		Set<FADPRMWaypoint> backupOpen = new HashSet<FADPRMWaypoint>();
		for (FADPRMWaypoint neighbor : (List<FADPRMWaypoint>) this.getEnvironment().getWaypointList()) {
			if (neighbor.equals(this.getGoal()))
				continue;
			if (this.getEnvironment().getNormalizedDistance(waypoint, neighbor) < MAX_DIST
					/ this.getEnvironment().getDiameter()) {
				density++;
				neighbor.incrementDensity();
			}
			if (this.open.contains(neighbor)) {
				backupOpen.add(neighbor);
				this.open.remove(neighbor);
			}
		}
		for (FADPRMWaypoint wpt : backupOpen) {
			this.open.add(wpt);
		}
		waypoint.setDensity(density);

		return;
	}

	/**
	 * Checks whether or not a given waypoint and the goal waypoint are connectable.
	 * 
	 * @param source the waypoint
	 * 
	 * @return true if the two waypoints are connectable, false otherwise
	 */
	public boolean connectToGoal(FADPRMWaypoint source) {
		if (this.areConnectable(source, this.getGoal(), 0)) {
			if (!this.getEdgeList().contains(new Edge(source, this.getGoal()))) {
				this.createEdge(source, this.getGoal());
			}
			return true;
		}
		return false;
	}

	/**
	 * Computes a path between a start and a goal waypoint.
	 */
	protected void computeOrImprovePath() {
		while (this.canExpand()) {
			FADPRMWaypoint source = this.pollExpandable();
			if (source.equals(this.getGoal())) {
				System.out.println("is goal");
				this.connectPlan(source);
				return;
			}
			FADPRMWaypoint newSuccessor;
			if (this.connectToGoal(source)) {
				System.out.println("found goal");
				this.computeCost(source, this.getGoal());
				newSuccessor = this.getGoal();
				this.removeExpandable(this.getGoal());
				this.getGoal().setBeta(this.getInflation());
				this.addExpandable(this.getGoal());
			} else {
//				System.out.println("expanding");
				newSuccessor = this.expand(source);
			}
			if (!newSuccessor.equals(this.getGoal())) {
				this.updateDensity(newSuccessor);
			}
			this.updateSuccessors(source);
			this.addExpanded(source);
		}
		return;
	}

	/**
	 * Corrects a trajectory, checking if any of its waypoints or edges is in
	 * conflict with terrain obstacles.
	 * 
	 * @param trajectory the planned trajectory
	 * 
	 * @return true if this trajectory is feasible, false otherwise
	 */
	protected boolean correctTrajectory(Trajectory trajectory) {
		if (trajectory == null)
			return false;

		HashSet<FADPRMWaypoint> conflictWaypoints = new HashSet<FADPRMWaypoint>();
		HashSet<Edge> conflictEdges = new HashSet<Edge>();

		for (Waypoint wpt : trajectory.getWaypoints()) {
			FADPRMWaypoint waypoint = (FADPRMWaypoint) wpt;
			if (this.getEnvironment().checkConflict(waypoint))
				conflictWaypoints.add(waypoint);
		}

		if (!conflictWaypoints.isEmpty()) {
			this.correctListsW(conflictWaypoints);
			return false;
		}
		Waypoint wpt1 = Iterables.get(trajectory.getWaypoints(), 0);
		Waypoint wpt2;
		for (int i = 0; i < trajectory.getLength() - 1; i++) {
			wpt2 = Iterables.get(trajectory.getWaypoints(), i + 1);
			if (this.getEnvironment().checkConflict(wpt1, wpt2)) {
				conflictEdges.add(new Edge(wpt1, wpt2));
			}
			wpt1 = wpt2;
		}
		if (!conflictEdges.isEmpty()) {
			this.correctListsE(conflictEdges);
			return false;
		}
		return true;
	}

	/**
	 * Corrects the waypoint and edge list by removing the waypoints that are in
	 * conflict with terrain obstacles.
	 * 
	 * @param conflictWaypoints the set of waypoints that are in conflict with
	 *            terrain obstacles
	 */
	protected void correctListsW(Set<FADPRMWaypoint> conflictWaypoints) {
		for (FADPRMWaypoint waypoint : conflictWaypoints) {
			this.getWaypointList().remove(waypoint);
			this.getEdgeList()
					.removeIf(s -> s.getPosition1().equals(waypoint) || s.getPosition2().equals(waypoint));
		}
	}

	/**
	 * Corrects the waypoint and edge list by removing the edges that are in
	 * conflict with terrain obstacles.
	 * 
	 * @param conflictEdges the set of edges that are in conflict with terrain
	 *            obstacles
	 */
	protected void correctListsE(HashSet<Edge> conflictEdges) {
		for (Edge edge : conflictEdges) {
			this.getEnvironment().getEdgeList().remove(edge);
		}
	}

	/**
	 * Computes a plan.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @return the planned trajectory from the origin to the destination with the
	 *         estimated time of departure
	 */
	protected Trajectory compute(Position origin, Position destination, ZonedDateTime etd) {
		Trajectory trajectory = null;

		while (!this.correctTrajectory(trajectory)) {
			this.computeOrImprovePath();
			System.out.println("trajectory");
			trajectory = this.createTrajectory();
			this.incrementCounter();
			this.addPathCost();
//			System.out.println("trajectory cost " + trajectory.getCost() + " " + trajectory.getDistance());
		}
		// for (FADPRMWaypoint waypoint : (Iterable<FADPRMWaypoint>)
		// trajectory.getWaypoints()) {
		// System.out.print(waypoint.getPathDD() + " ");
		// }
		return trajectory;
	}

	/**
	 * Initializes the planner to plan from an origin to a destination at a
	 * specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 */
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
		this.getStart().setEto(etd);
		// correct path dd
		this.getStart().setPathDD(0.5d);

		this.setGoal(this.createWaypoint(destination));
		// correct path dd
		this.getGoal().setPathDD(0.5);
		// small cheat to avoid goal density being 0
		this.getGoal().setDensity(1);
		
		this.updateWaypoint(this.getStart());
		this.updateWaypoint(this.getGoal());

		double distance = this.getEnvironment().getNormalizedDistance(this.getStart(), this.getGoal());
		this.getStart().setCTGoal(distance);
		this.getStart()
				.setH(this.getStart().getPathDD() / (1 + this.getStart().getLambda() * this.getStart().getCTGoal()));
		this.getGoal().setH(this.getGoal().getPathDD());


		this.addExpandable(this.getStart());
	}

	/**
	 * Updates the start position.
	 * 
	 * @return true if the start position has changed, false otherwise
	 */
	public boolean updateStart() {
		this.clearExpandables();
		FADPRMWaypoint oldStart = this.getStart();
		HashSet<FADPRMWaypoint> conflictWaypoints = new HashSet<FADPRMWaypoint>();
		conflictWaypoints.add(oldStart);
		this.correctListsW(conflictWaypoints);

		Set<FADPRMWaypoint> oldSuccessors = oldStart.getNeighbors();
		Set<FADPRMWaypoint> oldParents = new HashSet<FADPRMWaypoint>();

		for (FADPRMWaypoint waypoint : this.getWaypointList()) {
			if (waypoint.getPai() != null) {
				if (waypoint.getPai().equals(oldStart)) {
					oldParents.add(waypoint);
				}
			}
			waypoint.getNeighbors().remove(oldStart);
		}
		for (FADPRMWaypoint waypoint : this.getWaypointList()) {
			waypoint.setCost(0d);
			waypoint.setPai(null);
			if (waypoint.equals(this.plan.get(1))) {
				waypoint.setCost(0d);
				waypoint.setCost(waypoint.getPathDD());
				this.setStart(waypoint);
			}
		}
		for (FADPRMWaypoint succ : oldSuccessors) {
			if (!succ.equals(this.getStart()))
				this.getStart().addSuccessor(succ);
		}
		for (FADPRMWaypoint par : oldParents) {
			if (!par.equals(this.getStart())) {
				par.setCost(0d);
				this.createEdge(this.getStart(), par);
				this.computeCost(this.getStart(), par);
			}
		}
		for (FADPRMWaypoint wpt : this.getStart().getNeighbors()) {
			this.createEdge(this.getStart(), wpt);
			this.computeCost(this.getStart(), wpt);
		}
		this.getGoal().setCost(0d);
		this.getGoal().setPai(null);
		return this.getStart().equals(oldStart);

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
		System.out.println("desirability env "+this.getDesirabilityZones().size());
		this.initialize(origin, destination, etd);
		Trajectory trajectory = this.compute(origin, destination, etd);
		this.revisePlan(trajectory);
//		HashSet<Obstacle> diffObstacles = this.getNewObstacles();
		while (!this.isInflated()) {
			// while (!this.getStart().equals(this.getGoal())) {
			// if (!this.updateStart()) {
			// try {
			// Thread.sleep(2000);
			// } catch (InterruptedException e) {
			// e.printStackTrace();
			// }
			// }
//			this.updateEdges(diffObstacles);
			this.improve(origin, destination, etd);
//			diffObstacles.clear();
		}
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

	/**
	 * Plans a one time trajectory from an origin to a destination along waypoints
	 * at a specified estimated time of departure, considering the current beta
	 * value.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param waypoints the waypoints in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @return the planned trajectory from the origin to the destination along the
	 *         waypoints with the estimated time of departure
	 */
	public Trajectory oneTimePlan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
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
				Trajectory part = this.compute(currentOrigin, currentDestination, currentEtd);
				this.backup();
				// append partial trajectory to plan
				if ((!plan.isEmpty()) && (!part.isEmpty())) {
					plan.pollLast();
				}

				for (Waypoint waypoint : part.getWaypoints()) {
					plan.add(waypoint);
				}
				if (plan.peekLast().equals(currentOrigin)) {
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
		Trajectory trajectory = new Trajectory(plan);
		this.revisePlan(trajectory);
		System.out.println("trajectory cost " + trajectory.getCost() + " " + trajectory.getDistance());
		return trajectory;
	}

	/**
	 * Indicates whether or not this FADPRM planner supports a specified
	 * environment.
	 * 
	 * @param environment the environment
	 * 
	 * @return true if the environment is a planning continuum, false otherwise
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
}
