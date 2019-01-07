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
package com.cfar.swim.worldwind.ai.prm.basicprm;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

import com.cfar.swim.worldwind.ai.AbstractPlanner;
import com.cfar.swim.worldwind.ai.AnytimePlanner;
import com.cfar.swim.worldwind.ai.PlanRevisionListener;
import com.cfar.swim.worldwind.ai.Planner;
import com.cfar.swim.worldwind.ai.astar.arastar.ARAStarPlanner;
import com.cfar.swim.worldwind.ai.astar.astar.ForwardAStarPlanner;
import com.cfar.swim.worldwind.ai.prm.rigidprm.QueryMode;
import com.cfar.swim.worldwind.ai.prm.rigidprm.QueryPlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.planning.Edge;
import com.cfar.swim.worldwind.planning.Environment;
import com.cfar.swim.worldwind.planning.SamplingEnvironment;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;

import gov.nasa.worldwind.geom.Line;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;

/**
 * Realizes a basic PRM planner that constructs a roadmap by sampling points in
 * a continuous environment and plans a trajectory of an aircraft in an
 * environment considering a local cost and risk policy. The path is found using
 * a specified A* based algorithm.
 * 
 * @author Henrique Ferreira
 *
 */
public class BasicPRM extends AbstractPlanner implements AnytimePlanner {

	/** the maximum number of sampling iterations */
	protected int maxIter;

	/** the maximum number of neighbors a waypoint can be connected to */
	protected int maxNeighbors;

	/** the maximum distance between two neighboring waypoints */
	protected double maxDistance;

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

	/**
	 * Constructs a basic PRM planner for a specified aircraft and environment using
	 * default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 * 
	 * @see AbstractPlanner#AbstractPlanner(Aircraft, Environment)
	 */
	public BasicPRM(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		maxIter = 1000;
		maxNeighbors = 30;
		maxDistance = 200d;
	}

	/**
	 * Gets the maximum number of sampling iterations.
	 * 
	 * @return the maxIter the maximum number of sampling iterations
	 */
	public int getMaxIter() {
		return maxIter;
	}

	/**
	 * Sets the maximum number of sampling iterations.
	 * 
	 * @param maxIter the maximum number of sampling iterations to set
	 */
	public void setMaxIter(int maxIter) {
		this.maxIter = maxIter;
	}

	/**
	 * Gets the maximum number of neighbors a waypoint can have.
	 * 
	 * @return the maxNeighbors the maximum number of neighbors a waypoint can have
	 */
	public int getMaxNeighbors() {
		return maxNeighbors;
	}

	/**
	 * Sets the maximum number of neighbors a waypoint can have.
	 * 
	 * @param maxNeighbors the maximum number of neighbors a waypoint can have
	 */
	public void setMaxNeighbors(int maxNeighbors) {
		this.maxNeighbors = maxNeighbors;
	}

	/**
	 * Gets the maximum distance between two connected waypoints.
	 * 
	 * @return the maxDistance the maximum distance between two connected waypoints
	 */
	public double getMaxDistance() {
		return maxDistance;
	}

	/**
	 * Sets the maximum distance between two connected waypoints.
	 * 
	 * @param maxDistance the maximum distance between two connected waypoints to
	 *            set
	 */
	public void setMaxDistance(double maxDistance) {
		this.maxDistance = maxDistance;
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
		if ((1d <= initialInflation) &&
				(initialInflation >= this.finalInflation)) {
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
	 * Gets the sampling environment of this planner.
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
	public List<Waypoint> getWaypointList() {
		return (List<Waypoint>) this.getEnvironment().getWaypointList();
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
	 * Creates a Waypoint at a specified position.
	 * 
	 * @param position the position
	 * 
	 * @return the Waypoint at the specified position
	 */
	protected Waypoint createWaypoint(Position position) {
		Waypoint newWaypoint = new Waypoint(position);
		newWaypoint.setEto(this.getEnvironment().getTime());
		return newWaypoint;
	}

	/**
	 * Connects a given waypoint to another waypoints already sampled, which are
	 * closer than a maximum distance. The maximum number of neighbors a waypoint
	 * can be connected to is another connection constraint. Checks if the two
	 * waypoints are connectable and if there is a conflict with terrain obstacles.
	 * 
	 * @param waypoint the waypoint to be connected
	 */
	protected void connectWaypoint(Waypoint waypoint) {
		int numConnectedNeighbor = 0;

		this.getEnvironment().sortNearest(waypoint);

		for (Waypoint neighbor : this.getWaypointList()) {
			if (this.areConnectable(waypoint, neighbor, numConnectedNeighbor)) {
				if (!this.getEnvironment().checkConflict(neighbor, waypoint, getAircraft())) {
					numConnectedNeighbor++;
					this.createEdge(waypoint, neighbor);
				}
			}
		}
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

		if (super.getEnvironment().getDistance(neighbor, waypoint) < maxDistance && num < maxNeighbors) {
			if (capabilities.isFeasible(waypoint, neighbor, this.getEnvironment().getGlobe())
					|| capabilities.isFeasible(neighbor, waypoint, this.getEnvironment().getGlobe())) {
				connectable = true;
			}
		}

		return connectable;
	}

	/**
	 * Creates an edge between a source waypoint and a target waypoint, and adds it
	 * to the edge list
	 * 
	 * @param source the source waypoint
	 * @param target the target waypoint
	 */
	protected void createEdge(Waypoint source, Waypoint target) {

		Vec4 sourcePoint = this.getEnvironment().getGlobe().computePointFromPosition(source);
		Vec4 targetPoint = this.getEnvironment().getGlobe().computePointFromPosition(target);

		if(sourcePoint.equals(targetPoint))
			return;
		
		Edge edgeNew = new Edge(source, target, Line.fromSegment(sourcePoint, targetPoint));
		edgeNew.setCostIntervals(this.getEnvironment().embedIntervalTree(edgeNew.getLine()));
		this.getEdgeList().add(edgeNew);
	}

	/**
	 * Initializes the planner clearing the waypoint and edge lists.
	 */
	protected void initialize() {
		this.getWaypointList().clear();
		this.getEdgeList().clear();
	}

	/**
	 * Creates the roadmap by sampling positions from a continuous environment.
	 * First, checks if the waypoint position has conflicts with terrain. Then the
	 * waypoint is added to the waypoint list. After that, tries to connect this
	 * waypoint to others already sampled.
	 */
	protected void construct() {
		int num = 0;

		while (num < maxIter) {
			Waypoint waypoint = this.createWaypoint(this.getEnvironment().sampleRandomPosition());

			if (!this.getEnvironment().checkConflict(waypoint, getAircraft())) {
				this.getWaypointList().add(waypoint);
				this.connectWaypoint(waypoint);
				num++;
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
		Waypoint start = this.createWaypoint(origin);
		Waypoint goal = this.createWaypoint(destination);

		if (!this.getEnvironment().checkConflict(start, getAircraft())) {
			this.getWaypointList().add(start);
			this.connectWaypoint(start);
		}
		else {
			System.out.println("start invalid");
		}

		if (!this.getEnvironment().checkConflict(goal, getAircraft())) {
			this.getWaypointList().add(goal);
			this.connectWaypoint(goal);
		}
		else {
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
		Waypoint start = this.createWaypoint(origin);
		Waypoint goal = this.createWaypoint(destination);

		if (!this.getEnvironment().checkConflict(start, getAircraft())) {
			this.getWaypointList().add(start);
			this.connectWaypoint(start);
		}

		if (!this.getEnvironment().checkConflict(goal, getAircraft())) {
			this.getWaypointList().add(goal);
			this.connectWaypoint(goal);
		}

		for (Position pos : waypoints) {
			Waypoint waypoint = this.createWaypoint(pos);

			if (!this.getEnvironment().checkConflict(waypoint, getAircraft())) {
				this.getWaypointList().add(waypoint);
				this.connectWaypoint(waypoint);
			}
			else {
				System.out.println("wp invalid");
			}
		}
	}

	/**
	 * Realizes the post construction step of a basic PRM planner. The main goal is
	 * to improve the roadmap connectivity and to uniformize the distribution of
	 * waypoints. It selects the waypoints with fewer neighbors and creates a new
	 * waypoint around each of them.
	 */
	@SuppressWarnings("deprecation")
	public void postConstruction() {
		//TODO : Review how many iterations to consider for post construction step (it is time consuming)
		int maxPostIter = maxIter / 6;
		HashMap<Waypoint, Integer> map = new HashMap<Waypoint, Integer>();
		TreeMap<Waypoint, Integer> waypoints = new TreeMap<Waypoint, Integer>(new ValueComparator(map));

		for (Waypoint waypoint : this.getWaypointList()) {
			int connectedEdges = (int) this.getEdgeList().stream().filter(s -> s.contains(waypoint)).count();
			map.put(waypoint, new Integer(connectedEdges));
		}
		
		waypoints.putAll(map);
		
	    Set<Waypoint> keys = waypoints.keySet();
	    @SuppressWarnings("rawtypes")
		Iterator i = keys.iterator();
	    for (int num = 0; num < maxPostIter; num++) {
	    	Waypoint key = (Waypoint) i.next();
	    	double halfDistance = this.maxDistance / (Math.sqrt(3));
			Waypoint newWaypoint = this.createWaypoint(this.samplePosition(key, halfDistance));
			while (this.getEnvironment().checkConflict(newWaypoint, getAircraft())) {
				halfDistance = halfDistance / 2;
				newWaypoint = this.createWaypoint(this.samplePosition(key, halfDistance));
			}
			this.getWaypointList().add(newWaypoint);
			this.connectWaypoint(newWaypoint);
	    }
	}

	class ValueComparator implements Comparator<Waypoint> {
	    Map<Waypoint, Integer> base;

	    public ValueComparator(Map<Waypoint, Integer> base) {
	        this.base = base;
	    }

	    // Note: this comparator imposes orderings that are inconsistent with
	    // equals.
	    public int compare(Waypoint a, Waypoint b) {
	        if (base.get(a) >= base.get(b)) {
	            return 1;
	        } else {
	            return -1;
	        } // returning 0 would merge keys
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
	 * listener invokes the listeners of this Basic PRM planner.
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
			break;
		case ARA:
			ARAStarPlanner araStar = new ARAStarPlanner(this.getAircraft(), this.getEnvironment());
			araStar.setCostPolicy(this.getCostPolicy());
			araStar.setRiskPolicy(this.getRiskPolicy());
			araStar.setMinimumQuality(this.getMinimumQuality());
			araStar.setMaximumQuality(this.getMaximumQuality());
			araStar.setQualityImprovement(this.getQualityImprovement());
			this.setRevisionListeners(araStar);
			trajectory = araStar.plan(origin, destination, etd);
			break;
		/*
		 * case AD: ADStarPlanner adStar = new ADStarPlanner(this.getAircraft(),
		 * this.getEnvironment()); adStar.setCostPolicy(this.getCostPolicy());
		 * adStar.setRiskPolicy(this.getRiskPolicy());
		 * adStar.addPlanRevisionListener(new PlanRevisionListener() {
		 * 
		 * @Override public void revisePlan(Trajectory trajectory) { for
		 * (PlanRevisionListener listener : planRevisionListeners) {
		 * listener.revisePlan(trajectory); } } public void reviseObstacle() { for
		 * (PlanRevisionListener listener : planRevisionListeners) {
		 * listener.reviseObstacle(); } } }); trajectory = adStar.plan(origin,
		 * destination, etd); break;
		 */
		}
		return trajectory;
	}

	/**
	 * Invokes a query planner to find a trajectory from an origin to a destination
	 * along waypoints at a specified estimated time of departure.
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

		switch (planner) {
		case FAS:
			ForwardAStarPlanner aStar = new ForwardAStarPlanner(this.getAircraft(), this.getEnvironment());
			aStar.setCostPolicy(this.getCostPolicy());
			aStar.setRiskPolicy(this.getRiskPolicy());
			this.setRevisionListeners(aStar);
			trajectory = aStar.plan(origin, destination, waypoints, etd);
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
			break;
		/*
		 * case AD: ADStarPlanner adStar = new ADStarPlanner(this.getAircraft(),
		 * this.getEnvironment()); adStar.setCostPolicy(this.getCostPolicy());
		 * adStar.setRiskPolicy(this.getRiskPolicy());
		 * adStar.addPlanRevisionListener(new PlanRevisionListener() {
		 * 
		 * @Override public void revisePlan(Trajectory trajectory) { for
		 * (PlanRevisionListener listener : planRevisionListeners) {
		 * listener.revisePlan(trajectory); } }
		 * 
		 * @Override public void reviseObstacle() { for (PlanRevisionListener listener :
		 * planRevisionListeners) { listener.reviseObstacle(); } } }); trajectory =
		 * adStar.plan(origin, destination, waypoints, etd); break;
		 */
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
			this.extendsConstruction(origin, destination);
			this.postConstruction();
		} else if (this.getMode() == QueryMode.MULTIPLE) {
			this.extendsConstruction(origin, destination);
		}

		Trajectory trajectory = this.findPath(origin, destination, etd, this.planner);
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
		if (this.getEnvironment().getEdgeList().isEmpty()) {
			this.setMode(QueryMode.SINGLE);
		}

		if (this.getMode() == QueryMode.SINGLE) {
			this.initialize();
			this.construct();
			this.extendsConstruction(origin, destination, waypoints);
			this.postConstruction();
		} else if (this.getMode() == QueryMode.MULTIPLE) {
			this.extendsConstruction(origin, destination, waypoints);
		}

		Trajectory trajectory = this.findPath(origin, destination, etd, waypoints, this.planner);
		return trajectory;
	}

	/**
	 * Indicates whether or not this Basic PRM planner supports a specified
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
	 * waypoints.
	 * 
	 * @param waypoints the waypoints
	 * 
	 * @return true if waypoints are contained in the planner's environment,
	 *         false otherwise
	 */
	@Override
	public boolean supports(List<Position> waypoints) {
		boolean supports = false;
		
		supports = super.supports(waypoints);
		
		for(Position waypoint : waypoints) {
			if(this.getEnvironment().checkConflict(waypoint, getAircraft()))
				supports = false;
		}
		
		return supports;
	}
}
