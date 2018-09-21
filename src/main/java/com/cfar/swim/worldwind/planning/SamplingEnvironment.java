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
package com.cfar.swim.worldwind.planning;

import java.awt.Color;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.time.chrono.ChronoZonedDateTime;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;

import com.binarydreamers.trees.Interval;
import com.binarydreamers.trees.IntervalTree;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.geom.ContinuumBox;
import com.cfar.swim.worldwind.geom.CoordinateTransformations;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.ObstacleColor;
import com.cfar.swim.worldwind.render.TerrainObstacle;
import com.cfar.swim.worldwind.render.ThresholdRenderable;
import com.cfar.swim.worldwind.render.TimedRenderable;

import gov.nasa.worldwind.geom.Line;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Sphere;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Polyline;
import gov.nasa.worldwind.util.measure.LengthMeasurer;

/**
 * Realizes a sampling environment that implements an environment and can be
 * used for sampled based motion planning.
 * 
 * @author Manuel Rosa
 * @author Henrique Ferreira
 *
 */
public class SamplingEnvironment extends ContinuumBox implements Environment {

	/** the globe of this sampling environment */
	private Globe globe = null;

	/** the cost interval tree encoding temporal costs */
	private IntervalTree<ChronoZonedDateTime<?>> costIntervals = new IntervalTree<ChronoZonedDateTime<?>>(
			CostInterval.comparator);

	/** the current time of this sampling environment */
	private ZonedDateTime time = ZonedDateTime.now(ZoneId.of("UTC"));

	/** the obstacles embedded into this sampling environment */
	private HashSet<Obstacle> obstacles = new HashSet<Obstacle>();

	/** the terrain obstacles embedded into this sampling environment */
	private HashSet<TerrainObstacle> terrainObstacles = new HashSet<TerrainObstacle>();

	/** the list of already sampled waypoints */
	private List<Waypoint> waypointList = new ArrayList<Waypoint>();

	/** the list of edges in this environment */
	private List<Edge> edgeList = new ArrayList<Edge>();

	/** the current accumulated active cost of this sampling environment */
	private double activeCost = 1d;

	/** the threshold cost of this sampling environment */
	private double thresholdCost = 0d;

	/** the resolution of this sampling environment */
	private double resolution = 1d;

	/**
	 * Constructs a sampling environment based on a geometric box.
	 * 
	 * @param box the geometric box
	 * 
	 * @see Box#Box(gov.nasa.worldwind.geom.Box)
	 */
	public SamplingEnvironment(Box box) {
		super(box);
		this.update();
	}

	/**
	 * Constructs a sampling environment based on a geometric box and a given
	 * resolution.
	 * 
	 * @param box the geometric box
	 * @param resolution the resolution of this sampling environment
	 * 
	 * @see Box#Box(gov.nasa.worldwind.geom.Box)
	 */
	public SamplingEnvironment(Box box, double resolution) {
		super(box);
		this.resolution = resolution;
		this.update();
	}

	/**
	 * Gets the threshold cost of this sampling environment.
	 * 
	 * @return the threshold cost of this sampling environment
	 * 
	 * @see ThresholdRenderable#setThreshold(double)
	 */
	@Override
	public double getThreshold() {
		return this.thresholdCost;
	}

	/**
	 * Sets the threshold cost of this sampling environment.
	 * 
	 * @param thresholdCost the threshold cost of this sampling environment
	 * 
	 * @see ThresholdRenderable#setThreshold(double)
	 */
	@Override
	public void setThreshold(double thresholdCost) {
		this.thresholdCost = thresholdCost;
		this.updateVisibility();
	}

	/**
	 * Sets the globe of this sampling environment.
	 * 
	 * @param globe the globe of this sampling environment
	 * 
	 * @see Environment#setGlobe(Globe)
	 */
	@Override
	public void setGlobe(Globe globe) {
		this.globe = globe;
	}

	/**
	 * Gets the globe of this sampling environment.
	 * 
	 * @return the globe of this sampling environment
	 * 
	 * @see Environment#getGlobe()
	 */
	@Override
	public Globe getGlobe() {
		return this.globe;
	}

	/**
	 * Adds a cost interval to this sampling environment.
	 * 
	 * @param costInterval the cost interval to be added
	 */
	public void addCostInterval(CostInterval costInterval) {
		this.costIntervals.add(costInterval);
		this.update();
	}

	/**
	 * Removes a cost interval from this sampling environment.
	 * 
	 * @param costInterval the cost interval to be removed
	 * 
	 */
	public void removeCostInterval(CostInterval costInterval) {
		this.costIntervals.remove(costInterval);
		this.update();
	}

	/**
	 * Gets all cost intervals that are active at a specified time instant.
	 * 
	 * @param time the time instant
	 * 
	 * @return all cost intervals that are active at the specified time instant
	 * 
	 */
	public List<Interval<ChronoZonedDateTime<?>>> getCostIntervals(ZonedDateTime time) {
		// return this.costIntervals.searchInterval(new CostInterval(null, this.time));
		// //BUG?
		return this.costIntervals.searchInterval(new CostInterval(null, time));
	}

	/**
	 * Gets all cost intervals that are active during a specified time interval.
	 * 
	 * @param start the start time of the time interval
	 * @param end the end time of the time interval
	 * 
	 * @return all cost intervals that are active during the specified time interval
	 * 
	 */
	public List<Interval<ChronoZonedDateTime<?>>> getCostIntervals(ZonedDateTime start, ZonedDateTime end) {
		return this.costIntervals.searchInterval(new CostInterval(null, start, end));
	}

	/**
	 * Gets the accumulated cost of this sampling environment at specified time
	 * instant.
	 * 
	 * @param time the time instant
	 * 
	 * @return the accumulated cost of this sampling environment at the specified
	 *         time instant
	 */
	public double getCost(ZonedDateTime time) {
		return this.getCost(time, time);
	}

	/**
	 * Gets the accumulated cost of this sampling environment within a specified
	 * time span.
	 * 
	 * @param start the start time of the time span
	 * @param end the end time of the time span
	 * 
	 * @return the accumulated cost of this sampling environment within the
	 *         specified time span
	 */
	public double getCost(ZonedDateTime start, ZonedDateTime end) {
		double cost = 1d; // simple cost of normalized distance

		Set<String> costIntervalIds = new HashSet<String>();
		// add all (weighted) cost of the cell
		List<Interval<ChronoZonedDateTime<?>>> intervals = this.getCostIntervals(start, end);
		for (Interval<ChronoZonedDateTime<?>> interval : intervals) {
			if (interval instanceof CostInterval) {
				CostInterval costInterval = (CostInterval) interval;

				// only add costs of different overlapping cost intervals
				if (!costIntervalIds.contains(costInterval.getId())) {
					costIntervalIds.add(costInterval.getId());

					// TODO: implement a proper weighted cost calculation normalized from 0 to 100
					// TODO: the weight is affected by severity (reporting method) and currency
					// (reporting time)

					if ((interval instanceof WeightedCostInterval)) {
						cost += ((WeightedCostInterval) interval).getWeightedCost();
					} else {
						cost += costInterval.getCost();
					}
				}
			}
		}

		return cost;
	}

	/**
	 * Gets the current time of this sampling environment.
	 * 
	 * @return the current time of this sampling environment
	 * 
	 * @see TimedRenderable#getTime()
	 */
	@Override
	public ZonedDateTime getTime() {
		return this.time;
	}

	/**
	 * Sets the current time of this sampling environment.
	 * 
	 * @param time the current time of this sampling environment
	 * 
	 * @see TimedRenderable#setTime(ZonedDateTime)
	 */
	@Override
	public void setTime(ZonedDateTime time) {
		this.time = time;
		this.update();
	}

	/**
	 * Gets the obstacles of this sampling environment.
	 * 
	 * @return the obstacles
	 */
	public HashSet<Obstacle> getObstacles() {
		return obstacles;
	}

	/**
	 * Sets the obstacles of this sampling environment.
	 * 
	 * @param obstacles the obstacles to set
	 */
	public void setObstacles(HashSet<Obstacle> obstacles) {
		this.obstacles = obstacles;
	}

	/**
	 * Gets the resolution of this sampling environment
	 * 
	 * @return the resolution of this sampling environment
	 */
	public double getResolution() {
		return resolution;
	}

	/**
	 * Sets the resolution of this sampling environment
	 * 
	 * @param resolution the resolution to set
	 */
	public void setResolution(double resolution) {
		this.resolution = resolution;
	}

	/**
	 * Gets the terrain obstacles of this sampling environment.
	 * 
	 * @return the terrain obstacles
	 */
	public HashSet<TerrainObstacle> getTerrainObstacles() {
		return terrainObstacles;
	}

	/**
	 * Sets the terrain obstacles of this sampling environment.
	 * 
	 * @param terrainObstacles the terrain obstacles to set
	 */
	public void setTerrainObstacles(HashSet<TerrainObstacle> terrainObstacles) {
		this.terrainObstacles = terrainObstacles;
	}

	/**
	 * Gets the list of already sampled waypoints
	 * 
	 * @return the waypointList the list of waypoints
	 */
	public List<? extends Waypoint> getWaypointList() {
		return waypointList;
	}

	/**
	 * Sets the list of already sampled waypoints.
	 * 
	 * @param waypointList the list of waypoints to set
	 */
	public void setWaypointList(List<Waypoint> waypointList) {
		this.waypointList = waypointList;
	}

	/**
	 * Gets the list of edges in this environment.
	 * 
	 * @return the edgeList the list of edges
	 */
	public List<? extends Edge> getEdgeList() {
		return edgeList;
	}

	/**
	 * Sets the list of edges in this environment.
	 * 
	 * @param edgeList the list of edges to set
	 */
	public void setEdgeList(List<Edge> edgeList) {
		this.edgeList = edgeList;
	}

	/**
	 * Updates the list of edges in this environment to be in accordance to the list
	 * of waypoints, i.e. removes edges that contain waypoints (at least 1) which
	 * are no longer in the waypoint list.
	 */
	protected void refreshEdgeList() {
		List<Edge> validEdges = new ArrayList<Edge>();

		for (Edge edge : this.getEdgeList()) {
			Waypoint waypoint1 = (Waypoint) edge.getPosition1();
			if (!this.getWaypointList().contains(waypoint1))
				continue;

			Waypoint waypoint2 = (Waypoint) edge.getPosition2();
			if (!this.getWaypointList().contains(waypoint2))
				continue;

			validEdges.add(edge);
		}

		this.setEdgeList(validEdges);
	}

	/**
	 * Gets the edge from the list of edges in this environment which has both given
	 * positions.
	 * 
	 * @param position1 one of the positions to be checked
	 * @param position2 the other position to be checked
	 * 
	 * @return the edge containing both positions, if present, null otherwise
	 */
	public Optional<Edge> getEdge(Position position1, Position position2) {

		return this.edgeList.stream().filter(s -> s.contains(position1) && s.contains(position2)).findFirst();
	}

	/**
	 * Indicates whether or not this sampling environment contains a position.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return true if this sampling environment contains the position, false
	 *         otherwise
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see Environment#contains(Position)
	 * @see Box#contains(Vec4)
	 */
	@Override
	public boolean contains(Position position) {
		if (null != this.globe) {
			return super.contains(this.globe.computePointFromPosition(position));
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}

	/**
	 * Gets the center position in globe coordinates of this sampling environment.
	 * 
	 * @return the center position in globe coordinates of this sampling environment
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see Box#getCenter()
	 */
	@Override
	public Position getCenterPosition() {
		Position centerPosition = null;

		if (null != this.globe) {
			centerPosition = this.globe.computePositionFromPoint(this.getCenter());
		} else {
			throw new IllegalStateException("globe is not set");
		}

		return centerPosition;
	}

	/**
	 * Indicates whether or not this sampling environment is refined, that is, has
	 * children.
	 * 
	 * @return true if this sampling environment is refined, false otherwise
	 * 
	 */
	public boolean isRefined() {
		// TODO: review maximum resolution
		return this.getResolution() <= 0.01;
	}

	/**
	 * Gets the refinements, that is, children of this sampling environment.
	 *
	 * @return the refinements of this sampling environment
	 *
	 */
	public Set<SamplingEnvironment> getRefinements() {
		// TODO: Review
		return new HashSet<SamplingEnvironment>();
	}

	/**
	 * Refines, that is, adds children with a specified density to this planning
	 * grid.
	 * 
	 * @param density the refinement density
	 * 
	 */
	public void refine(int percentage) {
		// TODO: Review
		this.setResolution(this.getResolution() * (1 - percentage / 100d));
	}

	/**
	 * Coarsens, that is, removes the children of this sampling environment.
	 *
	 */
	public void coarsen() {
		// TODO: Review
		this.setResolution(this.getResolution() * 1.5);
	}

	/**
	 * Indicates whether or not a position is a waypoint in this sampling
	 * environment.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return true if the position is a waypoint in this sampling environment,
	 *         false otherwise
	 * 
	 * @throws IllegalStateException if the globe is not set
	 */
	public boolean isWaypoint(Position position) {
		if (null != this.globe) {
			return waypointList.contains(position);
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}

	/**
	 * Gets the adjacent waypoints of a position in this sampling environment.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return the adjacent waypoints of the position in this sampling environment,
	 *         or the waypoint position itself
	 * 
	 * @throws IllegalStateException if the globe is not set
	 */
	public Set<Position> getAdjacentWaypoints(Position position) {
		// TODO: How to define limit to classify an waypoint as adjacent?
		int kNear = 1;
		if (null != this.globe) {
			return new HashSet<Position>(this.findNearest(position, kNear));
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}

	/**
	 * Indicates whether or not a position is adjacent to a waypoint in this
	 * sampling environment.
	 * 
	 * @param position the position in globe coordinates
	 * @param waypoint the waypoint in globe coordinates
	 * 
	 * @return true if the position is adjacent to the waypoint in this planning
	 *         grid, false otherwise
	 * 
	 * @throws IllegalStateException if the globe is not set
	 */
	public boolean isAdjacentWaypoint(Position position, Position waypoint) {
		if (null != this.globe) {
			return this.getAdjacentWaypoints(position).contains(waypoint);
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}

	/**
	 * @see Environment#getNeighbors()
	 */
	@Override
	public Set<? extends Environment> getNeighbors() {
		// TODO: Review how to define neighborhood of sampling continuum environments
		return new HashSet<Environment>();
	}

	/**
	 * @see Environment#areNeighbors(Environment)
	 */
	@Override
	public boolean areNeighbors(Environment neighbor) {
		// TODO: Define based on getNeibors()
		return false;
	}

	/**
	 * Gets the neighbors of a position in this sampling environment. A full
	 * recursive search is performed considering non-parent cells only.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return the neighbors of the position in this sampling environment
	 * 
	 * @throws IllegalStateException if the globe is not set
	 */
	public Set<Position> getNeighbors(Position position) {
		Set<Position> neighbors = new HashSet<Position>();

		if (null != this.globe) {
			neighbors = this.getEdgeList().stream().filter(s -> s.contains(position))
					.map(s -> s.getOtherPosition(position)).collect(Collectors.toSet());
		} else {
			throw new IllegalStateException("globe is not set");
		}

		return neighbors;
	}

	/**
	 * Indicates whether or not two positions are neighbors in this sampling
	 * environment.
	 * 
	 * @param position the position
	 * @param neighbor the potential neighbor of the position
	 * 
	 * @return true if the two positions are neighbors, false otherwise
	 * 
	 * @throws IllegalStateException if the globe is not set
	 */
	public boolean areNeighbors(Position position, Position neighbor) {
		if (null != this.globe) {
			return this.getNeighbors(position).contains(neighbor);
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}

	/**
	 * Gets the distance between two positions in this sampling environment.
	 * 
	 * @param position1 the first position
	 * @param position2 the second position
	 * 
	 * @return the distance between the two positions in this sampling environment
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see Environment#getDistance(Position, Position)
	 */
	@Override
	public double getDistance(Position position1, Position position2) {
		if (null != this.globe) {
			ArrayList<Position> positions = new ArrayList<Position>();
			positions.add(position1);
			positions.add(position2);
			LengthMeasurer measurer = new LengthMeasurer(positions);
			measurer.setPathType(Polyline.LINEAR);
			measurer.setFollowTerrain(false);
			return measurer.getLength(this.globe);
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}

	/**
	 * Gets the normalized distance between two positions in this sampling
	 * environment.
	 * 
	 * @param position1 the first position
	 * @param position2 the second position
	 * 
	 * @return the normalized distance between the two positions in this sampling
	 *         environment
	 */
	@Override
	public double getNormalizedDistance(Position position1, Position position2) {
		return this.getDistance(position1, position2) / this.getDiameter();
	}

	/**
	 * Gets the step cost from an origin to a destination position within this
	 * sampling environment between a start and an end time given a cost policy and
	 * risk policy.
	 * 
	 * @param origin the origin position in globe coordinates
	 * @param destination the destination position in globe coordinates
	 * @param start the start time
	 * @param end the end time
	 * @param costPolicy the cost policy
	 * @param riskPolicy the risk policy
	 * 
	 * @return the step cost from the origin to the destination position
	 * 
	 * @throws IllegalStateException if there is no edge connecting both positions
	 */
	public double getStepCost(Position origin, Position destination, ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {

		Edge edge = null;
		Optional<Edge> optEdge = this.getEdge(origin, destination);

		if (!optEdge.isPresent()) {
			throw new IllegalStateException("no edge containing both positions");
		} else {
			edge = optEdge.get();
		}

		double stepCost = 0d, distance, cost;

		distance = this.getNormalizedDistance(origin, destination);

		cost = edge.calculateCost(start, end, costPolicy, riskPolicy);
		stepCost = distance * cost;

		return stepCost;
	}

	/**
	 * Gets the desirability step cost from an origin to a destination position
	 * within this sampling environment between a start and an end time given a cost
	 * policy and risk policy. This function is meant to be used for FADPRM Edges
	 * which include desirability and lambda values.
	 * 
	 * @param origin the origin position in globe coordinates
	 * @param destination the destination position in globe coordinates
	 * @param start the start time
	 * @param end the end time
	 * @param costPolicy the cost policy
	 * @param riskPolicy the risk policy
	 * 
	 * @return the step cost from the origin to the destination position
	 * 
	 * @throws IllegalStateException if there is no edge connecting both positions
	 */
	public double getDesirabilityStepCost(Position origin, Position destination, ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {

		FAPRMEdge edge = null;
		Optional<Edge> optEdge = this.getEdge(origin, destination);

		if (!optEdge.isPresent())
			throw new IllegalStateException("no edge containing both positions");
		else
			edge = (FAPRMEdge) optEdge.get();

		double stepCost = 0d, distance, cost;

		distance = this.getNormalizedDistance(origin, destination);

		cost = edge.calculateCost(start, end, costPolicy, riskPolicy);
		cost = distance * cost;

		double desirability = edge.getDesirability();
		double lambda = edge.getLambda();
		double costMultiplier = 1.5 * lambda - (lambda / desirability) + (1 / desirability) - 1;
		stepCost = cost * costMultiplier;

		return stepCost;
	}

	/**
	 * @see Environment#getLegCost(Position, Position, ZonedDateTime, ZonedDateTime,
	 *      CostPolicy, RiskPolicy)
	 */
	@Override
	public double getLegCost(Position origin, Position destination, ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {
		// TODO Auto-generated method stub
		return 0;
	}

	/**
	 * @see Environment#getLegCost(Environment, ZonedDateTime, ZonedDateTime,
	 *      CostPolicy, RiskPolicy)
	 */
	@Override
	public double getLegCost(Environment destination, ZonedDateTime start, ZonedDateTime end, CostPolicy costPolicy,
			RiskPolicy riskPolicy) {
		// TODO Auto-generated method stub
		return 0;
	}

	/**
	 * Gets the set of different obstacles containing the difference between the
	 * current obstacle set and a previous obstacle set received as input.
	 * 
	 * @param oldObstacles the set of old obstacles
	 * 
	 * @return the set of different obstacles
	 */
	@SuppressWarnings("unchecked")
	public HashSet<Obstacle> getDiffObstacles(HashSet<Obstacle> oldObstacles) {
		// Get obstacles currently present in this environment
		HashSet<Obstacle> newObstacles = (HashSet<Obstacle>) this.getObstacles().clone();

		// Compute which obstacles were removed
		HashSet<Obstacle> removedObstacles = new HashSet<Obstacle>(oldObstacles);
		removedObstacles.removeAll(newObstacles);

		// Compute which obstacles were added
		HashSet<Obstacle> addedObstacles = new HashSet<Obstacle>(newObstacles);
		addedObstacles.removeAll(oldObstacles);

		// Compute which obstacles are different than before
		HashSet<Obstacle> diffObstacles = new HashSet<Obstacle>();
		diffObstacles.addAll(removedObstacles);
		diffObstacles.addAll(addedObstacles);

		return diffObstacles;
	}

	/**
	 * Finds all the edges in the edge list which are affected by the given
	 * obstacle.
	 * 
	 * @param obstacle the obstacle to be considered for intersection
	 * 
	 * @return the set of affected edges
	 */
	public Set<Edge> findAffectedEdges(Obstacle obstacle) {
		return this.getEdgeList().stream()
				.filter(e -> obstacle.getExtent(this.getGlobe()).intersects(e.getLine()))
				.collect(Collectors.toSet());
	}

	/**
	 * Embeds an obstacle into the affected edges.
	 * 
	 * @param obstacle the obstacle to be embedded
	 */
	public void embedEdges(Obstacle obstacle) {
		for (Edge edge : this.getEdgeList()) {
			if (obstacle.getExtent(this.globe).intersects(edge.getLine())) {
				edge.addCostInterval(obstacle.getCostInterval());
			}
		}
	}

	/**
	 * Embeds an obstacle into this sampling environment.
	 * 
	 * @param obstacle the obstacle to be embedded
	 * 
	 * @return true if the obstacle has been embedded, false otherwise
	 * 
	 * @see Environment#embed(Obstacle)
	 */
	@Override
	public boolean embed(Obstacle obstacle) {
		boolean embedded = false;

		if (null != this.globe) {
			if (!this.isEmbedded(obstacle) && this.intersects(obstacle.getExtent(this.globe))) {
				this.addCostInterval(obstacle.getCostInterval());
				this.obstacles.add(obstacle);
				this.embedEdges(obstacle);

				embedded = true;
			}
		} else {
			throw new IllegalStateException("globe is not set");
		}

		return embedded;
	}

	/**
	 * Unembeds an obstacle from this sampling environment.
	 * 
	 * @param obstacle the obstacle to be unembedded
	 * 
	 * @return true if the obstacle has been unembedded, false otherwise
	 * 
	 * @see Environment#unembed(Obstacle)
	 */
	@Override
	public boolean unembed(Obstacle obstacle) {
		boolean unembedded = false;

		if (this.isEmbedded(obstacle)) {
			this.removeCostInterval(obstacle.getCostInterval());
			this.obstacles.remove(obstacle);

			unembedded = true;
		}

		return unembedded;
	}

	/**
	 * Updates this sampling environment for an embedded obstacle.
	 * 
	 * @param obstacle the embedded obstacle
	 * 
	 * @see Environment#refresh(Obstacle)
	 */
	@Override
	public void refresh(Obstacle obstacle) {
		if (this.obstacles.contains(obstacle)) {
			this.update();
		}
	}

	/**
	 * Unembeds all obstacles from this sampling environment.
	 * 
	 * @see Environment#unembedAll()
	 */
	@Override
	public void unembedAll() {
		Iterator<Obstacle> obstaclesIterator = this.obstacles.iterator();
		while (obstaclesIterator.hasNext()) {
			Obstacle obstacle = obstaclesIterator.next();
			this.removeCostInterval(obstacle.getCostInterval());
			obstaclesIterator.remove();
		}
	}

	/**
	 * Indicates whether or not an obstacle is embedded in this sampling
	 * environment.
	 * 
	 * @param obstacle the obstacle
	 * 
	 * @return true if the obstacle is embedded in this sampling environment, false
	 *         otherwise
	 * 
	 * @see Environment#isEmbedded(Obstacle)
	 */
	@Override
	public boolean isEmbedded(Obstacle obstacle) {
		return this.obstacles.contains(obstacle);
	}

	/**
	 * Embeds a terrain obstacle into this sampling environment.
	 * 
	 * @param obstacle the terrain obstacle to be embedded
	 * 
	 * @return true if the obstacle has been embedded, false otherwise
	 */
	public boolean embed(TerrainObstacle obstacle) {
		boolean embedded = false;

		if (null != this.globe) {
			if (!this.isEmbedded(obstacle) && this.intersects(obstacle.getExtent(this.globe))) {
				this.terrainObstacles.add(obstacle);

				embedded = true;
			}
		} else {
			throw new IllegalStateException("globe is not set");
		}

		return embedded;
	}

	/**
	 * Unembeds a terrain obstacle from this sampling environment.
	 * 
	 * @param obstacle the obstacle to be unembedded
	 * 
	 * @return true if the obstacle has been unembedded, false otherwise
	 */
	public boolean unembed(TerrainObstacle obstacle) {
		boolean unembedded = false;

		if (this.isEmbedded(obstacle)) {

			this.terrainObstacles.remove(obstacle);

			unembedded = true;
		}

		return unembedded;
	}

	/**
	 * Updates this sampling environment for an embedded terrain obstacle.
	 * 
	 * @param obstacle the embedded obstacle
	 */
	public void refresh(TerrainObstacle obstacle) {
		if (this.terrainObstacles.contains(obstacle)) {
			this.update();
		}
	}

	/**
	 * Unembeds all terrain obstacles from this sampling environment.
	 */
	public void unembedTerrainAll() {
		this.terrainObstacles.clear();
	}

	/**
	 * Indicates whether or not a terrain obstacle is embedded in this sampling
	 * environment.
	 * 
	 * @param obstacle the obstacle
	 * 
	 * @return true if the obstacle is embedded in this sampling environment, false
	 *         otherwise
	 */
	public boolean isEmbedded(TerrainObstacle obstacle) {
		return this.terrainObstacles.contains(obstacle);
	}

	/**
	 * Updates this sampling environment.
	 */
	protected void update() {
		this.updateActiveCost();
		this.updateAppearance();
		this.updateVisibility();
	}

	/**
	 * Updates the accumulated active cost of this sampling environment.
	 */
	protected void updateActiveCost() {
		this.activeCost = this.getCost(this.time);
	}

	/**
	 * Updates the visibility of this sampling environment.
	 */
	protected void updateVisibility() {
		this.setVisible(this.activeCost > this.thresholdCost);
	}

	/**
	 * Updates the appearance of this sampling environment.
	 */
	protected void updateAppearance() {
		Color activeColor = ObstacleColor.getColor(activeCost);
		float red = activeColor.getRed() / 255.0f;
		float green = activeColor.getGreen() / 255.0f;
		float blue = activeColor.getBlue() / 255.0f;
		float alpha = activeColor.getAlpha() / 255.0f;
		this.setColor(red, green, blue, alpha);
	}

	/**
	 * Gets the interval tree that is defined for a specified position.
	 * 
	 * @param position the position to be checked
	 * 
	 * @return the interval tree with all cost intervals
	 */
	public IntervalTree<ChronoZonedDateTime<?>> getIntervalTree(Position position) {
		IntervalTree<ChronoZonedDateTime<?>> intervalTree = new IntervalTree<ChronoZonedDateTime<?>>(
				CostInterval.comparator);

		if (null != this.globe) {
			for (Obstacle obstacle : obstacles) {
				if (this.intersects(obstacle.getExtent(this.globe))) {
					intervalTree.add(obstacle.getCostInterval());
				}
			}
		}

		return intervalTree;
	}

	/**
	 * Gets the interval tree that is defined for a specified line.
	 * 
	 * @param line the line to be checked
	 * 
	 * @return the interval tree with all cost intervals
	 */
	public IntervalTree<ChronoZonedDateTime<?>> embedIntervalTree(Line line) {
		IntervalTree<ChronoZonedDateTime<?>> intervalTree = new IntervalTree<ChronoZonedDateTime<?>>(
				CostInterval.comparator);

		for (Obstacle obstacle : this.getObstacles()) {
			if (obstacle.getExtent(this.getGlobe()).intersects(line)) {
				intervalTree.add(obstacle.getCostInterval());
			}
		}
		return intervalTree;
	}

	/**
	 * Creates a bounding box with a defined radius surrounding a given position.
	 * 
	 * @param position the position to be considered
	 * @param radius the radius for the bounding box
	 * 
	 * @return the bounding box surrounding the position
	 */
	public Box createBoundingBox(Position position, double radius) {
		Vec4 point = this.getGlobe().computePointFromPosition(position);
		List<Vec4> corners = new ArrayList<Vec4>();

		corners.add(point.add3(-radius, +radius, -radius));
		corners.add(point.add3(+radius, +radius, -radius));
		corners.add(point.add3(+radius, -radius, -radius));
		corners.add(point.add3(+radius, -radius, +radius));
		corners.add(point.add3(-radius, -radius, +radius));
		corners.add(point.add3(-radius, +radius, +radius));

		return new Box(gov.nasa.worldwind.geom.Box.computeBoundingBox(corners));
	}

	/**
	 * Samples a position from a continuous space defined in the current
	 * environment.
	 * 
	 * @return position in globe coordinates inside the environment
	 */
	public Position sampleRandomPosition() {
		Vec4[] corners = this.getCorners();

		// Point in box frame with all minimum coordinates
		Vec4 minimum = this.transformModelToBoxOrigin(corners[0]);
		// Point in box frame with all maximum coordinates
		Vec4 maximum = this.transformModelToBoxOrigin(corners[6]);

		double x, y, z;

		x = minimum.x + (new Random().nextDouble() * (maximum.x - minimum.x));
		y = minimum.y + (new Random().nextDouble() * (maximum.y - minimum.y));
		z = minimum.z + (new Random().nextDouble() * (maximum.z - minimum.z));

		Vec4 point = new Vec4(x, y, z);

		// Transform point from box frame to earth frame
		point = this.transformBoxOriginToModel(point);

		Position position = this.getGlobe().computePositionFromPoint(point);

		return position;
	}

	/**
	 * Samples a pseudo-random position from the intersection of the continuous
	 * space defined in the current environment and an ellipsoid defined by its two
	 * foci points and the maximum distance.
	 * 
	 * @param focusA the focus pointA in world coordinates
	 * @param focusB the focus pointA in world coordinates
	 * @param distance double the length for the major axis of the ellipsoid
	 * 
	 * @throws IllegalStateException if distance between foci is larger than bound
	 * 
	 * @return the pseudo-random sampled position
	 */
	public Position samplePositionEllipsoide(Position focusA, Position focusB, double distance) {
		// Test if the box is more restrictive than the ellipsoid "diameter"
		if (distance > this.getDiameter())
			return sampleRandomPosition();

		if (distance < this.getDistance(focusA, focusB)) {
			throw new IllegalStateException("Distance between foci larger than bound");
		}

		Position positionRand;

		// Middle Position from focusA and focusB
		Position positionM = CoordinateTransformations.middlePosition(focusA, focusB, getGlobe());

		// Ellipsoid parameters
		double a = distance / 2d;
		double c = this.getDistance(focusA, focusB) / 2d;
		double b = Math.sqrt(a * a - c * c);

		do {
			// Sample random point inside a unit sphere
			double r = 0d + new Random().nextDouble() * 1d;
			r = Math.sqrt(r); // to ensure uniform distribution in ellipsis since area is proportional to r^2
			double theta = 0d + new Random().nextDouble() * Math.PI;
			double phi = 0d + new Random().nextDouble() * 2 * Math.PI;
			Vec4 pointRand = CoordinateTransformations.polar2cartesian(phi, theta, r);

			// Transform from sphere to ellipsoid
			pointRand = new Vec4(pointRand.x * b, pointRand.y * a, pointRand.z * b);

			// Translation and rotation from local frame to global
			Vec4 aerB = CoordinateTransformations.llh2aer(positionM, focusB, getGlobe()), enuRand;
			double angleZ = Math.PI / 2 - aerB.x, angleX = aerB.y;
			enuRand = CoordinateTransformations.rotationZ(pointRand, -angleZ);
			enuRand = CoordinateTransformations.rotationX(enuRand, -angleX);
			positionRand = CoordinateTransformations.enu2llh(positionM, enuRand, getGlobe());

			// Check if point is inside box
		} while (!this.contains(positionRand));

		return positionRand;
	}

	/**
	 * Checks whether the given position is inside the given globe.
	 * 
	 * @param globe the globe
	 * @param position the position in global coordinates
	 * 
	 * @return true if the position is inside the globe and false otherwise
	 */
	public boolean isInsideGlobe(Globe globe, Position position) {
		Vec4 point = this.getGlobe().computePointFromPosition(position);
		return !globe.isPointAboveElevation(point, globe.getElevation(position.latitude, position.longitude));
	}

	/**
	 * Checks if a given position is in conflict with untraversable obstacles in the
	 * environment.
	 * 
	 * @param position the position in global coordinates
	 * @param aircraft the aircraft to be considered
	 * 
	 * @return boolean value true if there is a conflict
	 */
	public boolean checkConflict(Position position, Aircraft aircraft) {

		// Check if position is inside the globe
		if (this.isInsideGlobe(this.getGlobe(), position))
			return true;
		
		// Create sphere around the position of the aircraft with its radius of action
		Sphere sphere = new Sphere(getGlobe().computePointFromPosition(position), aircraft.getRadius());

		// Check conflict between terrain obstacles and aircraft sphere of action
		HashSet<TerrainObstacle> terrainSet = this.getTerrainObstacles();
		for (TerrainObstacle terrain : terrainSet) {
			// Check if obstacle contains the waypoint
			if(sphere.intersects(terrain.getFrustum(getGlobe()))) {
				return true;
			}
		}
		return false;
	}

	/**@param aircraft the aircraft to be considered
	 * Checks if a straight leg between the positions is in conflict with
	 * untraversable obstacles in the environment.
	 * 
	 * @param position1 the first position in global coordinates
	 * @param position2 the second waypoint in global coordinates
	 * @param aircraft the aircraft to be considered
	 * 
	 * @return boolean value true if there is a conflict
	 */
	public boolean checkConflict(Position position1, Position position2, Aircraft aircraft) {
		Vec4 point1 = this.getGlobe().computePointFromPosition(position1);
		Vec4 point2 = this.getGlobe().computePointFromPosition(position2);
		Vec4 aux;

		double x, y, z, dx, dy, dz, dist, dist2, theta, phi;

		dx = point2.x - point1.x;
		dy = point2.y - point1.y;
		dz = point2.z - point1.z;

		dist = point1.distanceTo3(point2);
		dist2 = point1.distanceTo2(point2);
		theta = Math.atan2(dz, dist2);
		phi = Math.atan2(dy, dx);

		double resolution = this.getResolution(); // meters in globe surface
		for (int p = 1; dist > resolution; p = p * 2) {
			for (int k = 0; k < p; k++) {
				x = point1.x + (1 / 2 + k) * dist * Math.cos(theta) * Math.cos(phi);
				y = point1.y + (1 / 2 + k) * dist * Math.cos(theta) * Math.sin(phi);
				z = point1.z + (1 / 2 + k) * dist * Math.sin(theta);
				aux = new Vec4(x, y, z);
				if (this.checkConflict(this.getGlobe().computePositionFromPoint(aux), aircraft))
					return true;
			}
			dist = dist / 2;
		}

		return false;
	}

	/**
	 * Finds the k-nearest waypoints to the given position.
	 * 
	 * @param position the position in global coordinates
	 * @param kNear number of waypoints to return
	 * 
	 * @return list of k-nearest waypoints sorted by increasing distance
	 */
	public List<? extends Position> findNearest(Position position, int kNear) {

		return this.getWaypointList().stream().sorted((p1, p2) -> Double
				.compare(this.getNormalizedDistance(p1, position), this.getNormalizedDistance(p2, position)))
				.filter(p -> !p.equals(position))
				.limit(kNear).collect(Collectors.toList());

	}
	
	public List<? extends Position> findNearestDist(Position position, double maxDist) {

		return this.getWaypointList().stream().sorted((p1, p2) -> Double
				.compare(this.getNormalizedDistance(p1, position), this.getNormalizedDistance(p2, position)))
				.filter(p -> this.getDistance(p, position) <= maxDist && !p.equals(position))
				.collect(Collectors.toList());

	}

	/**
	 * Sorts a list of elements by increasing distance to a given position.
	 * 
	 * @param position the position in global coordinates
	 */
	public void sortNearest(Position position) {

		this.setWaypointList(this.getWaypointList().stream().sorted((p1, p2) -> Double
				.compare(this.getNormalizedDistance(p1, position), this.getNormalizedDistance(p2, position)))
				.collect(Collectors.toList()));

	}

}
