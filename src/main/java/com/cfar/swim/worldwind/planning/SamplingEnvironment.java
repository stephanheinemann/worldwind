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
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.geom.ContinuumBox;
import com.cfar.swim.worldwind.geom.CubicGrid;
import com.cfar.swim.worldwind.geom.RegularGrid;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.ObstacleColor;
import com.cfar.swim.worldwind.render.TerrainCylinder;
import com.cfar.swim.worldwind.render.TerrainObstacle;
import com.cfar.swim.worldwind.render.ThresholdRenderable;
import com.cfar.swim.worldwind.render.TimedRenderable;
import com.cfar.swim.worldwind.render.airspaces.ObstacleCylinder;

import gov.nasa.worldwind.geom.Line;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Polyline;
import gov.nasa.worldwind.util.measure.LengthMeasurer;

/**
 * Realizes a planning continuum that implements an environment, can be used for
 * sampled based motion planning.
 * 
 * @author Manuel Rosa
 * @author Henrique Ferreira
 *
 */
public class SamplingEnvironment extends ContinuumBox implements Environment {

	/** the globe of this planning continuum */
	private Globe globe = null;

	/** the cost interval tree encoding temporal costs */
	private IntervalTree<ChronoZonedDateTime<?>> costIntervals = new IntervalTree<ChronoZonedDateTime<?>>(
			CostInterval.comparator);

	/** the current time of this planning continuum */
	private ZonedDateTime time = ZonedDateTime.now(ZoneId.of("UTC"));

	/** the obstacles embedded into this planning continuum */
	private HashSet<Obstacle> obstacles = new HashSet<Obstacle>();

	/** the terrain obstacles embedded into this planning continuum */
	private HashSet<TerrainObstacle> terrainObstacles = new HashSet<TerrainObstacle>();

	/** the list of already sampled waypoints */
	private List<Waypoint> waypointList = new ArrayList<Waypoint>();

	/** the list of edges in this environment */
	private List<Edge> edgeList = new ArrayList<Edge>();

	/** the current accumulated active cost of this planning continuum */
	// TODO Review usage of active cost
	private double activeCost = 1d;

	/** the threshold cost of this planning continuum */
	private double thresholdCost = 0d;

	/** the resolution of this planning continuum */
	private double resolution = 1d;

	/**
	 * Constructs a planning continuum based on a geometric box.
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
	 * Constructs a planning continuum based on a geometric box.
	 * 
	 * @param box the geometric box
	 * @param resolution the resolution of this planning continuum
	 * 
	 * @see Box#Box(gov.nasa.worldwind.geom.Box)
	 */
	public SamplingEnvironment(Box box, double resolution) {
		super(box);
		this.resolution = resolution;
		this.update();
	}

	/**
	 * Gets the threshold cost of this planning continuum.
	 * 
	 * @return the threshold cost of this planning continuum
	 * 
	 * @see ThresholdRenderable#setThreshold(double)
	 */
	@Override
	public double getThreshold() {
		return this.thresholdCost;
	}

	/**
	 * Sets the threshold cost of this planning continuum.
	 * 
	 * @param thresholdCost the threshold cost of this planning continuum
	 * 
	 * @see ThresholdRenderable#setThreshold(double)
	 */
	@Override
	public void setThreshold(double thresholdCost) {
		this.thresholdCost = thresholdCost;
		this.updateVisibility();
	}

	/**
	 * Sets the globe of this planning continuum.
	 * 
	 * @param globe the globe of this planning continuum
	 * 
	 * @see Environment#setGlobe(Globe)
	 */
	@Override
	public void setGlobe(Globe globe) {
		this.globe = globe;
	}

	/**
	 * Gets the globe of this planning continuum.
	 * 
	 * @return the globe of this planning continuum
	 * 
	 * @see Environment#getGlobe()
	 */
	@Override
	public Globe getGlobe() {
		return this.globe;
	}

	/**
	 * Adds a cost interval to this planning grid.
	 * 
	 * @param costInterval the cost interval to be added
	 */
	public void addCostInterval(CostInterval costInterval) {
		this.costIntervals.add(costInterval);
		this.update();
		// TODO: Should costs be automatically propagated to the affected child cells?
		// TODO: Should added children costs be propagated to parents?
		// TODO: What happens if children are added and removed?
		// TODO: Shall parents aggregate all costs?!
	}

	/**
	 * Removes a cost interval from this planning grid.
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
	 * Gets the accumulated cost of this planning grid at specified time instant.
	 * 
	 * @param time the time instant
	 * 
	 * @return the accumulated cost of this planning grid at the specified time
	 *         instant
	 */
	public double getCost(ZonedDateTime time) {
		return this.getCost(time, time);
	}

	/**
	 * Gets the accumulated cost of this planning grid within a specified time span.
	 * 
	 * @param start the start time of the time span
	 * @param end the end time of the time span
	 * 
	 * @return the accumulated cost of this planning grid within the specified time
	 *         span
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
	 * Gets the current time of this planning continuum.
	 * 
	 * @return the current time of this planning continuum
	 * 
	 * @see TimedRenderable#getTime()
	 */
	@Override
	public ZonedDateTime getTime() {
		return this.time;
	}

	/**
	 * Sets the current time of this planning continuum.
	 * 
	 * @param time the current time of this planning continuum
	 * 
	 * @see TimedRenderable#setTime(ZonedDateTime)
	 */
	// TODO: Review meaning, not clear
	@Override
	public void setTime(ZonedDateTime time) {
		this.time = time;
		this.update();
	}

	/**
	 * Gets the obstacles of this planning continuum.
	 * 
	 * @return the obstacles
	 */
	public HashSet<Obstacle> getObstacles() {
		return obstacles;
	}

	/**
	 * Sets the obstacles of this planning continuum.
	 * 
	 * @param obstacles the obstacles to set
	 */
	public void setObstacles(HashSet<Obstacle> obstacles) {
		this.obstacles = obstacles;
	}

	/**
	 * Gets the resolution of this planning continuum
	 * 
	 * @return the resolution of this planning continuum
	 */
	public double getResolution() {
		return resolution;
	}

	/**
	 * Sets the resolution of this planning continuum
	 * 
	 * @param resolution the resolution to set
	 */
	public void setResolution(double resolution) {
		this.resolution = resolution;
	}

	/**
	 * Gets the terrain obstacles of this planning continuum.
	 * 
	 * @return the terrain obstacles
	 */
	public HashSet<TerrainObstacle> getTerrainObstacles() {
		return terrainObstacles;
	}

	/**
	 * Sets the terrain obstacles of this planning continuum.
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
	public List<Edge> getEdgeList() {
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
	 * Indicates whether or not this planning continuum contains a position.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return true if this planning continuum contains the position, false
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
	 * Gets the center position in globe coordinates of this planning continuum.
	 * 
	 * @return the center position in globe coordinates of this planning continuum
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
	 * Indicates whether or not this planning grid is refined, that is, has
	 * children.
	 * 
	 * @return true if this planning grid is refined, false otherwise
	 * 
	 */
	public boolean isRefined() {
		// TODO: review maximum resolution
		return this.getResolution() <= 0.01;
	}

	/**
	 * Gets the refinements, that is, children of this planning grid.
	 *
	 * @return the refinements of this planning grid
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
	 * Coarsens, that is, removes the children of this planning grid.
	 *
	 */
	public void coarsen() {
		// TODO: Review
		this.setResolution(this.getResolution() * 1.5);
	}

	/**
	 * Indicates whether or not a position is a waypoint in this planning grid.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return true if the position is a waypoint in this planning grid, false
	 *         otherwise
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see RegularGrid#isWayoint(Vec4)
	 */
	public boolean isWaypoint(Position position) {
		if (null != this.globe) {
			return waypointList.contains(position);
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}

	/**
	 * Gets the adjacent waypoints of a position in this planning grid.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return the adjacent waypoints of the position in this planning grid, or the
	 *         waypoint position itself
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see RegularGrid#getAdjacentWaypoints(Vec4)
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
	 * planning grid.
	 * 
	 * @param position the position in globe coordinates
	 * @param waypoint the waypoint in globe coordinates
	 * 
	 * @return true if the position is adjacent to the waypoint in this planning
	 *         grid, false otherwise
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see RegularGrid#isAdjacentWaypoint(Vec4, Vec4)
	 */
	public boolean isAdjacentWaypoint(Position position, Position waypoint) {
		if (null != this.globe) {
			return this.getAdjacentWaypoints(position).contains(waypoint);
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}

	// TODO What are neighboring environments?
	@Override
	public Set<? extends Environment> getNeighbors() {
		return new HashSet<Environment>();
	}

	@Override
	public boolean areNeighbors(Environment neighbor) {
		return false;
	}

	/**
	 * Gets the neighbors of a position in this planning grid. A full recursive
	 * search is performed considering non-parent cells only.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return the neighbors of the position in this planning grid
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see CubicGrid#getNeighbors(Vec4)
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
	 * Indicates whether or not two positions are neighbors in this planning grid.
	 * 
	 * @param position the position
	 * @param neighbor the potential neighbor of the position
	 * 
	 * @return true if the two positions are neighbors, false otherwise
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see RegularGrid#areNeighbors(Vec4, Vec4)
	 */
	public boolean areNeighbors(Position position, Position neighbor) {
		if (null != this.globe) {
			return this.getNeighbors(position).contains(neighbor);
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}

	/**
	 * Gets the distance between two positions in this planning continuum.
	 * 
	 * @param position1 the first position
	 * @param position2 the second position
	 * 
	 * @return the distance between the two positions in this planning continuum
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
	 * Gets the normalized distance between two positions in this planning
	 * continuum.
	 * 
	 * @param position1 the first position
	 * @param position2 the second position
	 * 
	 * @return the normalized distance between the two positions in this planning
	 *         continuum
	 */
	@Override
	public double getNormalizedDistance(Position position1, Position position2) {
		return this.getDistance(position1, position2) / this.getDiameter();
	}

	/**
	 * Gets the step cost from an origin to a destination position within this
	 * planning continuum between a start and an end time given a cost policy and
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

		if (!optEdge.isPresent())
			throw new IllegalStateException("no edge containing both positions");
		else
			edge = optEdge.get();

		double stepCost = 0d, distance, cost;

		distance = this.getNormalizedDistance(origin, destination);

		cost = edge.calculateCost(start, end, costPolicy);

		if (riskPolicy.satisfies(cost - 1)) {
			cost = distance * cost;
		} else {
			cost = Double.POSITIVE_INFINITY;
		}

		stepCost = cost;

		return stepCost;
	}

	@Override
	public double getLegCost(Position origin, Position destination, ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getLegCost(Environment destination, ZonedDateTime start, ZonedDateTime end, CostPolicy costPolicy,
			RiskPolicy riskPolicy) {
		// TODO Auto-generated method stub
		return 0;
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
	 * Embeds an obstacle into the affected edges.
	 * 
	 * @param obstacle the obstacle to be embedded
	 */
	public void embedEdges(ObstacleCylinder obstacle) {
		for (Edge edge : this.getEdgeList()) {
			if (obstacle.getExtent(this.globe).intersects(edge.getLine())) {
				edge.addCostInterval(obstacle.getCostInterval());
			}
		}
	}

	/**
	 * Embeds an obstacle into this planning continuum.
	 * 
	 * @param obstacle the obstacle to be embedded
	 * 
	 * @return true if the obstacle has been embedded, false otherwise
	 * 
	 * @see Environment#embed(Obstacle)
	 */
	public boolean embed(ObstacleCylinder obstacle) {
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
	 * Embeds an obstacle into this planning continuum.
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
	 * Unembeds an obstacle from this planning continuum.
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
	 * Updates this planning continuum for an embedded obstacle.
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
	 * Unembeds all obstacles from this planning continuum.
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
	 * Indicates whether or not an obstacle is embedded in this planning continuum.
	 * 
	 * @param obstacle the obstacle
	 * 
	 * @return true if the obstacle is embedded in this planning continuum, false
	 *         otherwise
	 * 
	 * @see Environment#isEmbedded(Obstacle)
	 */
	@Override
	public boolean isEmbedded(Obstacle obstacle) {
		return this.obstacles.contains(obstacle);
	}

	/**
	 * Embeds an obstacle into this planning continuum.
	 * 
	 * @param obstacle the obstacle to be embedded
	 * 
	 * @return true if the obstacle has been embedded, false otherwise
	 * 
	 * @see Environment#embed(Obstacle)
	 */
	public boolean embed(TerrainCylinder obstacle) {
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
	 * Embeds an obstacle into this planning continuum.
	 * 
	 * @param obstacle the obstacle to be embedded
	 * 
	 * @return true if the obstacle has been embedded, false otherwise
	 * 
	 * @see Environment#embed(Obstacle)
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
	 * Unembeds an obstacle from this planning continuum.
	 * 
	 * @param obstacle the obstacle to be unembedded
	 * 
	 * @return true if the obstacle has been unembedded, false otherwise
	 * 
	 * @see Environment#unembed(Obstacle)
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
	 * Updates this planning continuum for an embedded obstacle.
	 * 
	 * @param obstacle the embedded obstacle
	 * 
	 * @see Environment#refresh(Obstacle)
	 */
	public void refresh(TerrainObstacle obstacle) {
		if (this.terrainObstacles.contains(obstacle)) {
			this.update();
		}
	}

	/**
	 * Unembeds all obstacles from this planning continuum.
	 * 
	 * @see Environment#unembedAll()
	 */
	public void unembedTerrainAll() {
		this.terrainObstacles.clear();
	}

	/**
	 * Indicates whether or not an obstacle is embedded in this planning continuum.
	 * 
	 * @param obstacle the obstacle
	 * 
	 * @return true if the obstacle is embedded in this planning continuum, false
	 *         otherwise
	 * 
	 * @see Environment#isEmbedded(Obstacle)
	 */
	public boolean isEmbedded(TerrainObstacle obstacle) {
		return this.terrainObstacles.contains(obstacle);
	}

	/**
	 * Updates this planning continuum.
	 */
	protected void update() {
		this.updateActiveCost();
		this.updateAppearance();
		this.updateVisibility();
	}

	/**
	 * Updates the accumulated active cost of this planning continuum.
	 */
	protected void updateActiveCost() {
		this.activeCost = this.getCost(this.time);
	}

	/**
	 * Updates the visibility of this planning continuum.
	 */
	protected void updateVisibility() {
		this.setVisible(this.activeCost > this.thresholdCost);
	}

	/**
	 * Updates the appearance of this planning continuum.
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
	 * TODO
	 * 
	 * @param line
	 * @return
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
	 * TODO
	 * 
	 * @param position
	 * @return
	 */
	public Box createBoundingBox(Position position) {
		Vec4 point = this.getGlobe().computePointFromPosition(position);
		List<Vec4> corners = new ArrayList<Vec4>();

		// TODO: create box according to aircraft dimensions
		double halfDistance = 0.1d;

		corners.add(point.add3(-halfDistance, +halfDistance, -halfDistance));
		corners.add(point.add3(+halfDistance, +halfDistance, -halfDistance));
		corners.add(point.add3(+halfDistance, -halfDistance, -halfDistance));
		corners.add(point.add3(+halfDistance, -halfDistance, +halfDistance));
		corners.add(point.add3(-halfDistance, -halfDistance, +halfDistance));
		corners.add(point.add3(-halfDistance, +halfDistance, +halfDistance));

		return new Box(gov.nasa.worldwind.geom.Box.computeBoundingBox(corners));
	}

	/**
	 * Samples a position from a continuous space defined in the current environment
	 * 
	 * @return position in global coordinates inside the environment
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
	 * Checks whether the given position is inside the given globe
	 * 
	 * @param globe the globe
	 * @param position the position in global coordinates
	 * @return true if the position is inside the globe and false otherwise
	 */
	public boolean isInsideGlobe(Globe globe, Position position) {
		Vec4 point;

		point = this.getGlobe().computePointFromPosition(position);
		return !globe.isPointAboveElevation(point, globe.getElevation(position.latitude, position.longitude));
	}

	/**
	 * Checks if a given position is in conflict with untraversable obstacles in the
	 * environment
	 * 
	 * @param waypoint the waypoint in global coordinates
	 * 
	 * @return boolean value true if there is a conflict
	 */
	public boolean checkConflict(Position position) {

		if (this.isInsideGlobe(this.getGlobe(), position))
			return true;

		// TODO : Implement a checker for conflict between a position and the
		// static, time independent and untraversable obstacles in the environment

		Box box = this.createBoundingBox(position);
		HashSet<TerrainObstacle> terrainSet = this.getTerrainObstacles();
		for (TerrainObstacle terrain : terrainSet) {
			// Check if obstacle contains the waypoint
			System.out.println("Terrain");
			if (terrain.getExtent(this.getGlobe()).intersects(box.getFrustum())) {
				return true;
			}
		}
		return false;
	}

	/**
	 * Checks if a straight leg between the waypoints is in conflict with
	 * untraversable obstacles in the environment
	 * 
	 * @param waypoint1 the first waypoint in global coordinates
	 * @param waypoint2 the second waypoint in global coordinates
	 * 
	 * @return boolean value true if there is a conflict
	 */
	public boolean checkConflict(Position position1, Position position2) {
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
				if (this.checkConflict(this.getGlobe().computePositionFromPoint(aux)))
					return true;
			}
			dist = dist / 2;
		}

		return false;
	}

	/**
	 * Finds the k-nearest waypoints to the given position
	 * 
	 * @param position the position in global coordinates
	 * @param kNear number of waypoints to return
	 * 
	 * @return list of k-nearest waypoints sorted by increasing distance
	 */
	public List<? extends Position> findNearest(Position position, int kNear) {

		return this.getWaypointList().stream().sorted((p1, p2) -> Double
				.compare(this.getNormalizedDistance(p1, position), this.getNormalizedDistance(p2, position)))
				.limit(kNear).collect(Collectors.toList());

	}

	/**
	 * Sorts a list of elements by increasing distance to a given position
	 * 
	 * @param position the position in global coordinates
	 */
	public void sortNearest(Position position) {

		this.setWaypointList(this.getWaypointList().stream().sorted((p1, p2) -> Double
				.compare(this.getNormalizedDistance(p1, position), this.getNormalizedDistance(p2, position)))
				.collect(Collectors.toList()));

	}

}
