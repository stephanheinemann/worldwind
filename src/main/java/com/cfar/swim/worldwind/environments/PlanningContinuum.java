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
package com.cfar.swim.worldwind.environments;

import java.awt.Color;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.time.chrono.ChronoZonedDateTime;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.Set;
import java.util.concurrent.CopyOnWriteArraySet;
import java.util.function.BiFunction;
import java.util.stream.Collectors;

import com.binarydreamers.trees.Interval;
import com.binarydreamers.trees.IntervalTree;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.planning.WeightedCostInterval;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.ObstacleColor;
import com.cfar.swim.worldwind.render.ThresholdRenderable;
import com.cfar.swim.worldwind.render.TimedRenderable;

import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.PolarPoint;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Quaternion;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.BasicShapeAttributes;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.Ellipsoid;
import gov.nasa.worldwind.render.Material;
import gov.nasa.worldwind.render.RigidShape;
import gov.nasa.worldwind.terrain.HighResolutionTerrain;
import gov.nasa.worldwind.util.measure.LengthMeasurer;

/**
 * Realizes a planning continuum that can be sampled by sampling based motion
 * planning algorithms.
 * 
 * @author Manuel Rosa
 * @author Henrique Ferreira
 * @author Stephan Heinemann
 */
public class PlanningContinuum extends Box
implements DynamicEnvironment, StructuredEnvironment, MultiResolutionEnvironment, SamplingEnvironment {
	
	/** the base cost of this planning continuum */
	private static final double BASE_COST = 1d;
	
	/** the globe of this planning continuum */
	private Globe globe = null;
	
	/** the current time of this planning continuum */
	private ZonedDateTime time = ZonedDateTime.now(ZoneId.of("UTC"));
	
	/** the obstacles embedded into this planning continuum */
	private HashSet<Obstacle> obstacles = new HashSet<Obstacle>();
	
	// TODO: too expensive, synchronize vertices access instead?
	/** the vertices (sampled positions) of this planning continuum */
	private Set<Position> vertices = new CopyOnWriteArraySet<Position>();
	
	// TODO: too expensive, synchronize edges access instead?
	/** the edges (sampled straight legs) of this planning continuum */
	private Set<Edge> edges = new CopyOnWriteArraySet<Edge>();
	
	/** the sampling shape within this planning continuum */
	private RigidShape samplingShape = null;
	
	/** the cost interval tree encoding temporal costs of this planning continuum */
	private IntervalTree<ChronoZonedDateTime<?>> costIntervals =
			new IntervalTree<ChronoZonedDateTime<?>>(CostInterval.comparator);
	
	/** the current accumulated active cost of this planning continuum */
	private double activeCost = 1d;
	
	/** the threshold cost of this planning continuum */
	private double thresholdCost = 0d;
	
	/** the resolution of this planning continuum */
	private double resolution = 50d;
	
	/** the structural change listeners of this planning continuum */
	private Set<StructuralChangeListener> listeners = new HashSet<StructuralChangeListener>();
	
	/**
	 * Constructs a new planning continuum based on a geometric box.
	 * 
	 * @param box the geometric box delimiting this planning continuum
	 * 
	 * @see Box#Box(gov.nasa.worldwind.geom.Box)
	 */
	public PlanningContinuum(Box box) {
		super(box);
		this.update();
	}
	
	/**
	 * Sets the globe of this planning continuum.
	 * 
	 * @param globe the globe to be set
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
	 * @return the globe of this sampling environment
	 * 
	 * @see Environment#getGlobe()
	 */
	@Override
	public Globe getGlobe() {
		return this.globe;
	}
	
	/**
	 * Determines whether or not this planning continuum has a globe.
	 * 
	 * @return true if this planning continuum has a globe, false otherwise
	 * 
	 * @see Environment#hasGlobe()
	 */
	@Override
	public boolean hasGlobe() {
		return (null != this.globe);
	}
	
	/**
	 * Determines whether or not a position is inside the globe of this
	 * planning continuum.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return true if the position is inside the globe of this planning
	 *         continuum, false otherwise
	 * 
	 * @throws IllegalStateException if this planning continuum has no globe
	 * 
	 * @see Environment#isInsideGlobe(Position)
	 */
	@Override
	public boolean isInsideGlobe(Position position) {
		if (this.hasGlobe()) {
			double elevation = this.getGlobe()
					.getElevation(position.latitude, position.longitude);
			return position.elevation < elevation;
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}
	
	/**
	 * Determines whether or not this planning continuum contains a position.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return true if this planning continuum contains the position,
	 *         false otherwise
	 * 
	 * @throws IllegalStateException if this planning continuum has no globe
	 * 
	 * @see Environment#contains(Position)
	 * @see Box#contains(Vec4)
	 */
	@Override
	public boolean contains(Position position) {
		if (this.hasGlobe()) {
			return super.contains(this.globe.computePointFromPosition(position));
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}
	
	/**
	 * Determines whether or not a position is a waypoint position in this
	 * planning continuum.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return true if the position is a waypoint position in this planning
	 *         continuum, false otherwise
	 * 
	 * @see Environment#isWaypointPosition(Position)
	 */
	@Override
	public boolean isWaypointPosition(Position position) {
		return this.vertices.contains(position);
	}
	
	/**
	 * Gets the adjacent waypoint positions of a position in this planning
	 * continuum.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return the adjacent waypoint positions of the position in this planning
	 *         continuum, or the waypoint position itself
	 * 
	 * @see Environment#getAdjacentWaypointPositions(Position)
	 */
	@Override
	public Set<Position> getAdjacentWaypointPositions(Position position) {
		return this.edges.stream()
			.filter(e -> e.isEndPosition(position))
			.map(e -> e.getOtherPosition(position))
			.collect(Collectors.toSet());
	}
	
	/**
	 * Determines whether or not a position is adjacent to a waypoint position
	 * in this planning continuum.
	 * 
	 * @param position the position in globe coordinates
	 * @param waypointPosition the waypoint position in globe coordinates
	 * 
	 * @return true if the position is adjacent to the waypoint position in
	 *         this planning continuum, false otherwise
	 * 
	 * @see Environment#isAdjacentWaypointPosition(Position, Position)
	 */
	@Override
	public boolean isAdjacentWaypointPosition(
			Position position, Position waypointPosition) {
		return this.getAdjacentWaypointPositions(position)
				.contains(waypointPosition);
	}
	
	/**
	 * Gets the center position of this planning continuum in globe coordinates.
	 * 
	 * @return the center position of this planning continuum in globe
	 *          coordinates
	 * 
	 * @throws IllegalStateException if this planning continuum has no globe
	 * 
	 * @see Environment#getCenterPosition()
	 * @see Box#getCenter()
	 */
	@Override
	public Position getCenterPosition() {
		if (this.hasGlobe()) {
			return this.globe.computePositionFromPoint(this.getCenter());
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}
	
	/**
	 * Gets the neighbors of a position in this planning continuum.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return the neighbors of the position in this planning continuum
	 * 
	 * @see Environment#getNeighbors(Position)
	 */
	@Override
	public Set<Position> getNeighbors(Position position) {
		return this.edges.stream()
				.filter(s -> s.isEndPosition(position))
				.map(s -> s.getOtherPosition(position))
				.collect(Collectors.toSet());
	}

	/**
	 * Determines whether or not two positions are neighbors in this planning
	 * continuum.
	 * 
	 * @param position the position in globe coordinates
	 * @param neighbor the potential neighbor of the position in globe
	 *                 coordinates
	 * 
	 * @return true if the two positions are neighbors, false otherwise
	 * 
	 * @see Environment#areNeighbors(Position, Position)
	 */
	public boolean areNeighbors(Position position, Position neighbor) {
		return this.getNeighbors(position).contains(neighbor);
	}
	
	/**
	 * Gets the distance between two positions in this planning continuum.
	 * 
	 * @param position1 the first position in globe coordinates
	 * @param position2 the second position in globe coordinates
	 * 
	 * @return the distance between the two positions in this planning
	 *         continuum
	 * 
	 * @throws IllegalStateException if this planning continuum has no globe
	 * 
	 * @see Environment#getDistance(Position, Position)
	 */
	@Override
	public double getDistance(Position position1, Position position2) {
		if (this.hasGlobe()) {
			ArrayList<Position> positions = new ArrayList<Position>();
			positions.add(position1);
			positions.add(position2);
			LengthMeasurer measurer = new LengthMeasurer(positions);
			measurer.setPathType(AVKey.LINEAR);
			measurer.setFollowTerrain(false);
			return measurer.getLength(this.getGlobe());
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}

	/**
	 * Gets the normalized distance between two positions in this planning
	 * continuum.
	 * 
	 * @param position1 the first position in globe coordinates
	 * @param position2 the second position in globe coordinates
	 * 
	 * @return the normalized distance between the two positions in this
	 *         planning continuum
	 */
	@Override
	public double getNormalizedDistance(Position position1, Position position2) {
		return this.getDistance(position1, position2) / this.getNormalizer();
	}
	
	/**
	 * Gets the distance normalizer of this planning continuum.
	 * 
	 * @return the distance normalizer of this planning continuum
	 * 
	 * @see Environment#getNormalizer()
	 */
	@Override
	public double getNormalizer() {
		return this.getDiameter();
	}
	
	/**
	 * Converts a normalized distance to a distance within this planning
	 * continuum.
	 * 
	 * @param normalizedDistance the normalized distance
	 * 
	 * @return the distance
	 * 
	 * @see Environment#toDistance(double)
	 */
	@Override
	public double toDistance(double normalizedDistance) {
		return normalizedDistance * this.getNormalizer();
	}
	
	/**
	 * Converts a distance to a normalized distance within this planning
	 * continuum.
	 * 
	 * @param distance the distance
	 * 
	 * @return the normalized distance
	 * 
	 * @see Environment#toNormalizedDistance(double)
	 */
	@Override
	public double toNormalizedDistance(double distance) {
		return distance / this.getNormalizer();
	}
	
	/**
	 * Adds a cost interval to this planning continuum.
	 * 
	 * @param costInterval the cost interval to be added
	 * 
	 * @see Environment#addCostInterval(CostInterval)
	 */
	@Override
	public void addCostInterval(CostInterval costInterval) {
		this.costIntervals.add(costInterval);
		this.update();
	}

	/**
	 * Removes a cost interval from this planning continuum.
	 * 
	 * @param costInterval the cost interval to be removed
	 * 
	 * @see Environment#removeCostInterval(CostInterval)
	 */
	@Override
	public void removeCostInterval(CostInterval costInterval) {
		this.costIntervals.remove(costInterval);
		this.update();
	}

	/**
	 * Gets the (overlapping) cost intervals at a specified time instant.
	 * 
	 * @param time the time instant
	 * 
	 * @return the cost intervals at the specified time instant
	 * 
	 * @see Environment#getCostIntervals(ZonedDateTime)
	 */
	@Override
	public List<Interval<ChronoZonedDateTime<?>>> getCostIntervals(ZonedDateTime time) {
		return this.costIntervals.searchInterval(new CostInterval(null, time));
	}

	/**
	 * Gets the (overlapping) cost intervals within a specified time span.
	 * 
	 * @param start the start time of the time span
	 * @param end the end time of the time span
	 * 
	 * @return the cost intervals within the specified time interval
	 * 
	 * @see Environment#getCostIntervals(ZonedDateTime, ZonedDateTime)
	 */
	@Override
	public List<Interval<ChronoZonedDateTime<?>>> getCostIntervals(ZonedDateTime start, ZonedDateTime end) {
		return this.costIntervals.searchInterval(new CostInterval(null, start, end));
	}
	
	/**
	 * Gets the base cost of a normalized step in this planning continuum.
	 * 
	 * @return the base cost of a normalized step in this planning continuum
	 * 
	 * @see Environment#getBaseCost()
	 */
	@Override
	public double getBaseCost() {
		return PlanningContinuum.BASE_COST;
	}
	
	/**
	 * Gets the accumulated cost of this planning continuum at a specified time
	 * instant.
	 * 
	 * @param time the time instant
	 * 
	 * @return the accumulated cost of this planning continuum at the specified
	 *         time instant
	 * 
	 * @see Environment#getCost(ZonedDateTime)
	 */
	@Override
	public double getCost(ZonedDateTime time) {
		return this.getCost(time, time);
	}
	
	/**
	 * Gets the accumulated cost of this planning continuum within a specified
	 * time span.
	 * 
	 * @param start the start time of the time span
	 * @param end the end time of the time span
	 * 
	 * @return the accumulated cost of this planning continuum within the
	 *         specified time span
	 * 
	 * @see Environment#getCost(ZonedDateTime, ZonedDateTime)
	 */
	@Override
	public double getCost(ZonedDateTime start, ZonedDateTime end) {
		double cost = this.getBaseCost(); // simple cost of normalized distance

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
	 * Gets the step cost from an origin to a destination position within this
	 * planning continuum between a start and an end time given a cost policy
	 * and risk policy.
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
	 * @throws IllegalStateException if no edge exists connecting both positions
	 * 
	 * @see Environment#getStepCost(Position, Position, ZonedDateTime, ZonedDateTime, CostPolicy, RiskPolicy)
	 */
	@Override
	public double getStepCost(
			Position origin, Position destination,
			ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {
		
		double stepCost = Double.POSITIVE_INFINITY;
		Optional<Edge> edge = this.findEdge(origin, destination);
		
		if (edge.isPresent()) {
			double distance = this.getNormalizedDistance(origin, destination);
			// TODO: consider environment air-data intervals
			double cost = edge.get().calculateCost(start, end, costPolicy, riskPolicy);
			stepCost = distance * cost;
		} else {
			throw new IllegalStateException("no edge containing both positions");
		}
		
		return stepCost;
	}
	
	/**
	 * Gets the leg cost from an origin to a destination position within this
	 * planning continuum between a start and an end time given a cost policy
	 * and risk policy.
	 * 
	 * @param origin the origin position in globe coordinates
	 * @param destination the destination position in globe coordinates
	 * @param start the start time
	 * @param end the end time
	 * @param costPolicy the cost policy
	 * @param riskPolicy the risk policy
	 * 
	 * @return the leg cost from the origin to the destination position
	 * 
	 * @see Environment#getLegCost(Position, Position, ZonedDateTime, ZonedDateTime, CostPolicy, RiskPolicy)
	 */
	@Override
	public double getLegCost(
			Position origin, Position destination,
			ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {
		
		double legCost = Double.POSITIVE_INFINITY;
		
		Edge leg = new Edge(this, origin, destination);
		double distance = this.getNormalizedDistance(origin, destination);
		double cost = leg.calculateCost(start, end, costPolicy, riskPolicy);
		legCost = distance * cost;
			
		return legCost;
	}
	
	/**
	 * Determines whether or not a straight leg of two positions collides with
	 * terrain of the globe of this planning continuum.
	 * 
	 * @param origin the origin position in globe coordinates
	 * @param destination the destination position in globe coordinates
	 * 
	 * @return true if the straight leg collides with terrain, false otherwise
	 * 
	 * @see Environment#collidesTerrain(Position, Position)
	 */
	@Override
	public boolean collidesTerrain(Position origin, Position destination) {
		boolean collidesTerrain = true;
		
		if (!this.isInsideGlobe(origin) && !this.isInsideGlobe(destination)) {
			HighResolutionTerrain terrain = new HighResolutionTerrain(
					this.getGlobe(), this.getResolution());
			// TODO: check position altitudes (ASL versus AGL)
			// TODO: include safe height and distance
			// TODO: include buildings (man-made obstacles)
			collidesTerrain = (null != terrain.intersect(origin, destination));
		}
		
		return collidesTerrain;
	}
	
	/**
	 * Embeds an obstacle into this planning continuum.
	 * 
	 * @param obstacle the obstacle to be embedded
	 * 
	 * @return true if the obstacle has been embedded, false otherwise
	 * 
	 * @throws IllegalStateException if this planning continuum has no globe
	 * 
	 * @see DynamicEnvironment#embed(Obstacle)
	 */
	@Override
	public boolean embed(Obstacle obstacle) {
		boolean embedded = false;
		
		if (this.hasGlobe()) {
			if (!this.isEmbedded(obstacle) && this.intersects(obstacle.getExtent(this.globe))) {
				this.addCostInterval(obstacle.getCostInterval());
				this.obstacles.add(obstacle);
				this.attachToEdges(obstacle);
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
	 * @see DynamicEnvironment#unembed(Obstacle)
	 */
	@Override
	public boolean unembed(Obstacle obstacle) {
		boolean unembedded = false;
		
		if (this.isEmbedded(obstacle)) {
			this.removeCostInterval(obstacle.getCostInterval());
			this.obstacles.remove(obstacle);
			this.detachFromEdges(obstacle);
			unembedded = true;
		}
		
		return unembedded;
	}
	
	/**
	 * Unembeds all obstacles from this planning continuum.
	 * 
	 * @see DynamicEnvironment#unembedAll()
	 */
	@Override
	public void unembedAll() {
		Iterator<Obstacle> obstaclesIterator = this.obstacles.iterator();
		while (obstaclesIterator.hasNext()) {
			Obstacle obstacle = obstaclesIterator.next();
			this.removeCostInterval(obstacle.getCostInterval());
			this.detachFromEdges(obstacle);
			obstaclesIterator.remove();
		}
	}
	
	/**
	 * Determines whether or not an obstacle is embedded in this planning
	 * continuum.
	 * 
	 * @param obstacle the obstacle
	 * 
	 * @return true if the obstacle is embedded in this planning continuum,
	 *         false otherwise
	 * 
	 * @see DynamicEnvironment#isEmbedded(Obstacle)
	 */
	@Override
	public boolean isEmbedded(Obstacle obstacle) {
		return this.obstacles.contains(obstacle);
	}
	
	/**
	 * Updates this planning continuum for an embedded obstacle.
	 * 
	 * @param obstacle the embedded obstacle
	 * 
	 * @see DynamicEnvironment#refresh(Obstacle)
	 */
	@Override
	public void refresh(Obstacle obstacle) {
		if (this.obstacles.contains(obstacle)) {
			this.update();
		}
	}
	
	/**
	 * Gets the waypoint positions of this planning continuum that are
	 * affected by an obstacle.
	 * 
	 * @param obstacle the obstacle
	 * @return the waypoint positions of this planning continuum that are
	 *         affected by the obstacle
	 * 
	 * @see DynamicEnvironment#getAffectedWaypointPositions(Set)
	 */
	@Override
	public Set<Position> getAffectedWaypointPositions(Obstacle obstacle) {
		Set<Position> affectedWaypointPositions = new HashSet<Position>();
		
		for (Edge edge : this.findAffectedEdges(obstacle)) {
			affectedWaypointPositions.add(edge.getFirstPosition());
			affectedWaypointPositions.add(edge.getSecondPosition());
		}
		
		return affectedWaypointPositions;
	}
	
	/**
	 * Gets the waypoint positions of this planning continuum that are
	 * affected by obstacles.
	 * 
	 * @param obstacles the obstacles
	 * @return the waypoint positions of this planning continuum that are
	 *         affected by the obstacles
	 *
	 * @see DynamicEnvironment#getAffectedWaypointPositions(Set)
	 */
	@Override
	public Set<Position> getAffectedWaypointPositions(Set<Obstacle> obstacles) {
		Set<Position> affectedWaypointPositions = new HashSet<Position>();
		
		for (Obstacle obstacle : obstacles) {
			affectedWaypointPositions.addAll(this.getAffectedWaypointPositions(obstacle));
		}
		
		return affectedWaypointPositions;
	}
	
	/**
	 * Adds a structural change listener to this planning continuum.
	 * 
	 * @param listener the structural change listener to be added
	 * 
	 * @see StructuredEnvironment#addStructuralChangeListener(StructuralChangeListener)
	 */
	@Override
	public void addStructuralChangeListener(StructuralChangeListener listener) {
		this.listeners.add(listener);
	}
	
	/**
	 * Removes a structural change listener from this planning continuum.
	 * 
	 * @param listener the structural change listener to be removed
	 * 
	 * @see StructuredEnvironment#removeStructuralChangeListener(StructuralChangeListener)
	 */
	@Override
	public void removeStructuralChangeListener(StructuralChangeListener listener) {
		this.listeners.remove(listener);
	}
	
	/**
	 * Notifies the structural change listeners of this planning continuum.
	 * 
	 * @see StructuredEnvironment#notifyStructuralChangeListeners()
	 */
	@Override
	public void notifyStructuralChangeListeners() {
		for (StructuralChangeListener listener : this.listeners) {
			listener.notifyStructuralChange();
		}
	}
	
	/**
	 * Gets the resolution of this planning continuum.
	 * 
	 * @return the resolution of this planning continuum in meters,
	 *         0 if infinite
	 * 
	 * @see MultiResolutionEnvironment#getResolution()
	 */
	@Override
	public double getResolution() {
		return this.resolution;
	}
	
	/**
	 * Sets the resolution of this planning continuum.
	 * 
	 * @param resolution the resolution to be set in meters
	 * 
	 * @throws IllegalArgumentException if the resolution is less than 0 or
	 *         greater than the diameter of this planning continuum
	 * 
	 * @see MultiResolutionEnvironment#setResolution(double)
	 */
	@Override
	public void setResolution(double resolution) {
		if (0d >= resolution || this.getDiameter() < resolution) {
			throw new IllegalArgumentException("invalid resolution");
		} else {
			this.resolution = resolution;
		}
	}
	
	/**
	 * Refines this planning continuum by a refinement factor.
	 * 
	 * @param factor the refinement factor
	 * 
	 * @throws IllegalArgumentException if the refinement factor is not positive
	 * 
	 * @see MultiResolutionEnvironment#refine(int)
	 */
	@Override
	public void refine(int factor) {
		if (1 > factor) {
			throw new IllegalArgumentException("invalid refinement factor");
		} else {
			this.resolution /= factor;
		}
	}
	
	/**
	 * Coarsens this planning continuum by a coarsening factor.
	 * 
	 * @param factor the coarsening factor
	 * 
	 * @throws IllegalArgumentException if the coarsening factor is not positive
	 * 
	 * @see MultiResolutionEnvironment#coarsen(int)
	 */
	@Override
	public void coarsen(int factor) {
		if (1 > factor) {
			throw new IllegalArgumentException("invalid coarsening factor");
		} else {
			this.resolution *= factor;
			if (this.getDiameter() < this.resolution) {
				this.resolution = this.getDiameter();
			}
		}
	}
	
	/**
	 * Determines whether or not this planning continuum is refined.
	 * 
	 * @return true if this planning continuum is refined, false otherwise
	 * 
	 * @see MultiResolutionEnvironment#isRefined()
	 */
	@Override
	public boolean isRefined() {
		return (this.getDiameter() > this.resolution);
	}
	
	/**
	 * Samples a random position from within this planning continuum using a
	 * uniform distribution.
	 * 
	 * @return the sampled position from within this planning continuum using
	 *         the uniform distribution in globe coordinates
	 * 
	 * @see SamplingEnvironment#sampleRandomUniformPosition()
	 */
	@Override
	public Position sampleRandomUniformPosition() {
		Vec4[] corners = this.getCorners();
		
		// point within the box frame with all minimum coordinates
		Vec4 minimum = this.transformModelToBoxOrigin(corners[0]);
		// point within the box frame with all maximum coordinates
		Vec4 maximum = this.transformModelToBoxOrigin(corners[6]);
		
		double x, y, z;
		
		x = minimum.x + (new Random().nextDouble() * (maximum.x - minimum.x));
		y = minimum.y + (new Random().nextDouble() * (maximum.y - minimum.y));
		z = minimum.z + (new Random().nextDouble() * (maximum.z - minimum.z));
		
		Vec4 point = new Vec4(x, y, z);
		
		// transform point from box frame to earth frame
		point = this.transformBoxOriginToModel(point);
		return this.getGlobe().computePositionFromPoint(point);
	}
	
	/**
	 * Samples a random position from within this planning continuum using a
	 * Gaussian (normal) distribution.
	 * 
	 * @return the sampled position from within this planning continuum using
	 *         the Gaussian (normal) distribution in globe coordinates
	 * 
	 * @see SamplingEnvironment#sampleRandomGaussianPosition()
	 */
	@Override
	public Position sampleRandomGaussianPosition() {
		Vec4[] corners = this.getCorners();
		
		// point within the box frame with all minimum coordinates
		Vec4 minimum = this.transformModelToBoxOrigin(corners[0]);
		// point within the box frame with all maximum coordinates
		Vec4 maximum = this.transformModelToBoxOrigin(corners[6]);
		
		double xMean, yMean, zMean, xSD, ySD, zSD, x, y, z;
		
		Random r = new Random();
		
		xMean = (minimum.x + maximum.x) / 2;
		yMean = (minimum.y + maximum.y) / 2;
		zMean = (minimum.z + maximum.z) / 2;
		
		xSD = (maximum.x - xMean) / 2;
		ySD = (maximum.y - yMean) / 2;
		zSD = (maximum.z - zMean) / 2;
		x = r.nextGaussian() * xSD + xMean;
		y = r.nextGaussian() * ySD + yMean;
		z = r.nextGaussian() * zSD + zMean;
		
		Vec4 point = new Vec4(x, y, z);
		
		// transform point from box frame to earth frame
		point = this.transformBoxOriginToModel(point);
		return this.getGlobe().computePositionFromPoint(point);
	}
	
	/**
	 * Samples a random position from within the intersection of this planning
	 * continuum and a default sized ellipsoid defined by two foci positions.
	 * 
	 * @param focusA the focus A of the ellipsoid in globe coordinates
	 * @param focusB the focus B of the ellipsoid in globe coordinates
	 * 
	 * @return the sampled position from within the intersection of this
	 *         planning continuum and the ellipsoid
	 * 
	 * @see SamplingEnvironment#sampleRandomEllipsoidPosition(Position, Position)
	 */
	@Override
	public Position sampleRandomEllipsoidPosition(
			Position focusA, Position focusB) {
		
		// extend minor diameter by 10% focus distance
		double diameter = this.getDistance(focusA, focusB)
				+ (0.1d * this.getDistance(focusA, focusB));
		
		// ellipsoid axis
		Vec4 va = this.getGlobe().computePointFromPosition(focusA);
		Vec4 vb = this.getGlobe().computePointFromPosition(focusB);
		Vec4 vc = vb.subtract3(va);
		
		// ellipsoid radii
		double a = diameter / 2d; // minor radius
		double c = vc.divide3(2d).getLength3(); // foci (vertical) radius
		double b = Math.sqrt((a * a) + (c * c)); // major radius
		
		return this.sampleRandomEllipsoidPosition(focusA, focusB, a, b);
	}
	
	/**
	 * Samples a random position from within the intersection of this 
	 * planning continuum and an ellipsoid defined by two foci positions
	 * and two ellipsoid radii.
	 * 
	 * @param focusA the focus A of the ellipsoid in globe coordinates
	 * @param focusB the focus B of the ellipsoid in globe coordinates
	 * @param a the minor ellipsoid radius
	 * @param b the major ellipsoid radius
	 * 
	 * @return the sampled position from within the intersection of this
	 *         planning continuum and the ellipsoid
	 * 
	 * @see SamplingEnvironment#sampleRandomEllipsoidPosition(Position, Position, double, double)
	 */
	@Override
	public Position sampleRandomEllipsoidPosition(
			Position focusA, Position focusB, double a, double b) {
		Position sample = null;
		
		// sample entire continuum if more restrictive
		if (a > this.getDiameter()) {
			sample = sampleRandomUniformPosition();
		} else {
			// ellipsoid axis and center
			Vec4 va = this.getGlobe().computePointFromPosition(focusA);
			Vec4 vb = this.getGlobe().computePointFromPosition(focusB);
			Vec4 vc = vb.subtract3(va);
			Vec4 center = va.add3(vc.divide3(2d));
			
			// ellipsoid radii
			double c = vc.divide3(2d).getLength3(); // foci (vertical) radius
			
			do {
				// sample unit sphere and extend to ellipsoid
				double latitude = (new Random().nextDouble() * 2d * Math.PI);
				double longitude = (new Random().nextDouble() * 2d * Math.PI);
				double radius = Math.sqrt(new Random().nextDouble());
				PolarPoint rup = PolarPoint.fromRadians(latitude, longitude, radius);
				Vec4 ruc = rup.toCartesian();
				Vec4 rc = new Vec4(ruc.x * a, ruc.y * b, ruc.z * c);
				
				// transform sample into ellipsoid
				Vec4[] alignmentAxis = new Vec4[] {Vec4.ZERO};
				Angle alignmentAngle = Vec4.axisAngle(vc, Vec4.UNIT_Y, alignmentAxis);
				Quaternion alignmentRotation = Quaternion.fromAxisAngle(alignmentAngle, alignmentAxis[0]);
				rc = rc.transformBy3(alignmentRotation);
				sample = this.getGlobe().computePositionFromPoint(center.add3(rc));
			
				// continue until sample is within intersection
			} while (!this.contains(sample));
		}
		
		return sample;
	}
	
	/**
	 * Gets the optimal number of sampled neighbors to be considered for a
	 * connection to a new sample in this planning continuum.
	 * 
	 * @return the optimal number of sampled neighbors to be considered for a
	 *         connection to a new sample in this planning continuum
	 * 
	 * @see SamplingEnvironment#getOptimalNumNearest()
	 */
	@Override
	public int getOptimalNumNearest() {
		return (int) Math.round(2 * Math.E * Math.log(this.vertices.size()));
	}
	
	/**
	 * Finds the k-nearest sampled positions for a given position in this
	 * planning continuum.
	 * 
	 * @param position the position to query in globe coordinates
	 * @param k the maximum number of sampled positions to be found
	 * 
	 * @return the k-nearest sampled positions for the given position in this
	 *         planning continuum sorted by increasing distance
	 * 
	 * @see SamplingEnvironment#findNearest(Position, int)
	 */
	@Override
	public Set<? extends Position> findNearest(Position position, int k) {
		return this.vertices.stream()
				.sorted((p1, p2) -> Double.compare(
						this.getNormalizedDistance(p1, position),
						this.getNormalizedDistance(p2, position)))
				.filter(p -> !p.equals(position))
				.limit(k)
				.collect(Collectors.toSet());
	}
	
	/**
	 * Finds the k-nearest sampled positions closer than a certain distance
	 * from a given position in this planning continuum.
	 * 
	 * @param position the position in global coordinates
	 * @param k the maximum number of sampled positions to be found
	 * @param distance the maximum distance from the position
	 * 
	 * @return the k-nearest sampled positions closer than the distance from
	 *         the position in this planning continuum sorted by increasing
	 *         distance
	 * 
	 * @see SamplingEnvironment#findNearest(Position, int, double)
	 */
	@Override
	public Set<? extends Position> findNearest(
			Position position, int k, double distance) {
		return this.vertices.stream()
				.sorted((p1, p2) -> Double.compare(
						this.getNormalizedDistance(p1, position),
						this.getNormalizedDistance(p2, position)))
				.filter(p -> (this.getDistance(p, position) <= distance)
						&& !p.equals(position))
				.limit(k)
				.collect(Collectors.toSet());
	}
	
	/**
	 * Finds the k-nearest sampled positions for a given position in this
	 * planning continuum using a particular distance metric.
	 * 
	 * @param position the position in global coordinates
	 * @param k the number of sampled positions to be found
	 * @param metric the distance metric to be applied
	 * 
	 * @return the k-nearest sampled positions for the given position in this
	 *         planning continuum using the distance metric, sorted by
	 *         increasing distance
	 * 
	 * @see SamplingEnvironment#findNearest(Position, int, BiFunction)
	 */
	@Override
	public Set<? extends Position> findNearest(
			Position position, int k,
			BiFunction<Position, Position, Double> metric) {
		return this.vertices.stream()
				.sorted((p1, p2) -> Double.compare(
						metric.apply(p1, position),
						metric.apply(p2, position)))
				.filter(p -> !p.equals(position))
				.limit(k)
				.collect(Collectors.toSet());
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
	@Override
	public void setTime(ZonedDateTime time) {
		this.time = time;
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
	 * Gets the sampling shape within this planning continuum.
	 * 
	 * @return the sampling shape within this planning continuum
	 */
	public RigidShape getSamplingShape() {
		return this.samplingShape;
	}
	
	/**
	 * Sets the sampling shape within this planning continuum.
	 * 
	 * @param samplingShape the sampling shape to be set
	 */
	public void setSamplingShape(RigidShape samplingShape) {
		this.samplingShape = samplingShape;
	}
	
	/**
	 * Determines whether or not this planning continuum has a sampling shape.
	 * 
	 * @return true if this planning planning continuum has a sampling shape,
	 *         false otherwise
	 */
	public boolean hasSamplingShape() {
		return (null != this.samplingShape);
	}
	
	/**
	 * Sets a box sampling shape within this planning continuum.
	 */
	public void setBoxShape() {
		Position cp = this.getCenterPosition();
		double a = 0d, b = 0d, c = 0d;
		Angle azimuth = Angle.ZERO;
		
		// determine vertical axis
		Vec4 normal = this.getGlobe().computeSurfaceNormalAtPoint(this.getCenter());
		double rns = normal.angleBetween3(this.getRAxis()).sin();
		double sns = normal.angleBetween3(this.getSAxis()).sin();
		double tns = normal.angleBetween3(this.getTAxis()).sin();
		
		// box orientation
		if (rns < Double.min(sns, tns)) {
			// r-axis is vertical
			a = this.getSLength() / 2d;
			b = this.getRLength() / 2d;
			c = this.getTLength() / 2d;
			Vec4 sv = this.getCenter().add3(this.getSAxis());
			Position sp = this.getGlobe().computePositionFromPoint(sv);
			azimuth = Position.greatCircleAzimuth(cp, sp);
		} else if (sns < Double.min(rns, tns)) {
			// s-axis is vertical
			a = this.getRLength() / 2d;
			b = this.getSLength() / 2d;
			c = this.getTLength() / 2d;
			Vec4 rv = this.getCenter().add3(this.getRAxis());
			Position rp = this.getGlobe().computePositionFromPoint(rv);
			azimuth = Position.greatCircleAzimuth(cp, rp);
		} else if (tns < Double.min(rns, sns)) {
			// t-axis is vertical
			a = this.getRLength() / 2d;
			b = this.getTLength() / 2d;
			c = this.getSLength() / 2d;
			Vec4 rv = this.getCenter().add3(this.getRAxis());
			Position rp = this.getGlobe().computePositionFromPoint(rv);
			azimuth = Position.greatCircleAzimuth(cp, rp);
		}
		
		// box shape and attributes
		gov.nasa.worldwind.render.Box samplingBox =
				new gov.nasa.worldwind.render.Box(cp, a, b, c,
				azimuth, Angle.ZERO, Angle.ZERO);
					
		BasicShapeAttributes attributes = new BasicShapeAttributes();
		attributes.setDrawOutline(false);
		attributes.setInteriorMaterial(Material.LIGHT_GRAY);
		attributes.setInteriorOpacity(0.2d);
		samplingBox.setAttributes(attributes);
		this.setSamplingShape(samplingBox);
	}
	
	/**
	 * Sets an default sized ellipsoid sampling shape within this planning
	 * continuum.
	 * 
	 * @param focusA the focus A of the ellipsoid in globe coordinates
	 * @param focusB the focus B of the ellipsoid in globe coordinates
	 */
	public void setEllipsoidShape(Position focusA, Position focusB) {
		Vec4 va = this.getGlobe().computePointFromPosition(focusA);
		Vec4 vb = this.getGlobe().computePointFromPosition(focusB);
		Vec4 vc = vb.subtract3(va);
		Vec4 center = va.add3(vc.divide3(2d));
		Position cp = this.getGlobe().computePositionFromPoint(center);
		
		// extend minor diameter by 10% focus distance
		double diameter = this.getDistance(focusA, focusB);
		diameter += 0.1d * diameter;
		
		// ellipsoid radii
		double a = diameter / 2d; // minor radius
		double c = vc.divide3(2d).getLength3(); // foci (vertical) radius
		double b = Math.sqrt((a * a) + (c * c)); // major radius
		
		// ellipsoid orientation
		Angle azimuth = Position.greatCircleAzimuth(focusA, focusB);
		Vec4 normal = this.getGlobe().computeSurfaceNormalAtPoint(center);
		Angle pitch = normal.angleBetween3(vc).subtract(Angle.POS90);
		if (0 < focusA.getLatitude().compareTo(focusB.getLatitude())) {
			pitch = pitch.multiply(-1d);
		}
		
		// ellipsoid shape and attributes
		Ellipsoid samplingEllipsoid = new Ellipsoid(cp, b, c, a,
				azimuth, pitch, Angle.ZERO);
					
		BasicShapeAttributes attributes = new BasicShapeAttributes();
		attributes.setDrawOutline(false);
		attributes.setInteriorMaterial(Material.LIGHT_GRAY);
		attributes.setInteriorOpacity(0.2d);
		samplingEllipsoid.setAttributes(attributes);
		this.setSamplingShape(samplingEllipsoid);
	}
	
	/**
	 * Gets the obstacles of this planning continuum.
	 * 
	 * @return the obstacles of this planning continuum
	 */
	protected Set<Obstacle> getObstacles() {
		return Collections.unmodifiableSet(this.obstacles);
	}
	
	/**
	 * Adds a vertex to this planning continuum.
	 * 
	 * @param vertex the vertex to be added
	 */
	public void addVertex(Position vertex) {
		this.vertices.add(vertex);
	}
	
	/**
	 * Removes a vertex from this planning continuum while implicitly removing
	 * the referencing edges to maintain structural integrity.
	 * 
	 * @param vertex the vertex to be removed
	 */
	public void removeVertex(Position vertex) {
		this.vertices.remove(vertex);
		// implicitly remove edges to maintain structural integrity
		this.removeEdges(this.edges.stream()
				.filter(e -> e.isEndPosition(vertex))
				.collect(Collectors.toSet()));
	}
	
	/**
	 * Clears the vertices of this planning continuum while implicitly removing
	 * all edges to maintain structural integrity.
	 */
	public void clearVertices() {
		this.vertices.clear();
		// implicitly clear edges to maintain structural integrity
		this.clearEdges();
	}
	
	/**
	 * Gets the number of vertices of this planning continuum.
	 * 
	 * @return the number of vertices of this planning continuum
	 */
	public int getNumVertices() {
		return this.vertices.size();
	}
	
	/**
	 * Finds a particular vertex of this planning continuum.
	 * 
	 * @param vertex the vertex to be found
	 * 
	 * @return the vertex of this planning continuum if found
	 */
	public Optional<? extends Position> findVertex(Position vertex) {
		return this.vertices.stream()
				.filter(s -> s.equals(vertex))
				.findFirst();
	}
	
	/**
	 * Gets the vertices of this planning continuum.
	 * 
	 * @return the vertices of this planning continuum
	 */
	public Iterable<Position> getVertices() {
		return this.vertices;
	}
	
	/**
	 * Adds an edge to this planning continuum while implicitly adding the end
	 * positions as vertices if necessary to maintain structural integrity.
	 * 
	 * @param edge the edge to be added
	 */
	public void addEdge(Edge edge) {
		if (this.edges.add(edge)) {
			// implicitly add vertices to maintain structural integrity
			this.addVertex(edge.getFirstPosition());
			this.addVertex(edge.getSecondPosition());
			this.notifyStructuralChangeListeners();
		}
	}
	
	/**
	 * Adds an edge to this planning continuum while implicitly adding the end
	 * positions as vertices if necessary to maintain structural integrity.
	 * 
	 * @param first the first end position of the edge
	 * @param second the second end position of the edge
	 */
	public void addEdge(Position first, Position second) {
		if (!first.equals(second) && this.edges.add(new Edge(this, first, second))) {
			// implicitly add vertices to maintain structural integrity
			this.addVertex(first);
			this.addVertex(second);
			this.notifyStructuralChangeListeners();
		}
	}
	
	/**
	 * Removes an edge from this planning continuum.
	 * 
	 * @param edge the edge to be removed
	 */
	public void removeEdge(Edge edge) {
		if (this.edges.remove(edge)) {
			this.notifyStructuralChangeListeners();
		}
	}
	
	/**
	 * Removes an edge from this planning continuum.
	 * 
	 * @param first the first end position of the edge
	 * @param second the second end position of the edge
	 */
	public void removeEdge(Position first, Position second) {
		if (!first.equals(second) && this.edges.remove(new Edge(this, first, second))) {
			this.notifyStructuralChangeListeners();
		}
	}
	
	/**
	 * Removes edges from this planning continuum.
	 * 
	 * @param edges the edges to be removed
	 */
	public void removeEdges(Collection<Edge> edges) {
		if (this.edges.removeAll(edges)) {
			this.notifyStructuralChangeListeners();
		}
	}
	
	/**
	 * Removes all edges from this planning continuum.
	 */
	public void clearEdges() {
		this.edges.clear();
		this.notifyStructuralChangeListeners();
	}
	
	/**
	 * Gets the number of edges of this planning continuum.
	 * 
	 * @return the number of edges of this planning continuum
	 */
	public int getNumEdges() {
		return this.edges.size();
	}
	
	/**
	 * Finds a particular edge of this planning continuum.
	 * 
	 * @param end1 one end position of the edge to be found
	 * @param end2 another end position of the edge to be found
	 * 
	 * @return the edge of this planning continuum if found
	 */
	public Optional<Edge> findEdge(Position end1, Position end2) {
		return this.edges.stream()
				.filter(s -> s.isEndPosition(end1)
						&& s.isEndPosition(end2))
				.findFirst();
	}
	
	/**
	 * Gets the edges of this planning continuum.
	 * 
	 * @return the edges of this planning continuum
	 */
	public Iterable<Edge> getEdges() {
		return this.edges;
	}
	
	/**
	 * Finds the edges of this planning continuum that are affected by an
	 * obstacle embedding.
	 * 
	 * @param obstacle the embedded obstacle
	 * 
	 * @return the edges of this planning continuum that are affected by the
	 *         obstacle embedding
	 */
	public Set<Edge> findAffectedEdges(Obstacle obstacle) {
		return this.edges.stream()
				.filter(e ->  e.intersects(obstacle.getExtent(this.getGlobe())))
				.collect(Collectors.toSet());
	}
	
	/**
	 * Attaches an obstacle to the affected edges of this planning continuum.
	 * 
	 * @param obstacle the obstacle to be attached
	 */
	protected void attachToEdges(Obstacle obstacle) {
		for (Edge edge : this.edges) {
			if (edge.intersects(obstacle.getExtent(this.getGlobe()))) {
				edge.addCostInterval(obstacle.getCostInterval());
			}
		}
	}
	
	/**
	 * Detaches an obstacle from the affected edges of this planning continuum.
	 * 
	 * @param obstacle the obstacle to be detached
	 */
	protected void detachFromEdges(Obstacle obstacle) {
		for (Edge edge : this.edges) {
			if (edge.intersects(obstacle.getExtent(this.getGlobe()))) {
				edge.removeCostInterval(obstacle.getCostInterval());
			}
		}
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
		for (Edge edge : this.edges) {
			edge.updateActiveCost();
		}
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
		for (Edge edge :this.edges) {
			edge.updateAppearance();
		}
	}
	
	/**
	 * Updates the visibility of this planning continuum.
	 */
	protected void updateVisibility() {
		this.setVisible(this.activeCost > this.thresholdCost);
		for (Edge edge : this.edges) {
			edge.updateVisibility();
		}
	}
	
	/**
	 * Renders this planning continuum using a drawing context.
	 * 
	 * @param dc the drawing context
	 */
	@Override
	public void render(DrawContext dc) {
		if (this.visible) {
			super.render(dc);
			if (this.hasSamplingShape()) {
				this.samplingShape.render(dc);
			}
			for (Edge edge : this.edges) {
				edge.render(dc);
			}
		}
	}
	
}
