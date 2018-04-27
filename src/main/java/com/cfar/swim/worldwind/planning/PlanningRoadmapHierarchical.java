/**
 * Copyright (c) 2016, Stephan Heinemann (UVic Center for Aerospace Research)
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
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import com.binarydreamers.trees.Interval;
import com.binarydreamers.trees.IntervalTree;
import com.cfar.swim.worldwind.ai.prm.basicprm.BasicPRM;
import com.cfar.swim.worldwind.ai.prm.lazyprm.LazyPRM;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.geom.ContinuumBox;
import com.cfar.swim.worldwind.geom.CubicGrid;
import com.cfar.swim.worldwind.geom.RegularGrid;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.ObstacleColor;
import com.cfar.swim.worldwind.render.TerrainObstacle;
import com.cfar.swim.worldwind.render.ThresholdRenderable;
import com.cfar.swim.worldwind.render.TimedRenderable;
import com.cfar.swim.worldwind.render.airspaces.ObstacleCylinder;

import gov.nasa.worldwind.geom.Line;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.Polyline;
import gov.nasa.worldwind.util.measure.LengthMeasurer;

/**
 * Realizes a planning roadmap that extends a planning continuum, by
 * incorporating waypoint and edge lists. Can be used for motion planning.
 * 
 * @author Henrique Ferreira
 *
 */
public class PlanningRoadmapHierarchical extends ContinuumBox implements DiscreteEnvironment {

	/** the maximum number of sampling iterations */
	public final int MAX_ITER;

	/** the maximum number of neighbors a waypoint can be connected to */
	public final int MAX_NEIGHBORS;

	/** the maximum distance between two neighboring waypoints */
	public final double MAX_DIST;

	public RoadmapConstructor roadmapConstructor;

	/** the globe of this planning continuum */
	private Globe globe = null;

	/** the cost interval tree encoding temporal costs */
	private IntervalTree<ChronoZonedDateTime<?>> costIntervals = new IntervalTree<ChronoZonedDateTime<?>>(
			CostInterval.comparator);

	/** the current time of this planning continuum */
	private ZonedDateTime time = ZonedDateTime.now(ZoneId.of("UTC"));

	/** the obstacles embedded into this planning continuum */
	private HashSet<Obstacle> obstacles = new HashSet<Obstacle>();

	/** the current accumulated active cost of this planning continuum */
	private double activeCost = 1d;

	/** the threshold cost of this planning continuum */
	private double thresholdCost = 0d;
 
	/** */
	private Edge edge = null;

	/** the parent of this regular grid */
	protected PlanningRoadmapHierarchical parent = null;

	/** the children of this regular grid */
	private Set<PlanningRoadmapHierarchical> children = new HashSet<PlanningRoadmapHierarchical>();

	/** the affected children of obstacle embeddings */
	private HashMap<Obstacle, List<PlanningRoadmapHierarchical>> affectedChildren = new HashMap<Obstacle, List<PlanningRoadmapHierarchical>>();

	/** the resolution of this planning continuum */
	private double resolution = 1d;
	
	/**
	 * Constructs a planning roadmap based on a box, a waypoint list and a edge list
	 * 
	 * @param box the box used to define this environment
	 * @param waypointList the list of waypoints
	 * @param edgeList the list of edges
	 */
	@SuppressWarnings("unchecked")
	public PlanningRoadmapHierarchical(Box box, Globe globe) {
		super(box);
		this.setGlobe(globe);
		MAX_ITER = 1000;
		MAX_NEIGHBORS = 30;
		MAX_DIST = 200d;
	}

	/**
	 * Constructs a planning roadmap based on a box.
	 * 
	 * @param box the box used to define this environment
	 * @param resolution the resolution of this planning continuum
	 */
	public PlanningRoadmapHierarchical(Box box, double resolution, RoadmapConstructor roadmapConstructor, Globe globe, int maxIter,
			int maxNeighbors, double maxDist) {
		super(box);
		this.resolution = resolution;
		this.update();
		this.setGlobe(globe);
		MAX_ITER = maxIter;
		MAX_NEIGHBORS = maxNeighbors;
		MAX_DIST = maxDist;
		this.roadmapConstructor = roadmapConstructor;
		this.constructRoadmap();
	}

	/**
	 * Indicates whether or not this planning grid contains a position.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return true if this planning grid contains the position, false otherwise
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
	 * Gets the eight corner positions in globe coordinates of this planning grid.
	 * 
	 * @return the eight corner positions in globe coordinates of this planning grid
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see Box#getCorners()
	 */
	public Position[] getCornerPositions() {
		Position[] cornerPositions = null;

		if (null != this.globe) {
			cornerPositions = new Position[8];
			Vec4[] corners = this.getCorners();
			for (int index = 0; index < 8; index++) {
				cornerPositions[index] = this.globe.computePositionFromPoint(corners[index]);
			}
		} else {
			throw new IllegalStateException("globe is not set");
		}

		return cornerPositions;
	}

	/**
	 * Indicates whether or not a position in globe coordinates is a corner of this
	 * planning grid considering numerical inaccuracies.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return true if the position is a corner of this planning grid considering
	 *         numerical inaccuracies, false otherwise
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see Box#isCorner(Vec4)
	 */
	public boolean isCorner(Position position) {
		boolean isCorner = false;

		if (null != this.globe) {
			isCorner = this.isCorner(this.globe.computePointFromPosition(position));
		} else {
			throw new IllegalStateException("globe is not set");
		}

		return isCorner;
	}

	/**
	 * Gets the center position in globe coordinates of this planning grid.
	 * 
	 * @return the center position in globe coordinates of this planning grid
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
	 * Indicates whether or not a position in globe coordinates is the center of
	 * this planning grid considering numerical inaccuracies.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return true if the position is the center of this planning grid considering
	 *         numerical inaccuracies, false otherwise
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see Box#isCorner(Vec4)
	 */
	public boolean isCenter(Position position) {
		boolean isCenter = false;

		if (null != this.globe) {
			isCenter = this.isCenter(this.globe.computePointFromPosition(position));
		} else {
			throw new IllegalStateException("globe is not set");
		}

		return isCenter;
	}

	/**
	 * Sets the globe of this planning grid.
	 * 
	 * @param globe the globe of this planning grid
	 * 
	 * @see Environment#setGlobe(Globe)
	 */
	@Override
	public void setGlobe(Globe globe) {
		for (PlanningRoadmapHierarchical grid : this.getAll()) {
			grid.globe = globe;
		}
	}

	/**
	 * Adds cubic-like children with the specified cell length to a child cell of
	 * this regular grid. The actual cell lengths will be adjusted to completely
	 * fill the child cell.
	 * 
	 * @param r the <code>R</code> index of the child cell
	 * @param s the <code>S</code> index of the child cell
	 * @param t the <code>T</code> index of the child cell
	 * @param length the length of a child cell on each axis
	 */
	public void addChildren(PlanningRoadmapHierarchical child) {
		if (null != child) {
			children.add(child);
		}
	}

	/**
	 * Embeds an obstacle cylinder with an associated cost interval into this
	 * planning grid.
	 * 
	 * @param obstacle the obstacle cylinder to be embedded
	 * 
	 * @return true if an embedding took place, false otherwise
	 * 
	 * @throws IllegalStateException if the globe is not set
	 */
	public boolean embed(ObstacleCylinder obstacle) {
		boolean embedded = false;

		if (null != this.globe) {
			if (!this.isEmbedded(obstacle) && this.intersects(obstacle.getExtent(this.globe))) {
				this.addCostInterval(obstacle.getCostInterval());
				this.obstacles.add(obstacle);

				for (PlanningRoadmapHierarchical child : this.getChildren()) {
					if (child.embed(obstacle)) {
						this.addAffectedChild(obstacle, child);
					}
				}

				embedded = true;
			}
		} else {
			throw new IllegalStateException("globe is not set");
		}

		return embedded;
	}

	/**
	 * Embeds an obstacle into this planning grid.
	 * 
	 * @param obstacle the obstacle to be embedded
	 * 
	 * @return true if the obstacle has been embedded, false otherwise
	 * 
	 * @see Environment#embed(Obstacle)
	 */
	public boolean embed(Obstacle obstacle) {
		boolean embedded = false;

		if (null != this.globe) {
			if (!this.isEmbedded(obstacle) && this.intersects(obstacle.getExtent(this.globe))) {
				this.addCostInterval(obstacle.getCostInterval());
				this.obstacles.add(obstacle);

				for (PlanningRoadmapHierarchical child : this.getChildren()) {
					if (child.embed(obstacle)) {
						this.addAffectedChild(obstacle, child);
					}
				}

				embedded = true;
			}
		} else {
			throw new IllegalStateException("globe is not set");
		}

		return embedded;
	}

	/**
	 * Unembeds an obstacle from this planning grid.
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

			if (this.affectedChildren.containsKey(obstacle)) {
				for (PlanningRoadmapHierarchical child : this.affectedChildren.get(obstacle)) {
					child.unembed(obstacle);
				}
				this.affectedChildren.remove(obstacle);
			}

			unembedded = true;
		}

		return unembedded;
	}

	/**
	 * Unembeds all obstacles from this planning grid.
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

			if (this.affectedChildren.containsKey(obstacle)) {
				for (PlanningRoadmapHierarchical child : this.affectedChildren.get(obstacle)) {
					child.unembed(obstacle);
				}
				this.affectedChildren.remove(obstacle);
			}
		}
	}

	/**
	 * Indicates whether or not an obstacle is embedded in this planning grid.
	 * 
	 * @param obstacle the obstacle
	 * 
	 * @return true if the obstacle is embedded in this planning grid, false
	 *         otherwise
	 * 
	 * @see Environment#isEmbedded(Obstacle)
	 */
	@Override
	public boolean isEmbedded(Obstacle obstacle) {
		return this.obstacles.contains(obstacle);
	}

	/**
	 * Adds an affected child to an obstacle embedding.
	 * 
	 * @param obstacle the obstacle of the embedding
	 * @param child the affected child
	 */
	private void addAffectedChild(Obstacle obstacle, PlanningRoadmapHierarchical child) {
		if (this.affectedChildren.containsKey(obstacle)) {
			this.affectedChildren.get(obstacle).add(child);
		} else {
			ArrayList<PlanningRoadmapHierarchical> children = new ArrayList<PlanningRoadmapHierarchical>();
			children.add(child);
			this.affectedChildren.put(obstacle, children);
		}
	}

	/**
	 * Removes all children from this regular grid.
	 */
	public void removeChildren() {
		this.children = null;
	}

	/**
	 * Indicates whether or not this regular grid has children.
	 * 
	 * @return true if this regular grid has children, false otherwise
	 */
	public boolean hasChildren() {
		return (null != this.children);
	}

	/**
	 * Gets the children of this regular grid.
	 * 
	 * @return the children of this regular grid
	 */
	public Set<PlanningRoadmapHierarchical> getChildren() {
		return this.children;
	}

	/**
	 * Indicates whether or not this regular grid has a particular child.
	 * 
	 * @param r the <code>R</code> index of the child cell
	 * @param s the <code>S</code> index of the child cell
	 * @param t the <code>T</code> index of the child cell
	 * 
	 * @return true if this regular grid has the particular child, false otherwise
	 */
	public boolean hasChild(PlanningRoadmapHierarchical child) {
		return (this.hasChildren() && this.children.contains(child));
	}

	/**
	 * Gets a particular child of this regular grid if present.
	 * 
	 * @param r the <code>R</code> index of the child cell
	 * @param s the <code>S</code> index of the child cell
	 * @param t the <code>T</code> index of the child cell
	 * 
	 * @return the particular child of this regular grid if present, null otherwise
	 */
	public PlanningRoadmapHierarchical getChild(PlanningRoadmapHierarchical child) {
		return this.children.stream().filter(t -> t.equals(child)).findFirst().get();
	}

	/**
	 * Indicates whether or not this regular grid has a parent.
	 * 
	 * @return true if this regular grid has a parent, false otherwise
	 */
	public boolean hasParent() {
		return (null != this.parent);
	}

	/**
	 * Gets the parent of this regular grid if present.
	 * 
	 * @return the parent of this regular grid if present, null otherwise
	 */
	public PlanningRoadmapHierarchical getParent() {
		return this.parent;
	}

	/**
	 * Gets all regular grids associated with this regular grid.
	 * 
	 * @return all regular grids associated with this regular grid
	 */
	public Set<PlanningRoadmapHierarchical> getAll() {
		Set<PlanningRoadmapHierarchical> all = new LinkedHashSet<PlanningRoadmapHierarchical>();
		all.add(this);

		if (this.hasChildren()) {
			all.addAll(children);
		}

		return all;
	}

	/**
	 * Gets the globe of this planning grid.
	 * 
	 * @return the globe of this planning grid
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
	 * Gets the interval tree with cost intervals.
	 * 
	 * @return the costIntervals
	 */
	public IntervalTree<ChronoZonedDateTime<?>> getCostIntervals() {
		return costIntervals;
	}

	/**
	 * Sets the interval tree with cost intervals.
	 * 
	 * @param costIntervals the costIntervals to set
	 */
	public void setCostIntervals(IntervalTree<ChronoZonedDateTime<?>> costIntervals) {
		this.costIntervals = costIntervals;
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
	 * Gets the current time of this planning grid.
	 * 
	 * @return the current time of this planning grid
	 * 
	 * @see TimedRenderable#getTime()
	 */
	@Override
	public ZonedDateTime getTime() {
		return this.time;
	}

	/**
	 * Sets the current time of this planning grid.
	 * 
	 * @param time the current time of this planning grid
	 * 
	 * @see TimedRenderable#setTime(ZonedDateTime)
	 */
	@Override
	public void setTime(ZonedDateTime time) {
		for (PlanningRoadmapHierarchical grid : this.getAll()) {
			grid.time = time;
			grid.update();
		}
	}

	/**
	 * Gets the threshold cost of this planning grid.
	 * 
	 * @return the threshold cost of this planning grid
	 * 
	 * @see ThresholdRenderable#setThreshold(double)
	 */
	@Override
	public double getThreshold() {
		return this.thresholdCost;
	}

	/**
	 * Sets the threshold cost of this planning grid.
	 * 
	 * @param thresholdCost the threshold cost of this planning grid
	 * 
	 * @see ThresholdRenderable#setThreshold(double)
	 */
	@Override
	public void setThreshold(double thresholdCost) {
		for (PlanningRoadmapHierarchical grid : this.getAll()) {
			grid.thresholdCost = thresholdCost;
			grid.updateVisibility();
		}
	}

	/**
	 * Updates this planning grid with all its children.
	 */
	public void refresh() {
		for (PlanningRoadmapHierarchical grid : this.getAll()) {
			grid.update();
		}
	}

	/**
	 * Updates this planning grid for an embedded obstacle.
	 * 
	 * @param obstacle the embedded obstacle
	 * 
	 * @see Environment#refresh(Obstacle)
	 */
	@Override
	public void refresh(Obstacle obstacle) {
		if (this.obstacles.contains(obstacle)) {
			this.update();
			if (this.affectedChildren.containsKey(obstacle)) {
				for (PlanningRoadmapHierarchical child : affectedChildren.get(obstacle)) {
					child.refresh(obstacle);
				}
			}
		}
	}

	/**
	 * Updates the accumulated active cost of this planning grid.
	 */
	protected void updateActiveCost() {
		this.activeCost = this.getCost(this.time);
	}

	/**
	 * Updates the visibility of this planning grid.
	 */
	protected void updateVisibility() {
		this.setVisible(this.activeCost > this.thresholdCost);
	}

	/**
	 * Updates the appearance of this planning grid.
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
	 * Gets the neighbors positions of a position in this roadmap.
	 * 
	 * @param position the position in global coordinates
	 * @return the neighbors of the position in this planning roadmap
	 * 
	 * @see com.cfar.swim.worldwind.planning.DiscreteEnvironment#getNeighbors(gov.nasa.worldwind.geom.Position)
	 */
	@Override
	public Set<Position> getNeighbors(Position position) {
		
		return null;
//		return super.getNeighbors(position);
	}
	
	/**
	 * Looks up the planning grid cells (maximum eight) containing a specified
	 * position in globe coordinates considering numerical inaccuracies.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return the planning grid cells containing the specified position
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see CubicGrid#lookupCells(Vec4)
	 */
	@SuppressWarnings("unchecked")
	public Set<? extends PlanningGrid> lookupCells(Position position) {
		Set<PlanningGrid> cells = null;

		if (null != this.globe) {
//			cells = (Set<PlanningGrid>) super.lookupCells(this.globe.computePointFromPosition(position));
		} else {
			throw new IllegalStateException("globe is not set");
		}

		return cells;
	}

	/**
	 * Gets the step cost from an origin to a destination position within this
	 * planning grid between a start and an end time given a cost policy and risk
	 * policy.
	 * 
	 * @param origin the origin position in globe coordinates
	 * @param destination the destination position in globe coordinates
	 * @param start the start time
	 * @param end the end time
	 * @param costPolicy the cost policy
	 * @param riskPolicy the risk policy
	 * 
	 * @return the step cost from the origin to the destination position
	 */
	public double getStepCost(Position origin, Position destination, ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {

		double stepCost = 0d;

		// compute participating cells
		Set<? extends PlanningGrid> segmentCells = this.lookupCells(origin);
		segmentCells.retainAll(this.lookupCells(destination));

		// an invalid step results in infinite costs
		if (segmentCells.isEmpty()) {
			return Double.POSITIVE_INFINITY;
		}

		List<Double> costs = new ArrayList<Double>();

		// compute initial distance cost
		// explicit distance cost computation is required if neighboring
		// cells are of different size (different level in the hierarchy)
		double distance = this.getNormalizedDistance(origin, destination);

		// compute cost of each adjacent cell
		for (PlanningGrid segmentCell : segmentCells) {
			// add all (weighted) cost of the cell
			double cellCost = segmentCell.getCost(start, end);
			// boost cell cost if local risk is not acceptable
			if (riskPolicy.satisfies(cellCost - 1)) {
				costs.add(distance * cellCost);
			} else {
				costs.add(Double.POSITIVE_INFINITY);
			}
		}

		// apply cost policy for final cost
		switch (costPolicy) {
		case MINIMUM:
			stepCost = costs.stream().mapToDouble(Double::doubleValue).min().getAsDouble();
			break;
		case MAXIMUM:
			stepCost = costs.stream().mapToDouble(Double::doubleValue).max().getAsDouble();
			break;
		case AVERAGE:
			stepCost = costs.stream().mapToDouble(Double::doubleValue).average().getAsDouble();
			break;
		}

		return stepCost;
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
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#getNeighbors()
	 */
	@Override
	public Set<? extends Environment> getNeighbors() {
		Set<PlanningRoadmapHierarchical> neighbors = new HashSet<PlanningRoadmapHierarchical>();
		
		if (null != this.globe) {
			neighbors = this.getParent().getChildren().stream()
					.filter(s -> s.edge.contains(this.edge.getPosition1()) || s.edge.contains(this.edge.getPosition2()))
					.collect(Collectors.toSet());
		} else {
			throw new IllegalStateException("globe is not set");
		}

		return neighbors;

	}
	
	/**
	 * @param neighbor
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#areNeighbors(com.cfar.swim.worldwind.planning.Environment)
	 */
	@Override
	public boolean areNeighbors(Environment neighbor) {
		if (null != this.globe) {
			return this.getNeighbors().contains(neighbor);
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}

	/**
	 * @param position
	 * @param neighbor
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#areNeighbors(gov.nasa.worldwind.geom.Position,
	 *      gov.nasa.worldwind.geom.Position)
	 */
	@Override
	public boolean areNeighbors(Position position, Position neighbor) {
		// TODO Auto-generated method stub
		return false;
	}

	/**
	 * Indicates whether or not this planning grid is refined, that is, has
	 * children.
	 * 
	 * @return true if this planning grid is refined, false otherwise
	 * 
	 */
	public boolean isRefined() {
		return this.hasChildren();
	}

	/**
	 * Gets the refinements, that is, children of this planning grid.
	 * 
	 * @return the refinements of this planning grid
	 * 
	 */
	public Set<PlanningRoadmapHierarchical> getRefinements() {
		return this.getChildren();
	}

	/**
	 * Refines, that is, adds children with a specified density to this planning
	 * grid.
	 * 
	 * @param density the refinement density
	 * 
	 */
	public void refine(int density) {
		// this.addChildren(density);
	}

	/**
	 * Coarsens, that is, removes the children of this planning grid.
	 *
	 */
	public void coarsen() {
		this.removeChildren();
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
	 * @param origin
	 * @param destination
	 * @param start
	 * @param end
	 * @param costPolicy
	 * @param riskPolicy
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#getLegCost(gov.nasa.worldwind.geom.Position,
	 *      gov.nasa.worldwind.geom.Position, java.time.ZonedDateTime,
	 *      java.time.ZonedDateTime, com.cfar.swim.worldwind.planning.CostPolicy,
	 *      com.cfar.swim.worldwind.planning.RiskPolicy)
	 */
	@Override
	public double getLegCost(Position origin, Position destination, ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {
		// TODO Auto-generated method stub
		return 0;
	}

	/**
	 * @param destination
	 * @param start
	 * @param end
	 * @param costPolicy
	 * @param riskPolicy
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.Environment#getLegCost(com.cfar.swim.worldwind.planning.Environment,
	 *      java.time.ZonedDateTime, java.time.ZonedDateTime,
	 *      com.cfar.swim.worldwind.planning.CostPolicy,
	 *      com.cfar.swim.worldwind.planning.RiskPolicy)
	 */
	@Override
	public double getLegCost(Environment destination, ZonedDateTime start, ZonedDateTime end, CostPolicy costPolicy,
			RiskPolicy riskPolicy) {
		// TODO Auto-generated method stub
		return 0;
	}

	/**
	 * @param position
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.DiscreteEnvironment#isWaypoint(gov.nasa.worldwind.geom.Position)
	 */
	@Override
	public boolean isWaypoint(Position position) {
		// TODO Auto-generated method stub
		return false;
	}

	/**
	 * @param position
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.DiscreteEnvironment#getAdjacentWaypoints(gov.nasa.worldwind.geom.Position)
	 */
	@Override
	public Set<Position> getAdjacentWaypoints(Position position) {
		// TODO Auto-generated method stub
		return null;
	}

	/**
	 * @param position
	 * @param waypoint
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.planning.DiscreteEnvironment#isAdjacentWaypoint(gov.nasa.worldwind.geom.Position,
	 *      gov.nasa.worldwind.geom.Position)
	 */
	@Override
	public boolean isAdjacentWaypoint(Position position, Position waypoint) {
		// TODO Auto-generated method stub
		return false;
	}
	
	protected void constructRoadmap() {
		if (this.roadmapConstructor == RoadmapConstructor.BASICPRM) {
			BasicPRM basicPRM = new BasicPRM(this, MAX_ITER, MAX_NEIGHBORS, MAX_DIST);
			basicPRM.construct();
		}
		if (this.roadmapConstructor == RoadmapConstructor.LAZYPRM) {
			LazyPRM lazyPRM = new LazyPRM(this, MAX_ITER, MAX_NEIGHBORS, MAX_DIST);
			lazyPRM.construct();
		}
	}

}
