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
import java.time.Duration;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.time.chrono.ChronoZonedDateTime;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import com.binarydreamers.trees.Interval;
import com.binarydreamers.trees.IntervalTree;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.geom.Cube;
import com.cfar.swim.worldwind.geom.CubicGrid;
import com.cfar.swim.worldwind.geom.RegularGrid;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.ObstacleColor;
import com.cfar.swim.worldwind.render.TerrainObstacle;
import com.cfar.swim.worldwind.render.ThresholdRenderable;
import com.cfar.swim.worldwind.render.TimedRenderable;
import com.cfar.swim.worldwind.render.airspaces.ObstacleCylinder;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Polyline;
import gov.nasa.worldwind.util.measure.LengthMeasurer;

/**
 * Realizes a planning grid that can encode spatial and temporal costs, and can
 * be used for motion planning.
 * 
 * @author Stephan Heinemann
 *
 */
public class PlanningGrid extends CubicGrid implements Environment {

	/** the globe of this planning grid */
	private Globe globe = null;

	/** the cost interval tree encoding temporal costs */
	private IntervalTree<ChronoZonedDateTime<?>> costIntervals = new IntervalTree<ChronoZonedDateTime<?>>(
			CostInterval.comparator);

	/** the current time of this planning grid */
	private ZonedDateTime time = ZonedDateTime.now(ZoneId.of("UTC"));

	/** the current accumulated active cost of this planning grid */
	private double activeCost = 1d;

	/** the threshold cost of this planning grid */
	private double thresholdCost = 0d;

	/** the obstacles embedded into this planning grid */
	private HashSet<Obstacle> obstacles = new HashSet<Obstacle>();

	/** the terrain obstacles embedded into this planning grid */
	private HashSet<TerrainObstacle> terrainObstacles = new HashSet<TerrainObstacle>();

	/** the affected children of obstacle embeddings */
	private HashMap<Obstacle, List<PlanningGrid>> affectedChildren = new HashMap<Obstacle, List<PlanningGrid>>();

	/**
	 * Constructs a planning grid based on a geometric cube without any children.
	 * 
	 * @param cube the geometric cube
	 * 
	 * @see CubicGrid#CubicGrid(Cube)
	 */
	public PlanningGrid(Cube cube) {
		super(cube);
		this.update();
	}

	/**
	 * Constructs a planning grid from a geometric cube representing a reference
	 * child.
	 * 
	 * @param refChild the geometric cube representing the reference child
	 * @param rCells the number of cubic children along the <code>R</code> axis
	 * @param sCells the number of cubic children along the <code>S</code> axis
	 * @param tCells the number of cubic children along the <code>T</code> axis
	 *
	 * @see CubicGrid#CubicGrid(Cube, int, int, int)
	 */
	public PlanningGrid(Cube refChild, int rCells, int sCells, int tCells) {
		super(refChild, rCells, sCells, tCells);
		this.refresh();
	}

	/**
	 * Constructs a new planning grid from three plane normals and six distances for
	 * each of the six faces of a geometric box without any children.
	 * 
	 * This factory method is used during child construction and supposed to be
	 * overridden by specializing classes.
	 * 
	 * @see CubicGrid#newInstance(Vec4[], double, double, double, double, double,
	 *      double)
	 */
	@Override
	protected PlanningGrid newInstance(Vec4[] axes, double rMin, double rMax, double sMin, double sMax, double tMin,
			double tMax) {

		Box b = new Box(axes, rMin, rMax, sMin, sMax, tMin, tMax);
		return new PlanningGrid(new Cube(b.getOrigin(), axes, b.getRLength()));
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
	 * Gets the neighbor corner positions of a specified corner position of this
	 * planning grid.
	 * 
	 * @param position the specified corner position
	 * 
	 * @return the neighbor corner positions of a the specified corner position if a
	 *         corner position, null otherwise
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see Box#getNeighborCorners(Vec4)
	 */
	public Position[] getNeighborCorners(Position position) {
		Position[] neighborCorners = null;

		if (null != this.globe) {
			neighborCorners = new Position[3];
			Vec4[] corners = this.getNeighborCorners(globe.computePointFromPosition(position));

			if (null != corners) {
				neighborCorners[0] = this.globe.computePositionFromPoint(corners[0]);
				neighborCorners[1] = this.globe.computePositionFromPoint(corners[1]);
				neighborCorners[2] = this.globe.computePositionFromPoint(corners[2]);
			}
		} else {
			throw new IllegalStateException("globe is not set");
		}

		return neighborCorners;
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
		for (PlanningGrid grid : this.getAll()) {
			grid.globe = globe;
		}
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
		for (PlanningGrid grid : this.getAll()) {
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
		for (PlanningGrid grid : this.getAll()) {
			grid.thresholdCost = thresholdCost;
			grid.updateVisibility();
		}
	}

	/**
	 * Updates this planning grid with all its children.
	 */
	public void refresh() {
		for (PlanningGrid grid : this.getAll()) {
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
				for (PlanningGrid child : affectedChildren.get(obstacle)) {
					child.refresh(obstacle);
				}
			}
		}
	}

	/**
	 * Updates this planning grid.
	 */
	protected void update() {
		this.updateActiveCost();
		this.updateAppearance();
		this.updateVisibility();
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

				for (PlanningGrid child : this.getChildren()) {
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
	@Override
	public boolean embed(Obstacle obstacle) {
		boolean embedded = false;

		if (null != this.globe) {
			if (!this.isEmbedded(obstacle) && this.intersects(obstacle.getExtent(this.globe))) {
				this.addCostInterval(obstacle.getCostInterval());
				this.obstacles.add(obstacle);

				for (PlanningGrid child : this.getChildren()) {
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

	// TODO: embed all relevant kinds of (airspace, aircraft) rigid shapes
	// TODO: think about dynamically changing the resolution and embedded shapes
	// TODO: embedding should be a recursive operation starting at the root grid
	// cell
	// TODO: only if parent grid is affected, propagation to children will occur
	// TODO: performance can be substantially improved by only considering relevant
	// children for propagation
	// TODO: maximum obstacle radius/extension can limit possible children.

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
				for (PlanningGrid child : this.affectedChildren.get(obstacle)) {
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
				for (PlanningGrid child : this.affectedChildren.get(obstacle)) {
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

	// TODO: TERRAIN OBSTACLES functions are not correctly implemented for planning
	// grid. The code below was copied from sampling environment.
	/**
	 * Embeds a terrain obstacle into this planning grid.
	 * 
	 * @param obstacle the terrain obstacle to be embedded
	 * 
	 * @return true if the terrain obstacle has been embedded, false otherwise
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
	 * Unembeds a terrain obstacle from this planning grid.
	 * 
	 * @param obstacle the terrain obstacle to be unembedded
	 * 
	 * @return true if the terrain obstacle has been unembedded, false otherwise
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
	 * Updates this planning grid for an embedded terrain obstacle.
	 * 
	 * @param obstacle the embedded terrain obstacle
	 * 
	 * @see Environment#refresh(Obstacle)
	 */
	public void refresh(TerrainObstacle obstacle) {
		if (this.terrainObstacles.contains(obstacle)) {
			this.update();
		}
	}

	/**
	 * Unembeds all terrain obstacles from this planning grid.
	 * 
	 * @see Environment#unembedAll()
	 */
	public void unembedTerrainAll() {
		this.terrainObstacles.clear();
	}

	/**
	 * Indicates whether or not a terrain obstacle is embedded in this planning
	 * grid.
	 * 
	 * @param obstacle the terrain obstacle
	 * 
	 * @return true if the terrain obstacle is embedded in this planning grid, false
	 *         otherwise
	 * 
	 * @see Environment#isEmbedded(Obstacle)
	 */
	public boolean isEmbedded(TerrainObstacle obstacle) {
		return this.terrainObstacles.contains(obstacle);
	}

	/**
	 * Adds an affected child to an obstacle embedding.
	 * 
	 * @param obstacle the obstacle of the embedding
	 * @param child the affected child
	 */
	private void addAffectedChild(Obstacle obstacle, PlanningGrid child) {
		if (this.affectedChildren.containsKey(obstacle)) {
			this.affectedChildren.get(obstacle).add(child);
		} else {
			ArrayList<PlanningGrid> children = new ArrayList<PlanningGrid>();
			children.add(child);
			this.affectedChildren.put(obstacle, children);
		}
	}

	/**
	 * Adds the specified number of children on each axis to this planning grid and
	 * propagates existing obstacle embeddings.
	 * 
	 * @param rCells the number of children on the <code>R</code> axis
	 * @param sCells the number of children on the <code>S</code> axis
	 * @param tCells the number of children on the <code>T</code> axis
	 * 
	 * @see CubicGrid#addChildren(int, int, int)
	 */
	@Override
	public void addChildren(int rCells, int sCells, int tCells) {
		super.addChildren(rCells, sCells, tCells);

		for (PlanningGrid child : this.getChildren()) {
			// initialize children
			child.setGlobe(this.globe);
			child.setTime(this.time);
			child.setThreshold(this.thresholdCost);
			child.update();

			// propagate obstacle embeddings
			for (Obstacle obstacle : this.obstacles) {
				if (obstacle instanceof ObstacleCylinder) {
					if (child.embed((ObstacleCylinder) obstacle)) {
						this.addAffectedChild(obstacle, child);
					}
				}
				// TODO: implement propagations for other extents
			}
		}
	}

	/**
	 * Removes all children from this planning grid.
	 * 
	 * @see CubicGrid#removeChildren()
	 */
	@Override
	public void removeChildren() {
		super.removeChildren();
		// remove affected children of all obstacle embeddings
		this.affectedChildren.clear();
	}

	/**
	 * Gets the children of this planning grid.
	 * 
	 * @return the children of this planning grid
	 * 
	 * @see CubicGrid#getChildren()
	 * @see Environment#getChildren()
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends PlanningGrid> getChildren() {
		return (Set<PlanningGrid>) super.getChildren();
	}

	/**
	 * Gets a particular child of this planning grid if present.
	 * 
	 * @param r the <code>R</code> index of the child cell
	 * @param s the <code>S</code> index of the child cell
	 * @param t the <code>T</code> index of the child cell
	 * 
	 * @return the particular child of this planning grid if present, null otherwise
	 * 
	 * @see CubicGrid#getChild(int, int, int)
	 */
	@Override
	public PlanningGrid getChild(int r, int s, int t) {
		return (PlanningGrid) super.getChild(r, s, t);
	}

	/**
	 * Gets the parent of this planning grid if present.
	 * 
	 * @return the parent of this planning grid if present, null otherwise
	 * 
	 * @see CubicGrid#getParent()
	 */
	@Override
	public PlanningGrid getParent() {
		return (PlanningGrid) super.getParent();
	}

	/**
	 * Gets all planning grids associated with this planning grid.
	 * 
	 * @return all planning grids associated with this planning grid
	 * 
	 * @see CubicGrid#getAll()
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends PlanningGrid> getAll() {
		return (Set<PlanningGrid>) super.getAll();
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
	public Set<? extends PlanningGrid> getRefinements() {
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
		this.addChildren(density);
	}

	/**
	 * Coarsens, that is, removes the children of this planning grid.
	 *
	 */
	public void coarsen() {
		this.removeChildren();
	}

	/**
	 * Looks up the planning grid cells (maximum eight) containing a specified point
	 * in world model coordinates considering numerical inaccuracies.
	 * 
	 * @param modelPoint the point in world model coordinates
	 * 
	 * @return the planning grid cells containing the specified point
	 * 
	 * @see CubicGrid#lookupCells(Vec4)
	 */
	@Override
	@SuppressWarnings("unchecked")
	public Set<? extends PlanningGrid> lookupCells(Vec4 modelPoint) {
		return (Set<PlanningGrid>) super.lookupCells(modelPoint);
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
			cells = (Set<PlanningGrid>) super.lookupCells(this.globe.computePointFromPosition(position));
		} else {
			throw new IllegalStateException("globe is not set");
		}

		return cells;
	}

	/**
	 * Finds all cells of this planning grid that satisfy a specified predicate. A
	 * full recursive search is performed considering only non-parent cells.
	 * 
	 * @param predicate the predicate
	 * 
	 * @return the cells of this planning grid that satisfy a predicate
	 * 
	 * @see CubicGrid#findCells(Predicate)
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends PlanningGrid> findCells(Predicate<RegularGrid> predicate) {
		return (Set<PlanningGrid>) super.findCells(predicate);
	}

	/**
	 * Finds all cells of this planning grid that satisfy a specified predicate
	 * taking a specified hierarchical depth into account. A zero depth does not
	 * consider any children. A negative depth performs a full recursive search and
	 * considers non-parent cells only.
	 * 
	 * @param predicate the predicate
	 * @param depth the hierarchical depth
	 * 
	 * @return the cells of this planning grid that satisfy a predicate taking the
	 *         hierarchical depth into account
	 * 
	 * @see CubicGrid#findCells(Predicate, int)
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends PlanningGrid> findCells(Predicate<RegularGrid> predicate, int depth) {
		return (Set<PlanningGrid>) super.findCells(predicate, depth);
	}

	/**
	 * Gets the intersection positions of a straight leg with the cells of this
	 * planning grid. A full recursive search is performed considering only
	 * non-parent cells.
	 * 
	 * @param origin the origin of the leg in globe coordinates
	 * @param destination the destination of the leg in globe coordinates
	 * 
	 * @return the intersection positions of the straight leg with the cells of this
	 *         planning grid
	 * 
	 * @throws IllegalStateException if the globe is not set
	 * 
	 * @see CubicGrid#getIntersectionPoints(Vec4, Vec4)
	 */
	public Iterable<? extends Position> getIntersectedPositions(Position origin, Position destination) {
		if (null != this.globe) {
			return super.getIntersectionPoints(this.globe.computePointFromPosition(origin),
					this.globe.computePointFromPosition(destination)).stream().map(this.globe::computePositionFromPoint)
							.collect(Collectors.toCollection(LinkedHashSet<Position>::new));
		} else {
			throw new IllegalStateException("globe is not set");
		}
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
			return super.isWayoint(this.globe.computePointFromPosition(position));
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
		if (null != this.globe) {
			Set<Vec4> waypoints = super.getAdjacentWaypoints(this.globe.computePointFromPosition(position));
			return waypoints.stream().map(this.globe::computePositionFromPoint).collect(Collectors.toSet());
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
			return super.isAdjacentWaypoint(this.globe.computePointFromPosition(position),
					this.globe.computePointFromPosition(waypoint));
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}

	/**
	 * Gets the neighbors of this planning grid. A full recursive search is
	 * performed considering only non-parent neighbors.
	 * 
	 * @return the non-parent neighbors of this planning grid
	 * 
	 * @see CubicGrid#getNeighbors()
	 * @see Environment#getNeighbors()
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends PlanningGrid> getNeighbors() {
		return (Set<PlanningGrid>) super.getNeighbors();
	}

	/**
	 * Gets the neighbors of this planning grid taking a specified hierarchical
	 * depth into account. A zero depth does not consider any neighboring children.
	 * A negative depth performs a full recursive search and considers non-parent
	 * neighbors only.
	 * 
	 * @param depth the hierarchical depth for finding neighbors
	 * 
	 * @return the neighbors of this planning grid
	 * 
	 * @see CubicGrid#getNeighbors(int)
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends PlanningGrid> getNeighbors(int depth) {
		return (Set<PlanningGrid>) super.getNeighbors(depth);
	}

	/**
	 * Indicates whether or not this planning grid is a neighbor of another
	 * environment.
	 * 
	 * @param neighbor the potential neighbor
	 * 
	 * @return true if this planning grid is a neighbor of the other environment,
	 *         false otherwise
	 * 
	 * @see RegularGrid#areNeighbors(RegularGrid)
	 */
	@Override
	public boolean areNeighbors(Environment neighbor) {
		return this.getNeighbors().contains(neighbor);
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

		// TODO: coordinate transformations might be too expensive for planning
		// TODO: planning could be based on Vec4 with a final transformation of the
		// route

		if (null != this.globe) {
			Set<Vec4> neighborPoints = super.getNeighbors(this.globe.computePointFromPosition(position));
			for (Vec4 neighbor : neighborPoints) {
				neighbors.add(this.globe.computePositionFromPoint(neighbor));
			}
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
			return super.areNeighbors(this.globe.computePointFromPosition(position),
					this.globe.computePointFromPosition(neighbor));
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}

	/**
	 * Gets the distance between two positions in this planning grid.
	 * 
	 * @param position1 the first position
	 * @param position2 the second position
	 * 
	 * @return the distance between the two positions in this planning grid
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
	 * Gets the normalized distance between two positions in this planning grid.
	 * 
	 * @param position1 the first position
	 * @param position2 the second position
	 * 
	 * @return the normalized distance between the two positions in this planning
	 *         grid
	 */
	@Override
	public double getNormalizedDistance(Position position1, Position position2) {
		return this.getDistance(position1, position2) / this.getNormalizer();
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
	 * Gets the leg cost from an origin to a destination position within this
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
	 * @return the leg cost from the origin to the destination position
	 * 
	 * @see Environment#getLegCost(Position, Position, ZonedDateTime, ZonedDateTime,
	 *      CostPolicy, RiskPolicy)
	 */
	@Override
	public double getLegCost(Position origin, Position destination, ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {

		double legCost = Double.POSITIVE_INFINITY;

		// compute leg performance
		Duration legDuration = Duration.between(start, end);
		double legDistance = this.getDistance(origin, destination);
		double legSpeed = legDistance / legDuration.getSeconds();

		// compute all intersection positions on this straight leg
		Iterator<? extends Position> positionIterator = this.getIntersectedPositions(origin, destination).iterator();

		// compute the cost of each leg segment
		if (positionIterator.hasNext()) {
			legCost = 0d;
			ZonedDateTime currentEto = start;
			Position current = positionIterator.next();

			while (positionIterator.hasNext()) {
				Position next = positionIterator.next();

				// compute step performance
				double stepDistance = this.getDistance(current, next);
				Duration stepDuration = Duration.ofSeconds(Math.round((stepDistance / legSpeed)));
				ZonedDateTime nextEto = currentEto.plus(stepDuration);

				legCost += this.getStepCost(current, next, currentEto, nextEto, costPolicy, riskPolicy);

				currentEto = nextEto;
				current = next;
			}
		}

		return legCost;
	}

	/**
	 * Gets the leg cost from the center of this planning grid to the center of
	 * another environment between a start and an end time given a cost policy and
	 * risk policy.
	 * 
	 * @param destination the destination environment
	 * @param start the start time
	 * @param end the end time
	 * @param costPolicy the cost policy
	 * @param riskPolicy the risk policy
	 * 
	 * @return the leg cost from the center of this environment to the center of the
	 *         destination environment
	 * 
	 * @see Environment#getLegCost(Environment, ZonedDateTime, ZonedDateTime,
	 *      CostPolicy, RiskPolicy)
	 */
	@Override
	public double getLegCost(Environment destination, ZonedDateTime start, ZonedDateTime end, CostPolicy costPolicy,
			RiskPolicy riskPolicy) {

		double legCost = Double.POSITIVE_INFINITY;

		// compute leg performance
		Duration legDuration = Duration.between(start, end);
		double legDistance = this.getDistance(this.getCenterPosition(), destination.getCenterPosition());
		double legSpeed = legDistance / legDuration.getSeconds();

		// compute all intersection positions on this straight leg
		Iterator<? extends Position> positionIterator = this
				.getIntersectedPositions(this.getCenterPosition(), destination.getCenterPosition()).iterator();

		// compute the cost of each leg segment
		if (positionIterator.hasNext()) {
			legCost = 0d;
			ZonedDateTime currentEto = start;
			Position current = positionIterator.next();

			while (positionIterator.hasNext()) {
				Position next = positionIterator.next();

				// compute step performance
				double stepDistance = this.getDistance(current, next);
				Duration stepDuration = Duration.ofSeconds(Math.round((stepDistance / legSpeed)));
				ZonedDateTime nextEto = currentEto.plus(stepDuration);

				legCost += this.getStepCost(current, next, currentEto, nextEto, costPolicy, riskPolicy);

				currentEto = nextEto;
				current = next;
			}
		}

		return legCost;
	}

}
