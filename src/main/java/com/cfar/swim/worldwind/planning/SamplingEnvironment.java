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
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import com.binarydreamers.trees.Interval;
import com.binarydreamers.trees.IntervalTree;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.geom.HierarchicalBox;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.ObstacleColor;
import com.cfar.swim.worldwind.render.ThresholdRenderable;
import com.cfar.swim.worldwind.render.TimedRenderable;
import com.cfar.swim.worldwind.render.airspaces.ObstacleCylinder;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.BasicShapeAttributes;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.Path;
import gov.nasa.worldwind.render.Polyline;
import gov.nasa.worldwind.util.measure.LengthMeasurer;

/**
 * Realizes a sampling environment that can encode spatial and temporal costs,
 * and can be used for motion planning.
 * 
 * @author Manuel Rosa
 * @author Henrique Ferreira
 *
 */
public class SamplingEnvironment extends HierarchicalBox implements Environment {

	/** the globe of this sampling environment */
	private Globe globe = null;

	/** the cost interval tree encoding temporal costs */
	private IntervalTree<ChronoZonedDateTime<?>> costIntervals = new IntervalTree<ChronoZonedDateTime<?>>(
			CostInterval.comparator);

	/** the current time of this sampling environment */
	private ZonedDateTime time = ZonedDateTime.now(ZoneId.of("UTC"));

	/** the current accumulated active cost of this sampling environment */
	private double activeCost = 1d;

	/** the threshold cost of this sampling environment */
	private double thresholdCost = 0d;

	/** the obstacles embedded into this sampling environment */
	private HashSet<Obstacle> obstacles = new HashSet<Obstacle>();

	/** the affected children of obstacle embeddings */
	private HashMap<Obstacle, List<SamplingEnvironment>> affectedChildren = new HashMap<Obstacle, List<SamplingEnvironment>>();

	/** the resolution of this sampling environment */
	private double resolution = 100d;

	/**
	 * Constructs a sampling environment based on a geometric box.
	 * 
	 * @param box the geometric box
	 */
	public SamplingEnvironment(Box box) {
		super(box);
		this.update();
	}

	/**
	 * Constructs a sampling environment based on a hierachical box.
	 * 
	 * @param box the hierarchical box
	 */
	public SamplingEnvironment(HierarchicalBox box) {
		super(box);
		this.refresh();
	}

	/**
	 * Constructs a sampling environment based on two points.
	 * 
	 * @param point1 the first point in world model coordinates
	 * @param point2 the second point in world model coordinates
	 * 
	 * @return the new sampling environment
	 */
	protected SamplingEnvironment createChild(Vec4 point1, Vec4 point2) {
		return new SamplingEnvironment(super.createInstance(point1, point2));
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
	 * Sets the globe of this sampling environment.
	 * 
	 * @param globe the globe of this sampling environment
	 * 
	 * @see Environment#setGlobe(Globe)
	 */
	@Override
	public void setGlobe(Globe globe) {
		for (SamplingEnvironment env : this.getAll()) {
			env.globe = globe;
		}
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
		// TODO: Should costs be automatically propagated to the affected child cells?
		// TODO: Should added children costs be propagated to parents?
		// TODO: What happens if children are added and removed?
		// TODO: Shall parents aggregate all costs?!
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
		for (SamplingEnvironment env : this.getAll()) {
			env.time = time;
			env.update();
		}
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
		for (SamplingEnvironment env : this.getAll()) {
			env.thresholdCost = thresholdCost;
			env.updateVisibility();
		}
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
	 * @return the affectedChildren
	 */
	public HashMap<Obstacle, List<SamplingEnvironment>> getAffectedChildren() {
		return affectedChildren;
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
	 * Updates this sampling environment with all its children.
	 */
	public void refresh() {
		for (SamplingEnvironment env : this.getAll()) {
			env.update();
		}
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
			if (this.affectedChildren.containsKey(obstacle)) {
				for (SamplingEnvironment child : affectedChildren.get(obstacle)) {
					child.refresh(obstacle);
				}
			}
		}
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
	 * Embeds an obstacle into this sampling environment.
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
				for (SamplingEnvironment child : this.getChildren()) {
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
				for (SamplingEnvironment child : this.getChildren()) {
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

			if (this.affectedChildren.containsKey(obstacle)) {
				for (SamplingEnvironment child : this.affectedChildren.get(obstacle)) {
					child.unembed(obstacle);
				}
				this.affectedChildren.remove(obstacle);
			}
			unembedded = true;
		}

		return unembedded;
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

			if (this.affectedChildren.containsKey(obstacle)) {
				for (SamplingEnvironment child : this.affectedChildren.get(obstacle)) {
					child.unembed(obstacle);
				}
				this.affectedChildren.remove(obstacle);
			}
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
	 * Adds an affected child to an obstacle embedding.
	 * 
	 * @param obstacle the obstacle of the embedding
	 * @param child the affected child
	 */
	protected void addAffectedChild(Obstacle obstacle, SamplingEnvironment child) {
		if (this.affectedChildren.containsKey(obstacle)) {
			this.affectedChildren.get(obstacle).add(child);
		} else {
			ArrayList<SamplingEnvironment> children = new ArrayList<SamplingEnvironment>();
			children.add(child);
			this.affectedChildren.put(obstacle, children);
		}
	}

	/**
	 * Adds a child to this sampling environment, by creating a new sampling
	 * environment based on two positions.
	 * 
	 * @param origin the origin position in globe coordinates
	 * @param other the other position in globe coordinates
	 */
	public void addChild(Position origin, Position other) {
		Vec4 pointOrigin = globe.computePointFromPosition(origin);
		Vec4 pointOther = globe.computePointFromPosition(other);

		SamplingEnvironment child = this.createChild(pointOrigin, pointOther);

		child.setGlobe(this.globe);
		child.setTime(this.time);
		child.setThreshold(this.thresholdCost);
		child.update();
		child.parent = this;
		this.cells.add(child);
		child.setOrigin(pointOrigin);

		// propagate obstacle embeddings
		for (Obstacle obstacle : this.obstacles) {
			if (obstacle instanceof ObstacleCylinder) {
				if (child.embed((ObstacleCylinder) obstacle)) {
					this.addAffectedChild(obstacle, child);
				}
			}
		}
	}

	/**
	 * Removes a specific child from this sampling environment.
	 */
	@Override
	public void removeChild(HierarchicalBox child) {
		super.removeChild(child);
	}

	/**
	 * Removes all children from this sampling environment.
	 */
	public void removeChildren() {
		super.removeChildren();
		// remove affected children of all obstacle embeddings
		this.affectedChildren.clear();
	}

	/**
	 * Gets the children of this sampling environment.
	 * 
	 * @return the children of this sampling environment
	 * 
	 * @see com.cfar.swim.worldwind.geom.HierarchicalBox#getChildren()
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends SamplingEnvironment> getChildren() {
		return (Set<SamplingEnvironment>) super.getChildren();
	}

	/**
	 * Gets a particular child of this sampling environment if present.
	 * 
	 * @param child the child sampling environment
	 * 
	 * @return the particular child of sampling environment if present, null
	 *         otherwise
	 */
	public SamplingEnvironment getChild(SamplingEnvironment child) {
		return (SamplingEnvironment) super.getChild(child);
	}

	/**
	 * Gets the parent of this sampling environment if present.
	 * 
	 * @return the parent of this sampling environment if present, null otherwise
	 */
	@Override
	public SamplingEnvironment getParent() {
		return (SamplingEnvironment) super.getParent();
	}

	/**
	 * Gets all sampling environments associated with this sampling environment.
	 * 
	 * @return all sampling environments associated with this sampling environment
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends SamplingEnvironment> getAll() {
		return (Set<SamplingEnvironment>) super.getAll();
	}

	/**
	 * Indicates whether or not this sampling environment is refined, that is, has
	 * children.
	 * 
	 * @return true if this sampling environment is refined, false otherwise
	 * 
	 */
	public boolean isRefined() {
		return this.hasChildren();
	}

	/**
	 * Gets the refinements, that is, children of this sampling environment.
	 * 
	 * @return the refinements of this sampling environment
	 * 
	 */
	public Set<? extends SamplingEnvironment> getRefinements() {
		return this.getChildren();
	}

	/**
	 * Refines, that is, adds children with a specified density to this sampling
	 * environment.
	 * 
	 * @param density the refinement density
	 * 
	 */
	public void refine(int density) {
		// TODO: review how to use integer to refine
		// this.addChildren(density);
	}

	/**
	 * Coarsens, that is, removes the children of this sampling environment.
	 *
	 */
	public void coarsen() {
		this.removeChildren();
	}

	/**
	 * Looks up the sampling environment cells containing a specified point in world
	 * model coordinates considering numerical inaccuracies.
	 * 
	 * @param modelPoint the point in world model coordinates
	 * 
	 * @return the sampling environment cells containing the specified point
	 * 
	 * @see com.cfar.swim.worldwind.geom.HierarchicalBox#lookupCells(gov.nasa.worldwind.geom.Vec4)
	 */
	@Override
	@SuppressWarnings("unchecked")
	public Set<? extends SamplingEnvironment> lookupCells(Vec4 modelPoint) {
		return (Set<SamplingEnvironment>) super.lookupCells(modelPoint);
	}

	/**
	 * Looks up the sampling environment cells containing a specified position in
	 * globe coordinates considering numerical inaccuracies.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return the sampling environment cells containing the specified position
	 * 
	 * @throws IllegalStateException if the globe is not set
	 */
	@SuppressWarnings("unchecked")
	public Set<? extends SamplingEnvironment> lookupCells(Position position) {
		Set<SamplingEnvironment> cells = null;

		if (null != this.globe) {
			cells = (Set<SamplingEnvironment>) super.lookupCells(this.globe.computePointFromPosition(position));
		} else {
			throw new IllegalStateException("globe is not set");
		}

		return cells;
	}

	/**
	 * Finds all cells of this sampling environment that satisfy a specified
	 * predicate. A full recursive search is performed considering only non-parent
	 * cells.
	 * 
	 * @param predicate the predicate
	 * 
	 * @return the cells of this sampling environment that satisfy a predicate
	 * 
	 * @see com.cfar.swim.worldwind.geom.HierarchicalBox#findCells(java.util.function.Predicate)
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends SamplingEnvironment> findCells(Predicate<HierarchicalBox> predicate) {
		return (Set<SamplingEnvironment>) super.findCells(predicate);
	}

	/**
	 * Finds all cells of this sampling environment that satisfy a specified
	 * predicate taking a specified hierarchical depth into account. A zero depth
	 * does not consider any children. A negative depth performs a full recursive
	 * search and considers non-parent cells only.
	 * 
	 * @param predicate the predicate
	 * @param depth the hierarchical depth
	 * 
	 * @return the cells of this sampling environment that satisfy a predicate
	 *         taking the hierarchical depth into account
	 * 
	 * @see com.cfar.swim.worldwind.geom.HierarchicalBox#findCells(java.util.function.Predicate,
	 *      int)
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends SamplingEnvironment> findCells(Predicate<HierarchicalBox> predicate, int depth) {
		return (Set<SamplingEnvironment>) super.findCells(predicate, depth);
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
			return super.isWaypoint(this.globe.computePointFromPosition(position));
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
		if (null != this.globe) {
			Set<Vec4> waypoints = super.getAdjacentWaypoints(this.globe.computePointFromPosition(position));
			return waypoints.stream().map(this.globe::computePositionFromPoint).collect(Collectors.toSet());
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
			return super.isAdjacentWaypoint(this.globe.computePointFromPosition(position),
					this.globe.computePointFromPosition(waypoint));
		} else {
			throw new IllegalStateException("globe is not set");
		}
	}

	/**
	 * Gets the neighbors of this sampling environment. A full recursive search is
	 * performed considering only non-parent neighbors.
	 * 
	 * @return the non-parent neighbors of this sampling environment
	 * 
	 * @see Environment#getNeighbors()
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends SamplingEnvironment> getNeighbors() {
		return (Set<SamplingEnvironment>) super.getNeighbors();
	}

	/**
	 * Gets the neighbors of this sampling environment taking a specified
	 * hierarchical depth into account. A zero depth does not consider any
	 * neighboring children. A negative depth performs a full recursive search and
	 * considers non-parent neighbors only.
	 * 
	 * @param depth the hierarchical depth for finding neighbors
	 * 
	 * @return the neighbors of this sampling environment
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends SamplingEnvironment> getNeighbors(int depth) {
		return (Set<SamplingEnvironment>) super.getNeighbors(depth);
	}

	/**
	 * Indicates whether or not this sampling environment is a neighbor of another
	 * environment.
	 * 
	 * @param neighbor the potential neighbor
	 * 
	 * @return true if this sampling environment is a neighbor of the other
	 *         environment, false otherwise
	 */
	@Override
	public boolean areNeighbors(Environment neighbor) {
		return this.getNeighbors().contains(neighbor);
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
			return super.areNeighbors(this.globe.computePointFromPosition(position),
					this.globe.computePointFromPosition(neighbor));
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
	 */
	@Override
	public double getStepCost(Position origin, Position destination, ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy) {

		double stepCost = 0d;

		// compute participating cells
		Set<? extends SamplingEnvironment> segmentCells = this.lookupCells(origin);
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
		for (SamplingEnvironment segmentCell : segmentCells) {
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
	 * TODO: NOT IMPLEMENTED YET
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
	 * TODO: NOT IMPLEMENTED YET
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
	 * Renders this sampling environment. If it has children, then the children are
	 * also rendered.
	 * 
	 * @param dc the drawing context
	 */
	@Override
	public void render(DrawContext dc) {
		if (this.visible) {
			if (this.hasParent()) {
				Path diagonal = this.createRenderableDiagonal();
				diagonal.render(dc);
			} else {
				super.render(dc);
			}
			if (this.hasChildren()) {
				for (SamplingEnvironment child : this.getChildren()) {
					child.render(dc);

				}
			}
		}
	}

	/**
	 * Creates a path between the two implicit waypoints of this sampling environment.
	 * 
	 * @return the diagonal between the two waypoints of this sampling environment
	 */
	public Path createRenderableDiagonal() {
		Position originPosition = globe.computePositionFromPoint(this.getOrigin());
		Position oppositePosition = globe.computePositionFromPoint(this.get3DOpposite());
		Path diagonal = new Path(originPosition, oppositePosition);
		diagonal.setAttributes(new BasicShapeAttributes());
		diagonal.getAttributes().setOutlineMaterial(this.getMaterial());
		return diagonal;
	}

	/**
	 * Samples a position from a continuous space defined in the sampling environment
	 * 
	 * @return position in global coordinates inside this sampling environment
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
	 * @param position the position in globe coordinates
	 * 
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
	 * @param waypoint the waypoint in globe coordinates
	 * 
	 * @return true if there is a conflict, and false otherwise
	 */
	public boolean checkConflict(Position position) {

		if (this.isInsideGlobe(this.getGlobe(), position))
			return true;

		// TODO : Implement a checker for conflict between a position and the
		// static, time independent and untraversable obstacles in the environment

		// Box box = this.createBoundingBox(position);
		// HashSet<TerrainObstacle> terrainSet = this.getTerrainObstacles();
		// for (TerrainObstacle terrain : terrainSet) {
		// // Check if obstacle contains the waypoint
		// System.out.println("Terrain");
		// if (terrain.getExtent(this.getGlobe()).intersects(box.getFrustum())) {
		// return true;
		// }
		// }
		return false;
	}

	/**
	 * Checks if a straight leg between the waypoints is in conflict with
	 * untraversable obstacles in the environment
	 * 
	 * @param waypoint1 the first waypoint in globe coordinates
	 * @param waypoint2 the second waypoint in globe coordinates
	 * 
	 * @return true if there is a conflict, and false otherwise
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
}
