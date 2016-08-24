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
import java.util.List;
import java.util.Set;

import com.binarydreamers.trees.Interval;
import com.binarydreamers.trees.IntervalTree;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.geom.RegularGrid;
import com.cfar.swim.worldwind.render.ObstacleColor;
import com.cfar.swim.worldwind.render.ThresholdRenderable;
import com.cfar.swim.worldwind.render.TimedRenderable;

import gov.nasa.worldwind.geom.Cylinder;
import gov.nasa.worldwind.geom.Extent;
import gov.nasa.worldwind.geom.Vec4;

/**
 * Realizes a non-uniform cost interval grid that can encode spatial and
 * temporal costs, and can be used for motion planning.
 * 
 * @author Stephan Heinemann
 *
 */
public class NonUniformCostIntervalGrid extends RegularGrid implements TimedRenderable, ThresholdRenderable {

	/** the cost interval tree encoding temporal costs */
	private IntervalTree<ChronoZonedDateTime<?>> costIntervals = new IntervalTree<ChronoZonedDateTime<?>>(CostInterval.comparator);
	
	/** the current time of this non-uniform cost interval grid */
	private ZonedDateTime time = ZonedDateTime.now(ZoneId.of("UTC"));
	
	/** the current active cost intervals of this non-uniform cost interval grid */
	private List<Interval<ChronoZonedDateTime<?>>> activeCostIntervals = this.getCostIntervals(time);
	
	/** the current accumulated active cost of this non-uniform cost interval grid */
	private int activeCost = 1;
	
	/** the threshold cost of this non-uniform cost interval grid */
	private int thresholdCost = 0;
	
	/** the embeddints of this non-uniform cost interval grid */
	private HashMap<Extent, CostInterval> embeddings = new HashMap<Extent, CostInterval>();
	
	/** the affected children of cost interval embeddings */
	private HashMap<CostInterval, List<NonUniformCostIntervalGrid>> affectedChildren = new HashMap<CostInterval, List<NonUniformCostIntervalGrid>>();
	
	/**
	 * Constructs a non-uniform cost interval grid based on a geometric box
	 * without any children.
	 * 
	 * @param box the geometric box
	 * 
	 * @see RegularGrid#RegularGrid(Box)
	 */
	public NonUniformCostIntervalGrid(Box box) {
		super(box);
	}
	
	/**
	 * Constructs a non-uniform cost interval grid based on a geometric box
	 * with the specified number of children on each axis.
	 * 
	 * @param box the geometric box
	 * @param rCells the number of children on the <code>R</code> axis
	 * @param sCells the number of children on the <code>S</code> axis
	 * @param tCells the number of children on the <code>T</code> axis
	 * 
	 * @see RegularGrid#RegularGrid(Box, int, int, int)
	 */
	public NonUniformCostIntervalGrid(Box box, int rCells, int sCells, int tCells) {
		super(box, rCells, sCells, tCells);
	}
	
	/**
	 * Constructs a non-uniform cost interval grid from three plane normals and
	 * six distances for each of the six faces of a geometric box without any
	 * children.
	 * 
	 * @param axes the three plane normals
	 * @param rMin the minimum distance on the <code>R</code> axis
	 * @param rMax the maximum distance on the <code>R</code> axis
	 * @param sMin the minimum distance on the <code>S</code> axis
	 * @param sMax the maximum distance on the <code>S</code> axis
	 * @param tMin the minimum distance on the <code>T</code> axis
	 * @param tMax the maximum distance on the <code>T</code> axis
	 * 
	 * @see RegularGrid#RegularGrid(Vec4[], double, double, double, double, double, double)
	 */
	public NonUniformCostIntervalGrid(
			Vec4[] axes,
			double rMin, double rMax,
			double sMin, double sMax,
			double tMin, double tMax) {
		super(axes, rMin, rMax, sMin, sMax, tMin, tMax);
	}
	
	/**
	 * Constructs a non-uniform cost interval grid from three plane normals and
	 * six distances for each of the six faces of a geometric box with the
	 * specified number of children on each axis.
	 * 
	 * @param axes the three plane normals
	 * @param rMin the minimum distance on the <code>R</code> axis
	 * @param rMax the maximum distance on the <code>R</code> axis
	 * @param sMin the minimum distance on the <code>S</code> axis
	 * @param sMax the maximum distance on the <code>S</code> axis
	 * @param tMin the minimum distance on the <code>T</code> axis
	 * @param tMax the maximum distance on the <code>T</code> axis
	 * @param rCells the number of children on the <code>R</code> axis
	 * @param sCells the number of children on the <code>S</code> axis
	 * @param tCells the number of children on the <code>T</code> axis
	 * 
	 * @see RegularGrid#RegularGrid(Vec4[], double, double, double, double, double, double, int, int, int)
	 */
	public NonUniformCostIntervalGrid(Vec4[] axes,
			double rMin, double rMax,
			double sMin, double sMax,
			double tMin, double tMax,
			int rCells, int sCells, int tCells) {
		super(axes, rMin, rMax, sMin, sMax, tMin, tMax, rCells, sCells, tCells);
	}
	
	/**
	 * Constructs a non-uniform cost interval grid from three plane normals and
	 * six distances for each of the six faces of a geometric box without any
	 * children. The current time and threshold cost are propagated to the child.
	 * 
	 * This factory method is used during child construction and supposed to be
	 * overridden by specializing classes.
	 * 
	 * @see NonUniformCostIntervalGrid#NonUniformCostIntervalGrid(Vec4[], double, double, double, double, double, double)
	 */
	@Override
	protected NonUniformCostIntervalGrid newInstance(
			Vec4[] axes,
			double rMin, double rMax,
			double sMin, double sMax,
			double tMin, double tMax) {
		NonUniformCostIntervalGrid grid = new NonUniformCostIntervalGrid(
				axes, rMin, rMax, sMin, sMax, tMin, tMax);
		grid.setTime(this.time);
		grid.setThreshold(this.thresholdCost);
		grid.update();
		return grid;
	}
	
	/**
	 * Looks up the non-uniform cost interval grid cells (maximum eight)
	 * containing a specified point in world model coordinates considering
	 * numerical inaccuracies.
	 * 
	 * @param modelPoint the point in world model coordinates
	 * 
	 * @return the non-uniform cost interval grid cells containing the specified point
	 */
	@Override
	@SuppressWarnings("unchecked")
	public List<? extends NonUniformCostIntervalGrid> lookupCells(Vec4 modelPoint) {
		return (List<NonUniformCostIntervalGrid>) super.lookupCells(modelPoint);
	}
	
	/**
	 * Adds a cost interval to this non-uniform cost interval grid.
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
	 * Removes a cost interval from this non-uniform cost interval grid.
	 * 
	 * @param costInterval the cost interval to be removed
	 */
	public void removeCostInterval(CostInterval costInterval) {
		this.costIntervals.remove(costInterval);
		this.update();
	}
	
	/**
	 * Gets all cost intervals that are active at a specified time instant.
	 * 
	 * @param time the time instant
	 * @return all cost intervals that are active at the specified time instant
	 */
	public List<Interval<ChronoZonedDateTime<?>>> getCostIntervals(ZonedDateTime time) {
		return this.costIntervals.searchInterval(new CostInterval(null, this.time));
	}
	
	/**
	 * Gets all cost intervals that are active during a specified time interval.
	 * 
	 * @param start the start time of the time interval
	 * @param end the end time of the time interval
	 * @return all cost intervals that are active during the specified time interval 
	 */
	public List<Interval<ChronoZonedDateTime<?>>> getCostIntervals(ZonedDateTime start, ZonedDateTime end) {
		return this.costIntervals.searchInterval(new CostInterval(null, start, end));
	}
	
	/**
	 * Gets the current time of this non-uniform cost interval grid.
	 * 
	 * @return the current time of this non-uniform cost interval grid
	 * 
	 * @see TimedRenderable#getTime()
	 */
	@Override
	public ZonedDateTime getTime() {
		return this.time;
	}
	
	/**
	 * Sets the current time of this non-uniform cost interval grid.
	 * 
	 * @param time the current time of this non-uniform cost interval grid
	 * 
	 * @see TimedRenderable#setTime(ZonedDateTime)
	 */
	@Override
	public void setTime(ZonedDateTime time) {
		this.time = time;
		this.update();
		
		if (this.hasChildren()) {
			for (int r = 0; r < this.cells.length; r++) {
				for (int s = 0; s < this.cells[r].length; s++) {
					for (int t = 0; t < this.cells[r][s].length; t++) {
						((TimedRenderable) this.cells[r][s][t]).setTime(time);
					}
				}
			}
		}
	}
	
	/**
	 * Gets the threshold cost of this non-uniform cost interval grid.
	 * 
	 * @return the threshold cost of this non-uniform cost interval grid
	 * 
	 * @see ThresholdRenderable#setThreshold(int)
	 */
	@Override
	public int getThreshold() {
		return this.thresholdCost;
	}
	
	/**
	 * Sets the threshold cost of this non-uniform cost interval grid.
	 * 
	 * @param thresholdCost the threshold cost of this non-uniform cost interval grid
	 * 
	 * @see ThresholdRenderable#setThreshold(int)
	 */
	@Override
	public void setThreshold(int thresholdCost) {
		this.thresholdCost = thresholdCost;
		this.updateVisibility();
		
		if (this.hasChildren()) {
			for (int r = 0; r < this.cells.length; r++) {
				for (int s = 0; s < this.cells[r].length; s++) {
					for (int t = 0; t < this.cells[r][s].length; t++) {
						((ThresholdRenderable) this.cells[r][s][t]).setThreshold(thresholdCost);
					}
				}
			}
		}
	}
	
	/**
	 * Updates this non-uniform cost interval grid.
	 */
	protected void update() {
		this.updateCostIntervals();
		this.updateActiveCost();
		this.updateAppearance();
		this.updateVisibility();
	}
	
	/**
	 * Updates the active cost intervals of this non-uniform cost interval grid.
	 */
	protected void updateCostIntervals() {
		this.activeCostIntervals = this.getCostIntervals(this.time);
	}
	
	/**
	 * Updates the accumulated active cost of this non-uniform cost interval grid.
	 */
	protected void updateActiveCost() {
		this.activeCost = 1; // default uniform cost
		
		Set<String> costIntervalIds = new HashSet<String>();
		for (Interval<ChronoZonedDateTime<?>> interval : this.activeCostIntervals) {
			CostInterval costInterval = (CostInterval) interval;
			
			// only add costs of different overlapping cost intervals
			if (!costIntervalIds.contains(costInterval.getId())) {
				costIntervalIds.add(costInterval.getId());
				this.activeCost += ((CostInterval) costInterval).getCost();
			}
		}
		// TODO: implement a proper weighted cost calculation normalized from 0 to 100
		// TODO: the weight is affected by severity (reporting method) and currency (reporting time)
	}
	
	/**
	 * Updates the visibility of this non-uniform cost interval grid.
	 */
	protected void updateVisibility() {
		this.setVisible(this.activeCost > this.thresholdCost);
	}
	
	/**
	 * Updates the appearance of this non-uniform cost interval grid.
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
	 * Embeds a geometric cylinder with an associated cost interval into this
	 * non-uniform cost interval grid.
	 * 
	 * @param cylinder the geometric cylinder to be embedded
	 * @param costInterval the associated cost interval
	 * 
	 * @return true if an embedding took place, false otherwise
	 */
	public boolean embed(Cylinder cylinder, CostInterval costInterval) {
		boolean embedded = false;
		
		if (this.intersectsCylinder(cylinder)) {
			this.addCostInterval(costInterval);
			this.embeddings.put(cylinder, costInterval);
			
			if (this.hasChildren()) {
				for (int r = 0; r < this.cells.length; r++) {
					for (int s = 0; s < this.cells[r].length; s++) {
						for (int t = 0; t < this.cells[r][s].length; t++) {
							NonUniformCostIntervalGrid child = (NonUniformCostIntervalGrid) this.cells[r][s][t];
							if (child.embed(cylinder, costInterval)) {
								this.addAffectedChild(costInterval, child);
							}
						}
					}
				}
			}
			embedded = true;
		}
		return embedded;
	}
	
	// TODO: embed all relevant kinds of (airspace, aircraft) rigid shapes
	// TODO: think about dynamically changing the resolution and embedded shapes
	// TODO: embedding should be a recursive operation starting at the root grid cell
	// TODO: only if parent grid is affected, propagation to children will occur
	// TODO: performance can be substantially improved by only considering relevant children for propagation
	
	/**
	 * Unembeds a geometric extent from this non-uniform cost interval grid.
	 * 
	 * @param extent the geometric extent to be unembedded
	 */
	public void unembed(Extent extent) {
		if (this.embeddings.containsKey(extent)) {
			CostInterval costInterval = this.embeddings.get(extent);
			this.removeCostInterval(costInterval);
			this.embeddings.remove(extent);
			
			if (this.affectedChildren.containsKey(costInterval)) {
				for (NonUniformCostIntervalGrid child : this.affectedChildren.get(costInterval)) {
					child.unembed(extent);
				}
				this.affectedChildren.remove(costInterval);
			}
		}
	}
	
	/**
	 * Adds an affected child to an embedding.
	 * 
	 * @param costInterval the cost interval of the embedding
	 * @param child the affected child
	 */
	private void addAffectedChild(CostInterval costInterval, NonUniformCostIntervalGrid child) {
		if (this.affectedChildren.containsKey(costInterval)) {
			this.affectedChildren.get(costInterval).add(child);
		} else {
			ArrayList<NonUniformCostIntervalGrid> children = new ArrayList<NonUniformCostIntervalGrid>();
			children.add(child);
			this.affectedChildren.put(costInterval, children);
		}
	}

	/**
	 * Adds the specified number of children on each axis to this non-uniform
	 * cost interval grid and propagates existing embeddings.
	 * 
	 * @param rCells the number of children on the <code>R</code> axis
	 * @param sCells the number of children on the <code>S</code> axis
	 * @param tCells the number of children on the <code>T</code> axis
	 * 
	 * @see RegularGrid#addChildren(int, int, int)
	 */
	@Override
	public void addChildren(int rCells, int sCells, int tCells) {
		super.addChildren(rCells, sCells, tCells);
		
		// propagate existing embeddings to children
		for (Extent extent : this.embeddings.keySet()) {
			for (int r = 0; r < this.cells.length; r++) {
				for (int s = 0; s < this.cells[r].length; s++) {
					for (int t = 0; t < this.cells[r][s].length; t++) {
						NonUniformCostIntervalGrid child = (NonUniformCostIntervalGrid) this.cells[r][s][t];
						if (extent instanceof Cylinder) {
							if (child.embed((Cylinder) extent, this.embeddings.get(extent))) {
								this.addAffectedChild(this.embeddings.get(extent), child);
							}
						}
						// TODO: implement propagations for other extents
					}
				}
			}
		}
	}

	/**
	 * Removes all children from this non-uniform cost interval grid.
	 */
	@Override
	public void removeChildren() {
		super.removeChildren();
		// remove affected children of all embeddings
		this.affectedChildren.clear();
	}
	
}
