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
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import com.binarydreamers.trees.Interval;
import com.binarydreamers.trees.IntervalTree;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.geom.RegularGrid;
import com.cfar.swim.worldwind.render.CostColor;
import com.cfar.swim.worldwind.render.ThresholdRenderable;
import com.cfar.swim.worldwind.render.TimedRenderable;

import gov.nasa.worldwind.geom.Cylinder;
import gov.nasa.worldwind.geom.Vec4;

public class NonUniformCostIntervalGrid extends RegularGrid implements TimedRenderable, ThresholdRenderable {

	private IntervalTree<ChronoZonedDateTime<?>> costIntervals = new IntervalTree<ChronoZonedDateTime<?>>(CostInterval.comparator);
	
	// TODO: a runnable or slider could update the time, or renderCell itself depending on the mode
	private ZonedDateTime time = ZonedDateTime.now(ZoneId.of("UTC"));
	
	private List<Interval<ChronoZonedDateTime<?>>> activeCostIntervals = this.getCostIntervals(time);
	
	private int activeCost = 1;
	// TODO: a slider to update threshold
	private int thresholdCost = 0;
	
	public NonUniformCostIntervalGrid(Box box) {
		super(box);
	}
	
	public NonUniformCostIntervalGrid(Box box, int rCells, int sCells, int tCells) {
		super(box, rCells, sCells, tCells);
	}
	
	public NonUniformCostIntervalGrid(
			Vec4[] axes,
			double rMin, double rMax,
			double sMin, double sMax,
			double tMin, double tMax) {
		super(axes, rMin, rMax, sMin, sMax, tMin, tMax);
	}
	
	public NonUniformCostIntervalGrid(Vec4[] axes,
			double rMin, double rMax,
			double sMin, double sMax,
			double tMin, double tMax,
			int rCells, int sCells, int tCells) {
		super(axes, rMin, rMax, sMin, sMax, tMin, tMax, rCells, sCells, tCells);
	}
	
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
	
	@Override
	@SuppressWarnings("unchecked")
	public List<? extends NonUniformCostIntervalGrid> lookupCells(Vec4 modelPoint) {
		return (List<NonUniformCostIntervalGrid>) super.lookupCells(modelPoint);
	}
	
	public void addCostInterval(CostInterval costInterval) {
		this.costIntervals.add(costInterval);
		this.update();
		// TODO: costs should be automatically propagated to the affected child cells?
		// TODO: should added children costs propagated to parents?
		// TODO: what happens if children are added and removed
		// TODO: shall parents aggregate all costs?!
	}
	
	public void removeCostInterval(CostInterval costInterval) {
		this.costIntervals.remove(costInterval);
		this.update();
	}
	
	public List<Interval<ChronoZonedDateTime<?>>> getCostIntervals(ZonedDateTime time) {
		return this.costIntervals.searchInterval(new CostInterval(null, this.time));
	}
	
	public List<Interval<ChronoZonedDateTime<?>>> getCostIntervals(ZonedDateTime start, ZonedDateTime end) {
		return this.costIntervals.searchInterval(new CostInterval(null, start, end));
	}
	
	/* (non-Javadoc)
	 * @see com.cfar.swim.worldwind.render.TimedRenderable#getTime()
	 */
	@Override
	public ZonedDateTime getTime() {
		return this.time;
	}
	
	/* (non-Javadoc)
	 * @see com.cfar.swim.worldwind.render.TimedRenderable#setTime(java.time.ZonedDateTime)
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
	
	/* (non-Javadoc)
	 * @see com.cfar.swim.worldwind.render.ThresholdRenderable#setThreshold(int)
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
	
	/* (non-Javadoc)
	 * @see com.cfar.swim.worldwind.render.ThresholdRenderable#getThreshold()
	 */
	@Override
	public int getThreshold() {
		return this.thresholdCost;
	}
	
	protected void update() {
		this.updateCostIntervals();
		this.updateActiveCost();
		this.updateAppearance();
		this.updateVisibility();
	}
	
	protected void updateCostIntervals() {
		this.activeCostIntervals = this.getCostIntervals(this.time);
	}
	
	protected void updateActiveCost() {
		this.activeCost = 1; // default uniform cost
		
		Set<String> costIntervalIds = new HashSet<String>();
		for (Interval<ChronoZonedDateTime<?>> interval : this.activeCostIntervals) {
			CostInterval costInterval = (CostInterval) interval;
			
			// only add costs of different overlapping cost intervals
			if (!costIntervalIds.contains(costInterval.getId())) {
				costIntervalIds.add(costInterval.getId());
				this.activeCost += ((CostInterval) costInterval).getWeightedCost();
			}
		}
		// TODO: implement a proper weighted cost calculation normalized from 0 to 100
		// TODO: the weight is affected by severity and currency (reporting time)
	}
	
	protected void updateVisibility() {
		this.setVisible(this.activeCost > this.thresholdCost);
	}
	
	protected void updateAppearance() {
		Color activeColor = CostColor.getColor(activeCost);
		float red = activeColor.getRed() / 255.0f;
		float green = activeColor.getGreen() / 255.0f;
		float blue = activeColor.getBlue() / 255.0f;
		float alpha = activeColor.getAlpha() / 255.0f;
		this.setColor(red, green, blue, alpha);
	}
	
	public void embed(Cylinder cylinder, CostInterval costInterval) {
		if (this.intersectsCylinder(cylinder)) {
			this.addCostInterval(costInterval);
			
			if (this.hasChildren()) {
				for (int r = 0; r < this.cells.length; r++) {
					for (int s = 0; s < this.cells[r].length; s++) {
						for (int t = 0; t < this.cells[r][s].length; t++) {
							((NonUniformCostIntervalGrid) this.cells[r][s][t]).embed(cylinder, costInterval);
						}
					}
				}
			}
		}
	}
	
	// TODO: embed all relevant kinds of (airspace) rigid shapes
	// TODO: think about dynamically changing the resolution and embedded shapes
	// TODO: embedding should be a recursive operation starting at the root grid cell
	// TODO: only if parent grid is affected, propagation to children will occur

}
