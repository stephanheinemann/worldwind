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
package com.cfar.swim.worldwind.render;

import java.time.ZoneId;
import java.time.ZonedDateTime;

import com.cfar.swim.worldwind.planning.CostInterval;

import gov.nasa.worldwind.WorldWind;
import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.BasicShapeAttributes;
import gov.nasa.worldwind.render.Material;
import gov.nasa.worldwind.render.Path;

/**
 * Realizes an obstacle path for motion planning.
 * 
 * @author Stephan Heinemann
 *
 */
public class ObstaclePath extends Path implements TimedRenderable, ThresholdRenderable {

	/** the current time of this obstacle path */
	private ZonedDateTime time = ZonedDateTime.now(ZoneId.of("UTC"));
	
	/** the cost interval of this obstacle path */
	private CostInterval costInterval = new CostInterval("");
	
	/** the threshold cost of this obstacle path */
	private int thresholdCost = 0;
	
	/** the active cost of this obstacle path */
	private int activeCost = 0;
	
	/**
	 * Constructs an obstacle path with the specified waypoint positions.
	 * 
	 * @param positions the waypoint positions of this obstacle path
	 */
	public ObstaclePath(Iterable<? extends Position> positions) {
		super(positions);
		this.setAttributes(new BasicShapeAttributes());
		this.getAttributes().setOutlineOpacity(0.75);
		this.getAttributes().setOutlineWidth(2d);
		this.getAttributes().setEnableLighting(true);
		this.getAttributes().setDrawInterior(false);
		this.getAttributes().setDrawOutline(true);
		this.getAttributes().setOutlineMaterial(Material.PINK);
		
		this.setAltitudeMode(WorldWind.ABSOLUTE);
		this.setPathType(AVKey.GREAT_CIRCLE);
		this.setShowPositions(true);
	}
	
	/**
	 * Gets the cost interval of this obstacle path.
	 * 
	 * @return the cost interval of this obstacle path
	 */
	public CostInterval getCostInterval() {
		return this.costInterval;
	}
	
	/**
	 * Sets the cost interval of this obstacle path and updates its
	 * representation accordingly.
	 * 
	 * @param costInterval the cost interval of this obstacle path
	 */
	public void setCostInterval(CostInterval costInterval) {
		this.costInterval = costInterval;
		this.update();
	}
	
	/**
	 * Sets the threshold cost of this obstacle path and updates its
	 * representation accordingly.
	 * 
	 * @param threshold the threshold cost of this obstacle path
	 *
	 * @see ThresholdRenderable#setThreshold(int)
	 */
	@Override
	public void setThreshold(int threshold) {
		this.thresholdCost = threshold;
		this.updateVisibility();
	}

	/**
	 * Gets the threshold cost of this obstacle path.
	 * 
	 * @return the threshold cost of this obstacle path
	 * 
	 * @see ThresholdRenderable#getThreshold()
	 */
	@Override
	public int getThreshold() {
		return this.thresholdCost;
	}

	/**
	 * Gets the time of this obstacle path.
	 * 
	 * @return the time of this obstacle path
	 * 
	 * @see TimedRenderable#getTime()
	 */
	@Override
	public ZonedDateTime getTime() {
		return this.time;
	}

	/**
	 * Sets the time of this obstacle path.
	 * 
	 * @param time the time of this obstacle path
	 * 
	 * @see TimedRenderable#setTime(ZonedDateTime)
	 */
	@Override
	public void setTime(ZonedDateTime time) {
		this.time = time;
		this.update();
	}
	
	/**
	 * Updates this obstacle path.
	 */
	protected void update() {
		this.updateActiveCost();
		this.updateVisibility();
		//this.updateAppearance();
	}
	
	/**
	 * Updates the active cost of this obstacle path.
	 */
	protected void updateActiveCost() {
		if (this.costInterval.contains(this.time)) {
			this.activeCost = this.costInterval.getCost();
		} else {
			this.activeCost = 0;
		}
	}
	
	/**
	 * Updates the visibility of this obstacle path.
	 */
	protected void updateVisibility() {
		this.setVisible(this.activeCost > this.thresholdCost);
	}
	
	/**
	 * Updates the appearance of this obstacle path.
	 */
	protected void updateAppearance() {
		this.getAttributes().setOutlineMaterial(new Material(ObstacleColor.getColor(activeCost)));
		// TODO: elements could change color, transparency or even an associated image/icon 
	}

}
