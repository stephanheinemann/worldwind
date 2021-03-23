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
import com.cfar.swim.worldwind.util.Depictable;
import com.cfar.swim.worldwind.util.Depiction;
import com.cfar.swim.worldwind.util.Enableable;

import gov.nasa.worldwind.Movable;
import gov.nasa.worldwind.WorldWind;
import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.geom.Extent;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.BasicShapeAttributes;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.Material;
import gov.nasa.worldwind.render.Path;
import gov.nasa.worldwind.render.Renderable;

/**
 * Realizes an obstacle path for motion planning.
 * 
 * @author Stephan Heinemann
 *
 */
public class ObstaclePath extends Path implements Obstacle {

	/** the current time of this obstacle path */
	private ZonedDateTime time = ZonedDateTime.now(ZoneId.of("UTC"));
	
	/** the cost interval of this obstacle path */
	private CostInterval costInterval = new CostInterval("");
	
	/** the deptiction of this obstacle path */
	protected Depiction depiction = null;
	
	/** the threshold cost of this obstacle path */
	private double thresholdCost = 0d;
	
	/** the active cost of this obstacle path */
	private double activeCost = 0d;
	
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
	 * Moves this obstacle path by adding a specified position.
	 * 
	 * @param position the position to be added to this obstacle path's position
	 * 
	 * @see Movable#move(Position)
	 */
	@Override
	public void move(Position position) {
		super.move(position);
		if (this.hasDepiction()) {
			this.depiction.move(position);
		}
	}
	
	/**
	 * Moves this obstacle path to a specified position.
	 * 
	 * @param position the position to move this obstacle path to
	 * 
	 * @see Movable#moveTo(Position)
	 */
	@Override
	public void moveTo(Position position) {
		super.moveTo(position);
		if (this.hasDepiction()) {
			this.depiction.moveTo(position);
		}
	}
	
	/**
	 * Gets the depiction of this obstacle path.
	 * 
	 * @return the depiction of this obstacle path
	 * 
	 * @see Depictable#getDepiction()
	 */
	@Override
	public Depiction getDepiction() {
		return this.depiction;
	}

	/**
	 * Sets the depiction of this obstacle path.
	 * 
	 * @param depiction the depiction of this obstacle path
	 * 
	 * @see Depictable#setDepiction(Depiction)
	 */
	@Override
	public void setDepiction(Depiction depiction) {
		this.depiction = depiction;
	}
	
	/**
	 * Indicates whether or not this obstacle path has a depiction.
	 * 
	 * @return true if this obstacle path has a depiction, false otherwise
	 * 
	 * @see Depictable#hasDepiction()
	 */
	@Override
	public boolean hasDepiction() {
		return (null != this.depiction);
	}
	
	/**
	 * Enables this obstacle path.
	 * 
	 * @see Enableable#enable()
	 */
	@Override
	public void enable() {
		this.costInterval.enable();
		this.update();
	}
	
	/**
	 * Disables this obstacle path.
	 * 
	 * @see Enableable#disable()
	 */
	@Override
	public void disable() {
		this.costInterval.disable();
		this.update();
	}
	
	/**
	 * Determines whether or not this obstacle path is enabled.
	 * 
	 * @return true if this obstacle path is enabled, false otherwise
	 * 
	 * @see Enableable#isEnabled()
	 */
	@Override
	public boolean isEnabled() {
		return this.costInterval.isEnabled();
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
	 * @see ThresholdRenderable#setThreshold(double)
	 */
	@Override
	public void setThreshold(double threshold) {
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
	public double getThreshold() {
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
		this.setVisible((0 != this.activeCost) && (this.activeCost > this.thresholdCost));
		if (null != this.depiction) {
			this.depiction.setVisible((0 != this.activeCost) && (this.activeCost > this.thresholdCost));
		}
	}
	
	/**
	 * Updates the appearance of this obstacle path.
	 */
	protected void updateAppearance() {
		this.getAttributes().setOutlineMaterial(new Material(ObstacleColor.getColor(activeCost)));
		// TODO: elements could change color, transparency or even an associated image/icon 
	}
	
	/**
	 * Renders this obstacle path.
	 * 
	 * @see Renderable#render(DrawContext)
	 */
	@Override
	public void render(DrawContext dc) {
		super.render(dc);
		if (null != this.depiction) {
			this.depiction.render(dc);
		}
	}
	
	/**
	 * Gets the geometric extent of this obstacle path for a specified globe.
	 * 
	 * @param globe the globe to be used for the conversion
	 * 
	 * @return the geometric extent of this obstacle path
	 * 
	 * @see Path#getExtent(Globe, double)
	 */
	@Override
	public Extent getExtent(Globe globe) {
		return super.getExtent(globe, 1d);
	}
	
	/**
	 * Determines whether or not this obstacle path equals another one based on
	 * their cost intervals.
	 * 
	 * @param o the other obstacle path
	 * 
	 * @return true if this obstacle path equals the other one based on their
	 *         cost intervals, false otherwise
	 * 
	 * @see Object#equals(Object)
	 */
	public final boolean equals(Object o) {
		boolean equals = false;
		
		if (this == o) {
			equals = true;
		} else if ((null != o) && (o instanceof ObstaclePath)) {
			ObstaclePath op = (ObstaclePath) o;
			equals = this.getCostInterval().equals(op.getCostInterval());
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this obstacle path based on its cost interval.
	 * 
	 * @return the hash code of this obstacle path based on its cost interval
	 * 
	 * @see Object#hashCode()
	 */
	public final int hashCode() {
		return this.getCostInterval().hashCode();
	}
	
}
