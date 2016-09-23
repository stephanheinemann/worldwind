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
package com.cfar.swim.worldwind.render.airspaces;

import java.time.ZoneId;
import java.time.ZonedDateTime;

import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.ObstacleColor;
import com.cfar.swim.worldwind.render.ThresholdRenderable;
import com.cfar.swim.worldwind.render.TimedRenderable;
import com.cfar.swim.worldwind.util.Depictable;
import com.cfar.swim.worldwind.util.Depiction;
import com.cfar.swim.worldwind.util.Enableable;

import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.GlobeAnnotation;
import gov.nasa.worldwind.render.Material;
import gov.nasa.worldwind.render.Renderable;
import gov.nasa.worldwind.render.airspaces.Box;

/**
 * Realizes an obstacle box for motion planning.
 * 
 * @author Stephan Heinemann
 *
 */
public class ObstacleBox extends Box implements Obstacle {

	/** the current time of this obstacle cylinder */
	private ZonedDateTime time = ZonedDateTime.now(ZoneId.of("UTC"));
	
	/** the cost interval of this obstacle cylinder */
	private CostInterval costInterval = new CostInterval("");
	
	/** the text annotation of this obstacle cylinder */
	protected GlobeAnnotation annotation = null;
	
	/** the depiction of this obstacle cylinder */
	protected Depiction depiction = null;
	
	/** the threshold cost of this obstacle cylinder */
	private int thresholdCost = 0;
	
	/** the active cost of this obstacle cylinder */
	private int activeCost = 0;
	
	/*
	 * TODO:
	 * Use ObstacleBox extends airspace box implements Obstacle to model altitude restrictions (min. altitudes, IFR, VFR altitudes). ThetaStar should plan vertically up and down through the costly airspace (CPDLC for landings) and perform string pulling at level. A desired altitude could be modeled as lowest cost altitude.
	 * Overlapping positive and negative cost airspaces could better indicate suitable landing and descent areas.
	 * Track airspaces could be useful too.
	 * PRM would require an IFR/VFR roadmap.
	 * 
	 * TODO: consider airspace Layer class
	 */
	
	public ObstacleBox(
			LatLon location1, LatLon location2,
			double leftWidth, double rightWidth,
			double bottom, double top) {
		super(location1, location2, leftWidth, rightWidth);
		this.setAltitudes(bottom, top);
		this.getAttributes().setOpacity(0.25);
		this.getAttributes().setDrawInterior(true);
		this.getAttributes().setDrawOutline(false);
	}
	
	/**
	 * Gets the depiction of this obstacle box.
	 * 
	 * @return the depiction of this obstacle box
	 * 
	 * @see Depictable#getDepiction()
	 */
	@Override
	public Depiction getDepiction() {
		return this.depiction;
	}

	/**
	 * Sets the depiction of this obstacle box.
	 * 
	 * @param depiction the depiction of this obstacle box
	 * 
	 * @see Depictable#setDepiction(Depiction)
	 */
	@Override
	public void setDepiction(Depiction depiction) {
		this.depiction = depiction;
	}
	
	/**
	 * Enables this obstacle box.
	 * 
	 * @see Enableable#enable()
	 */
	@Override
	public void enable() {
		this.costInterval.enable();
		this.update();
	}
	
	/**
	 * Disables this obstacle box.
	 * 
	 * @see Enableable#disable()
	 */
	@Override
	public void disable() {
		this.costInterval.disable();
		this.update();
	}
	
	/**
	 * Determines whether or not this obstacle box is enabled.
	 * 
	 * @return true if this obstacle box is enabled, false otherwise
	 * 
	 * @see Enableable#isEnabled()
	 */
	@Override
	public boolean isEnabled() {
		return this.costInterval.isEnabled();
	}
	
	/**
	 * Gets the cost interval of this obstacle box.
	 * 
	 * @return the cost interval of this obstacle box
	 */
	public CostInterval getCostInterval() {
		return this.costInterval;
	}
	
	/**
	 * Sets the cost interval of this obstacle box and updates its
	 * representation accordingly.
	 * 
	 * @param costInterval the cost interval of this obstacle box
	 */
	public void setCostInterval(CostInterval costInterval) {
		this.costInterval = costInterval;
		this.annotation = new GlobeAnnotation(costInterval.getId(), this.getReferencePosition());
		this.update();
	}
	
	/**
	 * Sets the threshold cost of this obstacle box and updates its
	 * representation accordingly.
	 * 
	 * @param threshold the threshold cost of this obstacle box
	 *
	 * @see ThresholdRenderable#setThreshold(int)
	 */
	@Override
	public void setThreshold(int threshold) {
		this.thresholdCost = threshold;
		this.updateVisibility();
	}

	/**
	 * Gets the threshold cost of this obstacle box.
	 * 
	 * @return the threshold cost of this obstacle box
	 * 
	 * @see ThresholdRenderable#getThreshold()
	 */
	@Override
	public int getThreshold() {
		return this.thresholdCost;
	}

	/**
	 * Gets the time of this obstacle box.
	 * 
	 * @return the time of this obstacle box
	 * 
	 * @see TimedRenderable#getTime()
	 */
	@Override
	public ZonedDateTime getTime() {
		return this.time;
	}

	/**
	 * Sets the time of this obstacle box.
	 * 
	 * @param time the time of this obstacle box
	 * 
	 * @see TimedRenderable#setTime(ZonedDateTime)
	 */
	@Override
	public void setTime(ZonedDateTime time) {
		this.time = time;
		this.update();
	}
	
	/**
	 * Updates this obstacle box.
	 */
	protected void update() {
		this.updateActiveCost();
		this.updateVisibility();
		this.updateAppearance();
	}
	
	/**
	 * Updates the active cost of this obstacle box.
	 */
	protected void updateActiveCost() {
		if (this.costInterval.contains(this.time)) {
			this.activeCost = this.costInterval.getCost();
		} else {
			this.activeCost = 0;
		}
	}
	
	/**
	 * Updates the visibility of this obstacle box.
	 */
	protected void updateVisibility() {
		this.setVisible((0 != this.activeCost) && (this.activeCost > this.thresholdCost));
		if (null != this.annotation) {
			this.annotation.getAttributes().setVisible((0 != this.activeCost) && (this.activeCost > this.thresholdCost));
		}
		if (null != this.depiction) {
			this.depiction.setVisible((0 != this.activeCost) && (this.activeCost > this.thresholdCost));
		}
	}
	
	/**
	 * Updates the appearance of this obstacle box.
	 */
	protected void updateAppearance() {
		this.getAttributes().setMaterial(new Material(ObstacleColor.getColor(activeCost)));
		if (0 > this.activeCost) {
			this.getAttributes().setOpacity(1.0);
		} else {
			this.getAttributes().setOpacity(0.5);
		}
		// TODO: elements could change color, transparency or even an associated image/icon 
	}
	
	/**
	 * Renders this obstacle box.
	 * 
	 * @see Renderable#render(DrawContext)
	 */
	@Override
	public void render(DrawContext dc) {
		super.render(dc);
		if (null != this.annotation) {
			this.annotation.render(dc);
		}
		if (null != this.depiction) {
			this.depiction.render(dc);
		}
	}
	
	// TODO: interpolation and geometric conversion methods

}
