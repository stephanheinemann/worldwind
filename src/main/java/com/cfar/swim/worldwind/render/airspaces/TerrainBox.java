/**
 * Copyright (c) 2018, Henrique Ferreira (UVic Center for Aerospace Research)
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

import java.awt.Color;

import com.cfar.swim.worldwind.render.TerrainObstacle;
import com.cfar.swim.worldwind.render.ThresholdRenderable;
import com.cfar.swim.worldwind.util.Depictable;
import com.cfar.swim.worldwind.util.Depiction;

import gov.nasa.worldwind.geom.Extent;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.Material;
import gov.nasa.worldwind.render.Renderable;
import gov.nasa.worldwind.render.airspaces.AbstractAirspace;
import gov.nasa.worldwind.render.airspaces.Box;

/**
 * Realizes a terrain obstacle box for motion planning.
 * 
 * @author Henrique Ferreira
 *
 */
public class TerrainBox extends Box implements TerrainObstacle {

	/** the depiction of this terrain obstacle box */
	protected Depiction depiction = null;

	/** the threshold cost of this terrain obstacle box */
	private double thresholdCost = 0d;

	/** the active cost of this terrain obstacle box */
	private double activeCost = 99d;

	/** the status of this terrain terrain obstacle box */
	private boolean isEnabled = true;

	/**
	 * TODO: comment Constructs a terrain obstacle box considering two LatLon
	 * positions.
	 * 
	 * @param location1 the first position
	 * @param location2 the second position
	 * @param leftWidth the
	 * @param rightWidth
	 * @param bottom
	 * @param top
	 */
	public TerrainBox(
			LatLon location1, LatLon location2,
			double leftWidth, double rightWidth,
			double bottom, double top) {
		super(location1, location2, leftWidth, rightWidth);
		this.setAltitudes(bottom, top);
		this.getAttributes().setOpacity(0.6);
		this.getAttributes().setDrawInterior(true);
		this.getAttributes().setDrawOutline(true);
		this.update();
	}

	/**
	 * Sets the threshold cost of this terrain obstacle box and updates its
	 * representation accordingly.
	 * 
	 * @param threshold the threshold cost of this terrain obstacle box
	 *
	 * @see ThresholdRenderable#setThreshold(double)
	 */
	@Override
	public void setThreshold(double threshold) {
		this.thresholdCost = threshold;
		this.updateVisibility();
	}

	/**
	 * Gets the threshold cost of this terrain obstacle box.
	 * 
	 * @return the threshold cost of this terrain obstacle box
	 * 
	 * @see ThresholdRenderable#getThreshold()
	 */
	@Override
	public double getThreshold() {
		return this.thresholdCost;
	}

	/**
	 * Gets the depiction of this terrain obstacle box.
	 * 
	 * @return the depictin of this terrain obstacle box
	 * 
	 * @see Depictable#getDepiction()
	 */
	@Override
	public Depiction getDepiction() {
		return this.depiction;
	}

	/**
	 * Sets the depiction of this terrain obstacle box.
	 * 
	 * @param depiction the depiction of this terrain obstacle box
	 * 
	 * @see Depictable#setDepiction(Depiction)
	 */
	@Override
	public void setDepiction(Depiction depiction) {
		this.depiction = depiction;
	}

	/**
	 * Indicates whether or not this terrain obstacle box has a depiction.
	 * 
	 * @return true if this terrain obstacle box has a depiction, false otherwise
	 * 
	 * @see Depictable#hasDepiction()
	 */
	@Override
	public boolean hasDepiction() {
		return (null != this.depiction);
	}

	/**
	 * Updates this terrain obstacle box.
	 */
	protected void update() {
		this.updateVisibility();
		this.updateAppearance();
	}

	/**
	 * Updates the visibility of this terrain obstacle box.
	 */
	protected void updateVisibility() {
		this.setVisible((0 != this.activeCost) && (this.activeCost > this.thresholdCost));
		if (null != this.depiction) {
			this.depiction.setVisible((0 != this.activeCost) && (this.activeCost > this.thresholdCost));
		}
	}

	/**
	 * Updates the appearance of this terrain obstacle box.
	 */
	protected void updateAppearance() {
		this.getAttributes().setMaterial(new Material(Color.BLACK));
		this.getAttributes().setOutlineMaterial(new Material(Color.WHITE));
		// TODO: elements could change color, transparency or even an associated
		// image/icon
	}

	/**
	 * Renders this terrain obstacle box.
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
	 * Enables this terrain obstacle box.
	 * 
	 * @see com.cfar.swim.worldwind.util.Enableable#enable()
	 */
	@Override
	public void enable() {
		this.isEnabled = true;
		this.update();
	}

	/**
	 * Disables this terrain obstacle box.
	 * 
	 * @see com.cfar.swim.worldwind.util.Enableable#disable()
	 */
	@Override
	public void disable() {
		this.isEnabled = false;
		this.update();
	}

	/**
	 * Determines whether or not this terrain obstacle box is enabled
	 * 
	 * @return
	 * 
	 * @see com.cfar.swim.worldwind.util.Enableable#isEnabled()
	 */
	@Override
	public boolean isEnabled() {
		return this.isEnabled;
	}

	/**
	 * Gets the geometric extent of this terrain obstacle box for a specified globe.
	 * 
	 * @param globe the globe to be used for the conversion
	 * 
	 * @return the geometric extent of this obstacle box
	 * 
	 * @see AbstractAirspace#getExtent(Globe, double)
	 */
	@Override
	public Extent getExtent(Globe globe) {
		return super.getExtent(globe, 1d);
	}
}
