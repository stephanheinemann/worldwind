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
import com.cfar.swim.worldwind.render.VerticalCylinder;
import com.cfar.swim.worldwind.util.Depictable;
import com.cfar.swim.worldwind.util.Depiction;

import gov.nasa.worldwind.geom.Cylinder;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.Material;
import gov.nasa.worldwind.render.Renderable;
import gov.nasa.worldwind.render.airspaces.AbstractAirspace;
import gov.nasa.worldwind.render.airspaces.CappedCylinder;

/**
 * Realizes a terrain obstacle cylinder for motion planning.
 * 
 * @author Henrique Ferreira
 *
 */
public class TerrainCylinder extends CappedCylinder implements TerrainObstacle {

	/** the depiction of this terrain obstacle cylinder */
	protected Depiction depiction = null;

	/** the threshold cost of this terrain obstacle cylinder */
	private double thresholdCost = 0d;

	/** the active cost of this terrain obstacle cylinder */
	private double activeCost = 99d;
	
	/** the status of this terrain obstacle cylinder */
	private boolean isEnabled = true;

	/**
	 * Constructs a terrain obstacle cylinder with a specified center position,
	 * height and radius.
	 * 
	 * @param centerPosition the center position of this terrain obstacle cylinder
	 * @param height the height in meters of this terrain obstacle cylinder
	 * @param radius the radius in meters of this terrain obstacle cylinder
	 * 
	 * @see VerticalCylinder#VerticalCylinder(Position, double, double)
	 */
	public TerrainCylinder(LatLon location, double bottom, double top, double radius) {
		super(location, radius);
		this.setAltitudes(bottom, top);
		this.getAttributes().setOpacity(0.6);
		this.getAttributes().setDrawInterior(true);
		this.getAttributes().setDrawOutline(true);
		this.update();
	}

	/**
	 * Sets the threshold cost of this terrain obstacle cylinder and updates its
	 * representation accordingly.
	 * 
	 * @param threshold the threshold cost of this terrain obstacle cylinder
	 *
	 * @see ThresholdRenderable#setThreshold(double)
	 */
	@Override
	public void setThreshold(double threshold) {
		this.thresholdCost = threshold;
		this.updateVisibility();
	}

	/**
	 * Gets the threshold cost of this terrain obstacle cylinder.
	 * 
	 * @return the threshold cost of this terrain obstacle cylinder
	 * 
	 * @see ThresholdRenderable#getThreshold()
	 */
	@Override
	public double getThreshold() {
		return this.thresholdCost;
	}

	/**
	 * Gets the depiction of this terrain obstacle cylinder.
	 * 
	 * @return the depictin of this terrain obstacle cylinder
	 * 
	 * @see Depictable#getDepiction()
	 */
	@Override
	public Depiction getDepiction() {
		return this.depiction;
	}

	/**
	 * Sets the depiction of this terrain obstacle cylinder.
	 * 
	 * @param depiction the depiction of this terrain obstacle cylinder
	 * 
	 * @see Depictable#setDepiction(Depiction)
	 */
	@Override
	public void setDepiction(Depiction depiction) {
		this.depiction = depiction;
	}

	/**
	 * Indicates whether or not this terrain obstacle cylinder has a depiction.
	 * 
	 * @return true if this terrain obstacle cylinder has a depiction, false otherwise
	 * 
	 * @see Depictable#hasDepiction()
	 */
	@Override
	public boolean hasDepiction() {
		return (null != this.depiction);
	}

	/**
	 * Updates this terrain obstacle cylinder.
	 */
	protected void update() {
		this.updateVisibility();
		this.updateAppearance();
	}

	/**
	 * Updates the visibility of this terrain obstacle cylinder.
	 */
	protected void updateVisibility() {
		this.setVisible((0 != this.activeCost) && (this.activeCost > this.thresholdCost));
		if (null != this.depiction) {
			this.depiction.setVisible((0 != this.activeCost) && (this.activeCost > this.thresholdCost));
		}
	}

	/**
	 * Updates the appearance of this terrain obstacle cylinder.
	 */
	protected void updateAppearance() {
		this.getAttributes().setMaterial(new Material(Color.BLACK));
		this.getAttributes().setOutlineMaterial(new Material(Color.WHITE));
		// TODO: elements could change color, transparency or even an associated image/icon 
	}

	/**
	 * Renders this terrain obstacle cylinder.
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
	 * Gets the geometric extent of this obstacle cylinder for a specified globe.
	 * 
	 * @param globe the globe to be used for the conversion
	 * 
	 * @return the geometric extent of this obstacle cylinder
	 * 
	 * @see AbstractAirspace#getExtent(Globe, double)
	 */
	@Override
	public Cylinder getExtent(Globe globe) {
		Position bcp = new Position(this.getCenter(), this.getAltitudes()[0]);
		Position tcp = new Position(this.getCenter(), this.getAltitudes()[1]);
		Vec4 bottomCenter = globe.computePointFromPosition(bcp);
		Vec4 topCenter = globe.computePointFromPosition(tcp);
		double radius = this.getRadii()[1];
		return new Cylinder(bottomCenter, topCenter, radius);
		// TODO: check available method
		//return super.getExtent(globe, 1d);
	}

	/**
	 * Enables this terrain obstacle cylinder.
	 * 
	 * @see com.cfar.swim.worldwind.util.Enableable#enable()
	 */
	@Override
	public void enable() {
		this.isEnabled = true;
		this.update();
	}

	/**
	 * Disables this terrain obstacle cylinder.
	 * 
	 * @see com.cfar.swim.worldwind.util.Enableable#disable()
	 */
	@Override
	public void disable() {
		this.isEnabled = false;
		this.update();
	}

	/**
	 * Determines whether or not this terrain obstacle cylinder is enabled
	 * @return
	
	 * @see com.cfar.swim.worldwind.util.Enableable#isEnabled()
	 */
	@Override
	public boolean isEnabled() {
		return this.isEnabled;
	}
}
