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
package com.cfar.swim.worldwind.render;

import com.cfar.swim.worldwind.util.Depictable;
import com.cfar.swim.worldwind.util.Depiction;

import gov.nasa.worldwind.geom.Extent;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.BasicShapeAttributes;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.Material;
import gov.nasa.worldwind.render.Renderable;
import gov.nasa.worldwind.render.RigidShape;

/**
 * Realizes a terrain obstacle cylinder for motion planning.
 * 
 * @author Henrique Ferreira
 *
 */
public class TerrainCylinder extends VerticalCylinder implements TerrainObstacle {

	/** the depiction of this terrain obstacle cylinder */
	protected Depiction depiction = null;

	/** the threshold cost of this terrain obstacle cylinder */
	private double thresholdCost = 0d;

	/** the active cost of this terrain obstacle cylinder */
	private double activeCost = 0d;

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
	public TerrainCylinder(Position centerPosition, double height, double radius) {
		super(centerPosition, height, radius);
		this.setAttributes(new BasicShapeAttributes());
		this.getAttributes().setInteriorOpacity(0.25);
		this.getAttributes().setEnableLighting(true);
		this.getAttributes().setDrawInterior(true);
		this.getAttributes().setDrawOutline(false);
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
		this.getAttributes().setInteriorMaterial(new Material(ObstacleColor.getColor(activeCost)));
		if (0 > this.activeCost) {
			this.getAttributes().setInteriorOpacity(1.0);
		} else {
			this.getAttributes().setInteriorOpacity(0.5);
		}
		// TODO: elements could change color, transparency or even an associated
		// image/icon
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
	 * Gets the geometric extent of this terrain obstacle cylinder for a specified globe.
	 * 
	 * @param globe the globe to be used for the conversion
	 * 
	 * @return the geometric extent (a bounding box) of this terrain obstacle cylinder
	 * 
	 * @see RigidShape#getExtent(Globe, double)
	 */
	@Override
	public Extent getExtent(Globe globe) {
		return super.getExtent(globe, 1d);
	}
}
