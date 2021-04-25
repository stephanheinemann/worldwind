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
import java.util.Arrays;

import com.cfar.swim.worldwind.render.TerrainObstacle;
import com.cfar.swim.worldwind.render.ThresholdRenderable;
import com.cfar.swim.worldwind.util.Depiction;

import gov.nasa.worldwind.geom.Extent;
import gov.nasa.worldwind.geom.Frustum;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Line;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.Material;
import gov.nasa.worldwind.render.Renderable;
import gov.nasa.worldwind.render.airspaces.AbstractAirspace;
import gov.nasa.worldwind.render.airspaces.Box;
import gov.nasa.worldwind.util.Logging;
import gov.nasa.worldwind.util.WWMath;

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
		
		return this.getBox(globe);
		// use of super.getExten doesn't rotate boxes
		// return super.getExtent(globe, 1d);
	}
	
	/**
	 * Gets the geometric frustum of this terrain obstacle box for a specified globe.
	 * 
	 * @param globe the globe to be used for the conversion
	 * 
	 * @return the geometric frustum of this obstacle box
	 * 
	 * @see com.cfar.swim.worldwind.geom.Box#getFrustum()
	 */
	@Override
	public Frustum getFrustum(Globe globe) {
		return (new com.cfar.swim.worldwind.geom.Box(this.getBox(globe))).getFrustum();
	}
	
	// TODO: this code is not available in the super class anymore
	protected static final int A_LOW_LEFT = 0;
    protected static final int A_LOW_RIGHT = 1;
    protected static final int A_UPR_LEFT = 2;
    protected static final int A_UPR_RIGHT = 3;
    protected static final int B_LOW_LEFT = 4;
    protected static final int B_LOW_RIGHT = 5;
    protected static final int B_UPR_LEFT = 6;
    protected static final int B_UPR_RIGHT = 7;
	
    // TODO: this code is not available in the super class anymore
	public static Vec4[] computeStandardVertices(Globe globe, double verticalExaggeration, TerrainBox box)
    {
        if (globe == null)
        {
            String message = Logging.getMessage("nullValue.GlobeIsNull");
            Logging.logger().severe(message);
            throw new IllegalArgumentException(message);
        }

        if (box == null)
        {
            String message = Logging.getMessage("nullValue.BoxIsNull");
            Logging.logger().severe(message);
            throw new IllegalArgumentException(message);
        }

        double[] altitudes = box.getAltitudes(verticalExaggeration);

        // Compute the Cartesian points of this Box's first and second locations at the upper and lower altitudes.
        Vec4 al = globe.computePointFromPosition(box.getLocations()[0], altitudes[0]);
        Vec4 au = globe.computePointFromPosition(box.getLocations()[0], altitudes[1]);
        Vec4 bl = globe.computePointFromPosition(box.getLocations()[1], altitudes[0]);
        Vec4 bu = globe.computePointFromPosition(box.getLocations()[1], altitudes[1]);

        // Compute vectors at the first and second locations that are perpendicular to the vector connecting the two
        // points and perpendicular to the Globe's normal at each point. These perpendicular vectors are used to
        // determine this Box's points to the left and right of its Cartesian points.
        Vec4 aNormal = globe.computeSurfaceNormalAtPoint(al);
        Vec4 bNormal = globe.computeSurfaceNormalAtPoint(bl);
        Vec4 ab = bl.subtract3(al).normalize3();
        Vec4 aPerp = ab.cross3(aNormal).normalize3();
        Vec4 bPerp = ab.cross3(bNormal).normalize3();

        Vec4[] vertices = new Vec4[8];
        vertices[TerrainBox.A_LOW_LEFT] = new Line(al, aPerp).getPointAt(-box.getWidths()[0]);
        vertices[TerrainBox.A_LOW_RIGHT] = new Line(al, aPerp).getPointAt(box.getWidths()[1]);
        vertices[TerrainBox.A_UPR_LEFT] = new Line(au, aPerp).getPointAt(-box.getWidths()[0]);
        vertices[TerrainBox.A_UPR_RIGHT] = new Line(au, aPerp).getPointAt(box.getWidths()[1]);
        vertices[TerrainBox.B_LOW_LEFT] = new Line(bl, bPerp).getPointAt(-box.getWidths()[0]);
        vertices[TerrainBox.B_LOW_RIGHT] = new Line(bl, bPerp).getPointAt(box.getWidths()[1]);
        vertices[TerrainBox.B_UPR_LEFT] = new Line(bu, bPerp).getPointAt(-box.getWidths()[0]);
        vertices[TerrainBox.B_UPR_RIGHT] = new Line(bu, bPerp).getPointAt(box.getWidths()[1]);
        return vertices;
    }
	
	/**
	 * Gets the geometric box of this terrain obstacle box for a specified globe.
	 * 
	 * @param globe the globe to be used for the conversion
	 * 
	 * @return the geometric box of this obstacle box
	 */
	public gov.nasa.worldwind.geom.Box getBox(Globe globe) {

		Vec4[] vertices = TerrainBox.computeStandardVertices(globe, 1d, this);

		Iterable<Vec4> verticesIterable = Arrays.asList(vertices);
		Vec4[] axes = WWMath.computePrincipalAxes(verticesIterable);
		
		// Compute r, s, t and min, max
		Vec4 r = axes[0];
		Vec4 s = axes[1];
		Vec4 t = axes[2];

		// Find the extremes along each axis.
		double minDotR = Double.MAX_VALUE;
		double maxDotR = -minDotR;
		double minDotS = Double.MAX_VALUE;
		double maxDotS = -minDotS;
		double minDotT = Double.MAX_VALUE;
		double maxDotT = -minDotT;

		verticesIterable = Arrays.asList(vertices);
		for (Vec4 p : verticesIterable) {
			if (p == null)
				continue;

			double pdr = p.dot3(r);
			if (pdr < minDotR)
				minDotR = pdr;
			if (pdr > maxDotR)
				maxDotR = pdr;

			double pds = p.dot3(s);
			if (pds < minDotS)
				minDotS = pds;
			if (pds > maxDotS)
				maxDotS = pds;

			double pdt = p.dot3(t);
			if (pdt < minDotT)
				minDotT = pdt;
			if (pdt > maxDotT)
				maxDotT = pdt;
		}

		if (maxDotR == minDotR)
			maxDotR = minDotR + 1;
		if (maxDotS == minDotS)
			maxDotS = minDotS + 1;
		if (maxDotT == minDotT)
			maxDotT = minDotT + 1;

		gov.nasa.worldwind.geom.Box geomBox = new gov.nasa.worldwind.geom.Box(axes, minDotR, maxDotR, minDotS, maxDotS,
				minDotT, maxDotT);

		return geomBox;
	}
}
