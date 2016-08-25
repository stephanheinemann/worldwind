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

import java.time.Duration;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.ObstacleColor;
import com.cfar.swim.worldwind.render.ThresholdRenderable;
import com.cfar.swim.worldwind.render.TimedRenderable;

import gov.nasa.worldwind.geom.Cylinder;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.GlobeAnnotation;
import gov.nasa.worldwind.render.Material;
import gov.nasa.worldwind.render.Renderable;
import gov.nasa.worldwind.render.airspaces.CappedCylinder;

/**
 * Realizes an obstacle cylinder for motion planning.
 * 
 * @author Stephan Heinemann
 *
 */
public class ObstacleCylinder extends CappedCylinder implements Obstacle, TimedRenderable, ThresholdRenderable {

	/** the current time of this obstacle cylinder */
	private ZonedDateTime time = ZonedDateTime.now(ZoneId.of("UTC"));
	
	/** the cost interval of this obstacle cylinder */
	private CostInterval costInterval = new CostInterval("");
	
	/** the text annotation of this obstacle cylinder */
	private GlobeAnnotation annotation = null;
	
	/** the threshold cost of this obstacle cylinder */
	private int thresholdCost = 0;
	
	/** the active cost of this obstacle cylinder */
	private int activeCost = 0;
	
	/**
	 * Constructs an obstacle cylinder at the specified location with the
	 * specified bottom and top altitudes, and outer radius.
	 * 
	 * @param location the location of this obstacle cylinder
	 * @param bottom the lower altitude of this obstacle cylinder in meters
	 * @param top the upper altitude of this obstacle cylinder in meters
	 * @param radius the outer radius of this obstacle cylinder in meters
	 * 
	 * @see CappedCylinder#CappedCylinder(LatLon, double)
	 */
	public ObstacleCylinder(LatLon location, double bottom, double top, double radius) {
		super(location, radius);
		this.setAltitudes(bottom, top);
		this.getAttributes().setOpacity(0.5);
		this.getAttributes().setDrawInterior(true);
		this.getAttributes().setDrawOutline(false);
	}
	
	/**
	 * Gets the cost interval of this obstacle cylinder.
	 * 
	 * @return the cost interval of this obstacle cylinder
	 */
	public CostInterval getCostInterval() {
		return this.costInterval;
	}
	
	/**
	 * Sets the cost interval of this obstacle cylinder and updates its
	 * representation accordingly.
	 * 
	 * @param costInterval the cost interval of this obstacle cylinder
	 */
	public void setCostInterval(CostInterval costInterval) {
		this.costInterval = costInterval;
		this.annotation = new GlobeAnnotation(costInterval.getId(), this.getReferencePosition());
		this.update();
	}
	
	/**
	 * Sets the threshold cost of this obstacle cylinder and updates its
	 * representation accordingly.
	 * 
	 * @param threshold the threshold cost of this obstacle cylinder
	 *
	 * @see ThresholdRenderable#setThreshold(int)
	 */
	@Override
	public void setThreshold(int threshold) {
		this.thresholdCost = threshold;
		this.updateVisibility();
	}

	/**
	 * Gets the threshold cost of this obstacle cylinder.
	 * 
	 * @return the threshold cost of this obstacle cylinder
	 * 
	 * @see ThresholdRenderable#getThreshold()
	 */
	@Override
	public int getThreshold() {
		return this.thresholdCost;
	}

	/**
	 * Gets the time of this obstacle cylinder.
	 * 
	 * @return the time of this obstacle cylinder
	 * 
	 * @see TimedRenderable#getTime()
	 */
	@Override
	public ZonedDateTime getTime() {
		return this.time;
	}

	/**
	 * Sets the time of this obstacle cylinder.
	 * 
	 * @param time the time of this obstacle cylinder
	 * 
	 * @see TimedRenderable#setTime(ZonedDateTime)
	 */
	@Override
	public void setTime(ZonedDateTime time) {
		this.time = time;
		this.update();
	}
	
	/**
	 * Updates this obstacle cylinder.
	 */
	protected void update() {
		this.updateActiveCost();
		this.updateVisibility();
		this.updateAppearance();
	}
	
	/**
	 * Updates the active cost of this obstacle cylinder.
	 */
	protected void updateActiveCost() {
		if (this.costInterval.contains(this.time)) {
			this.activeCost = this.costInterval.getCost();
		} else {
			this.activeCost = 0;
		}
	}
	
	/**
	 * Updates the visibility of this obstacle cylinder.
	 */
	protected void updateVisibility() {
		this.setVisible(this.activeCost > this.thresholdCost);
		if (null != annotation) {
			this.annotation.getAttributes().setVisible(this.activeCost > this.thresholdCost);
		}
	}
	
	/**
	 * Updates the appearance of this obstacle cylinder.
	 */
	protected void updateAppearance() {
		this.getAttributes().setMaterial(new Material(ObstacleColor.getColor(activeCost)));
		// TODO: elements could change color, transparency or even an associated image/icon 
	}
	
	/**
	 * Renders this obstacle cylinder.
	 * 
	 * @see Renderable#render(DrawContext)
	 */
	@Override
	public void render(DrawContext dc) {
		super.render(dc);
		if (null != this.annotation) {
			this.annotation.render(dc);
		}
	}

	/**
	 * Interpolates the midpoint obstacle cylinder between this and another
	 * obstacle cylinder. The interpolation considers the spatial and temporal
	 * attributes and modifies this obstacle cylinder accordingly.
	 * 
	 * @param other the other obstacle cylinder
	 * @return the interpolated midpoint obstacle cylinder between this and
	 *         the other obstacle cylinder
	 */
	public ObstacleCylinder interpolate(ObstacleCylinder other) {
		Position center = Position.interpolateGreatCircle(0.5d, this.getReferencePosition(), other.getReferencePosition());
		LatLon location = new LatLon(center.getLatitude(), center.getLongitude());
		
		double bottom = (this.getAltitudes()[0] + other.getAltitudes()[0]) * 0.5;
		double top = (this.getAltitudes()[1] + other.getAltitudes()[1]) * 0.5;
		double radius = (this.getRadii()[1] + other.getRadii()[1]) * 0.5;
		
		ObstacleCylinder interpolant = new ObstacleCylinder(location, bottom, top, radius);
		
		Duration startDuration = Duration.between(this.costInterval.getLower(), other.costInterval.getLower());
		startDuration = startDuration.dividedBy(2);
		ZonedDateTime start = this.costInterval.getLower().plus(startDuration);
		
		// ZonedDateTime end = this.costInterval.getUpper();
		this.costInterval.setUpper(start);
		ZonedDateTime end = other.costInterval.getLower();
		
		/*
		Duration endDuration = Duration.between(this.costInterval.getUpper(), other.costInterval.getUpper());
		endDuration = endDuration.dividedBy(2);
		ZonedDateTime end = this.costInterval.getUpper().plus(endDuration);
		*/
		
		// TODO: rounding or conservative ceiling might be more appropriate
		int cost = (this.costInterval.getCost() + other.costInterval.getCost()) / 2;
		
		CostInterval costInterval = new CostInterval(this.costInterval.getId(), start, end, cost);
		interpolant.setCostInterval(costInterval);
		
		return interpolant;
	}
	
	/**
	 * Interpolates the midpoint obstacle cylinders between this and another
	 * obstacle cylinder. The interpolation considers the spatial and temporal
	 * attributes and modifies this obstacle cylinder accordingly. It is
	 * performed recursively for the specified number of steps.
	 * 
	 * @param other the other obstacle cylinder
	 * @param steps the number of interpolation steps
	 * @return the midpoint obstacle cylinders between this and the other
	 *         obstacle cylinder
	 */
	public List<ObstacleCylinder> interpolate(ObstacleCylinder other, int steps) {
		List<ObstacleCylinder> interpolants = new ArrayList<>();
		
		if (1 == steps) {
			interpolants.add(this.interpolate(other));
		} else if (1 < steps) {
			ObstacleCylinder interpolant = this.interpolate(other);
			interpolants.addAll(this.interpolate(interpolant, steps - 1));
			interpolants.add(interpolant);
			interpolants.addAll(interpolant.interpolate(other, steps - 1));
		}
		
		return interpolants;
	}
	
	/**
	 * Converts this obstacle cylinder to a geometric cylinder.
	 * 
	 * @param globe the globe to be used for the conversion
	 * @return the geometric cylinder
	 */
	public Cylinder toGeometricCylinder(Globe globe) {
		Position bcp = new Position(this.getCenter(), this.getAltitudes()[0]);
		Position tcp = new Position(this.getCenter(), this.getAltitudes()[1]);
		Vec4 bottomCenter = globe.computePointFromPosition(bcp);
		Vec4 topCenter = globe.computePointFromPosition(tcp);
		double radius = this.getRadii()[1];
		
		return new Cylinder(bottomCenter, topCenter, radius);
	}

}
