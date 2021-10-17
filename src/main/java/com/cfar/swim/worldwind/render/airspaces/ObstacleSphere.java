/**
 * Copyright (c) 2021, Stephan Heinemann (UVic Center for Aerospace Research)
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

import com.cfar.swim.worldwind.geom.Collisions;
import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.ObstacleColor;
import com.cfar.swim.worldwind.render.ThresholdRenderable;
import com.cfar.swim.worldwind.render.TimedRenderable;
import com.cfar.swim.worldwind.util.Depictable;
import com.cfar.swim.worldwind.util.Depiction;
import com.cfar.swim.worldwind.util.Enableable;

import gov.nasa.worldwind.Movable;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Sphere;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.Material;
import gov.nasa.worldwind.render.Renderable;
import gov.nasa.worldwind.render.airspaces.AbstractAirspace;
import gov.nasa.worldwind.render.airspaces.SphereAirspace;

/**
 * Realizes an obstacle sphere for motion planning.
 * 
 * @author Stephan Heinemann
 *
 */
public class ObstacleSphere extends SphereAirspace implements Obstacle {

	/** the current time of this obstacle sphere */
	private ZonedDateTime time = ZonedDateTime.now(ZoneId.of("UTC"));
	
	/** the cost interval of this obstacle sphere */
	private CostInterval costInterval = new CostInterval("");
	
	/** the depiction of this obstacle sphere */
	protected Depiction depiction = null;
	
	/** the threshold cost of this obstacle sphere */
	private double thresholdCost = 0d;
	
	/** the active cost of this obstacle sphere */
	private double activeCost = 0d;
	
	/**
	 * Constructs an obstacle sphere at the specified position with the
	 * specified radius.
	 * 
	 * @param center the center of this obstacle sphere
	 * @param radius the radius of this obstacle sphere in meters
	 * 
	 * @see SphereAirspace#SphereAirspace(LatLon, double)
	 */
	public ObstacleSphere(Position center, double radius) {
		super(LatLon.fromDegrees(center.getLatitude().degrees, center.getLongitude().degrees), radius);
		this.setAltitudes(center.getAltitude() - radius, center.getAltitude() + radius);
		this.getAttributes().setInteriorOpacity(0.25);
		this.getAttributes().setDrawInterior(true);
		this.getAttributes().setDrawOutline(false);
	}
	
	/**
	 * Moves this obstacle sphere by adding a specified position.
	 * 
	 * @param position the position to be added to this obstacle sphere's position
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
	 * Moves this obstacle sphere to a specified position.
	 * 
	 * @param position the position to move this obstacle sphere to
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
	 * Gets the depiction of this obstacle sphere.
	 * 
	 * @return the depiction of this obstacle sphere
	 * 
	 * @see Depictable#getDepiction()
	 */
	@Override
	public Depiction getDepiction() {
		return this.depiction;
	}

	/**
	 * Sets the depiction of this obstacle sphere.
	 * 
	 * @param depiction the depiction of this obstacle sphere
	 * 
	 * @see Depictable#setDepiction(Depiction)
	 */
	@Override
	public void setDepiction(Depiction depiction) {
		this.depiction = depiction;
	}
	
	/**
	 * Indicates whether or not this obstacle sphere has a depiction.
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
	 * Enables this obstacle sphere.
	 * 
	 * @see Enableable#enable()
	 */
	@Override
	public void enable() {
		this.costInterval.enable();
		this.update();
	}
	
	/**
	 * Disables this obstacle sphere.
	 * 
	 * @see Enableable#disable()
	 */
	@Override
	public void disable() {
		this.costInterval.disable();
		this.update();
	}
	
	/**
	 * Determines whether or not this obstacle sphere is enabled.
	 * 
	 * @return true if this obstacle sphere is enabled, false otherwise
	 * 
	 * @see Enableable#isEnabled()
	 */
	@Override
	public boolean isEnabled() {
		return this.costInterval.isEnabled();
	}
	
	/**
	 * Gets the cost interval of this obstacle sphere.
	 * 
	 * @return the cost interval of this obstacle sphere
	 */
	public CostInterval getCostInterval() {
		return this.costInterval;
	}
	
	/**
	 * Sets the cost interval of this obstacle sphere and updates its
	 * representation accordingly.
	 * 
	 * @param costInterval the cost interval of this obstacle sphere
	 */
	public void setCostInterval(CostInterval costInterval) {
		this.costInterval = costInterval;
		this.update();
	}
	
	/**
	 * Sets the threshold cost of this obstacle sphere and updates its
	 * representation accordingly.
	 * 
	 * @param threshold the threshold cost of this obstacle sphere
	 *
	 * @see ThresholdRenderable#setThreshold(double)
	 */
	@Override
	public void setThreshold(double threshold) {
		this.thresholdCost = threshold;
		this.updateVisibility();
	}

	/**
	 * Gets the threshold cost of this obstacle sphere.
	 * 
	 * @return the threshold cost of this obstacle sphere
	 * 
	 * @see ThresholdRenderable#getThreshold()
	 */
	@Override
	public double getThreshold() {
		return this.thresholdCost;
	}

	/**
	 * Gets the time of this obstacle sphere.
	 * 
	 * @return the time of this obstacle sphere
	 * 
	 * @see TimedRenderable#getTime()
	 */
	@Override
	public ZonedDateTime getTime() {
		return this.time;
	}

	/**
	 * Sets the time of this obstacle sphere.
	 * 
	 * @param time the time of this obstacle sphere
	 * 
	 * @see TimedRenderable#setTime(ZonedDateTime)
	 */
	@Override
	public void setTime(ZonedDateTime time) {
		this.time = time;
		this.update();
	}
	
	/**
	 * Updates this obstacle sphere.
	 */
	protected void update() {
		this.updateActiveCost();
		this.updateVisibility();
		this.updateAppearance();
	}
	
	/**
	 * Updates the active cost of this obstacle sphere.
	 */
	protected void updateActiveCost() {
		if (this.costInterval.contains(this.time)) {
			this.activeCost = this.costInterval.getCost();
		} else {
			this.activeCost = 0;
		}
	}
	
	/**
	 * Updates the visibility of this obstacle sphere.
	 */
	protected void updateVisibility() {
		this.setVisible((0 != this.activeCost) && (this.activeCost > this.thresholdCost));
		if (null != this.depiction) {
			this.depiction.setVisible((0 != this.activeCost) && (this.activeCost > this.thresholdCost));
		}
	}
	
	/**
	 * Updates the appearance of this obstacle sphere.
	 */
	protected void updateAppearance() {
		this.getAttributes().setInteriorMaterial(new Material(ObstacleColor.getColor(activeCost)));
		if (0 > this.activeCost) {
			this.getAttributes().setInteriorOpacity(1.0);
		} else {
			this.getAttributes().setInteriorOpacity(0.5);
		}
		// TODO: elements could change color, transparency or even an associated image/icon 
	}
	
	/**
	 * Renders this obstacle sphere.
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
	 * Gets the geometric extent of this obstacle sphere for a specified globe.
	 * 
	 * @param globe the globe to be used for the conversion
	 * 
	 * @return the geometric extent of this obstacle sphere
	 * 
	 * @see AbstractAirspace#getExtent(Globe, double)
	 */
	@Override
	public Sphere getExtent(Globe globe) {
		Vec4 centerPoint = globe.computePointFromPosition(
				this.getLocation(), this.getAltitudes()[0]);
		return new Sphere(centerPoint, this.getRadius());
		// TODO: return super.getExtent(globe, 1d);
	}
	
	/**
	 * Gets the volume of the extent of this obstacle sphere for a specified
	 * globe.
	 * 
	 * @param globe the globe to be used for the conversion
	 * 
	 * @return the volume of the geometric extent of this obstacle sphere
	 * 
	 * @see Obstacle#getVolume(Globe)
	 */
	@Override
	public double getVolume(Globe globe) {
		Sphere sphere = this.getExtent(globe);
		return (4d / 3d) * Math.PI
				* sphere.getRadius() * sphere.getRadius() * sphere.getRadius();
	}
	
	/**
	 * Determines whether or not this obstacle sphere intersects another
	 * obstacle for a specified globe.
	 * 
	 * @param globe the globe to be used for the conversion
	 * @param obstacle the other obstacle
	 * 
	 * @see Obstacle#intersects(Globe, Obstacle)
	 */
	@Override
	public boolean intersects(Globe globe, Obstacle obstacle) {
		return (this == obstacle) || Collisions.collide(
				this.getExtent(globe), obstacle.getExtent(globe));
	}
	
	// TODO: interpolation and geometric conversion methods
	
	/**
	 * Determines whether or not this obstacle sphere equals another one based
	 * on their cost intervals.
	 * 
	 * @param o the other obstacle sphere
	 * 
	 * @return true if this obstacle sphere equals the other one based on their
	 *         cost intervals, false otherwise
	 * 
	 * @see Object#equals(Object)
	 */
	public final boolean equals(Object o) {
		boolean equals = false;
		
		if (this == o) {
			equals = true;
		} else if ((null != o) && (o instanceof ObstacleSphere)) {
			ObstacleSphere os = (ObstacleSphere) o;
			equals = this.getCostInterval().equals(os.getCostInterval());
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this obstacle sphere based on its cost interval.
	 * 
	 * @return the hash code of this obstacle sphere based on its cost interval
	 * 
	 * @see Object#hashCode()
	 */
	public final int hashCode() {
		return this.getCostInterval().hashCode();
	}
	
}
