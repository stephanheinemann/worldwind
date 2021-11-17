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

import java.time.Duration;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

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
import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Extent;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Sector;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.Material;
import gov.nasa.worldwind.render.Renderable;
import gov.nasa.worldwind.render.airspaces.AbstractAirspace;
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
	
	/** the depiction of this obstacle cylinder */
	protected Depiction depiction = null;
	
	/** the threshold cost of this obstacle cylinder */
	private double thresholdCost = 0d;
	
	/** the active cost of this obstacle cylinder */
	private double activeCost = 0d;
	
	/*
	 * TODO:
	 * Use ObstacleBox extends airspace box implements Obstacle to model
	 * altitude restrictions (min. altitudes, IFR, VFR altitudes). ThetaStar
	 * should plan vertically up and down through the costly airspace (CPDLC
	 * for landings) and perform string pulling at level. A desired altitude
	 * could be modeled as lowest cost altitude. Overlapping positive and
	 * negative cost airspaces could better indicate suitable landing and
	 * descent areas. Track airspaces could be useful too.
	 * PRM would require an IFR/VFR roadmap.
	 * 
	 * TODO: consider airspace Layer class
	 */
	
	/**
	 * Constructs a new obstacle box from a box.
	 * 
	 * @param box the box
	 */
	public ObstacleBox(Box box) {
		super(box);
	}
	
	/**
	 * Constructs a new obstacle box from a track line specified by two
	 * locations, a right and left width as well as a bottom and top altitude.
	 * 
	 * @param begin the begin location
	 * @param end the end location
	 * @param leftWidth the left width from the track line
	 * @param rightWidth the right width from the tack line
	 * @param bottom the bottom altitude
	 * @param top the top altitude
	 */
	public ObstacleBox(
			LatLon begin, LatLon end,
			double leftWidth, double rightWidth,
			double bottom, double top) {
		super(begin, end, leftWidth, rightWidth);
		this.setAltitudes(bottom, top);
		this.getAttributes().setInteriorOpacity(0.25);
		this.getAttributes().setDrawInterior(true);
		this.getAttributes().setDrawOutline(false);
	}
	
	/**
	 * Moves this obstacle box by adding a specified position.
	 * 
	 * @param position the position to be added to this obstacle box's position
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
	 * Moves this obstacle box to a specified position.
	 * 
	 * @param position the position to move this obstacle box to
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
	 * Indicates whether or not this obstacle box has a depiction.
	 * 
	 * @return true if this obstacle box has a depiction, false otherwise
	 * 
	 * @see Depictable#hasDepiction()
	 */
	@Override
	public boolean hasDepiction() {
		return (null != this.depiction);
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
		this.update();
	}
	
	/**
	 * Sets the threshold cost of this obstacle box and updates its
	 * representation accordingly.
	 * 
	 * @param threshold the threshold cost of this obstacle box
	 *
	 * @see ThresholdRenderable#setThreshold(double)
	 */
	@Override
	public void setThreshold(double threshold) {
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
	public double getThreshold() {
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
		if (null != this.depiction) {
			this.depiction.setVisible((0 != this.activeCost) && (this.activeCost > this.thresholdCost));
		}
	}
	
	/**
	 * Updates the appearance of this obstacle box.
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
	 * Renders this obstacle box.
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
	 * Interpolates the midpoint obstacle box between this and another
	 * obstacle box. The interpolation considers the spatial and temporal
	 * attributes and modifies this obstacle box accordingly.
	 * 
	 * @param other the other obstacle box
	 * @return the interpolated midpoint obstacle box between this and
	 *         the other obstacle box
	 */
	public ObstacleBox interpolate(ObstacleBox other) {
		LatLon beginLocation = (LatLon) Position.interpolateGreatCircle(0.5d, this.getLocations()[0], other.getLocations()[0]);
		LatLon endLocation = (LatLon) Position.interpolateGreatCircle(0.5d, this.getLocations()[1], other.getLocations()[1]);
		
		double leftWidth = (this.getWidths()[0] + other.getWidths()[0]) * 0.5d;
		double rightWdith = (this.getWidths()[1] + other.getWidths()[1]) * 0.5d;
		double bottom = (this.getAltitudes()[0] + other.getAltitudes()[0]) * 0.5d;
		double top = (this.getAltitudes()[1] + other.getAltitudes()[1]) * 0.5d;
		
		ObstacleBox interpolant = new ObstacleBox(beginLocation, endLocation, leftWidth, rightWdith, bottom, top);
		
		Duration startDuration = Duration.between(this.costInterval.getLower(), other.costInterval.getLower());
		startDuration = startDuration.dividedBy(2l);
		ZonedDateTime start = this.costInterval.getLower().plus(startDuration);
		
		// ZonedDateTime end = this.costInterval.getUpper();
		this.costInterval.setUpper(start);
		ZonedDateTime end = other.costInterval.getLower();
		
		/*
		Duration endDuration = Duration.between(this.costInterval.getUpper(), other.costInterval.getUpper());
		endDuration = endDuration.dividedBy(2l);
		ZonedDateTime end = this.costInterval.getUpper().plus(endDuration);
		*/
		
		// TODO: rounding or conservative ceiling might be more appropriate
		double cost = (this.costInterval.getCost() + other.costInterval.getCost()) / 2d;
		
		CostInterval costInterval = new CostInterval(this.costInterval.getId(), start, end, cost);
		interpolant.setCostInterval(costInterval);
		
		return interpolant;
	}
	
	/**
	 * Interpolates the midpoint obstacle boxes between this and another
	 * obstacle box. The interpolation considers the spatial and temporal
	 * attributes and modifies this obstacle box accordingly. It is
	 * performed recursively for the specified number of steps.
	 * 
	 * @param other the other obstacle box
	 * @param steps the number of interpolation steps
	 * @return the midpoint obstacle boxes between this and the other
	 *         obstacle box
	 */
	public List<ObstacleBox> interpolate(ObstacleBox other, int steps) {
		List<ObstacleBox> interpolants = new ArrayList<>();
		
		if (1 == steps) {
			interpolants.add(this.interpolate(other));
		} else if (1 < steps) {
			ObstacleBox interpolant = this.interpolate(other);
			interpolants.addAll(this.interpolate(interpolant, steps - 1));
			interpolants.add(interpolant);
			interpolants.addAll(interpolant.interpolate(other, steps - 1));
		}
		
		return interpolants;
	}
	
	/**
	 * Gets the center of this obstacle box disregarding its width.
	 * 
	 * @return the center of this obstacle box disregarding its width
	 * 
	 * @see Obstacle#getCenter()
	 */
	public Position getCenter() {
		LatLon centerLocation = Position.interpolateGreatCircle(
				0.5d, this.getLocations()[0], this.getLocations()[1]);
		double centerAltitude = this.getAltitudes()[0]
				+ ((this.getAltitudes()[1] - this.getAltitudes()[0]) * 0.5d);
		return new Position(centerLocation, centerAltitude);
	}
	
	/**
	 * Gets the sector of this obstacle box for a specified globe.
	 * 
	 * @param globe the globe to compute the sector
	 * 
	 * @return the sector of this obstacle box
	 */
	public Sector getSector(Globe globe) {
		Angle b2e = Position.greatCircleAzimuth(this.getLocations()[0], this.getLocations()[1]);
		Angle e2b = Position.greatCircleAzimuth(this.getLocations()[1], this.getLocations()[0]);
		Angle lwa = Angle.fromRadians(this.getWidths()[0] / globe.getRadiusAt(this.getCenter()));
		Angle rwa = Angle.fromRadians(this.getWidths()[1] / globe.getRadiusAt(this.getCenter()));
		LatLon lb = Position.greatCircleEndPosition(this.getLocations()[0], Angle.POS90.add(b2e).normalize(), lwa);
		LatLon re = Position.greatCircleEndPosition(this.getLocations()[1], Angle.POS90.add(e2b).normalize(), rwa);
		return Sector.boundingSector(lb, re);
	}
	
	/**
	 * Gets the geometric extent of this obstacle box for a specified globe.
	 * 
	 * @param globe the globe to be used for the conversion
	 * 
	 * @return the geometric extent of this obstacle box
	 * 
	 * @see AbstractAirspace#getExtent(Globe, double)
	 */
	@Override
	public gov.nasa.worldwind.geom.Box getExtent(Globe globe) {
		Angle b2e = Position.greatCircleAzimuth(this.getLocations()[0], this.getLocations()[1]);
		Angle e2b = Position.greatCircleAzimuth(this.getLocations()[1], this.getLocations()[0]);
		Angle lwa = Angle.fromRadians(this.getWidths()[0] / globe.getRadiusAt(this.getCenter()));
		Angle rwa = Angle.fromRadians(this.getWidths()[1] / globe.getRadiusAt(this.getCenter()));
		LatLon rb = Position.greatCircleEndPosition(this.getLocations()[0], Angle.POS90.add(b2e).normalize(), rwa);
		LatLon lb = Position.greatCircleEndPosition(this.getLocations()[0], Angle.NEG90.add(b2e).normalize(), lwa);
		LatLon re = Position.greatCircleEndPosition(this.getLocations()[1], Angle.NEG90.add(e2b).normalize(), rwa);
		LatLon le = Position.greatCircleEndPosition(this.getLocations()[1], Angle.POS90.add(e2b).normalize(), lwa);
		LatLon[] locations = {rb, lb, re, le};
		
		// TODO: consider high resolution terrain for sector minimum and maximum elevations
		List<Double> elevations = Arrays.stream(locations)
			.map(l -> globe.getElevation(l.getLatitude(), l.getLongitude()))
			.collect(Collectors.toList());
		
		double la = this.getAltitudes()[0];
		double ua = this.getAltitudes()[1];
		
		if (AVKey.ABOVE_GROUND_LEVEL.equals(this.getAltitudeDatum()[0])) {
			la += elevations.stream().min(Double::compareTo).get();
		}
		if (AVKey.ABOVE_GROUND_LEVEL.equals(this.getAltitudeDatum()[1])) {
			ua += elevations.stream().max(Double::compareTo).get();
		}
		
		List<Vec4> points = new ArrayList<>();
		points.add(globe.computePointFromPosition(rb, la));
		points.add(globe.computePointFromPosition(lb, la));
		points.add(globe.computePointFromPosition(re, la));
		points.add(globe.computePointFromPosition(le, la));
		points.add(globe.computePointFromPosition(this.getCenter(), ua));
		
		return gov.nasa.worldwind.geom.Box.computeBoundingBox(points);
		// TODO: super.getExtent(globe, 1d);
	}
	
	/**
	 * Gets the volume of the extent of this obstacle box for a specified
	 * globe.
	 * 
	 * @param globe the globe to be used for the conversion
	 * 
	 * @return the volume of the geometric extent of this obstacle box
	 * 
	 * @see Obstacle#getVolume(Globe)
	 */
	@Override
	public double getVolume(Globe globe) {
		double volume = 0d;
		
		Extent extent = this.getExtent(globe);
		if ((null != extent) && (extent instanceof Box)) {
			gov.nasa.worldwind.geom.Box box =
					(gov.nasa.worldwind.geom.Box) extent;
			volume = box.getRLength() * box.getSLength() * box.getTLength();
		}
		
		return volume;
	}
	
	/**
	 * Determines whether or not this obstacle box intersects another
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
	 * Determines whether or not this obstacle box equals another one based on
	 * their cost intervals.
	 * 
	 * @param o the other obstacle box
	 * 
	 * @return true if this obstacle box equals the other one based on their
	 *         cost intervals, false otherwise
	 * 
	 * @see Object#equals(Object)
	 */
	public final boolean equals(Object o) {
		boolean equals = false;
		
		if (this == o) {
			equals = true;
		} else if ((null != o) && (o instanceof ObstacleBox)) {
			ObstacleBox ob = (ObstacleBox) o;
			equals = this.getCostInterval().equals(ob.getCostInterval());
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this obstacle box based on its cost interval.
	 * 
	 * @return the hash code of this obstacle box based on its cost interval
	 * 
	 * @see Object#hashCode()
	 */
	public final int hashCode() {
		return this.getCostInterval().hashCode();
	}

}
