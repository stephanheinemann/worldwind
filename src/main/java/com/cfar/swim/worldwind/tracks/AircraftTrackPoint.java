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
package com.cfar.swim.worldwind.tracks;

import java.time.Duration;
import java.time.ZoneId;
import java.time.ZonedDateTime;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.Material;
import gov.nasa.worldwind.render.markers.BasicMarker;
import gov.nasa.worldwind.render.markers.BasicMarkerAttributes;
import gov.nasa.worldwind.render.markers.BasicMarkerShape;
import gov.nasa.worldwind.render.markers.MarkerAttributes;
import gov.nasa.worldwind.tracks.TrackPoint;

/**
 * Realizes an aircraft track point on an actually flown track obtained via a
 * datalink downlink.
 * 
 * @author Stephan Heinemann
 *
 */
public class AircraftTrackPoint extends BasicMarker implements TrackPoint {
	
	/** the time at this aircraft track point */
	private ZonedDateTime time = ZonedDateTime.now(ZoneId.of("UTC"));
	
	/** the maximum age to show this aircraft track point */
	private Duration maxAge = Duration.ofSeconds(10);
	
	/**
	 * Constructs a new aircraft track point at a specified position.
	 * 
	 * @param position the position of this aircraft track point
	 * 
	 * @see BasicMarker#BasicMarker(Position, MarkerAttributes)
	 */
	public AircraftTrackPoint(Position position) {
		super(position, new BasicMarkerAttributes(
				Material.GREEN,
				BasicMarkerShape.ORIENTED_SPHERE, 1d));
		this.getAttributes().setHeadingMaterial(Material.RED);
		this.setPitch(Angle.ZERO);
		this.setRoll(Angle.ZERO);
		this.setHeading(Angle.ZERO);
	}
	
	/**
	 * Constructs a new aircraft track point at a specified position
	 * with an aircraft attitude.
	 * 
	 * @param position the position of this aircraft track point
	 * @param attitude the attitude of this aircraft track point
	 * 
	 * @see BasicMarker#BasicMarker(Position, MarkerAttributes, Angle)
	 */
	public AircraftTrackPoint(Position position, AircraftAttitude attitude) {
		super(position, new BasicMarkerAttributes(
				Material.GREEN,
				BasicMarkerShape.ORIENTED_SPHERE, 1d));
		this.getAttributes().setHeadingMaterial(Material.RED);
		this.setPitch(attitude.getPitch());
		this.setRoll(attitude.getBank());
		this.setHeading(attitude.getHeading());
	}
	
	/**
	 * Constructs a new aircraft track point at a specified position with
	 * marker attributes.
	 * 
	 * @param position the position of this aircraft track point
	 * @param attributes the attributes of this aircraft track point
	 * 
	 * @see BasicMarker#BasicMarker(Position, MarkerAttributes)
	 */
	public AircraftTrackPoint(Position position, MarkerAttributes attributes) {
		super(position, attributes);
		this.setPitch(Angle.ZERO);
		this.setRoll(Angle.ZERO);
		this.setHeading(Angle.ZERO);
	}
	
	/**
	 * Constructs a new aircraft track point at a specified position,
	 * with an aircraft attitude and marker attributes.
	 * 
	 * @param position the position of this aircraft track point
	 * @param attitude the attitude of this aircraft track point
	 * @param attributes the attributes of this aircraft track point
	 * 
	 * @see BasicMarker#BasicMarker(Position, MarkerAttributes, Angle)
	 */
	public AircraftTrackPoint(
			Position position, AircraftAttitude attitude, MarkerAttributes attributes) {
		super(position, attributes);
		this.setPitch(attitude.getPitch());
		this.setRoll(attitude.getBank());
		this.setHeading(attitude.getHeading());
	}
	
	/**
	 * Gets the bank angle of this aircraft track point.
	 * 
	 * @return the bank angle of this aircraft track point
	 */
	public Angle getBank() {
		return this.getRoll();
	}
	
	/**
	 * Sets the bank angle of this aircraft track point.
	 * 
	 * @param bank the bank angle to be set
	 */
	public void setBank(Angle bank) {
		this.setRoll(bank);
	}
	
	/**
	 * Gets the yaw angle of this aircraft track point.
	 * 
	 * @return the yaw angle of this aircraft track point
	 */
	public Angle getYaw() {
		return this.getHeading();
	}
	
	/**
	 * Sets the yaw angle of this aircraft track point.
	 * 
	 * @param yaw the yaw angle to be set
	 */
	public void setYaw(Angle yaw) {
		this.setHeading(yaw);
	}
	
	/**
	 * Gets the latitude of this aircraft track point.
	 * 
	 * @return the latitude of this aircraft track point
	 * 
	 * @see TrackPoint#getLatitude()
	 */
	@Override
	public double getLatitude() {
		return this.getPosition().getLatitude().getDegrees();
	}
	
	/**
	 * Sets the latitude of this aircraft track point.
	 * 
	 * @param latitude the latitude to be set
	 * 
	 * @see TrackPoint#setLatitude(double)
	 */
	@Override
	public void setLatitude(double latitude) {
		this.position = new Position(
				Angle.fromDegrees(latitude),
				this.getPosition().getLongitude(),
				this.getElevation());
	}
	
	/**
	 * Gets the longitude of this aircraft track point.
	 * 
	 * @return the longitude of this aircraft track point
	 * 
	 * @see TrackPoint#getLongitude()
	 */
	@Override
	public double getLongitude() {
		return this.getPosition().getLatitude().getDegrees();
	}
	
	/**
	 * Sets the longitude of this aircraft track point.
	 * 
	 * @param longitude the longitude to be set
	 * 
	 * @see TrackPoint#setLongitude(double)
	 */
	@Override
	public void setLongitude(double longitude) {
		this.position = new Position(
				this.getPosition().getLatitude(),
				Angle.fromDegrees(longitude),
				this.getElevation());
	}
	
	/**
	 * Gets the elevation of this aircraft track point.
	 * 
	 * @return the elevation of this aircraft track point
	 * 
	 * @see TrackPoint#getElevation()
	 */
	@Override
	public double getElevation() {
		return this.getPosition().getElevation();
	}
	
	/**
	 * Sets the elevation of this aircraft track point.
	 * 
	 * @param elevation the elevation to be set
	 * 
	 * @see TrackPoint#setElevation(double)
	 */
	@Override
	public void setElevation(double elevation) {
		this.position = new Position(
				this.getPosition().getLatitude(),
				this.getPosition().getLongitude(),
				elevation);	
	}
	
	/**
	 * Gets the actual time over of this aircraft track point.
	 * 
	 * @return the actual time over this aircraft track point
	 */
	public ZonedDateTime getAto() {
		return this.time;
	}
	
	/**
	 * Gets the actual time over of this aircraft track point.
	 * 
	 * @return the actual time over this aircraft track point
	 * 
	 * @see TrackPoint#getTime()
	 */
	@Override
	public String getTime() {
		return this.time.toString();
	}
	
	/**
	 * Sets the actual time over this aircraft track point.
	 * 
	 * @param time the actual time over this aircraft track point
	 */
	public void setAto(ZonedDateTime time) {
		this.time = time;
	}
	
	/**
	 * Sets the actual time over this aircraft track point.
	 * 
	 * @param time the actual time over this aircraft track point
	 * 
	 * @see TrackPoint#setTime(String)
	 */
	@Override
	public void setTime(String time) {
		this.time = ZonedDateTime.parse(time);
	}
	
	/**
	 * Gets the maximum age to show this aircraft track point.
	 * 
	 * @return the maximum age to show this aircraft track point
	 */
	public Duration getMaxAge() {
		return this.maxAge;
	}
	
	/**
	 * Sets the maximum age to show this aircraft track point.
	 * 
	 * @param maxAge the maximum age to show this aircraft track point
	 */
	public void setMaxAge(Duration maxAge) {
		this.maxAge = maxAge;
	}
	
	/**
	 * Determines whether or not this aircraft track point is old.
	 * 
	 * @return true if this aircraft track point is old, false otherwise
	 */
	public boolean isOld() {
		ZonedDateTime now = ZonedDateTime.now(ZoneId.of("UTC"));
		Duration age = Duration.between(this.time, now);
		return (0 < age.compareTo(this.maxAge));
	}
	
	/**
	 * Updates the opacity of this aircraft track point according to its age
	 * and maximum age.
	 */
	private void updateOpacity() {
		ZonedDateTime now = ZonedDateTime.now(ZoneId.of("UTC"));
		Duration age = Duration.between(this.time, now);
		if (1 == age.compareTo(this.maxAge)) {
			this.getAttributes().setOpacity(0d);
		} else {
			Duration remaining = this.maxAge.minus(age);
			double opacity = ((double) remaining.getSeconds() / (double) maxAge.getSeconds());
			if (0d > opacity) opacity = 0d;
			this.getAttributes().setOpacity(opacity);
		}
	}
	
	/**
	 * Renders this aircraft track point after updating its opacity.
	 * 
	 * @see BasicMarker#render(DrawContext, Vec4, double)
	 */
	@Override
	public void render(DrawContext dc, Vec4 point, double radius) {
		this.updateOpacity();
		super.render(dc, point, radius);
	}
	
	/**
	 * Renders this aircraft track point after updating its opacity.
	 * 
	 * @see BasicMarker#render(DrawContext, Vec4, double, boolean)
	 */
	@Override
	public void render(DrawContext dc, Vec4 point, double radius, boolean isRelative) {
		this.updateOpacity();
		super.render(dc, point, radius, isRelative);
	}
	
	/**
	 * Gets the string representation of this aircraft track point.
	 * 
	 * @return the string representation of this aircraft track point
	 */
	@Override
	public String toString() {
		return "(("
				+ this.getLatitude() + ", "
				+ this.getLongitude() + ", "
				+ this.getElevation() + "), ("
				+ this.getPitch() + ", "
				+ this.getBank() + ", "
				+ this.getYaw() + "), "
				+ this.getTime() + ")";
	}
	
}
