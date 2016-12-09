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
package com.cfar.swim.worldwind.planning;

import java.time.Duration;
import java.time.ZoneId;
import java.time.ZonedDateTime;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.markers.BasicMarker;
import gov.nasa.worldwind.render.markers.MarkerAttributes;


/**
 * Realizes a track point on an actually flown track.
 * 
 * @author Stephan Heinemann
 *
 */
public class TrackPoint extends BasicMarker {

	/** the time at this track point */
	private ZonedDateTime time = ZonedDateTime.now(ZoneId.of("UTC"));
	
	/** the maximum age to show this track point */
	private Duration maxAge = Duration.ofSeconds(10);
	
	/**
	 * Constructs a new track point with specified position and attributes.
	 * 
	 * @param position the position of this track point
	 * @param attrs the attributes of this track point
	 * 
	 * @see BasicMarker#BasicMarker(Position, MarkerAttributes)
	 */
	public TrackPoint(Position position, MarkerAttributes attrs) {
		super(position, attrs);
	}
	
	/**
	 * Constructs a new track point with specified position, attributes and
	 * heading.
	 * 
	 * @param position the position of this track point
	 * @param attrs the attributes of this track point
	 * @param heading the heading of this trackpoing
	 * 
	 * @see BasicMarker#BasicMarker(Position, MarkerAttributes, Angle)
	 */
	public TrackPoint(Position position, MarkerAttributes attrs, Angle heading) {
		super(position, attrs, heading);
	}
	
	/**
	 * Gets the time of this track point.
	 * 
	 * @return the time of this track point
	 */
	public ZonedDateTime getTime() {
		return this.time;
	}
	
	/**
	 * Sets the time of this track point.
	 * 
	 * @param time the time to be set
	 */
	public void setTime(ZonedDateTime time) {
		this.time = time;
	}
	
	/**
	 * Gets the maximum age to show this track point.
	 * 
	 * @return the maximum age to show this track point
	 */
	public Duration getMaxAge() {
		return this.maxAge;
	}
	
	/**
	 * Sets the maximum age to show this track point.
	 * 
	 * @param maxAge the maximum age to be set
	 */
	public void setMaxAge(Duration maxAge) {
		this.maxAge = maxAge;
	}
	
	/**
	 * Updates the opacity of this track point according to its age and
	 * maximum age.
	 */
	private void updateOpacity() {
		ZonedDateTime now = ZonedDateTime.now(ZoneId.of("UTC"));
		Duration age = Duration.between(this.time, now);
		if (1 == age.compareTo(maxAge)) {
			this.getAttributes().setOpacity(0d);
		} else {
			Duration remaining = maxAge.minus(age);
			double opacity = ((double) remaining.getSeconds() / (double) maxAge.getSeconds());
			if (0d > opacity) opacity = 0d;
			this.getAttributes().setOpacity(opacity);
		}
	}
	
	/**
	 * Renders this track point after updating its opacity.
	 * 
	 * @see BasicMarker#render(DrawContext, Vec4, double)
	 */
	@Override
	public void render(DrawContext dc, Vec4 point, double radius) {
		this.updateOpacity();
		super.render(dc, point, radius);
	}
	
	/**
	 * Renders this track point after updating its opacity.
	 * 
	 * @see BasicMarker#render(DrawContext, Vec4, double, boolean)
	 */
	@Override
	public void render(DrawContext dc, Vec4 point, double radius, boolean isRelative) {
		this.updateOpacity();
		super.render(dc, point, radius, isRelative);
	}
	
}
