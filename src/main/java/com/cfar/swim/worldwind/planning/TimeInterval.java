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

import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.time.chrono.ChronoZonedDateTime;
import java.util.Comparator;
import java.util.Objects;

import com.binarydreamers.trees.Interval;

/**
 * Realizes a time interval based on ZonedDateTime.
 * 
 * @see com.binarydreamers.trees.Interval
 * @see java.time.ZonedDateTime
 * 
 * @author Stephan Heinemann
 *
 */
public class TimeInterval implements Interval<ChronoZonedDateTime<?>> {

	/** the start time of this time interval */
	private ZonedDateTime start = ZonedDateTime.now(ZoneId.of("UTC"));
	
	/** the end time of this time interval */
	private ZonedDateTime end = ZonedDateTime.now(ZoneId.of("UTC"));
	
	/**
	 * Constructs a time (instant) interval based on the current UTC time for
	 * start and end times.
	 */
	public TimeInterval() {
	}
	
	/**
	 * Constructs a time (instant) interval based on a specified time for
	 * start and end times.
	 * 
	 * @param time the start and end time of this time interval
	 */
	public TimeInterval(ZonedDateTime time) {
		this.start = time;
		this.end = time;
	}

	/**
	 * Constructs a time interval from specified start and end times.
	 * 
	 * @param start the start time of this time interval
	 * @param end the end time of this time interval
	 */
	public TimeInterval(ZonedDateTime start, ZonedDateTime end) {
		this.start = start;
		this.end = end;
	}
	
	/**
	 * Gets the start time of this time interval.
	 */
	@Override
	public ZonedDateTime getLower() {
		return this.start;
	}

	/**
	 * Sets the start time of this time interval.
	 * 
	 * @param start the start time of this time interval
	 */
	public void setLower(ZonedDateTime start) {
		this.start = start;
	}
	
	/**
	 * Gets the end time of this time interval.
	 */
	@Override
	public ZonedDateTime getUpper() {
		return this.end;
	}
	
	/**
	 * Sets the end time of this time interval.
	 * 
	 * @param end the end time of this time interval
	 */
	public void setUpper(ZonedDateTime end) {
		this.end = end;
	}
	
	/**
	 * Determines whether or not a time instant is contained in this time
	 * interval.
	 * 
	 * @param time the time instant
	 * @return true if the time instant is contained in this time interval,
	 *         false otherwise
	 */
	public boolean contains(ZonedDateTime time) {
		// TODO: possibly exclude end time to be consistent with Duration implementation
		return
			(this.start.isEqual(time) || this.start.isBefore(time)) &&
			(this.end.isAfter(time) || this.end.isEqual(time));
	}
	
	/**
	 * Determines whether or not another time interval intersects this time
	 * interval.
	 * 
	 * @param timeInterval the other time interval
	 * @return true if the other time interval intersects this time interval,
	 *         false, otherwise
	 */
	public boolean intersects(TimeInterval timeInterval) {
		return (this.contains(timeInterval.start) ||
				this.contains(timeInterval.end) ||
				timeInterval.contains(this.start) ||
				timeInterval.contains(this.end));
	}
	
	/**
	 * Computes the intersection between this and another time interval.
	 * 
	 * @param timeInterval the other time interval
	 * @return the intersection between this and another time interval
	 */
	public TimeInterval intersect(TimeInterval timeInterval) {
		TimeInterval intersection = null;
		
		if (this.intersects(timeInterval)) {
			intersection = new TimeInterval();
			if (this.contains(timeInterval.start)) {
				intersection.setLower(timeInterval.start);
			}
			if (this.contains(timeInterval.end)) {
				intersection.setUpper(timeInterval.end);
			}
			if (timeInterval.contains(this.start)) {
				intersection.setLower(this.start);
			}
			if (timeInterval.contains(this.end)) {
				intersection.setUpper(this.end);
			}
		}
		
		return intersection;
	}
	
	/**
	 * Realizes a time interval comparator.
	 */
	public static final Comparator<Interval<ChronoZonedDateTime<?>>> comparator = new Comparator<Interval<ChronoZonedDateTime<?>>>() {
		
		/**
		 * Compares two time intervals using the start and end times as primary
		 * and secondary keys, respectively.
		 * 
		 * @see java.util.Comparator#compare(Object, Object)
		 */
		@Override
		public int compare(Interval<ChronoZonedDateTime<?>> o1, Interval<ChronoZonedDateTime<?>> o2) {
			int value = o1.getLower().compareTo(o2.getLower());
			if (0 == value) {
				value = o1.getUpper().compareTo(o2.getUpper());
			}
			
			return value;
		}
	};
	
	/**
	 * Determines whether or not this time interval equals another time
	 * interval based on their start and end times.
	 * 
	 * @param o the other time interval
	 * 
	 * @return true if this time interval equals the other time interval based
	 *         on their start and end times, false otherwise
	 * 
	 * @see Object#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = false;
		
		if (this == o) {
			equals = true;
		} else if ((null != o) && (this.getClass() == o.getClass())) {
			TimeInterval ti = (TimeInterval) o;
			equals = ((this.start.equals(ti.start))
					&& (this.end.equals(ti.end)));
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this time interval based on its start and end
	 * times.
	 * 
	 * @return the hash code of this time interval based on its start and end
	 *         times
	 * 
	 * @see Object#hashCode()
	 */
	@Override
	public int hashCode() {
		return Objects.hash(this.start, this.end);
	}
	
}
