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
package com.cfar.swim.worldwind.planning;

import java.time.ZonedDateTime;
import java.time.chrono.ChronoZonedDateTime;
import java.util.Comparator;
import java.util.Objects;

import com.binarydreamers.trees.Interval;
import com.cfar.swim.worldwind.util.Enableable;
import com.cfar.swim.worldwind.util.Identifiable;

/**
 * Realizes a cost interval based on TimeInterval and a cost value.
 * 
 * @see TimeInterval
 * 
 * @author Stephan Heinemann
 *
 */
public class CostInterval extends TimeInterval implements Identifiable, Enableable {
	
	/** the unique identifier of this cost interval */
	protected final String id;
	
	/** the cost of this cost interval */
	protected double cost = 0d;
	
	/** the enabled status of this cost interval */
	private boolean isEnabled = true;
	
	/**
	 * Constructs a cost (instant) interval based on the current UTC time for
	 * start and end times with a specified identifier and a cost of zero.
	 * 
	 * @param id the identifier of this cost interval
	 */
	public CostInterval(String id) {
		super();
		this.id = id;
	}
	
	/**
	 * Constructs a cost (instant) interval based on a specified time for
	 * start and end times with a specified identifier and a cost of zero.
	 * 
	 * @param id the identifier of this cost interval
	 * @param time the start and end time of this cost interval
	 */
	public CostInterval(String id, ZonedDateTime time) {
		super(time);
		this.id = id;
	}
	
	/**
	 * Constructs a cost interval from specified start and end times with a
	 * specified identifier and a cost of zero.
	 * 
	 * @param id the identifier of this cost interval
	 * @param start the start time of this cost interval
	 * @param end the end time of this cost interval
	 */
	public CostInterval(String id, ZonedDateTime start, ZonedDateTime end) {
		super(start, end);
		this.id = id;
	}
	
	/**
	 * Constructs a cost interval from specified start and end times with a
	 * specified identifier and cost.
	 * 
	 * @param id the identifier of this cost interval
	 * @param start the start time of this cost interval
	 * @param end the end time of this cost interval
	 * @param cost the cost of this cost interval
	 */
	public CostInterval(String id, ZonedDateTime start, ZonedDateTime end, double cost) {
		this(id, start, end);
		this.cost = cost;
	}
	
	/**
	 * Constructs a cost interval from a specified time interval with a
	 * specified  identifier and a cost of zero.
	 * 
	 * @param id the identifier of this cost interval
	 * @param timeInterval the time interval of this cost interval
	 */
	public CostInterval(String id, TimeInterval timeInterval) {
		this(id, timeInterval.getLower(), timeInterval.getUpper());
	}
	
	/**
	 * Constructs a cost interval from a specified time interval with a
	 * specified identifier and cost.
	 * 
	 * @param id the identifier of this cost interval
	 * @param timeInterval the time interval of this cost interval
	 * @param cost the cost of this cost interval
	 */
	public CostInterval(String id, TimeInterval timeInterval, double cost) {
		this(id, timeInterval);
		this.cost = cost;
	}
	
	/**
	 * Gets the identifier of this cost interval.
	 * 
	 * @return the identifier of this cost interval
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return this.id;
	}
	
	/**
	 * Gets the cost of this cost interval. A disabled cost interval
	 * always returns zero cost.
	 * 
	 * @return the cost of this cost interval
	 */
	public double getCost() {
		double cost = 0d;
		
		if (this.isEnabled) {
			cost = this.cost;
		}
		
		return cost;
	}
	
	/**
	 * Sets the cost of this cost interval.
	 * 
	 * @param cost the cost of this cost interval
	 */
	public void setCost(double cost) {
		this.cost = cost;
	}
	
	/**
	 * Enables this cost interval.
	 * 
	 * @see Enableable#enable()
	 */
	@Override
	public void enable() {
		this.isEnabled = true;
	}
	
	/**
	 * Disables this cost interval.
	 * 
	 * @see Enableable#disable()
	 */
	@Override
	public void disable() {
		this.isEnabled = false;
	}
	
	/**
	 * Determines whether or not this cost interval is enabled.
	 * 
	 * @return true if this cost interval is enabled, false otherwise
	 */
	@Override
	public boolean isEnabled() {
		return this.isEnabled;
	}
	
	/**
	 * Realizes a cost interval comparator.
	 */
	public static final Comparator<Interval<ChronoZonedDateTime<?>>> comparator = new Comparator<Interval<ChronoZonedDateTime<?>>>() {
		
		/**
		 * Compares two cost intervals using the start and end times as primary
		 * and secondary keys, respectively. The costs and identifiers serve as
		 * ternary and quaternary keys, respectively.
		 * 
		 * @see java.util.Comparator#compare(Object, Object)
		 */
		@Override
		public int compare(Interval<ChronoZonedDateTime<?>> o1, Interval<ChronoZonedDateTime<?>> o2) {
			int value = TimeInterval.comparator.compare(o1, o2);
			
			if (0 == value) {
				if ((o1 instanceof CostInterval) && (o2 instanceof CostInterval)) {
					CostInterval c1 = (CostInterval) o1;
					CostInterval c2 = (CostInterval) o2;
					value = Double.compare(c1.cost, c2.cost);
					
					if (0 == value) {
						value = c1.id.compareTo(c2.id);
					}
				}
			}
			
			return value;
		}
	};
	
	/**
	 * Determines whether or not this cost interval equals another cost
	 * interval based on their identifiers, costs as well as start and end
	 * times.
	 * 
	 * @param o the other cost interval
	 * 
	 * @return true if this cost interval equals the other cost interval based
	 *         on their identifiers, costs as well as start and end times,
	 *         false otherwise
	 * 
	 * @see TimeInterval#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = super.equals(o);
		
		if (equals) {
			CostInterval ci = (CostInterval) o;
			equals = this.id.equals(ci.id)
					&& (this.cost == ci.cost);
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this cost interval based on its identifier, cost
	 * as well as start and end times.
	 * 
	 * @return the hash code of this cost interval based on its identifier,
	 *         cost as well as start and end times
	 * 
	 * @see TimeInterval#hashCode()
	 */
	@Override
	public int hashCode() {
		return Objects.hash(
				super.hashCode(),
				this.id,
				this.cost);
	}
	
}
