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

import com.binarydreamers.trees.Interval;

public class CostInterval extends TimeInterval {
	
	private String id = null;
	private int cost = 0;
	
	public CostInterval(String id) {
		this(
			id,
			ZonedDateTime.now(ZoneId.of("UTC")),
			ZonedDateTime.now(ZoneId.of("UTC")).plusYears(1000));
	}
	
	public CostInterval(String id, ZonedDateTime time) {
		super(time);
		this.id = id;
	}
	
	public CostInterval(String id, ZonedDateTime start, ZonedDateTime end) {
		super(start, end);
		this.id = id;
	}
	
	public CostInterval(String id, ZonedDateTime start, ZonedDateTime end, int cost) {
		this(id, start, end);
		this.cost = cost;
	}
	
	public CostInterval(String id, TimeInterval timeInterval) {
		this(id, timeInterval.getLower(), timeInterval.getUpper());
	}
	
	public CostInterval(String id, TimeInterval timeInterval, int cost) {
		this(id, timeInterval);
		this.cost = cost;
	}
	
	public String getId() {
		return this.id;
	}
	
	public int getCost() {
		return this.cost;
	}

	public void setCost(int cost) {
		this.cost = cost;
	}

	public int getWeightedCost() {
		// TODO: costs could be weighted or a weight factor applied explicitly
		return this.cost;
	}
	
	public static final Comparator<Interval<ChronoZonedDateTime<?>>> comparator = new Comparator<Interval<ChronoZonedDateTime<?>>>() {
		@Override
		public int compare(Interval<ChronoZonedDateTime<?>> o1, Interval<ChronoZonedDateTime<?>> o2) {
			int value = TimeInterval.comparator.compare(o1, o2);
			
			if (0 == value) {
				if ((o1 instanceof CostInterval) && (o2 instanceof CostInterval)) {
					CostInterval c1 = (CostInterval) o1;
					CostInterval c2 = (CostInterval) o2;
					value = Integer.compare(c1.cost, c2.cost);
					
					if (0 == value) {
						value = c1.id.compareTo(c2.id);
					}
				}
			}
			
			return value;
		}
	};
	
}
