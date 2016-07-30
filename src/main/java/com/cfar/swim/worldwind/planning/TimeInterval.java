package com.cfar.swim.worldwind.planning;

import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.time.chrono.ChronoZonedDateTime;
import java.util.Comparator;

import com.binarydreamers.trees.Interval;

public class TimeInterval implements Interval<ChronoZonedDateTime<?>> {

	private ZonedDateTime start = ZonedDateTime.now(ZoneId.of("UTC"));
	private ZonedDateTime end = ZonedDateTime.now(ZoneId.of("UTC"));
	
	public TimeInterval() {
	}
	
	public TimeInterval(ZonedDateTime time) {
		this.start = time;
		this.end = time;
	}

	public TimeInterval(ZonedDateTime start, ZonedDateTime end) {
		this.start = start;
		this.end = end;
	}
	
	@Override
	public ZonedDateTime getLower() {
		return this.start;
	}

	@Override
	public ZonedDateTime getUpper() {
		return this.end;
	}
	
	public static final Comparator<Interval<ChronoZonedDateTime<?>>> comparator = new Comparator<Interval<ChronoZonedDateTime<?>>>() {
		@Override
		public int compare(Interval<ChronoZonedDateTime<?>> o1, Interval<ChronoZonedDateTime<?>> o2) {
			int value = o1.getLower().compareTo(o2.getLower());
			if (0 == value) {
				value = o1.getUpper().compareTo(o2.getUpper());
			}
			
			return value;
		}
	};
	
}
