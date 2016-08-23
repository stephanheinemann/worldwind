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
package com.cfar.swim.worldwind.data;

public class Conversions {
	
	public static final String UOM_DEGREE = "deg";
	public static final String UOM_FLIGHT_LEVEL = "fl";
	public static final String UOM_KNOT = "[kn_i]";
	public static final String UOM_METER = "m";
	public static final String UOM_METER_PER_SECOND = "m/s";
	public static final String UOM_NAUTICAL_MILE = "[nmi_i]";
	public static final String UOM_RADIAN = "rad";
	
	// TODO: gov.nasa.worldwind.util.WWMath
	public static final double METER_PER_FOOT = 0.3048;
	public static final double METER_PER_NAUTICAL_MILE = 1852;
	public static final double METER_PER_SECOND_PER_KNOT = 0.514444;
	
	public static final double PRESSURE_STD = 1013.25; // hPa
	public static final double PRESSURE_GRADIENT = 27; // ft/hPa
	
	public static final double DEGREE_PER_RADIAN = 57.2958;
	
	public static double flightLevelsToMeters(double flightLevels) {
		// TODO: assumes standard pressure at all locations
		return (flightLevels * 100.0f) * Conversions.METER_PER_FOOT;
	}
	
	public static double knotsToMetersPerSecond(double knots) {
		return knots * Conversions.METER_PER_SECOND_PER_KNOT;
	}
	
	public static double nauticalMilesToMeters(double nauticalMiles) {
		return nauticalMiles * Conversions.METER_PER_NAUTICAL_MILE;
	}
	
	public static double radiansToDegrees(double radians) {
		return radians * Conversions.DEGREE_PER_RADIAN;
	}
	
	public static double toDegrees(double angle, String uom) {
		if (!uom.equalsIgnoreCase(Conversions.UOM_DEGREE)) {
			if (uom.equalsIgnoreCase(Conversions.UOM_RADIAN)) {
				angle = radiansToDegrees(angle);
			}
			// TODO: implement other conversions
		}
		return angle;
	}
	
	public static double toMeters(double distance, String uom) {
		if (!uom.equalsIgnoreCase(Conversions.UOM_METER)) {
			if (uom.equalsIgnoreCase(Conversions.UOM_FLIGHT_LEVEL)) {
				distance = Conversions.flightLevelsToMeters(distance);
			} else if (uom.equalsIgnoreCase(Conversions.UOM_NAUTICAL_MILE)) {
				distance = Conversions.nauticalMilesToMeters(distance);
			}
			// TODO: implement other conversions
		}
		return distance;
	}
	
	public static double toMetersPerSecond(double velocity, String uom) {
		if (!uom.equalsIgnoreCase(Conversions.UOM_METER_PER_SECOND)) {
			if (uom.equalsIgnoreCase(Conversions.UOM_KNOT)) {
				velocity = Conversions.knotsToMetersPerSecond(velocity);
			}
			// TODO: implement other conversions
		}
		return velocity;
	}
}
