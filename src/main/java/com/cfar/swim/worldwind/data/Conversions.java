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
