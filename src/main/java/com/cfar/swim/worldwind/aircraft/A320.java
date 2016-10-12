package com.cfar.swim.worldwind.aircraft;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.symbology.SymbologyConstants;

public class A320 extends FixedWingCivilAircraft {

	// TODO: use environment AirDataIntervals (otherwise very simplified)
	public static final double MAX_ANGLE_OF_CLIMB_SPEED = 113d; // m/s = 220 KTAS
	public static final double MAX_RATE_OF_CLIMB_SPEED = 139d; // m/s = 270 KTAS
	public static final double CRUISE_CLIMB_SPEED = 257d; // m/s = 500 KTAS
	public static final double CRUISE_SPEED = 568d; // m/s = 520 KTAS
	public static final double CRUISE_DESCENT_SPEED = 257d; // m/s = 500 KTAS
	public static final double APPROACH_SPEED = 113d; // m/s = 220 KTAS
	public static final double MAX_GLIDE_SPEED = 98d; // m/s = 190 KTAS
	public static final double MAX_RATE_OF_DESCENT_SPEED = 281d; // m/s = 547 KTAS
	public static final double MAX_SPEED = 281d; // m/s = 547 KTAS
	
	public static final double MAX_RATE_OF_CLIMB = 12d; // m/s = 2400 ft/min
	public static final double CRUISE_RATE_OF_CLIMB = 8d; // m/s = 1500 ft/min
	public static final double CRUISE_RATE_OF_DESCENT = 5d; // m/s = 1000 ft/min
	public static final double APPROACH_RATE_OF_DESCENT = 9d; // m/s = 1800 ft/min
	public static final double MAX_RATE_OF_DESCENT = 18d; // m/s = 3500 ft/min
	public static final Angle MAX_ANGLE_OF_CLIMB = Angle.fromDegrees(15d);
	
	public A320(Position position, double radius, CombatIdentification cid) {
		super(position, radius, cid);
		this.capabilities = new Capabilities();
		this.capabilities.setMaximumAngleOfClimbSpeed(A320.MAX_ANGLE_OF_CLIMB_SPEED);
		this.capabilities.setMaximumRateOfClimb(A320.MAX_RATE_OF_CLIMB);
		this.capabilities.setCruiseClimbSpeed(A320.CRUISE_CLIMB_SPEED);
		this.capabilities.setCruiseSpeed(A320.CRUISE_SPEED);
		this.capabilities.setCruiseDescentSpeed(A320.CRUISE_DESCENT_SPEED);
		this.capabilities.setApproachSpeed(A320.APPROACH_SPEED);
		this.capabilities.setMaximumGlideSpeed(A320.MAX_GLIDE_SPEED);
		this.capabilities.setMaximumRateOfDescentSpeed(A320.MAX_RATE_OF_DESCENT_SPEED);
		this.capabilities.setMaximumSpeed(A320.MAX_SPEED);
		this.capabilities.setMaximumRateOfClimb(A320.MAX_RATE_OF_CLIMB);
		this.capabilities.setCruiseRateOfClimb(A320.CRUISE_RATE_OF_CLIMB);
		this.capabilities.setCruiseRateOfDescent(A320.CRUISE_RATE_OF_DESCENT);
		this.capabilities.setApproachRateOfDescent(A320.APPROACH_RATE_OF_DESCENT);
		this.capabilities.setMaximumRateOfDescent(A320.MAX_RATE_OF_DESCENT);
		this.capabilities.setMaximumAngleOfClimb(A320.MAX_ANGLE_OF_CLIMB);
		
		// TODO: use actual live data for symbol annotations...
		this.getDepiction().setModifier(SymbologyConstants.SPEED, A320.CRUISE_SPEED);
	}

}
