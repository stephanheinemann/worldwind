package com.cfar.swim.worldwind.aircraft;

import gov.nasa.worldwind.geom.Position;

public class Iris extends Quadcopter {

	// TODO: test/find proper capabilities
	public static final int CRUISE_SPEED = 5;
	public static final int MAX_SPEED = 7;
	public static final int MAX_CLIMB_RATE = 200;
	public static final int MAX_DESCENT_RATE = 200;
	
	public Iris(Position position, double radius, CombatIdentification cid) {
		super(position, radius, cid);
		this.capabilities = new Capabilities();
		this.capabilities.setCruiseSpeed(Iris.CRUISE_SPEED);
		this.capabilities.setMaximumClimbRate(Iris.MAX_CLIMB_RATE);
		this.capabilities.setMaximumDescentRate(Iris.MAX_DESCENT_RATE);
		this.capabilities.setMaximumSpeed(Iris.MAX_SPEED);
		// TODO: ...
	}

}
