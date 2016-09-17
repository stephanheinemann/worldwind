package com.cfar.swim.worldwind.aircraft;

public class Capabilities {

	private int cruiseSpeed = 0; // knots
	private int maximumSpeed = 0; // knots
	private int maximumClimbRate = 0; // ft/min
	private int maximumDescentRate = 0; // ft/min
	
	// TODO: average fuel consumption, endurance, range, radius of action...
	// TODO: equipment (de-icing, gear, floats...)
	
	public int getCruiseSpeed() {
		return this.cruiseSpeed;
	}
	
	public void setCruiseSpeed(int cruiseSpeed) {
		this.cruiseSpeed = cruiseSpeed;
	}

	public int getMaximumSpeed() {
		return maximumSpeed;
	}

	public void setMaximumSpeed(int maximumSpeed) {
		this.maximumSpeed = maximumSpeed;
	}

	public int getMaximumClimbRate() {
		return maximumClimbRate;
	}

	public void setMaximumClimbRate(int maximumClimbRate) {
		this.maximumClimbRate = maximumClimbRate;
	}

	public int getMaximumDescentRate() {
		return maximumDescentRate;
	}

	public void setMaximumDescentRate(int maximumDescentRate) {
		this.maximumDescentRate = maximumDescentRate;
	}
	
}
