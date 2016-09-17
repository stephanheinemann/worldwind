package com.cfar.swim.worldwind.ai;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.Environment;

public abstract class AbstractPlanner implements Planner {

	private Aircraft aircraft = null;
	private Environment environment = null;
	
	public AbstractPlanner(Aircraft aircraft, Environment environment) {
		this.aircraft = aircraft;
		this.environment = environment;
	}
	
	@Override
	public Aircraft getAircraft() {
		return this.aircraft;
	}
	
	@Override
	public Environment getEnvironment() {
		return this.environment;
	}

}
