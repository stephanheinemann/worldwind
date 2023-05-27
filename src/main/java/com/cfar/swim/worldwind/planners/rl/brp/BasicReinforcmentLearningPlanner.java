package com.cfar.swim.worldwind.planners.rl.brp;

import java.time.ZonedDateTime;
import java.util.List;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.planners.AbstractPlanner;
import com.cfar.swim.worldwind.planners.cgs.astar.ForwardAStarPlanner;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.registries.Specification;

import gov.nasa.worldwind.geom.Position;

public class BasicReinforcmentLearningPlanner extends AbstractPlanner {

	public BasicReinforcmentLearningPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		// TODO Auto-generated constructor stub
	}

	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		AbstractPlanner planner = new ForwardAStarPlanner(getAircraft(), getEnvironment());
		return planner.plan(origin, destination, etd);
	}

	@Override
	public Trajectory plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
		AbstractPlanner planner = new ForwardAStarPlanner(getAircraft(), getEnvironment());
		return planner.plan(origin, destination, waypoints, etd);
	}

	@Override
	public String getId() {
		// TODO Auto-generated method stub
		return Specification.PLANNER_BRP_ID;
	}

}
