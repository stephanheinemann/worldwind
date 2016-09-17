package com.cfar.swim.worldwind.ai.thetastar;

import java.util.List;

import com.cfar.swim.worldwind.ai.AbstractPlanner;
import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.planning.CubicPlanningGrid;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.render.Path;

public class ThetaStarPlanner extends AbstractPlanner {

	
	public ThetaStarPlanner(Aircraft aircraft, CubicPlanningGrid environment) {
		super(aircraft, environment);
	}

	@Override
	public CubicPlanningGrid getEnvironment() {
		return (CubicPlanningGrid) super.getEnvironment();
	}
	
	@Override
	public Path plan(Position start, Position goal) {
		CubicPlanningGrid grid = this.getEnvironment();
		
		Vec4 startPoint = grid.getGlobe().computePointFromPosition(start);
		Vec4 goalPoint = grid.getGlobe().computePointFromPosition(goal);
		
		grid.lookupCells(startPoint);
		grid.lookupCells(goalPoint);
		
		// TODO Auto-generated method stub		
		return new Path(start, goal);
	}

	@Override
	public Path plan(Position start, Position goal, List<Position> pois) {
		// TODO Auto-generated method stub
		return null;
	}

}
