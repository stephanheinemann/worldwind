package com.cfar.swim.worldwind.tests;

import static org.junit.Assert.*;

import java.time.ZonedDateTime;

import org.junit.Test;

import com.cfar.swim.worldwind.ai.astar.ForwardAStarPlanner;
import com.cfar.swim.worldwind.aircraft.CombatIdentification;
import com.cfar.swim.worldwind.aircraft.Iris;
import com.cfar.swim.worldwind.geom.Cube;
import com.cfar.swim.worldwind.planning.PlanningGrid;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Earth;
import gov.nasa.worldwind.render.Path;

public class PlannerTest {

	@Test
	public void forwardAStarTest() {
		Vec4[] axes = new Vec4[] {Vec4.UNIT_X, Vec4.UNIT_Y, Vec4.UNIT_Z, Vec4.UNIT_W};
        // the reference cube has to be offset from the origin for the position computation to work
		Cube cube = new Cube(new Vec4(1000, 1000, 1000), axes, 1);
        PlanningGrid planningGrid = new PlanningGrid(cube, 2, 2, 2);
        planningGrid.setGlobe(new Earth());
        
        Position origin = planningGrid.getCornerPositions()[0];
        Position destination = planningGrid.getCornerPositions()[6];
        ZonedDateTime etd = ZonedDateTime.now();
        Iris iris = new Iris(origin, 5000, CombatIdentification.FRIEND);
        
        ForwardAStarPlanner planner = new ForwardAStarPlanner(iris, planningGrid);
        Path path = planner.plan(origin, destination, etd);
		assertNotNull(path);
		assertEquals(6, planner.getPlan().size());
	}

}
