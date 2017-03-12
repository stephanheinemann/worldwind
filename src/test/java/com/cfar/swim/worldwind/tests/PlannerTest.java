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
package com.cfar.swim.worldwind.tests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import java.time.ZonedDateTime;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import org.junit.Test;

import com.cfar.swim.worldwind.ai.astar.ForwardAStarPlanner;
import com.cfar.swim.worldwind.aircraft.CombatIdentification;
import com.cfar.swim.worldwind.aircraft.Iris;
import com.cfar.swim.worldwind.geom.Cube;
import com.cfar.swim.worldwind.geom.Neighborhood;
import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.planning.PlanningGrid;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.google.common.collect.Iterables;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Earth;
import gov.nasa.worldwind.render.Path;

public class PlannerTest {

	@Test
	@SuppressWarnings("unchecked")
	public void forwardAStarTest() {
		Vec4[] axes = new Vec4[] {Vec4.UNIT_X, Vec4.UNIT_Y, Vec4.UNIT_Z, Vec4.UNIT_W};
        // the reference cube has to be offset from the origin for the position computation to work
		Cube cube = new Cube(new Vec4(1000, 1000, 1000), axes, 1);
        PlanningGrid planningGrid = new PlanningGrid(cube, 2, 2, 2);
        planningGrid.setNeighborhood(Neighborhood.VERTEX_6);
        planningGrid.setGlobe(new Earth());
        
        Position origin = planningGrid.getCornerPositions()[0];
        Position destination = planningGrid.getCornerPositions()[6];
        ZonedDateTime etd = ZonedDateTime.now();
        Iris iris = new Iris(origin, 5000, CombatIdentification.FRIEND);
        
        ForwardAStarPlanner planner = new ForwardAStarPlanner(iris, planningGrid);
        Path path = planner.plan(origin, destination, etd);
        assertNotNull(path);
		assertEquals(7, Iterables.size(path.getPositions()));
		
		planningGrid.setNeighborhood(Neighborhood.VERTEX_26);
		path = planner.plan(origin, destination, etd);
        assertNotNull(path);
		assertEquals(3, Iterables.size(path.getPositions()));
		
		planningGrid.setNeighborhood(Neighborhood.VERTEX_6);
		List<Position> waypoints = Arrays.asList(planningGrid.getCornerPositions());
		path = planner.plan(origin, destination, waypoints, etd);
		assertEquals(8, waypoints.size());
		assertEquals(19 , Iterables.size(path.getPositions()));
		
		destination = planningGrid.getCornerPositions()[7];
		path = planner.plan(origin, destination, waypoints, etd);
		assertEquals(8, waypoints.size());
		assertEquals(17 , Iterables.size(path.getPositions()));
		
		PlanningGrid child = planningGrid.getChild(0, 0, 1);
		ZonedDateTime start = ZonedDateTime.now().minusYears(1);
		ZonedDateTime end = ZonedDateTime.now().plusYears(1);
		child.addCostInterval(new CostInterval("ci1", start, end, 25d));
		destination = planningGrid.getCornerPositions()[6];
		path = planner.plan(origin, destination, etd);
		assertNotNull(path);
		assertEquals(7, Iterables.size(path.getPositions()));
		
		Iterator<Position> positions = (Iterator<Position>) path.getPositions().iterator();
		while (positions.hasNext()) {
			assertEquals(false, child.isCorner(positions.next()));
		}
		
		child = planningGrid.getChild(0, 1, 0);
		child.addCostInterval(new CostInterval("ci2", start, end, 50d));
		child = planningGrid.getChild(1, 0, 0);
		child.addCostInterval(new CostInterval("ci3", start, end, 50d));
		path = planner.plan(origin, destination, etd);
		assertNotNull(path);
		assertEquals(7, Iterables.size(path.getPositions()));
		
		positions = (Iterator<Position>) path.getPositions().iterator();
		child = planningGrid.getChild(0, 0, 1);
		boolean isLowCost = false;
		while (positions.hasNext()) {
			if (child.isCorner(positions.next())) {
				isLowCost = true;
			}
		}
		assertEquals(true, isLowCost);
		
		planner.setRiskPolicy(RiskPolicy.AVOIDANCE);
		path = planner.plan(origin, destination, etd);
		assertEquals(0, Iterables.size(path.getPositions()));
		
		planner.setRiskPolicy(RiskPolicy.IGNORANCE);
		origin = planningGrid.getChild(0, 0, 0).getCenterPosition();
        destination = planningGrid.getChild(1, 1, 1).getCenterPosition();
        path = planner.plan(origin, destination, etd);
        assertEquals(3, Iterables.size(path.getPositions()));
	}

}
