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

import java.time.ZonedDateTime;

import org.junit.Test;

import com.cfar.swim.worldwind.geom.Cube;
import com.cfar.swim.worldwind.geom.Neighborhood;
import com.cfar.swim.worldwind.geom.precision.PrecisionDouble;
import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.PlanningGrid;
import com.cfar.swim.worldwind.planning.RiskPolicy;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Earth;

public class PlanningGridTest {

	@Test
	public void testStructure() {
		Vec4[] axes = new Vec4[] {Vec4.UNIT_X, Vec4.UNIT_Y, Vec4.UNIT_Z, Vec4.UNIT_W};
        // the reference cube has to be offset from the origin for the position computation to work
		Cube cube = new Cube(new Vec4(1000, 1000, 1000), axes, 1);
        PlanningGrid planningGrid = new PlanningGrid(cube, 10, 5, 5);
        planningGrid.setNeighborhood(Neighborhood.VERTEX_6);
        planningGrid.setGlobe(new Earth());
        assertEquals(true, planningGrid.hasChildren());
        
        PlanningGrid child = planningGrid.getChild(0, 0, 0);
        assertEquals(8, child.getCornerPositions().length);
        
        Position position = child.getCornerPositions()[0];
        assertEquals(true, child.isCorner(position));
        assertEquals(true, planningGrid.isCorner(position));
        assertEquals(3, child.getNeighbors(position).size());
        assertEquals(3, planningGrid.getNeighbors(position).size());
        assertEquals(true, child.areNeighbors(child.getCornerPositions()[0], child.getCornerPositions()[1]));
        
        planningGrid.setNeighborhood(Neighborhood.VERTEX_26);
        assertEquals(7, child.getNeighbors(position).size());
        assertEquals(7, planningGrid.getNeighbors(position).size());
        
        planningGrid.setNeighborhood(Neighborhood.VERTEX_6);
        child = planningGrid.getChild(1, 1, 1);
        position = child.getCornerPositions()[0];
        assertEquals(true, child.isCorner(position));
        assertEquals(false, planningGrid.isCorner(position));
        assertEquals(3, child.getNeighbors(position).size());
        assertEquals(6, planningGrid.getNeighbors(position).size());
        
        planningGrid.setNeighborhood(Neighborhood.VERTEX_26);
        assertEquals(7, child.getNeighbors(position).size());
        assertEquals(26, planningGrid.getNeighbors(position).size());
        
        planningGrid.setNeighborhood(Neighborhood.VERTEX_6);
        position = child.getCenterPosition();
        assertEquals(false, child.isCorner(position));
        assertEquals(true, child.isCenter(position));
        assertEquals(0, child.getNeighbors(position).size());
        assertEquals(8, child.getAdjacentWaypoints(position).size());
        assertEquals(0, planningGrid.getNeighbors(position).size());
        assertEquals(8, planningGrid.getAdjacentWaypoints(position).size());
        assertEquals(false, child.isWaypoint(position));
        assertEquals(false, planningGrid.isWaypoint(position));
        assertEquals(true, child.areNeighbors(child.getCornerPositions()[0], child.getCornerPositions()[1]));
        assertEquals(true, planningGrid.areNeighbors(child.getCornerPositions()[0], child.getCornerPositions()[1]));   
	}
	
	@Test
	public void testCosts() {
		Vec4[] axes = new Vec4[] {Vec4.UNIT_X, Vec4.UNIT_Y, Vec4.UNIT_Z, Vec4.UNIT_W};
		// the reference cube has to be offset from the origin for the position computation to work
        Cube cube = new Cube(new Vec4(1000, 1000, 1000), axes, 1);
        PlanningGrid planningGrid = new PlanningGrid(cube, 10, 5, 5);
        planningGrid.setGlobe(new Earth());
        assertEquals(1d, planningGrid.getCost(ZonedDateTime.now()), PrecisionDouble.EPSILON);
        
        PlanningGrid child = planningGrid.getChild(0, 0, 0);
        assertEquals(1d, child.getCost(ZonedDateTime.now()), PrecisionDouble.EPSILON);
        assertEquals(1d / child.getNormalizer(), child.getStepCost(
        		child.getCornerPositions()[0],
        		child.getCornerPositions()[1],
        		ZonedDateTime.now().minusYears(10),
        		ZonedDateTime.now().plusYears(10),
        		CostPolicy.AVERAGE, RiskPolicy.AVOIDANCE), PrecisionDouble.EPSILON);
        
        child.addCostInterval(new CostInterval("obstacle1", ZonedDateTime.now(), ZonedDateTime.now(), 50d));
        assertEquals(Double.POSITIVE_INFINITY, child.getStepCost(
        		child.getCornerPositions()[0],
        		child.getCornerPositions()[1],
        		ZonedDateTime.now().minusYears(10),
        		ZonedDateTime.now().plusYears(10),
        		CostPolicy.AVERAGE, RiskPolicy.AVOIDANCE), PrecisionDouble.EPSILON);
        assertEquals(51d / child.getNormalizer(), child.getStepCost(
        		child.getCornerPositions()[0],
        		child.getCornerPositions()[1],
        		ZonedDateTime.now().minusYears(10),
        		ZonedDateTime.now().plusYears(10),
        		CostPolicy.AVERAGE, RiskPolicy.SAFETY), PrecisionDouble.EPSILON);
        
        child.addCostInterval(new CostInterval("obstacle1", ZonedDateTime.now(), ZonedDateTime.now(), 50d));
        child.addCostInterval(new CostInterval("obstacle2", ZonedDateTime.now(), ZonedDateTime.now(), 30d));
        assertEquals(Double.POSITIVE_INFINITY, child.getStepCost(
        		child.getCornerPositions()[0],
        		child.getCornerPositions()[1],
        		ZonedDateTime.now().minusYears(10),
        		ZonedDateTime.now().plusYears(10),
        		CostPolicy.AVERAGE, RiskPolicy.EFFECTIVENESS), PrecisionDouble.EPSILON);
        assertEquals(81d / child.getNormalizer(), child.getStepCost(
        		child.getCornerPositions()[0],
        		child.getCornerPositions()[1],
        		ZonedDateTime.now().minusYears(10),
        		ZonedDateTime.now().plusYears(10),
        		CostPolicy.AVERAGE, RiskPolicy.IGNORANCE), PrecisionDouble.EPSILON);
        assertEquals(1d / child.getNormalizer(), child.getNormalizedDistance(
        		child.getCornerPositions()[0],
        		child.getCornerPositions()[1]),
        		PrecisionDouble.EPSILON);
        
        child.addChildren(2);
        assertEquals(true, child.hasChildren());
        assertEquals(true, child.hasChild(0, 0, 0));
        
        PlanningGrid grandChild = child.getChild(0, 0, 0);
        assertEquals(0.5d / grandChild.getNormalizer(), grandChild.getNormalizedDistance(
        		grandChild.getCornerPositions()[0],
        		grandChild.getCornerPositions()[1]),
        		PrecisionDouble.EPSILON);
        
	}

}
