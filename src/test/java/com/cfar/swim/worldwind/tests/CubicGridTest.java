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
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import com.cfar.swim.worldwind.geom.Cube;
import com.cfar.swim.worldwind.geom.CubicGrid;
import com.cfar.swim.worldwind.geom.precision.PrecisionDouble;

import gov.nasa.worldwind.geom.Intersection;
import gov.nasa.worldwind.geom.Line;
import gov.nasa.worldwind.geom.Vec4;

public class CubicGridTest {
	
	@Test
	public void testStructure() {
		Vec4[] axes = new Vec4[] {Vec4.UNIT_X, Vec4.UNIT_Y, Vec4.UNIT_Z, Vec4.UNIT_W};
        Cube cube = new Cube(Vec4.ZERO, axes, 1);
        CubicGrid cubicGrid = new CubicGrid(cube, 10, 5, 5);
        assertEquals(true, cubicGrid.hasChildren());
        assertEquals(true, cubicGrid.hasChild(0, 0, 0));
        assertEquals(false, cubicGrid.hasParent());
        assertEquals(250 , cubicGrid.getChildren().size());
        
        CubicGrid child = cubicGrid.getChild(0, 0, 0);
        assertEquals(false, child.hasChildren());
        assertEquals(true, child.hasParent());
        
        child.addChildren(0.5);
        child.getChild(1, 1, 1).addChildren(2);
        assertEquals(true, child.hasChildren());
        assertEquals(8, child.getChildren().size());
        assertEquals(true, child.hasChild(1, 1, 1));
        assertEquals(8, child.getChild(1, 1, 1).getChildren().size());
        assertEquals(17, child.getAll().size());
        assertEquals(267, cubicGrid.getAll().size());
        
        child.removeChildren();
        assertEquals(false, child.hasChildren());
        assertEquals(251, cubicGrid.getAll().size());
        
        cubicGrid.removeChildren();
        assertEquals(false, cubicGrid.hasChildren());
        assertEquals(1, cubicGrid.getAll().size());
	}
	
	@Test
	public void testNormalizer() {
		Vec4[] axes = new Vec4[] {Vec4.UNIT_X, Vec4.UNIT_Y, Vec4.UNIT_Z, Vec4.UNIT_W};
        Cube cube = new Cube(Vec4.ZERO, axes, 1);
        CubicGrid cubicGrid = new CubicGrid(cube, 10, 5, 5);
        
        for (CubicGrid grid : cubicGrid.getAll()) {
        	assertEquals(cube.getDiameter(), grid.getNormalizer(), PrecisionDouble.EPSILON);
        }
        
        cubicGrid.getChild(0, 0, 0).addChildren(0.5d);
        
        for (CubicGrid grid : cubicGrid.getAll()) {
        	assertEquals(cube.getDiameter(), grid.getNormalizer(), PrecisionDouble.EPSILON);
        }
        
        cubicGrid.getChild(0, 0, 1).addChildren(2);
        
        for (CubicGrid grid : cubicGrid.getAll()) {
        	assertEquals(cube.getDiameter(), grid.getNormalizer(), PrecisionDouble.EPSILON);
        }
		
        cubicGrid.removeChildren();
        
        for (CubicGrid grid : cubicGrid.getAll()) {
        	assertEquals(cube.getDiameter(), grid.getNormalizer(), PrecisionDouble.EPSILON);
        }
	}
	
	@Test
	public void testIntersections() {
		Vec4[] axes = new Vec4[] {Vec4.UNIT_X, Vec4.UNIT_Y, Vec4.UNIT_Z, Vec4.UNIT_W};
        Cube cube = new Cube(Vec4.ZERO, axes, 1);
        CubicGrid cubicGrid = new CubicGrid(cube, 10, 5, 5);
        cubicGrid.getChild(0, 0, 0).addChildren(2);
        Line line = Line.fromSegment(cubicGrid.getCorners()[0], cubicGrid.getCorners()[1]);
        
        assertTrue(null != cubicGrid.intersect(line));
        assertTrue(null != cubicGrid.getChild(0, 0, 0).intersect(line));
        
        CubicGrid grandChild = cubicGrid.getChild(0, 0, 0).getChild(0, 0, 0);
        Intersection[] intersections = grandChild.intersect(line);
        assertTrue(null != intersections);
        assertEquals(2, intersections.length);
        assertTrue(intersections[0].isTangent());
        assertTrue(intersections[1].isTangent());
        assertEquals(grandChild.getCorners()[0], intersections[0].getIntersectionPoint());
        assertEquals(grandChild.getCorners()[1], intersections[1].getIntersectionPoint());
	}

}
