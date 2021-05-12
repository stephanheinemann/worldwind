/**
 * Copyright (c) 2021, Stephan Heinemann (UVic Center for Aerospace Research)
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
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.geom.precision.PrecisionDouble;
import com.cfar.swim.worldwind.geom.precision.PrecisionVec4;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Intersection;
import gov.nasa.worldwind.geom.Line;
import gov.nasa.worldwind.geom.Plane;
import gov.nasa.worldwind.geom.Sector;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Earth;

/**
 * Performs box tests.
 * 
 * @author Stephan Heinemann
 *
 */
public class BoxTest {

	/**
	 * Tests box structures.
	 */
	@Test
	public void testStructure() {
		//Vec4[] axes = { Vec4.UNIT_X, Vec4.UNIT_Y, Vec4.UNIT_Z };
		Vec4[] axes = { new Vec4(1, 0, 0), new Vec4(0, 1, 0), new Vec4(0, 0, 1) };
		
		Box box = new Box(Vec4.ZERO);
		Vec4 half = new Vec4(0.5d, 0.5d, 0.5d);
		Vec4 nHalf = half.getNegative3();
		assertEquals(Vec4.ZERO, box.getCenter());
		assertEquals(nHalf, box.getOrigin());
		assertEquals(half, box.getOrigin().add3(axes[0]).add3(axes[1]).add3(axes[2]));
		assertEquals(half, box.getOppositeCorners(nHalf)[1]);
		
		box = new Box(half);
		assertEquals(half, box.getCenter());
		assertEquals(Vec4.ZERO, box.getOrigin());
		assertEquals(Vec4.ONE, box.getOrigin().add3(axes[0]).add3(axes[1]).add3(axes[2]));
		assertEquals(Vec4.ONE, box.getOppositeCorners(box.getOrigin())[1]);
		
		Vec4 two = new Vec4(2d, 2d, 2d);
		box = new Box(Vec4.ZERO, axes, 2, 2, 2);
		Vec4 origin = box.getOrigin();
		Vec4 center = box.getCenter();
		Vec4[] corners = box.getCorners();
		assertEquals(Vec4.ONE, center);
		assertTrue(box.isCenter(center));
		assertTrue(box.isOrigin(origin));
		assertTrue(box.isCorner(origin));
		assertEquals(two, box.getOrigin().add3(two));
		assertEquals(two, box.getOppositeCorners(box.getOrigin())[1]);
		
		Plane[] planes = box.getPlanes();
		assertEquals(Plane.intersect(planes[0], planes[2], planes[4]), origin);
		assertEquals(8, corners.length);
		
		for (int index = 0; index < corners.length; index++) {
			assertEquals(index, box.getCornerIndex(corners[index]));
		}
		
		Vec4[] neighborCorners = box.getNeighborCorners(corners[Box.CORNER_INDEX_BOTTOM_LOWER_LEFT]);
		Vec4[] oppositeCorners = box.getOppositeCorners(corners[Box.CORNER_INDEX_BOTTOM_LOWER_LEFT]);
		Vec4[] otherCorners = box.getOtherCorners(corners[Box.CORNER_INDEX_BOTTOM_LOWER_LEFT]);
		assertEquals(otherCorners.length - 2, neighborCorners.length + oppositeCorners.length);
		
		assertEquals(Vec4.ZERO, box.transformModelToBoxOrigin(origin.toHomogeneousPoint3()));
		assertEquals(Vec4.ZERO, box.transformModelToBoxCenter(center.toHomogeneousPoint3()));
		
		box = new Box(nHalf, axes, 2, 2, 2);
		assertEquals(half, box.getCenter());
		assertTrue(box.isCenter(box.getCenter()));
		assertTrue(box.isOrigin(box.getOrigin()));
		assertTrue(box.isCorner(box.getOrigin()));
		assertEquals(Vec4.ONE.add3(half), box.getOrigin().add3(two));
		assertEquals(Vec4.ONE.add3(half), box.getOppositeCorners(box.getOrigin())[1]);
		
		Vec4 nOne = new Vec4(-1.0d, -1.0d, -1.0d);
		box = new Box(nOne, axes, 1d, 1d, 1d);
		assertEquals(nHalf, box.getCenter());
		assertTrue(box.isCenter(box.getCenter()));
		assertTrue(box.isOrigin(box.getOrigin()));
		assertTrue(box.isCorner(box.getOrigin()));
		assertEquals(nOne.add3(Vec4.ONE), box.getOrigin().add3(Vec4.ONE));
		assertEquals(nOne.add3(Vec4.ONE), box.getOppositeCorners(box.getOrigin())[1]);
		
		Box box1 = new Box(axes, 10, 20, 10, 20, 10, 20);
		Box box2 = new Box(axes, 5, 15, 5, 15, 5, 15);
		Box box3 = new Box(axes, 20, 21, 20, 21, 20, 21);
		Box box4 = new Box(axes, 15, 20, 5, 15, 5, 15);
		Box box5 = new Box(axes, -5, -4, -5, -4, -5, -4);
		Box box6 = new Box(axes, -4, 1, -4, 1, -4, 1);
		assertTrue(box.intersects(box));
		assertTrue(box1.intersects(box1));
		assertTrue(box2.intersects(box2));
		assertTrue(box1.intersects(box2));
		assertTrue(box2.intersects(box1));
		assertFalse(box.intersects(box1));
		assertFalse(box.intersects(box2));
		assertTrue(box1.intersects(box3));
		assertTrue(box3.intersects(box1));
		assertTrue(box2.intersects(box4));
		assertFalse(box5.intersects(box4));
		assertTrue(box6.intersects(box5));
	}
	
	/**
	 * Tests box intersections.
	 */
	@Test
	public void testIntersections() {
		//Vec4[] axes = { Vec4.UNIT_X, Vec4.UNIT_Y, Vec4.UNIT_Z };
		Vec4[] axes = { new Vec4(1, 0, 0), new Vec4(0, 1, 0), new Vec4(0, 0, 1) };
		Box box1 = new Box(axes, 10, 20, 10, 20, 10, 20);
		
		Line line = Line.fromSegment(box1.getCorners()[0], box1.getCorners()[6]);
		Intersection[] intersections = box1.intersect(line);
		assertTrue(null != intersections);
		assertEquals(2, intersections.length);
		assertFalse(intersections[0].isTangent());
		assertFalse(intersections[1].isTangent());
		assertEquals(box1.getCorners()[0], intersections[0].getIntersectionPoint());
		assertEquals(box1.getCorners()[6], intersections[1].getIntersectionPoint());
		
		line = Line.fromSegment(box1.getCorners()[0], box1.getCorners()[1]);
		intersections = box1.intersect(line);
		assertTrue(null != intersections);
		assertEquals(2, intersections.length);
		assertTrue(intersections[0].isTangent());
		assertTrue(intersections[1].isTangent());
		assertEquals(box1.getCorners()[0], intersections[0].getIntersectionPoint());
		assertEquals(box1.getCorners()[1], intersections[1].getIntersectionPoint());
	}
	
	/**
	 * Tests box transformations.
	 */
	@Test
	public void testTransformations() {
		Sector sector = new Sector(
            	Angle.fromDegrees(50.0),
            	Angle.fromDegrees(60.0),
            	Angle.fromDegrees(-15.0),
            	Angle.fromDegrees(5.0));
        gov.nasa.worldwind.geom.Box bb = Sector.computeBoundingBox(new Earth(), 1.0, sector, 0.0, 500000.0);
        Box box = new Box(bb);
        //Vec4 point = new Vec4(-889843.9756064053, 5581603.786233872, 3111118.000796212, 1.0);
        Vec4 point = box.getCenter();
        assertTrue(box.getFrustum().contains(point));
        
        Vec4[] corners = box.getCorners();
        assertEquals(Vec4.ZERO, box.transformModelToBoxOrigin(corners[0]));
        assertEquals(new PrecisionVec4(box.getOrigin().add3(box.getTAxis())), new PrecisionVec4(corners[1]));
        assertEquals(new PrecisionVec4(new Vec4(0, 0, box.getTLength())), new PrecisionVec4(box.transformModelToBoxOrigin(corners[1])));
	
        Vec4 ld = new Vec4(box.getRLength(), box.getSLength(), box.getTLength());
        Vec4 hd = ld.multiply3(0.5d);
        Vec4 nhd = hd.getNegative3();
        Vec4 gd = box.getRAxis().add3(box.getSAxis()).add3(box.getTAxis());
        assertEquals(ld.getLength3(), box.getDiameter(), PrecisionDouble.EPSILON);
        assertEquals(new PrecisionVec4(box.getOrigin().add3(gd)), new PrecisionVec4(corners[6]));
        assertEquals(new PrecisionVec4(Vec4.ZERO), new PrecisionVec4(box.transformModelToBoxCenter(box.getCenter())));
        assertEquals(new PrecisionVec4(nhd) , new PrecisionVec4(box.transformModelToBoxCenter(box.getOrigin())));
        
        assertEquals(new PrecisionVec4(Vec4.ZERO), new PrecisionVec4(box.transformModelToBoxOrigin(box.getOrigin())));
        assertEquals(new PrecisionVec4(ld), new PrecisionVec4(box.transformModelToBoxOrigin(corners[6])));
        assertEquals(new PrecisionVec4(hd), new PrecisionVec4(box.transformModelToBoxOrigin(box.getCenter())));
	}
	
}
