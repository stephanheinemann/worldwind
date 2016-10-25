package com.cfar.swim.worldwind.tests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import com.cfar.swim.worldwind.geom.Box;

import gov.nasa.worldwind.geom.Intersection;
import gov.nasa.worldwind.geom.Line;
import gov.nasa.worldwind.geom.Vec4;

public class BoxTest {

	@Test
	public void testStructure() {
		Box box = new Box(Vec4.ZERO);
		//Vec4[] axes = { Vec4.UNIT_X, Vec4.UNIT_Y, Vec4.UNIT_Z };
		Vec4[] axes = { new Vec4(1, 0, 0), new Vec4(0, 1, 0), new Vec4(0, 0, 1) };
		Vec4 origin = box.getOrigin();
		Vec4 center = box.getCenter();
		Vec4[] corners = box.getCorners();
		// TODO: center does not equal center used for construction?
		//assertEquals(Vec4.ZERO, center);
		assertTrue(box.isCenter(center));
		assertTrue(box.isOrigin(origin));
		assertEquals(corners[Box.CORNER_INDEX_BOTTOM_LOWER_LEFT], origin);
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

}
