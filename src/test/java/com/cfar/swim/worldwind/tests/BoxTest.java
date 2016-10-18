package com.cfar.swim.worldwind.tests;

import static org.junit.Assert.*;

import org.junit.Test;

import com.cfar.swim.worldwind.geom.Box;

import gov.nasa.worldwind.geom.Vec4;

public class BoxTest {

	@Test
	public void testStructure() {
		Box box = new Box(Vec4.ZERO);
		Vec4 origin = box.getOrigin();
		Vec4 center = box.getCenter();
		Vec4[] corners = box.getCorners();
		// TODO: assertEquals(Vec4.ZERO, center);
		assertEquals(true, box.isCenter(center));
		assertEquals(true, box.isOrigin(origin));
		assertEquals(corners[Box.CORNER_INDEX_BOTTOM_LOWER_LEFT], origin);
		assertEquals(8, corners.length);
		
		for (int index = 0; index < corners.length; index++) {
			assertEquals(index, box.getCornerIndex(corners[index]));
		}
		
		Vec4[] neighborCorners = box.getNeighborCorners(corners[Box.CORNER_INDEX_BOTTOM_LOWER_LEFT]);
		Vec4[] oppositeCorners = box.getOppositeCorners(corners[Box.CORNER_INDEX_BOTTOM_LOWER_LEFT]);
		Vec4[] otherCorners = box.getOtherCorners(corners[Box.CORNER_INDEX_BOTTOM_LOWER_LEFT]);
		assertEquals(otherCorners.length - 2, neighborCorners.length + oppositeCorners.length);
	
		assertEquals(Vec4.ZERO, box.transformModelToBoxOrigin(origin));
		assertEquals(Vec4.ZERO, box.transformModelToBoxCenter(center));
	}

}
