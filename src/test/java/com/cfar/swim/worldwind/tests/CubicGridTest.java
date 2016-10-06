package com.cfar.swim.worldwind.tests;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import com.cfar.swim.worldwind.geom.Cube;
import com.cfar.swim.worldwind.geom.CubicGrid;
import com.cfar.swim.worldwind.geom.precision.PrecisionDouble;

import gov.nasa.worldwind.geom.Vec4;

public class CubicGridTest {
	
	@Test
	public void testConstruction() {
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
        	assertEquals(1.0, grid.getNormalizer(), PrecisionDouble.EPSILON);
        }
        
        cubicGrid.getChild(0, 0, 0).addChildren(0.5d);
        
        for (CubicGrid grid : cubicGrid.getAll()) {
        	assertEquals(0.5, grid.getNormalizer(), PrecisionDouble.EPSILON);
        }
        
        cubicGrid.getChild(0, 0, 1).addChildren(2);
        
        for (CubicGrid grid : cubicGrid.getAll()) {
        	assertEquals(0.5, grid.getNormalizer(), PrecisionDouble.EPSILON);
        }
        
        cubicGrid.getChild(0, 0, 1).getChild(0, 0, 0).addChildren(10);
        
        for (CubicGrid grid : cubicGrid.getAll()) {
        	assertEquals(0.05, grid.getNormalizer(), PrecisionDouble.EPSILON);
        }
		
        cubicGrid.getChild(0, 0, 0).removeChildren();
        
        for (CubicGrid grid : cubicGrid.getAll()) {
        	assertEquals(0.05, grid.getNormalizer(), PrecisionDouble.EPSILON);
        }
        
        cubicGrid.getChild(0, 0, 1).getChild(0, 0, 0).removeChildren();
        
        for (CubicGrid grid : cubicGrid.getAll()) {
        	assertEquals(0.5, grid.getNormalizer(), PrecisionDouble.EPSILON);
        }
        
        cubicGrid.getChild(0, 0, 1).removeChildren();
        
        for (CubicGrid grid : cubicGrid.getAll()) {
        	assertEquals(1.0, grid.getNormalizer(), PrecisionDouble.EPSILON);
        }
        
        cubicGrid.removeChildren();
        
        for (CubicGrid grid : cubicGrid.getAll()) {
        	assertEquals(grid.getLength(), grid.getNormalizer(), PrecisionDouble.EPSILON);
        }
	}

}
