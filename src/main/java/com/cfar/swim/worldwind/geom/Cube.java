package com.cfar.swim.worldwind.geom;

import gov.nasa.worldwind.geom.Vec4;

/**
 * Realizes a geometric cube that allows for convenient containment and
 * intersection checks considering numerical inaccuracies.
 * 
 * @author Stephan Heinemann
 * 
 */
public class Cube extends Box {
	
	/**
	 * Constructs a new box with specified origin, unit axes and side length.
	 * 
	 * @param origin the origin of this cube
	 * @param axes the unit axes of this cube
	 * @param length the side length along all axes of this cube
	 */
	public Cube(Vec4 origin, Vec4[] axes, double length) {
		super(origin, axes, length, length, length);
	}
	
	/**
	 * Gets the side length of this cube.
	 * 
	 * @return the side length of this cube.
	 */
	public double getLength() {
		return this.rLength;
	}

}
