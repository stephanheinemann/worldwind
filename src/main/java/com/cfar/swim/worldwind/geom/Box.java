package com.cfar.swim.worldwind.geom;

import gov.nasa.worldwind.geom.Cylinder;
import gov.nasa.worldwind.geom.Matrix;
import gov.nasa.worldwind.geom.Vec4;

public class Box extends gov.nasa.worldwind.geom.Box {

	/**
	 * the epsilon used compensate for numerical inaccuracies
	 */
	protected static final double EPSILON = 1E-8;
	
	/**
	 * @see gov.nasa.worldwind.geom.Box
	 */
	public Box(Vec4 point) {
		super(point);
	}
	
	/**
	 * @see gov.nasa.worldwind.geom.Box
	 */
	public Box(
			Vec4[] axes,
			double rMin, double rMax,
			double sMin, double sMax,
			double tMin, double tMax) {
		super(axes, rMin, rMax, sMin, sMax, tMin, tMax);
	}
	
	/**
	 * Constructs a new box from a geometric box.
	 * 
	 * @param box the geometric box
	 * 
	 * @see gov.nasa.worldwind.geom.Box
	 */
	public Box(gov.nasa.worldwind.geom.Box box) {
		super(
			box.getBottomCenter(),
			box.getTopCenter(),
			box.getCenter(),
			box.getRAxis(),
			box.getSAxis(),
			box.getTAxis(),
			box.getUnitRAxis(),
			box.getUnitSAxis(),
			box.getUnitTAxis(),
			box.getRLength(),
			box.getSLength(),
			box.getTLength(),
			box.getPlanes()
			);
	}
	
	/**
	 * Indicates whether or not two values are equal considering numerical
	 * inaccuracies.
	 * 
	 * @param a the first value
	 * @param b the second value
	 * 
	 * @return true if two values are equal considering numerical inaccuracies,
	 *         false otherwise
	 */
	protected static boolean equalsEpsilon(double a, double b) {
		return Math.abs(a - b) < EPSILON;
	}
	
	/**
	 * Indicates whether or not a value lies is within a range considering
	 * numerical inaccuracies.
	 * 
	 * @param d the value
	 * @param l the lower bound of the range
	 * @param u the upper bound of the range
	 * 
	 * @return true if the value lies within the range considering numerical
	 *         inaccuracies, false otherwise
	 */
	protected static boolean isInRangeEpsilon(double d, double l, double u) {
		return ((l - EPSILON) <= d) && ((u + EPSILON) >= d);
	}
	
	/**
	 * Transforms a Cartesian world model vector into a box vector
	 * using the first corner of this box as origin.
	 * 
	 * @param modelPoint the world model vector
	 * 
	 * @return the box vector
	 */
	public Vec4 transformModelToBoxOrigin(Vec4 modelPoint) {
		Vec4[] unitAxes = {ru, su, tu};
		Vec4 origin = this.getCorners()[0];
		Matrix transformMatrix = Matrix.fromLocalOrientation(origin , unitAxes).getInverse();
		return modelPoint.transformBy4(transformMatrix);
	}
	
	// TODO: documentation
	public Vec4 transformModelToBoxCenter(Vec4 modelPoint) {
		Vec4[] unitAxes = {ru, su, tu};
		/*
		Vec4 x = this.getCorners()[1].subtract3(this.getCorners()[0]);
		Vec4 y = this.getCorners()[3].subtract3(this.getCorners()[0]);
		Vec4 z = this.getCorners()[4].subtract3(this.getCorners()[0]);
		Vec4[] unitAxes = {x, y, z};
		*/
		
		Vec4 origin = this.getCenter();
		Matrix transformMatrix = Matrix.fromLocalOrientation(origin , unitAxes).getInverse();
		return modelPoint.transformBy4(transformMatrix);
	}
	
	/**
	 * Indicates whether or not a value lies on the <code>R</code> axis of this
	 * box considering numerical inaccuracies.
	 * 
	 * @param r the value
	 * 
	 * @return true if the value lies on the <code>R</code> axis considering
	 *         numerical inaccuracies, false otherwise
	 */
	protected boolean containsREpsilon(double r) {
		return Box.isInRangeEpsilon(r, 0.0, this.rLength);
	}
	
	/**
	 * Indicates whether or not a value lies on the <code>S</code> axis of this
	 * box considering numerical inaccuracies.
	 * 
	 * @param s the value
	 * 
	 * @return true if the value lies on the <code>S</code> axis considering
	 *         numerical inaccuracies, false otherwise
	 */
	protected boolean containsSEpsilon(double s) {
		return Box.isInRangeEpsilon(s, 0.0, this.sLength);
	}
	
	/**
	 * Indicates whether or not a value lies on the <code>T</code> axis of this
	 * box considering numerical inaccuracies.
	 * 
	 * @param t the value
	 * 
	 * @return true if the value lies on the <code>T</code> axis considering
	 *         numerical inaccuracies, false otherwise
	 */
	protected boolean containsTEpsilon(double t) {
		return Box.isInRangeEpsilon(t, 0.0, this.tLength);
	}
	
	/**
	 * Indicates whether or not a vector in box coordinates is contained
	 * in this box considering numerical inaccuracies.
	 * 
	 * @param v the vector in box coordinates
	 * 
	 * @return true if this box contains the vector, false otherwise
	 */
	protected boolean containsV(Vec4 v) {
		return this.containsREpsilon(v.x) && this.containsSEpsilon(v.y) && this.containsTEpsilon(v.z);
	}
	
	/**
	 * Indicates whether or not a point in world model coordinates is
	 * contained in this box.
	 * 
	 * @param modelPoint the point in world model coordinates
	 * 
	 * @return true if this regular grid contains the point, false otherwise
	 */
	public boolean contains(Vec4 modelPoint) {
		boolean contains = false;
		
		Vec4 localPoint = this.transformModelToBoxOrigin(modelPoint);
		if (this.containsV(localPoint)) {
			contains = true;
		}
		
		return contains;
	}
	
	protected boolean intersectsLineSegment(Vec4 p0, Vec4 p1, Vec4 expansion) {
		// transform line segment end points to box coordinates 
		p0 = this.transformModelToBoxCenter(p0);
		p1 = this.transformModelToBoxCenter(p1);
		
		// line segment midpoint in box coordinates
		Vec4 midpoint = p0.add3(p1).multiply3(0.5);
		Vec4 midpointLength = midpoint.getAbs3();
		
		// line half segment in box coordinates
		Vec4 halfSegment = p1.subtract3(midpoint);
		Vec4 halfSegmentLength = halfSegment.getAbs3();
		
		// box sides half lengths
		Vec4 halfSideLength = new Vec4(this.rLength, this.sLength, this.tLength);
		halfSideLength = halfSideLength.multiply3(0.5);
		halfSideLength = halfSideLength.add3(expansion);
		
		// world coordinate axes as separating axes
		if (midpointLength.x > (halfSideLength.x + halfSegmentLength.x))
			return false; // segment cannot intersect on x axis
		if (midpointLength.y > (halfSideLength.y + halfSegmentLength.y))
			return false; // segment cannot intersect on y axis
		if (midpointLength.z > (halfSideLength.z + halfSegmentLength.z))
			return false; // segment cannot intersect on z axis
		
		halfSegmentLength = halfSegmentLength.add3(Box.EPSILON, Box.EPSILON, Box.EPSILON);
		
		// cross products of segment direction vector with coordinate axes
		if (Math.abs((midpoint.y * halfSegment.z) - (midpoint.z * halfSegment.y)) >
			((halfSideLength.y * halfSegmentLength.z) + (halfSideLength.z * halfSegmentLength.y)))
			return false;
		if (Math.abs((midpoint.z * halfSegment.x) - (midpoint.x * halfSegment.z)) >
			((halfSideLength.x * halfSegmentLength.z) + (halfSideLength.z * halfSegmentLength.x)))
			return false;
		if (Math.abs((midpoint.x * halfSegment.y) - (midpoint.y * halfSegment.x)) >
			((halfSideLength.x * halfSegmentLength.y) + (halfSideLength.y * halfSegmentLength.x)))
			return false;
		
		return true;
	}
	
	public boolean intersectsLineSegment(Vec4 p0, Vec4 p1) {
		return this.intersectsLineSegment(p0, p1, Vec4.ZERO);
	}
	
	public boolean intersectsCylinder(Cylinder cylinder) {
		// expand box by effective cylinder radii
		double rx = cylinder.getEffectiveRadius(this.planes[0]); // r-min plane
		double ry = cylinder.getEffectiveRadius(this.planes[2]); // s-min plane
		double rz = cylinder.getEffectiveRadius(this.planes[4]); // t-min plane
		// the expansion includes the bounding box of the cylinder
		Vec4 expansion = new Vec4(rx, ry, rz);
		// perform the conservatively approximate (cylinder bounding box) intersection check
		return this.intersectsLineSegment(cylinder.getBottomCenter(), cylinder.getTopCenter(), expansion);
	} 
	
}
