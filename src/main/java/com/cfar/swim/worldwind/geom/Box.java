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
package com.cfar.swim.worldwind.geom;

import gov.nasa.worldwind.geom.Cylinder;
import gov.nasa.worldwind.geom.Matrix;
import gov.nasa.worldwind.geom.Vec4;

/**
 * Realizes a geometric box that allows for convenient containment and
 * intersection checks considering numerical inaccuracies.
 * 
 * @author Stephan Heinemann
 * 
 */
public class Box extends gov.nasa.worldwind.geom.Box {

	/**
	 * the bottom lower left corner index of this box
	 */
	public static final int CORNER_INDEX_BOTTOM_LOWER_LEFT = 0;
	
	/**
	 * the bottom lower right corner index of this box
	 */
	public static final int CORNER_INDEX_BOTTOM_LOWER_RIGHT = 1;
	
	/**
	 * the bottom upper right corner index of this box
	 */
	public static final int CORNER_INDEX_BOTTOM_UPPER_RIGHT = 2;
	
	/**
	 * the bottom upper left corner index of this box
	 */
	public static final int CORNER_INDEX_BOTTOM_UPPER_LEFT = 3;
	
	/**
	 * the top lower left corner index of this box
	 */
	public static final int CORNER_INDEX_TOP_LOWER_LEFT = 4;
	
	/**
	 * the bottom lower right corner index of this box
	 */
	public static final int CORNER_INDEX_TOP_LOWER_RIGHT = 5;
	
	/**
	 * the top upper right corner index of this box
	 */
	public static final int CORNER_INDEX_TOP_UPPER_RIGHT = 6;
	
	/**
	 * the top upper left corner index of this box
	 */
	public static final int CORNER_INDEX_TOP_UPPER_LEFT = 7;
	
	/**
	 * the epsilon used compensate for numerical inaccuracies
	 */
	protected static final double EPSILON = 1E-8;
	
	/**
	 * @see gov.nasa.worldwind.geom.Box#Box(Vec4)
	 */
	public Box(Vec4 point) {
		super(point);
	}
	
	/**
	 * @see gov.nasa.worldwind.geom.Box#Box(Vec4[], double, double, double, double, double, double)
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
	 * Indicates whether or not two vectors are equals considering numerical
	 * inaccuracies.
	 * 
	 * @param u the first vector
	 * @param v the second vector
	 * @return true if two vectors are equals considering numerical inaccuracies,
	 *         false otherwise
	 */
	protected static boolean equalsEpsilon(Vec4 u, Vec4 v) {
		// TODO: data type should features its own methods, e.g., NumVec4.equals()
		return Box.equalsEpsilon(u.x, v.x) &&
				Box.equalsEpsilon(u.y, v.y) &&
				Box.equalsEpsilon(u.z, v.z) &&
				Box.equalsEpsilon(u.w, v.w);
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
	 * Indicates whether or not a point in world model coordinates equals a
	 * corner of this box considering numerical inaccuracies. 
	 * 
	 * @param point the point in world model coordinates
	 * @return true if the point equals a corner of this box considering
	 *         numerical inaccuracies, false otherwise
	 */
	public boolean isCorner(Vec4 point) {
		boolean isCorner = false;
		
		for (Vec4 corner : this.getCorners()) {
			if (Box.equalsEpsilon(point, corner)) {
				isCorner = true;
				break;
			}
		}
		
		return isCorner;
	}
	
	/**
	 * Indicates whether or not a point in world model coordinates equals the
	 * center of this box considering numerical inaccuracies.
	 * 
	 * @param point the point in world model coordinates
	 * @return true if the point equals the center of this box considering
	 *         numerical inaccuracies, false otherwise
	 */
	public boolean isCenter(Vec4 point) {
		return Box.equalsEpsilon(point, this.getCenter());
	}
	
	/**
	 * Gets the corner index of a specified corner.
	 * 
	 * @param corner the corner
	 * @return the corner index of the corner if a corner,
	 *         -1 otherwise
	 */
	public int getCornerIndex(Vec4 corner) {
		int cornerIndex = -1;
		Vec4[] corners = this.getCorners();
		
		for (int index = 0; index < 8; index++) {
			if (Box.equalsEpsilon(corner, corners[index])) {
				cornerIndex = index;
				break;
			}
		}
		
		return cornerIndex;
	}
	
	/**
	 * Gets the neighbor corners of a specified corner of this box.
	 * 
	 * @param corner the corner of this box
	 * @return the neighbor corners of the corner if a corner,
	 *         null otherwise
	 */
	public Vec4[] getNeighborCorners(Vec4 corner) {
		Vec4[] neighborCorners = null;
		Vec4[] corners = this.getCorners();
		int cornerIndex = this.getCornerIndex(corner);
		
		if ((cornerIndex >= 0) && cornerIndex < corners.length) {
			neighborCorners = new Vec4[3];
			if (Box.CORNER_INDEX_BOTTOM_LOWER_LEFT == cornerIndex) {
				neighborCorners[0] = corners[Box.CORNER_INDEX_BOTTOM_LOWER_RIGHT];
				neighborCorners[1] = corners[Box.CORNER_INDEX_BOTTOM_UPPER_LEFT];
				neighborCorners[2] = corners[Box.CORNER_INDEX_TOP_LOWER_LEFT];
			} else if (Box.CORNER_INDEX_BOTTOM_LOWER_RIGHT == cornerIndex) {
				neighborCorners[0] = corners[Box.CORNER_INDEX_BOTTOM_LOWER_LEFT];
				neighborCorners[1] = corners[Box.CORNER_INDEX_BOTTOM_UPPER_RIGHT];
				neighborCorners[2] = corners[Box.CORNER_INDEX_TOP_LOWER_RIGHT];
			} else if (Box.CORNER_INDEX_BOTTOM_UPPER_RIGHT == cornerIndex) {
				neighborCorners[0] = corners[Box.CORNER_INDEX_BOTTOM_LOWER_RIGHT];
				neighborCorners[1] = corners[Box.CORNER_INDEX_BOTTOM_UPPER_LEFT];
				neighborCorners[2] = corners[Box.CORNER_INDEX_TOP_UPPER_RIGHT];
			} else if (Box.CORNER_INDEX_BOTTOM_UPPER_LEFT == cornerIndex) {
				neighborCorners[0] = corners[Box.CORNER_INDEX_BOTTOM_LOWER_LEFT];
				neighborCorners[1] = corners[Box.CORNER_INDEX_BOTTOM_UPPER_RIGHT];
				neighborCorners[2] = corners[Box.CORNER_INDEX_TOP_UPPER_LEFT];
			} else if (Box.CORNER_INDEX_TOP_LOWER_LEFT == cornerIndex) {
				neighborCorners[0] = corners[Box.CORNER_INDEX_BOTTOM_LOWER_LEFT];
				neighborCorners[1] = corners[Box.CORNER_INDEX_TOP_LOWER_RIGHT];
				neighborCorners[2] = corners[Box.CORNER_INDEX_TOP_UPPER_LEFT];
			} else if (Box.CORNER_INDEX_TOP_LOWER_RIGHT == cornerIndex) {
				neighborCorners[0] = corners[Box.CORNER_INDEX_BOTTOM_LOWER_RIGHT];
				neighborCorners[1] = corners[Box.CORNER_INDEX_TOP_LOWER_LEFT];
				neighborCorners[2] = corners[Box.CORNER_INDEX_TOP_UPPER_RIGHT];
			} else if (Box.CORNER_INDEX_TOP_UPPER_RIGHT == cornerIndex) {
				neighborCorners[0] = corners[Box.CORNER_INDEX_BOTTOM_UPPER_RIGHT];
				neighborCorners[1] = corners[Box.CORNER_INDEX_TOP_LOWER_RIGHT];
				neighborCorners[2] = corners[Box.CORNER_INDEX_TOP_UPPER_LEFT];
			} else if (Box.CORNER_INDEX_TOP_UPPER_LEFT == cornerIndex) {
				neighborCorners[0] = corners[Box.CORNER_INDEX_BOTTOM_UPPER_LEFT];
				neighborCorners[1] = corners[Box.CORNER_INDEX_TOP_LOWER_LEFT];
				neighborCorners[2] = corners[Box.CORNER_INDEX_TOP_UPPER_RIGHT];
			}
		}
		
		return neighborCorners;
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
		// TODO: box orientation seems to be arbitrary and requires corner point calculation instead of direct transformation
		Vec4 localPoint = this.transformModelToBoxCenter(modelPoint);
		Vec4 halfDiagonal = (new Vec4(this.rLength, this.sLength, this.tLength)).multiply3(0.5);
		return localPoint.add3(halfDiagonal);
	}
	
	/**
	 * Transforms a Cartesian world model vector into a box vector
	 * using the center of this box as origin.
	 * 
	 * @param modelPoint the world model vector
	 * @return the box vector
	 */
	public Vec4 transformModelToBoxCenter(Vec4 modelPoint) {
		Vec4[] unitAxes = {this.ru, this.su, this.tu};
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
	
	/**
	 * Indicates whether or not a line segment intersects this box when
	 * expanding its sides with an expansion vector.
	 * 
	 * @param p0 the first point of the line segment
	 * @param p1 the second point of the line segment
	 * @param expansion the expansion vector to expand the box
	 * @return true if the line segment intersects this box when expanding its
	 *         sides with the expansion vector, false otherwise
	 */
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
	
	/**
	 * Indicates whether or not a line segment intersects this box.
	 * 
	 * @param p0 the first point of the line segment
	 * @param p1 the second point of the line segment
	 * @return true if the line segment intersects this box, false otherwise
	 */
	public boolean intersectsLineSegment(Vec4 p0, Vec4 p1) {
		return this.intersectsLineSegment(p0, p1, Vec4.ZERO);
	}
	
	/**
	 * Indicates whether or not (the bounding box of) a cylinder intersects this
	 * box. 
	 * 
	 * @param cylinder the cylinder
	 * @return true if the (bounding box of the) cylinder intersects this box,
	 *         false otherwise
	 */
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
