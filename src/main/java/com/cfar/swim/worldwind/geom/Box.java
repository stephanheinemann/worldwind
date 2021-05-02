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
package com.cfar.swim.worldwind.geom;

import com.cfar.swim.worldwind.geom.precision.PrecisionDouble;
import com.cfar.swim.worldwind.geom.precision.PrecisionVec4;

import gov.nasa.worldwind.geom.Cylinder;
import gov.nasa.worldwind.geom.Extent;
import gov.nasa.worldwind.geom.Frustum;
import gov.nasa.worldwind.geom.Line;
import gov.nasa.worldwind.geom.Matrix;
import gov.nasa.worldwind.geom.Plane;
import gov.nasa.worldwind.geom.TransformationMatrix;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.util.Logging;

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
	 * the origin of this box
	 */
	protected Vec4 origin = Vec4.ZERO;
	
	/**
	 * the local transformation matrix of this box
	 */
	protected Matrix toLocalOrigin = null;
	
	/**
	 * {@inheritDoc}
	 * @see gov.nasa.worldwind.geom.Box#Box(Vec4)
	 */
	public Box(Vec4 point) {
		// TODO: file bug report for worldwind 2.0 
		// gov.nasa.worldwind.geom.Box#Box(Vec4)
		// constructs box completely wrong - center and dimensions 
		//super(point);
		this(Box.createInstance(point));
		this.origin = super.getCorners()[Box.CORNER_INDEX_BOTTOM_LOWER_LEFT];
		this.toLocalOrigin = TransformationMatrix.toLocalOrientation(this.origin, this.getAxes());
	}
	
	/**
	 * {@inheritDoc}
	 * @see gov.nasa.worldwind.geom.Box#Box(Vec4[], double, double, double, double, double, double)
	 */
	public Box(
			Vec4[] axes,
			double rMin, double rMax,
			double sMin, double sMax,
			double tMin, double tMax) {
		super(axes, rMin, rMax, sMin, sMax, tMin, tMax);
		this.origin = super.getCorners()[Box.CORNER_INDEX_BOTTOM_LOWER_LEFT];
		this.toLocalOrigin = TransformationMatrix.toLocalOrientation(this.origin, this.getAxes());
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
		this.origin = super.getCorners()[Box.CORNER_INDEX_BOTTOM_LOWER_LEFT];
		this.toLocalOrigin = TransformationMatrix.toLocalOrientation(this.origin, this.getAxes());
	}
	
	/**
	 * Constructs a new box with specified origin, unit axes and side lengths.
	 * 
	 * @param origin the origin of this box
	 * @param axes the unit axes of this box
	 * @param rLength the side length along the <code>R</code> axes of this box
	 * @param sLength the side length along the <code>S</code> axes of this box
	 * @param tLength the side length along the <code>T</code> axes of this box
	 */
	public Box(Vec4 origin, Vec4[] axes, double rLength, double sLength, double tLength) {
		super(
			axes,
			axes[0].dot3(origin), axes[0].dot3(origin) + rLength,
			axes[1].dot3(origin), axes[1].dot3(origin) + sLength,
			axes[2].dot3(origin), axes[2].dot3(origin) + tLength);
		this.origin = origin;
		this.toLocalOrigin = TransformationMatrix.toLocalOrientation(this.origin, this.getAxes());
	}
	
	/**
	 * {@inheritDoc}
	 */
	protected Box(
			Vec4 bottomCenter, Vec4 topCenter, Vec4 center,
			Vec4 r, Vec4 s, Vec4 t, Vec4 ru, Vec4 su, Vec4 tu,
	        double rLength, double sLength, double tLength,
	        Plane[] planes) {
		super(bottomCenter, topCenter, center, r, s, t, ru, su, tu, rLength, sLength, tLength, planes);
		this.origin = super.getCorners()[Box.CORNER_INDEX_BOTTOM_LOWER_LEFT];
		this.toLocalOrigin = TransformationMatrix.toLocalOrientation(this.origin, this.getAxes());
	}
	
	/**
	 * Creates a new unit box centered at a specified point.
	 * 
	 * @param center the center of the unit box in model model coordinates
	 * 
	 * @return a new unit box centered at the center point
	 */
	protected static Box createInstance(Vec4 center) {
		if (center == null)
        {
            String msg = Logging.getMessage("nullValue.PointIsNull");
            Logging.logger().severe(msg);
            throw new IllegalArgumentException(msg);
        }

        Vec4 ru = new Vec4(1, 0, 0, 1);
        Vec4 su = new Vec4(0, 1, 0, 1);
        Vec4 tu = new Vec4(0, 0, 1, 1);

        Vec4 r = ru;
        Vec4 s = su;
        Vec4 t = tu;

        double rLength = 1d;
        double sLength = 1d;
        double tLength = 1d;

        // Plane normals point outwards from the box.
        Plane[] planes = new Plane[6];
        double d = center.getLength3();
        double dr = (center.x < 0) ? 0.5d : -0.5d;
        double ds = (center.y < 0) ? 0.5d : -0.5d;
        double dt = (center.z < 0) ? 0.5d : -0.5d;
        planes[0] = new Plane(-ru.x, -ru.y, -ru.z, d + dr);
        planes[1] = new Plane(+ru.x, +ru.y, +ru.z, d - dr);
        planes[2] = new Plane(-su.x, -su.y, -su.z, d + ds);
        planes[3] = new Plane(+su.x, +su.y, +su.z, d - ds);
        planes[4] = new Plane(-tu.x, -tu.y, -tu.z, d + dt);
        planes[5] = new Plane(+tu.x, +tu.y, +tu.z, d - dt);

        Vec4 rHalf = r.multiply3(0.5);
        Vec4 topCenter = center.add3(rHalf);
        Vec4 bottomCenter = center.subtract3(rHalf);
        
        return new Box(
        		bottomCenter, topCenter, center,
        		r, s, t, ru, su, tu,
		        rLength, sLength, tLength,
		        planes);
	}
	
	/**
	 * Gets the axes of this box.
	 * 
	 * @return the axes of this box
	 */
	public Vec4[] getAxes() {
		return new Vec4[] {this.r, this.s, this.t };
	}
	
	/**
	 * Gets the unit axes of this box.
	 * 
	 * @return the unit axes of this box
	 */
	public Vec4[] getUnitAxes() {
		return new Vec4[] {this.ru, this.su, this.tu};
	}
	
	/**
	 * Gets the origin of this box.
	 * 
	 * @return the origin of this box
	 */
	public Vec4 getOrigin() {
		return this.origin;
	}
	
	/**
	 * Determines whether or not a point in world model coordinates equals the
	 * origin of this box considering numerical inaccuracies.
	 * 
	 * @param point the point in world model coordinates
	 * 
	 * @return true if the point equals the origin of this box considering
	 *         numerical inaccuracies, false otherwise
	 */
	public boolean isOrigin(Vec4 point) {
		PrecisionVec4 v1 = new PrecisionVec4(point.toHomogeneousPoint3());
		PrecisionVec4 v2 = new PrecisionVec4(this.origin.toHomogeneousPoint3());
		return v1.equals(v2);
	}
	
	/**
	 * Determines whether or not a point in world model coordinates equals the
	 * center of this box considering numerical inaccuracies.
	 * 
	 * @param point the point in world model coordinates
	 * 
	 * @return true if the point equals the center of this box considering
	 *         numerical inaccuracies, false otherwise
	 */
	public boolean isCenter(Vec4 point) {
		PrecisionVec4 v1 = new PrecisionVec4(point.toHomogeneousPoint3());
		PrecisionVec4 v2 = new PrecisionVec4(this.getCenter().toHomogeneousPoint3());
		return v1.equals(v2);
	}
	
	/**
	 * Determines whether or not a point in world model coordinates equals a
	 * corner of this box considering numerical inaccuracies. 
	 * 
	 * @param point the point in world model coordinates
	 * 
	 * @return true if the point equals a corner of this box considering
	 *         numerical inaccuracies, false otherwise
	 */
	public boolean isCorner(Vec4 point) {
		boolean isCorner = false;
		
		for (Vec4 corner : super.getCorners()) {
			PrecisionVec4 v1 = new PrecisionVec4(point.toHomogeneousPoint3());
			PrecisionVec4 v2 = new PrecisionVec4(corner.toHomogeneousPoint3());
			if (v1.equals(v2)) {
				isCorner = true;
				break;
			}
		}
		
		return isCorner;
	}
	
	/**
	 * Gets the corner index of a specified corner.
	 * 
	 * @param corner the corner
	 * 
	 * @return the corner index of the corner if a corner, -1 otherwise
	 */
	public int getCornerIndex(Vec4 corner) {
		int cornerIndex = -1;
		Vec4[] corners = super.getCorners();
		
		for (int index = 0; index < 8; index++) {
			PrecisionVec4 v1 = new PrecisionVec4(corner.toHomogeneousPoint3());
			PrecisionVec4 v2 = new PrecisionVec4(corners[index].toHomogeneousPoint3());
			if (v1.equals(v2)) {
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
	 * 
	 * @return the neighbor corners of the corner if a corner, null otherwise
	 */
	public Vec4[] getNeighborCorners(Vec4 corner) {
		Vec4[] neighborCorners = null;
		Vec4[] corners = super.getCorners();
		int cornerIndex = this.getCornerIndex(corner);
		
		if ((cornerIndex >= 0) && (cornerIndex < corners.length)) {
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
	 * Gets the opposite corners of a specified corner of this box.
	 * 
	 * @param corner the corner of this box
	 * 
	 * @return the opposite corners of the corner if a corner, null otherwise
	 */
	public Vec4[] getOppositeCorners(Vec4 corner) {
		Vec4[] oppositeCorners = null;
		Vec4[] corners = super.getCorners();
		int cornerIndex = this.getCornerIndex(corner);
		
		if ((cornerIndex >= 0) && (cornerIndex < corners.length)) {
			oppositeCorners = new Vec4[2];
		
			if (Box.CORNER_INDEX_BOTTOM_LOWER_LEFT == cornerIndex) {
				oppositeCorners[0] = corners[Box.CORNER_INDEX_BOTTOM_UPPER_RIGHT];
				oppositeCorners[1] = corners[Box.CORNER_INDEX_TOP_UPPER_RIGHT];
			} else if (Box.CORNER_INDEX_BOTTOM_LOWER_RIGHT == cornerIndex) {
				oppositeCorners[0] = corners[Box.CORNER_INDEX_BOTTOM_UPPER_LEFT];
				oppositeCorners[1] = corners[Box.CORNER_INDEX_TOP_UPPER_LEFT];
			} else if (Box.CORNER_INDEX_BOTTOM_UPPER_RIGHT == cornerIndex) {
				oppositeCorners[0] = corners[Box.CORNER_INDEX_BOTTOM_LOWER_LEFT];
				oppositeCorners[1] = corners[Box.CORNER_INDEX_TOP_LOWER_LEFT];
			} else if (Box.CORNER_INDEX_BOTTOM_UPPER_LEFT == cornerIndex) {
				oppositeCorners[0] = corners[Box.CORNER_INDEX_BOTTOM_LOWER_RIGHT];
				oppositeCorners[1] = corners[Box.CORNER_INDEX_TOP_LOWER_RIGHT];
			} else if (Box.CORNER_INDEX_TOP_LOWER_LEFT == cornerIndex) {
				oppositeCorners[0] = corners[Box.CORNER_INDEX_TOP_UPPER_RIGHT];
				oppositeCorners[1] = corners[Box.CORNER_INDEX_BOTTOM_UPPER_RIGHT];
			} else if (Box.CORNER_INDEX_TOP_LOWER_RIGHT == cornerIndex) {
				oppositeCorners[0] = corners[Box.CORNER_INDEX_TOP_UPPER_LEFT];
				oppositeCorners[1] = corners[Box.CORNER_INDEX_BOTTOM_UPPER_LEFT];
			} else if (Box.CORNER_INDEX_TOP_UPPER_RIGHT == cornerIndex) {
				oppositeCorners[0] = corners[Box.CORNER_INDEX_TOP_LOWER_LEFT];
				oppositeCorners[1] = corners[Box.CORNER_INDEX_BOTTOM_LOWER_LEFT];
			} else if (Box.CORNER_INDEX_TOP_UPPER_LEFT == cornerIndex) {
				oppositeCorners[0] = corners[Box.CORNER_INDEX_TOP_LOWER_RIGHT];
				oppositeCorners[1] = corners[Box.CORNER_INDEX_BOTTOM_LOWER_RIGHT];
			}
		}
		
		return oppositeCorners;
	}
	
	/**
	 * Gets the other corners of a specified corner of this box.
	 * 
	 * @param corner the corner of this box
	 * 
	 * @return the other corners of the corner if a corner, null otherwise
	 */
	public Vec4[] getOtherCorners(Vec4 corner) {
		Vec4[] otherCorners = null;
		Vec4[] corners = super.getCorners();
		int cornerIndex = this.getCornerIndex(corner);
		
		if ((cornerIndex >= 0) && (cornerIndex < corners.length)) {
			otherCorners = new Vec4[7];
			
			for (int index = 0, otherIndex = 0; index < corners.length; index++) {
				if (cornerIndex != index) {
					otherCorners[otherIndex] = corners[index];
					otherIndex++;
				}
			}
		}
		
		return otherCorners;
	}
	
	/**
	 * Transforms a Cartesian world model vector into a box vector using the
	 * first corner of this box as origin.
	 * 
	 * @param modelPoint the world model vector
	 * 
	 * @return the box vector
	 */
	public Vec4 transformModelToBoxOrigin(Vec4 modelPoint) {
		return modelPoint.transformBy4(this.toLocalOrigin);
	}
	
	/**
	 * Transforms a Cartesian world model vector into a box vector using the
	 * center of this box as origin.
	 * 
	 * @param modelPoint the world model vector
	 * 
	 * @return the box vector
	 */
	public Vec4 transformModelToBoxCenter(Vec4 modelPoint) {
		Matrix transformMatrix = TransformationMatrix.toLocalOrientation(
				this.getCenter(), this.getAxes());
		return modelPoint.transformBy4(transformMatrix);
	}
	
	/**
	 * Determines whether or not a value lies on the <code>R</code> axis of
	 * this box considering numerical inaccuracies.
	 * 
	 * @param r the value
	 * 
	 * @return true if the value lies on the <code>R</code> axis considering
	 *         numerical inaccuracies, false otherwise
	 */
	protected boolean containsR(double r) {
		return (new PrecisionDouble(r)).isInRange(0.0, this.rLength);
	}
	
	/**
	 * Determines whether or not a value lies on the <code>S</code> axis of
	 * this box considering numerical inaccuracies.
	 * 
	 * @param s the value
	 * 
	 * @return true if the value lies on the <code>S</code> axis considering
	 *         numerical inaccuracies, false otherwise
	 */
	protected boolean containsS(double s) {
		return (new PrecisionDouble(s)).isInRange(0.0, this.sLength);
	}
	
	/**
	 * Determines whether or not a value lies on the <code>T</code> axis of
	 * this box considering numerical inaccuracies.
	 * 
	 * @param t the value
	 * 
	 * @return true if the value lies on the <code>T</code> axis considering
	 *         numerical inaccuracies, false otherwise
	 */
	protected boolean containsT(double t) {
		return (new PrecisionDouble(t)).isInRange(0.0, this.tLength);
	}
	
	/**
	 * Determines whether or not a vector in box coordinates is contained in
	 * this box considering numerical inaccuracies.
	 * 
	 * @param v the vector in box coordinates
	 * 
	 * @return true if this box contains the vector, false otherwise
	 */
	protected boolean containsV(Vec4 v) {
		return this.containsR(v.x) && this.containsS(v.y) && this.containsT(v.z);
	}
	
	/**
	 * Determines whether or not a point in world model coordinates is
	 * contained in this box.
	 * 
	 * @param modelPoint the point in world model coordinates
	 * 
	 * @return true if this box contains the point, false otherwise
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
	 * Determines whether or not a line segment intersects this box when
	 * expanding its sides with an expansion vector.
	 * 
	 * @param p0 the first point of the line segment
	 * @param p1 the second point of the line segment
	 * @param expansion the expansion vector to expand the box
	 * 
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
		
		halfSegmentLength = halfSegmentLength.add3(
			PrecisionDouble.EPSILON,
			PrecisionDouble.EPSILON,
			PrecisionDouble.EPSILON);
		
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
	 * Determines whether or not a line segment intersects this box.
	 * 
	 * @param p0 the first point of the line segment
	 * @param p1 the second point of the line segment
	 * 
	 * @return true if the line segment intersects this box, false otherwise
	 */
	public boolean intersectsLineSegment(Vec4 p0, Vec4 p1) {
		return this.intersectsLineSegment(p0, p1, Vec4.ZERO);
	}
	
	/**
	 * Determines whether or not (the bounding box of) a cylinder intersects
	 * this box. 
	 * 
	 * @param cylinder the cylinder
	 * 
	 * @return true if the (bounding box of the) cylinder intersects this box,
	 *         false otherwise
	 */
	public boolean intersectsCylinderBoundingBox(Cylinder cylinder) {
		// expand box by effective cylinder radii
		double rx = cylinder.getEffectiveRadius(this.planes[0]); // r-min plane
		double ry = cylinder.getEffectiveRadius(this.planes[2]); // s-min plane
		double rz = cylinder.getEffectiveRadius(this.planes[4]); // t-min plane
		// the expansion includes the bounding box of the cylinder
		Vec4 expansion = new Vec4(rx, ry, rz);
		// perform the conservatively approximate (cylinder bounding box) intersection check
		return this.intersectsLineSegment(cylinder.getBottomCenter(), cylinder.getTopCenter(), expansion);
	}
	
	/**
	 * Determines whether or not a cylinder intersects this box.
	 * 
	 * @param cylinder the cylinder
	 * 
	 * @return true if the cylinder intersects this box, false otherwise
	 */
	public boolean intersectsCylinder(Cylinder cylinder) {
		Vec4 boxCenter = this.getCenter();
		Vec4 cylCenter = cylinder.getCenter();
		
		// distance from box center to cylinder center
		double distanceCenter = boxCenter.distanceTo3(cylCenter);
		// vector between box center and cylinder center
		Vec4 subCenter = boxCenter.subtract3(cylCenter);
		
		// shortest distance from box corner to cylinder center 
		double distanceNearest = Double.MAX_VALUE;
		Vec4 nearestCorner = null;
		
		// nearest box corner to cylinder center
		for (Vec4 corner : this.getCorners()) {
			double distanceCorner = corner.distanceTo3(cylCenter);
			if (distanceNearest > distanceCorner) {
				distanceNearest = distanceCorner;
				nearestCorner = corner;
			}
		}
		
		// vector between nearest box corner and cylinder center
		Vec4 subNearest = nearestCorner.subtract3(cylCenter);
		
		// angle between the axis of the cylinder and the vector from the
		// center of the cylinder to the nearest corner
		double angleNearest = Math.acos(
				cylinder.getAxisUnitDirection().dot3(subNearest)
				/ subNearest.getLength3());
		
		// angle between the axis of the cylinder and the vector from the
		// center of the cylinder to the center of the box
		double angleCenter = Math.acos(
				cylinder.getAxisUnitDirection().dot3(subCenter)
				/ subCenter.getLength3());
		
		// critical angle of the cylinder is the angle that separates the
		// bottom and top planes from the lateral surface
		double angleCritical = Math.atan(
				cylinder.getCylinderRadius()
				/ (cylinder.getCylinderHeight() / 2));
		
		// maxDistNearest and maxDistCenter are the maximum distances between
		// the center of the cylinder and its surface in the direction (angle)
		// of the nearest corner and center of the box, respectively
		double maxDistNearest = 0, maxDistCenter = 0;
		
		if ((angleNearest <= angleCritical)
				|| (angleNearest >= (Math.PI - angleCritical))) {
			maxDistNearest = (cylinder.getCylinderHeight() / 2)
					/ Math.cos(angleNearest);
		} else {
			maxDistNearest = cylinder.getCylinderRadius()
					/ Math.cos((Math.PI / 2) - angleNearest);
		}
		
		if ((angleCenter <= angleCritical)
				|| (angleCenter >= (Math.PI - angleCritical))) {
			maxDistCenter = (cylinder.getCylinderHeight() / 2)
					/ Math.cos(angleCenter);
		} else {
			maxDistCenter = cylinder.getCylinderRadius()
					/ Math.cos((Math.PI / 2) - angleCenter);
		}
		
		// intersection is present if the cylinder contains the center of the
		// box or its the nearest corner, or if the box contains the center of
		// the cylinder
		return ((distanceCenter <= maxDistCenter)
				|| (distanceNearest <= maxDistNearest)
				|| this.contains(cylCenter));
	}
	
	/**
	 * Gets a frustum representation of this box which is slightly expanded
	 * to account for numerical inaccuracies.
	 * 
	 * @return a frustum representation of this box
	 */
	public Frustum getFrustum() {
		// TODO: other intersection methods can be removed
		Plane[] frustumPlanes = new Plane[6];
		
		// frustum planes point inwards, box planes point outwards
		// xMin <= xMax must be satisfied for any box axes x
		for (int index = 0; index < this.planes.length; index++) {
			Vec4 planeNormal = this.planes[index].getNormal();
			// invert box plane to create frustum plane
			// expand frustum by epsilon to detect touching extents
			// the expansion must be a fraction of the default epsilon
			// to allow for a contraction within the default epsilon
			frustumPlanes[index] = new Plane(
					-planeNormal.x,
					-planeNormal.y,
					-planeNormal.z,
					-planeNormal.w + (PrecisionDouble.EPSILON * 0.001d));
		}
		
	    return new Frustum(
	    		frustumPlanes[0],
	    		frustumPlanes[1],
	    		frustumPlanes[2],
	    		frustumPlanes[3],
	    		frustumPlanes[4],
	    		frustumPlanes[5]);
	}
	
	/**
	 * Determines whether or not an extent intersects this box.
	 * 
	 * @param extent the extent
	 * 
	 * @return true if the extent intersects this box, false otherwise
	 * 
	 * @see Frustum#intersects(Extent)
	 */
	public boolean intersects(Extent extent) {
		return extent.intersects(this.getFrustum());
	}
	
	/**
	 * Determines whether or not a line segment intersects this box.
	 * 
	 * @param pa one end of the line segment
	 * @param pb the other end of the line segment
	 *  
	 * @return true if the line segment intersects or is contained in this box,
	 *         false otherwise
	 * 
	 * @see Frustum#intersectsSegment(Vec4, Vec4)
	 */
	public boolean intersectsSegment(Vec4 pa, Vec4 pb) {
		// TODO: file bug report for worldwind 2.0 
		// gov.nasa.worldwind.geom.Frustum#intersectsSegment(Vec4)
		// detects false intersects since one successful plane clip is not sufficient
		//return this.getFrustum().intersectsSegment(pa, pb);
		return (null != Line.clipToFrustum(pa, pb, this.getFrustum()));
	}
	
	/*
	@Override
	public Intersection[] intersect(Line line) {
		Plane[] boxPlanes = new Plane[6];
		
		for (int index = 0; index < this.planes.length; index++) {
			Vec4 planeNormal = this.planes[index].getNormal();
			// expand box by epsilon for intersection		
			boxPlanes[index] = new Plane(
					planeNormal.x,
					planeNormal.y,
					planeNormal.z,
					planeNormal.w - (PrecisionDouble.EPSILON * 0.001d));
		}
		
		return WWMath.polytopeIntersect(line, boxPlanes);
	}
	*/
	
}
