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

import java.util.Arrays;
import java.util.Objects;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.cfar.swim.worldwind.geom.precision.Precision;
import com.jogamp.opengl.GL2;

import gov.nasa.worldwind.geom.Extent;
import gov.nasa.worldwind.geom.Intersection;
import gov.nasa.worldwind.geom.Line;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.Renderable;
import gov.nasa.worldwind.util.OGLStackHandler;

/**
 * Realizes a line segment on a continuous line.
 * 
 * @author Stephan Heinemann
 *
 */
public class LineSegment /* extends Line */ implements Renderable {

	// NOTE: Line is a final class and cannot be extended
	
	/** the line of this line segment */
	private Line line;
	
	/** the first point on this line segment */
	private Vec4 first;
	
	/** the second point on this line segment */
	private Vec4 second;
	
	/** the drawing color of this line segment */
	private float[] color = {1.0f, 1.0f, 1.0f, 1.0f};
	
	/** the visibility of this line segment */
	private boolean visible = true;
	
	/**
	 * Constructs a new line segment from two end points.
	 * 
	 * @param first the first end point
	 * @param second the second end point
	 */
	public LineSegment(Vec4 first, Vec4 second) {
		this.first = first;
		this.second = second;
		this.line = Line.fromSegment(first, second);
	}
	
	/**
	 * Gets the first end point of this line segment.
	 * 
	 * @return the first end point of this line segment
	 */
	public Vec4 getFirst() {
		return this.first;
	}
	
	/**
	 * Sets the first end point of this line segment.
	 * 
	 * @param first the first end point to be set
	 */
	public void setFirst(Vec4 first) {
		this.first = first;
		this.line = Line.fromSegment(first, this.second);
	}
	
	/**
	 * Gets the second end point of this line segment.
	 * 
	 * @return the second end point of this line segment
	 */
	public Vec4 getSecond() {
		return this.second;
	}
	
	/**
	 * Sets the second end point of this line segment.
	 * 
	 * @param second the second end point of this line segment
	 */
	public void setSecond(Vec4 second) {
		this.second = second;
		this.line = Line.fromSegment(this.first, second);
	}
	
	/**
	 * Gets the other end point of this line segment.
	 * 
	 * @param endpoint one end point of this line segment
	 * 
	 * @return the other end point of this line segment, null otherwise
	 */
	public Vec4 getOther(Vec4 endpoint) {
		if (this.first.equals(endpoint)) {
			return this.second;
		} else if (this.second.equals(endpoint)) {
			return this.first;
		} else {
			return null;
		}
	}
	
	/**
	 * Determines whether or not a point is an end point of this line segment.
	 * 
	 * @param point the point to be checked
	 * 
	 * @return true if the point is an end point of this line segment,
	 *         false otherwise
	 */
	public boolean isEndpoint(Vec4 point) {
		return this.first.equals(point) || this.second.equals(point);
	} 
	
	/**
	 * Gets the inverse of this line segment with swapped end points.
	 * 
	 * @return the inverse of this line segment
	 */
	public LineSegment getInverse() {
		return new LineSegment(this.second, this.first);
	}
	
	/**
	 * Gets the line containing this line segment.
	 * 
	 * @return the line containing this line segment
	 */
	public Line getLine() {
		return this.line;
	}
	
	/**
	 * Gets the vector representing this line segment.
	 * 
	 * @return the vector representing this line segment
	 */
	public Vec4 getVector() {
		return this.second.subtract3(this.first);
	}
	
	/**
	 * Gets the length of this line segment.
	 * 
	 * @return the length of this line segment
	 */
	public double getLength() {
		return this.getVector().getLength3();
	}
	
	/**
	 * Determines whether or not this line segment contains a point within a
	 * default maximum tolerance distance.
	 * 
	 * @param point the point to be checked
	 * 
	 * @return true if this line segment contains the point, false otherwise
	 */
	public boolean contains(Vec4 point) {
		return this.contains(point, Precision.EPSILON);
	}
	
	/**
	 * Determines whether or not this line segment contains a point within a
	 * maximum tolerance distance.
	 * 
	 * @param point the point to be checked
	 * @param epsilon the maximum tolerance distance
	 *  
	 * @return true if this line segment contains the point, false otherwise
	 */
	public boolean contains(Vec4 point, double epsilon) {
		return (epsilon >= Line.distanceToSegment(this.first, this.second, point));
	}
	
	/**
	 * Gets the intersections of the line of this line segment with an extent
	 * in both directions.
	 * 
	 * @param extent the extent to intersect
	 * 
	 * @return the intersections of the line of this line segment with the
	 *         extent in both directions, null otherwise
	 */
	protected Intersection[] lineIntersect(Extent extent) {
		Intersection[] intersections = null;
		// NOTE: line intersections are only tested from the line origin
		// but *not* in both directions, explicitly include both directions
		Intersection[] i1 = extent.intersect(this.getLine());
		Intersection[] i2 = extent.intersect(this.getInverse().getLine());
		if ((null != i1) && (null != i2)) {
			intersections = Stream.concat(
					Arrays.stream(extent.intersect(this.getLine())),
					Arrays.stream(extent.intersect(this.getInverse().getLine())))
			.collect(Collectors.toSet())
			.toArray(Intersection[]::new);
		} else if ((null != i1) && (null == i2)) {
			intersections = i1;
		} else if ((null == i1) && (null != i2)) {
			intersections = i2;
		}
			
		return intersections;
	}
	
	/**
	 * Determines whether or not this line segment intersects an extent within
	 * a default maximum tolerance distance.
	 * 
	 * @param extent the extent to be checked
	 * 
	 * @return true if this line segment intersects the extent, false otherwise
	 */
	public boolean intersects(Extent extent) {
		return this.intersects(extent, Precision.EPSILON);
	}
	
	/**
	 * Determines whether or not this line segment intersects an extent within
	 * a maximum tolerance distance.
	 * 
	 * @param extent the extent to be checked
	 * @param epsilon the maximum tolerance distance
	 * 
	 * @return true if this line segment intersects the extent, false otherwise
	 */
	public boolean intersects(Extent extent, double epsilon) {
		boolean intersects = false;
		// NOTE: line intersections are only tested from the line origin
		// but *not* in both directions, explicitly include both directions
		Intersection[] intersections = this.lineIntersect(extent);
		
		if ((null != intersections) && (0 < intersections.length)) {
			// check if line segment pierces extent
			for (Intersection intersection : intersections) {
				intersects |= this.contains(
						intersection.getIntersectionPoint(), epsilon);
			}
			// check if line segment is contained within extent
			if (!intersects) {
				intersects = this.isContained(extent, epsilon);
			}
		}
		
		return intersects;
	}
	
	/**
	 * Determines whether or not this line segment is fully contained within an
	 * extent using a default maximum tolerance distance.
	 * 
	 * @param extent the extent to be checked
	 * 
	 * @return true if this line segment is fully contained within the extent,
	 *         false otherwise
	 */
	public boolean isContained(Extent extent) {
		return this.isContained(extent, Precision.EPSILON);
	}
	
	/**
	 * Determines whether or not this line segment is fully contained within an
	 * extent using a maximum tolerance distance.
	 * 
	 * @param extent the extent to be checked
	 * @param epsilon the maximum tolerance distance
	 * 
	 * @return true if this line segment is fully contained within the extent,
	 *         false otherwise 
	 */
	public boolean isContained(Extent extent, double epsilon) {
		boolean isContained = false;
		// NOTE: line intersections are only tested from the line origin
		// but *not* in both directions, explicitly include both directions
		Intersection[] intersections = this.lineIntersect(extent);
		
		if ((null != intersections) && (2 <= intersections.length)) {
			// line segment within the extent...
			LineSegment extentSegment = new LineSegment(
					intersections[0].getIntersectionPoint(),
					intersections[1].getIntersectionPoint());
			// ...contains this line segment
			isContained = extentSegment.contains(this.first, epsilon)
					&& extentSegment.contains(this.second, epsilon);
		}
		
		return isContained;
	}
	
	/**
	 * Sets the drawing color of this line segment.
	 * 
	 * @param red the red color component between 0.0 and 1.0
	 * @param green the green color component between 0.0 and 1.0
	 * @param blue the blue color component between 0.0 and 1.0
	 * @param alpha the alpha component between 0.0 and 1.0
	 */
	public void setColor(float red, float green, float blue, float alpha) {
		this.color[0] = red;
		this.color[1] = green;
		this.color[2] = blue;
		this.color[3] = alpha;
	}
	
	/**
	 * Sets the visibility of this line segment.
	 * 
	 * @param visible true if this edge is visible, false otherwise
	 */
	public void setVisible(boolean visible) {
		this.visible = visible;
	}
	
	/**
	 * Renders this line segment using a drawing context.
	 * 
	 * @param dc the drawing context
	 * 
	 * @see Renderable#render(DrawContext)
	 */
	@Override
	public void render(DrawContext dc) {
		if ((null != dc) && (!dc.isPickingMode()) && this.visible) {
	        GL2 gl = dc.getGL().getGL2();
	        OGLStackHandler ogsh = new OGLStackHandler();
	        ogsh.pushAttrib(gl, GL2.GL_COLOR_BUFFER_BIT
	            | GL2.GL_CURRENT_BIT
	            | GL2.GL_LINE_BIT);
	        try {
	            gl.glLineWidth(5);
	            gl.glLineStipple(1, (short) 0x00FF);
	            gl.glEnable(GL2.GL_LINE_STIPPLE);
	            gl.glColor4f(this.color[0], this.color[1], this.color[2], this.color[3]);
	            this.drawLineSegment(dc);
	            gl.glDisable(GL2.GL_LINE_STIPPLE);
	        } finally {
	            ogsh.pop(gl);
	        }
		}
	}
	
	/**
	 * Draws this line segment using a drawing context.
	 * 
	 * @param dc the drawing context
	 */
	protected void drawLineSegment(DrawContext dc) {
		GL2 gl = dc.getGL().getGL2();
		gl.glBegin(GL2.GL_LINES);
		gl.glVertex3d(this.first.x, this.first.y, this.first.z);
		gl.glVertex3d(this.second.x, this.second.y, this.second.z);
		gl.glEnd();
	}
	
	/**
	 * Determines whether or not this line segment equals another line segment
	 * based on their end positions.
	 * 
	 * @param o the other line segment
	 * 
	 * @return true if the end positions of this line segment equal the end
	 *         positions of the other line segment (regardless of order),
	 *         false otherwise
	 * 
	 * @see Object#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = false;

		if (this == o) {
			equals = true;
		} else if ((null != o) && (this.getClass() == o.getClass())) {
			LineSegment segment = (LineSegment) o;
			equals = this.isEndpoint(segment.getFirst())
					&& this.isEndpoint(segment.getSecond());
		}

		return equals;
	}
	
	/**
	 * Gets the hash code of this line segment based on its end positions.
	 * 
	 * @return the hash code of this line segment based on its end positions
	 * 
	 * @see Object#hashCode()
	 */
	@Override
	public int hashCode() {
		return Objects.hash(this.getFirst(), this.getSecond());
	}
	
	/**
	 * Gets the string representation of this line segment.
	 * 
	 * @return the string representation of this line segment
	 */
	@Override
	public String toString() {
		return "[" + this.getFirst() + ", " + this.getSecond() + "]";
	}
	
}
