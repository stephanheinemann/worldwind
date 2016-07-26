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

import java.util.ArrayList;
import java.util.List;

import javax.media.opengl.GL;
import javax.media.opengl.GL2;

import gov.nasa.worldwind.geom.Box;
import gov.nasa.worldwind.geom.Matrix;
import gov.nasa.worldwind.geom.Plane;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.util.OGLStackHandler;

/**
 * Realizes a regular three-dimensional grid that can be used motion planning.
 * 
 * @author Stephan Heinemann
 */
public class RegularGrid extends Box {

	/**
	 * the epsilon used compensate for numerical inaccuracies
	 */
	private static final double EPSILON = 1E-8;
	
	/**
	 * the children of this regular grid
	 */
	private RegularGrid[][][] cells = null;
	
	// TODO: data, occurrences and drawing based on data and severity
	private double cost = Math.random();
	
	/**
	 * Constructs a new regular grid from a geometric box without any children.
	 * 
	 * @param box the geometric box
	 * 
	 * @see gov.nasa.worldwind.geom.Box
	 */
	public RegularGrid(Box box) {
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
	 * Constructs a new regular grid from a geometric box with the specified
	 * number of children on each axis.
	 * 
	 * @param box the geometric box
	 * @param rCells the number of children on the <code>R</code> axis
	 * @param sCells the number of children on the <code>S</code> axis
	 * @param tCells the number of children on the <code>T</code> axis
	 * 
	 * @see gov.nasa.worldwind.geom.Box
	 */
	public RegularGrid(Box box, int rCells, int sCells, int tCells) {
		this(box);
		addChildren(rCells, sCells, tCells);
	}
	
	/**
	 * Constructs a new regular grid from three plane normals and six distances
	 * for each of the six faces of a geometric box without any children.
	 * 
	 * @param axes the three plane normals
	 * @param rMin the minimum distance on the <code>R</code> axis
	 * @param rMax the maximum distance on the <code>R</code> axis
	 * @param sMin the minimum distance on the <code>S</code> axis
	 * @param sMax the maximum distance on the <code>S</code> axis
	 * @param tMin the minimum distance on the <code>T</code> axis
	 * @param tMax the maximum distance on the <code>T</code> axis
	 * 
	 * @see gov.nasa.worldwind.geom.Box
	 */
	public RegularGrid(
			Vec4[] axes,
			double rMin, double rMax,
			double sMin, double sMax,
			double tMin, double tMax) {
		super(axes, rMin, rMax, sMin, sMax, tMin, tMax);
	}
	
	/**
	 * Constructs a new regular grid from three plane normals and six distances
	 * for each of the six faces of a geometric box with the specified number
	 * of children on each axis.
	 * 
	 * @param axes the three plane normals
	 * @param rMin the minimum distance on the <code>R</code> axis
	 * @param rMax the maximum distance on the <code>R</code> axis
	 * @param sMin the minimum distance on the <code>S</code> axis
	 * @param sMax the maximum distance on the <code>S</code> axis
	 * @param tMin the minimum distance on the <code>T</code> axis
	 * @param tMax the maximum distance on the <code>T</code> axis
	 * @param rCells the number of children on the <code>R</code> axis
	 * @param sCells the number of children on the <code>S</code> axis
	 * @param tCells the number of children on the <code>T</code> axis
	 * 
	 * @see gov.nasa.worldwind.geom.Box
	 */
	public RegularGrid(Vec4[] axes,
			double rMin, double rMax,
			double sMin, double sMax,
			double tMin, double tMax,
			int rCells, int sCells, int tCells) {
		this(axes, rMin, rMax, sMin, sMax, tMin, tMax);
		this.addChildren(rCells, sCells, tCells);
	}
	
	/**
	 * Adds the specified number of children on each axis to this regular grid.
	 * 
	 * @param rCells the number of children on the <code>R</code> axis
	 * @param sCells the number of children on the <code>S</code> axis
	 * @param tCells the number of children on the <code>T</code> axis
	 */
	public void addChildren(int rCells, int sCells, int tCells) {
		this.cells = new RegularGrid[rCells][sCells][tCells];
		
		// use the parent cell unit axes for all child cells
		Vec4[] axes = {this.getUnitRAxis(), this.getUnitSAxis(), this.getUnitTAxis()};
		
		// shorten the parent cell axes for the child cells
		Vec4 cellRAxis = this.getRAxis().divide3(rCells);
		Vec4 cellSAxis = this.getSAxis().divide3(sCells);
		Vec4 cellTAxis = this.getTAxis().divide3(tCells);
		
		// use the left-most parent cell face planes to compute the child cells 
		Plane rPlane = this.planes[0];
		Plane sPlane = this.planes[2];
		Plane tPlane = this.planes[4];
		
		for (int r = 0; r < rCells; r++) {
			for (int s = 0; s < sCells; s++) {
				for (int t = 0; t < tCells; t++) {
					// translate the left-most planes of each axis to form the new
					// left- and right-most planes of each axis for each child cell
					this.cells[r][s][t] = new RegularGrid(
							axes,
							rPlane.getDistance() - rPlane.getNormal().dot3(cellRAxis.multiply3(r)),
							rPlane.getDistance() - rPlane.getNormal().dot3(cellRAxis.multiply3(r + 1)),
							sPlane.getDistance() - sPlane.getNormal().dot3(cellSAxis.multiply3(s)),
							sPlane.getDistance() - sPlane.getNormal().dot3(cellSAxis.multiply3(s + 1)),
							tPlane.getDistance() - tPlane.getNormal().dot3(cellTAxis.multiply3(t)),
							tPlane.getDistance() - tPlane.getNormal().dot3(cellTAxis.multiply3(t + 1)));
				}
			}
		}
	}
	
	/**
	 * Adds children with approximately the specified cell lengths to this
	 * regular grid. The actual cell lengths will be adjusted to completely
	 * fill this regular grid.
	 * 
	 * @param rLength the length of a child cell on the <code>R</code> axis
	 * @param sLength the length of a child cell on the <code>S</code> axis
	 * @param tLength the length of a child cell on the <code>T</code> axis
	 */
	public void addChildren(double rLength, double sLength, double tLength) {
		if ((this.rLength >= rLength) && (this.sLength >= sLength) && (this.tLength >= tLength)) {
			int rCells = (int) Math.round(this.rLength / rLength);
			int sCells = (int) Math.round(this.sLength / sLength);
			int tCells = (int) Math.round(this.tLength / tLength);
			
			this.addChildren(rCells, sCells, tCells);
		}
	}
	
	/**
	 * Adds cubic-like children with the specified cell length to this regular
	 * grid. The actual cell lengths will be adjusted to completely
	 * fill this regular grid.
	 * 
	 * @param length the length of a child cell on each axis
	 */
	public void addChildren(double length) {
		this.addChildren(length, length, length);
	}
	
	/**
	 * Adds the specified number of children on each axis to a child cell of
	 * this regular grid if present.
	 * 
	 * @param r the <code>R</code> index of the child cell
	 * @param s the <code>S</code> index of the child cell
	 * @param t the <code>T</code> index of the child cell
	 * @param rCells the number of children on the <code>R</code> axis
	 * @param sCells the number of children on the <code>S</code> axis
	 * @param tCells the number of children on the <code>T</code> axis
	 */
	public void addChildren(int r, int s, int t, int rCells, int sCells, int tCells) {
		RegularGrid child = this.getChild(r, s, t);
		if (null != child) {
			child.addChildren(rCells, sCells, tCells);
		}
	}
	
	/**
	 * Adds children with approximately the specified cell lengths to a child
	 * cell of this regular grid if present. The actual cell lengths will be
	 * adjusted to completely fill the child cell.
	 * 
	 * @param r the <code>R</code> index of the child cell
	 * @param s the <code>S</code> index of the child cell
	 * @param t the <code>T</code> index of the child cell
	 * @param rLength the length of a child cell on the <code>R</code> axis
	 * @param sLength the length of a child cell on the <code>S</code> axis
	 * @param tLength the length of a child cell on the <code>T</code> axis
	 */
	public void addChildren(int r, int s, int t, double rLength, double sLength, double tLength) {
		RegularGrid child = this.getChild(r, s, t);
		if (null != child) {
			child.addChildren(rLength, sLength, tLength);
		}
	}
	
	/**
	 * Adds cubic-like children with the specified cell length to a child cell
	 * of this regular grid. The actual cell lengths will be adjusted to
	 * completely fill the child cell.
	 * 
	 * @param r the <code>R</code> index of the child cell
	 * @param s the <code>S</code> index of the child cell
	 * @param t the <code>T</code> index of the child cell
	 * @param length the length of a child cell on each axis
	 */
	public void addChildren(int r, int s, int t, double length) {
		RegularGrid child = this.getChild(r, s, t);
		if (null != child) {
			child.addChildren(length);
		}
	}
	
	/**
	 * Removes all children from this regular grid.
	 */
	public void removeChildren() {
		this.cells = null;
	}
	
	/**
	 * Indicates whether or not this regular grid has children.
	 * 
	 * @return true if this regular grid has children, false otherwise
	 */
	public boolean hasChildren() {
		return (null != this.cells);
	}
	
	/**
	 * Indicates whether or not this regular grid has a particular child.
	 * 
	 * @param r the <code>R</code> index of the child cell
	 * @param s the <code>S</code> index of the child cell
	 * @param t the <code>T</code> index of the child cell
	 * 
	 * @return true if this regular grid has the particular child, false otherwise
	 */
	public boolean hasChild(int r, int s, int t) {
		return
			this.hasChildren() &&
			(r >= 0) && (r < this.cells.length) &&
			(s >= 0) && (s < this.cells[r].length) &&
			(t >= 0) && (t < this.cells[r][s].length);
	}
	
	/**
	 * Gets a particular child of this regular grid if present.
	 * 
	 * @param r the <code>R</code> index of the child cell
	 * @param s the <code>S</code> index of the child cell
	 * @param t the <code>T</code> index of the child cell
	 * 
	 * @return the particular child of this regular grid if present,
	 *         null otherwise
	 */
	public RegularGrid getChild(int r, int s, int t) {
		RegularGrid child = null;
		
		if (this.hasChild(r, s, t)) {
			child = this.cells[r][s][t];
		}
		
		return child;
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
	private static boolean equalsEpsilon(double a, double b) {
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
	private static boolean isInRangeEpsilon(double d, double l, double u) {
		return ((l - EPSILON) <= d) && ((u + EPSILON) >= d);
	}
	
	/**
	 * Transforms a Cartesian world model vector into a regular grid vector
	 * using the first corner of this regular grid as origin.
	 * 
	 * @param modelPoint the world model vector
	 * 
	 * @return the regular grid vector
	 */
	public Vec4 transformModelToGrid(Vec4 modelPoint) {
		Vec4[] unitAxes = {ru, su, tu};
		Vec4 origin = this.getCorners()[0];
		Matrix transformMatrix = Matrix.fromLocalOrientation(origin , unitAxes).getInverse();
		return modelPoint.transformBy4(transformMatrix);
	}
	
	/**
	 * Indicates whether or not a value lies on the <code>R</code> axis of this
	 * regular grid considering numerical inaccuracies.
	 * 
	 * @param r the value
	 * 
	 * @return true if the value lies on the <code>R</code> axis considering
	 *         numerical inaccuracies, false otherwise
	 */
	private boolean containsREpsilon(double r) {
		return RegularGrid.isInRangeEpsilon(r, 0.0, this.rLength);
	}
	
	/**
	 * Indicates whether or not a value lies on the <code>S</code> axis of this
	 * regular grid considering numerical inaccuracies.
	 * 
	 * @param s the value
	 * 
	 * @return true if the value lies on the <code>S</code> axis considering
	 *         numerical inaccuracies, false otherwise
	 */
	private boolean containsSEpsilon(double s) {
		return RegularGrid.isInRangeEpsilon(s, 0.0, this.sLength);
	}
	
	/**
	 * Indicates whether or not a value lies on the <code>T</code> axis of this
	 * regular grid considering numerical inaccuracies.
	 * 
	 * @param t the value
	 * 
	 * @return true if the value lies on the <code>T</code> axis considering
	 *         numerical inaccuracies, false otherwise
	 */
	private boolean containsTEpsilon(double t) {
		return RegularGrid.isInRangeEpsilon(t, 0.0, this.tLength);
	}
	
	/**
	 * Indicates whether or not a vector in regular grid coordinates is contained
	 * in this regular grid considering numerical inaccuracies.
	 * 
	 * @param v the vector in regular grid coordinates
	 * 
	 * @return true if this regular grid contains the vector, false otherwise
	 */
	private boolean containsV(Vec4 v) {
		return this.containsREpsilon(v.x) && this.containsSEpsilon(v.y) && this.containsTEpsilon(v.z);
	}
	
	/**
	 * Indicates whether or not a point in world model coordinates is
	 * contained in this regular grid.
	 * 
	 * @param modelPoint the point in world model coordinates
	 * 
	 * @return true if this regular grid contains the point, false otherwise
	 */
	public boolean contains(Vec4 modelPoint) {
		boolean contains = false;
		
		Vec4 localPoint = this.transformModelToGrid(modelPoint);
		if (this.containsV(localPoint)) {
			contains = true;
		}
		
		return contains;
	}
	
	/**
	 * Gets the cell indices (maximum two) that are associated with child cells
	 * along the specified axis parameters considering numerical inaccuracies.
	 * 
	 * @param value the point on the cell axis
	 * @param length the length of the cell axis
	 * @param cells the number of child cells along the cell axis
	 * 
	 * @return the cell indices that are associated with child cells
	 */
	private static int[] getCellIndices(double value, double length, int cells) {
		int index[] = {-1, -1};
		
		if (RegularGrid.isInRangeEpsilon(value, 0.0, length)) {
			double cellLength = length / cells;
			double cellSegment = value / cellLength;
			double cellIndex = Math.ceil(cellSegment);
			
			if (!RegularGrid.equalsEpsilon(cellSegment, cellIndex)) {
				cellIndex =  Math.floor(cellSegment);
			}
			
			if (RegularGrid.equalsEpsilon(cellSegment, cellIndex)) {
				// two neighboring cells are affected
				index[0] = (int) cellIndex - 1;
				if (cellIndex < cells) {
					index[1] = (int) cellIndex;
				}
			} else {
				// only one cell is affected
				index[0] = (int) cellIndex;
			}
		}
		
		return index;
	}
	
	/**
	 * Looks up the cells (maximum eight) containing a specified point in
	 * world model coordinates considering numerical inaccuracies.
	 * 
	 * @param modelPoint the point in world model coordinates
	 * 
	 * @return the cells containing the specified point
	 */
	public List<RegularGrid> lookupCells(Vec4 modelPoint) {
		List<RegularGrid> lookedUpCells = new ArrayList<RegularGrid>(8);
		
		// transform point to cell (body) coordinates
		Vec4 cellPoint = this.transformModelToGrid(modelPoint);
		
		if (this.containsV(cellPoint)) {
			if (this.hasChildren()) {
				// lookup containing cells using the cell (body) coordinates
				int[] rCellIndices = RegularGrid.getCellIndices(cellPoint.x, this.rLength, this.cells.length);
				int[] sCellIndices = RegularGrid.getCellIndices(cellPoint.y, this.sLength, this.cells[0].length);
				int[] tCellIndices = RegularGrid.getCellIndices(cellPoint.z, this.tLength, this.cells[0][0].length);
				// each axis has at most two valid indices - a maximum of eight cells
				for (int r : rCellIndices) {
					if (r != -1) {
						for (int s : sCellIndices) {
							if (s != -1) {
								for (int t : tCellIndices) {
									if (t != -1) {
										lookedUpCells.addAll(this.cells[r][s][t].lookupCells(modelPoint));
									}
								}
							}
						}
					}
				}
			} else {
				lookedUpCells.add(this);
			}
		}
		
		return lookedUpCells;
	}
	
	/**
	 * Renders this regular grid. If a grid cell has children, then only the
	 * children are rendered.
	 */
	@Override
	public void render(DrawContext dc) {
		if (this.hasChildren()) {
			for (int r = 0; r < this.cells.length; r++) {
				for(int s = 0; s < this.cells[r].length; s++) {
					for (int t = 0; t < this.cells[r][s].length; t++) {
						this.cells[r][s][t].render(dc);
					}
				}
			}
		} else {
			// TODO: drawing should depend on cell transition cost threshold
			if (cost > .75) {
				super.render(dc);
			}
		}
	}
	
	/**
	 * Draws this regular grid.
	 * 
	 * @see gov.nasa.worldwind.geom.Box
	 */
	@Override
	protected void drawBox(DrawContext dc, Vec4 a, Vec4 b, Vec4 c, Vec4 d)
    {
        Vec4 e = a.add3(r);
        Vec4 f = d.add3(r);
        GL2 gl = dc.getGL().getGL2(); // GL initialization checks for GL2 compatibility.

        dc.getView().pushReferenceCenter(dc, bottomCenter);
        OGLStackHandler ogsh = new OGLStackHandler();
        ogsh.pushModelview(gl);
        try
        {
        	// TODO: color should reflect cell transition cost
        	//gl.glColor3f(1f, 0f, 0f);
        	gl.glColor3d(cost, 1.0 - cost, 0d);
        	gl.glEnable(GL.GL_CULL_FACE);
            //gl.glFrontFace(GL.GL_CCW);
        	this.drawOutline(dc, a, b, c, d);
        	gl.glTranslated(r.x, r.y, r.z);
        	this.drawOutline(dc, a, b, c, d);
        	gl.glPopMatrix();
            gl.glPushMatrix();
            this.drawOutline(dc, a, e, f, d);
        	gl.glTranslated(s.x, s.y, s.z);
        	this.drawOutline(dc, a, e, f, d);
        }
        finally
        {
            ogsh.pop(gl);
            dc.getView().popReferenceCenter(dc);
        }
    }
	
	protected void drawQuad(DrawContext dc, Vec4 a, Vec4 b, Vec4 c, Vec4 d) {
		GL2 gl = dc.getGL().getGL2();
		gl.glBegin(GL2.GL_QUADS);
		gl.glVertex3d(a.x, a.y, a.z);
        gl.glVertex3d(b.x, b.y, b.z);
        gl.glVertex3d(c.x, c.y, c.z);
        gl.glVertex3d(d.x, d.y, d.z);
		gl.glEnd();
	}
	
}
