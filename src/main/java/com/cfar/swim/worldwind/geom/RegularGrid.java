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

import java.util.HashSet;
import java.util.Set;
import java.util.stream.Collectors;

import javax.media.opengl.GL2;

import com.cfar.swim.worldwind.geom.precision.PrecisionDouble;
import com.cfar.swim.worldwind.geom.precision.PrecisionVec4;

import gov.nasa.worldwind.geom.Plane;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.util.OGLStackHandler;

/**
 * Realizes a regular three-dimensional grid that can be used for motion
 * planning.
 * 
 * @author Stephan Heinemann
 * 
 */
public class RegularGrid extends Box {
	
	/**
	 * the parent of this regular grid
	 */
	protected RegularGrid parent = null;
	
	/**
	 * the children of this regular grid
	 */
	protected RegularGrid[][][] cells = null;

	/**
	 * the drawing color of this regular grid
	 */
	private float[] color = {1.0f, 1.0f, 1.0f, 1.0f};
	
	/**
	 * the visibility state of this regular grid
	 */
	protected boolean visible = true;
	
	/**
	 * Constructs a new regular grid from a geometric box without any children.
	 * 
	 * @param box the geometric box
	 * 
	 * @see com.cfar.swim.worldwind.geom.Box#Box(gov.nasa.worldwind.geom.Box)
	 */
	public RegularGrid(Box box) {
		super(box);
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
	 * @see com.cfar.swim.worldwind.geom.Box#Box(Vec4[], double, double, double, double, double, double)
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
	 * Constructs a new regular grid from three plane normals and six distances
	 * for each of the six faces of a geometric box without any children.
	 * 
	 * This factory method is used during child construction and supposed to be
	 * overridden by specializing classes.
	 * 
	 * @see RegularGrid#RegularGrid(Vec4[], double, double, double, double, double, double)
	 */
	protected RegularGrid newInstance(
			Vec4[] axes,
			double rMin, double rMax,
			double sMin, double sMax,
			double tMin, double tMax) {
		return new RegularGrid(axes, rMin, rMax, sMin, sMax, tMin, tMax);
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
					// TODO: R >= S >= T (Note: No check is made to ensure the order of the face locations.)
					this.cells[r][s][t] = this.newInstance(
							axes,
							rPlane.getDistance() - rPlane.getNormal().dot3(cellRAxis.multiply3(r)),
							rPlane.getDistance() - rPlane.getNormal().dot3(cellRAxis.multiply3(r + 1)),
							sPlane.getDistance() - sPlane.getNormal().dot3(cellSAxis.multiply3(s)),
							sPlane.getDistance() - sPlane.getNormal().dot3(cellSAxis.multiply3(s + 1)),
							tPlane.getDistance() - tPlane.getNormal().dot3(cellTAxis.multiply3(t)),
							tPlane.getDistance() - tPlane.getNormal().dot3(cellTAxis.multiply3(t + 1)));
					this.cells[r][s][t].parent = this;
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
	 * Indicates whether or not this regular grid has a parent.
	 * 
	 * @return true if this regular grid has a parent, false otherwise
	 */
	public boolean hasParent() {
		return (null != this.parent);
	}
	
	/**
	 * Gets the parent of this regular grid if present.
	 * 
	 * @return the parent of this regular grid if present,
	 *         null otherwise
	 */
	public RegularGrid getParent() {
		return this.parent;
	}
	
	/**
	 * Gets all regular grids associated with this regular grid.
	 * 
	 * @return all regular grids associated with this regular grid
	 */
	public Set<? extends RegularGrid> getAll() {
		Set<RegularGrid> all = new HashSet<RegularGrid>();
		all.add(this);
		
		if (this.hasChildren()) {
			for (int r = 0; r < this.cells.length; r++) {
				for (int s = 0; s < this.cells[r].length; s++) {
					for (int t = 0; t < this.cells[r][s].length; t++) {
						all.addAll(this.cells[r][s][t].getAll());
					}
				}
			}
		}
		
		return all;
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
		
		if (PrecisionDouble.isInRange(value, 0.0, length)) {
			double cellLength = length / cells;
			double cellSegment = value / cellLength;
			double cellIndex = Math.ceil(cellSegment);
			
			if (!PrecisionDouble.equals(cellSegment, cellIndex)) {
				cellIndex =  Math.floor(cellSegment);
			}
			
			if (PrecisionDouble.equals(cellSegment, cellIndex)) {
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
	 * Looks up the regular grid cells (maximum eight) containing a specified
	 * point in world model coordinates considering numerical inaccuracies.
	 * Cells are looked up recursively and only non-parent cells are
	 * considered.
	 * 
	 * @param modelPoint the point in world model coordinates
	 * @return the regular non-parent grid cells containing the specified point
	 */
	public Set<? extends RegularGrid> lookupCells(Vec4 modelPoint) {
		return this.lookupCells(modelPoint, -1);
	}
	
	/**
	 * Looks up the regular grid cells (maximum eight) containing a specified
	 * point in world model coordinates considering numerical inaccuracies.
	 * Cells are looked up recursively to a specified depth level. A zero depth
	 * does not consider any children. A negative depth performs a full
	 * recursive search and considers non-parent cells only. 
	 * 
	 * @param modelPoint the point in world model coordinates
	 * @param depth the hierarchical depth of the lookup operation
	 * 
	 * @return the regular grid cells containing the specified point
	 */
	public Set<? extends RegularGrid> lookupCells(Vec4 modelPoint, int depth) {
		Set<RegularGrid> lookedUpCells = new HashSet<RegularGrid>(8);
		
		// transform point to cell (body) coordinates
		Vec4 cellPoint = this.transformModelToBoxOrigin(modelPoint);
		
		if (this.containsV(cellPoint)) {
			if (this.hasChildren() && (0 != depth)) {
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
										// TODO: cells seem to be oriented arbitrarily although constructed consistently
										int ri = this.cells.length -1 - r;
										int si = this.cells[0].length -1 - s;
										int ti = this.cells[0][0].length - 1 - t;
										
										// TODO: checking containment for all orientation possibilities is not a nice solution
										lookedUpCells.addAll(this.cells[r][s][t].lookupCells(modelPoint, depth - 1));
										lookedUpCells.addAll(this.cells[r][s][ti].lookupCells(modelPoint, depth - 1));
										lookedUpCells.addAll(this.cells[r][si][t].lookupCells(modelPoint, depth - 1));
										lookedUpCells.addAll(this.cells[r][si][ti].lookupCells(modelPoint, depth - 1));
										lookedUpCells.addAll(this.cells[ri][s][t].lookupCells(modelPoint, depth - 1));
										lookedUpCells.addAll(this.cells[ri][s][ti].lookupCells(modelPoint, depth - 1));
										lookedUpCells.addAll(this.cells[ri][si][t].lookupCells(modelPoint, depth - 1));
										lookedUpCells.addAll(this.cells[ri][si][ti].lookupCells(modelPoint, depth - 1));
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
	 * Gets the children of this regular grid.
	 * 
	 * @return the children of this regular grid
	 */
	public Set<? extends RegularGrid> getChildren() {
		Set<RegularGrid> children = new HashSet<RegularGrid>();
		
		if (this.hasChildren()) {
			for (int r = 0; r < this.cells.length; r++) {
				for (int s = 0; s < this.cells[r].length; s++) {
					for (int t = 0; t < this.cells[r][s].length; t++) {
						children.add(this.cells[r][s][t]);
					}
				}
			}
		}
		
		return children;
	}
	
	/**
	 * Gets the neighbors of this regular grid. A full recursive
	 * search is performed considering only non-parent neighbors.
	 * 
	 * @return the non-parent neighbors of this regular grid
	 * 
	 * @see RegularGrid#getNeighbors(int)
	 */
	public Set<? extends RegularGrid> getNeighbors() {
		return this.getNeighbors(-1);
	}
	
	/**
	 * Gets the neighbors of this regular grid taking a specified hierarchical
	 * depth into account. A zero depth does not consider any neighboring
	 * children. A negative depth performs a full recursive search and
	 * considers non-parent neighbors only.
	 * 
	 * @param depth the hierarchical depth for finding neighbors
	 * 
	 * @return the neighbors of this regular grid
	 */
	public Set<? extends RegularGrid> getNeighbors(int depth) {
		Set<RegularGrid> neighbors = new HashSet<RegularGrid>(26);
		
		if (this.hasParent()) {
			Vec4[] corners = this.getCorners();
			if (-1 == depth) {
				depth--;
			}
			
			// TODO: possibly not correct - all neighboring wall cells are relevant
			for (Vec4 corner : corners) {
				neighbors.addAll(this.parent.lookupCells(corner, depth + 1));
			}
			// TODO: possibly too expensive
			neighbors.remove(this.getAll());
		}
		
		return neighbors;
	}
	
	/**
	 * Indicates whether or not this regular grid is a neighbor of another
	 * regular grid.
	 * 
	 * @param neighbor the potential neighbor
	 * 
	 * @return true if this regular grid is a neighbor of the other regular
	 *         grid, false otherwise
	 * 
	 * @see RegularGrid#getNeighbors()
	 */
	public boolean areNeighbors(RegularGrid neighbor) {
		return this.getNeighbors().contains(neighbor);
	}
	
	/**
	 * Indicates whether or not this regular grid is a neighbor of another
	 * regular grid taking a specified hierarchical depth into account.
	 * 
	 * @param neighbor the potential neighbor
	 * @param depth the hierarchical depth
	 * 
	 * @return true if this regular grid is a neighbor of the other regular
	 *         grid taking the hierarchical depth into account, false otherwise
	 * 
	 * @see RegularGrid#getNeighbors(int)
	 */
	public boolean areNeighbors(RegularGrid neighbor, int depth) {
		return this.getNeighbors(depth).contains(neighbor);
	}
	
	/**
	 * Gets the neighbors of a point in this regular grid. A full recursive
	 * search is performed considering non-parent cells only.
	 * 
	 * @param point the point in world model coordinates
	 * 
	 * @return the neighbors of the point in this regular grid
	 * 
	 * @see RegularGrid#getNeighbors(Vec4, int)
	 */
	public Set<Vec4> getNeighbors(Vec4 point) {
		return this.getNeighbors(point, -1);
	}
	
	/**
	 * Gets the neighbors of a point in this regular grid taking a specified
	 * hierarchical depth into account. A zero depth does not consider any
	 * children. A negative depth performs a full recursive search and
	 * considers non-parent cells only.
	 * 
	 * @param point the point in world model coordinates
	 * @param depth the hierarchical depth for finding neighbors
	 * 
	 * @return the neighbors of the point in this regular grid 
	 */
	public Set<Vec4> getNeighbors(Vec4 point, int depth) {
		Set<Vec4> neighbors = new HashSet<Vec4>(6);
		Set<? extends RegularGrid> cells = this.lookupCells(point, depth);
		
		for (RegularGrid cell : cells) {
			Vec4[] neighborCorners;
			
			if (cell.isCorner(point)) {
				// point is a corner of the cell
				neighborCorners = cell.getNeighborCorners(point);
			} else {
				// point within cell, on edge or plane (consider all corners as neighbors)
				// TODO: possibly only consider adjacent corners of edges and planes
				neighborCorners = cell.getCorners();
			}
			// reduce precision for proper set addition
			//neighbors.addAll(neighborCorners.stream().map(PrecisionVec4::new).collect(Collectors.toSet()));
			
			for (Vec4 neighborCorner : neighborCorners) {
				boolean contains = false;
				for (Vec4 neighbor : neighbors) {
					if ((new PrecisionVec4(neighborCorner)).equals(new PrecisionVec4(neighbor))) {
						contains = true;
						break;
					}
				}
				if (!contains) {
					neighbors.add(neighborCorner);
				}
			}
		}
		
		return neighbors;
	}
	
	/**
	 * Indicates whether or not two points are neighbors in this regular grid.
	 * 
	 * @param point the point
	 * @param neighbor the potential neighbor of the point
	 * 
	 * @return true if the two points are neighbors, false otherwise
	 * 
	 * @see RegularGrid#getNeighbors(Vec4)
	 */
	public boolean areNeighbors(Vec4 point, Vec4 neighbor) {
		return this.getNeighbors(point).stream().map(PrecisionVec4::new).collect(Collectors.toSet()).contains(new PrecisionVec4(neighbor));
	}
	
	/**
	 * Indicates whether or not two points are neighbors in this regular grid
	 * taking a specified hierarchical depth into account.
	 * 
	 * @param point the point
	 * @param neighbor the potential neighbor of the point
	 * @param depth the hierarchical depth
	 * 
	 * @return true if the two points are neighbors taking the hierarchical
	 *              depth into account, false otherwise
	 * 
	 * @see RegularGrid#getNeighbors(Vec4, int)
	 */
	public boolean areNeighbors(Vec4 point, Vec4 neighbor, int depth) {
		return this.getNeighbors(point, depth).stream().map(PrecisionVec4::new).collect(Collectors.toSet()).contains(new PrecisionVec4(neighbor));
	}
	
	/**
	 * Renders this regular grid. If a grid cell has children, then only the
	 * children are rendered.
	 * 
	 * @param dc the drawing context
	 */
	@Override
	public void render(DrawContext dc) {
		if (this.visible) {
			if (this.hasChildren()) {
				for (int r = 0; r < this.cells.length; r++) {
					for(int s = 0; s < this.cells[r].length; s++) {
						for (int t = 0; t < this.cells[r][s].length; t++) {
							this.cells[r][s][t].render(dc);
						}
					}
				}
			} else {
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
        	gl.glColor4f(this.color[0], this.color[1], this.color[2], this.color[3]);
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
	
	/**
	 * Sets the drawing color of this regular grid.
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
	 * Sets the visibility state of this regular grid.
	 * 
	 * @param visible true if this regular grid is visible, false otherwise
	 */
	public void setVisible(boolean visible) {
		this.visible = visible;
	}
	
	// TODO: remove if not needed
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
