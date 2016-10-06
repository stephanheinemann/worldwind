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

import java.util.Set;

import com.cfar.swim.worldwind.geom.precision.PrecisionDouble;

import gov.nasa.worldwind.geom.Vec4;

/**
 * Realizes a hierarchical cubic three-dimensional grid.
 * 
 * @author Stephan Heinemann
 * 
 */
public class CubicGrid extends RegularGrid {
	
	/**
	 * the normalizer for distances of this cubic grid
	 */
	protected double normalizer = Double.NaN;
	
	/**
	 * Constructs a new cubic grid from a geometric cube without any children. 
	 * 
	 * @param cube the geometric cube
	 * 
	 * @see com.cfar.swim.worldwind.geom.Cube#Cube(gov.nasa.worldwind.geom.Vec4, gov.nasa.worldwind.geom.Vec4[], double)
	 */
	public CubicGrid(Cube cube) {
		super(cube);
		this.normalizer = cube.getLength();
	}
	
	/**
	 * Constructs a new cubic grid from a geometric cube representing a
	 * reference child.
	 * 
	 * @param refChild the geometric cube representing the reference child
	 * @param rCells the number of cubic children along the <code>R</code> axis
	 * @param sCells the number of cubic children along the <code>S</code> axis
	 * @param tCells the number of cubic children along the <code>T</code> axis
	 */
	public CubicGrid(Cube refChild, int rCells, int sCells, int tCells) {
		super(
			refChild.getUnitAxes(),
			refChild.getUnitRAxis().dot3(refChild.origin),
			refChild.getUnitRAxis().dot3(refChild.origin) + (refChild.getLength() * rCells),
			refChild.getUnitSAxis().dot3(refChild.origin),
			refChild.getUnitSAxis().dot3(refChild.origin) + (refChild.getLength() * sCells),
			refChild.getUnitTAxis().dot3(refChild.origin),
			refChild.getUnitTAxis().dot3(refChild.origin) + (refChild.getLength() * tCells));
		this.normalizer = refChild.getLength();
		super.addChildren(rCells, sCells, tCells);
	}
	
	/**
	 * Constructs a new cubic grid from three plane normals and six distances
	 * for each of the six faces of a geometric box without any children.
	 * 
	 * This factory method is used during child construction and supposed to be
	 * overridden by specializing classes.
	 * 
	 * @see RegularGrid#newInstance(Vec4[], double, double, double, double, double, double)
	 */
	@Override
	protected CubicGrid newInstance(
			Vec4[] axes,
			double rMin, double rMax,
			double sMin, double sMax,
			double tMin, double tMax) {
		Box b = new Box(axes, rMin, rMax, sMin, sMax, tMin, tMax);
		return new CubicGrid(new Cube(b.getOrigin(), axes, b.getRLength()));
	}
	
	/**
	 * Gets the normalizer of this cubic grid.
	 * 
	 * @return the normalizer of this cubic grid
	 */
	public double getNormalizer() {
		return this.normalizer;
	}
	
	/**
	 * Updates the normalizer of this cubic grid after structural modifications.
	 */
	protected void updateNormalizer() {
		CubicGrid root = this;
		while (root.hasParent()) {
			root = root.getParent();
		}
		
		Set<? extends CubicGrid> grids = root.getAll();
		double normalizer = grids.stream().map(CubicGrid::getLength).min(Double::compare).get().doubleValue();
		
		for (CubicGrid grid : grids) {
			grid.normalizer = normalizer;
		}
	}
	
	/**
	 * Gets the length of this cubic grid.
	 * 
	 * @return the length of this cubic grid
	 * 
	 * @see Box#getRLength()
	 */
	public double getLength() {
		return this.rLength;
	}
	
	/**
	 * Adds the specified number of children on all axes of this cubic
	 * grid.
	 * 
	 * @param cells the number of children on all axes
	 * 
	 * @see RegularGrid#addChildren(int, int, int)
	 */
	public void addChildren(int cells) {
		super.addChildren(cells, cells, cells);
		this.updateNormalizer();
	}
	
	/**
	 * Adds the specified number of children on each axis to this cubic grid.
	 * 
	 * @param rCells the number of children on the <code>R</code> axis
	 * @param sCells the number of children on the <code>S</code> axis
	 * @param tCells the number of children on the <code>T</code> axis
	 * 
	 * @throws IllegalArgumentException if children cannot be cubes
	 * 
	 * @see RegularGrid#addChildren(int, int, int)
	 */
	@Override
	public void addChildren(int rCells, int sCells, int tCells) {
		PrecisionDouble rLength = new PrecisionDouble(this.rLength / rCells);
		PrecisionDouble sLength = new PrecisionDouble(this.sLength / sCells);
		PrecisionDouble tLength = new PrecisionDouble(this.tLength / tCells);
		
		if ((rLength.equals(sLength)) && (sLength.equals(tLength))) {
			super.addChildren(rCells, sCells, tCells);
			this.updateNormalizer();
		} else {
			throw new IllegalArgumentException("children must be cubes");
		}
	}
	
	/**
	 * Adds children with approximately the specified cell lengths to this
	 * cubic grid. The actual cell lengths will be adjusted to completely
	 * fill this cubic grid.
	 * 
	 * @param rLength the length of a child cell on the <code>R</code> axis
	 * @param sLength the length of a child cell on the <code>S</code> axis
	 * @param tLength the length of a child cell on the <code>T</code> axis
	 * 
	 * @throws IllegalArgumentException if children cannot be cubes, that is,
	 *         if the specified lengths are not equal
	 * 
	 * @see RegularGrid#addChildren(double, double, double)
	 */
	@Override
	public void addChildren(double rLength, double sLength, double tLength) {
		if ((rLength == sLength) && (sLength == tLength)) {
			super.addChildren(rLength, sLength, tLength);
			this.updateNormalizer();
		} else {
			throw new IllegalArgumentException("children must be cubes");
		}
	}
	
	/**
	 * Removes all children from this cubic grid.
	 * 
	 * @see RegularGrid#removeChildren()
	 */
	@Override
	public void removeChildren() {
		super.removeChildren();
		this.updateNormalizer();
	}
	
	/**
	 * Gets the children of this cubic grid.
	 * 
	 * @return the children of this cubic grid
	 * 
	 * @see RegularGrid#getChildren()
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends CubicGrid> getChildren() {
		return (Set<CubicGrid>) super.getChildren();
	}
	
	/**
	 * Gets a particular child of this cubic grid if present.
	 * 
	 * @param r the <code>R</code> index of the child cell
	 * @param s the <code>S</code> index of the child cell
	 * @param t the <code>T</code> index of the child cell
	 * 
	 * @return the particular child of this cubic grid if present,
	 *         null otherwise
	 * 
	 * @see RegularGrid#getChild(int, int, int)
	 */
	@Override
	public CubicGrid getChild(int r, int s, int t) {
		return (CubicGrid) super.getChild(r, s, t);
	}
	
	/**
	 * Gets the parent of this cubic grid if present.
	 * 
	 * @return the parent of this cubic grid if present,
	 *         null otherwise
	 * 
	 * @see RegularGrid#getParent()
	 */
	@Override
	public CubicGrid getParent() {
		return (CubicGrid) super.getParent();
	}
	
	/**
	 * Gets all cubic grids associated with this cubic grid.
	 * 
	 * @return all cubic grids associated with this cubic grid
	 * 
	 * @see RegularGrid#getAll()
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends CubicGrid> getAll() {
		return (Set<CubicGrid>) super.getAll();
	}
	
	/**
	 * Looks up the cubic grid cells (maximum eight) containing a specified
	 * point in world model coordinates considering numerical inaccuracies.
	 * Cells are looked up recursively and only non-parent cells are
	 * considered.
	 * 
	 * @param modelPoint the point in world model coordinates
	 * 
	 * @return the cubic non-parent grid cells containing the specified point
	 * 
	 * @see RegularGrid#lookupCells(Vec4)
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends CubicGrid> lookupCells(Vec4 modelPoint) {
		return (Set<CubicGrid>) super.lookupCells(modelPoint);
	}
	
	/**
	 * Looks up the cubic grid cells (maximum eight) containing a specified
	 * point in world model coordinates considering numerical inaccuracies.
	 * Cells are looked up recursively to a specified depth level. A zero depth
	 * does not consider any children. A negative depth performs a full
	 * recursive search and considers non-parent cells only. 
	 * 
	 * @param modelPoint the point in world model coordinates
	 * @param depth the hierarchical depth of the lookup operation
	 * 
	 * @return the cubic grid cells containing the specified point
	 * 
	 * @see RegularGrid#lookupCells(Vec4, int)
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends CubicGrid> lookupCells(Vec4 modelPoint, int depth) {
		return (Set<CubicGrid>) super.lookupCells(modelPoint, depth);
	}
	
	/**
	 * Gets the neighbors of this cubic grid. A full recursive
	 * search is performed considering only non-parent neighbors.
	 * 
	 * @return the non-parent neighbors of this cubic grid
	 * 
	 * @see RegularGrid#getNeighbors()
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends CubicGrid> getNeighbors() {
		return (Set<CubicGrid>) super.getNeighbors();
	}
	
	/**
	 * Gets the neighbors of this cubic grid taking a specified hierarchical
	 * depth into account. A zero depth does not consider any neighboring
	 * children. A negative depth performs a full recursive search and
	 * considers non-parent neighbors only.
	 * 
	 * @param depth the hierarchical depth for finding neighbors
	 * 
	 * @return the neighbors of this cubic grid
	 * 
	 * @see RegularGrid#getNeighbors(int)
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends CubicGrid> getNeighbors(int depth) {
		return (Set<CubicGrid>) super.getNeighbors(depth);
	}
	
}
