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
		this.addChildren(rCells, sCells, tCells);
		this.updateNormalizer();
	}
	
	protected CubicGrid newInstance(
			Vec4[] axes,
			double rMin, double rMax,
			double sMin, double sMax,
			double tMin, double tMax) {
		Box b = new Box(axes, rMin, rMax, sMin, sMax, tMin, tMax);
		return new CubicGrid(new Cube(b.getOrigin(), axes, b.getRLength()));
	}
	
	protected void updateNormalizer() {
		if (this.hasChildren()) {
			if (this.getChild(0, 0, 0).normalizer < this.normalizer) {
				this.normalizer = this.getChild(0, 0, 0).normalizer;
				this.propagateUpNormalizer();
			} else {
				this.propagateDownNormalizer();
			}
		} else {
			this.normalizer = this.getLength();
			this.propagateUpNormalizer();
		}
	}
	
	protected void propagateUpNormalizer() {
		if (this.hasParent()) {
			CubicGrid parent = this.getParent();
			if (parent.normalizer > this.normalizer) {
				parent.normalizer = this.normalizer;
				parent.propagateUpNormalizer();
			} else {
				parent.propagateDownNormalizer();
			}
		} else {
			this.propagateDownNormalizer();
		}
	}
	
	protected void propagateDownNormalizer() {
		if (this.hasChildren()) {
			this.getAll().stream().map(c -> c.normalizer == this.normalizer);
		}
	}
	
	public double getLength() {
		return this.rLength;
	}
	
	public void addChildren(int cells) {
		super.addChildren(cells, cells, cells);
		this.updateNormalizer();
	}
	
	@Override
	public void addChildren(int rCells, int sCells, int tCells) {
		if ((rCells == sCells) && (sCells == tCells)) {
			super.addChildren(rCells, sCells, tCells);
			this.updateNormalizer();
		} else {
			throw new IllegalArgumentException();
		}
	}
	
	@Override
	public void addChildren(double rLength, double sLength, double tLength) {
		if ((rLength == sLength) && (sLength == tLength)) {
			super.addChildren(rLength, sLength, tLength);
			this.updateNormalizer();
		} else {
			throw new IllegalArgumentException();
		}
	}
	
	@Override
	public void removeChildren() {
		super.removeChildren();
		this.updateNormalizer();
	}
	
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends CubicGrid> getChildren() {
		return (Set<CubicGrid>) super.getChildren();
	}
	
	@Override
	public CubicGrid getChild(int r, int s, int t) {
		return (CubicGrid) super.getChild(r, s, t);
	}
	
	@Override
	public CubicGrid getParent() {
		return (CubicGrid) super.getParent();
	}
	
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends CubicGrid> getAll() {
		return (Set<CubicGrid>) super.getAll();
	}
	
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends CubicGrid> lookupCells(Vec4 modelPoint) {
		return (Set<CubicGrid>) super.lookupCells(modelPoint);
	}
	
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends CubicGrid> lookupCells(Vec4 modelPoint, int depth) {
		return (Set<CubicGrid>) super.lookupCells(modelPoint, depth);
	}
	
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends CubicGrid> getNeighbors() {
		return (Set<CubicGrid>) super.getNeighbors();
	}
	
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends CubicGrid> getNeighbors(int depth) {
		return (Set<CubicGrid>) super.getNeighbors(depth);
	}
	
}
