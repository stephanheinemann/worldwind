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
	}
	
	// TODO: override addChildren methods that do not result in a cubic grid
	// TODO: implement hierarchical normalization (cube.getLength())
}
