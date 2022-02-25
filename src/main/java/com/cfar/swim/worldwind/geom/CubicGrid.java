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
import java.util.LinkedHashSet;
import java.util.Set;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.geom.precision.PrecisionDouble;
import com.cfar.swim.worldwind.geom.precision.PrecisionVec4;

import gov.nasa.worldwind.geom.Line;
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
	private double normalizer = Double.NaN;
	
	/**
	 * Constructs a new cubic grid from a geometric cube without any children. 
	 * 
	 * @param cube the geometric cube
	 * 
	 * @see com.cfar.swim.worldwind.geom.Cube#Cube(gov.nasa.worldwind.geom.Vec4, gov.nasa.worldwind.geom.Vec4[], double)
	 */
	public CubicGrid(Cube cube) {
		super(cube);
		this.normalizer = cube.getDiameter();
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
		this.normalizer = refChild.getDiameter();
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
	 * Gets the distance normalizer of this cubic grid.
	 * 
	 * @return the distance normalizer of this cubic grid
	 */
	public double getNormalizer() {
		return this.normalizer;
	}
	
	/**
	 * Sets the normalizer of this cubic grid.
	 * 
	 * @param normalizer the normalizer to be set
	 * 
	 * @throws IllegalArgumentException if the normalizer is less than 1
	 */
	public void setNormalizer(double normalizer) {
		if (1d > normalizer) {
			throw new IllegalArgumentException("invalid normalizer");
		}
		this.normalizer = normalizer;
		this.updateNormalizer();
	}
	
	/**
	 * Updates the normalizer of this cubic grid after structural modifications.
	 */
	protected void updateNormalizer() {
		for (CubicGrid child : this.getChildren()) {
			child.normalizer = this.normalizer;
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
		this.addChildren(cells, cells, cells);
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
	 * vector in world model coordinates considering numerical inaccuracies.
	 * Cells are looked up recursively and only non-parent cells are
	 * considered.
	 * 
	 * @param modelVector the vector in world model coordinates
	 * 
	 * @return the cubic non-parent grid cells containing the specified vector
	 * 
	 * @see RegularGrid#lookupCells(Vec4)
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends CubicGrid> lookupCells(Vec4 modelVector) {
		return (Set<CubicGrid>) super.lookupCells(modelVector);
	}
	
	/**
	 * Looks up the cubic grid cells (maximum eight) containing a specified
	 * vector in world model coordinates considering numerical inaccuracies.
	 * Cells are looked up recursively to a specified depth level. A zero depth
	 * does not consider any children. A negative depth performs a full
	 * recursive search and considers non-parent cells only. 
	 * 
	 * @param modelVector the vector in world model coordinates
	 * @param depth the hierarchical depth of the lookup operation
	 * 
	 * @return the cubic grid cells containing the specified vector
	 * 
	 * @see RegularGrid#lookupCells(Vec4, int)
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends CubicGrid> lookupCells(Vec4 modelVector, int depth) {
		return (Set<CubicGrid>) super.lookupCells(modelVector, depth);
	}
	
	/**
	 * Finds all cells of this cubic grid that satisfy a specified predicate.
	 * A full recursive search is performed considering only non-parent cells.
	 * 
	 * @param predicate the predicate
	 * 
	 * @return the cells of this cubic grid that satisfy a predicate
	 * 
	 * @see RegularGrid#findCells(Predicate)
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends CubicGrid> findCells(Predicate<RegularGrid> predicate) {
		return (Set<CubicGrid>) super.findCells(predicate);
	}
	
	/**
	 * Finds all cells of this cubic grid that satisfy a specified predicate
	 * taking a specified hierarchical depth into account. A zero depth does
	 * not consider any children. A negative depth performs a full recursive
	 * search and considers non-parent cells only.
	 * 
	 * @param predicate the predicate
	 * @param depth the hierarchical depth
	 * 
	 * @return the cells of this cubic grid that satisfy a predicate taking
	 *         the hierarchical depth into account
	 * 
	 * @see RegularGrid#findCells(Predicate, int)
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends CubicGrid> findCells(Predicate<RegularGrid> predicate, int depth) {
		return (Set<CubicGrid>) super.findCells(predicate, depth);
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
	
	/**
	 * Computes the directions for a cubic grid traversal using Bresenham's
	 * algorithm.
	 * 
	 * @param source the source vector in grid coordinates
	 * @param target the target vector in grid coordinates
	 * 
	 * @return the directions of steps on each axis, that is, -1, 0 or 1
	 */
	private int[] computeDirections(Vec4 source, Vec4 target) {
		int[] directions = new int[3];
		PrecisionVec4 s = new PrecisionVec4(source);
		PrecisionVec4 t = new PrecisionVec4(target);
		
		directions[0] = (s.x < t.x) ? 1 : ((s.x > t.x) ? -1 : 0);
		directions[1] = (s.y < t.y) ? 1 : ((s.y > t.y) ? -1 : 0);
		directions[2] = (s.z < t.z) ? 1 : ((s.z > t.z) ? -1 : 0);
		
		return directions;
	}
	
	/**
	 * Computes the cell indices for a vector in this cubic grid favoring
	 * specified directions to be applied in case of multiple matches.
	 * 
	 * @param vector the vector in grid coordinates
	 * @param directions the favoring directions
	 * 
	 * @return the grid cell coordinates for the vector in this cubic grid
	 *         favoring the directions
	 */
	private int[] computeIndices(Vec4 vector, int[] directions) {
		int[] indices = new int[3];
		
		int[] rci = RegularGrid.getCellIndices(vector.x, this.rLength, this.cells.length);
		int[] sci = RegularGrid.getCellIndices(vector.y, this.sLength, this.cells[0].length);
		int[] tci = RegularGrid.getCellIndices(vector.z, this.tLength, this.cells[0][0].length);
		
		// take directions into account to decide for the better index
		if ((rci[0] != -1) && (rci[1] != -1)) {
			indices[0] = (0 < directions[0]) ? Math.max(rci[0], rci[1]) : Math.min(rci[0], rci[1]);
		} else {
			indices[0] = (rci[0] != -1) ? rci[0] : rci[1];
		}
		
		if ((sci[0] != -1) && (sci[1] != -1)) {
			indices[1] = (0 < directions[1]) ? Math.max(sci[0], sci[1]) : Math.min(sci[0], sci[1]);
		} else {
			indices[1] = (sci[0] != -1) ? sci[0] : sci[1];
		}
		
		if ((tci[0] != -1) && (tci[1] != -1)) {
			indices[2] = (0 < directions[2]) ? Math.max(tci[0], tci[1]) : Math.min(tci[0], tci[1]);
		} else {
			indices[2] = (tci[0] != -1) ? tci[0] : tci[1];
		}
		
		return indices;
	}
	
	/**
	 * Computes the minimum and maximum reference boundaries of a vector for a
	 * traversal of this cubic grid using Bresenham's algorithm.
	 * 
	 * @param vector the vector in grid coordinates
	 * @param directions the traversal directions
	 * 
	 * @return the minimum and maximum reference boundaries of the vector
	 */
	private double[][] computeMinMax(Vec4 vector, int[] directions) {
		double[][] minmax = new double[3][2];
		double side = this.getChild(0, 0, 0).getLength();
		int[] indices = this.computeIndices(vector, directions);
		
		minmax[0][0] = side * indices[0];
		minmax[0][1] = minmax[0][0] + side;
		
		minmax[1][0] = side * indices[1];
		minmax[1][1] = minmax[1][0] + side;
		
		minmax[2][0] = side * indices[2];
		minmax[2][1] = minmax[2][0] + side;
		
		return minmax;
	}
	
	/**
	 * Computes the reference distances of two vectors for a traversal of this
	 * cubic grid using Bresenham's algorithm.
	 * 
	 * @param source the source vector in grid coordinates
	 * @param target the target vector in grid coordinates
	 * @param directions the traversal directions
	 * 
	 * @return the reference distances of the two vectors
	 */
	@SuppressWarnings("unused")
	private double[][] computeDistances(Vec4 source, Vec4 target, int[] directions) {
		double[][] distances = new double[3][2];
		double[][] srcMinMax = this.computeMinMax(source, directions);
		double side = this.getChild(0, 0, 0).getLength();
		PrecisionDouble dx = new PrecisionDouble(target.x - source.x);
		PrecisionDouble dy = new PrecisionDouble(target.y - source.y);
		PrecisionDouble dz = new PrecisionDouble(target.z - source.z);
		
		if (0 < dx.doubleValue()) {
			distances[0][0] = (source.x - srcMinMax[0][0]) / Math.abs(dx.getOriginal());
			distances[0][1] = side / Math.abs(dx.getOriginal());
		} else if (0 > dx.doubleValue()) {
			distances[0][0] = (srcMinMax[0][1] - source.x) / Math.abs(dx.getOriginal());
			distances[0][1] = side / Math.abs(dx.getOriginal());
		} else {
			distances[0][0] = Double.POSITIVE_INFINITY;
			distances[0][1] = Double.POSITIVE_INFINITY;
		}
		
		if (0 < dy.doubleValue()) {
			distances[1][0] = (source.y - srcMinMax[1][0]) / Math.abs(dy.getOriginal());
			distances[1][1] = side / Math.abs(dy.getOriginal());
		} else if (0 > dy.doubleValue()) {
			distances[1][0] = (srcMinMax[1][1] - source.y) / Math.abs(dy.getOriginal());
			distances[1][1] = side / Math.abs(dy.getOriginal());
		} else {
			distances[1][0] = Double.POSITIVE_INFINITY;
			distances[1][1] = Double.POSITIVE_INFINITY;
		}
		
		if (0 < dz.doubleValue()) {
			distances[2][0] = (source.z - srcMinMax[2][0]) / Math.abs(dz.getOriginal());
			distances[2][1] = side / Math.abs(dz.getOriginal());
		} else if (0 > dz.doubleValue()) {
			distances[2][0] = (srcMinMax[2][1] - source.z) / Math.abs(dz.getOriginal());
			distances[2][1] = side / Math.abs(dz.getOriginal());
		} else {
			distances[2][0] = Double.POSITIVE_INFINITY;
			distances[2][1] = Double.POSITIVE_INFINITY;
		}
		
		return distances;
	}
	
	/**
	 * Gets the intersection vectors of a line segment with the cells of this
	 * cubic grid. A full recursive search is performed considering only
	 * non-parent cells.
	 * 
	 * @param source the source vector of the line segment in model coordinates
	 * @param target the target vector of the line segment in model coordinates
	 * 
	 * @return the intersection vectors of the line segment with the cells of
	 *         this cubic grid
	 */
	public LinkedHashSet<Vec4> getIntersectionVectors(Vec4 source, Vec4 target) {
		return this.getIntersectionVectors(source, target, -1);
	}
	
	/**
	 * Gets the intersection vectors of a line segment with the cells of this
	 * cubic grid taking a specified hierarchical depth into account. A zero
	 * depth does not consider any children. A negative depth performs a full
	 * recursive search and considers non-parent cells only.
	 * 
	 * @param source the source vector of the line segment in model coordinates
	 * @param target the target vector of the line segment in model coordinates
	 * @param depth the hierarchical depth
	 * 
	 * @return the intersection vectors of the line segment with the cells of
	 *         this cubic grid taking the hierarchical depth into account
	 */
	public LinkedHashSet<Vec4> getIntersectionVectors(Vec4 source, Vec4 target, int depth) {
		LinkedHashSet<PrecisionVec4> intersectionVectors = new LinkedHashSet<PrecisionVec4>();
		
		// clip the end points of the segment to this cell
		Vec4[] endPoints = Line.clipToFrustum(source, target, this.getFrustum());
		
		// if this cell is intersected by the line segment
		if (null != endPoints) {
			intersectionVectors.add(new PrecisionVec4(endPoints[0]));
			
			// recurse for children if required
			if ((depth != 0) && this.hasChildren()) {
				Vec4 src = this.transformModelToBoxOrigin(endPoints[0]);
				Vec4 tgt = this.transformModelToBoxOrigin(endPoints[1]);
				
				// compute primary step directions
				int[] directions = this.computeDirections(src, tgt);
				
				// compute source and target child cell indices
				int[] srcIndices = this.computeIndices(src, directions);
				int[] tgtIndices = this.computeIndices(tgt, directions);
				
				// while not at the target index
				while (!Arrays.equals(srcIndices, tgtIndices)) {
					CubicGrid intersectedChild = this.getChild(
							srcIndices[0],
							srcIndices[1],
							srcIndices[2]);
					// collect intersection vectors recursively in order
					// TODO: observed NPE here: intersectedChild? Precision?
					intersectionVectors.addAll(
							intersectedChild.getIntersectionVectors(source, target, depth - 1)
								.stream()
								.map(PrecisionVec4::new)
								.collect(Collectors.toCollection(LinkedHashSet<PrecisionVec4>::new)));
					
					// perform one step towards target index
					endPoints = Line.clipToFrustum(source, target, intersectedChild.getFrustum());
					CubicGrid nr = this.getChild(
							srcIndices[0] + directions[0],
							srcIndices[1],
							srcIndices[2]);
					CubicGrid ns = this.getChild(
							srcIndices[0],
							srcIndices[1] + directions[1],
							srcIndices[2]);
					
					if ((null != nr) && (0 != directions[0]) && nr.contains(endPoints[1]) ) {
						srcIndices[0] += directions[0];
					} else if ((null != ns) && (0 != directions[1]) && ns.contains(endPoints[1])) {
						srcIndices[1] += directions[1];
					} else {
						srcIndices[2] += directions[2];
					}
				}
				// target child cell
				CubicGrid intersectedChild = this.getChild(
						srcIndices[0],
						srcIndices[1],
						srcIndices[2]);
				// collect intersection vectors recursively in order
				intersectionVectors.addAll(
						intersectedChild.getIntersectionVectors(source, target, depth - 1)
							.stream()
							.map(PrecisionVec4::new)
							.collect(Collectors.toCollection(LinkedHashSet<PrecisionVec4>::new)));
			}
			
			intersectionVectors.add(new PrecisionVec4(endPoints[1]));
		}
		
		return intersectionVectors
				.stream()
				.map(PrecisionVec4::getOriginal)
				.collect(Collectors.toCollection(LinkedHashSet<Vec4>::new));
	}
	
}
