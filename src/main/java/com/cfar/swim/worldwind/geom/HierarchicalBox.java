/**
 * Copyright (c) 2018, Manuel Rosa (UVic Center for Aerospace Research)
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

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import javax.media.opengl.GL2;

import com.cfar.swim.worldwind.geom.precision.PrecisionVec4;

import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.Material;
import gov.nasa.worldwind.render.Path;
import gov.nasa.worldwind.util.Logging;
import gov.nasa.worldwind.util.OGLStackHandler;

/**
 * @author Manuel Rosa
 *
 */
public class HierarchicalBox extends Box {

	/** the parent of this hierarchical box */
	protected HierarchicalBox parent = null;

	/** the children of this hierarchical box */
	protected HashSet<HierarchicalBox> cells = new HashSet<HierarchicalBox>();

	/** the drawing color of this hierarchical box */
	private float[] color = { 1.0f, 1.0f, 1.0f, 1.0f };

	/** the visibility state of this hierarchical box */
	protected boolean visible = true;

	// ------------------ CONSTRUCTORS ----------------
	/**
	 * Constructs a new hierarchical box based on a geometric box without any children.
	 * 
	 * @param box the geometric box
	 * 
	 * @see com.cfar.swim.worldwind.geom.Box#Box(gov.nasa.worldwind.geom.Box)
	 */
	public HierarchicalBox(Box box) {
		super(box);
	}

	public HierarchicalBox(Vec4[] axes, double rMin, double rMax, double sMin, double sMax, double tMin, double tMax) {
		super(axes, rMin, rMax, sMin, sMax, tMin, tMax);
	}

	public HierarchicalBox(Vec4 origin, Vec4[] axes, double rLength, double sLength, double tLength) {
		super(origin, axes, rLength, sLength, tLength);
	}

	// ------------------ NEW METHODS ----------------
	public HierarchicalBox createInstance(Vec4 point1, Vec4 point2) {
		Vec4[] axes = this.getUnitAxes();
		List<Vec4> points = new ArrayList<Vec4>();

		points.add(point1);
		points.add(point2);
		Box box = HierarchicalBox.computeBoundingBox(points, axes);

		HierarchicalBox hierarchicalBox = new HierarchicalBox(box);

		return hierarchicalBox;
	}

	public void addChild(Vec4 point1, Vec4 point2) {
		HierarchicalBox child = this.createInstance(point1, point2);
		cells.add(child);
	}

	public Set<? extends HierarchicalBox> getChildren() {
		return this.cells;
	}

	/**
	 * Removes all children from this hierarchical box.
	 */
	public void removeChildren() {
		this.cells.clear();
	}

	/**
	 * Indicates whether or not this hierarchical box has children.
	 * 
	 * @return true if this hierarchical box has children, false otherwise
	 */
	public boolean hasChildren() {
		return !this.cells.isEmpty();
	}

	public void removeChild(HierarchicalBox child) {
		cells.remove(child);
	}

	public HierarchicalBox getChild() {
		// TODO
		return null;
	}

	public boolean hasChild(HierarchicalBox child) {
		// TODO
		return false;
	}

	/**
	 * Indicates whether or not this hierarchical box has a parent.
	 * 
	 * @return true if this hierarchical box has a parent, false otherwise
	 */
	public boolean hasParent() {
		return (null != this.parent);
	}

	/**
	 * Gets the parent of this hierarchical box if present.
	 * 
	 * @return the parent of this hierarchical box if present, null otherwise
	 */
	public HierarchicalBox getParent() {
		return this.parent;
	}

	/**
	 * Gets all hierarchical boxes associated with this hierarchical box.
	 * 
	 * @return all hierarchical boxes associated with this hierarchical box
	 */
	public Set<? extends HierarchicalBox> getAll() {
		Set<HierarchicalBox> all = new LinkedHashSet<HierarchicalBox>();
		all.add(this);

		if (this.hasChildren()) {
			for (HierarchicalBox cell : cells) {
				all.addAll(cell.getAll());
			}
		}

		return all;
	}

	/**
	 * Looks up the hierarchical box cells (maximum eight) containing a specified point
	 * in world model coordinates considering numerical inaccuracies. Cells are
	 * looked up recursively and only non-parent cells are considered.
	 * 
	 * @param modelPoint the point in world model coordinates
	 * 
	 * @return the regular non-parent grid cells containing the specified point
	 */
	public Set<? extends HierarchicalBox> lookupCells(Vec4 modelPoint) {
		return this.lookupCells(modelPoint, -1);
	}

	/**
	 * Looks up the hierarchical box cells containing a specified point in world model
	 * coordinates considering numerical inaccuracies. Cells are looked up
	 * recursively to a specified depth level. A zero depth does not consider any
	 * children. A negative depth performs a full recursive search and considers
	 * non-parent cells only.
	 * 
	 * @param modelPoint the point in world model coordinates
	 * @param depth the hierarchical depth of the lookup operation
	 * 
	 * @return the hierarchical box cells containing the specified point
	 */
	public Set<? extends HierarchicalBox> lookupCells(Vec4 modelPoint, int depth) {
		Set<HierarchicalBox> lookedUpCells = new HashSet<HierarchicalBox>(8);

		// TODO: review if conversion to boxOrigin is needed
		// transform point to cell (body) coordinates
		Vec4 cellPoint = this.transformModelToBoxOrigin(modelPoint);

		if (this.containsV(cellPoint)) {
			lookedUpCells.add(this);
			if (this.hasChildren() && (0 != depth)) {
				for (HierarchicalBox child : this.getChildren()) {
					lookedUpCells.addAll(child.lookupCells(modelPoint, depth - 1));
				}
			}
		}
		return lookedUpCells;

	}

	/**
	 * Finds all cells of this hierarchical box that satisfy a specified predicate. A
	 * full recursive search is performed considering only non-parent cells.
	 * 
	 * @param predicate the predicate
	 * 
	 * @return the cells of this hierarchical box that satisfy a predicate
	 */
	public Set<? extends HierarchicalBox> findCells(Predicate<HierarchicalBox> predicate) {
		return this.findCells(predicate, -1);
	}

	/**
	 * Finds all cells of this hierarchical box that satisfy a specified predicate
	 * taking a specified hierarchical depth into account. A zero depth does not
	 * consider any children. A negative depth performs a full recursive search and
	 * considers non-parent cells only.
	 * 
	 * @param predicate the predicate
	 * @param depth the hierarchical depth
	 * 
	 * @return the cells of this hierarchical box that satisfy a predicate taking the
	 *         hierarchical depth into account
	 */
	public Set<? extends HierarchicalBox> findCells(Predicate<HierarchicalBox> predicate, int depth) {
		Set<HierarchicalBox> foundCells = new HashSet<HierarchicalBox>();

		if (predicate.test(this)) {
			if (this.hasChildren() && depth != 0) {
				for (HierarchicalBox child : this.getChildren()) {
					foundCells.addAll(child.findCells(predicate, depth - 1));
				}
			} else {
				foundCells.add(this);
			}
		}

		return foundCells;
	}

	/**
	 * Indicates whether or not a point is a waypoint in this hierarchical box. A full
	 * recursive search is performed considering only non-parent cells.
	 * 
	 * @param point the point in world model coordinates
	 * 
	 * @return true if the point is a waypoint in this hierarchical box, false otherwise
	 * 
	 * @see RegularGrid#isWaypoint(Vec4, int)
	 */
	public boolean isWaypoint(Vec4 point) {
		return this.isWaypoint(point, -1);
	}

	/**
	 * Indicates whether or not a point is a waypoint in this hierarchical box taking a
	 * specified hierarchical depth into account. A zero depth does not consider any
	 * children. A negative depth performs a full recursive search and considers
	 * non-parent cells only.
	 * 
	 * @param point the point in world model coordinates
	 * @param depth the hierarchical depth
	 * 
	 * @return true if the point is a waypoint in this hierarchical box taking the
	 *         hierarchical depth into account, false otherwise
	 */
	public boolean isWaypoint(Vec4 point, int depth) {
		Set<? extends HierarchicalBox> cells = this.lookupCells(point, depth);

		// Corners of the first environment are not waypoints
		cells.removeIf(c -> !c.hasParent());
		return 0 < cells.stream().filter(c -> c.isOrigin(point) || c.is3DOpposite(point)).count();
	}

	/**
	 * Gets the adjacent waypoints of a point in this hierarchical box. A full recursive
	 * search is performed considering only non-parent cells.
	 * 
	 * @param point the point in world model coordinates
	 * 
	 * @return the adjacent waypoints of the point in this hierarchical box, or the
	 *         waypoint itself
	 * 
	 * @see RegularGrid#getAdjacentWaypoints(Vec4, int)
	 */
	public Set<Vec4> getAdjacentWaypoints(Vec4 point) {
		return this.getAdjacentWaypoints(point, -1);
	}

	/**
	 * Gets the adjacent waypoints of a point in this hierarchical box taking a
	 * specified hierarchical depth into account. A zero depth does not consider any
	 * children. A negative depth performs a full recursive search and considers
	 * non-parent cells only.
	 * 
	 * @param point the point in world model coordinates
	 * @param depth the hierarchical depth
	 * 
	 * @return the adjacent waypoints of the point in this hierarchical box taking the
	 *         hierarchical depth into account, or the waypoint itself
	 */
	public Set<Vec4> getAdjacentWaypoints(Vec4 point, int depth) {
		Set<Vec4> adjacentWaypoints = new HashSet<Vec4>();
		Set<? extends HierarchicalBox> cells = this.lookupCells(point, depth);
		cells.removeIf(c -> !c.hasParent());

		if (cells.isEmpty()) {
			Set<Vec4> auxiliarWaypoints = new HashSet<Vec4>();
			for (HierarchicalBox cell : this.getAll()) {
				auxiliarWaypoints.add(cell.getOrigin());
				auxiliarWaypoints.add(cell.get3DOpposite());
			}
			//TODO: review maximum number of adjacent waypoints
			adjacentWaypoints = auxiliarWaypoints.stream()
					.sorted((o1, o2) -> ((Double) o1.distanceTo3(point)).compareTo((Double) o2.distanceTo3(point)))
					.limit(5).collect(Collectors.toSet());
		} else {
			for (HierarchicalBox cell : cells) {
				adjacentWaypoints.add(cell.getOrigin());
				adjacentWaypoints.add(cell.get3DOpposite());
			}
		}

		return adjacentWaypoints;
	}

	/**
	 * Indicates whether or not a point is adjacent to a waypoint in this regular
	 * grid.
	 * 
	 * @param point the point in world model coordinates
	 * @param waypoint the waypoint in world model coordinates
	 * 
	 * @return true if the point is adjacent to the waypoint in this hierarchical box,
	 *         false otherwise
	 */
	public boolean isAdjacentWaypoint(Vec4 point, Vec4 waypoint) {
		return this.getAdjacentWaypoints(point).stream().map(PrecisionVec4::new).collect(Collectors.toSet())
				.contains(new PrecisionVec4(waypoint));
	}

	/**
	 * Gets the neighbors of this hierarchical box. A full recursive search is performed
	 * considering only non-parent neighbors.
	 * 
	 * @return the non-parent neighbors of this hierarchical box
	 * 
	 * @see RegularGrid#getNeighbors(int)
	 */
	public Set<? extends HierarchicalBox> getNeighbors() {
		return this.getNeighbors(-1);
	}

	/**
	 * Gets the neighbors of this hierarchical box taking a specified hierarchical depth
	 * into account. A zero depth does not consider any neighboring children. A
	 * negative depth performs a full recursive search and considers non-parent
	 * neighbors only.
	 * 
	 * @param depth the hierarchical depth for finding neighbors
	 * 
	 * @return the neighbors of this hierarchical box
	 */
	public Set<? extends HierarchicalBox> getNeighbors(int depth) {
		// TODO: review implementation
		Set<HierarchicalBox> neighbors = new HashSet<HierarchicalBox>();

		if (this.hasParent()) {
			Set<HierarchicalBox> flatNeighbors = new HashSet<HierarchicalBox>();

			Vec4[] corners = new Vec4[2];
			corners[0] = this.getOrigin();
			corners[1] = this.get3DOpposite();
			// compute same level neighbors first
			for (Vec4 corner : corners) {
				flatNeighbors.addAll(this.parent.lookupCells(corner, 0));
			}
			flatNeighbors.remove(this);

			if (0 != depth) {
				for (HierarchicalBox neighbor : flatNeighbors) {
					neighbors.addAll(neighbor.findCells(c -> c.intersects(this), depth));
				}
			}

		}

		return neighbors;
	}

	/**
	 * Indicates whether or not this hierarchical box is a neighbor of another regular
	 * grid.
	 * 
	 * @param neighbor the potential neighbor
	 * 
	 * @return true if this hierarchical box is a neighbor of the other hierarchical box,
	 *         false otherwise
	 * 
	 * @see RegularGrid#getNeighbors()
	 */
	public boolean areNeighbors(HierarchicalBox neighbor) {
		return this.getNeighbors().contains(neighbor);
	}

	/**
	 * Indicates whether or not this hierarchical box is a neighbor of another regular
	 * grid taking a specified hierarchical depth into account.
	 * 
	 * @param neighbor the potential neighbor
	 * @param depth the hierarchical depth
	 * 
	 * @return true if this hierarchical box is a neighbor of the other hierarchical box
	 *         taking the hierarchical depth into account, false otherwise
	 * 
	 * @see RegularGrid#getNeighbors(int)
	 */
	public boolean areNeighbors(HierarchicalBox neighbor, int depth) {
		return this.getNeighbors(depth).contains(neighbor);
	}

	/**
	 * Gets the neighbors of a point in this hierarchical box. A full recursive search
	 * is performed considering non-parent cells only.
	 * 
	 * @param point the point in world model coordinates
	 * 
	 * @return the neighbors of the point in this hierarchical box
	 * 
	 * @see RegularGrid#getNeighbors(Vec4, int)
	 */
	public Set<Vec4> getNeighbors(Vec4 point) {
		return this.getNeighbors(point, -1);
	}

	/**
	 * Gets the neighbors of a point in this hierarchical box taking a specified
	 * hierarchical depth into account. A zero depth does not consider any children.
	 * A negative depth performs a full recursive search and considers non-parent
	 * cells only.
	 * 
	 * @param point the point in world model coordinates
	 * @param depth the hierarchical depth for finding neighbors
	 * 
	 * @return the neighbors of the point in this hierarchical box
	 */
	public Set<Vec4> getNeighbors(Vec4 point, int depth) {
		Set<Vec4> neighbors = new HashSet<Vec4>();
		Set<? extends HierarchicalBox> cells = this.lookupCells(point, depth);
		cells.removeIf(c -> !c.hasParent());
		Set<HierarchicalBox> cells2 = new HashSet<>();
		for (HierarchicalBox cell : cells) {
			cells2.addAll(cell.getNeighbors());
		}
		cells2.removeIf(c -> !c.hasParent());
		
		for (HierarchicalBox cell : cells2) {
			if (cell.isOrigin(point))
				neighbors.add(cell.get3DOpposite());
			else if (cell.is3DOpposite(point))
				neighbors.add(cell.getOrigin());
		}

		return neighbors;
	}

	/**
	 * Indicates whether or not two points are neighbors in this hierarchical box.
	 * 
	 * @param point the point
	 * @param neighbor the potential neighbor of the point
	 * 
	 * @return true if the two points are neighbors, false otherwise
	 * 
	 * @see RegularGrid#getNeighbors(Vec4)
	 */
	public boolean areNeighbors(Vec4 point, Vec4 neighbor) {
		return this.getNeighbors(point).stream().map(Vec4::toHomogeneousPoint3).map(PrecisionVec4::new)
				.collect(Collectors.toSet()).contains(new PrecisionVec4(neighbor.toHomogeneousPoint3()));
	}

	/**
	 * Indicates whether or not two points are neighbors in this hierarchical box taking
	 * a specified hierarchical depth into account.
	 * 
	 * @param point the point
	 * @param neighbor the potential neighbor of the point
	 * @param depth the hierarchical depth
	 * 
	 * @return true if the two points are neighbors taking the hierarchical depth
	 *         into account, false otherwise
	 * 
	 * @see RegularGrid#getNeighbors(Vec4, int)
	 */
	public boolean areNeighbors(Vec4 point, Vec4 neighbor, int depth) {
		return this.getNeighbors(point, depth).stream().map(Vec4::toHomogeneousPoint3).map(PrecisionVec4::new)
				.collect(Collectors.toSet()).contains(new PrecisionVec4(neighbor.toHomogeneousPoint3()));
	}

	// ------------------ DRAWING METHODS ----------------

	// TODO: include construction from axes, origin and lengths


	/**
	 * Draws this hierarchical box.
	 * 
	 * @see gov.nasa.worldwind.geom.Box
	 */
	@Override
	protected void drawBox(DrawContext dc, Vec4 a, Vec4 b, Vec4 c, Vec4 d) {
		Vec4 e = a.add3(r);
		Vec4 f = d.add3(r);
		GL2 gl = dc.getGL().getGL2(); // GL initialization checks for GL2
		// compatibility.

		dc.getView().pushReferenceCenter(dc, bottomCenter);
		OGLStackHandler ogsh = new OGLStackHandler();
		ogsh.pushModelview(gl);
		try {
			gl.glColor4f(this.color[0], this.color[1], this.color[2], this.color[3]);
			this.drawOutline(dc, a, b, c, d);
			gl.glTranslated(r.x, r.y, r.z);
			this.drawOutline(dc, a, b, c, d);
			gl.glPopMatrix();
			gl.glPushMatrix();
			this.drawOutline(dc, a, e, f, d);
			gl.glTranslated(s.x, s.y, s.z);
			this.drawOutline(dc, a, e, f, d);
		} finally {
			ogsh.pop(gl);
			dc.getView().popReferenceCenter(dc);
		}
	}

	/**
	 * Sets the drawing color of this hierarchical box.
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
	
	public Material getMaterial() {
		return new Material(new Color(this.color[0], this.color[1], this.color[2], this.color[3]));
	}

	/**
	 * Sets the visibility state of this hierarchical box.
	 * 
	 * @param visible true if this hierarchical box is visible, false otherwise
	 */
	public void setVisible(boolean visible) {
		this.visible = visible;
	}

	public static Box computeBoundingBox(Iterable<? extends Vec4> points, Vec4[] axes) {
		if (points == null) {
			String msg = Logging.getMessage("nullValue.PointListIsNull");
			Logging.logger().severe(msg);
			throw new IllegalArgumentException(msg);
		}

		if (axes == null) {
			String msg = Logging.getMessage("generic.ListIsEmpty");
			Logging.logger().severe(msg);
			throw new IllegalArgumentException(msg);
		}

		Vec4 r = axes[0];
		Vec4 s = axes[1];
		Vec4 t = axes[2];

		// Find the extremes along each axis.
		double minDotR = Double.MAX_VALUE;
		double maxDotR = -minDotR;
		double minDotS = Double.MAX_VALUE;
		double maxDotS = -minDotS;
		double minDotT = Double.MAX_VALUE;
		double maxDotT = -minDotT;

		for (Vec4 p : points) {
			if (p == null)
				continue;

			double pdr = p.dot3(r);
			if (pdr < minDotR)
				minDotR = pdr;
			if (pdr > maxDotR)
				maxDotR = pdr;

			double pds = p.dot3(s);
			if (pds < minDotS)
				minDotS = pds;
			if (pds > maxDotS)
				maxDotS = pds;

			double pdt = p.dot3(t);
			if (pdt < minDotT)
				minDotT = pdt;
			if (pdt > maxDotT)
				maxDotT = pdt;
		}

		if (maxDotR == minDotR)
			maxDotR = minDotR + 1;
		if (maxDotS == minDotS)
			maxDotS = minDotS + 1;
		if (maxDotT == minDotT)
			maxDotT = minDotT + 1;

		return new Box(axes, minDotR, maxDotR, minDotS, maxDotS, minDotT, maxDotT);
	}

}