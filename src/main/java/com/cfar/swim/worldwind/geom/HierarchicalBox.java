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

import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.Set;
import java.util.function.Predicate;

import javax.media.opengl.GL2;

import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.util.OGLStackHandler;

/**
 * @author Manuel Rosa
 *
 */
public class HierarchicalBox extends Box {

	/**
	 * the parent of this regular grid
	 */
	protected HierarchicalBox parent = null;

	/**
	 * the children of this regular grid
	 */
	protected HashSet<HierarchicalBox> cells = new HashSet<HierarchicalBox>();

	/**
	 * the drawing color of this regular grid
	 */
	private float[] color = { 1.0f, 1.0f, 1.0f, 1.0f };

	/**
	 * the visibility state of this regular grid
	 */
	protected boolean visible = true;

	// ------------------ CONSTRUCTORS ----------------
	/**
	 * Constructs a new continuum box a geometric box without any children.
	 * 
	 * @param box the geometric box
	 * 
	 * @see com.cfar.swim.worldwind.geom.Box#Box(gov.nasa.worldwind.geom.Box)
	 */
	public HierarchicalBox(Box box) {
		super(box);
	}

	/**
	 * Constructs a new regular grid from three plane normals and six distances for
	 * each of the six faces of a geometric box without any children.
	 * 
	 * @param axes the three plane normals
	 * @param rMin the minimum distance on the <code>R</code> axis
	 * @param rMax the maximum distance on the <code>R</code> axis
	 * @param sMin the minimum distance on the <code>S</code> axis
	 * @param sMax the maximum distance on the <code>S</code> axis
	 * @param tMin the minimum distance on the <code>T</code> axis
	 * @param tMax the maximum distance on the <code>T</code> axis
	 * 
	 * @see com.cfar.swim.worldwind.geom.Box#Box(Vec4[], double, double, double,
	 *      double, double, double)
	 */
	public HierarchicalBox(Vec4[] axes, double rMin, double rMax, double sMin, double sMax, double tMin, double tMax) {
		super(axes, rMin, rMax, sMin, sMax, tMin, tMax);
	}

	// ------------------ NEW METHODS ----------------
	public HierarchicalBox createInstance(Vec4 point1, Vec4 point2) {
		Vec4[] axes = this.getAxes();

		Vec4 boxPoint1 = this.transformModelToBoxOrigin(point1);
		Vec4 boxPoint2 = this.transformModelToBoxOrigin(point2);

		HierarchicalBox hierarchicalBox = new HierarchicalBox(axes, boxPoint1.x, boxPoint2.x, boxPoint1.y, boxPoint2.y,
				boxPoint1.z, boxPoint2.z);
		hierarchicalBox.parent = this;

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
	 * Removes all children from this regular grid.
	 */
	public void removeChildren() {
		this.cells.clear();
	}

	/**
	 * Indicates whether or not this regular grid has children.
	 * 
	 * @return true if this regular grid has children, false otherwise
	 */
	public boolean hasChildren() {
		return this.cells.isEmpty();
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
	 * @return the parent of this regular grid if present, null otherwise
	 */
	public HierarchicalBox getParent() {
		return this.parent;
	}

	/**
	 * Gets all regular grids associated with this regular grid.
	 * 
	 * @return all regular grids associated with this regular grid
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
	 * Looks up the regular grid cells (maximum eight) containing a specified point
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
	 * Looks up the regular grid cells (maximum eight) containing a specified point
	 * in world model coordinates considering numerical inaccuracies. Cells are
	 * looked up recursively to a specified depth level. A zero depth does not
	 * consider any children. A negative depth performs a full recursive search and
	 * considers non-parent cells only.
	 * 
	 * @param modelPoint the point in world model coordinates
	 * @param depth the hierarchical depth of the lookup operation
	 * 
	 * @return the regular grid cells containing the specified point
	 */
	public Set<? extends HierarchicalBox> lookupCells(Vec4 modelPoint, int depth) {
		Set<HierarchicalBox> lookedUpCells = new HashSet<HierarchicalBox>(8);

		// TODO: review if conversion to boxOrigin is needed
		// transform point to cell (body) coordinates
		Vec4 cellPoint = this.transformModelToBoxOrigin(modelPoint);

		if (this.containsV(cellPoint)) {
			if (this.hasChildren() && (0 != depth)) {
				for (HierarchicalBox child : this.getChildren()) {
					lookedUpCells.addAll(child.lookupCells(modelPoint, depth - 1));
				}
			}
		} else {
			lookedUpCells.add(this);
		}

		return lookedUpCells;

	}

	/**
	 * Finds all cells of this regular grid that satisfy a specified predicate. A
	 * full recursive search is performed considering only non-parent cells.
	 * 
	 * @param predicate the predicate
	 * 
	 * @return the cells of this regular grid that satisfy a predicate
	 */
	public Set<? extends HierarchicalBox> findCells(Predicate<HierarchicalBox> predicate) {
		return this.findCells(predicate, -1);
	}

	/**
	 * Finds all cells of this regular grid that satisfy a specified predicate
	 * taking a specified hierarchical depth into account. A zero depth does not
	 * consider any children. A negative depth performs a full recursive search and
	 * considers non-parent cells only.
	 * 
	 * @param predicate the predicate
	 * @param depth the hierarchical depth
	 * 
	 * @return the cells of this regular grid that satisfy a predicate taking the
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
	 * Indicates whether or not a point is a waypoint in this regular grid.
	 * A full recursive search is performed considering only non-parent cells.
	 * 
	 * @param point the point in world model coordinates
	 * 
	 * @return true if the point is a waypoint in this regular grid,
	 *         false otherwise
	 * 
	 * @see RegularGrid#isWaypoint(Vec4, int)
	 */
	public boolean isWaypoint(Vec4 point) {
		return this.isWaypoint(point, -1);
	}
	
	/**
	 * Indicates whether or not a point is a waypoint in this regular grid
	 * taking a specified hierarchical depth into account. A zero depth does
	 * not consider any children. A negative depth performs a full recursive
	 * search and considers non-parent cells only.
	 * 
	 * @param point the point in world model coordinates
	 * @param depth the hierarchical depth
	 * 
	 * @return true if the point is a waypoint in this regular grid taking the
	 *         hierarchical depth into account, false otherwise
	 */
	public boolean isWaypoint(Vec4 point, int depth) {
		Set<? extends HierarchicalBox> cells = this.lookupCells(point, depth);
		
		return 0 < cells.stream().filter(c -> c.isCenter(point)).count();
	}

	// ------------------ DRAWING METHODS ----------------

	// TODO: include construction from axes, origin and lengths

	/**
	 * Renders this regular grid. If a grid cell has children, then only the
	 * children are rendered.
	 * 
	 * @param dc the drawing context
	 */
	@Override
	public void render(DrawContext dc) {
		if (this.visible)
			super.render(dc);
	}

	/**
	 * Draws this regular grid.
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