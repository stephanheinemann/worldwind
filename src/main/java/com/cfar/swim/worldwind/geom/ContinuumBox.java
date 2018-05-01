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

import javax.media.opengl.GL2;

import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.util.OGLStackHandler;

/**
 * TODO describe
 * 
 * @author Manuel Rosa
 *
 */
public class ContinuumBox extends Box {

	/**
	 * the drawing color of this regular grid
	 */
	private float[] color = { 1.0f, 1.0f, 1.0f, 1.0f };

	/**
	 * the visibility state of this regular grid
	 */
	protected boolean visible = true;

	/**
	 * Constructs a new continuum box a geometric box without any children.
	 * 
	 * @param box the geometric box
	 * 
	 * @see com.cfar.swim.worldwind.geom.Box#Box(gov.nasa.worldwind.geom.Box)
	 */
	public ContinuumBox(Box box) {
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
	public ContinuumBox(Vec4[] axes, double rMin, double rMax, double sMin, double sMax, double tMin, double tMax) {
		super(axes, rMin, rMax, sMin, sMax, tMin, tMax);
	}

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
