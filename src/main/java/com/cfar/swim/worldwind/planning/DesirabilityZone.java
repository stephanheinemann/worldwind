/**
 * Copyright (c) 2018, Henrique Ferreira (UVic Center for Aerospace Research)
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
package com.cfar.swim.worldwind.planning;

import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.geom.ContinuumBox;

import gov.nasa.worldwind.geom.Line;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Renderable;

/**
 * Realizes a desirability zone.
 * 
 * @author Henrique Ferreira
 *
 */
public class DesirabilityZone extends ContinuumBox implements Renderable {

	/** the globe of this desirability zone */
	private Globe globe = null;

	/** the desirability of this desirability zone */
	private double desirability = 0.5d;

	/**
	 * Constructs a desirability zone based on a geometric box and a specified
	 * desirability value.
	 * 
	 * @param box the geometric box
	 * @param desirability the desirability
	 * 
	 * @see Box#Box(gov.nasa.worldwind.geom.Box)
	 */
	public DesirabilityZone(Box box, double desirability) {
		super(box);
		this.desirability = desirability;
		this.update();
	}

	/**
	 * Sets the globe of this desirability zone.
	 * 
	 * @param globe the globe of this desirability zone
	 * 
	 * @see Environment#setGlobe(Globe)
	 */
	public void setGlobe(Globe globe) {
		this.globe = globe;
	}

	/**
	 * Gets the globe of this desirability zone.
	 * 
	 * @return the globe of this desirability zone
	 * 
	 * @see Environment#getGlobe()
	 */
	public Globe getGlobe() {
		return this.globe;
	}

	/**
	 * Gets the desirability value of this desirability zone.
	 * 
	 * @return the desirability value of this desirability zone
	 */
	public double getDesirability() {
		return desirability;
	}

	/**
	 * Sets the desirability value of this desirability zone.
	 * 
	 * @param desirability the desirability to set
	 */
	public void setDesirability(double desirability) {
		this.desirability = desirability;
	}

	/**
	 * Updates this desirability zone.
	 */
	protected void update() {
		this.updateAppearance();
	}

	/**
	 * Updates the appearance of this desirability zone.
	 */
	protected void updateAppearance() {

		float r = 0, g = 0, b = 0;
		float des = (float) desirability;
		if (this.desirability <= 0.5) {
			r = 255;
			g = 510 * des;
			b = 510 * des;
		}
		if (this.desirability > 0.5) {
			r = -510 * des + 510;
			g = 255;
			b = -510 * des + 510;
		}
		float red = r / 255.0f;
		float green = g / 255.0f;
		float blue = b / 255.0f;
		float alpha = 0.5f;
		this.setColor(red, green, blue, alpha);
	}

	/**
	 * Gets the desirability cost of a line formed by two given positions, checking
	 * if it intersects this desirability zone.
	 * 
	 * @param position1 the first position
	 * @param position2 the second position
	 * 
	 * @return the desirability value of this desirability zone, if the line
	 *         intersects the zone, 0d otherwise
	 */
	/*
	public double getDesirabilityCost(Position position1, Position position2) {
		Vec4 point1 = this.getGlobe().computePointFromPosition(position1);
		Vec4 point2 = this.getGlobe().computePointFromPosition(position2);

		if (this.intersects(new Line(point1, point2))) {
			return this.desirability;
		}
		return 0d;
	}*/
}
