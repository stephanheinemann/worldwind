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
package com.cfar.swim.worldwind.environments;

import com.cfar.swim.worldwind.geom.precision.PrecisionPosition;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;

/**
 * Realizes a directed edge within a planning environment based on two sampled
 * positions featuring temporal costs.
 * 
 * @author Stephan Heinemann
 *
 */
public class DirectedEdge extends Edge {

	/**
	 * Constructs a new directed edge within a planning environment based on
	 * two end positions.
	 * 
	 * @param environment the planning environment containing this directed edge
	 * @param first the first end position of this directed edge
	 * @param second the second end position of this directed edge
	 * 
	 * @see Edge#Edge(Environment, Position, Position)
	 */
	public DirectedEdge(Environment environment, Position first, Position second) {
		super(environment, first, second);
	}
	
	/**
	 * Gets the opening bearing of a position in degrees in relation to this
	 * directed edge.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return the opening bearing of the position in degrees
	 */
	public Angle getOpeningBearing(Position position) {
		Angle ob = Angle.ZERO;
		
		// reduce precision for comparison
		PrecisionPosition first = new PrecisionPosition(this.getFirstPosition());
		PrecisionPosition second = new PrecisionPosition(position);
		
		if (!first.equals(second)) {
			DirectedEdge oe = new DirectedEdge(
					this.getEnvironment(), this.getFirstPosition(), position);
			Vec4 ov = oe.getSecond().subtract3(oe.getFirst());
			Vec4 v =  this.getSecond().subtract3(this.getFirst());
			
			ob = v.angleBetween3(ov);
		}
		
		return ob;
	}
	
	/**
	 * Gets the closing bearing of a position in degrees in relation to this
	 * directed edge.
	 * 
	 * @param position the position in globe coordinates
	 * 
	 * @return the closing bearing of the position in degrees
	 */
	public Angle getClosingBearing(Position position) {
		Angle cb = Angle.ZERO;
		
		// reduce precision for comparison
		PrecisionPosition first = new PrecisionPosition(position);
		PrecisionPosition second = new PrecisionPosition(this.getSecondPosition());
		
		if (!second.equals(first)) {
			DirectedEdge ce = new DirectedEdge(
					this.getEnvironment(), position, this.getSecondPosition());
			Vec4 cv = ce.getSecond().subtract3(ce.getFirst());
			Vec4 v =  this.getSecond().subtract3(this.getFirst());
			
			cb = v.angleBetween3(cv);
		}
		
		return cb;
	}
	
	/**
	 * Determines whether or not this directed edge equals another directed
	 * edge based on their end positions.
	 * 
	 * @param o the other directed edge
	 * 
	 * @return true if the end positions of this directed edge equal the end
	 *         positions of the other directed edge (with regard to order),
	 *         false otherwise
	 * 
	 * @see Edge#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = super.equals(o);

		if (equals) {
			DirectedEdge de = (DirectedEdge) o;
			equals = this.getFirstPosition().equals((de.getFirstPosition()))
					&& this.getSecondPosition().equals((de.getSecondPosition()));
		}

		return equals;
	}
	
	/**
	 * Gets the string representation of this directed edge.
	 * 
	 * @return the string representation of this directed edge
	 * 
	 * @see Edge#toString()
	 */
	@Override
	public String toString() {
		return "[" + this.getFirstPosition() + " -> "
				+ this.getSecondPosition() + "]";
	}
	
}
