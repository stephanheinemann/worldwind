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

import gov.nasa.worldwind.geom.Line;
import gov.nasa.worldwind.geom.Position;

/**
 * Realizes a Desirability edge that extends a basic edge by incorporating a
 * desirability value and a lambda factor.
 * 
 * @author Henrique Ferreira
 *
 */
public class DesirabilityEdge extends Edge {

	/** the parameter lambda that weights the desirability influence on the cost */
	private double lambda;

	/** the desirability value of this waypoint */
	private double desirability;

	/**
	 * Constructs an Edge based on two positions.
	 * 
	 * @param position1 the first position
	 * @param position2 the second position
	 */
	public DesirabilityEdge(Position position1, Position position2) {
		super(position1, position2);
	}

	/**
	 * Constructs an Edge based on two positions and a line.
	 * 
	 * @param position1 the first position
	 * @param position2 the second position
	 * @param line the line formed by the two positions
	 */
	public DesirabilityEdge(Position position1, Position position2, Line line) {
		super(position1, position2, line);
	}

	/**
	 * Gets the parameter lambda of this desirability edge.
	 * 
	 * @return the lambda the parameter lambda of this desirability edge
	 */
	public double getLambda() {
		return lambda;
	}

	/**
	 * Sets the parameter lambda of this desirability edge.
	 * 
	 * @param lambda the parameter lambda to set
	 */
	public void setLambda(double lambda) {
		this.lambda = lambda;
	}

	/**
	 * Gets the desirability value of this desirability edge.
	 * 
	 * @return the desirability value of this desirability edge.
	 */
	public double getDesirability() {
		return desirability;
	}

	/**
	 * Sets the desirability value of this desirability edge.
	 * 
	 * @param desirability the desirability value to set
	 */
	public void setDesirability(double desirability) {
		this.desirability = desirability;
	}
}
