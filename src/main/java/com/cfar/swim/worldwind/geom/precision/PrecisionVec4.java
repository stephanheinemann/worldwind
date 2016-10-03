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
package com.cfar.swim.worldwind.geom.precision;

import gov.nasa.worldwind.geom.Vec4;

/**
 * Realizes a vector with a specified numerical precision.
 * 
 * @author Stephan Heinemann
 *
 */
public class PrecisionVec4 extends Vec4 implements Precision {

	/**
	 * the precision of this precision vector
	 */
	private int precision = Precision.PRECISION;
	
	/**
	 * the original vector of this precision vector
	 */
	private Vec4 original = null;
	
	/**
	 * Constructs a new precision vector from a vector using the default
	 * precision.
	 * 
	 * @param vector the vector
	 * 
	 * @see Precision#PRECISION
	 */
	public PrecisionVec4(Vec4 vector) {
		// TODO: the correct default precision should probably be a function (percentage) of the value
		super(
			new PrecisionDouble(vector.x, Precision.PRECISION).doubleValue(),
			new PrecisionDouble(vector.y, Precision.PRECISION).doubleValue(),
			new PrecisionDouble(vector.z, Precision.PRECISION).doubleValue(),
			new PrecisionDouble(vector.w, Precision.PRECISION).doubleValue());
			this.original = vector;
	}
	
	/**
	 * Constructs a new precision vector from a vector using a specified
	 * precision.
	 * 
	 * @param vector the vector
	 * @param precision the precision
	 */
	public PrecisionVec4(Vec4 vector, int precision) {
		super(
			new PrecisionDouble(vector.x, precision).doubleValue(),
			new PrecisionDouble(vector.y, precision).doubleValue(),
			new PrecisionDouble(vector.z, precision).doubleValue(),
			new PrecisionDouble(vector.w, precision).doubleValue());
		this.precision = precision;
		this.original = vector;
	}
	
	/**
	 * Gets the precision of this precision vector.
	 * 
	 * @return the precision of this precision vector
	 * 
	 * @see Precision#getPrecision()
	 */
	@Override
	public int getPrecision() {
		return this.precision;
	}

	/**
	 * Sets the precision of this precision vector.
	 * 
	 * @return a new precision vector with the set precision
	 * 
	 * @see Precision#setPrecision(int)
	 */
	@Override
	public PrecisionVec4 setPrecision(int precision) {
		return new PrecisionVec4(this.original, precision);
	}
	
	/**
	 * Gets the original vector of this precision vector.
	 * 
	 * @return the original vector of this precision vector
	 * 
	 * @see Precision#getOriginal()
	 */
	@Override
	public Vec4 getOriginal() {
		return this.original;
	}
	
}
