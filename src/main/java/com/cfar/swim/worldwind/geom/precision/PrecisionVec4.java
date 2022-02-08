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
	private int precision = Precision.DECA_MICRO;
	
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
	 * @see PrecisionDouble
	 * @see Vec4
	 */
	public PrecisionVec4(Vec4 vector) {
		// TODO: the correct default precision should probably be a function (percentage) of the value
		super(
			new PrecisionDouble(vector.x, Precision.DECA_MICRO).doubleValue(),
			new PrecisionDouble(vector.y, Precision.DECA_MICRO).doubleValue(),
			new PrecisionDouble(vector.z, Precision.DECA_MICRO).doubleValue(),
			new PrecisionDouble(vector.w, Precision.DECA_MICRO).doubleValue());
			this.original = vector;
	}
	
	/**
	 * Constructs a new precision vector from a vector using a specified
	 * precision.
	 * 
	 * @param vector the vector
	 * @param precision the precision
	 * 
	 * @see PrecisionDouble
	 * @see Vec4
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
	
	/**
	 * Indicates whether or not a specified vector is colinear to this
	 * precision vector.
	 * 
	 * @param vector the vector
	 * 
	 * @return true if the vector is colinear to this precision vector,
	 *         false otherwise
	 */
	public boolean isColinear(Vec4 vector) {
		PrecisionDouble d = new PrecisionDouble(this.original.dot3(vector), this.precision);
		return d.equals(new PrecisionDouble(1, this.precision));
	}
	
	/**
	 * Indicates whether or not a specified vector is orthonormal to this
	 * precision vector.
	 * 
	 * @param vector the vector
	 * 
	 * @return true if the vector is orthonormal to this precision vector,
	 *         false otherwise
	 */
	public boolean isOrthonormal(Vec4 vector) {
		PrecisionDouble d = new PrecisionDouble(this.original.dot3(vector), this.precision);
		return d.equals(new PrecisionDouble(0, this.precision));
	}
	
	/**
	 * Indicates whether or not two specified vectors are orthonormal to each
	 * other and this precision vector.
	 * 
	 * @param a the first vector
	 * @param b the second vector
	 * 
	 * @return true if the two vectors are orthonormal to each other and this
	 *         precision vector
	 */
	public boolean areOrthonormal(Vec4 a, Vec4 b) {
		PrecisionVec4 pa = new PrecisionVec4(a, this.precision);
		return this.isOrthonormal(a) && this.isOrthonormal(b) && pa.isOrthonormal(b);
	}
	
}
