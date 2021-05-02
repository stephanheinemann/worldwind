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

import java.math.BigDecimal;
import java.math.RoundingMode;

/**
 * Realizes a double value with a specified numerical precision.
 * 
 * @author Stephan Heinemann
 *
 */
public class PrecisionDouble extends BigDecimal implements Precision {
	
	private static final long serialVersionUID = 1L;
	
	/**
	 * the precision of this precision double
	 */
	private int precision = Precision.PRECISION;
	
	/**
	 * the original double of this precision double
	 */
	private double original = Double.NaN;
	
	/**
	 * Constructs a new precision double from a double value using the default
	 * precision.
	 * 
	 * @param d the double value
	 * 
	 * @see Precision#PRECISION
	 */
	public PrecisionDouble(double d) {
		// TODO: the correct default precision should probably be a function (percentage) of the value
		this(new BigDecimal(d).setScale(Precision.PRECISION, RoundingMode.HALF_UP));
		this.original = d;
	}
	
	/**
	 * Constructs a new precision double from a double value using a specified
	 * precision.
	 * 
	 * @param d the double value
	 * @param precision the precision
	 */
	public PrecisionDouble(double d, int precision) {
		this(new BigDecimal(d).setScale(precision, RoundingMode.HALF_UP));
		this.precision = precision;
		this.original = d;
	}
	
	/**
	 * Construct a new precision double from a big decimal (base class).
	 * 
	 * @param d the big decimal
	 */
	protected PrecisionDouble(BigDecimal d) {
		super(d.doubleValue());
	}
	
	/**
	 * Indicates whether or not this precision double lies is within a range
	 * taking its precision into account for the comparison.
	 * 
	 * @param l the lower bound of the range
	 * @param u the upper bound of the range
	 * 
	 * @return true if this precision double lies within the range taking its
	 *         precision into account for the comparison, false otherwise
	 */
	public boolean isInRange(double l, double u) {
		double pd = this.doubleValue();
		double pl = (new PrecisionDouble(l, this.precision)).doubleValue();
		double pu = (new PrecisionDouble(u, this.precision)).doubleValue();
		return (pl <= pd) && (pu >= pd);
	}

	/**
	 * Gets the precision of this precision double.
	 * 
	 * @return the precision of this precision double
	 * 
	 * @see Precision#getPrecision()
	 */
	@Override
	public int getPrecision() {
		return this.precision;
	}

	/**
	 * Sets the precision of this (immutable) precision double.
	 * 
	 * @return a new precision double with the set precision
	 * 
	 * @see Precision#setPrecision(int)
	 */
	@Override
	public PrecisionDouble setPrecision(int precision) {
		return new PrecisionDouble(this.original, precision);
	}

	/**
	 * Gets the original double of this precision double.
	 * 
	 * @return the original double of this precision double
	 * 
	 * @see Precision#getOriginal()
	 */
	@Override
	public Double getOriginal() {
		return Double.valueOf(this.original);
	}
	
	// TODO: override arithmetic operations to propagate
	// operations on the original value
}
