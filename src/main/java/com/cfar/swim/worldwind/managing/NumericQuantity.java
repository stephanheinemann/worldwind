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
package com.cfar.swim.worldwind.managing;

import java.util.Comparator;
import java.util.Objects;

/**
 * Realizes a numeric quantity.
 * 
 * @author Stephan Heinemann
 *
 * @see Quantity
 */
public class NumericQuantity implements Quantity {
	
	// TODO: consider AbstractQuantity for compareTo, equals, hashCode
	
	/** the default serial identification of this numeric quantity */
	private static final long serialVersionUID = 1L;
	
	/** the numeric value of this numeric quantity */
	private final Number value; 
	
	/** the normalizer of this numeric quantity */
	private final Number normalizer;
	
	/**
	 * Constructs a new numeric quantity based on a numeric value.
	 * 
	 * @param value the numeric value
	 * 
	 * @throws IllegalArgumentException if the numeric value is invalid
	 */
	public NumericQuantity(Number value) {
		this(value, 1);
	}
	
	/**
	 * Constructs a new numeric quantity based on a numeric value and a
	 * quantity normalizer.
	 * 
	 * @param value the numeric value
	 * @param normalizer the normalizer to be applied to the quantity measure
	 * 
	 * @throws IllegalArgumentException if the numeric value is invalid
	 */
	public NumericQuantity(Number value, Number normalizer) {
		if ((null == value) || (0d >= value.doubleValue())) {
			throw new IllegalArgumentException("numeric value is invalid");
		}
		this.value = value;
		this.normalizer = normalizer;
	}
	
	/**
	 * Gets the numeric value of this numeric quantity.
	 * 
	 * @return the numeric value of this numeric quantity
	 * 
	 * @see Quantity#get()
	 */
	@Override
	public double get() {
		return this.value.doubleValue();
	}
	
	/**
	 * Gets the normalized numeric value of this numeric quantity.
	 * 
	 * @return the normalized numeric value of this numeric quantity
	 * 
	 * @see Quantity#getNormalized()
	 */
	@Override
	public double getNormalized() {
		return this.get() * this.normalizer.doubleValue();
	}
	
	/**
	 * Compares this numeric quantity to another quantity.
	 * 
	 * @param quantity the other quantity
	 * 
	 * @return a negative integer, zero, or a positive integer as this
	 *         normalized numeric quantity is less than, equal to, or greater
	 *         than the other normalized quantity, respectively
	 * 
	 * @see Comparator#compare(Object, Object)
	 */
	@Override
	public int compareTo(Quantity quantity) {
		return Comparator.comparingDouble(
				Quantity::getNormalized).compare(this, quantity);
	}
	
	/**
	 * Determines whether or not this numeric quantity equals another numeric
	 * quantity.
	 * 
	 * @param o the other numeric quantity
	 * 
	 * @return true, if this normalized numeric quantity equals the other
	 *         normalized numeric quantity, false otherwise
	 * 
	 * @see Object#equals(Object)
	 */
	@Override
	public final boolean equals(Object o) {
		boolean equals = false;

		if (this == o) {
			equals = true;
		} else if ((null != o) && (this.getClass() == o.getClass())) {
			equals = (this.getNormalized()
					== ((NumericQuantity) o).getNormalized());
		}

		return equals;
	}
	
	/**
	 * Gets the hash code of this numeric quantity.
	 * 
	 * @return the hash code of this numeric quantity
	 * 
	 * @see Object#hashCode()
	 */
	@Override
	public final int hashCode() {
		return Objects.hash(this.getNormalized());
	}
	
}
