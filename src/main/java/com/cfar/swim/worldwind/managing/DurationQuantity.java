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

import java.time.Duration;
import java.util.Comparator;
import java.util.Objects;

import com.cfar.swim.worldwind.geom.precision.Precision;

/**
 * Realizes a duration quantity.
 * 
 * @author Stephan Heinemann
 *
 * @see Quantity
 */
public class DurationQuantity implements Quantity {
	
	// TODO: consider AbstractQuantity for compareTo, equals, hashCode
	
	/** the default serial identification of this duration quantity */
	private static final long serialVersionUID = 1L;
	
	/** the quantity of this duration quantity */
	private final Duration quantity;
	
	/** the quantity normalizer of this duration quantity */
	private final double normalizer;
	
	/**
	 * Constructs a new duration quantity based on a duration.
	 * 
	 * @param quantity the duration
	 * 
	 * @throws IllegalArgumentException if the quantity is invalid
	 */
	public DurationQuantity(Duration quantity) {
		this(quantity, 1d);
	}
	
	/**
	 * Constructs a new duration quantity based on a duration and a quantity
	 * normalizer.
	 * 
	 * @param quantity the duration
	 * @param normalizer the normalizer to be applied to the quantity measure
	 * 
	 * @throws IllegalArgumentException if the quantity is invalid
	 */
	public DurationQuantity(Duration quantity, double normalizer) {
		if ((null == quantity) || quantity.isNegative() || quantity.isZero()) {
			throw new IllegalArgumentException("quantity is invalid");
		}
		this.quantity = quantity;
		this.normalizer = normalizer;
	}
	
	/**
	 * Gets the duration quantity in seconds.
	 * 
	 * @return the duration quantity in seconds
	 * 
	 * @see Quantity#get()
	 */
	@Override
	public double get() {
		return this.quantity.getSeconds() + (this.quantity.getNano() * Precision.UNIT_NANO);
	}
	
	/**
	 * Gets the normalized duration quantity in seconds.
	 * 
	 * @return the normalized duration quantity in seconds
	 * 
	 * @see Quantity#getNormalized()
	 */
	@Override
	public double getNormalized() {
		return this.get() * this.normalizer;
	}
	
	/**
	 * Compares this duration quantity to another quantity.
	 * 
	 * @param quantity the other quantity
	 * 
	 * @return a negative integer, zero, or a positive integer as this
	 *         normalized duration quantity is less than, equal to, or greater
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
	 * Determines whether or not this duration quantity equals another
	 * duration quantity.
	 * 
	 * @param o the other duration quantity
	 * 
	 * @return true, if this duration quantity equals the other duration
	 *         quantity, false otherwise
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
					== ((DurationQuantity) o).getNormalized());
		}

		return equals;
	}
	
	/**
	 * Gets the hash code of this duration quantity.
	 * 
	 * @return the hash code of this duration quantity
	 * 
	 * @see Object#hashCode()
	 */
	@Override
	public final int hashCode() {
		return Objects.hash(this.getNormalized());
	}
	
}
