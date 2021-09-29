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
 * Abstracts a performance consisting of quality and quantity.
 * 
 * @author Stephan Heinemann
 *
 * @see Performance
 * @see Quality
 * @see Quantity
 */
public abstract class AbstractPerformance implements Performance {
	
	// TODO: System information: memory, processor, cores, oshi artifact
	// https://github.com/oshi/oshi
	
	/** the quality of this abstract performance */
	private final Quality quality;
	
	/** the quantity of this abstract performance */
	private final Quantity quantity;
	
	/**
	 * Construct a new abstract performance based on a quality and quantity.
	 * 
	 * @param quality the performance quality
	 * @param quantity the performance quantity
	 */
	public AbstractPerformance(Quality quality, Quantity quantity) {
		if (null == quality) {
			throw new IllegalArgumentException("quality is invalid");
		}
		if (null == quantity) {
			throw new IllegalArgumentException("quantity is invalid");
		}
		this.quality = quality;
		this.quantity = quantity;
	}
	
	/**
	 * Gets the abstract performance measure dividing quality by quantity.
	 * 
	 * @return the abstract performance measure
	 * 
	 * @see Performance#get()
	 */
	@Override
	public double get() {
		return this.quality.get() / this.quantity.get();
	}
	
	/**
	 * Compares this abstract performance to another performance.
	 * 
	 * @param performance the other performance
	 * 
	 * @return a negative integer, zero, or a positive integer as this
	 *         abstract performance is less than, equal to, or greater than
	 *         the other performance, respectively
	 * 
	 * @see Comparator#compare(Object, Object)
	 */
	@Override
	public int compareTo(Performance performance) {
		return Comparator.comparingDouble(Performance::get).compare(this, performance);
	}
	
	/**
	 * Determines whether or not this abstract performance equals another
	 * abstract performance.
	 * 
	 * @param o the other abstract performance
	 * 
	 * @return true, if this abstract performance equals the other abstract
	 *         performance, false otherwise
	 * 
	 * @see Object#equals(Object)
	 */
	@Override
	public final boolean equals(Object o) {
		boolean equals = false;

		if (this == o) {
			equals = true;
		} else if ((null != o) && (this.getClass() == o.getClass())) {
			equals = this.get() == ((AbstractPerformance) o).get();
		}

		return equals;
	}
	
	/**
	 * Gets the hash code of this abstract performance.
	 * 
	 * @return the hash code of this abstract performance
	 * 
	 * @see Object#hashCode()
	 */
	@Override
	public final int hashCode() {
		return Objects.hash(this.get());
	}
	
}