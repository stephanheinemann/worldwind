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

import java.time.ZonedDateTime;
import java.util.Comparator;
import java.util.Objects;

/**
 * Abstracts a performance consisting of quality and quantity within a context.
 * 
 * @author Stephan Heinemann
 *
 * @see Performance
 * @see Quality
 * @see Quantity
 */
public abstract class AbstractPerformance implements Performance {
		
	/** the default serial identification of this abstract performance */
	private static final long serialVersionUID = 1L;
	
	/** the quality of this abstract performance */
	private final Quality quality;
	
	/** the quantity of this abstract performance */
	private final Quantity quantity;
	
	/** the context of this abstract performance */
	private final PerformanceContext context;
	
	/** the epoch of this abstract performance */
	private final ZonedDateTime epoch;
	
	/**
	 * Construct a new abstract performance based on an epoch, a quality and
	 * quantity.
	 * 
	 * @param epoch the performance epoch 
	 * @param quality the performance quality
	 * @param quantity the performance quantity
	 * 
	 * @throws IllegalArgumentException if the epoch, quality or quantity are
	 *                                  invalid
	 */
	public AbstractPerformance(
			ZonedDateTime epoch, Quality quality, Quantity quantity) {
		if ((null == epoch) || epoch.isAfter(ZonedDateTime.now())) {
			throw new IllegalArgumentException("epoch is invalid");
		}
		if (null == quality) {
			throw new IllegalArgumentException("quality is invalid");
		}
		if ((null == quantity) || (0d >= quantity.get())) {
			throw new IllegalArgumentException("quantity is invalid");
		}
		this.epoch = epoch;
		this.quality = quality;
		this.quantity = quantity;
		this.context = new PerformanceContext();
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
	 * Gets the normalized abstract performance measure dividing normalized
	 * quality by normalized quantity.
	 * 
	 * @return the normalized abstract performance measure
	 * 
	 * @see Performance#getNormalized()
	 */
	@Override
	public double getNormalized() {
		return this.quality.getNormalized() / this.quantity.getNormalized();
	}
	
	/**
	 * Gets the quality of this abstract performance.
	 * 
	 * @return the quality of this abstract performance
	 * 
	 * @see Performance#getQuality()
	 */
	@Override
	public Quality getQuality() {
		return this.quality;
	}
	
	/**
	 * Gets the quantity of this abstract performance.
	 * 
	 * @return the quantity of this abstract performance
	 * 
	 * @see Performance#getQuantity()
	 */
	@Override
	public Quantity getQuantity() {
		return this.quantity;
	}
	
	/**
	 * Gets the context of this abstract performance.
	 * 
	 * @return the context of this abstract performance
	 * 
	 * @see Performance#getContext()
	 */
	@Override
	public PerformanceContext getContext() {
		return this.context;
	}
	
	/**
	 * Gets the epoch of this abstract performance.
	 * 
	 * @return the epoch of this abstract performance
	 * 
	 * @see Performance#getEpoch()
	 */
	@Override
	public ZonedDateTime getEpoch() {
		return this.epoch;
	}
	
	/**
	 * Compares this abstract performance to another performance.
	 * 
	 * @param performance the other performance
	 * 
	 * @return a negative integer, zero, or a positive integer as this
	 *         normalized abstract performance is less than, equal to, or
	 *         greater than the other normalized performance, respectively
	 * 
	 * @see Comparator#compare(Object, Object)
	 */
	@Override
	public int compareTo(Performance performance) {
		return Comparator.comparingDouble(
				Performance::getNormalized).compare(this, performance);
	}
	
	/**
	 * Determines whether or not this abstract performance equals another
	 * abstract performance.
	 * 
	 * @param o the other abstract performance
	 * 
	 * @return true, if this normalized abstract performance equals the other
	 *         normalized abstract performance, false otherwise
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
					== ((AbstractPerformance) o).getNormalized());
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
		return Objects.hash(this.getNormalized());
	}
	
	/**
	 * Gets the string representation of this abstract performance.
	 * 
	 * @return the string representation of this abstract performance
	 * 
	 * @see Object#toString()
	 */
	@Override
	public String toString() {
		return  this.getEpoch().toString() + ": ["
				+ String.valueOf(this.getQuality().get()) + ", "
				+ String.valueOf(this.getQuantity().get()) + "] "
				+ String.valueOf(this.getQuality().getNormalized()) + " / "
				+ String.valueOf(this.getQuantity().getNormalized()) + " = "
				+ String.valueOf(this.getNormalized());
	}
	
}
