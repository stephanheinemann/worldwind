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

/**
 * Realizes a duration quantity.
 * 
 * @author Stephan Heinemann
 *
 * @see Quantity
 */
public class DurationQuantity implements Quantity {
	
	/** the quantity of this duration quantity */
	private final Duration quantity;
	
	/**
	 * Constructs a new duration quantity based on a duration.
	 * 
	 * @param quantity the duration
	 * 
	 * @throws IllegalArgumentException if the quantity is invalid
	 */
	public DurationQuantity(Duration quantity) {
		if ((null == quantity) || ((null != quantity)
				&& (quantity.isNegative() || quantity.isZero()))) {
			throw new IllegalArgumentException("quantity is invalid");
		}
		this.quantity = quantity;
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
		return quantity.getSeconds() + (quantity.getNano() * 10E-9d);
	}
	
}
