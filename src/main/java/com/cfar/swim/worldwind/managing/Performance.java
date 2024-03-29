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

import java.io.Serializable;
import java.time.ZonedDateTime;

/**
 * Describes a performance.
 * 
 * @author Stephan Heinemann
 *
 * @see Quality
 * @see Quantity
 */
public interface Performance extends Comparable<Performance>, Serializable {
	
	/**
	 * Gets the measure of this performance.
	 * 
	 * @return the measure of this performance
	 */
	public double get();
	
	/**
	 * Gets the normalized measure of this performance.
	 * 
	 * @return the normalized measure of this performance
	 */
	public double getNormalized();
	
	/**
	 * Gets the quality of this performance.
	 * 
	 * @return the quality of this performance
	 */
	public Quality getQuality();
	
	/**
	 * Gets the quantity of this performance.
	 * 
	 * @return the quantity of this performance
	 */
	public Quantity getQuantity();
	
	/**
	 * Gets the context of this performance.
	 * 
	 * @return the context of this performance
	 */
	public PerformanceContext getContext();
	
	/**
	 * Gets the epoch of this performance.
	 * 
	 * @return the epoch of this performance
	 */
	public ZonedDateTime getEpoch();
	
}
