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

import com.cfar.swim.worldwind.planning.Trajectory;

/**
 * Realizes a trajectory quality.
 * 
 * @author Stephan Heinemann
 *
 * @see Quality
 */
public class TrajectoryQuality implements Quality {
	
	/** the quality of this trajectory quality */
	final Trajectory quality;
	
	/**
	 * Constructs a new trajectory quality based on a trajectory.
	 * 
	 * @param quality the trajectory
	 * 
	 * @throws IllegalArgumentException if the quality is invalid
	 */
	public TrajectoryQuality(Trajectory quality) {
		if (null == quality) {
			throw new IllegalArgumentException("quality is invalid");
		}	
		this.quality = quality;
	}
	
	/**
	 * Gets the trajectory quality in overall cost.
	 * 
	 * @return the trajectory quality in overall cost
	 * 
	 * @see Quality#get()
	 */
	@Override
	public double get() {
		double measure = 0d;
		
		if (!quality.isEmpty()) {
			measure = 1d / quality.getCost();
			// TODO: weighted quality
			// TODO: number of legs / edges (heading changes)
			// TODO: horizontal versus vertical distance (cost calculation)
			// TODO: ETE, ETA
		}
		
		return measure;
	}
	
	/**
	 * Compares this trajectory quality to another quality.
	 * 
	 * @param quality the other quality
	 * 
	 * @return a negative integer, zero, or a positive integer as this
	 *         trajectory quality is less than, equal to, or greater than the
	 *         other quality, respectively
	 * 
	 * @see Comparator#compare(Object, Object)
	 */
	@Override
	public int compareTo(Quality quality) {
		return Comparator.comparingDouble(Quality::get).compare(this, quality);
	}
	
	/**
	 * Determines whether or not this trajectory quality equals another
	 * trajectory quality.
	 * 
	 * @param o the other trajectory quality
	 * 
	 * @return true, if this trajectory quality equals the other trajectory
	 *         quality, false otherwise
	 * 
	 * @see Object#equals(Object)
	 */
	@Override
	public final boolean equals(Object o) {
		boolean equals = false;

		if (this == o) {
			equals = true;
		} else if ((null != o) && (this.getClass() == o.getClass())) {
			equals = this.get() == ((TrajectoryQuality) o).get();
		}

		return equals;
	}
	
	/**
	 * Gets the hash code of this trajectory quality.
	 * 
	 * @return the hash code of this trajectory quality
	 * 
	 * @see Object#hashCode()
	 */
	@Override
	public final int hashCode() {
		return Objects.hash(this.get());
	}
	
}
