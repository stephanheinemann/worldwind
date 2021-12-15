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
	
	// TODO: consider AbstractQuality for compareTo, equals, hashCode
	
	/** the default serial identification of this trajectory quality */
	private static final long serialVersionUID = 1L;
	
	/** the zero trajectory quality */
	public static final TrajectoryQuality ZERO = new TrajectoryQuality(new Trajectory(), 1d);
	
	/** the quality measure of this trajectory quality */
	private final double measure;
	
	/** the quality normalizer of this trajectory quality */
	private final double normalizer;
	
	/**
	 * Constructs a new trajectory quality based on a trajectory.
	 * 
	 * @param trajectory the trajectory
	 * 
	 * @throws IllegalArgumentException if the trajectory is invalid
	 */
	public TrajectoryQuality(Trajectory trajectory) {
		this(trajectory, 1d);
	}
	
	/**
	 * Constructs a new trajectory quality based on a trajectory and a quality
	 * normalizer.
	 * 
	 * @param trajectory the trajectory
	 * @param normalizer the normalizer to be applied to the quality measure
	 * 
	 * @throws IllegalArgumentException if the trajectory is invalid
	 */
	public TrajectoryQuality(Trajectory trajectory, double normalizer) {
		if (null == trajectory) {
			throw new IllegalArgumentException("trajectory is invalid");
		}
		
		if (trajectory.isEmpty()) {
			this.measure = 0d;
		} else {
			this.measure = 1d / trajectory.getCost();
			// TODO: weighted quality
			// TODO: number of legs / edges (heading changes)
			// TODO: horizontal versus vertical distance (cost calculation)
			// TODO: ETE, ETA
		}
		
		this.normalizer = normalizer;
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
		return this.measure;
	}
	
	/**
	 * Gets the normalized trajectory quality in overall cost.
	 * 
	 * @return the normalized trajectory quality in overall cost
	 * 
	 * @see Quality#getNormalized()
	 */
	@Override
	public double getNormalized() {
		return this.get() * this.normalizer;
	}
	
	/**
	 * Compares this trajectory quality to another quality.
	 * 
	 * @param quality the other quality
	 * 
	 * @return a negative integer, zero, or a positive integer as this
	 *         normalized trajectory quality is less than, equal to, or greater
	 *         than the other normalized quality, respectively
	 * 
	 * @see Comparator#compare(Object, Object)
	 */
	@Override
	public int compareTo(Quality quality) {
		return Comparator.comparingDouble(
				Quality::getNormalized).compare(this, quality);
	}
	
	/**
	 * Determines whether or not this trajectory quality equals another
	 * trajectory quality.
	 * 
	 * @param o the other trajectory quality
	 * 
	 * @return true, if this normalized trajectory quality equals the other
	 *         normalized trajectory quality, false otherwise
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
					== ((TrajectoryQuality) o).getNormalized());
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
		return Objects.hash(this.getNormalized());
	}
	
}
