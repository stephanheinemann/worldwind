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
package com.cfar.swim.worldwind.environments;

import java.util.Set;
import java.util.function.BiFunction;

import gov.nasa.worldwind.geom.Position;

/**
 * Describes a sampling environment featuring different sampling aspects.
 * 
 * @author Stephan Heinemann
 *
 */
public interface SamplingEnvironment extends Environment {

	/**
	 * Samples a random position from within this sampling environment using a
	 * uniform distribution.
	 * 
	 * @return the sampled position from within this sampling environment using
	 *         the uniform distribution in globe coordinates
	 */
	public Position sampleRandomUniformPosition();
	
	/**
	 * Samples a random position from within this sampling environment using a
	 * Gaussian (normal) distribution.
	 * 
	 * @return the sampled position from within this sampling environment using
	 *         the Gaussian (normal) distribution in globe coordinates
	 */ 
	public Position sampleRandomGaussianPosition();
	
	/**
	 * Samples a random position from within the intersection of this sampling
	 * environment and a default sized ellipsoid defined by two foci positions.
	 * 
	 * @param focusA the focus A of the ellipsoid in globe coordinates
	 * @param focusB the focus B of the ellipsoid in globe coordinates
	 * 
	 * @return the sampled position from within the intersection of this
	 *         sampling environment and the ellipsoid
	 */
	public Position sampleRandomEllipsoidPosition(
			Position focusA, Position focusB);
	
	/**
	 * Samples a random position from within the intersection of this 
	 * sampling environment and an ellipsoid defined by two foci positions
	 * and two ellipsoid radii.
	 * 
	 * @param focusA the focus A of the ellipsoid in globe coordinates
	 * @param focusB the focus B of the ellipsoid in globe coordinates
	 * @param a the minor ellipsoid radius
	 * @param b the major ellipsoid radius
	 * 
	 * @return the sampled position from within the intersection of this
	 *         planning continuum and the ellipsoid
	 */
	public Position sampleRandomEllipsoidPosition(
			Position focusA, Position focusB, double a, double b);
	
	/**
	 * Gets the optimal number of sampled neighbors to be considered for a
	 * connection to a new sample in this sampling environment.
	 * 
	 * @return the optimal number of sampled neighbors to be considered for a
	 *         connection to a new sample in this sampling environment
	 */
	public int getOptimalNumNearest();
	
	/**
	 * Finds the k-nearest sampled positions for a given position in this
	 * sampling environment.
	 * 
	 * @param position the position to query in globe coordinates
	 * @param k the maximum number of sampled positions to be found
	 * 
	 * @return the k-nearest sampled positions for the given position in this
	 *         sampling environment sorted by increasing distance
	 */
	public Set<? extends Position> findNearest(Position position, int k);
	
	/**
	 * Finds the k-nearest sampled positions closer than a certain distance
	 * from a given position in this sampling environment.
	 * 
	 * @param position the position in global coordinates
	 * @param k the maximum number of sampled positions to be found
	 * @param distance the maximum distance from the position
	 * 
	 * @return the k-nearest sampled positions closer than the distance from
	 *         the position in this sampling environment sorted by increasing
	 *         distance
	 */
	public Set<? extends Position> findNearest(
			Position position, int k, double distance);
	
	/**
	 * Finds the k-nearest sampled positions for a given position in this
	 * sampling environment using a particular distance metric.
	 * 
	 * @param position the position in global coordinates
	 * @param k the maximum number of sampled positions to be found
	 * @param metric the distance metric to be applied
	 * 
	 * @return the k-nearest sampled positions for the given position in this
	 *         sampling environment using the distance metric, sorted by
	 *         increasing distance
	 */
	public Set<? extends Position> findNearest(
			Position position, int k,
			BiFunction<Position, Position, Double> metric);
	
}
