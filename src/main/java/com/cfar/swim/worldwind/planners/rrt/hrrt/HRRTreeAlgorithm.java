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
package com.cfar.swim.worldwind.planners.rrt.hrrt;

/**
 * Enumerates the possible heuristic algorithms to guide the search for a
 * heuristic RRT planner.
 * 
 * @author Manuel Rosa
 *
 */
public enum HRRTreeAlgorithm {
	
	/**
	 * heuristically-guided RRT algorithm: tests if the nearest neighbor
	 * features a sufficiently high quality to pass the probabilistic quality
	 * test; if it does not, a new random sample is taken
	 */
	hRRT,
	
	/**
	 * iterative k-nearest algorithm: iteratively considers the k-nearest
	 * neighbors sorted by deceasing quality until one features a sufficiently
	 * high quality to pass the probabilistic quality test
	 */
	IkRRT,
	
	/**
	 * best of k-nearest algorithm: considers only the neighbor featuring the
	 * highest quality from the k-nearest neighbors and tests if its quality
	 * is sufficiently high to pass the probabilistic quality test
	 */
	BkRRT
	
}
