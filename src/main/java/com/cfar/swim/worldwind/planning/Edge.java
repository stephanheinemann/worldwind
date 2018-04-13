/**
 * Copyright (c) 2018, Henrique Ferreira (UVic Center for Aerospace Research)
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
package com.cfar.swim.worldwind.planning;

import com.cfar.swim.worldwind.ai.continuum.SampledWaypoint;

/**
 * Realizes an edge of a roadmap based on two sampled waypoints.
 * 
 * @author Henrique Ferreira
 *
 */
public class Edge {

	/** the origin SampledWaypoint of this edge */
	private SampledWaypoint wpt1;

	/** the destination SampledWaypoint of this edge */
	private SampledWaypoint wpt2;

	/**
	 * Constructs an Edge based on two waypoints.
	 * 
	 * @param wpt1
	 * @param wpt2
	 */
	public Edge(SampledWaypoint wpt1, SampledWaypoint wpt2) {
		this.wpt1 = wpt1;
		this.wpt2 = wpt2;
	}

	/**
	 * Gets the first waypoint of this edge
	 * 
	 * @return the first waypoint of this edge
	 */
	public SampledWaypoint getWpt1() {
		return wpt1;
	}

	/**
	 * Sets the first waypoint of this edge
	 * 
	 * @param wpt1 the first waypoint of this edge
	 */
	public void setWpt1(SampledWaypoint wpt1) {
		this.wpt1 = wpt1;
	}

	/**
	 * Gets the second waypoint of this edge
	 * 
	 * @return the second waypoint of this edge
	 */
	public SampledWaypoint getWpt2() {
		return wpt2;
	}

	/**
	 * Sets the second waypoint of this edge
	 * 
	 * @param wpt1 the second waypoint of this edge
	 */
	public void setWpt2(SampledWaypoint wpt2) {
		this.wpt2 = wpt2;
	}

	/**
	 * Gets the hash code of this edge based on the hash code of the two waypoints
	 * 
	 * @return the multiplication of the hash code of the two waypoints
	 * 
	 * @see Object#hashCode()
	 */
	@Override
	public final int hashCode() {
		int result;
		result = wpt1.hashCode();
		result = result * wpt2.hashCode();
		return result;
	}
}
