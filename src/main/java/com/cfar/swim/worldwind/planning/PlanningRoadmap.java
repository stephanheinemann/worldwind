/**
 * Copyright (c) 2016, Stephan Heinemann (UVic Center for Aerospace Research)
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

import com.cfar.swim.worldwind.ai.prm.basicprm.BasicPRM;
import com.cfar.swim.worldwind.ai.prm.lazyprm.LazyPRM;
import com.cfar.swim.worldwind.geom.Box;
import gov.nasa.worldwind.globes.Globe;

/**
 * Realizes a planning roadmap that extends a planning continuum, by
 * incorporating waypoint and edge lists. Can be used for motion planning.
 * 
 * @author Henrique Ferreira
 *
 */
public class PlanningRoadmap extends SamplingEnvironment {

	/** the maximum number of sampling iterations */
	public final int MAX_ITER;

	/** the maximum number of neighbors a waypoint can be connected to */
	public final int MAX_NEIGHBORS;

	/** the maximum distance between two neighboring waypoints */
	public final double MAX_DIST;

	public RoadmapConstructor roadmapConstructor;

	/**
	 * Constructs a planning roadmap based on a box, a waypoint list and a edge list
	 * 
	 * @param box the box used to define this environment
	 * @param waypointList the list of waypoints
	 * @param edgeList the list of edges
	 */
	public PlanningRoadmap(Box box, Globe globe) {
		super(box);
		this.setGlobe(globe);
		MAX_ITER = 1000;
		MAX_NEIGHBORS = 30;
		MAX_DIST = 200d;
	}

	/**
	 * Constructs a planning roadmap based on a box.
	 * 
	 * @param box the box used to define this environment
	 * @param resolution the resolution of this planning continuum
	 */
	public PlanningRoadmap(Box box, double resolution, RoadmapConstructor roadmapConstructor, Globe globe, int maxIter,
			int maxNeighbors, double maxDist) {
		super(box);
		this.setGlobe(globe);
		this.update();
		MAX_ITER = maxIter;
		MAX_NEIGHBORS = maxNeighbors;
		MAX_DIST = maxDist;
		this.roadmapConstructor = roadmapConstructor;
		this.constructRoadmap();
		this.update();
	}

	protected void constructRoadmap() {
		if (this.roadmapConstructor == RoadmapConstructor.BASICPRM) {
			BasicPRM basicPRM = new BasicPRM(this, MAX_ITER, MAX_NEIGHBORS, MAX_DIST);
			basicPRM.construct();
		}
		if (this.roadmapConstructor == RoadmapConstructor.LAZYPRM) {
			LazyPRM lazyPRM = new LazyPRM(this, MAX_ITER, MAX_NEIGHBORS, MAX_DIST);
			lazyPRM.construct();
		}
	}

}
