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
import com.cfar.swim.worldwind.geom.HierarchicalBox;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.airspaces.ObstacleCylinder;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
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
	public int MAX_ITER;

	/** the maximum number of neighbors a waypoint can be connected to */
	public int MAX_NEIGHBORS;

	/** the maximum distance between two neighboring waypoints */
	public double MAX_DIST;

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
	
	public PlanningRoadmap(HierarchicalBox box) {
		super(box);
		this.refresh();
	}

	/**
	 * @return the mAX_ITER
	 */
	public int getMAX_ITER() {
		return MAX_ITER;
	}

	/**
	 * @param mAX_ITER the mAX_ITER to set
	 */
	public void setMAX_ITER(int mAX_ITER) {
		MAX_ITER = mAX_ITER;
	}

	/**
	 * @return the mAX_NEIGHBORS
	 */
	public int getMAX_NEIGHBORS() {
		return MAX_NEIGHBORS;
	}

	/**
	 * @param mAX_NEIGHBORS the mAX_NEIGHBORS to set
	 */
	public void setMAX_NEIGHBORS(int mAX_NEIGHBORS) {
		MAX_NEIGHBORS = mAX_NEIGHBORS;
	}

	/**
	 * @return the mAX_DIST
	 */
	public double getMAX_DIST() {
		return MAX_DIST;
	}

	/**
	 * @param mAX_DIST the mAX_DIST to set
	 */
	public void setMAX_DIST(double mAX_DIST) {
		MAX_DIST = mAX_DIST;
	}

	/**
	 * @return the roadmapConstructor
	 */
	public RoadmapConstructor getRoadmapConstructor() {
		return roadmapConstructor;
	}

	/**
	 * @param roadmapConstructor the roadmapConstructor to set
	 */
	public void setRoadmapConstructor(RoadmapConstructor roadmapConstructor) {
		this.roadmapConstructor = roadmapConstructor;
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
	
	@Override
	public void addChild(Position origin, Position other) {
		Globe globe = this.getGlobe();
		Vec4 pointOrigin = globe.computePointFromPosition(origin);
		Vec4 pointOther = globe.computePointFromPosition(other);

		PlanningRoadmap child = new PlanningRoadmap(super.createChild(pointOrigin, pointOther));

		child.setGlobe(globe);
		child.setTime(this.getTime());
		child.setThreshold(this.getThreshold());
		child.update();
		child.parent = this;
		this.cells.add(child);
		child.setOrigin(pointOrigin);
		
//		 propagate obstacle embeddings
		for (Obstacle obstacle : this.getObstacles()) {
			if (obstacle instanceof ObstacleCylinder) {
				if (child.embed((ObstacleCylinder) obstacle)) {
					this.addAffectedChild(obstacle, child);
				}
			}
		}
		child.setMAX_DIST(this.MAX_DIST);
		child.setMAX_ITER(this.MAX_ITER);
		child.setMAX_NEIGHBORS(this.MAX_NEIGHBORS);
		child.setRoadmapConstructor(this.roadmapConstructor);
	}
	
	
	/**
	 * Refines, that is, adds children with a specified density to this planning
	 * grid.
	 * 
	 * @param density the refinement density
	 * 
	 */
	@Override
	public void refine(int density) {
		if (this.roadmapConstructor == RoadmapConstructor.BASICPRM) {
			BasicPRM basicPRM = new BasicPRM(this, density, this.MAX_NEIGHBORS, this.MAX_DIST);
			basicPRM.construct();
		}
		if (this.roadmapConstructor == RoadmapConstructor.LAZYPRM) {
			LazyPRM lazyPRM = new LazyPRM(this, density, this.MAX_NEIGHBORS, this.MAX_DIST);
			lazyPRM.construct();
		}
	}
	
	/**
	 * Coarsens, that is, removes the children of this planning grid.
	 *
	 */
	@Override
	public void coarsen() {
		this.removeChildren();
	}

}
