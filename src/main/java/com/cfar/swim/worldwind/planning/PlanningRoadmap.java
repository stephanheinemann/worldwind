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

import java.util.HashSet;
import java.util.Set;

import com.cfar.swim.worldwind.ai.prm.basicprm.BasicPRMConstructor;
import com.cfar.swim.worldwind.ai.prm.lazyprm.LazyPRMConstructor;
import com.cfar.swim.worldwind.geom.Box;
import com.cfar.swim.worldwind.geom.HierarchicalBox;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.airspaces.ObstacleCylinder;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.Path;

/**
 * Realizes a planning roadmap that extends a sampling environment,
 * incorporating a roadmap constructor, which populates the environment.
 * 
 * @author Henrique Ferreira
 *
 */
public class PlanningRoadmap extends SamplingEnvironment {

	/** the maximum number of sampling iterations */
	protected int maxIter;

	/** the maximum number of neighbors a position can be connected to */
	protected int maxNeighbors;

	/** the maximum distance between two neighboring sampled positions */
	protected double maxDist;

	/** the constructor used to populate this planning roadmap */
	protected RoadmapConstructor roadmapConstructor;

	/**
	 * Constructs a planning roadmap based on a box.
	 * 
	 * @param box the geometric box
	 * @param resolution the resolution of this planning roadmap
	 * @param roadmapConstructor the constructor used to populate this planning
	 *            roadmap
	 * @param globe the globe
	 * @param maxIter the maximum number of iterations
	 * @param maxNeighbors the maximum number of neighbors
	 * @param maxDist the maximum distance between two connected neighbors
	 */
	public PlanningRoadmap(Box box, double resolution, RoadmapConstructor roadmapConstructor, Globe globe, int maxIter,
			int maxNeighbors, double maxDist) {
		super(box);
		this.setGlobe(globe);
		this.maxIter = maxIter;
		this.maxNeighbors = maxNeighbors;
		this.maxDist = maxDist;
		this.roadmapConstructor = roadmapConstructor;
		this.constructRoadmap();
		this.update();
	}

	/**
	 * Constructs a planning roadmap based on a hierarchical box.
	 * 
	 * @param box the hierarchical box used to define this environment
	 */
	public PlanningRoadmap(HierarchicalBox box) {
		super(box);
		this.refresh();
	}

	/**
	 * Gets the maximum number of sampling iterations of this planning roadmap.
	 * 
	 * @return the maxIter the maximum number of sampling iterations of this
	 *         planning roadmap
	 */
	public int getMaxIter() {
		return maxIter;
	}

	/**
	 * Sets the maximum number of sampling iterations of this planning roadmap.
	 * 
	 * @param maxIter the maxIter to set
	 */
	public void setMaxIter(int maxIter) {
		this.maxIter = maxIter;
	}

	/**
	 * Gets the maximum number of neighbors a position can be connected to.
	 * 
	 * @return the maxNeighbors the maximum number of neighbors a position can be
	 *         connected to
	 */
	public int getMaxNeighbors() {
		return maxNeighbors;
	}

	/**
	 * Sets the maximum number of neighbors a position can be connected to.
	 * 
	 * @param maxNeighbors the maxNeighbors to set
	 */
	public void setMaxNeighbors(int maxNeighbors) {
		this.maxNeighbors = maxNeighbors;
	}

	/**
	 * Gets the maximum distance between two neighboring sampled positions.
	 * 
	 * @return the maxDist the maximum distance between two neighboring sampled
	 *         positions
	 */
	public double getMaxDist() {
		return maxDist;
	}

	/**
	 * Sets the maximum distance between two neighboring sampled positions.
	 * 
	 * @param maxDist the maxDist to set
	 */
	public void setMaxDist(double maxDist) {
		this.maxDist = maxDist;
	}

	/**
	 * Gets the constructor used to populate this planning roadmap.
	 * 
	 * @return the roadmapConstructor the constructor used to populate this planning
	 *         roadmap
	 */
	public RoadmapConstructor getRoadmapConstructor() {
		return roadmapConstructor;
	}

	/**
	 * Sets the constructor used to populate this planning roadmap.
	 * 
	 * @param roadmapConstructor the roadmapConstructor to set
	 */
	public void setRoadmapConstructor(RoadmapConstructor roadmapConstructor) {
		this.roadmapConstructor = roadmapConstructor;
	}

	/**
	 * Populates this planning roadmap based on the constructor stored in the
	 * variable roadmapConstructor.
	 */
	protected void constructRoadmap() {
		if (this.roadmapConstructor == RoadmapConstructor.BASICPRM) {
			BasicPRMConstructor basicPRM = new BasicPRMConstructor(this, maxIter, maxNeighbors, maxDist);
			basicPRM.construct();
		}
		if (this.roadmapConstructor == RoadmapConstructor.LAZYPRM) {
			LazyPRMConstructor lazyPRM = new LazyPRMConstructor(this, maxIter, maxNeighbors, maxDist);
			lazyPRM.construct();
		}
	}

	/**
	 * Corrects a trajectory, checking if any of its positions or edges is in
	 * conflict with terrain obstacles.
	 * 
	 * @param trajectory the planned trajectory
	 * 
	 * @return true if this trajectory is feasible, false otherwise
	 */
	public boolean correctTrajectory(Trajectory trajectory) {
		// TODO: only position checks are done. Edge conflicts are still not done
		if (trajectory == null)
			return false;

		HashSet<Waypoint> conflictWaypoints = new HashSet<Waypoint>();

		for (Waypoint waypoint : trajectory.getWaypoints()) {
			if (this.checkConflict(waypoint)) {
				conflictWaypoints.add(waypoint);
			}
		}
		if (!conflictWaypoints.isEmpty()) {
			this.correctLists(conflictWaypoints);
			return false;
		}
		return true;
	}

	/**
	 * Corrects the waypoint and edge list by removing the waypoints that are in
	 * conflict with terrain obstacles.
	 * 
	 * @param conflictWaypoints the set of waypoints that are in conflict with
	 *            terrain obstacles
	 */
	@SuppressWarnings("unchecked")
	protected void correctLists(HashSet<Waypoint> conflictWaypoints) {
		Set<PlanningRoadmap> conflictCells = new HashSet<PlanningRoadmap>();
		for (Waypoint waypoint : conflictWaypoints) {
			conflictCells.addAll((Set<PlanningRoadmap>) this.lookupCells(waypoint));
		}
		conflictCells.removeIf(c -> !c.hasParent());
		for (PlanningRoadmap roadmap : conflictCells) {
			this.removeChild(roadmap);
		}
	}

	/**
	 * Adds a child to this planning roadmap, constructing a new planning roadmap
	 * based on two positions.
	 * 
	 * @param origin the origin position in globe coordinates
	 * @param other the other position in globe coordinates
	 * 
	 * @see com.cfar.swim.worldwind.planning.SamplingEnvironment#addChild(gov.nasa.worldwind.geom.Position,
	 *      gov.nasa.worldwind.geom.Position)
	 */
	@Override
	public void addChild(Position origin, Position other) {
		Globe globe = this.getGlobe();
		Vec4 pointOrigin = globe.computePointFromPosition(origin);
		Vec4 pointOther = globe.computePointFromPosition(other);

		PlanningRoadmap child = new PlanningRoadmap(super.createInstance(pointOrigin, pointOther));

		child.setGlobe(globe);
		child.setTime(this.getTime());
		child.setThreshold(this.getThreshold());
		child.update();
		child.parent = this;
		this.cells.add(child);
		child.setOrigin(pointOrigin);

		// propagate obstacle embeddings
		for (Obstacle obstacle : this.getObstacles()) {
			if (obstacle instanceof ObstacleCylinder) {
				if (child.embed((ObstacleCylinder) obstacle)) {
					this.addAffectedChild(obstacle, child);
				}
			}
		}
		child.setMaxDist(this.maxDist);
		child.setMaxIter(this.maxIter);
		child.setMaxNeighbors(this.maxNeighbors);
		child.setRoadmapConstructor(this.roadmapConstructor);
	}

	/**
	 * Refines, that is, adds children with a specified density to this planning
	 * roadmap.
	 * 
	 * @param density the maximum number of positions to sample
	 * 
	 * @see com.cfar.swim.worldwind.planning.SamplingEnvironment#refine(int)
	 */
	@Override
	public void refine(int density) {
		if (this.roadmapConstructor == RoadmapConstructor.BASICPRM) {
			BasicPRMConstructor basicPRM = new BasicPRMConstructor(this, density, this.maxNeighbors, this.maxDist);
			if (basicPRM.supports(this))
				basicPRM.construct();
		}
		if (this.roadmapConstructor == RoadmapConstructor.LAZYPRM) {
			LazyPRMConstructor lazyPRM = new LazyPRMConstructor(this, density, this.maxNeighbors, this.maxDist);
			if (lazyPRM.supports(this))
				lazyPRM.construct();
		}
	}

	/**
	 * Coarsens, that is, removes the children of this planning roadmap.
	 *
	 * @see com.cfar.swim.worldwind.planning.SamplingEnvironment#coarsen()
	 */
	@Override
	public void coarsen() {
		this.removeChildren();
	}

	/**
	 * Gets all planning roadmaps associated with this planning roadmap.
	 * 
	 * @return all planning roadmaps associated with this planning roadmap
	 * 
	 * @see com.cfar.swim.worldwind.planning.SamplingEnvironment#getAll()
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Set<? extends PlanningRoadmap> getAll() {
		return (Set<PlanningRoadmap>) super.getAll();
	}

	/**
	 * Renders this planning roadmap. If it has children, then the children are also
	 * rendered.
	 * 
	 * @param dc the drawing context
	 */
	@Override
	public void render(DrawContext dc) {
		if (this.visible) {
			if (this.hasParent()) {
				Path diagonal = this.createRenderableDiagonal();
				diagonal.render(dc);
			} else {
				super.render(dc);
			}
			if (this.hasChildren()) {
				for (SamplingEnvironment child : this.getChildren()) {
					child.render(dc);

				}
			}
		}
	}
}
