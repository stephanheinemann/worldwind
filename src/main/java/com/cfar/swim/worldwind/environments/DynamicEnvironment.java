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

import com.cfar.swim.worldwind.render.Obstacle;

import gov.nasa.worldwind.geom.Position;

/**
 * Describes a dynamic environment featuring dynamic obstacle embeddings.
 * 
 * @author Stephan Heinemann
 *
 */
public interface DynamicEnvironment extends Environment {
	
	/**
	 * Embeds an obstacle into this dynamic environment.
	 * 
	 * @param obstacle the obstacle to be embedded
	 * 
	 * @return true if the obstacle has been embedded, false otherwise
	 */
	public boolean embed(Obstacle obstacle);
	
	/**
	 * Umembeds an obstacle from this dynamic environment.
	 * 
	 * @param obstacle the obstacle to be unembedded
	 * 
	 * @return true if the obstacle has been unembedded, false otherwise
	 */
	public boolean unembed(Obstacle obstacle);
	
	/**
	 * Unembeds all obstacles from this dynamic environment.
	 */
	public void unembedAll();
	
	/**
	 * Determines whether or not an obstacle is embedded in this dynamic
	 * environment.
	 * 
	 * @param obstacle the obstacle
	 * 
	 * @return true if the obstacle is embedded in this dynamic environment,
	 *         false otherwise
	 */
	public boolean isEmbedded(Obstacle obstacle);
	
	/**
	 * Updates this dynamic environment for an embedded obstacle.
	 * 
	 * @param obstacle the embedded obstacle
	 */
	public void refresh(Obstacle obstacle);
	
	/**
	 * Gets the waypoint positions of this dynamic environment that are
	 * affected by an obstacle embedding.
	 * 
	 * @param obstacle the embedded obstacle
	 * @return the waypoint positions of this dynamic environment that are
	 *         affected by an obstacle embedding
	 */
	public Set<Position> getAffectedWaypointPositions(Obstacle obstacle);
	
	/**
	 * Gets the waypoint positions of this dynamic environment that are
	 * affected by obstacle embeddings.
	 * 
	 * @param obstacles the embedded obstacles
	 * @return the waypoint positions of this dynamic environment that are
	 *         affected by obstacle embeddings
	 */
	public Set<Position> getAffectedWaypointPositions(Set<Obstacle> obstacles);
	
}
