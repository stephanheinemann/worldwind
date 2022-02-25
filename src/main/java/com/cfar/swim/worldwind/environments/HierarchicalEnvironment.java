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

import java.time.ZonedDateTime;
import java.util.Set;

import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;

/**
 * Describes a hierarchical environment featuring a hierarchy of contained
 * environments.
 * 
 * @author Stephan Heinemann
 *
 */
public interface HierarchicalEnvironment extends StructuredEnvironment {
	
	/**
	 * Determines whether or not this hierarchical environment has children.
	 * 
	 * @return true if this hierarchical environment has children,
	 *         false otherwise
	 */
	public boolean hasChildren();
	
	/**
	 * Gets the children of this hierarchical environment.
	 * 
	 * @return the children of this hierarchical environment
	 * 
	 */
	public Set<? extends HierarchicalEnvironment> getChildren();
	
	/**
	 * Determines whether or not this hierarchical environment has a parent.
	 * 
	 * @return true if this hierarchical environment has a parent,
	 *         false otherwise
	 */
	public boolean hasParent();
	
	/**
	 * Gets the parent of this hierarchical environment if present.
	 * 
	 * @return the parent of this hierarchical environment if present,
	 *         null otherwise
	 */
	public HierarchicalEnvironment getParent();
	
	/**
	 * Gets all hierarchical environments associated with this hierarchical
	 * environment.
	 * 
	 * @return all hierarchical environments associated with this hierarchical
	 *         environment
	 */
	public Set<? extends HierarchicalEnvironment> getAll();
	
	/**
	 * Gets the neighbors of this hierarchical environment.
	 * 
	 * @return the neighbors of this hierarchical environment
	 */
	public Set<? extends HierarchicalEnvironment> getNeighbors();
	
	/**
	 * Determines whether or not this hierarchical environment is a neighbor of
	 * another hierarchical environment.
	 * 
	 * @param neighbor the potential neighbor of this hierarchical environment
	 * 
	 * @return true if this hierarchical environment is a neighbor of the other
	 *         hierarchical environment, false otherwise
	 */
	public boolean areNeighbors(HierarchicalEnvironment neighbor);
	
	/**
	 * Gets the leg cost from the center of this (child) hierarchical
	 * environment to the center of another (child) hierarchical environment
	 * between a start and an end time given a cost policy and risk policy.
	 * 
	 * @param destination the destination hierarchical environment
	 * @param start the start time
	 * @param end the end time
	 * @param costPolicy the cost policy
	 * @param riskPolicy the risk policy
	 * 
	 * @return the leg cost from the center of this hierarchical environment
	 *         to the center of the destination hierarchical environment
	 */
	public double getLegCost(
			HierarchicalEnvironment destination,
			ZonedDateTime start, ZonedDateTime end,
			CostPolicy costPolicy, RiskPolicy riskPolicy);
	
}
