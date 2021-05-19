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

/**
 * Describes a multi-resolution environment featuring multiple resolutions.
 * 
 * @author Stephan Heinemann
 *
 */
public interface MultiResolutionEnvironment extends Environment {
	
	/**
	 * Determines whether or not this multi-resolution environment is refined.
	 * 
	 * @return true if this multi-resolution environment is refined,
	 *         false otherwise
	 */
	public boolean isRefined();
	
	/**
	 * Gets the refinements of this multi-resolution environment.
	 * 
	 * @return the refinements of this multi-resolution environment
	 */
	public Set<? extends MultiResolutionEnvironment> getRefinements();
	
	/**
	 * Refines this multi-resolution environment with a refinement density.
	 * 
	 * @param density the refinement density
	 */
	public void refine(int density);
	
	/**
	 * Coarsens this multi-resolution environment.
	 */
	public void coarsen();
	
	/**
	 * Adds a structural change listener to this multi-resolution environment.
	 * 
	 * @param listener the structural change listener to be added
	 */
	public void addStructuralChangeListener(StructuralChangeListener listener);
	
	/**
	 * Removes a structural change listener from this multi-resolution environment.
	 * 
	 * @param listener the structural change listener to be removed
	 */
	public void removeStructuralChangeListener(StructuralChangeListener listener);
	
	/**
	 * Notifies the structural change listeners of this multi-resolution environment.
	 */
	public void notifyStructuralChangeListeners();
	
}
