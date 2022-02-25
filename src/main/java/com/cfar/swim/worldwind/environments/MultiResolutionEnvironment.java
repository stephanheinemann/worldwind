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

/**
 * Describes a multi-resolution environment featuring multiple resolutions.
 * 
 * @author Stephan Heinemann
 *
 */
public interface MultiResolutionEnvironment extends Environment {
	
	/**
	 * Gets the resolution of this multi-resolution environment.
	 * 
	 * @return the resolution of this multi-resolution environment
	 */
	public double getResolution();
	
	/**
	 * Sets the resolution of this multi-resolution environment.
	 * 
	 * @param resolution the resolution to be set
	 */
	public void setResolution(double resolution);
	
	/**
	 * Refines this multi-resolution environment by a refinement factor.
	 * 
	 * @param factor the refinement factor
	 */
	public void refine(int factor);
	
	/**
	 * Coarsens this multi-resolution environment by a coarsening factor.
	 * 
	 * @param factor the coarsening factor
	 */
	public void coarsen(int factor);
	
	/**
	 * Determines whether or not this multi-resolution environment is refined.
	 * 
	 * @return true if this multi-resolution environment is refined,
	 *         false otherwise
	 */
	public boolean isRefined();
	
}
