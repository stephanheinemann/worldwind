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
package com.cfar.swim.worldwind.ai;

/**
 * Describes an anytime planner which issues plans featuring a specified
 * minimum quality or better. An anytime planner continuously attempts to
 * revise a previously issued plan by incrementally improving its quality
 * until a maximum quality or better has been achieved.
 * 
 * @author Stephan Heinemann
 * 
 */
public interface AnytimePlanner extends Planner {
	
	/**
	 * Gets the minimum required quality of a plan issued by this anytime
	 * planner.
	 * 
	 * @return the minimum required quality of a plan issued by this anytime
	 *         planner
	 */
	public double getMinimumQuality();
	
	/**
	 * Sets the minimum required quality of a plan issued by this anytime
	 * planner.
	 * 
	 * @param minQuality the minimum required quality of a plan issued by this
	 *                   anytime planner
	 */
	public void setMinimumQuality(double minQuality);
	
	/**
	 * Gets the maximum required quality of a plan issued by this anytime
	 * planner.
	 * 
	 * @return the maximum required quality of a plan issued by this anytime
	 *         planner
	 */
	public double getMaximumQuality();
	
	/**
	 * Sets the maximum required quality of a plan issued by this anytime
	 * planner.
	 * 
	 * @param maxQuality the maximum required quality of a plan issued by this
	 *                   anytime planner
	 */
	public void setMaximumQuality(double maxQuality);
	
	/**
	 * Gets the required quality improvement between consecutively issued plans
	 * employed by this anytime planner.
	 * 
	 * @return the required quality improvement employed by this anytime
	 *         planner
	 */
	public double getQualityImprovement();
	
	/**
	 * Sets the required quality improvement between consecutively issued plans
	 * employed by this anytime planner.
	 * 
	 * @param improvement the required quality improvement employed by this
	 *                    anytime planner
	 */
	public void setQualityImprovement(double improvement);
	
}
