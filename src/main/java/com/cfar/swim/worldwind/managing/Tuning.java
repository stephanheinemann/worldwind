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
package com.cfar.swim.worldwind.managing;

import java.io.Serializable;
import java.util.List;

import com.cfar.swim.worldwind.registries.Properties;
import com.cfar.swim.worldwind.registries.Specification;

/**
 * Describes a tuning of a tunable.
 * 
 * @author Stephan Heinemann
 * 
 * @param <Tunable> the tunable
 */
public interface Tuning<Tunable> extends Serializable {
	
	/**
	 * Gets the specification of the tunable of this tuning.
	 * 
	 * @return the specification of the tunable of this tuning
	 */
	public Specification<Tunable> getSpecification();
	
	/**
	 * Sets the specification of the tunable of this tuning.
	 * 
	 * @param specification the specification to be set
	 */
	public void setSpecification(Specification<Tunable> specification);
	
	/**
	 * Tunes the specification of the tunable of this tuning.
	 * 
	 * @return the tuned candidate properties for the specification
	 */
	public List<Properties<Tunable>> tune();
	
	/**
	 * Tunes the specification of a tunable.
	 * 
	 * @param specification the specification to be tuned.
	 * 
	 * @return the tuned candidate properties for the specification
	 */
	public List<Properties<Tunable>> tune(Specification<Tunable> specification);
	
}
