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

import java.util.Objects;

import com.cfar.swim.worldwind.registries.Specification;

/**
 * Abstracts a tuning of a tunable.
 * 
 * @author Stephan Heinemann
 *
 * @param <Tunable> the tunable
 */
public abstract class AbstractTuning<Tunable> implements Tuning<Tunable> {
	
	/** the default serial identification of this abstract tuning */
	private static final long serialVersionUID = 1L;
	
	/** the specification of the tunable of this feature tuning */
	private Specification<Tunable> specification;
	
	/**
	 * Constructs a new abstract tuning based on a specification of the
	 * tunable.
	 * 
	 * @param specification the specification of the tunable
	 * 
	 * @throws IllegalArgumentException if the specification is invalid
	 */
	public AbstractTuning(Specification<Tunable> specification) {
		this.setSpecification(specification);
	}
	
	/**
	 * Gets the specification of the tunable of this abstract tuning.
	 * 
	 * @return the specification of the tunable of this abstract tuning
	 * 
	 * @see Tuning#getSpecification()
	 */
	@Override
	public Specification<Tunable> getSpecification() {
		return this.specification;
	}
	
	/**
	 * Sets the specification of the tunable of this abstract tuning.
	 * 
	 * @param specification the specification to be set
	 * 
	 * @throws IllegalArgumentException if the specification is invalid
	 *
	 * @see Tuning#setSpecification(Specification)
	 */
	@Override
	public void setSpecification(Specification<Tunable> specification) {
		if (null == specification) {
			throw new IllegalArgumentException("specification is invalid");
		}
		this.specification = specification;
	}
	
	/**
	 * Determines whether or not this abstract tuning equals another abstract
	 * tuning based on their specifications.
	 * 
	 * @param o the other abstract tuning
	 * 
	 * @return true, if the specification of this abstract tuning equals the
	 *         specification of the other abstract tuning, false otherwise
	 * 
	 * @see Object#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = false;
		
		if (this == o) {
			equals = true;
		} else if ((null != o) && (this.getClass() == o.getClass())) {
			AbstractTuning<?> at = (AbstractTuning<?>) o;
			equals = this.getSpecification().equals(at.getSpecification());
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this abstract tuning based on its specification.
	 * 
	 * @return the hash code of this abstract tuning based on its specification
	 * 
	 * @see Object#hashCode()
	 */
	@Override
	public int hashCode() {
		return Objects.hash(this.specification);
	}
	
}
