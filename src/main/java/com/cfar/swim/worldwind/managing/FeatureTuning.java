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

import java.util.List;
import java.util.Objects;

import com.cfar.swim.worldwind.registries.Properties;
import com.cfar.swim.worldwind.registries.Specification;

/**
 * Abstracts a feature tuning of a tunable.
 * 
 * @author Stephan Heineamnn
 *
 * @param <Tunable> the tunable
 * 
 * @see AbstractTuning
 */
public abstract class FeatureTuning<Tunable> extends AbstractTuning<Tunable> {
	
	/** the features of this feature tuning */
	private Features features;
	
	/**
	 * Constructs a new feature tuning based on a specification and features.
	 * 
	 * @param specification the specification
	 * @param features the features
	 * 
	 * @throws IllegalArgumentException if the specification or features are
	 *         invalid
	 * 
	 * @see AbstractTuning#AbstractTuning(Specification)
	 */
	public FeatureTuning(
			Specification<Tunable> specification, Features features) {
		super(specification);
		this.setFeatures(features);
	}
	
	/**
	 * Gets the features of this feature tuning.
	 * 
	 * @return the features of this feature tuning
	 */
	public Features getFeatures() {
		return this.features;
	}
	
	/**
	 * Sets the features of this feature tuning.
	 * 
	 * @param features the features to be set
	 * 
	 * @throws IllegalArgumentException if the features are invalid
	 */
	public void setFeatures(Features features) {
		if (null == features) {
			throw new IllegalArgumentException("features are invalid");
		}
		this.features = features;
	}
	
	/**
	 * Tunes the specification of a tunable according to features.
	 * 
	 * @param specification the specification to be tuned.
	 * @param features the features
	 * 
	 * @return the tuned candidate properties for the specification
	 */
	public abstract List<Properties<Tunable>> tune(
			Specification<Tunable> specification, Features features);
	
	/**
	 * Determines whether or not this feature tuning equals another feature
	 * tuning based on their specifications and features.
	 * 
	 * @param o the other feature tuning
	 * 
	 * @return true, if the specification and features of this feature tuning
	 *         equal the specification and features of the other feature
	 *         tuning, respectively, false otherwise
	 * 
	 * @see AbstractTuning#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = super.equals(o);
		
		if (equals) {
			FeatureTuning<?> ft = (FeatureTuning<?>) o;
			equals = this.getFeatures().equals(ft.getFeatures());
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this feature tuning based on its specification and
	 * features.
	 * 
	 * @return the hash code of this feature tuning based on its specification
	 *         and features
	 * 
	 * @see AbstractTuning#hashCode()
	 */
	@Override
	public int hashCode() {
		return Objects.hash(super.hashCode(), this.features.hashCode());
	}
	
}
