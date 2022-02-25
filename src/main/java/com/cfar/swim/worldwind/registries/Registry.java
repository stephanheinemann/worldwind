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
package com.cfar.swim.worldwind.registries;

import java.util.Collections;
import java.util.Optional;
import java.util.Set;
import java.util.TreeSet;

/**
 * Realizes a generic registry of registrees. Registries are used in sessions
 * to register, for example, specifications for available environments and
 * planners. These specifications can then be passed to factories to create
 * instances according to a specification.
 * 
 * @author Stephan Heinemann
 *
 * @param <Registree> the specification registree
 * 
 * @see Specification
 * @see Factory
 */
public class Registry<Registree> {
	
	/** the specifications of this registry */
	Set<Specification<Registree>> specifications = new TreeSet<>();
	
	/**
	 * Gets a specification of this registry.
	 * 
	 * @param id the identifier of the specification
	 * @return the identified specification, or otherwise null
	 */
	public Specification<Registree> getSpecification(String id) {
		Specification<Registree> specification = null;
		Optional<Specification<Registree>> optSpecification = specifications.stream().filter(s -> s.getId().equals(id)).findFirst();
		
		if (optSpecification.isPresent()) {
			specification = optSpecification.get();
		}
		
		return specification;
	}
	
	/**
	 * Gets the specifications of this registry.
	 * 
	 * @return the specifications of this registry
	 */
	public Set<Specification<Registree>> getSpecifications() {
		return Collections.unmodifiableSet(this.specifications);
	}
	
	/**
	 * Adds a specification to this registry.
	 * 
	 * @param specification the specification to be added
	 */
	public void addSpecification(Specification<Registree> specification) {
		this.specifications.add(specification);
	}
	
	/**
	 * Removes a specification from this registry.
	 * 
	 * @param specification the specification to be removed
	 */
	public void removeSpecification(Specification<Registree> specification) {
		this.specifications.remove(specification);
	}
	
	/**
	 * Clears the specifications of this registry.
	 */
	public void clearSpecifications() {
		this.specifications.clear();
	}

}
