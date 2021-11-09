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
package com.cfar.swim.worldwind.data;

import java.net.URI;

import com.cfar.swim.worldwind.util.Identifiable;

/**
 * Realizes a SWIM resource.
 * 
 * @author Stephan Heinemann
 *
 */
public class SwimResource implements Identifiable {
	
	/** the resource URI of this SWIM resource */
	private final URI resource;
	
	/** the protocol of this SWIM resource */
	private final SwimProtocol protocol;
	
	/**
	 * Constructs a new SWIM resource.
	 * 
	 * @param resource the resource URI
	 * @param protocol the resource protocol
	 */
	public SwimResource(URI resource, SwimProtocol protocol) {
		this.resource = resource;
		this.protocol = protocol;
	}
	
	/**
	 * Gets the identifier of this SWIM resource.
	 * 
	 * @return the identifier of this SWIM resource
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return this.resource.toString();
	}
	
	/**
	 * Gets the resource of this SWIM resource.
	 * 
	 * @return the resource of this SWIM resource
	 */
	public URI getResource() {
		return this.resource;
	}
	
	/**
	 * Gets the protocol of this SWIM resource.
	 * 
	 * @return the protocol of this SWIM resource
	 */
	public SwimProtocol getProtocol() {
		return this.protocol;
	}
	
}
