/**
 * Copyright (c) 2016, Stephan Heinemann (UVic Center for Aerospace Research)
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
package com.cfar.swim.worldwind.registries.connections;

import java.util.Objects;

/**
 * Realizes the properties bean of a dronekit datalink.
 * 
 * @author Stephan Heinemann
 *
 */
public class DronekitDatalinkProperties extends DatalinkProperties {
	
	/** the standard localhost of a dronekit datalink properties bean */
	public static final String DATALINK_LOCALHOST = "localhost";
	
	/** the standard gRPC port of a dronekit datalink properties bean */
	public static final int DATALINK_GRPCPORT = 50051;
	
	/** the remote host of this dronekit datalink properties bean */
	private String host;
	
	/** the remote port of this dronekit datalink properties bean */
	private int port;
	
	/**
	 * Constructs a new dronekit datalink properties bean.
	 */
	public DronekitDatalinkProperties() {
		this.host = DronekitDatalinkProperties.DATALINK_LOCALHOST;
		this.port = DronekitDatalinkProperties.DATALINK_GRPCPORT;
	}
	
	/**
	 * Gets the host of this dronekit datalink properties bean.
	 * 
	 * @return the host of this dronekit datalink properties bean
	 */
	public String getHost() {
		return this.host;
	}
	
	/**
	 * Sets the host of this dronekit datalink properties bean.
	 * 
	 * @param host the host to be set
	 */
	public void setHost(String host) {
		this.host = host;
	}
	
	/**
	 * Gets the port of this dronekit datalink properties bean.
	 * 
	 * @return the port of this dronekit datalink properties bean
	 */
	public int getPort() {
		return this.port;
	}
	
	/**
	 * Sets the port of this dronekit datalink properties bean.
	 * 
	 * @param port the port to be set
	 */
	public void setPort(int port) {
		this.port = port;
	}
	
	/**
	 * Determines whether or not this dronekit datalink properties bean equals
	 * another dronekit datalink properties bean based on their aggregated
	 * properties.
	 * 
	 * @param o the other dronekit datalink properties bean
	 * 
	 * @return true, if the aggregated properties of this dronekit datalink
	 *         properties bean equal the aggregated properties of the other
	 *         dronekit datalink properties bean, false otherwise
	 * 
	 * @see DatalinkProperties#equals(Object)
	 */
	@Override
	public final boolean equals(Object o) {
		boolean equals = super.equals(o);
		
		if (equals && (o instanceof DronekitDatalinkProperties)) {
			DronekitDatalinkProperties ddlp = (DronekitDatalinkProperties) o;
			equals = (this.host.equals(ddlp.host) && (this.port == ddlp.port));
		}
			
		return equals;
	}
	
	/**
	 * Gets the hash code of this dronekit datalink properties bean based on
	 * its aggregated properties.
	 * 
	 * @return the hash code of this dronekit datalink properties bean based on
	 *         its aggregated properties
	 * 
	 * @see DatalinkProperties#hashCode()
	 */
	@Override
	public final int hashCode() {
		return Objects.hash(super.hashCode(), this.host, this.port);
	}
	
}
