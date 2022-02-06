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
package com.cfar.swim.worldwind.registries.connections;

import java.util.Objects;

import javax.validation.constraints.Max;
import javax.validation.constraints.Min;
import javax.validation.constraints.NotEmpty;
import javax.validation.constraints.NotNull;

/**
 * Realizes the properties bean of a mavlink datalink.
 * 
 * @author Stephan Heinemann
 *
 */
public class MavlinkDatalinkProperties extends DatalinkProperties {

	/** the default serial identification of this mavlink datalink properties bean */
	private static final long serialVersionUID = 1L;
	
	/** the default message scan limit of this mavlink datalink properties bean*/
	public static final int MAVLINK_DEFAULT_SCAN_LIMIT = 1000;
	
	/** the default host of this mavlink datalink properties bean */
	public static final String MAVLINK_DEFAULT_HOST = "172.16.104.128";
	
	/** the default port of this mavlink datalink properties bean */
	public static final int MAVLINK_DEFAULT_PORT = 14550;
	
	/** the default source identifier of this mavlink datalink properties bean */
	public static final int MAVLINK_DEFAULT_SOURCE_ID = 254;
	
	/** the default target identifier of this mavlink datalink properties bean */
	public static final int MAVLINK_DEFAULT_TARGET_ID = 1;
	
	/** the host of this mavlink datalink properties bean */
	@NotNull(message = "{property.connection.datalink.mavlink.host.null}")
	@NotEmpty(message = "{property.connection.datalink.mavlink.host.empty}")
	private String host;
	
	/** the port of this mavlink datalink properties bean */
	@Min(value = 1024, message = "{property.connection.datalink.mavlink.port.min}")
	@Max(value = 65535, message = "{property.connection.datalink.mavlink.port.max}")
	private int port;
	
	/** the source identifier of this mavlink datalink properties bean */
	@Min(value = 0, message = "{property.connection.datalink.mavlink.sourceId.min}")
	@Max(value = 255, message = "{property.connection.datalink.mavlink.sourceId.max}")
	private int sourceId;
	
	/** the target identifier of this mavlink datalink properties bean */
	@Min(value = 0, message = "{property.connection.datalink.mavlink.targetId.min}")
	@Max(value = 255, message = "{property.connection.datalink.mavlink.targetId.max}")
	private int targetId;
	
	
	/**
	 * Constructs a new mavlink datalink properties bean.
	 */
	public MavlinkDatalinkProperties() {
		this.host = MavlinkDatalinkProperties.MAVLINK_DEFAULT_HOST;
		this.port = MavlinkDatalinkProperties.MAVLINK_DEFAULT_PORT;
		this.sourceId = MavlinkDatalinkProperties.MAVLINK_DEFAULT_SOURCE_ID;
		this.targetId = MavlinkDatalinkProperties.MAVLINK_DEFAULT_TARGET_ID;
	}
	
	/**
	 * Gets the host of this mavlink datalink properties bean.
	 * 
	 * @return the host of this mavlink datalink properties bean
	 */
	public String getHost() {
		return this.host;
	}
	
	/**
	 * Sets the host of this mavlink datalink properties bean.
	 * 
	 * @param host the host to be set
	 */
	public void setHost(String host) {
		this.host = host;
	}
	
	/**
	 * Gets the port of this mavlink datalink properties bean.
	 * 
	 * @return the port of this mavlink datalink properties bean
	 */
	public int getPort() {
		return this.port;
	}
	
	/**
	 * Sets the port of this mavlink datalink properties bean.
	 * 
	 * @param port the port to be set
	 */
	public void setPort(int port) {
		this.port = port;
	}
	
	/**
	 * Gets the source identifier of this mavlink datalink properties bean.
	 * 
	 * @return the source identifier of this mavlink datalink properties bean
	 */
	public int getSourceId() {
		return this.sourceId;
	}
	
	/**
	 * Sets the source identifier of this mavlink datalink properties bean.
	 * 
	 * @param sourceId the source identifier to be set
	 */
	public void setSourceId(int sourceId) {
		this.sourceId = sourceId;
	}
	
	/**
	 * Gets the target identifier of this mavlink datalink properties bean.
	 * 
	 * @return the target identifier of this mavlink datalink properties bean
	 */
	public int getTargetId() {
		return this.targetId;
	}
	
	/**
	 * Sets the target identifier of this mavlink datalink properties bean.
	 * 
	 * @param targetId the target identifier to be set
	 */
	public void setTargetId(int targetId) {
		this.targetId = targetId;
	}
	
	/**
	 * Determines whether or not this mavlink datalink properties bean equals
	 * another mavlink datalink properties bean based on their aggregated
	 * properties.
	 * 
	 * @param o the other mavlink datalink properties bean
	 * 
	 * @return true, if the aggregated properties of this mavlink datalink
	 *         properties bean equal the aggregated properties of the other
	 *         mavlink datalink properties bean, false otherwise
	 * 
	 * @see DatalinkProperties#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = super.equals(o);
		
		if (equals) {
			MavlinkDatalinkProperties mdlp = (MavlinkDatalinkProperties) o;
			equals = this.host.equals(mdlp.host)
					&& (this.port == mdlp.port)
					&& (this.sourceId == mdlp.sourceId)
					&& (this.targetId == mdlp.targetId);
		}
			
		return equals;
	}
	
	/**
	 * Gets the hash code of this mavlink datalink properties bean based on
	 * its aggregated properties.
	 * 
	 * @return the hash code of this mavlink datalink properties bean based on
	 *         its aggregated properties
	 * 
	 * @see DatalinkProperties#hashCode()
	 */
	@Override
	public final int hashCode() {
		return Objects.hash(super.hashCode(),
				this.host, this.port, this.sourceId, this.targetId);
	}
	
}
