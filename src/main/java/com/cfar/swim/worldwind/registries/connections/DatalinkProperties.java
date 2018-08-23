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

import com.cfar.swim.worldwind.connections.Datalink;
import com.cfar.swim.worldwind.registries.Properties;


/**
 * Abstracts datalink properties applicable to all datalinks.
 * 
 * @author Stephan Heinemann
 *
 */
public abstract class DatalinkProperties implements Properties<Datalink> {
	
	/**	the description of this datalink properties bean */
	private String description = "Description not yet available.";
	
	/** the downlink period of this datalink properties bean */
	private long downlinkPeriod = 1000; // ms
	
	/**
	 * Gets the downlink (monitoring) period of this datalink properties bean.
	 * 
	 * @return the downlink period of this datalink properties bean in milliseconds
	 */
	public long getDownlinkPeriod() {
		return this.downlinkPeriod;
	}
	
	/**
	 * Sets the downlink (monitoring) period of this datalink properties bean.
	 * 
	 * @param downlinkPeriod the downlink period to be set in milliseconds
	 */
	public void setDownlinkPeriod(long downlinkPeriod) {
		this.downlinkPeriod = downlinkPeriod;
	}
	
	/**
	 * Gets the description of this datalink properties bean.
	 * 
	 * @return the description of this datalink properties bean
	 */
	public String getDescription() {
		return description;
	}

	/**
	 * Sets the description of this datalink properties bean.
	 * 
	 * @param description the description of this datalink properties bean
	 */
	public void setDescription(String description) {
		this.description = description;
	}

	/**
	 * Clones this datalink properties bean.
	 * 
	 * @return a clone of this datalink properties bean
	 * 
	 * @see Properties#clone()
	 */
	@Override
	public DatalinkProperties clone() {
		DatalinkProperties clone = null;
		try {
			clone = (DatalinkProperties) super.clone();
		} catch (CloneNotSupportedException e) {
			e.printStackTrace();
		}
		return clone;
	}

}
