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

import com.cfar.swim.worldwind.connections.SwimConnection;
import com.cfar.swim.worldwind.registries.Properties;

/**
 * Abstracts SWIM connection properties applicable to all SWIM connections.
 * 
 * @author Stephan Heinemann
 *
 */
public abstract class SwimConnectionProperties implements Properties<SwimConnection> {
	
	/** the default serial identification of this SWIM connection properties bean */
	private static final long serialVersionUID = 1L;
	
	/** the AIXM subscription status of this SWIM connection properties bean */
	private boolean subscribesAIXM = false;
	
	/** the FIXM subscription status of this SWIM connection properties bean */
	private boolean subscribesFIXM = false; 
	
	/** the WXXM subscription status of this SWIM connection properties bean */
	private boolean subscribesWXXM = false;
	
	/** the IWXXM subscription status of this SWIM connection properties bean */
	private boolean subscribesIWXXM = false;
	
	/** the AMXM subscription status of this SWIM connection properties bean */
	private boolean subscribesAMXM = false;
	
	/**
	 * Gets the AIXM subscription status of this SWIM connection properties bean.
	 * 
	 * @return the AIXM subscription status of this SWIM connection properties bean
	 */
	public boolean getSubscribesAIXM() {
		return this.subscribesAIXM;
	}
	
	/**
	 * Sets the AIXM subscription status of this SWIM connection properties bean.
	 * 
	 * @param subscribesAIXM the AIXM subscription status to be set
	 */
	public void setSubscribesAIXM(boolean subscribesAIXM) {
		this.subscribesAIXM = subscribesAIXM;
	}
	
	/**
	 * Gets the FIXM subscription status of this SWIM connection properties bean.
	 * 
	 * @return the FIXM subscription status of this SWIM connection properties bean
	 */
	public boolean getSubscribesFIXM() {
		return this.subscribesFIXM;
	}
	
	/**
	 * Sets the FIXM subscription status of this SWIM connection properties bean.
	 * 
	 * @param subscribesFIXM the FIXM subscription status to be set
	 */
	public void setSubscribesFIXM(boolean subscribesFIXM) {
		this.subscribesFIXM = subscribesFIXM;
	}
	
	/**
	 * Gets the WXXM subscription status of this SWIM connection properties bean.
	 * 
	 * @return the WXXM subscription status of this SWIM connection properties bean
	 */
	public boolean getSubscribesWXXM() {
		return this.subscribesWXXM;
	}
	
	/**
	 * Sets the WXXM subscription status of this SWIM connection properties bean.
	 * 
	 * @param subscribesWXXM the WXXM subscription status to be set
	 */
	public void setSubscribesWXXM(boolean subscribesWXXM) {
		this.subscribesWXXM = subscribesWXXM;
	}
	
	/**
	 * Gets the IWXXM subscription status of this SWIM connection properties bean.
	 * 
	 * @return the IWXXM subscription status of this SWIM connection properties bean
	 */
	public boolean getSubscribesIWXXM() {
		return this.subscribesIWXXM;
	}
	
	/**
	 * Sets the IWXXM subscription status of this SWIM connection properties bean.
	 * 
	 * @param subscribesIWXXM the IWXXM subscription status to be set
	 */
	public void setSubscribesIWXXM(boolean subscribesIWXXM) {
		this.subscribesIWXXM = subscribesIWXXM;
	}
	
	/**
	 * Gets the AMXM subscription status of this SWIM connection properties bean.
	 * 
	 * @return the AMXM subscription status of this SWIM connection properties bean
	 */
	public boolean getSubscribesAMXM() {
		return this.subscribesAMXM;
	}
	
	/**
	 * Sets the AMXM subscription status of this SWIM connection properties bean.
	 * 
	 * @param subscribesAMXM the AMXM subscription status to be set
	 */
	public void setSubscribesAMXM(boolean subscribesAMXM) {
		this.subscribesAMXM = subscribesAMXM;
	}
	
	/**
	 * Clones this SWIM connection properties bean.
	 * 
	 * @return a clone of this SWIM connection properties bean
	 * 
	 * @see Properties#clone()
	 */
	@Override
	public SwimConnectionProperties clone() {
		SwimConnectionProperties clone = null;
		try {
			clone = (SwimConnectionProperties) super.clone();
		} catch (CloneNotSupportedException e) {
			e.printStackTrace();
		}
		return clone;
	}
	
	/**
	 * Determines whether or not this SWIM connection properties bean equals
	 * another SWIM connection properties bean based on their aggregated
	 * properties.
	 * 
	 * @param o the other SWIM connection properties bean
	 * 
	 * @return true, if the aggregated properties of this SWIM connection
	 *         properties bean equal the aggregated properties of the other
	 *         SWIM connection properties bean, false otherwise
	 * 
	 * @see Object#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = false;
		
		if (this == o) {
			equals = true;
		} else if ((null != o) && (this.getClass() == o.getClass())) {
			SwimConnectionProperties scp = (SwimConnectionProperties) o;
			equals = (this.subscribesAIXM == scp.subscribesAIXM)
					&& (this.subscribesAMXM == scp.subscribesAMXM)
					&& (this.subscribesFIXM == scp.subscribesFIXM)
					&& (this.subscribesIWXXM == scp.subscribesIWXXM)
					&& (this.subscribesWXXM == scp.subscribesWXXM);
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this SWIM connection properties bean based on its
	 * aggregated properties.
	 * 
	 * @return the hash code of this SWIM connection properties bean based on
	 *         its aggregated properties
	 * 
	 * @see Object#hashCode()
	 */
	@Override
	public int hashCode() {
		return Objects.hash(
				this.subscribesAIXM,
				this.subscribesAMXM,
				this.subscribesFIXM,
				this.subscribesIWXXM,
				this.subscribesWXXM);
	}
	
}
