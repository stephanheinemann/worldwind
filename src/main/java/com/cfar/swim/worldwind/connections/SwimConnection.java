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
package com.cfar.swim.worldwind.connections;

import java.util.HashSet;

import com.cfar.swim.worldwind.data.SwimData;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.connections.SwimConnectionProperties;
import com.cfar.swim.worldwind.session.ObstacleManager;
import com.cfar.swim.worldwind.session.ObstacleProvider;

/**
 * Abstracts a SWIM connection.
 * 
 * @author Stephan Heinemann
 *
 */
public abstract class SwimConnection implements Connection, ObstacleProvider {
	
	/** the SWIM subscriptions of this SWIM connection */
	private HashSet<SwimData> subscriptions = new HashSet<SwimData>();
	
	/** this obstacle manager of this SWIM connection */
	private ObstacleManager obstacleManager = null;
	
	/** indicates whether or not this SWIM connection automatically commits obstacle changes */
	private boolean autoCommit = false;
	
	/**
	 * Connects this SWIM connection.
	 */
	@Override
	public abstract void connect();
	
	/**
	 * Disconnects this SWIM connection.
	 */
	@Override
	public abstract void disconnect();
	
	/**
	 * Determines whether or not this SWIM connection is connected.
	 * 
	 * 
	 * @return true if this SWIM connection is connected, false otherwise
	 */
	@Override
	public abstract boolean isConnected();
	
	/**
	 * Subscribes this connection to a specified SWIM data protocol.
	 * 
	 * @param protocol the SWIM data protocol to be subscribed to
	 */
	public synchronized void subscribe(SwimData protocol) {
		this.subscriptions.add(protocol);
	}
	
	/**
	 * Unsubscribes this connection from a specified SWIM data protocol.
	 * 
	 * @param protocol the SWIM data protocol to be unsubscribed from
	 */
	public synchronized void unsubscribe(SwimData protocol) {
		this.subscriptions.remove(protocol);
	}
	
	/**
	 * Determines whether or not this SWIM connection has subscribed to a
	 * specified SWIM data protocol.
	 * 
	 * @param protocol the SWIM data protocol

	 * @return true if this SWIM connection has subscribed to the specified
	 *         SWIM data protocol
	 */
	public synchronized boolean hasSubscribed(SwimData protocol) {
		return this.subscriptions.contains(protocol);
	}
	
	/**
	 * Gets the obstacle manager of this SWIM connection.
	 * 
	 * @return the obstacle manager of this SWIM connection
	 * 
	 * @see ObstacleProvider#getObstacleManager()
	 */
	@Override
	public synchronized ObstacleManager getObstacleManager() {
		return this.obstacleManager;
	}
	
	/**
	 * Sets the obstacle manager of this SWIM connection.
	 * 
	 * @param obstacleManager the obstacle manager to be set
	 * 
	 * @see ObstacleProvider#setObstacleManager(ObstacleManager)
	 */
	@Override
	public synchronized void setObstacleManager(ObstacleManager obstacleManager) {
		this.obstacleManager = obstacleManager;
	}
	
	/**
	 * Determines whether or not this SWIM connection has an obstacle manager.
	 * 
	 * @return true if this SWIM connection has an obstacle manager,
	 *         false otherwise
	 * 
	 * @see ObstacleProvider#hasObstacleManager()
	 */
	@Override
	public synchronized boolean hasObstacleManager() {
		return (null != this.obstacleManager);
	}
	/**
	 * Gets whether or not this SWIM connection automatically commits obstacle
	 * changes.
	 * 
	 * @return true if this SWIM connection automatically commits obstacle
	 *         changes, false otherwise
	 * 
	 * @see ObstacleProvider#getAutoCommit()
	 */
	@Override
	public synchronized boolean getAutoCommit() {
		return this.autoCommit;
	}
	
	/**
	 * Sets whether or not this SWIM connection automatically commits obstacle
	 * changes.
	 * 
	 * @param autoCommit true if this SWIM connection automatically commits
	 *                   obstacle changes, false otherwise
	 * 
	 * @see ObstacleProvider#setAutoCommit(boolean)
	 */
	@Override
	public synchronized void setAutoCommit(boolean autoCommit) {
		this.autoCommit = autoCommit;
	}
	
	/**
	 * Determines whether or not this SWIM connection matches a specification.
	 * 
	 * @param specification the specification to be matched
	 * 
	 * @return true if the this SWIM connection matches the specification,
	 *         false otherwise
	 * 
	 * @see Connection#matches(Specification)
	 */
	@Override
	public boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = false;
		
		if ((null != specification) && (specification.getProperties() instanceof SwimConnectionProperties)) {
			SwimConnectionProperties scp = (SwimConnectionProperties) specification.getProperties();
			matches = (this.hasSubscribed(SwimData.AIXM) == scp.getSubscribesAIXM()
					&& (this.hasSubscribed(SwimData.AMXM) == scp.getSubscribesAMXM())
					&& (this.hasSubscribed(SwimData.FIXM) == scp.getSubscribesFIXM())
					&& (this.hasSubscribed(SwimData.IWXXM) == scp.getSubscribesIWXXM())
					&& (this.hasSubscribed(SwimData.WXXM) == scp.getSubscribesWXXM())
					&& (this.autoCommit == scp.getAutoCommit()));
		}
		
		return matches;
	}
	
}
