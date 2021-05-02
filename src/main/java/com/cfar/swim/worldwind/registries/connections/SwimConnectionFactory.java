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

import com.cfar.swim.worldwind.connections.LiveSwimConnection;
import com.cfar.swim.worldwind.connections.SimulatedSwimConnection;
import com.cfar.swim.worldwind.connections.SwimConnection;
import com.cfar.swim.worldwind.data.SwimData;
import com.cfar.swim.worldwind.registries.AbstractFactory;
import com.cfar.swim.worldwind.registries.Specification;

/**
 * Realizes a SWIM connection factory to create SWIM connections according to
 * customized SWIM connection specifications.
 * 
 * @author Stephan Heinemann
 * 
 * @see AbstractFactory
 * @see Specification
 */
public class SwimConnectionFactory extends AbstractFactory<SwimConnection> {
	
	/**
	 * Constructs a new SWIM connection factory without a customized
	 * specification.
	 * 
	 * @see AbstractFactory
	 */
	public SwimConnectionFactory() {
		super();
	}
	
	/**
	 * Constructs a new SWIM connection factory to create registered SWIM
	 * connections according to a customized SWIM connection specification.
	 * 
	 * @param specification the SWIM connection specification describing the
	 *                      registered SWIM connection
	 * 
	 * @see AbstractFactory
	 */
	public SwimConnectionFactory(Specification<SwimConnection> specification) {
		super(specification);
	}

	/**
	 * Creates a new SWIM connection according to the customized SWIM
	 * connection specification of this SWIM connection factory.
	 * 
	 * @return the created SWIM connection
	 * 
	 * @see AbstractFactory#createInstance()
	 */
	@Override
	public SwimConnection createInstance() {
		SwimConnection connection = null;
		
		if (this.hasSpecification()) {
			if (this.specification.getId().equals(Specification.CONNECTION_SWIM_SIMULATED_ID)) {
				SimulatedSwimConnectionProperties properties = (SimulatedSwimConnectionProperties) this.specification.getProperties();
				connection = new SimulatedSwimConnection(
						properties.getResourceDirectory(),
						properties.getUpdatePeriod(),
						properties.getUpdateProbability(),
						properties.getUpdateQuantity());
				if (properties.getSubscribesAIXM())
					connection.subscribe(SwimData.AIXM);
				if (properties.getSubscribesFIXM())
					connection.subscribe(SwimData.FIXM);
				if (properties.getSubscribesWXXM())
					connection.subscribe(SwimData.WXXM);
				if (properties.getSubscribesIWXXM())
					connection.subscribe(SwimData.IWXXM);
				if (properties.getSubscribesAMXM())
					connection.subscribe(SwimData.AMXM);
				connection.setAutoCommit(properties.getAutoCommit());
			} else if (this.specification.getId().equals(Specification.CONNECTION_SWIM_LIVE_ID)) {
				LiveSwimConnectionProperties properties = (LiveSwimConnectionProperties) this.specification.getProperties();
				connection = new LiveSwimConnection();
				// TODO: set properties for live SWIM connection
				if (properties.getSubscribesAIXM())
					connection.subscribe(SwimData.AIXM);
				if (properties.getSubscribesFIXM())
					connection.subscribe(SwimData.FIXM);
				if (properties.getSubscribesWXXM())
					connection.subscribe(SwimData.WXXM);
				if (properties.getSubscribesIWXXM())
					connection.subscribe(SwimData.IWXXM);
				if (properties.getSubscribesAMXM())
					connection.subscribe(SwimData.AMXM);
				connection.setAutoCommit(properties.getAutoCommit());
			}
		}
		
		return connection;
	}

}
