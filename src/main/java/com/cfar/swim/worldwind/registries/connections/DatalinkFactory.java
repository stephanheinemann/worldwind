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
import com.cfar.swim.worldwind.connections.DronekitDatalink;
import com.cfar.swim.worldwind.connections.SimulatedDatalink;
import com.cfar.swim.worldwind.registries.Factory;
import com.cfar.swim.worldwind.registries.Specification;


/**
 * Realizes a datalink factory to create datalinks according to
 * customized datalink specifications.
 * 
 * @author Stephan Heinemann
 * 
 * @see Factory
 * @see Specification
 */
public class DatalinkFactory implements Factory<Datalink> {
	
	/**
	 * Creates a new datalink according to a customized datalink specification.
	 * 
	 * @param specification the customized datalink specification
	 * 
	 * @return the created datalink
	 * 
	 * @see Factory#createInstance(Specification)
	 */
	@Override
	public Datalink createInstance(Specification<Datalink> specification) {
		Datalink connection = null;
		
		if (specification.getId().equals(Specification.DATALINK_SIMULATED)) {
			SimulatedDatalinkProperties properties = (SimulatedDatalinkProperties) specification.getProperties();
			connection = new SimulatedDatalink();
			connection.setDownlinkPeriod(properties.getDownlinkPeriod());
		} else if (specification.getId().equals(Specification.DATALINK_DRONEKIT)) {			
			DronekitDatalinkProperties properties = (DronekitDatalinkProperties) specification.getProperties();
			connection = new DronekitDatalink(properties.getHost(), properties.getPort());
			connection.setDownlinkPeriod(properties.getDownlinkPeriod());
		}
		
		return connection;
	}

}
