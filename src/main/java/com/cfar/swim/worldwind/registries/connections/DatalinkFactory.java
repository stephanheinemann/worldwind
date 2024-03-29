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

import java.time.Duration;

import com.cfar.swim.worldwind.connections.Datalink;
import com.cfar.swim.worldwind.connections.DronekitDatalink;
import com.cfar.swim.worldwind.connections.MavlinkDatalink;
import com.cfar.swim.worldwind.connections.SimulatedDatalink;
import com.cfar.swim.worldwind.registries.AbstractFactory;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.tracks.AircraftTrackError;

import gov.nasa.worldwind.geom.Angle;


/**
 * Realizes a datalink factory to create datalinks according to
 * customized datalink specifications.
 * 
 * @author Stephan Heinemann
 * 
 * @see AbstractFactory
 * @see Specification
 */
public class DatalinkFactory extends AbstractFactory<Datalink> {
	
	/**
	 * Constructs a new datalink factory without a customized specification.
	 * 
	 * @see AbstractFactory
	 */
	public DatalinkFactory() {
		super();
	}
	
	/**
	 * Constructs a new datalink factory to create registered datalinks
	 * according to a customized datalink specification.
	 * 
	 * @param specification the datalink specification describing the
	 *                      registered datalink
	 * 
	 * @see AbstractFactory
	 */
	public DatalinkFactory(Specification<Datalink> specification) {
		super(specification);
	}

	/**
	 * Creates a new datalink according to the customized datalink
	 * specification of this datalink factory.
	 * 
	 * @return the created datalink, or null if no datalink could be created
	 * 
	 * @see AbstractFactory#createInstance()
	 */
	@Override
	public Datalink createInstance() {
		Datalink connection = null;
		
		if (this.getSpecification().getId().equals(Specification.CONNECTION_DATALINK_SIMULATED_ID)) {
			SimulatedDatalinkProperties properties = (SimulatedDatalinkProperties) this.getSpecification().getProperties();
			connection = new SimulatedDatalink();
			connection.setDownlinkPeriod(Duration.ofSeconds(properties.getDownlinkPeriod()));
			((SimulatedDatalink) connection).setUplinkDelay(Duration.ofMillis(properties.getUplinkDelay()));
			AircraftTrackError maxTrackError = new AircraftTrackError();
			maxTrackError.setCrossTrackError(properties.getMaxCrossTrackError());
			maxTrackError.setTimingError(Duration.ofSeconds(properties.getMaxTimingError()));
			maxTrackError.setOpeningBearingError(Angle.POS90);
			maxTrackError.setClosingBearingError(Angle.POS90);
			((SimulatedDatalink) connection).setMaxTrackError(maxTrackError);
			((SimulatedDatalink) connection).setErrorProbablity(properties.getErrorProbability());
		} else if (this.getSpecification().getId().equals(Specification.CONNECTION_DATALINK_DRONEKIT_ID)) {
			DronekitDatalinkProperties properties = (DronekitDatalinkProperties) this.getSpecification().getProperties();
			connection = new DronekitDatalink(properties.getHost(), properties.getPort());
			connection.setDownlinkPeriod(Duration.ofSeconds(properties.getDownlinkPeriod()));
		} else if (this.getSpecification().getId().equals(Specification.CONNECTION_DATALINK_MAVLINK_ID)) {
			MavlinkDatalinkProperties properties = (MavlinkDatalinkProperties) this.getSpecification().getProperties();
			connection = new MavlinkDatalink(properties.getHost(), properties.getPort(),
					properties.getSourceId(), properties.getTargetId());
			connection.setDownlinkPeriod(Duration.ofSeconds(properties.getDownlinkPeriod()));
		}
		
		return connection;
	}

}
