package com.cfar.swim.worldwind.registries.connections;

import com.cfar.swim.worldwind.connections.AircraftConnection;
import com.cfar.swim.worldwind.connections.DronekitConnection;
import com.cfar.swim.worldwind.connections.SimulatedAircraftConnection;
import com.cfar.swim.worldwind.registries.Factory;
import com.cfar.swim.worldwind.registries.Specification;

public class AircraftConnectionFactory implements Factory<AircraftConnection> {

	@Override
	public AircraftConnection createInstance(Specification<AircraftConnection> specification) {
		AircraftConnection connection = null;
		
		if (specification.getId().equals(Specification.CONNECTION_AIRCRAFT_SIMULATED)) {
			connection = new SimulatedAircraftConnection();
		} else if (specification.getId().equals(Specification.CONNECTION_AIRCRAFT_DRONEKIT)) {			
			DronekitConnectionProperties properties = (DronekitConnectionProperties) specification.getProperties();
			connection = new DronekitConnection(properties.getHost(), properties.getPort());
		}
		
		return connection;
	}

}
