package com.cfar.swim.worldwind.registries.connections;

import com.cfar.swim.worldwind.connections.AircraftConnection;
import com.cfar.swim.worldwind.registries.Properties;

public class DronekitConnectionProperties implements Properties<AircraftConnection> {

	public static final String AIRCRAFT_CONNECTION_LOCALHOST = "localhost";
	public static final int AIRCRAFT_CONNECTION_GRPCPORT = 50051;
	
	private String host;
	private int port;
	
	public DronekitConnectionProperties() {
		this.host = DronekitConnectionProperties.AIRCRAFT_CONNECTION_LOCALHOST;
		this.port = DronekitConnectionProperties.AIRCRAFT_CONNECTION_GRPCPORT;
	}
	
	public String getHost() {
		return this.host;
	}
	
	public void setHost(String host) {
		this.host = host;
	}
	
	public int getPort() {
		return this.port;
	}
	
	public void setPort(int port) {
		this.port = port;
	}
	
	/**
	 * Clones this aircraft connections properties bean.
	 * 
	 * @return a clone of this aircraft connections properties bean
	 * 
	 * @see Properties#clone()
	 */
	@Override
	public Properties<AircraftConnection> clone() {
		DronekitConnectionProperties clone = null;
		try {
			clone = (DronekitConnectionProperties) super.clone();
		} catch (CloneNotSupportedException e) {
			e.printStackTrace();
		}
		return clone;
	}
	
}
