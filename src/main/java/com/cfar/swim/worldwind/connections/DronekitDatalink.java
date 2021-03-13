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

import com.cfar.swim.droneconnect.Armed;
import com.cfar.swim.droneconnect.DroneConnectGrpc;
import com.cfar.swim.droneconnect.DroneConnectGrpc.DroneConnectBlockingStub;
import com.cfar.swim.droneconnect.DroneConnectGrpc.DroneConnectStub;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.connections.DronekitDatalinkProperties;
import com.cfar.swim.droneconnect.Mode;
import com.cfar.swim.droneconnect.Null;
import com.cfar.swim.droneconnect.Safety;
import com.cfar.swim.droneconnect.TakeoffToAltitude;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.Path;
import io.grpc.Channel;
import io.grpc.netty.NegotiationType;
import io.grpc.netty.NettyChannelBuilder;
import io.grpc.stub.StreamObserver;


/**
 * Realizes a dronekit datalink.
 * 
 * @author Stephan Heinemann
 *
 */
public class DronekitDatalink extends Datalink {
	
	/** the remote host of this dronekit datalink */
	private String host;
	
	/** the remote port of this dronekit datalink */
	private int port;
	
	/** the netty channel of this dronekit datalink */
	private Channel channel;
	
	/** the blocking RPC stub of this dronekit datalink */
	private DroneConnectBlockingStub blockingStub;
	
	/** the RPC stub of this dronekit datalink */
	private DroneConnectStub stub;
	
	// TODO: implement maximum blocking delay
	
	/**
	 * Constructs a new donekit datalink with a specified remote host and port.
	 * 
	 * @param host the remote dronekit host
	 * @param port the remote dronekit port
	 */
	public DronekitDatalink(String host, int port) {
		this.host = host;
		this.port = port;
		this.channel = null;
		this.blockingStub = null;
	}
	
	/**
	 * Connects this dronekit datalink.
	 * 
	 * @see Datalink#connect()
	 */
	@Override
	public void connect() {
		this.channel = NettyChannelBuilder.forAddress(this.host, this.port)
				.negotiationType(NegotiationType.PLAINTEXT)
				.build();
		this.blockingStub = DroneConnectGrpc.newBlockingStub(this.channel);
		this.stub = DroneConnectGrpc.newStub(this.channel);
	}
	
	/**
	 * Disconnects this dronekit datalink.
	 * 
	 * @see Datalink#disconnect()
	 */
	@Override
	public void disconnect() {
		this.channel = null;
		this.blockingStub = null;
	}
	
	/**
	 * Indicates whether or not this dronekit datalink is connected.
	 * 
	 * @return true if this dronekit datalink is connected, false otherwise
	 * 
	 * @see Datalink#isConnected()
	 */
	@Override
	public boolean isConnected() {
		return (null != this.blockingStub);
	}
	
	/**
	 * Gets the aircraft heading via this dronekit datalink.
	 * 
	 * @return the aircraft heading obtained via this dronekit datalink
	 */
	public Angle getAircraftHeading() {
		// TODO: implement
		return Angle.ZERO;
	}
	
	/**
	 * Gets the aircraft pitch via this dronekit datalink.
	 * 
	 * @return the aircraft pitch obtained via this dronekit datalink
	 */
	public Angle getAircraftPitch() {
		// TODO: implement
		return Angle.ZERO;
	}
	
	/**
	 * Gets the aircraft bank via this dronekit datalink.
	 * 
	 * @return the aircraft bank obtained via this dronekit datalink
	 */
	public Angle getAircraftBank() {
		// TODO: implement
		return Angle.ZERO;
	}
	
	/**
	 * Gets the aircraft yaw via this dronekit datalink.
	 * 
	 * @return the aircraft yaw obtained via this dronekit datalink
	 */
	public Angle getAircraftYaw() {
		// TODO: implement
		return Angle.ZERO;
	}
	
	/**
	 * Gets the aircraft position via this dronekit datalink.
	 * 
	 * @return the aircraft position obtained via this dronekit datalink
	 * 
	 * @see Datalink#getAircraftPosition()
	 */
	public Position getAircraftPosition() {
		Position aircraftPosition = null;
		
		if (this.isConnected()) {
			Null request = Null.newBuilder().build();
			com.cfar.swim.droneconnect.Position position = this.blockingStub.getPosition(request);
			aircraftPosition = new Position(
					Angle.fromDegrees(position.getLat()),
					Angle.fromDegrees(position.getLon()),
					position.getGpsAltitude());
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
		
		return aircraftPosition;
	}
	
	/**
	 * Gets the aircraft mode via this dronekit datalink.
	 * 
	 * @return the aircraft mode obtained via this dronekit datalink
	 * 
	 * @see Datalink#getAircraftMode()
	 */
	@Override
	public String getAircraftMode() {
		String aircraftMode = null;
		
		if (this.isConnected()) {
			Null request = Null.newBuilder().build();
			aircraftMode = this.blockingStub.hasMode(request).getMode();
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
		
		return aircraftMode;
	}
	
	/**
	 * Sets the aircraft mode via this dronekit datalink.
	 * 
	 * @param aircraftMode the aircraft mode to be set
	 * 
	 * @see Datalink#setAircraftMode(String)
	 */
	@Override
	public void setAircraftMode(String aircraftMode) {
		if (this.isConnected()) {
			Mode mode = Mode.newBuilder().setMode(aircraftMode).build();
			blockingStub.setMode(mode);
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
	}
	
	/**
	 * Enables the aircraft safety via this dronekit datalink.
	 * 
	 * @see Datalink#enableAircraftSafety()
	 */
	@Override
	public void enableAircraftSafety() {
		if (this.isConnected()) {
			Safety safety = Safety.newBuilder().setSafety(true).build();
			blockingStub.setSafety(safety);
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
	}
	
	/**
	 * Disables the aircraft safety via this dronekit datalink.
	 * 
	 * @see Datalink#disableAircraftSafety()
	 */
	@Override
	public void disableAircraftSafety() {
		if (this.isConnected()) {
			Safety safety = Safety.newBuilder().setSafety(false).build();
			blockingStub.setSafety(safety);
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
	}
	
	/**
	 * Indicates whether or not the aircraft safety is enabled for the aircraft
	 * connected via this dronekit datalink.
	 * 
	 * @return true if the aircraft safety is enabled, false otherwise
	 * 
	 * @see Datalink#isAircraftSafetyEnabled()
	 */
	@Override
	public boolean isAircraftSafetyEnabled() {
		boolean isAircraftSafetyEnabled = false;
		
		if (this.isConnected()) {
			Null request = Null.newBuilder().build();
			isAircraftSafetyEnabled = blockingStub.getSafety(request).getSafety();
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
		
		return isAircraftSafetyEnabled;
	}
	
	/**
	 * Arms the aircraft via this dronekit datalink.
	 * 
	 * @see Datalink#armAircraft()
	 */
	@Override
	public void armAircraft() {
		if (this.isConnected()) {
			Armed armed = Armed.newBuilder().setArm(true).build();
			blockingStub.setArmed(armed);
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
	}
	
	/**
	 * Disarms the aircraft via this dronekit datalink.
	 * 
	 * @see Datalink#disarmAircraft()
	 */
	@Override
	public void disarmAircraft() {
		if (this.isConnected()) {
			Armed armed = Armed.newBuilder().setArm(false).build();
			blockingStub.setArmed(armed);
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
	}
	
	/**
	 * Indicates whether or not the aircraft connected via this dronekit
	 * datalink is armed.
	 * 
	 * @return true if the aircraft is armed, false otherwise
	 * 
	 * @see Datalink#isAircraftArmed()
	 */
	@Override
	public boolean isAircraftArmed() {
		boolean isArmed = false;
		
		if (this.isConnected()) {
			Null request = Null.newBuilder().build();
			isArmed = blockingStub.isArmed(request).getArm();
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
		
		return isArmed;
	}
	
	/**
	 * Uploads a flight path to the aircraft connected via this dronekit
	 * datalink.
	 * 
	 * @param path the flight path to be uploaded
	 * 
	 * @see Datalink#uploadFlightPath(Path)
	 */
	@Override
	public void uploadFlightPath(Path path) {
		if (this.isConnected()) {
			NullStreamObserver nso = new NullStreamObserver();
			StreamObserver<com.cfar.swim.droneconnect.Position> pso = this.stub.setPath(nso);
			for (Position position : path.getPositions()) {
				com.cfar.swim.droneconnect.Position pos = com.cfar.swim.droneconnect.Position
						.newBuilder()
						.setLat(position.getLatitude().getDegrees())
						.setLon(position.getLongitude().getDegrees())
						.setGpsAltitude(position.getAltitude())
						.setRelativeAltitude(position.getAltitude())
						.setUseRelativeAltitude(false)
						.build();
				pso.onNext(pos);
			}
			pso.onCompleted();
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
	}
	
	/**
	 * Realizes a null stream observer.
	 * 
	 * @author Stephan Heinemann
	 *
	 */
	private class NullStreamObserver implements StreamObserver<Null> {

		@Override
		public void onNext(Null value) {
		}

		@Override
		public void onError(Throwable t) {
		}

		@Override
		public void onCompleted() {
		}
	}
	
	/**
	 * Initiates a take-off for the aircraft connected via this dronekit
	 * datalink.
	 * 
	 * @see Datalink#takeOff()
	 */
	@Override
	public void takeOff() {
		if (this.isConnected()) {
			// TODO: include take-off setup
			TakeoffToAltitude altitude = TakeoffToAltitude.newBuilder().setAltitude(5d).build();
			this.blockingStub.takeoff(altitude);
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
	}
	
	/**
	 * Initiates a landing for the aircraft connected via this dronekit
	 * datalink.
	 * 
	 * @see Datalink#land()
	 */
	@Override
	public void land() {
		if (this.isConnected()) {
			// TODO: flight mode enumeration
			Mode landMode = Mode.newBuilder().setMode("LAND").build();
			this.blockingStub.setMode(landMode);
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
	}
	
	/**
	 * Initiates a return to and landing at the launch position for the
	 * aircraft connected via this dronekit datalink.
	 * 
	 * @see Datalink#returnToLaunch()
	 */
	@Override
	public void returnToLaunch() {
		if (this.isConnected()) {
			// TODO: flight mode enumeration
			Mode landMode = Mode.newBuilder().setMode("RTL").build();
			this.blockingStub.setMode(landMode);
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
	}
	
	/**
	 * Determines whether or not this dronekit datalink matches a specification.
	 * 
	 * @param specification the specification to be matched
	 * 
	 * @return true if the this dronekit datalink matches the specification,
	 *         false otherwise
	 * 
	 * @see Datalink#matches(Specification)
	 */
	@Override
	public final boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = super.matches(specification);
		
		if (matches && (specification.getProperties() instanceof DronekitDatalinkProperties)) {
			DronekitDatalinkProperties ddlp = (DronekitDatalinkProperties) specification.getProperties();
			matches = (this.host.equals(ddlp.getHost()))
						&& (this.port == ddlp.getPort())
						&& (specification.getId().equals(Specification.DATALINK_DRONEKIT));
		}
	
		return matches;
	}
	
}
