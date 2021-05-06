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
package com.cfar.swim.worldwind.connections;

import java.time.ZoneId;
import java.time.ZoneOffset;
import java.time.ZonedDateTime;

import com.cfar.swim.droneconnect.Armed;
import com.cfar.swim.droneconnect.Attitude;
import com.cfar.swim.droneconnect.DroneConnectGrpc;
import com.cfar.swim.droneconnect.DroneConnectGrpc.DroneConnectBlockingStub;
import com.cfar.swim.droneconnect.DroneConnectGrpc.DroneConnectStub;
import com.cfar.swim.droneconnect.Heading;
import com.cfar.swim.droneconnect.Mode;
import com.cfar.swim.droneconnect.Null;
import com.cfar.swim.droneconnect.Safety;
import com.cfar.swim.droneconnect.Status;
import com.cfar.swim.droneconnect.TakeoffToAltitude;
import com.cfar.swim.droneconnect.Time;
import com.cfar.swim.droneconnect.TimedPosition;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.connections.DronekitDatalinkProperties;
import com.cfar.swim.worldwind.tracks.AircraftTrackPoint;

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
	
	/** the unknown status of an aircraft connected via a dronekit datalink */
	public static final String STATUS_UNKNOWN = "UNKNOWN";
	
	/** the active status of an aircraft connected via a dronekit datalink */
	public static final String STATUS_ACTIVE = "ACTIVE";
	
	/** the unknown mode of an aircraft connected via a dronekit datalink */
	public static final String MODE_UNKNOWN = "UNKNOWN";
	
	/** the auto mode of an aircraft connected via a dronekit datalink */
	public static final String MODE_AUTO = "AUTO";
	
	/** the land mode of an aircraft connected via a dronekit datalink */
	public static final String MODE_LAND = "LAND";
	
	/** the return to land mode of an aircraft connected via a dronekit datalink */
	public static final String MODE_RTL = "RTL";
	
	/** the stabilize mode of an aircraft connected via a dronekit datalink */
	public static final String MODE_STABILIZE = "STABILIZE";
	
	// TODO: available status and mode constants (consider enum)
	
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
		this.getAircraftTrack().setName(host + ":" + port);
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
	 * Determines whether or not this dronekit datalink is connected.
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
	 * Gets the aircraft status via this dronekit datalink.
	 * 
	 * @return the aircraft status obtained via this dronekit datalink
	 * 
	 * @see Datalink#getAircraftStatus()
	 */
	@Override
	public String getAircraftStatus() {
		String aircraftStatus = null;
		
		if (this.isConnected()) {
			Null request = Null.newBuilder().build();
			Status status = this.blockingStub.getStatus(request);
			aircraftStatus = status.getStatus();
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
		
		return aircraftStatus;
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
	 * Gets the aircraft heading via this dronekit datalink.
	 * 
	 * @return the aircraft heading obtained via this dronekit datalink
	 * 
	 * @see Datalink#getAircraftHeading()
	 */
	@Override
	public Angle getAircraftHeading() {
		Angle aircraftHeading = null;
		
		if (this.isConnected()) {
			Null request = Null.newBuilder().build();
			Heading heading = this.blockingStub.getHeading(request);
			aircraftHeading = Angle.fromDegrees(heading.getHeading());
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
		
		return aircraftHeading;
	}
	
	/**
	 * Gets the aircraft pitch via this dronekit datalink.
	 * 
	 * @return the aircraft pitch obtained via this dronekit datalink
	 * 
	 * @see Datalink#getAircraftPitch()
	 */
	@Override
	public Angle getAircraftPitch() {
		Angle aircraftPitch = null;
		
		if (this.isConnected()) {
			Null request = Null.newBuilder().build();
			Attitude attitude = this.blockingStub.getAttitude(request);
			aircraftPitch = Angle.fromRadians(attitude.getPitch());
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
		
		return aircraftPitch;
	}
	
	/**
	 * Gets the aircraft bank via this dronekit datalink.
	 * 
	 * @return the aircraft bank obtained via this dronekit datalink
	 * 
	 * @see Datalink#getAircraftBank()
	 */
	@Override
	public Angle getAircraftBank() {
		Angle aircraftBank = null;
		
		if (this.isConnected()) {
			Null request = Null.newBuilder().build();
			Attitude attitude = this.blockingStub.getAttitude(request);
			aircraftBank = Angle.fromRadians(attitude.getBank());
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
		
		return aircraftBank;
	}
	
	/**
	 * Gets the aircraft yaw via this dronekit datalink.
	 * 
	 * @return the aircraft yaw obtained via this dronekit datalink
	 * 
	 * @see Datalink#getAircraftYaw()
	 */
	@Override
	public Angle getAircraftYaw() {
		Angle aircraftYaw = null;
		
		if (this.isConnected()) {
			Null request = Null.newBuilder().build();
			Attitude attitude = this.blockingStub.getAttitude(request);
			aircraftYaw = Angle.fromRadians(attitude.getYaw());
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
		
		return aircraftYaw;
	}
	
	/**
	 * Gets the aircraft position via this dronekit datalink.
	 * 
	 * @return the aircraft position obtained via this dronekit datalink
	 * 
	 * @see Datalink#getAircraftPosition()
	 */
	@Override
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
	 * Gets an aircraft track point via this dronekit datalink.
	 * 
	 * @return an aircraft track point obtained via this dronekit datalink
	 * 
	 * @see Datalink#getAircraftTrackPoint()
	 */
	@Override
	public AircraftTrackPoint getAircraftTrackPoint() {
		AircraftTrackPoint aircraftTrackPoint = null;
		
		if (this.isConnected()) {
			Null request = Null.newBuilder().build();
			// TODO: refactor dronekit interface: getTrackPoint() including attitude
			TimedPosition timedPosition = this.blockingStub.getTimedPosition(request);
			Attitude attitude = this.blockingStub.getAttitude(request);
			
			aircraftTrackPoint = new AircraftTrackPoint(new Position(
					Angle.fromDegrees(timedPosition.getPosition().getLat()),
					Angle.fromDegrees(timedPosition.getPosition().getLon()),
					timedPosition.getPosition().getGpsAltitude()));
			Time time = timedPosition.getTime();
			aircraftTrackPoint.setAto(ZonedDateTime.of(
					time.getYear(), time.getMonth(), time.getDay(),
					time.getHour(), time.getMinute(), time.getSecond(), 0,
					ZoneId.ofOffset("UTC", ZoneOffset.UTC)));
			aircraftTrackPoint.setPitch(Angle.fromDegrees(attitude.getPitch()));
			aircraftTrackPoint.setBank(Angle.fromDegrees(attitude.getBank()));
			aircraftTrackPoint.setYaw(Angle.fromDegrees(attitude.getYaw()));
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
		
		return aircraftTrackPoint;
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
	 * Determines whether or not the aircraft safety is enabled for the
	 * aircraft connected via this dronekit datalink.
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
	 * Determines whether or not the aircraft connected via this dronekit
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
	 * Uploads a mission flight path to the aircraft connected via this
	 * dronekit datalink.
	 * 
	 * @param flightPath the mission flight path to be uploaded
	 * 
	 * @see Datalink#uploadMission(Path)
	 */
	@Override
	public void uploadMission(Path flightPath) {
		if (this.isConnected()) {
			NullStreamObserver nso = new NullStreamObserver();
			// TODO: refactor dronekit interface: setMission()
			StreamObserver<com.cfar.swim.droneconnect.Position> pso = this.stub.setPath(nso);
			for (Position position : flightPath.getPositions()) {
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
	 * Downloads a mission flight path from the aircraft connected via this
	 * dronekit datalink.
	 * 
	 * @return the downloaded mission flight path
	 * 
	 * @see Datalink#downloadMission()
	 */
	@Override
	public Path downloadMission() {
		// TODO: refactor dronekit interface: implement stub.getMission()
		return null;
	}
	
	/**
	 * Gets the next position of the mission flight path from the aircraft
	 * connected via this dronekit datalink.
	 * 
	 * @return the next position of the mission flight path
	 * 
	 * @see Datalink#getNextMissionPosition()
	 */
	@Override
	public Position getNextMissionPosition() {
		Position nextPosition = null;
		
		if (this.isConnected()) {
			Null request = Null.newBuilder().build();
			// TODO: refactor dronekit interface: getNextMissionPosition()
			com.cfar.swim.droneconnect.Position position = this.blockingStub.getNextWaypoint(request).getPosition();
			nextPosition = new Position(
					Angle.fromDegrees(position.getLat()),
					Angle.fromDegrees(position.getLon()),
					position.getGpsAltitude());
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
		
		return nextPosition;
	}
	
	/**
	 * Gets the index of the next position of the mission flight path from
	 * the aircraft connected via this dronekit datalink.
	 * 
	 * @return the index of the next position of the mission flight path
	 * 
	 * @see Datalink#getNextMissionPositionIndex()
	 */
	@Override
	public int getNextMissionPositionIndex() {
		int index = -1;
		
		if (this.isConnected()) {
			Null request = Null.newBuilder().build();
			// TODO: refactor dronekit interface: getNextMissionPositionIndex()
			index = this.blockingStub.getNextWaypoint(request).getIndex();
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}

		return index;
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
			// TODO: include take-off setup (initial altitude)
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
			Mode landMode = Mode.newBuilder().setMode(DronekitDatalink.MODE_LAND).build();
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
			Mode landMode = Mode.newBuilder().setMode(DronekitDatalink.MODE_RTL).build();
			this.blockingStub.setMode(landMode);
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
	}
	
	/**
	 * Determines whether or not the aircraft connected via this dronekit
	 * datalink is airborne.
	 * 
	 * @return true if the aircraft is airborne, false otherwise
	 * 
	 * @see Datalink#isAirborne()
	 */
	public boolean isAirborne() {
		boolean isAirborne = false;
		
		if (this.isConnected()) {
			Null request = Null.newBuilder().build();
			isAirborne = this.blockingStub.getStatus(request)
					.getStatus().equals(DronekitDatalink.STATUS_ACTIVE);
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
		
		return isAirborne;
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
						&& (specification.getId().equals(Specification.CONNECTION_DATALINK_DRONEKIT_ID));
		}
	
		return matches;
	}
	
}
