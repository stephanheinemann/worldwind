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

import java.io.EOFException;
import java.io.IOException;
import java.math.BigInteger;
import java.net.Socket;
import java.time.Duration;
import java.time.Instant;
import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.Optional;

import com.cfar.swim.worldwind.geom.precision.Precision;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.connections.MavlinkDatalinkProperties;
import com.cfar.swim.worldwind.tracks.AircraftAttitude;
import com.cfar.swim.worldwind.tracks.AircraftTrackPoint;
import com.cfar.swim.worldwind.util.Identifiable;
import com.google.common.collect.Iterables;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.Path;
import gov.nasa.worldwind.util.Logging;
import io.dronefleet.mavlink.MavlinkConnection;
import io.dronefleet.mavlink.MavlinkMessage;
import io.dronefleet.mavlink.common.Attitude;
import io.dronefleet.mavlink.common.CommandAck;
import io.dronefleet.mavlink.common.CommandInt;
import io.dronefleet.mavlink.common.CommandLong;
import io.dronefleet.mavlink.common.ExtendedSysState;
import io.dronefleet.mavlink.common.GlobalPositionInt;
import io.dronefleet.mavlink.common.Heartbeat;
import io.dronefleet.mavlink.common.MavAutopilot;
import io.dronefleet.mavlink.common.MavCmd;
import io.dronefleet.mavlink.common.MavComponent;
import io.dronefleet.mavlink.common.MavFrame;
import io.dronefleet.mavlink.common.MavLandedState;
import io.dronefleet.mavlink.common.MavMissionResult;
import io.dronefleet.mavlink.common.MavMissionType;
import io.dronefleet.mavlink.common.MavMode;
import io.dronefleet.mavlink.common.MavModeFlag;
import io.dronefleet.mavlink.common.MavState;
import io.dronefleet.mavlink.common.MavType;
import io.dronefleet.mavlink.common.MissionAck;
import io.dronefleet.mavlink.common.MissionCount;
import io.dronefleet.mavlink.common.MissionCurrent;
import io.dronefleet.mavlink.common.MissionItem;
import io.dronefleet.mavlink.common.MissionItemInt;
import io.dronefleet.mavlink.common.MissionRequest;
import io.dronefleet.mavlink.common.MissionRequestInt;
import io.dronefleet.mavlink.common.MissionRequestList;
import io.dronefleet.mavlink.common.Ping;
import io.dronefleet.mavlink.common.SetMode;
import io.dronefleet.mavlink.common.SystemTime;
import io.dronefleet.mavlink.common.VfrHud;
import io.dronefleet.mavlink.util.EnumValue;

/**
 * Realizes a mavlink datalink.
 * 
 * @author Stephan Heinemann
 *
 */
@SuppressWarnings("deprecation")
public class MavlinkDatalink extends Datalink {
	
	/** the host of this mavlink datalink */
	private String host = MavlinkDatalinkProperties.MAVLINK_DEFAULT_HOST;
	
	/** the port of this mavlink datalink */
	private int port = MavlinkDatalinkProperties.MAVLINK_DEFAULT_PORT;
	
	/** the source identifier of this mavlink datalink (GCS) */
	private int sourceId = MavlinkDatalinkProperties.MAVLINK_DEFAULT_SOURCE_ID;
	
	/** the target identifier of this mavlink datalink (vehicle) */
	private int targetId = MavlinkDatalinkProperties.MAVLINK_DEFAULT_TARGET_ID;
	
	/** indicates whether or not this mavlink datalink is connected */
	private boolean isConnected = false;
	
	/** the TCP client socket of this mavlink datalink */
	private Socket tcp = new Socket();
	
	/** the mavlink connection of this mavlink datalink */
	MavlinkConnection mavlink = null;
	
	/** the uplink delay of this mavlink datalink */
	private Duration uplinkDelay = Duration.ofMillis(100l);
	
	
	/**
	 * Constructs a new mavlink datalink.
	 * 
	 * @param host the host of the mavlink service
	 * @param port the port of the mavlink service
	 * @param sourceId the source identifier of this mavlink source (GCS)
	 * @param targetId the target identifier of the mavlink target (vehicle)
	 */
	public MavlinkDatalink(String host, int port, int sourceId, int targetId) {
		this.host = host;
		this.port = port;
		this.sourceId = sourceId;
		this.targetId = targetId;
		this.getAircraftTrack().setName(this.getTargetId()
				+ "@" + this.getHost() + ":" + this.getPort());
	}
	
	/**
	 * Gets the identifier of this mavlink datalink.
	 * 
	 * @return the identifier of this mavlink datalink
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return Specification.CONNECTION_DATALINK_MAVLINK_ID;
	}
	
	/**
	 * Gets the host of this mavlink datalink.
	 * 
	 * @return the host of this mavlink datalink
	 */
	public String getHost() {
		return this.host;
	}
	
	/**
	 * Gets the port of this mavlink datalink.
	 * 
	 * @return the port of this mavlink datalink
	 */
	public int getPort() {
		return this.port;
	}
	
	/**
	 * Gets the source identifier of this mavlink datalink (GCS).
	 * 
	 * @return the source identifier of this mavlink datalink
	 */
	public int getSourceId() {
		return this.sourceId;
	}
	
	/**
	 * Gets the target identifier of this mavlink datalink (vehicle).
	 * 
	 * @return the target identifier of this mavlink datalink
	 */
	public int getTargetId() {
		return this.targetId;
	}
	
	/**
	 * Connects this mavlink datalink.
	 * 
	 * @see Datalink#connect()
	 */
	@Override
	public synchronized void connect() {
		Logging.logger().info("connecting mavlink...");
		try {
			this.tcp = new Socket(this.getHost(), this.getPort());
			this.mavlink = MavlinkConnection.create(
		            this.tcp.getInputStream(),
		            this.tcp.getOutputStream());
			this.isConnected = true;
			this.determineUplinkDelay();
		} catch (Exception e) {
			e.printStackTrace();
			this.disconnect();
		}
	}
	
	/**
	 * Disconnect this mavlink datalink.
	 * 
	 * @see Datalink#disconnect()
	 */
	@Override
	public synchronized void disconnect() {
		Logging.logger().info("disconnecting mavlink...");
		try {
			this.tcp.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		this.mavlink = null;
		this.isConnected = false;
	}
	
	/**
	 * Determines whether or not this mavlink datalink is connected.
	 * 
	 * @return true if this mavlink datalink is connected, false otherwise
	 * 
	 * @see Datalink#isConnected()
	 */
	@Override
	public synchronized boolean isConnected() {
		return this.isConnected;
	}
	
	/**
	 * Sniffs this mavlink datalink.
	 * 
	 * @param systemId the system identifier of sniffed mavlink messages
	 * @param componentId the component identifier of sniffed mavlink messages
	 * @param payloadClasses the possible payloads of sniffed mavlink messages
	 * @param messageLimit the limit of mavlink messages to scan
	 */
	protected synchronized void sniff(int systemId, int componentId,
			Collection<Class<?>> payloadClasses, int messageLimit) {
		int receivedMessages = 0;
		
		while (receivedMessages < messageLimit) {
			try {
				MavlinkMessage<?> message = this.mavlink.next();
				
				if ((null != message)
						//&& (message.getOriginSystemId() == systemId)
						//&& (message.getOriginComponentId() == componentId)
						&& (payloadClasses.contains(message.getPayload().getClass()))) {
					Logging.logger().info(message.getPayload().toString());
				}
			
				receivedMessages++;
			} catch (EOFException eofe) {
				Logging.logger().warning("creating new mavlink connection");
				this.disconnect();
				this.connect();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}
	
	/**
	 * Sends a mavlink datalink command.
	 * 
	 * @param command the command to be sent
	 */
	protected synchronized void sendCommand(Object command) {
		try {
			this.mavlink.send1(this.getSourceId(),
					MavComponent.MAV_COMP_ID_PATHPLANNER.ordinal(), command);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * Receives a mavlink datalink message payload.
	 * 
	 * @param payloadClasses the possible payloads of the mavlink message
	 * @param delay delay before receiving the mavlink message
	 * @param preskip skip all messages before receiving the mavlink message
	 * @param postskip skip all messages after receiving the mavlink message
	 * @param messageLimit the limit of mavlink messages to scan
	 * 
	 * @return the received mavlink message payload, if any
	 */
	protected synchronized Optional<?> receiveMessage(
			Collection<Class<?>> payloadClasses, boolean delay,
			boolean preskip, boolean postskip, int messageLimit) {
		Optional<?> payload = Optional.empty();
		int receivedMessages = 0;
		
		// delay reception
		if (delay) {
			try {
				Thread.sleep(this.getUplinkDelay().toMillis());
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		
		// skip messages before receiving
		if (preskip) {
			try {
				this.tcp.getInputStream().skip(
						this.tcp.getInputStream().available() - 1);
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		
		// receive new messages
		while (payload.isEmpty() && (receivedMessages < messageLimit)) {
			try {
				MavlinkMessage<?> message = this.mavlink.next();
				if ((null != message)
						&& (this.getTargetId() == message.getOriginSystemId())
						&& (payloadClasses.contains(message.getPayload().getClass()))) {
					payload = Optional.of(message.getPayload());
				}
				receivedMessages++;
			} catch (IOException e) {
				Logging.logger().warning("creating new mavlink connection");
				this.disconnect();
				this.connect();
			}
		}
		
		// skip messages after receiving
		if (postskip) {
			try {
				this.tcp.getInputStream().skip(
						this.tcp.getInputStream().available() - 1);
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		
		return payload;
	}
	
	/**
	 * Receives a mavlink datalink command protocol acknowledgement.
	 * 
	 * @param command the command to be acknowledged
	 * 
	 * @return the command protocol acknowledgment, if any 
	 */
	protected synchronized Optional<CommandAck> receiveCommandAck(MavCmd command) {
		Optional<CommandAck> ack = Optional.empty();
		
		Optional<?> payload = this.receiveMessage(
				Collections.singleton(CommandAck.class), true, false, false,
				MavlinkDatalinkProperties.MAVLINK_DEFAULT_SCAN_LIMIT);
		
		if (payload.isPresent()) {
			CommandAck commandAck = (CommandAck) payload.get();
			if ((this.getSourceId() == commandAck.targetSystem())
					&& (command == commandAck.command().entry())) {
				ack = Optional.of(commandAck);
				Logging.logger().info("mavlink datalink received command ack result "
						+ commandAck.result().entry().name() + " for "
						+ command.name());
			} else {
				// TODO: possibly retry for correct command ack
				Logging.logger().warning("mavlink datalink received incorrect command ack for "
						+ command.name() + ": "
						+ commandAck.command().entry().name());
			}
		} else {
			Logging.logger().warning("mavlink datalink received no command ack for "
					+ command.name());
		}
		
		return ack;
	}
	
	/**
	 * Receives a mavlink datalink mission protocol acknowledgement.
	 * 
	 * @return the mission protocol acknowledgment, if any 
	 */
	protected synchronized Optional<MissionAck> receiveMissionAck() {
		Optional<MissionAck> ack = Optional.empty();
		
		Optional<?> payload = this.receiveMessage(
				Collections.singleton(MissionAck.class), true, false, false,
				MavlinkDatalinkProperties.MAVLINK_DEFAULT_SCAN_LIMIT);
		
		if (payload.isPresent()) {
			MissionAck missionAck = (MissionAck) payload.get();
			if ((this.getSourceId() == missionAck.targetSystem())) {
				ack = Optional.of(missionAck);
				Logging.logger().info("mavlink datalink received mission ack "
						+ missionAck.type().entry().name());
			} else {
				// TODO: possibly retry for correct mission ack
				Logging.logger().warning("mavlink datalink received incorrect misison ack");
			}
		} else {
			Logging.logger().warning("mavlink datalink received no mission ack");
		}
		
		return ack;
	}
	
	/**
	 * Gets the roundtrip delay of this mavlink datalink.
	 * 
	 * @return the roundtrip delay of this mavlink datalink, null otherwise
	 * 
	 * @see Datalink#getRoundtripDelay()
	 */
	public synchronized Duration getRoundtripDelay() {
		Duration delay = null;
		
		if (this.isConnected()) {
			ZonedDateTime pingTime = ZonedDateTime.now();
			Ping ping = Ping.builder()
					.targetSystem(0)
					.seq(0l)
					.timeUsec(BigInteger.valueOf((long)
							(pingTime.toInstant().toEpochMilli() / Precision.UNIT_MILLI)))
					.build();
			this.sendCommand(ping);
			
			Optional<?> payload = this.receiveMessage(
					Collections.singleton(Ping.class), false, false, false,
					MavlinkDatalinkProperties.MAVLINK_DEFAULT_SCAN_LIMIT);
			ZonedDateTime pongTime = ZonedDateTime.now();
			
			if (payload.isPresent()) {
				Ping pong = (Ping) payload.get();
				
				if ((this.getSourceId() == pong.targetSystem())
						&& (0l == pong.seq())) {
					delay = Duration.between(pingTime, pongTime);
				}
			} else {
				Logging.logger().warning("mavlink datalink received no pong");
			}
		}
		
		return delay;
	}
	
	/**
	 * Gets the uplink delay of this mavlink datalink.
	 * 
	 * @return the uplink delay of this mavlink datalink
	 */
	protected synchronized Duration getUplinkDelay() {
		return this.uplinkDelay;
	}
	
	/**
	 * Sets the uplink delay of this mavlink datalink.
	 * 
	 * @param uplinkDelay the uplink delay to be set
	 * 
	 * @throws IllegalArgumentException if the uplink delay is invalid
	 */
	protected synchronized void setUplinkDelay(Duration uplinkDelay) {
		if ((null == uplinkDelay) || uplinkDelay.isNegative()
				|| uplinkDelay.isZero()) {
			throw new IllegalArgumentException("uplink delay is invalid");
		}
		this.uplinkDelay = uplinkDelay;
	}
	
	/**
	 * Determines the uplink delay of this mavlink datalink.
	 */
	protected synchronized void determineUplinkDelay() {
		Duration roundtrip = this.getRoundtripDelay();
		if ((null != roundtrip) && !roundtrip.isNegative()
				&& !roundtrip.isZero()) {
			this.setUplinkDelay(roundtrip.dividedBy(2l));
		}
	}
	
	/**
	 * Gets the aircraft system time via this mavlink datalink.
	 * 
	 * @return the aircraft system time obtained via this mavlink datalink,
	 *         null otherwise
	 * 
	 * @see Datalink#getSystemTime()
	 */
	@Override
	public synchronized ZonedDateTime getSystemTime() {
		ZonedDateTime time = null;
		
		if (this.isConnected()) {
			Optional<?> payload = this.receiveMessage(
					Collections.singleton(SystemTime.class), false, true, false,
					MavlinkDatalinkProperties.MAVLINK_DEFAULT_SCAN_LIMIT);
			
			if (payload.isPresent()) {
				SystemTime systime = (SystemTime) payload.get();
				Instant instant = Instant.ofEpochMilli(Math.round(
						systime.timeUnixUsec().longValue() * Precision.UNIT_MILLI));
				time = ZonedDateTime.ofInstant(instant, ZoneOffset.UTC);
			} else {
				Logging.logger().warning("mavlink datalink received no system time");
			}
		} else {
			Logging.logger().warning("mavlink datalink is disconnected");
		}
		
		return time;
	}
	
	/**
	 * Emits a heart beat via this mavlink datalink.
	 * 
	 * @see Datalink#emitHeartbeat()
	 */
	@Override
	public synchronized void emitHeartbeat() {
		if (this.isConnected()) {
			Heartbeat heartbeat = Heartbeat.builder()
					.type(MavType.MAV_TYPE_GCS)
					.autopilot(MavAutopilot.MAV_AUTOPILOT_INVALID)
					.build();
			this.sendCommand(heartbeat);
			this.determineUplinkDelay();
		} else {
			Logging.logger().warning("mavlink datalink is disconnected");
		}
	}
	
	/**
	 * Gets the aircraft status via this mavlink datalink.
	 * 
	 * @return the aircraft status obtained via this mavlink datalink,
	 *         null otherwise
	 * 
	 * @see Datalink#getAircraftStatus()
	 */
	@Override
	public synchronized String getAircraftStatus() {
		String status = null;
		
		if (this.isConnected()) {
			Optional<?> payload = this.receiveMessage(
					Collections.singleton(Heartbeat.class), false, true, false,
					MavlinkDatalinkProperties.MAVLINK_DEFAULT_SCAN_LIMIT);
			
			if (payload.isPresent()) {
				EnumValue<MavState> mavState =
						((Heartbeat) payload.get()).systemStatus();
				status = mavState.entry().name();
			} else {
				Logging.logger().warning("mavlink datalink received no status");
			}
		} else {
			Logging.logger().warning("mavlink datalink is disconnected");
		}
		
		return status;
	}
	
	/**
	 * Gets the aircraft mode via this mavlink datalink.
	 * 
	 * @return the aircraft mode obtained via this mavlink datalink,
	 *         null otherwise
	 * 
	 * @see Datalink#getAircraftMode()
	 */
	@Override
	public synchronized String getAircraftMode() {
		String mode = null;
		
		if (this.isConnected()) {
			Optional<?> payload = this.receiveMessage(
					Collections.singleton(Heartbeat.class), false, true, false,
					MavlinkDatalinkProperties.MAVLINK_DEFAULT_SCAN_LIMIT);
			
			if (payload.isPresent()) {
				EnumValue<MavModeFlag> baseMode =
						((Heartbeat) payload.get()).baseMode();
				mode = "";
				if (baseMode.flagsEnabled(MavModeFlag.MAV_MODE_FLAG_AUTO_ENABLED)) {
					mode += MavModeFlag.MAV_MODE_FLAG_AUTO_ENABLED.name() + " ";
				}
				if (baseMode.flagsEnabled(MavModeFlag.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) {
					mode += MavModeFlag.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED.name() + " ";
				}
				if (baseMode.flagsEnabled(MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED)) {
					mode += MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED.name() + " ";
				}
				if (baseMode.flagsEnabled(MavModeFlag.MAV_MODE_FLAG_HIL_ENABLED)) {
					mode += MavModeFlag.MAV_MODE_FLAG_HIL_ENABLED.name() + " ";
				}
				if (baseMode.flagsEnabled(MavModeFlag.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)) {
					mode += MavModeFlag.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED.name() + " ";
				}
				if (baseMode.flagsEnabled(MavModeFlag.MAV_MODE_FLAG_SAFETY_ARMED)) {
					mode += MavModeFlag.MAV_MODE_FLAG_SAFETY_ARMED.name() + " ";
				}
				if (baseMode.flagsEnabled(MavModeFlag.MAV_MODE_FLAG_STABILIZE_ENABLED)) {
					mode += MavModeFlag.MAV_MODE_FLAG_STABILIZE_ENABLED.name() + " ";
				}
				if (baseMode.flagsEnabled(MavModeFlag.MAV_MODE_FLAG_TEST_ENABLED)) {
					mode += MavModeFlag.MAV_MODE_FLAG_TEST_ENABLED.name() + " ";
				}
				mode = mode.trim();
			} else {
				Logging.logger().warning("mavlink datalink received no mode");
			}
		} else {
			Logging.logger().warning("mavlink datalink is disconnected");
		}
		
		return mode;
	}
	
	/**
	 * Sets the aircraft mode via this mavlink datalink.
	 * 
	 * @param aircraftMode the aircraft mode to be set
	 * 
	 * @see Datalink#setAircraftMode(String)
	 */
	@Override
	public synchronized void setAircraftMode(String aircraftMode) {
		if (this.isConnected()) {
			Logging.logger().info("setting mode " + aircraftMode + "...");
			CommandLong mode = CommandLong.builder()
					.targetSystem(this.getTargetId())
					.command(MavCmd.MAV_CMD_DO_SET_MODE)
					.param1(MavMode.valueOf(aircraftMode).ordinal())
					.build();
			this.sendCommand(mode);
			this.receiveCommandAck(MavCmd.MAV_CMD_DO_SET_MODE);
		} else {
			Logging.logger().warning("mavlink datalink is disconnected");
		}
	}
	
	/**
	 * Gets the aircraft attitude via this mavlink datalink.
	 * 
	 * @return the aircraft attitude obtianed via this mavlink datalink,
	 *         null otherwise
	 * 
	 * @see Datalink#getAircraftAttitude()
	 */
	@Override
	public synchronized AircraftAttitude getAircraftAttitude() {
		AircraftAttitude attitude = null;
		
		if (this.isConnected()) {
			Optional<?> payload = this.receiveMessage(
					Collections.singleton(Attitude.class), false, true, false,
					MavlinkDatalinkProperties.MAVLINK_DEFAULT_SCAN_LIMIT);
			
			if (payload.isPresent()) {
				Attitude att = (Attitude) payload.get();
				attitude = new AircraftAttitude(
						Angle.fromRadians(att.pitch()),
						Angle.fromRadians(att.roll()),
						Angle.fromRadians(att.yaw()));
			} else {
				Logging.logger().warning("mavlink datalink received no attitude");
			}
		} else {
			Logging.logger().warning("mavlink datalink is disconnected");
		}
		
		return attitude;
	}
	
	/**
	 * Gets the aircraft pitch via this mavlink datalink.
	 * 
	 * @return the aircraft pitch obtained via this mavlink datalink,
	 *         null otherwise
	 * 
	 * @see Datalink#getAircraftPitch()
	 */
	@Override
	public synchronized Angle getAircraftPitch() {
		Angle pitch = null;
				
		AircraftAttitude attitude = this.getAircraftAttitude();
		if (null != attitude) {
			pitch = attitude.getPitch();
		}
		
		return pitch;
	}
	
	/**
	 * Gets the aircraft bank via this mavlink datalink.
	 * 
	 * @return the aircraft bank obtained via this mavlink datalink,
	 *         null otherwise
	 * 
	 * @see Datalink#getAircraftBank()
	 */
	@Override
	public synchronized Angle getAircraftBank() {
		Angle bank = null;
		
		AircraftAttitude attitude = this.getAircraftAttitude();
		if (null != attitude) {
			bank = attitude.getBank();
		}
		
		return bank;
	}
	
	/**
	 * Gets the aircraft yaw via this mavlink datalink.
	 * 
	 * @return the aircraft yaw obtained via this mavlink datalink, null otherise
	 * 
	 * @see Datalink#getAircraftYaw()
	 */
	@Override
	public synchronized Angle getAircraftYaw() {
		Angle yaw = null;
		
		AircraftAttitude attitude = this.getAircraftAttitude();
		if (null != attitude) {
			yaw = attitude.getHeading();
		}
		
		return yaw;
	}
	
	/**
	 * Gets the aircraft heading via this mavlink datalink.
	 * 
	 * @return the aircraft heading obtained via this mavlink datalink,
	 *         null otherwise
	 * 
	 * @see Datalink#getAircraftHeading()
	 */
	@Override
	public synchronized Angle getAircraftHeading() {
		Angle heading = null;
		
		if (this.isConnected()) {
			Optional<?> payload = this.receiveMessage(
					Collections.singleton(GlobalPositionInt.class), false, true, false,
					MavlinkDatalinkProperties.MAVLINK_DEFAULT_SCAN_LIMIT);
			
			if (payload.isPresent()) {
				heading = Angle.fromDegrees(
						((GlobalPositionInt) payload.get()).hdg() * Precision.UNIT_CENTI);
			} else {
				Logging.logger().warning("mavlink datlink received no heading");
			}
		} else {
			Logging.logger().warning("mavlink datalink is disconnected");
		}
		
		return heading;
	}
	
	/**
	 * Gets the aircraft position via this mavlink datalink.
	 * 
	 * @return the aircraft position obtained via this mavlink datalink,
	 *         null otherwise
	 * 
	 * @see Datalink#getAircraftPosition()
	 */
	@Override
	public synchronized Position getAircraftPosition() {
		Position position = null;
		
		if (this.isConnected()) {
			Optional<?> payload = this.receiveMessage(
					Collections.singleton(GlobalPositionInt.class), false, true, false,
					MavlinkDatalinkProperties.MAVLINK_DEFAULT_SCAN_LIMIT);
			
			if (payload.isPresent()) {
				GlobalPositionInt gpi = (GlobalPositionInt) payload.get();
				position = Position.fromDegrees(
						gpi.lat() * Precision.UNIT_HECTO_NANO,
						gpi.lon() * Precision.UNIT_HECTO_NANO,
						gpi.alt() * Precision.UNIT_MILLI);
			} else {
				Logging.logger().warning("mavlink datlink received no position");
			}
		} else {
			Logging.logger().warning("mavlink datalink is disconnected");
		}
		
		return position;
	}
	
	/**
	 * Gets an aircraft track point via this mavlink datalink.
	 * 
	 * @return an aircraft track point obtained via this mavlink datalink,
	 *         null otherwise
	 * 
	 * @see Datalink#getAircraftTrackPoint()
	 */
	@Override
	public synchronized AircraftTrackPoint getAircraftTrackPoint() {
		AircraftTrackPoint trackPoint = null;
		
		// TODO: track point should be equipped with remote system time
		Position position = this.getAircraftPosition();
		// TODO: messages should not be skipped and close together
		AircraftAttitude attitude = this.getAircraftAttitude();
		
		if ((null != position) && (null != attitude)) {
			trackPoint = new AircraftTrackPoint(position, attitude);
		} else if ((null != position)) {
			trackPoint = new AircraftTrackPoint(position);
		}
		
		return trackPoint;
	}
	
	/**
	 * Enables the aircraft guidance via this mavlink datalink.
	 * 
	 * @see Datalink#enableAircraftGuidance()
	 */
	@Override
	public synchronized void enableAircraftGuidance() {
		//if (!this.isAircraftGuidanceEnabled()) {
			Logging.logger().info("enabling guidance...");
			this.setAircraftMode(MavMode.MAV_MODE_GUIDED_ARMED.name());
		//}
	}
	
	/**
	 * Disables the aircraft guidance via this mavlink datalink.
	 * 
	 * @see Datalink#disableAircraftGuidance()
	 */
	@Override
	public synchronized void disableAircraftGuidance() {
		//if (this.isAircraftGuidanceEnabled()) {
			Logging.logger().info("disabling guidance...");
			this.setAircraftMode(MavMode.MAV_MODE_GUIDED_DISARMED.name());
		//}
	}
	
	/**
	 * Determines whether or not the aircraft guidance is enabled for the
	 * aircraft connected via this mavlink datalink.
	 * 
	 * @return true if the aircraft guidance is enabled, false otherwise
	 * 
	 * @see Datalink#isAircraftGuidanceEnabled()
	 */
	@Override
	public synchronized boolean isAircraftGuidanceEnabled() {
		boolean isGuided = false;
		
		String mode = this.getAircraftMode();
		if (null != mode) {
			isGuided = mode.contains(
					MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED.name());
		}
		
		return isGuided;
	}
	
	/**
	 * Arms the aircraft via this mavlink datalink.
	 * 
	 * @see Datalink#armAircraft()
	 */
	@Override
	public synchronized void armAircraft() {
		if (this.isConnected() && !this.isAircraftArmed() && !this.isAirborne()) {
			Logging.logger().info("arming...");
			
			CommandInt arm = CommandInt.builder()
				.targetSystem(this.getTargetId())
				.command(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM)
				.param1(1f) // arm
				.param2(0f) // perform safety checks
				.build();
			this.sendCommand(arm);
			this.receiveCommandAck(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM);
		} else {
			Logging.logger().warning("mavlink datalink is disconnected, or aircraft disarmed or airborne");
		}
	}
	
	/**
	 * Disarms the aircraft via this mavlink datalink.
	 * 
	 * @see Datalink#disarmAircraft()
	 */
	@Override
	public synchronized void disarmAircraft() {
		if (this.isConnected() && this.isAircraftArmed() && !this.isAirborne()) {
			CommandInt disarm = CommandInt.builder()
					.targetSystem(this.getTargetId())
					.command(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM)
					.param1(0f) // disarm
					.param2(0f) // perform safety checks
					.build();
			this.sendCommand(disarm);
			this.receiveCommandAck(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM);
		} else {
			Logging.logger().warning("mavlink datalink is disconnected, or aircraft armed or airborne");
		}
	}
	
	/**
	 * Determines whether or not the aircraft connected via this mavlink
	 * datalink is armed.
	 * 
	 * @return true if the aircraft is armed, false otherwise
	 * 
	 * @see Datalink#isAircraftArmed()
	 */
	@Override
	public synchronized boolean isAircraftArmed() {
		boolean isArmed = false;
		
		String mode = this.getAircraftMode();
		if (null != mode) {
			isArmed = mode.contains(
					MavModeFlag.MAV_MODE_FLAG_SAFETY_ARMED.name());
		}
		
		return isArmed;
	}
	
	/**
	 * Gets the next position of the mission flight path from the aircraft
	 * connected via this mavlink datalink.
	 * 
	 * @return the next position of the mission flight path, null otherwise
	 * 
	 * @see Datalink#getNextMissionPosition()
	 */
	@Override
	public synchronized Position getNextMissionPosition() {
		Position next = null;
		
		int index = this.getNextMissionPositionIndex();
		
		if (-1 < index) {
			Path mission = this.downloadMission(true);
			Iterable<? extends Position> positions = mission.getPositions();
			
			if (index < Iterables.size(positions)) {
				// use cached mission
				next = Iterables.get(mission.getPositions(), index);
			} else {
				// use downloaded mission
				mission = this.downloadMission(false);
				positions = mission.getPositions();
				
				if (index < Iterables.size(positions)) {
					next = Iterables.get(mission.getPositions(), index);
				} else {
					Logging.logger().warning("mavlink datalink received invalid mission index");
				}
			}
		} else {
			Logging.logger().warning("mavlink datalink received no next mission position");
		}
		
		return next;
	}
	
	/**
	 * Gets the index of the next position of the mission flight path from
	 * the aircraft connected via this mavlink datalink.
	 * 
	 * @return the index of the next position of the mission flight path,
	 *         -1 otherwise
	 * 
	 * @see Datalink#getNextMissionPositionIndex()
	 */
	@Override
	public synchronized int getNextMissionPositionIndex() {
		int seq = -1;
		
		if (this.isConnected()) {
			Optional<?> payload = this.receiveMessage(
					Collections.singleton(MissionCurrent.class), false, true, false,
					MavlinkDatalinkProperties.MAVLINK_DEFAULT_SCAN_LIMIT);
			
			if (payload.isPresent()) {
				seq = ((MissionCurrent) payload.get()).seq();
			} else {
				Logging.logger().warning("mavlink datlink received no next mission index");
			}
		} else {
			Logging.logger().warning("mavlink datalink is disconnected");
		}
		
		return seq;
	}
	
	/**
	 * Initiates a take-off for the aircraft connected via this mavlink
	 * datalink.
	 * 
	 * @see Datalink#takeOff()
	 */
	@Override
	public synchronized void takeOff() {
		if (this.isConnected() && !this.isAirborne()) {
			Logging.logger().info("taking off...");
			
			this.enableAircraftGuidance();
			this.armAircraft();
			
			/*
			CommandLong takeoff = CommandLong.builder()
					.targetSystem(this.getTargetId())
					.command(MavCmd.MAV_CMD_NAV_TAKEOFF)
					.build();
			this.sendCommand(takeoff);
			this.receiveCommandAck(MavCmd.MAV_CMD_NAV_TAKEOFF);
			*/
		} else {
			Logging.logger().warning("mavlink datalink is not connected or aircraft airborne");
		}
	}
	
	/**
	 * Initiates a landing for the aircraft connected via this mavlink
	 * datalink.
	 * 
	 * @see Datalink#land()
	 */
	@Override
	public synchronized void land() {
		if (this.isConnected() && this.isAirborne()) {
			Logging.logger().info("landing...");
			
			//SetMode{targetSystem=1, baseMode=EnumValue{value=157, entry=null}, customMode=100925440}
			EnumValue<MavMode> baseMode = EnumValue.create(157);
			SetMode landMode = SetMode.builder()
					.targetSystem(this.getTargetId())
					.baseMode(baseMode)
					.customMode(100925440l)
					.build();
			this.sendCommand(landMode);
			this.receiveCommandAck(MavCmd.MAV_CMD_DO_SET_MODE);
			
			/*
			CommandLong land = CommandLong.builder()
					.targetSystem(this.getTargetId())
					.command(MavCmd.MAV_CMD_DO_LAND_START)
					.build();
			this.sendCommand(land);
			this.receiveCommandAck(MavCmd.MAV_CMD_DO_LAND_START);
			*/
		} else {
			Logging.logger().warning("mavlink datalink is not connected or aircraft not airborne");
		}
	}
	
	/**
	 * Initiates a return to and landing at the launch position for the
	 * aircraft connected via this mavlink datalink.
	 * 
	 * @see Datalink#returnToLaunch()
	 */
	@Override
	public synchronized void returnToLaunch() {
		if (this.isConnected() && this.isAirborne()) {
			Logging.logger().info("returning...");
			
			//SetMode{targetSystem=1, baseMode=EnumValue{value=157, entry=null}, customMode=84148224}
			EnumValue<MavMode> baseMode = EnumValue.create(157);
			SetMode returnMode = SetMode.builder()
					.targetSystem(this.getTargetId())
					.baseMode(baseMode)
					.customMode(84148224l)
					.build();
			this.sendCommand(returnMode);
			this.receiveCommandAck(MavCmd.MAV_CMD_DO_SET_MODE);
			
			/*
			CommandLong rtl = CommandLong.builder()
					.targetSystem(this.getTargetId())
					.command(MavCmd.MAV_CMD_NAV_RETURN_TO_LAUNCH)
					.build();
			this.sendCommand(rtl);
			this.receiveCommandAck(MavCmd.MAV_CMD_NAV_RETURN_TO_LAUNCH);
			*/
		} else {
			Logging.logger().warning("mavlink datalink is not connected or aircraft not airborne");
		}
	}
	
	// TODO: look into other solutions to deal with low frequency messages
	/** the validity period of the airborne state */
	private static final Duration AIRBORNE_STATE_VALIDITY = Duration.ofSeconds(1);
	
	/** the time stamp of the airborne state of this mavlink datalink */
	private ZonedDateTime airborneStateTime = ZonedDateTime.now();
	
	/** the airborne state of this mavlink datalink */
	private boolean airborne = false;
	
	/**
	 * Determines whether or not the current airborne state is valid.
	 * 
	 * @return true if the current airborne state is valid, false otherwise
	 */
	protected boolean isAirborneStateValid() {
		Duration airborneStateAge = Duration.between(
				airborneStateTime, ZonedDateTime.now());
		return (0 > airborneStateAge.compareTo(
				MavlinkDatalink.AIRBORNE_STATE_VALIDITY));
	}
	
	/**
	 * Determines whether or not the aircraft connected via this mavlink
	 * datalink is airborne.
	 * 
	 * @return true if the aircraft is airborne, false otherwise
	 * 
	 * @see Datalink#isAirborne()
	 */
	@Override
	public synchronized boolean isAirborne() {
		if (!this.isAirborneStateValid()) {
			if (this.isConnected()) {
				Optional<?> payload = this.receiveMessage(
						Collections.singleton(ExtendedSysState.class), false, false, true,
						MavlinkDatalinkProperties.MAVLINK_DEFAULT_SCAN_LIMIT);
				
				if (payload.isPresent()) {
					ExtendedSysState ess = (ExtendedSysState) payload.get();
					this.airborneStateTime = ZonedDateTime.now();
					this.airborne = ess.landedState().flagsEnabled(
							MavLandedState.MAV_LANDED_STATE_IN_AIR)
							|| ess.landedState().flagsEnabled(
									MavLandedState.MAV_LANDED_STATE_TAKEOFF)
							|| ess.landedState().flagsEnabled(
									MavLandedState.MAV_LANDED_STATE_LANDING);
				} else {
					Logging.logger().warning("mavlink datalink received no extended system state");
				}
			} else {
				Logging.logger().warning("mavlink datalink is disconnected");
			}
		}
		
		return this.airborne;
	}
	
	/**
	 * Gets the airspeed of the aircraft connected via this mavlink datalink.
	 * 
	 * @return the airspeed of the aircraft connected via this mavlink
	 *         datalink, -1 otherwise
	 * 
	 * @see Datalink#getAirspeed()
	 */
	@Override
	public synchronized int getAirspeed() {
		int airspeed = -1;
		
		if (this.isConnected()) {
			Optional<?> payload = this.receiveMessage(
					Collections.singleton(VfrHud.class), false, true, false,
					MavlinkDatalinkProperties.MAVLINK_DEFAULT_SCAN_LIMIT);
			
			if (payload.isPresent()) {
				airspeed = Math.round(((VfrHud) payload.get()).airspeed());
			} else {
				Logging.logger().warning("mavlink datalink received no airspeed");
			}
		} else {
			Logging.logger().warning("mavlink datalink is disconnected");
		}
		
		return airspeed;
	}
	
	/**
	 * Sets the airspeed of the aircraft connected via this mavlink datalink.
	 * 
	 * @param airspeed the airspeed to be set
	 * 
	 * @see Datalink#setAirspeed(int)
	 */
	@Override
	public synchronized void setAirspeed(int airspeed) {
		if (this.isConnected() && (-1 < airspeed)) {
			CommandLong setAirspeed = CommandLong.builder()
					.targetSystem(this.getTargetId())
					.command(MavCmd.MAV_CMD_DO_CHANGE_SPEED)
					.param1(0f) // airspeed
					.param2(airspeed)
					.build();
			this.sendCommand(setAirspeed);
			this.receiveCommandAck(MavCmd.MAV_CMD_DO_CHANGE_SPEED);
		} else {
			Logging.logger().warning("mavlink datalink is disconnected or airspeed invalid");
		}
	}
	
	/**
	 * Gets the ground speed of the aircraft connected via this mavlink
	 * datalink.
	 * 
	 * @return the ground speed of the aircraft connected via this mavlink
	 *         datalink, -1 otherwise
	 * 
	 * @see Datalink#getGroundSpeed()
	 */
	@Override
	public synchronized int getGroundSpeed() {
		int groundSpeed = -1;
		
		if (this.isConnected()) {
			Optional<?> payload = this.receiveMessage(
					Collections.singleton(VfrHud.class), false, true, false,
					MavlinkDatalinkProperties.MAVLINK_DEFAULT_SCAN_LIMIT);
			
			if (payload.isPresent()) {
				groundSpeed = Math.round(((VfrHud) payload.get()).groundspeed());
			} else {
				Logging.logger().warning("mavlink datalink received no ground speed");
			}
		} else {
			Logging.logger().warning("mavlink datalink is disconnected");
		}
		
		return groundSpeed;
	}
	
	/**
	 * Sets the ground speed of the aircraft connected via this mavlink
	 * datalink.
	 * 
	 * @param groundSpeed the ground speed to be set
	 * 
	 * @see Datalink#setGroundSpeed(int)
	 */
	@Override
	public synchronized void setGroundSpeed(int groundSpeed) {
		if (this.isConnected() && (-1 < groundSpeed)) {
			CommandLong setGroundSpeed = CommandLong.builder()
					.targetSystem(this.getTargetId())
					.command(MavCmd.MAV_CMD_DO_CHANGE_SPEED)
					.param1(1f) // ground speed
					.param2(groundSpeed)
					.build();
			this.sendCommand(setGroundSpeed);
			this.receiveCommandAck(MavCmd.MAV_CMD_DO_CHANGE_SPEED);
		} else {
			Logging.logger().warning("mavlink datalink is disconnected or ground speed invalid");
		}
	}
	
	/**
	 * Gets the climb speed of the aircraft connected via this mavlink
	 * datalink.
	 * 
	 * @return the climb speed of the aircraft connected via this mavlink
	 *         datalink, 0 otherwise
	 * 
	 * @see Datalink#getClimbSpeed()
	 */
	@Override
	public synchronized int getClimbSpeed() {
		int climbSpeed = 0;
		
		if (this.isConnected()) {
			Optional<?> payload = this.receiveMessage(
					Collections.singleton(VfrHud.class), false, true, false,
					MavlinkDatalinkProperties.MAVLINK_DEFAULT_SCAN_LIMIT);
			
			if (payload.isPresent()) {
				climbSpeed = Math.round(((VfrHud) payload.get()).climb());
			} else {
				Logging.logger().warning("mavlink datalink received no climb speed");
			}
		} else {
			Logging.logger().warning("mavlink datalink is disconnected");
		}
		
		return climbSpeed;
	}
	
	/**
	 * Sets the climb speed of the aircraft connected via this mavlink
	 * datalink.
	 * 
	 * @param climbSpeed the climb speed to be set
	 * 
	 * @see Datalink#setClimbSpeed(int)
	 */
	@Override
	public synchronized void setClimbSpeed(int climbSpeed) {
		if (this.isConnected() && (-1 < climbSpeed)) {
			CommandLong setClimbSpeed = CommandLong.builder()
					.targetSystem(this.getTargetId())
					.command(MavCmd.MAV_CMD_DO_CHANGE_SPEED)
					.param1(2f) // climb speed
					.param2(climbSpeed)
					.build();
			this.sendCommand(setClimbSpeed);
			this.receiveCommandAck(MavCmd.MAV_CMD_DO_CHANGE_SPEED);
		} else {
			Logging.logger().warning("mavlink datalink is disconnected or climb speed invalid");
		}
	}
	
	/**
	 * Gets the descent speed of the aircraft connected via this mavlink
	 * datalink.
	 * 
	 * @return the descent speed of the aircraft connected via this mavlink
	 *         datalink, 0 otherwise
	 * 
	 * @see Datalink#getDescentSpeed()
	 */
	@Override
	public synchronized int getDescentSpeed() {
		int descentSpeed = 0;
		
		if (this.isConnected()) {
			Optional<?> payload = this.receiveMessage(
					Collections.singleton(VfrHud.class), false, true, false,
					MavlinkDatalinkProperties.MAVLINK_DEFAULT_SCAN_LIMIT);
			
			if (payload.isPresent()) {
				descentSpeed = -Math.round(((VfrHud) payload.get()).climb());
			} else {
				Logging.logger().warning("mavlink datalink received no airspeed");
			}
		} else {
			Logging.logger().warning("mavlink datalink is disconnected");
		}
		
		return descentSpeed;
	}
	
	/**
	 * Sets the descent speed of the aircraft connected via this mavlink
	 * datalink.
	 * 
	 * @param descentSpeed the descent speed to be set
	 * 
	 * @see Datalink#setDescentSpeed(int)
	 */
	@Override
	public synchronized void setDescentSpeed(int descentSpeed) {
		if (this.isConnected() && (-1 < descentSpeed)) {
			CommandLong setDescentSpeed = CommandLong.builder()
					.targetSystem(this.getTargetId())
					.command(MavCmd.MAV_CMD_DO_CHANGE_SPEED)
					.param1(3f) // descent speed
					.param2(descentSpeed)
					.build();
			this.sendCommand(setDescentSpeed);
			this.receiveCommandAck(MavCmd.MAV_CMD_DO_CHANGE_SPEED);
		} else {
			Logging.logger().warning("mavlink datalink is disconnected or descent speed invalid");
		}
	}
	
	/**
	 * Uploads a mission flight path to the aircraft connected via this mavlink
	 * datalink.
	 * 
	 * @param mission the mission flight path to be uploaded
	 *
	 * @see Datalink#uploadMission(Path)
	 */
	@Override
	public synchronized void uploadMission(Path mission) {
		if (this.isConnected() && (null != mission) && (null != mission.getPositions())) {
			Logging.logger().info("uploading mission...");
			
			ArrayList<Position> positions = new ArrayList<>();
			Iterables.addAll(positions, mission.getPositions());
			int size = positions.size();
			
			// send mission count
			MissionCount count = MissionCount.builder()
					.targetSystem(this.getTargetId())
					.count(size)
					.missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
					.build();
			this.sendCommand(count);
			
			// uploading mission
			HashSet<Class<?>> payloads = new HashSet<>();
			payloads.add(MissionAck.class);
			payloads.add(MissionRequest.class);
			payloads.add(MissionRequestInt.class);
			
			boolean uploading = true;
			while (uploading) {	
				Optional<?> payload = this.receiveMessage(payloads, true, false, false,
						MavlinkDatalinkProperties.MAVLINK_DEFAULT_SCAN_LIMIT);
				
				if (payload.isPresent()) {
					if (payload.get() instanceof MissionAck) {
						MissionAck ack = (MissionAck) payload.get();
						
						if ((this.getSourceId() == ack.targetSystem())
								&& (MavMissionResult.MAV_MISSION_ACCEPTED
										== ack.type().entry())) {
							super.uploadMission(mission);
							uploading = false;
							Logging.logger().info("mavlink datalink successfully uploaded mission");
						} else if (this.getSourceId() == ack.targetSystem()) {
							MissionAck cancel = MissionAck.builder()
									.targetSystem(this.getTargetId())
									.type(MavMissionResult.MAV_MISSION_OPERATION_CANCELLED)
									.missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
									.build();
							this.sendCommand(cancel);
							uploading = false;
							Logging.logger().warning("mavlink datalink failed to upload mission: " + ack);
						}
					} else if (payload.get() instanceof MissionRequest) {
						MissionRequest request = (MissionRequest) payload.get();
						
						if ((this.getSourceId() == request.targetSystem())
								&& (MavMissionType.MAV_MISSION_TYPE_MISSION
										== request.missionType().entry())) {
							Position position = positions.get(request.seq());
							int current = (0 == request.seq()) ? 1 : 0;
							int autocontinue = ((size - 1) == request.seq()) ? 0 : 1;
							
							MissionItemInt item = MissionItemInt.builder()
									.targetSystem(this.getTargetId())
									.seq(request.seq())
									.frame(MavFrame.MAV_FRAME_GLOBAL_INT)
									.command(MavCmd.MAV_CMD_NAV_WAYPOINT)
									.current(current)
									.autocontinue(autocontinue)
									.param4(Float.NaN) // default yaw / heading angle
									.x((int) Math.round(position.getLatitude().getDegrees() / Precision.UNIT_HECTO_NANO))
									.y((int) Math.round(position.getLongitude().getDegrees() / Precision.UNIT_HECTO_NANO))
									.z((float) position.getAltitude())
									.missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
									.build();
							this.sendCommand(item);
						}
					} else if (payload.get() instanceof MissionRequestInt) {
						MissionRequestInt request = (MissionRequestInt) payload.get();
						
						if ((this.getSourceId() == request.targetSystem())
								&& (MavMissionType.MAV_MISSION_TYPE_MISSION
										== request.missionType().entry())) {
							Position position = positions.get(request.seq());
							int current = (0 == request.seq()) ? 1 : 0;
							int autocontinue = ((size - 1) == request.seq()) ? 0 : 1;
							
							MissionItemInt item = MissionItemInt.builder()
									.targetSystem(this.getTargetId())
									.seq(request.seq())
									.frame(MavFrame.MAV_FRAME_GLOBAL_INT)
									.command(MavCmd.MAV_CMD_NAV_WAYPOINT)
									.current(current)
									.autocontinue(autocontinue)
									.param4(Float.NaN) // default yaw / heading angle
									.x((int) Math.round(position.getLatitude().getDegrees() / Precision.UNIT_HECTO_NANO))
									.y((int) Math.round(position.getLongitude().getDegrees() / Precision.UNIT_HECTO_NANO))
									.z((float) position.getAltitude())
									.missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
									.build();
							this.sendCommand(item);
						}
					}
				} else {
					MissionAck cancel = MissionAck.builder()
							.targetSystem(this.getTargetId())
							.type(MavMissionResult.MAV_MISSION_OPERATION_CANCELLED)
							.missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
							.build();
					this.sendCommand(cancel);
					uploading = false;
					Logging.logger().warning("mavlink datalink received no mission PDU");
				}
			}
		} else {
			Logging.logger().warning("mavlink datalink is disconnected or mission invalid");
		}
	}
	
	/**
	 * Downloads a mission flight path from the aircraft connected via this
	 * mavlink datalink.
	 * 
	 * @param cached indicates whether or not the mission cache is used
	 * 
	 * @return the downloaded mission flight path
	 * 
	 * @see Datalink#downloadMission(boolean)
	 */
	@Override
	public synchronized Path downloadMission(boolean cached) {
		Path mission = super.downloadMission(cached);
		
		if (!cached) {
			if (this.isConnected()) {
				Logging.logger().info("downloading mission...");
				
				ArrayList<Position> positions = new ArrayList<>();
				// request mission
				MissionRequestList requestList = MissionRequestList.builder()
					.targetSystem(this.getTargetId())
					.missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
					.build();
				this.sendCommand(requestList);
				
				// downloading mission
				HashSet<Class<?>> payloads = new HashSet<>();
				payloads.add(MissionAck.class);
				payloads.add(MissionCount.class);
				payloads.add(MissionItem.class);
				payloads.add(MissionItemInt.class);
				
				int seq = 0, size = 0;
				boolean downloading = true;
				while (downloading) {	
					Optional<?> payload = this.receiveMessage(payloads, true, false, false,
							MavlinkDatalinkProperties.MAVLINK_DEFAULT_SCAN_LIMIT);
					
					if (payload.isPresent()) {
						if (payload.get() instanceof MissionAck) {
							MissionAck ack = (MissionAck) payload.get();
							
							if ((this.getSourceId() == ack.targetSystem())
									&& (MavMissionResult.MAV_MISSION_ACCEPTED
											!= ack.type().entry())) {
								MissionAck cancel = MissionAck.builder()
										.targetSystem(this.getTargetId())
										.type(MavMissionResult.MAV_MISSION_OPERATION_CANCELLED)
										.missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
										.build();
								this.sendCommand(cancel);
								downloading = false;
								positions.clear();
								Iterables.addAll(positions, this.downloadMission(true).getPositions());
								Logging.logger().warning("mavlink datalink failed to download mission: " + ack);
							}
						} else if (payload.get() instanceof MissionCount) {
							MissionCount count = (MissionCount) payload.get();
							
							if ((this.getSourceId() == count.targetSystem())
									&& (MavMissionType.MAV_MISSION_TYPE_MISSION
											== count.missionType().entry())) {
								size = count.count();
								positions.ensureCapacity(size);
								
								if (seq < size) {
									MissionRequestInt request = MissionRequestInt.builder()
											.targetSystem(this.getTargetId())
											.seq(seq)
											.missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
											.build();
									this.sendCommand(request);
								}
							}
						} else if (payload.get() instanceof MissionItem) {
							MissionItem item = (MissionItem) payload.get();
							
							if ((this.getSourceId() == item.targetSystem())
									&& (seq == item.seq())
									&& (MavFrame.MAV_FRAME_GLOBAL_INT == item.frame().entry())
									&& (MavMissionType.MAV_MISSION_TYPE_MISSION
											== item.missionType().entry())) {
								// TODO: check waypoint type and actions
								Position position = Position.fromDegrees(
										item.x() * Precision.UNIT_HECTO_NANO,
										item.y() * Precision.UNIT_HECTO_NANO,
										item.z());
								positions.add(item.seq(), position);
								
								seq++;
								if (seq < size) {
									MissionRequestInt request = MissionRequestInt.builder()
											.targetSystem(this.getTargetId())
											.seq(seq)
											.missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
											.build();
									this.sendCommand(request);
								} else {
									MissionAck ack = MissionAck.builder()
											.targetComponent(this.getTargetId())
											.type(MavMissionResult.MAV_MISSION_ACCEPTED)
											.missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
											.build();
									this.sendCommand(ack);
									downloading = false;
									Logging.logger().info("mavlink datalink successfully downloaded mission");
								}
							}
						} else if (payload.get() instanceof MissionItemInt) {
							MissionItemInt item = (MissionItemInt) payload.get();
							
							if ((this.getSourceId() == item.targetSystem())
									&& (seq == item.seq())
									&& (MavFrame.MAV_FRAME_GLOBAL_INT == item.frame().entry())
									&& (MavMissionType.MAV_MISSION_TYPE_MISSION
											== item.missionType().entry())) {
								// TODO: check waypoint type and actions
								Position position = Position.fromDegrees(
										item.x() * Precision.UNIT_HECTO_NANO,
										item.y() * Precision.UNIT_HECTO_NANO,
										item.z());
								positions.add(item.seq(), position);
								
								seq++;
								if (seq < size) {
									MissionRequestInt request = MissionRequestInt.builder()
											.targetSystem(this.getTargetId())
											.seq(seq)
											.missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
											.build();
									this.sendCommand(request);
								} else {
									MissionAck ack = MissionAck.builder()
											.targetComponent(this.getTargetId())
											.type(MavMissionResult.MAV_MISSION_ACCEPTED)
											.missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
											.build();
									this.sendCommand(ack);
									downloading = false;
									Logging.logger().info("mavlink datalink successfully downloaded mission");
								}
							}
						}
					} else {
						MissionAck cancel = MissionAck.builder()
								.targetSystem(this.getTargetId())
								.type(MavMissionResult.MAV_MISSION_OPERATION_CANCELLED)
								.missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
								.build();
						this.sendCommand(cancel);
						downloading = false;
						positions.clear();
						Iterables.addAll(positions, this.downloadMission(true).getPositions());
						Logging.logger().warning("mavlink datalink received no mission PDU");
					}
				
					mission = new Path(positions);
				}
			} else {
				Logging.logger().warning("mavlink datalink is disconnected");
			}
		}
		
		return mission;
	}
	
	/**
	 * Determines whether or not this mavlink datalink matches a specification.
	 * 
	 * @param specification the specification to be matched
	 * 
	 * @return true if the this mavlink datalink matches the specification,
	 *         false otherwise
	 * 
	 * @see Datalink#matches(Specification)
	 */
	@Override
	public boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = super.matches(specification);
		
		if (matches && (specification.getProperties() instanceof MavlinkDatalinkProperties)) {
			MavlinkDatalinkProperties properties =
					(MavlinkDatalinkProperties) specification.getProperties();
			matches = this.getHost().equals(properties.getHost())
					&& (this.getPort() == properties.getPort())
					&& (this.getSourceId() == properties.getSourceId())
					&& (this.getTargetId() == properties.getTargetId());
		}
	
		return matches;
	}
	
	/**
	 * Updates this mavlink datalink according to a specification.
	 * 
	 * @param specification the specification to be used for the update
	 * 
	 * @return true if this mavlink datalink has been updated, false otherwise
	 * 
	 * @see Datalink#update(Specification)
	 */
	@Override
	public boolean update(Specification<? extends FactoryProduct> specification) {
		boolean updated = super.update(specification);
		
		if (updated && (specification.getProperties() instanceof MavlinkDatalinkProperties)) {
			MavlinkDatalinkProperties properties =
					(MavlinkDatalinkProperties) specification.getProperties();
			if (!this.isConnected()) {
				this.host = properties.getHost();
				this.port = properties.getPort();
				this.sourceId = properties.getSourceId();
				this.targetId = properties.getTargetId();
				this.getAircraftTrack().setName(this.getTargetId()
						+ "@" + this.getHost() + ":" + this.getPort());
			} else {
				updated = false;
			}
		}
		
		return updated;
	}
	
}
