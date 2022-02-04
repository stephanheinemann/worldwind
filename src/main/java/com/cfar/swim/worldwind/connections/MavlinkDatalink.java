package com.cfar.swim.worldwind.connections;

import java.io.EOFException;
import java.io.IOException;
import java.net.Socket;
import java.time.Duration;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.Optional;

import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.tracks.AircraftAttitude;
import com.cfar.swim.worldwind.tracks.AircraftTrackPoint;
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
import io.dronefleet.mavlink.common.GlobalPositionInt;
import io.dronefleet.mavlink.common.Heartbeat;
import io.dronefleet.mavlink.common.MavCmd;
import io.dronefleet.mavlink.common.MavFrame;
import io.dronefleet.mavlink.common.MavMissionResult;
import io.dronefleet.mavlink.common.MavMissionType;
import io.dronefleet.mavlink.common.MavMode;
import io.dronefleet.mavlink.common.MavModeFlag;
import io.dronefleet.mavlink.common.MavState;
import io.dronefleet.mavlink.common.MissionAck;
import io.dronefleet.mavlink.common.MissionCount;
import io.dronefleet.mavlink.common.MissionCurrent;
import io.dronefleet.mavlink.common.MissionItem;
import io.dronefleet.mavlink.common.MissionItemInt;
import io.dronefleet.mavlink.common.MissionRequest;
import io.dronefleet.mavlink.common.MissionRequestInt;
import io.dronefleet.mavlink.common.MissionRequestList;
import io.dronefleet.mavlink.common.SetMode;
import io.dronefleet.mavlink.common.UtmDataAvailFlags;
import io.dronefleet.mavlink.common.UtmFlightState;
import io.dronefleet.mavlink.common.UtmGlobalPosition;
import io.dronefleet.mavlink.util.EnumValue;


@SuppressWarnings("deprecation")
public class MavlinkDatalink extends Datalink {
	
	private static final int MESSAGE_SCAN_LIMIT = 1000;
	
	private Duration uplinkDelay = Duration.ofMillis(100l);
	
	/** the remote host of this mavlink datalink */
	private String host = "172.16.104.128";
	
	/** the remote port of this mavlink datalink */
	private int port = 14550;
	
	// TODO: system id (254 - different from QGC), component id (1 - in general)
	
	private int targetId = 1;
	private int sourceId = 255;
	
	private boolean isConnected = false;
	
	private Socket tcp = new Socket();
	
	MavlinkConnection mavlink = null;
	
	public MavlinkDatalink(/* host, port, id */) {
		this.getAircraftTrack().setName(this.getHost() + ":" + this.getPort());
	}
	
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
	
	public int getSourceId() {
		return this.sourceId;
	}
	
	public int getTargetId() {
		return this.targetId;
	}
	
	/**
	 * Gets the uplink delay of this mavlink datalink.
	 * 
	 * @return the uplink delay of this mavlink datalink
	 */
	public Duration getUplinkDelay() {
		return this.uplinkDelay;
	}
	
	/**
	 * Sets the uplink delay of this mavlink datalink.
	 * 
	 * @param uplinkDelay the uplink delay to be set
	 * 
	 * @throws IllegalArgumentException if the uplink delay is invalid
	 */
	public void setUplinkDelay(Duration uplinkDelay) {
		if ((null == uplinkDelay) ||  uplinkDelay.isZero()) {
			throw new IllegalArgumentException("uplink delay is invalid");
		}
		this.uplinkDelay = uplinkDelay;
	}
	
	@Override
	public void connect() {
		Logging.logger().info("connecting mavlink...");
		try {
			this.tcp = new Socket(this.getHost(), this.getPort());
			this.mavlink = MavlinkConnection.create(
		            this.tcp.getInputStream(), 
		            this.tcp.getOutputStream());
			this.isConnected = true;
		} catch (Exception e) {
			e.printStackTrace();
			this.disconnect();
		}
	}
	
	@Override
	public void disconnect() {
		Logging.logger().info("disconnecting mavlink...");
		try {
			this.tcp.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		this.mavlink = null;
		this.isConnected = false;
	}
	
	@Override
	public boolean isConnected() {
		return this.isConnected;
	}
	
	private void sniff(int systemId, int componentId, Collection<Class<?>> payloadClasses, int messageLimit) {
		int receivedMessages = 0;
		
		while (receivedMessages < messageLimit) {
			try {
				MavlinkMessage<?> message = this.mavlink.next();
			
				// TODO: possibly forward QGC traffic
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
	
	private void sendCommand(Object command) {
		try {
			this.mavlink.send1(this.getSourceId(), 0, command);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	private Optional<?> receiveMessage(
			Collection<Class<?>> payloadClasses, boolean delay,
			boolean skip, int messageLimit) {
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
		
		// skip old messages
		if (skip) {
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
		
		return payload;
	}
	
	private Optional<CommandAck> receiveCommandAck(MavCmd command) {
		Optional<CommandAck> ack = Optional.empty();
		
		Optional<?> payload = this.receiveMessage(
				Collections.singleton(CommandAck.class), true, false,
				MavlinkDatalink.MESSAGE_SCAN_LIMIT);
		
		if (payload.isPresent()) {
			CommandAck commandAck = (CommandAck) payload.get();
			if ((this.getSourceId() == commandAck.targetSystem())
					&& (command == commandAck.command().entry())) {
				ack = Optional.of(commandAck);
				Logging.logger().info(
						"mavlink datalink received command ack result "
						+ commandAck.result().entry().name() + " for "
						+ command.name());
			} else {
				// TODO: possibly retry for correct command ack
				Logging.logger().warning(
						"mavlink datalink received incorrect command ack for "
						+ command.name() + ": "
						+ commandAck.command().entry().name());
			}
		} else {
			Logging.logger().warning(
					"mavlink datalink received no command ack for "
					+ command.name());
		}
		
		return ack;
	}
	
	private Optional<MissionAck> receiveMissionAck() {
		Optional<MissionAck> ack = Optional.empty();
		
		Optional<?> payload = this.receiveMessage(
				Collections.singleton(MissionAck.class), true, false,
				MavlinkDatalink.MESSAGE_SCAN_LIMIT);
		
		if (payload.isPresent()) {
			MissionAck missionAck = (MissionAck) payload.get();
			if ((this.getSourceId() == missionAck.targetSystem())) {
				ack = Optional.of(missionAck);
				Logging.logger().info(
						"mavlink datalink received mission ack "
						+ missionAck.type().entry().name());
			} else {
				// TODO: possibly retry for correct mission ack
				Logging.logger().warning(
						"mavlink datalink received incorrect misison ack");
			}
		} else {
			Logging.logger().warning(
					"mavlink datalink received no mission ack");
		}
		
		return ack;
	}
	
	@Override
	public synchronized String getAircraftStatus() {
		String status = null;
		
		if (this.isConnected()) {
			Optional<?> payload = this.receiveMessage(
					Collections.singleton(Heartbeat.class),
					false, true, MavlinkDatalink.MESSAGE_SCAN_LIMIT);
			
			if (payload.isPresent()) {
				EnumValue<MavState> mavState =
						((Heartbeat) payload.get()).systemStatus();
				status = mavState.entry().name();
			} else {
				Logging.logger().warning(
						"mavlink datalink received no status");
			}
		} else {
			Logging.logger().warning("mavlink datalink is disconnected");
		}
		
		return status;
	}
	
	@Override
	public synchronized String getAircraftMode() {
		String mode = null;
		
		if (this.isConnected()) {
			Optional<?> payload = this.receiveMessage(
					Collections.singleton(Heartbeat.class),
					false, true, MavlinkDatalink.MESSAGE_SCAN_LIMIT);
			
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
	
	@Override
	public synchronized void setAircraftMode(String aircraftMode) {
		if (this.isConnected()) {
			Logging.logger().info("setting mode " + aircraftMode + "...");
			CommandLong mode = CommandLong.builder()
					.command(MavCmd.MAV_CMD_DO_SET_MODE)
					.targetSystem(this.getTargetId())
					.param1(MavMode.valueOf(aircraftMode).ordinal())
					.build();
			this.sendCommand(mode);
			this.receiveCommandAck(MavCmd.MAV_CMD_DO_SET_MODE);
		} else {
			Logging.logger().warning("mavlink datalink is disconnected");
		}
	}
	
	public synchronized AircraftAttitude getAircraftAttitude() {
		AircraftAttitude attitude = null;
		
		if (this.isConnected()) {
			Optional<?> payload = this.receiveMessage(
					Collections.singleton(Attitude.class),
					false, true, MavlinkDatalink.MESSAGE_SCAN_LIMIT);
			
			if (payload.isPresent()) {
				Attitude att = (Attitude) payload.get();
				attitude = new AircraftAttitude(
						Angle.fromRadians(att.pitch()),
						Angle.fromRadians(att.roll()),
						Angle.fromRadians(att.yaw()));
			} else {
				Logging.logger().warning(
						"mavlink datalink received no attitude");
			}
		} else {
			Logging.logger().warning("mavlink datalink is disconnected");
		}
		
		return attitude;
	}
	
	@Override
	public synchronized Angle getAircraftPitch() {
		Angle pitch = null;
				
		AircraftAttitude attitude = this.getAircraftAttitude();
		if (null != attitude) {
			pitch = attitude.getPitch();
		}
		
		return pitch;
	}
	
	@Override
	public synchronized Angle getAircraftBank() {
		Angle bank = null;
		
		AircraftAttitude attitude = this.getAircraftAttitude();
		if (null != attitude) {
			bank = attitude.getBank();
		}
		
		return bank;
	}
	
	@Override
	public synchronized Angle getAircraftYaw() {
		Angle yaw = null;
		
		AircraftAttitude attitude = this.getAircraftAttitude();
		if (null != attitude) {
			yaw = attitude.getHeading();
		}
		
		return yaw;
	}
	
	@Override
	public synchronized Angle getAircraftHeading() {
		Angle heading = null;
		
		if (this.isConnected()) {
			Optional<?> payload = this.receiveMessage(
					Collections.singleton(GlobalPositionInt.class),
					false, true, MavlinkDatalink.MESSAGE_SCAN_LIMIT);
			
			if (payload.isPresent()) {
				heading = Angle.fromDegrees(
						((GlobalPositionInt) payload.get()).hdg() * 1E-2d);
			} else {
				Logging.logger().warning(
						"mavlink datlink received no heading");
			}
		} else {
			Logging.logger().warning("mavlink datalink is disconnected");
		}
		
		return heading;
	}
	
	@Override
	public synchronized Position getAircraftPosition() {
		Position position = null;
		
		if (this.isConnected()) {
			Optional<?> payload = this.receiveMessage(
					Collections.singleton(GlobalPositionInt.class),
					false, true, MavlinkDatalink.MESSAGE_SCAN_LIMIT);
			
			if (payload.isPresent()) {
				GlobalPositionInt gpi = (GlobalPositionInt) payload.get();
				position = Position.fromDegrees(
						gpi.lat() * 1E-7d,
						gpi.lon() * 1E-7d,
						gpi.alt() * 1E-3d);
			} else {
				Logging.logger().warning(
						"mavlink datlink received no position");
			}
		} else {
			Logging.logger().warning("mavlink datalink is disconnected");
		}
		
		return position;
	}
	
	@Override
	public synchronized AircraftTrackPoint getAircraftTrackPoint() {
		AircraftTrackPoint trackPoint = null;
		
		Position position = this.getAircraftPosition();
		AircraftAttitude attitude = this.getAircraftAttitude();
		
		if ((null != position) && (null != attitude)) {
			trackPoint = new AircraftTrackPoint(position, attitude);
		} else if ((null != position)) {
			trackPoint = new AircraftTrackPoint(position);
		}
		
		/*
		ArrayList<Class<?>> payloadClasses = new ArrayList<>();
		payloadClasses.add(CommandAck.class);
		payloadClasses.add(CommandInt.class);
		payloadClasses.add(CommandLong.class);
		payloadClasses.add(SetMode.class);
		payloadClasses.add(MissionRequest.class);
		payloadClasses.add(MissionRequestInt.class);
		payloadClasses.add(MissionAck.class);
		payloadClasses.add(MissionCount.class);
		payloadClasses.add(MissionRequestList.class);
		payloadClasses.add(MissionItem.class);
		payloadClasses.add(MissionItemInt.class);
		this.sniff(255, 0, payloadClasses, 1000);
		*/
		
		//Logging.logger().info("next mission position index = " + this.getNextMissionPositionIndex());
		//Logging.logger().info("next mission position = " + this.getNextMissionPosition());
		//Logging.logger().info("airborne = " + this.isAirborne());
		
		return trackPoint;
	}
	
	@Override
	public synchronized void enableAircraftSafety() {
		if (!this.isAircraftSafetyEnabled()) {
			Logging.logger().info("enabling safety...");
			this.setAircraftMode(MavMode.MAV_MODE_GUIDED_DISARMED.name());
		}
	}
	
	@Override
	public synchronized void disableAircraftSafety() {
		//if (this.isAircraftSafetyEnabled()) {
			Logging.logger().info("disabling safety...");
			this.setAircraftMode(MavMode.MAV_MODE_GUIDED_ARMED.name());
		//}
	}
	
	@Override
	public synchronized boolean isAircraftSafetyEnabled() {
		boolean safetyEnabled = false;
		
		String mode = this.getAircraftMode();
		if (null != mode) {
			safetyEnabled = mode.contains(
					MavModeFlag.MAV_MODE_FLAG_SAFETY_ARMED.name());
		}
		
		return safetyEnabled;
	}
	
	@Override
	public synchronized void armAircraft() {
		if (this.isConnected() /*&& !this.isAircraftArmed()*/) {
			Logging.logger().info("arming...");
			
			CommandInt arm = CommandInt.builder()
				.command(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM)
				.targetSystem(this.getTargetId())
				.param1(1f) // arm
				.param2(0f) // perform safety checks
				.build();
			this.sendCommand(arm);
			this.receiveCommandAck(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM);
		}
	}
	
	@Override
	public synchronized void disarmAircraft() {
		if (this.isConnected() && this.isAircraftArmed() && !this.isAirborne()) {
			CommandInt disarm = CommandInt.builder()
					.command(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM)
					.targetSystem(this.getTargetId())
					.param1(0f) // disarm
					.param2(0f) // perform safety checks
					.build();
			this.sendCommand(disarm);
			this.receiveCommandAck(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM);
		}
	}
	
	@Override
	public synchronized boolean isAircraftArmed() {
		return !this.isAircraftSafetyEnabled();
	}
	
	@Override
	public synchronized Position getNextMissionPosition() {
		Position next = null;
		
		if (this.isConnected()) {
			// TODO: another option is to download the mission (or use the cache)
			// in combination with the sequence number
			Optional<?> payload = this.receiveMessage(
					Collections.singleton(UtmGlobalPosition.class),
					false, true, MavlinkDatalink.MESSAGE_SCAN_LIMIT);
			if (payload.isPresent()) {
				UtmGlobalPosition ugp = (UtmGlobalPosition) payload.get();
				if (ugp.flags().flagsEnabled(
						UtmDataAvailFlags.UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE)
						//&& this.getAircraftMode().contains(
						//		MavModeFlag.MAV_MODE_FLAG_GUIDED_ENABLED.name())
						) {
					next = Position.fromDegrees(
							ugp.nextLat() * 1E-7d,
							ugp.nextLon() * 1E-7d,
							ugp.nextAlt() * 1E-3d);
				} else {
					Logging.logger().warning("mavlink datalink received invalid next mission position");
				}
			} else {
				Logging.logger().warning("mavlink datlink received no next mission position");
			}
		} else {
			Logging.logger().warning("mavlink datalink is disconnected");
		}
		
		return next;
	}
	
	@Override
	public synchronized int getNextMissionPositionIndex() {
		int seq = -1;
		
		if (this.isConnected()) {
			Optional<?> payload = this.receiveMessage(
					Collections.singleton(MissionCurrent.class),
					false, true, MavlinkDatalink.MESSAGE_SCAN_LIMIT);
			
			if (payload.isPresent()) {
				seq = ((MissionCurrent) payload.get()).seq();
			} else {
				Logging.logger().warning(
						"mavlink datlink received no next mission index");
			}
		} else {
			Logging.logger().warning("mavlink datalink is disconnected");
		}
		
		return seq;
	}
	
	@Override
	public synchronized void takeOff() {
		/*
		if (this.isConnected() && !this.isAirborne()) {
			Logging.logger().info("taking off...");
			
			CommandLong takeoff = CommandLong.builder()
					.command(MavCmd.MAV_CMD_NAV_TAKEOFF)
					.build();
			this.sendCommand(takeoff);
			this.receiveAck(MavCmd.MAV_CMD_NAV_TAKEOFF);
		}
		*/
	}
	
	@Override
	public synchronized void land() {
		if (this.isConnected() && this.isAirborne()) {
			Logging.logger().info("landing...");
			
			//SetMode{targetSystem=1, baseMode=EnumValue{value=157, entry=null}, customMode=100925440}
			EnumValue<MavMode> baseMode = EnumValue.create(157);
			SetMode landMode = SetMode.builder()
				.baseMode(baseMode)
				.customMode(100925440l)
				.build();
			this.sendCommand(landMode);
			this.receiveCommandAck(MavCmd.MAV_CMD_DO_SET_MODE);
			
			/*
			this.setAircraftMode(MavMode.MAV_MODE_AUTO_ARMED.name());
			CommandLong land = CommandLong.builder()
					.command(MavCmd.MAV_CMD_NAV_LAND)
					.param1(0f)
					.param2(PrecisionLandMode.PRECISION_LAND_MODE_OPPORTUNISTIC.ordinal())
					.build();
			this.receiveAck(MavCmd.MAV_CMD_NAV_LAND);
			*/
			/*
			CommandLong land = CommandLong.builder()
					.command(MavCmd.MAV_CMD_DO_LAND_START)
					.build();
			this.sendCommand(land);
			this.receiveAck(MavCmd.MAV_CMD_DO_LAND_START);
			*/
		}
	}
	
	@Override
	public synchronized void returnToLaunch() {
		if (this.isConnected() && this.isAirborne()) {
			Logging.logger().info("returning...");
			
			//SetMode{targetSystem=1, baseMode=EnumValue{value=157, entry=null}, customMode=84148224}
			EnumValue<MavMode> baseMode = EnumValue.create(157);
			SetMode returnMode = SetMode.builder()
				.baseMode(baseMode)
				.customMode(84148224l)
				.build();
			this.sendCommand(returnMode);
			this.receiveCommandAck(MavCmd.MAV_CMD_DO_SET_MODE);
			
			/*
			CommandLong rtl = CommandLong.builder()
					.command(MavCmd.MAV_CMD_DO_SET_MODE)
					.param1(157f)
					.param2(84148224f)
					.build();
			this.sendCommand(rtl);
			this.receiveAck(MavCmd.MAV_CMD_DO_SET_MODE);
			*/
			/*
			this.setAircraftMode(MavMode.MAV_MODE_AUTO_ARMED.name());
			CommandLong rtl = CommandLong.builder()
					.command(MavCmd.MAV_CMD_NAV_RETURN_TO_LAUNCH)
					.build();
			this.sendCommand(rtl);
			this.receiveAck(MavCmd.MAV_CMD_NAV_RETURN_TO_LAUNCH);
			*/
		}
	}
	
	@Override
	public synchronized boolean isAirborne() {
		boolean airborne = false;
		
		//ExtendedSysState.class
		//MavLandedState.MAV_LANDED_STATE_IN_AIR
		if (this.isConnected()) {
			Optional<?> payload = this.receiveMessage(
					Collections.singleton(UtmGlobalPosition.class),
					false, true, MavlinkDatalink.MESSAGE_SCAN_LIMIT);
			if (payload.isPresent()) {
				UtmGlobalPosition ugp = (UtmGlobalPosition) payload.get();
				if (UtmFlightState.UTM_FLIGHT_STATE_AIRBORNE == ugp.flightState().entry()) {
					airborne = true;
				}
			} else {
				Logging.logger().warning(
						"mavlink datlink received no flight state");
			}
		} else {
			Logging.logger().warning("mavlink datalink is disconnected");
		}
		
		return airborne;
	}
	
	/**
	 * Uploads a mission flight path to the aircraft connected via this
	 * mavlink datalink.
	 * 
	 * @param mission the mission flight path to be uploaded
	 *
	 * @see Datalink#uploadMission(Path)
	 */
	@Override
	public synchronized void uploadMission(Path mission) {
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
			Optional<?> payload = this.receiveMessage(payloads, true, false,
					MavlinkDatalink.MESSAGE_SCAN_LIMIT);
			
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
								.x((int) Math.round(position.getLatitude().getDegrees() * 1E7d))
								.y((int) Math.round(position.getLongitude().getDegrees() * 1E7d))
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
								.x((int) Math.round(position.getLatitude().getDegrees() * 1E7d))
								.y((int) Math.round(position.getLongitude().getDegrees() * 1E7d))
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
				Optional<?> payload = this.receiveMessage(payloads, true, false,
						MavlinkDatalink.MESSAGE_SCAN_LIMIT);
				
				if (payload.isPresent()) {
					if (payload.get() instanceof MissionAck) {
						Logging.logger().info("received mission ack");
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
							Logging.logger().warning("mavlink datalink failed to download mission: " + ack);
						}
					} else if (payload.get() instanceof MissionCount) {
						Logging.logger().info("received mission count");
						MissionCount count = (MissionCount) payload.get();
						
						if ((this.getSourceId() == count.targetSystem())
								&& (MavMissionType.MAV_MISSION_TYPE_MISSION
										== count.missionType().entry())) {
							Logging.logger().info("correct source id and mission type");
							size = count.count();
							positions.ensureCapacity(size);
							
							if (seq < size) {
								Logging.logger().info("seq = " + seq + ", size = " + size);
								MissionRequestInt request = MissionRequestInt.builder()
										.targetSystem(this.getTargetId())
										.seq(seq)
										.missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
										.build();
								this.sendCommand(request);
							}
						}
					} else if (payload.get() instanceof MissionItem) {
						Logging.logger().info("received mission item");
						MissionItem item = (MissionItem) payload.get();
						
						if ((this.getSourceId() == item.targetSystem())
								&& (MavFrame.MAV_FRAME_GLOBAL_INT == item.frame().entry())
								&& (MavMissionType.MAV_MISSION_TYPE_MISSION
										== item.missionType().entry())) {
							Logging.logger().info("correct source id and mission type");
							
							// TODO: check nav waypoint
							Position position = Position.fromDegrees(
									item.x() * 1E-7d,
									item.y() * 1E-7d,
									item.z());
							positions.add(item.seq(), position);
							
							seq++;
							if (seq < size) {
								Logging.logger().info("seq = " + seq + ", size = " + size);
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
						Logging.logger().info("received mission item int");
						MissionItemInt item = (MissionItemInt) payload.get();
						
						if ((this.getSourceId() == item.targetSystem())
								&& (MavFrame.MAV_FRAME_GLOBAL_INT == item.frame().entry())
								&& (MavMissionType.MAV_MISSION_TYPE_MISSION
										== item.missionType().entry())) {
							Logging.logger().info("correct source id and mission type");
							
							// TODO: check nav waypoint
							Position position = Position.fromDegrees(
									item.x() * 1E-7d,
									item.y() * 1E-7d,
									item.z());
							positions.add(item.seq(), position);
							
							seq++;
							if (seq < size) {
								Logging.logger().info("seq = " + seq + ", size = " + size);
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
					Logging.logger().warning("mavlink datalink received no mission PDU");
				}
			
				mission = new Path(positions);
			}
		}
		
		return mission;
	}
	
}
