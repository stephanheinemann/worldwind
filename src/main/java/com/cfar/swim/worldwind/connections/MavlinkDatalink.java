package com.cfar.swim.worldwind.connections;

import com.cfar.swim.worldwind.tracks.AircraftTrackPoint;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.util.Logging;
import io.dronefleet.mavlink.MavlinkConnection;
import io.dronefleet.mavlink.MavlinkConnection.Builder;;


public class MavlinkDatalink extends Datalink {
	
	private boolean isConnected = false;
	
	@Override
	public String getId() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void connect() {
		// TODO Auto-generated method stub
		Logging.logger().info("connecting...");
		Builder builder = MavlinkConnection.builder(null, null);
		this.isConnected = true;
	}

	@Override
	public void disconnect() {
		// TODO Auto-generated method stub
		Logging.logger().info("disconnecting...");
		this.isConnected = false;
	}

	@Override
	public boolean isConnected() {
		// TODO Auto-generated method stub
		return this.isConnected;
	}

	@Override
	public String getAircraftStatus() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public String getAircraftMode() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void setAircraftMode(String aircraftMode) {
		// TODO Auto-generated method stub

	}

	@Override
	public Angle getAircraftHeading() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Angle getAircraftPitch() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Angle getAircraftBank() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Angle getAircraftYaw() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Position getAircraftPosition() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public AircraftTrackPoint getAircraftTrackPoint() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void enableAircraftSafety() {
		// TODO Auto-generated method stub

	}

	@Override
	public void disableAircraftSafety() {
		// TODO Auto-generated method stub

	}

	@Override
	public boolean isAircraftSafetyEnabled() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void armAircraft() {
		// TODO Auto-generated method stub

	}

	@Override
	public void disarmAircraft() {
		// TODO Auto-generated method stub

	}

	@Override
	public boolean isAircraftArmed() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public Position getNextMissionPosition() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public int getNextMissionPositionIndex() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void takeOff() {
		// TODO Auto-generated method stub

	}

	@Override
	public void land() {
		// TODO Auto-generated method stub

	}

	@Override
	public void returnToLaunch() {
		// TODO Auto-generated method stub

	}

	@Override
	public boolean isAirborne() {
		// TODO Auto-generated method stub
		return false;
	}

}
