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
import com.cfar.swim.droneconnect.Mode;
import com.cfar.swim.droneconnect.Null;
import com.cfar.swim.droneconnect.Safety;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.Path;
import io.grpc.Channel;
import io.grpc.netty.NegotiationType;
import io.grpc.netty.NettyChannelBuilder;
import io.grpc.stub.StreamObserver;

public class DronekitConnection extends AircraftConnection {
	
	private String host;
	private int port;
	
	private Channel channel;
	private DroneConnectBlockingStub blockingStub;
	private DroneConnectStub stub;
	
	public DronekitConnection(String host, int port) {
		this.host = host;
		this.port = port;
		this.channel = null;
		this.blockingStub = null;
	}
	
	@Override
	public void connect() {
		this.channel = NettyChannelBuilder.forAddress(this.host, this.port)
				.negotiationType(NegotiationType.PLAINTEXT)
				.build();
		this.blockingStub = DroneConnectGrpc.newBlockingStub(this.channel);
		this.stub = DroneConnectGrpc.newStub(this.channel);
	}
	
	@Override
	public void disconnect() {
		this.channel = null;
		this.blockingStub = null;
	}
	
	@Override
	public boolean isConnected() {
		return (null != this.blockingStub);
	}
	
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
	
	@Override
	public void setAircraftMode(String aircraftMode) {
		if (this.isConnected()) {
			Mode mode = Mode.newBuilder().setMode(aircraftMode).build();
			blockingStub.setMode(mode);
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
	}
	
	@Override
	public void enableAircraftSafety() {
		if (this.isConnected()) {
			Safety safety = Safety.newBuilder().setSafety(true).build();
			blockingStub.setSafety(safety);
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
	}
	
	@Override
	public void disableAircraftSafety() {
		if (this.isConnected()) {
			Safety safety = Safety.newBuilder().setSafety(false).build();
			blockingStub.setSafety(safety);
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
	}
	
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
	
	@Override
	public void armAircraft() {
		if (this.isConnected()) {
			Armed armed = Armed.newBuilder().setArm(true).build();
			blockingStub.setArmed(armed);
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
	}
	
	@Override
	public void disarmAircraft() {
		if (this.isConnected()) {
			Armed armed = Armed.newBuilder().setArm(false).build();
			blockingStub.setArmed(armed);
		} else {
			throw new IllegalStateException("dronekit is not connected");
		}
	}
	
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
	
	@Override
	public void uploadPath(Path path) {
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
	
}
