package com.cfar.swim.worldwind.tests;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import com.cfar.swim.worldwind.connections.SimulatedSwimConnection;

public class SwimConnectionTest {

	@Test
	public void testSimulatedConnection() {
		SimulatedSwimConnection ssc = new SimulatedSwimConnection();
		ssc.connect();
		assertTrue(ssc.isConnected());
		Thread.yield();
		// TODO: test SWIM request listener
		ssc.disconnect();
		assertFalse(ssc.isConnected());
	}
	
}
