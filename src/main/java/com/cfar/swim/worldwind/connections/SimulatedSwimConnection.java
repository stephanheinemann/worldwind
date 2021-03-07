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

import java.net.URI;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class SimulatedSwimConnection extends SwimConnection {

	/** the resource directory of this simulated SWIM connection */
	private URI resourceDirectory = URI.create("classpath:xml/iwxxm/");
	
	/** the update period of this simulated SWIM connection */
	private long updatePeriod = 5000; // ms
	
	/** the update probability of this simulated SWIM connection */
	private float updateProbability = 0.5f;
	
	/** the update quantity of this simulated SWIM connection */
	private int updateQuantity = 1;
	
	/** the executor of this simulated SWIM connection */
	private ScheduledExecutorService executor = null;
	
	public SimulatedSwimConnection() {}
	
	public SimulatedSwimConnection(
			URI resourceDirectory,
			long updatePeriod,
			float updateProbability,
			int updateQuantity) {
		
		this.resourceDirectory = resourceDirectory;
		this.updatePeriod = updatePeriod;
		this.updateProbability = updateProbability;
		this.updateQuantity = updateQuantity;
		
		// load updateQuantity files to be updated
	}
	
	
	@Override
	public void connect() {
		this.executor = Executors.newSingleThreadScheduledExecutor();
		this.executor.scheduleAtFixedRate(new Runnable() {

			@Override
			public void run() {
				System.out.println("running simulated SWIM connection");
				// load files (IWXXM loader)
				// add / enable / disable obstacles in associated scenario
			}}, 0, this.updatePeriod, TimeUnit.MILLISECONDS);
	}

	@Override
	public void disconnect() {
		this.executor.shutdown();
		// TODO: possibly await termination
	}

	@Override
	public boolean isConnected() {
		return !((null == this.executor) || this.executor.isShutdown());
	}
	
}
