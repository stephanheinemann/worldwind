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

import java.io.File;
import java.net.URI;
import java.net.URL;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import javax.xml.bind.JAXBException;

import com.cfar.swim.worldwind.data.SwimData;
import com.cfar.swim.worldwind.iwxxm.IwxxmLoader;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.connections.SimulatedSwimConnectionProperties;

/**
 * Realizes a simulated SWIM connection
 * 
 * @author Stephan Heinemann
 *
 */
public class SimulatedSwimConnection extends SwimConnection {

	/** the resource directory of this simulated SWIM connection */
	private URI resourceDirectory; 
	
	/** the update period of this simulated SWIM connection */
	private long updatePeriod;
	
	/** the update probability of this simulated SWIM connection */
	private float updateProbability;
	
	/** the update quantity of this simulated SWIM connection */
	private int updateQuantity;
	
	/** the executor of this simulated SWIM connection */
	private ScheduledExecutorService executor;
	
	/**
	 * Constructs a default simulated SWIM connection.
	 */
	public SimulatedSwimConnection() {
		this.resourceDirectory = URI.create("classpath:xml/iwxxm/");
		this.updatePeriod = 5000; // ms
		this.updateProbability = 0.5f;
		this.updateQuantity = 1;
		this.executor = null;
	}
	
	/**
	 * Constructs a simulated SWIM connection with a specified resource
	 * directory, update period, probability and quantity.
	 * 
	 * @param resourceDirectory the resource directory
	 * @param updatePeriod the update period
	 * @param updateProbability the update probability
	 * @param updateQuantity the update quantity
	 */
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
	
	/**
	 * Connects this simulated SWIM connection.
	 */
	@Override
	public void connect() {
		if (!this.isConnected()) {
			this.executor = Executors.newSingleThreadScheduledExecutor();
			this.executor.scheduleAtFixedRate(new Runnable() {
	
				@Override
				public void run() {
					System.out.println("running simulated SWIM connection");
					
					//resourceDirectory.toURL().getFile();
					
					//URL resourceURL = this.getClass().getClassLoader().getResource("/home/stephan/");
					//System.out.println(resourceURL.getFile());
					/*
					File directory = new File("/home/stephan");
					if (directory.isDirectory()) {
						for (String filename : directory.list())
							System.out.println(filename);
					}
					*/
					
					/*
					try {
						IwxxmLoader loader = new IwxxmLoader();
					} catch (JAXBException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					*/
					// load files (IWXXM loader)
					// add / enable / disable obstacles in associated scenario
					if (hasSubscribed(SwimData.IWXXM)) {
						System.out.println("IWXXM has been subscribed");
					}
					// ...
				}}, 0, this.updatePeriod, TimeUnit.MILLISECONDS);
		}
	}
	
	/**
	 * Disconnects this simulated SWIM connection.
	 */
	@Override
	public void disconnect() {
		if (this.isConnected()) {
			this.executor.shutdown();
			this.executor = null;
		}
	}
	
	/**
	 * Determines whether or not this simulated SWIM connection is connected.
	 * 
	 * @return true if this simulated SWIM connection is connected,
	 *         false otherwise
	 */
	@Override
	public boolean isConnected() {
		return (null != this.executor);
	}

	/**
	 * Determines whether or not this simulated SWIM connection matches a
	 * specification.
	 * 
	 * @param specification the specification to be matched
	 * 
	 * @return true if the this simulated SWIM connection matches the
	 *         specification, false otherwise
	 * 
	 * @see SwimConnection#matches(Specification)
	 */
	@Override
	public final boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = super.matches(specification);
		
		if (matches && (specification.getProperties() instanceof SimulatedSwimConnectionProperties)) {
			SimulatedSwimConnectionProperties sscp = (SimulatedSwimConnectionProperties) specification.getProperties();
			matches = (this.resourceDirectory.equals(sscp.getResourceDirectory())
					&& (this.updatePeriod == sscp.getUpdatePeriod())
					&& (this.updateProbability == sscp.getUpdateProbability())
					&& (this.updateQuantity == sscp.getUpdateQuantity()))
					&& (specification.getId().equals(Specification.SWIM_SIMULATED));
		}
		
		return matches;
	}
	
}
