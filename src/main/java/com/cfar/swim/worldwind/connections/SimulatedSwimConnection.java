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

import java.io.IOException;
import java.net.URISyntaxException;
import java.net.URL;
import java.nio.file.FileSystem;
import java.nio.file.FileSystemNotFoundException;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collections;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

import javax.xml.bind.JAXBException;

import org.xml.sax.InputSource;

import com.cfar.swim.worldwind.data.SwimData;
import com.cfar.swim.worldwind.iwxxm.IwxxmLoader;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.connections.SimulatedSwimConnectionProperties;
import com.cfar.swim.worldwind.render.Obstacle;

/**
 * Realizes a simulated SWIM connection
 * 
 * @author Stephan Heinemann
 *
 */
public class SimulatedSwimConnection extends SwimConnection {
	
	/** the AIXM directory name */
	public static final String AIXM_DIRECTORY = "aixm";
	
	/** the AMXM directory name */
	public static final String AMXM_DIRECTORY = "amxm";
	
	/** the FIXM directory name */
	public static final String FIXM_DIRECTORY = "fixm";
	
	/** the IWXXM directory name */
	public static final String IWXXM_DIRECTORY = "iwxxm";
	
	/** the WXXM directory name */
	public static final String WXXM_DIRECTORY = "wxxm";
	
	/** the resource directory of this simulated SWIM connection */
	private String resourceDirectory; 
	
	/** the file system of the resource directory */
	private FileSystem resourceFileSystem;
	
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
		this.resourceDirectory = SimulatedSwimConnectionProperties.SWIM_RESOURCE_DIRECTORY;
		this.updatePeriod = SimulatedSwimConnectionProperties.SWIM_UPDATE_PERIOD;
		this.updateProbability = SimulatedSwimConnectionProperties.SWIM_UPDATE_PROBABILITY;
		this.updateQuantity = SimulatedSwimConnectionProperties.SWIM_UPDATE_QUANTITY;
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
			String resourceDirectory,
			long updatePeriod,
			float updateProbability,
			int updateQuantity) {
		
		this.resourceDirectory = resourceDirectory;
		this.updatePeriod = updatePeriod;
		this.updateProbability = updateProbability;
		this.updateQuantity = updateQuantity;
	}
	
	/**
	 * Connects this simulated SWIM connection.
	 */
	@Override
	public void connect() {
		if (!this.isConnected()) {
			URL directoryURL = this.getClass().getClassLoader().getResource(resourceDirectory);
			
			if (null != directoryURL) {
				Path swimDirectory = null;
				try {
					// use platform file system for file URIs
					swimDirectory = Paths.get(directoryURL.toURI());
					this.resourceFileSystem = null;
				} catch (URISyntaxException use) {
					use.printStackTrace();
				} catch (FileSystemNotFoundException fsnfe) {
					// create file system for jar URIs
					try {
						this.resourceFileSystem = FileSystems.newFileSystem(
								directoryURL.toURI(), Collections.emptyMap());
						swimDirectory = Paths.get(directoryURL.toURI());
					} catch (URISyntaxException use) {
						use.printStackTrace();
					} catch (IOException ioe) {
						ioe.printStackTrace();
					}	
				}
				
				if ((null != swimDirectory)
						&& (Files.isDirectory(swimDirectory, LinkOption.NOFOLLOW_LINKS))) {
					this.executor = Executors.newSingleThreadScheduledExecutor();
					this.executor.scheduleAtFixedRate(
							new SimulatedSwimLoader(swimDirectory),
							0, this.updatePeriod, TimeUnit.MILLISECONDS);
				}
			}
		}
	}
	
	/**
	 * Disconnects this simulated SWIM connection.
	 */
	@Override
	public void disconnect() {
		if (this.isConnected()) {
			this.executor.shutdown();
			try {
				this.executor.awaitTermination(this.updatePeriod, TimeUnit.MILLISECONDS);
			} catch (InterruptedException ie) {
				ie.printStackTrace();
			}
			this.executor = null;
			try {
				if (null != this.resourceFileSystem) {
					this.resourceFileSystem.close();
				}
			} catch (IOException ioe) {
				ioe.printStackTrace();
			}
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
					&& (specification.getId().equals(Specification.CONNECTION_SWIM_SIMULATED_ID));
		}
		
		return matches;
	}
	
	/**
	 * Realizes a simulated SWIM loader.
	 * 
	 * @author Stephan Heinemann
	 */
	private class SimulatedSwimLoader implements Runnable {

		/** the SWIM resource directory of this simulated SWIM loader */
		private Path swimDirectory;
		
		/**
		 * Constructs a new simulated SWIM loader with a specified SWIM
		 * resource directory.
		 * 
		 * @param swimDirectory the SWIM resource directory
		 */
		public SimulatedSwimLoader(Path swimDirectory) {
			this.swimDirectory = swimDirectory;
		}
		
		/**
		 * Runs the simulated SWIM loader.
		 */
		@Override
		public void run() {
			if (hasSubscribed(SwimData.IWXXM)) {
				Path iwxxmDirectory = swimDirectory.resolve(SimulatedSwimConnection.IWXXM_DIRECTORY);
				try {
					IwxxmLoader loader = new IwxxmLoader();
					Set<Path> iwxxmFiles = Files.list(iwxxmDirectory).collect(Collectors.toSet());
					for (Path iwxxmFile : iwxxmFiles) {
						int updates = 0;
						if ((0f != updateProbability)
								&& (Math.random() <= updateProbability)
								&& (updates < updateQuantity)) {
							updates++;
							Set<Obstacle>obstacles = loader.load(new InputSource(Files.newInputStream(iwxxmFile)));
							if (null != obstacles) {
								// TODO: obstacle manager access should be atomic
								if (hasObstacleManager()) {
									getObstacleManager().submitObstacleChange(obstacles);
									if (getAutoCommit()) {
										getObstacleManager().commitObstacleChange();
									}
								}
							}
						}
					}
				} catch (IOException | JAXBException e) {
					e.printStackTrace();
				}
			}
			// TODO: check other subscriptions
		}
		
	}
	
}
