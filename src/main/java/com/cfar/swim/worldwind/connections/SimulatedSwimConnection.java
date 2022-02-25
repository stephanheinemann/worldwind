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
import java.time.Duration;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.data.SwimLoader;
import com.cfar.swim.worldwind.data.SwimProtocol;
import com.cfar.swim.worldwind.data.SwimResource;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.connections.SimulatedSwimConnectionProperties;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.util.Identifiable;

/**
 * Realizes a simulated SWIM connection
 * 
 * @author Stephan Heinemann
 *
 */
public class SimulatedSwimConnection extends SwimConnection {
	
	/** the resource directory of this simulated SWIM connection */
	private String resourceDirectory; 
	
	/** the file system of the resource directory */
	private FileSystem resourceFileSystem;
	
	/** the update period of this simulated SWIM connection */
	private Duration updatePeriod;
	
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
		this.updatePeriod = Duration.ofMillis(SimulatedSwimConnectionProperties.SWIM_UPDATE_PERIOD);
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
			Duration updatePeriod,
			float updateProbability,
			int updateQuantity) {
		
		this.resourceDirectory = resourceDirectory;
		this.updatePeriod = updatePeriod;
		this.updateProbability = updateProbability;
		this.updateQuantity = updateQuantity;
		this.executor = null;
	}
	
	/**
	 * Gets the identifier of this simulated SWIM connection.
	 * 
	 * @return the identifier of this simulated SWIM connection
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return Specification.CONNECTION_SWIM_SIMULATED_ID;
	}
	
	/**
	 * Gets the resource directory of this simulated SWIM connection.
	 * 
	 * @return the resource directory of this simulated SWIM connection
	 */
	public String getResourceDirectory() {
		return this.resourceDirectory;
	}
	
	/**
	 * Gets the update period of this simulated SWIM connection.
	 * 
	 * @return the update period of this simulated SWIM connection
	 */
	public Duration getUpdatePeriod() {
		return this.updatePeriod;
	}
	
	/**
	 * Gets the update probability of this simulated SWIM connection.
	 * 
	 * @return the update probability of this simulated SWIM connection
	 */
	public float getUpdateProbability() {
		return this.updateProbability;
	}
	
	/**
	 * Gets the update quantity of this simulated SWIM connection.
	 * 
	 * @return the update quantity of this simulated SWIM connection
	 */
	public int getUpdateQuantity() {
		return this.updateQuantity;
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
							0, this.updatePeriod.toMillis(), TimeUnit.MILLISECONDS);
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
				this.executor.awaitTermination(this.updatePeriod.toMillis(), TimeUnit.MILLISECONDS);
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
	public boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = super.matches(specification);
		
		if (matches && (specification.getProperties() instanceof SimulatedSwimConnectionProperties)) {
			SimulatedSwimConnectionProperties properties =
					(SimulatedSwimConnectionProperties) specification.getProperties();
			matches = (this.resourceDirectory.equals(properties.getResourceDirectory())
					&& (this.updatePeriod.equals(Duration.ofMillis(properties.getUpdatePeriod())))
					&& (this.updateProbability == properties.getUpdateProbability())
					&& (this.updateQuantity == properties.getUpdateQuantity()));
		}
		
		return matches;
	}
	
	/**
	 * Updates this simulated SWIM connection according to a specification.
	 * 
	 * @param specification the specification to be used for the update
	 * 
	 * @return true if this simulated SWIM connection has been updated,
	 *         false otherwise
	 * 
	 * @see SwimConnection#update(Specification)
	 */
	@Override
	public boolean update(Specification<? extends FactoryProduct> specification) {
		boolean updated = super.update(specification);
		
		if (updated && (specification.getProperties() instanceof SimulatedSwimConnectionProperties)) {
			SimulatedSwimConnectionProperties properties =
					(SimulatedSwimConnectionProperties) specification.getProperties();
			this.resourceDirectory = properties.getResourceDirectory();
			this.updatePeriod = Duration.ofMillis(properties.getUpdatePeriod());
			this.updateProbability = properties.getUpdateProbability();
			this.updateQuantity = properties.getUpdateQuantity();
		}
		
		return updated;
	}
	
	/**
	 * Realizes a simulated SWIM loader.
	 * 
	 * @author Stephan Heinemann
	 */
	private class SimulatedSwimLoader implements Runnable {
		
		/** the SWIM loader of this simulated SWIM loader */
		private SwimLoader swimLoader = new SwimLoader();
		
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
			HashMap<SwimProtocol, List<Path>> swimFiles = new HashMap<>();
			
			try {
				if (hasSubscribed(SwimProtocol.AIXM)) {
					swimFiles.put(SwimProtocol.AIXM,
							Files.list(this.swimDirectory.resolve(SwimProtocol.AIXM.name().toLowerCase()))
							.collect(Collectors.toList()));
				}
				if (hasSubscribed(SwimProtocol.FIXM)) {
					swimFiles.put(SwimProtocol.FIXM,
							Files.list(this.swimDirectory.resolve(SwimProtocol.FIXM.name().toLowerCase()))
							.collect(Collectors.toList()));
				}
				if (hasSubscribed(SwimProtocol.IWXXM)) {
					swimFiles.put(SwimProtocol.IWXXM,
							Files.list(this.swimDirectory.resolve(SwimProtocol.IWXXM.name().toLowerCase()))
							.collect(Collectors.toList()));
				}
				// TODO: check other subscriptions
			} catch (IOException ioe) {
				ioe.printStackTrace();
			}
			
			int updates = 0;
			List<SwimProtocol> protocols = swimFiles.keySet().stream().collect(Collectors.toList());
			Collections.shuffle(protocols);
			for (SwimProtocol protocol : protocols) {
				Collections.shuffle(swimFiles.get(protocol));
				for (Path swimFile : swimFiles.get(protocol)) {
					if ((0f != updateProbability)
							&& (Math.random() <= updateProbability)
							&& (updates < updateQuantity)) {
						SwimResource resource = new SwimResource(swimFile.toUri(), protocol);
						Set<Obstacle>obstacles = this.swimLoader.load(resource);
						if (!obstacles.isEmpty()) {
							updates++;
							// TODO: obstacle manager access should be atomic
							if (hasObstacleManager()) {
								getObstacleManager().submitAddObstacles(obstacles);
							}
						}
					}
				}
			}
		}
	}
	
}
