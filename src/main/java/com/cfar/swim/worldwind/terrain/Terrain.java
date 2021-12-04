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
package com.cfar.swim.worldwind.terrain;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.net.URI;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

import gov.nasa.worldwind.globes.ElevationModel;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.terrain.CompoundElevationModel;
import gov.nasa.worldwind.terrain.LocalElevationModel;

/**
 * Realizes a terrain consisting of elevation tiles.
 * 
 * @author Stephan Heinemann
 *
 */
public class Terrain extends LocalElevationModel {
	
	/** the cache of terrain names and their associated tiles */
	private static Map<String, LocalTile> terrainCache = new HashMap<>();
	
	/** the terrain names and their associated tiles of this terrain */
	private Map<String, LocalTile> terrain = new HashMap<>();
	
	/** the last added tile of this terrain */
	private LocalTile addedTile = null;
	
	/**
	 * Empties the terrain cache.
	 */
	public static void emptyCache() {
		Terrain.terrainCache.clear();
	}
	
	/**
	 * Adds a terrain tile to this terrain.
	 * 
	 * @param terrain the terrain file containing the terrain tile to be added
	 * 
	 * @return true if the terrain tile has been added, false otherwise
	 */
	public boolean add(File terrain) {
		boolean added = false;
		
		if (!this.terrain.containsKey(terrain.getName())) {
			if (Terrain.terrainCache.containsKey(terrain.getName())) {
				this.tiles.add(Terrain.terrainCache.get(terrain.getName()));
				this.adjustMinMax(Terrain.terrainCache.get(terrain.getName()));
				this.terrain.put(terrain.getName(),
						Terrain.terrainCache.get(terrain.getName()));
				added = true;
			} else {
				try {
					this.addElevations(terrain);
					if (null != this.addedTile) {
						Terrain.terrainCache.put(terrain.getName(), this.addedTile);
						this.terrain.put(terrain.getName(), this.addedTile);
						added = true;
					}
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}
		
		return added;
	}
	
	/**
	 * Removes a terrain tile from this terrain.
	 * 
	 * @param terrain the name of the terrain tile to be removed
	 * 
	 * @return true if the terrain tile has been removed, false otherwise
	 */
	public boolean remove(String terrain) {
		boolean removed = false;
		
		if (this.terrain.containsKey(terrain)) {
			removed = this.tiles.remove(this.terrain.get(terrain));
			this.adjustMinMax(null);
			this.terrain.remove(terrain);
		}
		
		return removed;
	}
	
	/**
	 * Clears the terrain tiles of this terrain.
	 */
	public void clear() {
		this.tiles.clear();
		this.adjustMinMax(null);
		this.terrain.clear();
	}
	
	/**
	 * Gets the tile names of this terrain.
	 * 
	 * @return the tile names of this terrain
	 */
	public Set<String> getNames() {
		return new TreeSet<String>(this.terrain.keySet());
	}
	
	/**
	 * Determines whether or not this terrain is empty.
	 * 
	 * @return true if this terrain is empty, false otherwise
	 */
	public boolean isEmpty() {
		return this.terrain.isEmpty();
	}
	
	/**
	 * Adjusts the minimum and maximum elevations of this terrain after adding
	 * a terrain tile.
	 * 
	 * @param tile the added terrain tile
	 */
	@Override
	protected void adjustMinMax(LocalTile tile) {
		super.adjustMinMax(tile);
		if (null != tile) {
			this.addedTile = tile;
		}
	}
	
	// TODO: consider configuration for WMS available at
	// https://datacube.services.geo.ca/ows/elevation
	
	/** the terrain resources of this terrain */
	public static final String[] TERRAIN_RESOURCES = {
			"dtm_1m_utm10_w_2_137.tif",
			"dtm_1m_utm10_w_2_138.tif",
			"dtm_1m_utm10_w_3_137.tif",
			"dtm_1m_utm10_w_3_138.tif"
	};
	
	/** the workspace resource of this terrain */
	public static final URI WORKSPACE_RESOURCE = URI.create("file:///var/tmp/safcs/terrain");
	
	/**
	 * Loads all local terrain resources into a local elevation model.
	 * 
	 * @return the local elevation model
	 */
	public static LocalElevationModel loadTerrain() {
		LocalElevationModel elevationModel = new LocalElevationModel();
		
		for (String terrainResource : TERRAIN_RESOURCES) {
			InputStream resourceStream = Terrain.class.getResourceAsStream(terrainResource);
			
			try {
				Files.createDirectories(Path.of(Terrain.WORKSPACE_RESOURCE));
				Path target = Path.of(Terrain.WORKSPACE_RESOURCE.getPath(), terrainResource);
				Files.copy(resourceStream, target, StandardCopyOption.REPLACE_EXISTING);
				elevationModel.addElevations(target.toFile());
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		
		return elevationModel;
	}
	
	/**
	 * Imports all local terrain resources into the elevation model of a globe.
	 * 
	 * @param globe the globe featuring a compound elevation model
	 */
	public static void importTerrain(Globe globe) {
		ElevationModel elevationModel = globe.getElevationModel();
		
		if (elevationModel instanceof CompoundElevationModel) {
			((CompoundElevationModel) elevationModel).addElevationModel(Terrain.loadTerrain());
		} else {
			globe.setElevationModel(Terrain.loadTerrain());
		}
	}
	
}
