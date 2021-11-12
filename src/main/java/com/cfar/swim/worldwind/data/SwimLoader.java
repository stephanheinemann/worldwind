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
package com.cfar.swim.worldwind.data;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import javax.xml.bind.JAXBException;

import com.cfar.swim.worldwind.data.fixm.FixmLoader;
import com.cfar.swim.worldwind.data.iwxxm.IwxxmLoader;
import com.cfar.swim.worldwind.render.Obstacle;

/**
 * Realizes a SWIM obstacle loader.
 * 
 * @author Stephan Heinemann
 *
 */
public class SwimLoader implements ObstacleLoader {
	
	/** the supported SWIM loaders of this SWIM loader */
	private HashMap<SwimProtocol, ObstacleLoader> loaders = new HashMap<>();
	
	/**
	 * Constructs a new SWIM obstacle loader.
	 */
	public SwimLoader() {
		try {
			loaders.put(SwimProtocol.FIXM, new FixmLoader());
			loaders.put(SwimProtocol.IWXXM, new IwxxmLoader());
			// TODO: add more protocols
		} catch (JAXBException e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * Loads SWIM obstacles from a SWIM resource.
	 * 
	 * @param resource the SWIM resource
	 * 
	 * @return the loaded obstacles
	 * 
	 * @see ObstacleLoader#load(SwimResource)
	 */
	@Override
	public Set<Obstacle> load(SwimResource resource) {
		Set<Obstacle> obstacles = new HashSet<>();
		
		if (resource.getProtocol().isPresent()) {
			obstacles.addAll(this.loaders.get(resource.getProtocol().get()).load(resource));
		} else {
			for (ObstacleLoader loader : loaders.values()) {
				obstacles.addAll(loader.load(resource));
			}
		}
		
		return obstacles;
	}
	
}
