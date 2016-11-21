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
package com.cfar.swim.worldwind.aircraft;

import com.cfar.swim.worldwind.render.airspaces.ObstacleSphere;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.render.Material;
import gov.nasa.worldwind.symbology.milstd2525.MilStd2525Constants;

public abstract class Aircraft extends ObstacleSphere {

	protected Capabilities capabilities = null;
	
	public Aircraft(Position position, double radius) {
		super(position, radius);
	}

	public Capabilities getCapabilities() {
		return this.capabilities;
	}
	
	protected abstract String getSymbolIdentifier(CombatIdentification cid);
	
	protected Material getMaterial(CombatIdentification cid) {
		switch(cid) {
		case UNKNOWN:
			return MilStd2525Constants.MATERIAL_UNKNOWN;
		case FRIEND:
			return MilStd2525Constants.MATERIAL_FRIEND;
		case NEUTRAL:
			return MilStd2525Constants.MATERIAL_NEUTRAL;
		case HOSTILE:
			return MilStd2525Constants.MATERIAL_HOSTILE;
		default:
			return MilStd2525Constants.MATERIAL_UNKNOWN;
		}
	}
	
	@Override
	protected void updateVisibility() {
		super.updateVisibility();
	}
	
}
