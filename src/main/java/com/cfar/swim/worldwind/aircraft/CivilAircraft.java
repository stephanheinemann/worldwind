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
package com.cfar.swim.worldwind.aircraft;

import com.cfar.swim.worldwind.util.Depiction;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.symbology.SymbologyConstants;
import gov.nasa.worldwind.symbology.milstd2525.MilStd2525TacticalSymbol;

/**
 * Abstracts a civil aircraft.
 * 
 * @author Stephan Heinemann
 *
 */
public abstract class CivilAircraft extends Aircraft {
	
	/** the symbol identification code of an unknown civil aircraft */
	public static final String SIDC_CIV_AIRCRAFT_UNKOWN = "SUAPC----------";
	
	/** the symbol identification code of a friendly civil aircraft */
	public static final String SIDC_CIV_AIRCRAFT_FRIEND = "SFAPC----------";
	
	/** the symbol identification code of a neutral civil aircraft */
	public static final String SIDC_CIV_AIRCRAFT_NEUTRAL = "SNAPC----------";
	
	/** the symbol identification code of a hostile civil aircraft */
	public static final String SIDC_CIV_AIRCRAFT_HOSTILE = "SHAPC----------";
	
	/**
	 * Constructs a new civil aircraft at a specified position with a specified
	 * separation radius and combat identification.
	 * 
	 * @param position the position
	 * @param radius the separation radius
	 * @param cid the combat identification
	 */
	public CivilAircraft(Position position, double radius, CombatIdentification cid) {
		super(position, radius, cid);
		this.depiction = new Depiction(new MilStd2525TacticalSymbol(this.getSymbolIdentifier(cid), position));
		this.getAttributes().setInteriorMaterial(this.getMaterial(cid));
		
		// TODO: use actual live data for symbol annotations...
		this.getDepiction().setModifier(SymbologyConstants.ALTITUDE_DEPTH, position.getAltitude());
	}
	
	/**
	 * Gets the symbol identification of this civil aircraft from a specified
	 * combat identification.
	 * 
	 * @return the symbol identification of this civil aircraft
	 * 
	 * @see Aircraft#getSymbolIdentifier(CombatIdentification)
	 */
	@Override
	protected String getSymbolIdentifier(CombatIdentification cid) {
		switch (cid) {
		case UNKNOWN:
			return CivilAircraft.SIDC_CIV_AIRCRAFT_UNKOWN;
		case FRIEND:
			return CivilAircraft.SIDC_CIV_AIRCRAFT_FRIEND;
		case NEUTRAL:
			return CivilAircraft.SIDC_CIV_AIRCRAFT_NEUTRAL;
		case HOSTILE:
			return CivilAircraft.SIDC_CIV_AIRCRAFT_HOSTILE;
		default:
			return CivilAircraft.SIDC_CIV_AIRCRAFT_UNKOWN;
		}
	}

}
