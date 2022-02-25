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
import gov.nasa.worldwind.symbology.milstd2525.MilStd2525TacticalSymbol;

/**
 * Abstracts a fixed wing civil aircraft.
 * 
 * @author Stephan Heinemann
 *
 */
public abstract class FixedWingCivilAircraft extends CivilAircraft {

	/** the symbol identification code of an unknown fixed wing civil aircraft */
	public static final String SIDC_CIV_FW_AIRCRAFT_UNKOWN = "SUAPCF---------"; 
	
	/** the symbol identification code of a friendly fixed wing civil aircraft */
	public static final String SIDC_CIV_FW_AIRCRAFT_FRIEND = "SFAPCF---------";
	
	/** the symbol identification code of a neutral fixed wing civil aircraft */
	public static final String SIDC_CIV_FW_AIRCRAFT_NEUTRAL = "SNAPCF---------";
	
	/** the symbol identification code of a hostile fixed wing civil aircraft */
	public static final String SIDC_CIV_FW_AIRCRAFT_HOSTILE = "SHAPCF---------";
	
	/**
	 * Constructs a new fixed wing civil aircraft at a specified position with a
	 * specified separation radius and combat identification.
	 * 
	 * @param position the position
	 * @param radius the separation radius
	 * @param cid the combat identification
	 */
	public FixedWingCivilAircraft(Position position, double radius, CombatIdentification cid) {
		super(position, radius, cid);
		this.depiction = new Depiction(new MilStd2525TacticalSymbol(this.getSymbolIdentifier(cid), position));
		
		// TODO: use actual live data for symbol annotations...
	}
	
	/**
	 * Gets the symbol identification of this fixed wing civil aircraft from a
	 * specified combat identification.
	 * 
	 * @return the symbol identification of this fixed wing civil aircraft
	 * 
	 * @see CivilAircraft#getSymbolIdentifier(CombatIdentification)
	 */
	@Override
	protected String getSymbolIdentifier(CombatIdentification cid) {
		switch (cid) {
		case UNKNOWN:
			return FixedWingCivilAircraft.SIDC_CIV_FW_AIRCRAFT_UNKOWN;
		case FRIEND:
			return FixedWingCivilAircraft.SIDC_CIV_FW_AIRCRAFT_FRIEND;
		case NEUTRAL:
			return FixedWingCivilAircraft.SIDC_CIV_FW_AIRCRAFT_NEUTRAL;
		case HOSTILE:
			return FixedWingCivilAircraft.SIDC_CIV_FW_AIRCRAFT_HOSTILE;
		default:
			return FixedWingCivilAircraft.SIDC_CIV_FW_AIRCRAFT_UNKOWN;
		}
	}
	
}
