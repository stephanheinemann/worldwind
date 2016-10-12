package com.cfar.swim.worldwind.aircraft;

import com.cfar.swim.worldwind.util.Depiction;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.symbology.milstd2525.MilStd2525TacticalSymbol;

public abstract class FixedWingCivilAircraft extends CivilAircraft {

	public static final String SIDC_CIV_FW_AIRCRAFT_UNKOWN = "SUAPCF---------"; 
	public static final String SIDC_CIV_FW_AIRCRAFT_FRIEND = "SFAPCF---------";
	public static final String SIDC_CIV_FW_AIRCRAFT_NEUTRAL = "SNAPCF---------";
	public static final String SIDC_CIV_FW_AIRCRAFT_HOSTILE = "SHAPCF---------";
	
	public FixedWingCivilAircraft(Position position, double radius, CombatIdentification cid) {
		super(position, radius, cid);
		this.depiction = new Depiction(new MilStd2525TacticalSymbol(this.getSymbolIdentifier(cid), position));
		
		// TODO: use actual live data for symbol annotations...
	}

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
