package com.cfar.swim.worldwind.aircraft;

import com.cfar.swim.worldwind.util.Depiction;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.symbology.SymbologyConstants;
import gov.nasa.worldwind.symbology.milstd2525.MilStd2525TacticalSymbol;

public abstract class CivilAircraft extends Aircraft {
	
	public static final String SIDC_CIV_AIRCRAFT_UNKOWN = "SUAPC----------"; 
	public static final String SIDC_CIV_AIRCRAFT_FRIEND = "SFAPC----------";
	public static final String SIDC_CIV_AIRCRAFT_NEUTRAL = "SNAPC----------";
	public static final String SIDC_CIV_AIRCRAFT_HOSTILE = "SHAPC----------";
	
	public CivilAircraft(Position position, double radius, CombatIdentification cid) {
		super(position, radius);
		this.depiction = new Depiction(new MilStd2525TacticalSymbol(this.getSymbolIdentifier(cid), position));
		this.getAttributes().setMaterial(this.getMaterial(cid));
		
		// TODO: use actual live data for symbol annotations...
		this.getDepiction().setModifier(SymbologyConstants.ALTITUDE_DEPTH, position.getAltitude());
	}

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
