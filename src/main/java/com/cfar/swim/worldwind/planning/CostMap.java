package com.cfar.swim.worldwind.planning;

import java.util.HashMap;

public class CostMap extends HashMap<String, Integer> {

	private static final long serialVersionUID = 1L;

	public CostMap() {
		this.put("http://codes.wmo.int/49-2/SigWxPhenomena/OBSC_TS", 100);
	}
	
}
