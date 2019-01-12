/**
 * Copyright (c) 2018, Henrique Ferreira (UVic Center for Aerospace Research)
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
package com.cfar.swim.worldwind.registries.planners;


/**
 * Realizes the properties bean of a OFAD PRM planner.
 * 
 * @author Henrique Ferreira
 *
 */
public class OFADPRMProperties extends FADPRMProperties implements OnlinePlannerProperties {
	
	/** the description of this planner properties bean */
	private final static String DESCRIPTION_OFADPRM = "Online Flexible Anytime Dynamic PRM: Flexible online version of PRM with anytime and dynamic capabilities. Requires a datalink connection.";
	
	/** the state of the online capabilities of the planner */
	private boolean online = true;
	
	/** the distance threshold to consider a position displacement as worthy of a new plan */
	private double positionThreshold = 2d; 

	/**
	 * Constructs a new online flexible anytime dynamic PRM planner properties bean.
	 */
	public OFADPRMProperties() {
		super();
		this.setDescription(DESCRIPTION_OFADPRM);
		this.setOnline(true);
		this.setPositionThreshold(2d);
	}

	/**
	 * Checks if the online capabilities of the planner mode are active or not.
	 * 
	 * @return true if the planner mode is set to online, false otherwise
	 */
	public boolean isOnline() {
		return online;
	}
	
	/**
	 * Sets the online capabilities of the planner as are active or not.
	 * 
	 * @param online the state of the online capabilities
	 */
	public void setOnline(boolean online) {
		this.online = online;
	}

	/**
	 * Gets the distance threshold to consider a position displacement as worthy of a new plan.
	 * 
	 * @return the distance threshold for each position
	 */
	public double getPositionThreshold() {
		return positionThreshold;
	}

	/**
	 * Sets the distance threshold to consider a position displacement as worthy of a new plan.
	 * 
	 * @param positionThreshold the distance threshold for each position
	 */
	public void setPositionThreshold(double positionThreshold) {
		this.positionThreshold = positionThreshold;
	}

}
