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
package com.cfar.swim.worldwind.tracks;

import gov.nasa.worldwind.geom.Angle;

/**
 * Realizes an aircraft attitude obtained via a datalink downlink.
 * 
 * @author Stephan Heinemann
 *
 */
public class AircraftAttitude {
	
	/** the pitch angle of this aircraft attitude */
	private Angle pitch = Angle.ZERO;
	
	/** the bank angle of this aircraft attitude */
	private Angle bank = Angle.ZERO;
	
	/** the heading angle of this aircraft attitude */
	private Angle heading = Angle.ZERO;
	
	/**
	 * Constructs a new aircraft attitude with a specified pitch, bank, and
	 * heading angle.
	 *  
	 * @param pitch the pitch angle of this aircraft attitude
	 * @param bank the bank angle of this aircraft attitude
	 * @param heading the heading angle of this aircraft attitude
	 */
	public AircraftAttitude(Angle pitch, Angle bank, Angle heading) {
		this.pitch = pitch;
		this.bank = bank;
		this.heading = heading;
	}
	
	/**
	 * Gets the pitch angle of this aircraft attitude.
	 * 
	 * @return the pitch angle of this aircraft attitude
	 */
	public Angle getPitch() {
		return pitch;
	}
	
	/**
	 * Sets the pitch angle of this aircraft attitude.
	 * 
	 * @param pitch the pitch angle to be set
	 */
	public void setPitch(Angle pitch) {
		this.pitch = pitch;
	}
	
	/**
	 * Gets the bank angle of this aircraft attitude.
	 * 
	 * @return the bank angle of this aircraft attitude
	 */
	public Angle getBank() {
		return bank;
	}
	
	/**
	 * Sets the bank angle of this aircraft attitude.
	 * 
	 * @param bank the bank angle to be set
	 */
	public void setBank(Angle bank) {
		this.bank = bank;
	}
	
	/**
	 * Gets the heading angle of this aircraft attitude.
	 * 
	 * @return the heading angle of this aircraft attitude
	 */
	public Angle getHeading() {
		return heading;
	}
	
	/**
	 * Sets the heading angle of this aircraft attitude.
	 * 
	 * @param heading the heading angle to be set
	 */
	public void setHeading(Angle heading) {
		this.heading = heading;
	}
	
	/**
	 * Gets the string representation of this aircraft attitude.
	 * 
	 * @return the string representation of this aircraft attitude
	 */
	public String toString() {
		return "(" + this.getPitch() + ", "
				+ this.getBank() + ", "
				+ this.getHeading() + ")";
	}
	
}
