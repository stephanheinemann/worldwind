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

import java.time.Duration;

import gov.nasa.worldwind.geom.Angle;

/**
 * Realizes an aircraft track error consisting of a cross track error, a
 * bearing error and a timing error.
 * 
 * @author Stephan Heinemann
 *
 */
public class AircraftTrackError {
	
	/** the zero aircraft track error */
	public static final AircraftTrackError ZERO = new AircraftTrackError();
	
	/** the cross track error of this aircraft track error */
	private double crossTrackError = 0d; // [m]
	
	/** the opening bearing error of this aircraft track error */
	private Angle openingBearingError = Angle.ZERO; // [deg]
	
	/** the closing bearing error of this aircraft track error */
	private Angle closingBearingError = Angle.ZERO; // [deg]
	
	/** the timing error of this aircraft track error */
	private Duration timingError = Duration.ZERO;
	
	/**
	 * Gets the cross track error of this aircraft track error.
	 * 
	 * @return the cross track error of this aircraft track error
	 */
	public double getCrossTrackError() {
		return this.crossTrackError;
	}
	
	/**
	 * Sets the cross track error of this aircraft track error.
	 * 
	 * @param crossTrackError the cross track error to be set
	 */
	public void setCrossTrackError(double crossTrackError) {
		this.crossTrackError = crossTrackError;
	}
	
	/**
	 * Gets the opening bearing error of this aircraft track error.
	 * 
	 * @return the opening bearing error of this aircraft track error
	 */
	public Angle getOpeningBearingError() {
		return this.openingBearingError;
	}
	
	/**
	 * Sets the opening bearing error of this aircraft track error.
	 * 
	 * @param openingBearingError the bearing error to be set
	 */
	public void setOpeningBearingError(Angle openingBearingError) {
		this.openingBearingError = openingBearingError;
	}
	
	/**
	 * Gets the closing bearing error of this aircraft track error.
	 * 
	 * @return the closing bearing error of this aircraft track error
	 */
	public Angle getClosingBearingError() {
		return this.closingBearingError;
	}
	
	/**
	 * Sets the closing bearing error of this aircraft track error.
	 * 
	 * @param closingBearingError the bearing error to be set
	 */
	public void setClosingBearingError(Angle closingBearingError) {
		this.closingBearingError = closingBearingError;
	}
	
	/**
	 * Gets the timing error of this aircraft track error.
	 * 
	 * @return the timing error of this aircraft track error
	 */
	public Duration getTimingError() {
		return timingError;
	}
	
	/**
	 * Sets the timing error of this aircraft track error.
	 * 
	 * @param timingError the timing error to be set
	 */
	public void setTimingError(Duration timingError) {
		this.timingError = timingError;
	}
	
}
