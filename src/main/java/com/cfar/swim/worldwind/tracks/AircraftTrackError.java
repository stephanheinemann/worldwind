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
 * Realizes an aircraft track error consisting of a cross track error, an
 * altitude error, a bearing error and a timing error.
 * 
 * @author Stephan Heinemann
 *
 */
public class AircraftTrackError {
	
	/** the zero aircraft track error */
	public static final AircraftTrackError ZERO = AircraftTrackError.zeroAircraftTrackError();
	
	/** the maximum aircraft track error */
	public static final AircraftTrackError MAX = AircraftTrackError.maxAircraftTrackError();
	
	/** the cross track error of this aircraft track error */
	private double crossTrackError = 0d; // [m]
	
	/** the altitude error of this aircraft track error */
	private double altitudeError = 0d; // [m]
	
	/** the opening bearing error of this aircraft track error */
	private Angle openingBearingError = Angle.ZERO; // [deg]
	
	/** the closing bearing error of this aircraft track error */
	private Angle closingBearingError = Angle.ZERO; // [deg]
	
	/** the timing error of this aircraft track error */
	private Duration timingError = Duration.ZERO;
	
	/**
	 * Creates a zero aircraft track error.
	 * 
	 * @return a zero aircraft track error
	 */
	public static AircraftTrackError zeroAircraftTrackError() {
		return new AircraftTrackError();
	}
	
	/**
	 * Creates a maximum aircraft track error.
	 * 
	 * @return a maximum aircraft track error
	 */
	public static AircraftTrackError maxAircraftTrackError() {
		AircraftTrackError maxAircraftTrackError = new AircraftTrackError();
		maxAircraftTrackError.setCrossTrackError(Double.POSITIVE_INFINITY);
		maxAircraftTrackError.setAltitudeError(Double.POSITIVE_INFINITY);
		maxAircraftTrackError.setOpeningBearingError(Angle.POS90);
		maxAircraftTrackError.setClosingBearingError(Angle.POS90);
		maxAircraftTrackError.setTimingError(Duration.ofSeconds(Long.MAX_VALUE));
		return maxAircraftTrackError;
	}
	
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
	 * 
	 * @throws IllegalArgumentException if the cross track error is negative
	 */
	public void setCrossTrackError(double crossTrackError) {
		if (0d > crossTrackError) {
			throw new IllegalArgumentException("invalid cross track error");
		}
		this.crossTrackError = crossTrackError;
	}
	
	/**
	 * Gets the altitude error of this aircraft track error.
	 * 
	 * @return the altitude error of this aircraft track error
	 */
	public double getAltitudeError() {
		return this.altitudeError;
	}
	
	/**
	 * Sets the altitude error of this aircraft track error.
	 * 
	 * @param altitudeError the altitude error to be set
	 * 
	 * @throws IllegalArgumentException if the altitude error is negative
	 */
	public void setAltitudeError(double altitudeError) {
		if (0d > altitudeError) {
			throw new IllegalArgumentException("invalid altitude error");
		}
		this.altitudeError = altitudeError;
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
	 * 
	 * @throws IllegalArgumentException if the opening bearing error is null
	 */
	public void setOpeningBearingError(Angle openingBearingError) {
		if (null == openingBearingError) {
			throw new IllegalArgumentException("invalid opening bearing error");
		}
		this.openingBearingError = Angle.clamp(
				openingBearingError, Angle.ZERO, Angle.POS90);
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
	 * 
	 * @throws IllegalArgumentException if the closing bearing error is null
	 */
	public void setClosingBearingError(Angle closingBearingError) {
		if (null == closingBearingError) {
			throw new IllegalArgumentException("invalid closing bearing error");
		}
		this.closingBearingError = Angle.clamp(
				closingBearingError, Angle.ZERO, Angle.POS90);
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
	 * 
	 * @throws IllegalArgumentException if the timing error is null or negative
	 */
	public void setTimingError(Duration timingError) {
		if ((null == timingError) || timingError.isNegative()) {
			throw new IllegalArgumentException("invalid timing error");
		}
		this.timingError = timingError;
	}
	
}
