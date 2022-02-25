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
import gov.nasa.worldwind.geom.Position;

/**
 * Realizes an aircraft track point error consisting of a position error and
 * a timing error.
 * 
 * @author Stephan Heinemann
 *
 */
public class AircraftTrackPointError {
	
	/** the zero aircraft track point error */
	public static final AircraftTrackPointError ZERO = AircraftTrackPointError.zeroAircraftTrackPointError();
	
	/** the maximum aircraft track point error */
	public static final AircraftTrackPointError MAX = AircraftTrackPointError.maxAircraftTrackPointError();
	
	/** the position error of this aircraft track point error */
	private Position positionError = Position.ZERO;
	
	/** the horizontal error of this aircraft track point error */
	private double horizontalError = 0d; // [m]
	
	/** the vertical error of this aircraft track point error */
	private double verticalError = 0d; // [m]
	
	/** the timing error of this aircraft track point error */
	private Duration timingError = Duration.ZERO;
	
	/**
	 * Creates a zero aircraft track point error.
	 * 
	 * @return a zero aircraft track point error
	 */
	public static AircraftTrackPointError zeroAircraftTrackPointError() {
		return new AircraftTrackPointError();
	}
	
	/**
	 * Creates a maximum aircraft track point error.
	 * 
	 * @return a maximum aircraft track point error
	 */
	public static AircraftTrackPointError maxAircraftTrackPointError() {
		AircraftTrackPointError maxAircraftTrackPointError = new AircraftTrackPointError();
		maxAircraftTrackPointError.setPositionError(
				new Position(Angle.POS180, Angle.POS180, Double.POSITIVE_INFINITY));
		maxAircraftTrackPointError.setHorizontalError(Double.POSITIVE_INFINITY);
		maxAircraftTrackPointError.setVerticalError(Double.POSITIVE_INFINITY);
		maxAircraftTrackPointError.setTimingError(Duration.ofSeconds(Long.MAX_VALUE));
		return maxAircraftTrackPointError;
	}
	
	/**
	 * Gets the position error of this aircraft track point error.
	 * 
	 * @return the position error of this aircraft track point error
	 */
	public Position getPositionError() {
		return positionError;
	}
	
	/**
	 * Sets the position error of this aircraft track point error.
	 * 
	 * @param positionError the position error to be set
	 * 
	 * @throws IllegalArgumentException if the position error is null or negative
	 */
	public void setPositionError(Position positionError) {
		if ((null == positionError)
				|| (0d > positionError.getElevation())
				|| (0d > positionError.getLatitude().compareTo(Angle.ZERO))
				|| (0d > positionError.getLongitude().compareTo(Angle.ZERO))) {
			throw new IllegalArgumentException("invalid position error");
		}
		this.positionError = positionError;
	}
	
	/**
	 * Gets the horizontal error of this aircraft track point error.
	 * 
	 * @return the horizontal error of this aircraft track point error
	 */
	public double getHorizontalError() {
		return horizontalError;
	}
	
	/**
	 * Sets the horizontal error of this aircraft track point error.
	 * 
	 * @param horizontalError the horizontal error to be set
	 *
	 * @throws IllegalArgumentException if the horizontal error is negative
	 */
	public void setHorizontalError(double horizontalError) {
		if (0d > horizontalError) {
			throw new IllegalArgumentException("invalid horizontal error");
		}
		this.horizontalError = horizontalError;
	}
	
	/**
	 * Gets the vertical error of this aircraft track point error.
	 * 
	 * @return the vertical error of this aircraft track point error
	 */
	public double getVerticalError() {
		return verticalError;
	}
	
	/**
	 * Sets the vertical error of this aircraft track point error.
	 * 
	 * @param verticalError the vertical error to be set
	 * 
	 * @throws IllegalArgumentException if the vertical error is negative
	 */
	public void setVerticalError(double verticalError) {
		if (0d > verticalError) {
			throw new IllegalArgumentException("invalid vertical error");
		}
		this.verticalError = verticalError;
	}
	
	/**
	 * Gets the timing error of this aircraft track point error.
	 * 
	 * @return the timing error of this aircraft track point error
	 */
	public Duration getTimingError() {
		return timingError;
	}
	
	/**
	 * Sets the timing error of this aircraft track point error.
	 * 
	 * @param timingError the timing error to be set
	 * 
	 * @throws IllegalArgumentException if the timing error is null or negative
	 */
	public void setTimingError(Duration timingError) {
		if ((null == timingError) || (timingError.isNegative())) {
			throw new IllegalArgumentException("invalid timing error");
		}
		this.timingError = timingError;
	}
	
}
