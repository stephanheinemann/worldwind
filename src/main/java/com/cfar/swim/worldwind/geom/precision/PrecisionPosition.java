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
package com.cfar.swim.worldwind.geom.precision;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;

/**
 * Realizes a position with a specified numerical precision.
 * 
 * @author Stephan Heinemann
 *
 */
public class PrecisionPosition extends Position implements Precision, Comparable<Position> {

	/**
	 * the precision of this precision position
	 */
	private int precision = Precision.PRECISION;
	
	/**
	 * the altitude precision of this precision position
	 */
	private int altPrecision = Precision.PRECISION;
	
	/**
	 * the original position of this precision position
	 */
	private Position original = null;
	
	/**
	 * Constructs a new precision position from a position using the default
	 * precision.
	 * 
	 * @param position the position
	 * 
	 * @see Precision#PRECISION
	 */
	public PrecisionPosition(Position position) {
		// TODO: the correct default precision should probably be a function (percentage) of the value
		super(
			Angle.fromDegrees((new PrecisionDouble(position.latitude.degrees, Precision.PRECISION)).doubleValue()),
			Angle.fromDegrees((new PrecisionDouble(position.longitude.degrees, Precision.PRECISION)).doubleValue()),
			(new PrecisionDouble(position.elevation, Precision.PRECISION)).doubleValue());
		this.original = position;
	}

	/**
	 * Constructs a new precision position from a position using a specified
	 * precision.
	 * 
	 * @param position the position
	 * @param precision the precision
	 */
	public PrecisionPosition(Position position, int precision) {
		super(
			Angle.fromDegrees((new PrecisionDouble(position.latitude.degrees, precision)).doubleValue()),
			Angle.fromDegrees((new PrecisionDouble(position.longitude.degrees, precision)).doubleValue()),
			(new PrecisionDouble(position.elevation, precision)).doubleValue());
		this.precision = precision;
		this.altPrecision = precision;
		this.original = position;
	}
	
	/**
	 * Constructs a new precision position from a position using specified
	 * location and altitude precisions.
	 * 
	 * @param position the position
	 * @param locPrecision the location precision
	 * @param altPrecision the altitude precision
	 */
	public PrecisionPosition(Position position, int locPrecision, int altPrecision) {
		super(
			Angle.fromDegrees((new PrecisionDouble(position.latitude.degrees, locPrecision)).doubleValue()),
			Angle.fromDegrees((new PrecisionDouble(position.longitude.degrees, locPrecision)).doubleValue()),
			(new PrecisionDouble(position.elevation, altPrecision)).doubleValue());
		this.precision = locPrecision;
		this.altPrecision = altPrecision;
		this.original = position;
	}
	
	/**
	 * Gets the location precision of this precision position.
	 * 
	 * @return the location precision of this precision position
	 * 
	 * @see Precision#getPrecision()
	 */
	@Override
	public int getPrecision() {
		return this.precision;
	}
	
	/**
	 * Sets the precision of this precision position.
	 * 
	 * @param precision the precision to be set
	 * 
	 * @return a new precision position with the set precision
	 * 
	 * @see Precision#setPrecision(int)
	 */
	@Override
	public PrecisionPosition setPrecision(int precision) {
		return new PrecisionPosition(this.original, precision);
	}
	
	/**
	 * Gets the altitude precision of this precision position.
	 * 
	 * @return the altitude precision of this precision position
	 */
	public int getAltitudePrecision() {
		return this.altPrecision;
	}
	
	/**
	 * Sets the precisions of this precision position.
	 * 
	 * @param locPrecision the location precision to be set
	 * @param altPrecision the altitude precision to be set
	 * 
	 * @return a new precision position with the set precisions
	 */
	public PrecisionPosition setPrecisions(int locPrecision, int altPrecision) {
		return new PrecisionPosition(this.original, locPrecision, altPrecision);
	}
	
	/**
	 * Gets the original position of this precision position.
	 * 
	 * @return the original position of this precision position
	 * 
	 * @see Precision#getOriginal()
	 */
	@Override
	public Position getOriginal() {
		return this.original;
	}
	
	/**
	 * Compares this precision position to another position based on their
	 * latitude (primary) and longitude (secondary) angles.
	 * 
	 * @param position the other position
	 * 
	 * @return -1, 0, 1, if this precision position is less than, equal, or
	 *         greater, respectively, than the other position based on their
	 *         latitude (primary) and longitude (secondary)
	 * 
	 * @see Comparable#compareTo(Object)
	 */
	@Override
	public int compareTo(Position position) {
		int compareTo = this.latitude.compareTo(position.latitude);
		if (0 == compareTo) {
			compareTo = this.longitude.compareTo(position.longitude);
		}
		return compareTo;
	}
	
}
