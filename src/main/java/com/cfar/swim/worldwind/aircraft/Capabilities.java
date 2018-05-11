/**
 * Copyright (c) 2016, Stephan Heinemann (UVic Center for Aerospace Research)
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

import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.Iterator;

import com.cfar.swim.worldwind.geom.precision.PrecisionDouble;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;

/**
 * Realizes the capabilities of an aircraft (or any moving agent).
 * 
 * @author Stephan Heinemann
 *
 */
public class Capabilities {

	// all capabilities are stored in SI units

	/** the maximum angle of climb speed of this capabilities bean in m/s */
	private double maximumAngleOfClimbSpeed = 0d; // m/s

	/** the maximum rate of climb speed of this capabilities bean in m/s */
	private double maximumRateOfClimbSpeed = 0d; // m/s

	/** the cruise climb speed of this capabilities bean in m/s */
	private double cruiseClimbSpeed = 0d; // m/s

	/** the cruise speed of this capabilities bean in m/s */
	private double cruiseSpeed = 0d; // m/s

	/** the cruise descent speed of this capabilities bean in m/s */
	private double cruiseDescentSpeed = 0d; // m/s

	/** the approach speed of this capabilities bean in m/s */
	private double approachSpeed = 0d; // m/s

	/** the maximum glide speed of this capabilities bean in m/s */
	private double maximumGlideSpeed = 0d; // m/s

	/** the maximum rate of descent speed of this capabilities bean in m/s */
	private double maximumRateOfDescentSpeed = 0d; // m/s

	/** the maximum speed of this capabilities bean in m/s */
	private double maximumSpeed = 0d; // m/s

	/** maximum rate of climb of this capabilities bean in m/s */
	private double maximumRateOfClimb = 0d; // m/s

	/** the cruise rate of climb of this capabilities bean in m/s */
	private double cruiseRateOfClimb = 0d; // m/s

	/** the cruise rate of descent of this capabilities bean in m/s */
	private double cruiseRateOfDescent = 0d; // m/s

	/** the approach rate of descent of this capabilities bean in m/s */
	private double approachRateOfDescent = 0d; // m/s

	/** the maximum rate of descent of this capabilities bean in m/s */
	private double maximumRateOfDescent = 0d; // m/s

	/** the maximum angle of climb of this capabilities bean in degrees */
	private Angle maximumAngleOfClimb = Angle.fromDegrees(0); // deg

	// TODO: climb-speed-distance performance
	// TODO: possibly feature parameterization for air movement (horizontal /
	// vertical)
	// TODO: possibly feature parameterization for air density (temperature /
	// pressure / humidity)
	// TODO: create a new AirDataInterval class which aggregates AirData class
	// TODO: AirData, Surface/GroundData extends EnvironmentData
	// TODO: environment would have to store air property intervals
	// TODO: average fuel consumption, endurance, range, radius of action...
	// TODO: equipment (de-icing, gear, floats...)

	/**
	 * Gets the maximum angle of climb speed of this capabilities bean.
	 * 
	 * @return the maximum angle of climb speed of this capabilities bean in m/s
	 */
	public double getMaximumAngleOfClimbSpeed() {
		return maximumAngleOfClimbSpeed;
	}

	/**
	 * Sets the maximum angle of climb speed of this capabilities bean.
	 * 
	 * @param maximumAngleOfClimbSpeed the maximum angle of climb speed in m/s
	 */
	public void setMaximumAngleOfClimbSpeed(double maximumAngleOfClimbSpeed) {
		this.maximumAngleOfClimbSpeed = maximumAngleOfClimbSpeed;
	}

	/**
	 * Gets the maximum rate of climb speed of this capabilities bean.
	 * 
	 * @return the maximum rate of climb speed of this capabilities bean in m/s
	 */
	public double getMaximumRateOfClimbSpeed() {
		return maximumRateOfClimbSpeed;
	}

	/**
	 * Sets the maximum rate of climb speed of this capabilities bean.
	 * 
	 * @param maximumRateOfClimbSpeed the maximum rate of climb speed in m/s
	 */
	public void setMaximumRateOfClimbSpeed(double maximumRateOfClimbSpeed) {
		this.maximumRateOfClimbSpeed = maximumRateOfClimbSpeed;
	}

	/**
	 * Gets the cruise climb speed of this capabilities bean.
	 * 
	 * @return the cruise climb speed of this capabilities bean in m/s
	 */
	public double getCruiseClimbSpeed() {
		return cruiseClimbSpeed;
	}

	/**
	 * Sets the cruise climb speed of this capabilities bean.
	 * 
	 * @param cruiseClimbSpeed the cruise climb speed in m/s
	 */
	public void setCruiseClimbSpeed(double cruiseClimbSpeed) {
		this.cruiseClimbSpeed = cruiseClimbSpeed;
	}

	/**
	 * Gets the cruise speed of this capabilities bean.
	 * 
	 * @return the cruise speed of this capabilities bean in m/s
	 */
	public double getCruiseSpeed() {
		return this.cruiseSpeed;
	}

	/**
	 * Sets the cruise speed of this capabilities bean.
	 * 
	 * @param cruiseSpeed the cruise speed in m/s
	 */
	public void setCruiseSpeed(double cruiseSpeed) {
		this.cruiseSpeed = cruiseSpeed;
	}

	/**
	 * Gets the cruise descent speed of this capabilities bean.
	 * 
	 * @return the cruise descent speed of this capabilities bean in m/s
	 */
	public double getCruiseDescentSpeed() {
		return cruiseDescentSpeed;
	}

	/**
	 * Sets the cruise descent speed of this capabilities bean.
	 * 
	 * @param cruiseDescentSpeed the cruise descent speed in m/s
	 */
	public void setCruiseDescentSpeed(double cruiseDescentSpeed) {
		this.cruiseDescentSpeed = cruiseDescentSpeed;
	}

	/**
	 * Gets the approach speed of this capabilities bean.
	 * 
	 * @return the approach speed of this capabilities bean in m/s
	 */
	public double getApproachSpeed() {
		return approachSpeed;
	}

	/**
	 * Sets the approach speed of this capabilities bean.
	 * 
	 * @param approachSpeed the approach speed in m/s
	 */
	public void setApproachSpeed(double approachSpeed) {
		this.approachSpeed = approachSpeed;
	}

	/**
	 * Gets the maximum glide speed of this capabilities bean.
	 * 
	 * @return the maximum glide speed of this capabilities bean in m/s
	 */
	public double getMaximumGlideSpeed() {
		return maximumGlideSpeed;
	}

	/**
	 * Sets the maximum glide speed of this capabilities bean.
	 * 
	 * @param maximumGlideSpeed the maximum glide speed in m/s
	 */
	public void setMaximumGlideSpeed(double maximumGlideSpeed) {
		this.maximumGlideSpeed = maximumGlideSpeed;
	}

	/**
	 * Gets the maximum rate of descent speed of this capabilities bean.
	 * 
	 * @return the maximum rate of descent speed of this capabilities bean in m/s
	 */
	public double getMaximumRateOfDescentSpeed() {
		return maximumRateOfDescentSpeed;
	}

	/**
	 * Sets the maximum rate of descent speed of this capabilities bean.
	 * 
	 * @param maximumRateOfDescentSpeed the maximum rate of descent speed in m/s
	 */
	public void setMaximumRateOfDescentSpeed(double maximumRateOfDescentSpeed) {
		this.maximumRateOfDescentSpeed = maximumRateOfDescentSpeed;
	}

	/**
	 * Gets the maximum speed of this capabilities bean.
	 * 
	 * @return the maximum speed of this capabilities bean in m/s
	 */
	public double getMaximumSpeed() {
		return maximumSpeed;
	}

	/**
	 * Sets the maximum speed of this capabilities bean.
	 * 
	 * @param maximumSpeed the maximum speed in m/s
	 */
	public void setMaximumSpeed(double maximumSpeed) {
		this.maximumSpeed = maximumSpeed;
	}

	/**
	 * Gets the maximum rate of climb of this capabilities bean.
	 * 
	 * @return the maximum rate of climb of this capabilities bean in m/s
	 */
	public double getMaximumRateOfClimb() {
		return maximumRateOfClimb;
	}

	/**
	 * Sets the maximum rate of climb of this capabilities bean.
	 * 
	 * @param maximumRateOfClimb the maximum rate of climb in m/s
	 */
	public void setMaximumRateOfClimb(double maximumRateOfClimb) {
		this.maximumRateOfClimb = maximumRateOfClimb;
	}

	/**
	 * Gets the cruise rate of climb of this capabilities bean.
	 * 
	 * @return the cruise rate of climb of this capabilities bean in m/s
	 */
	public double getCruiseRateOfClimb() {
		return cruiseRateOfClimb;
	}

	/**
	 * Sets the cruise rate of climb of this capabilities bean.
	 * 
	 * @param cruiseRateOfClimb the cruise rate of climb in m/s
	 */
	public void setCruiseRateOfClimb(double cruiseRateOfClimb) {
		this.cruiseRateOfClimb = cruiseRateOfClimb;
	}

	/**
	 * Gets the cruise rate of descent of this capabilities bean.
	 * 
	 * @return the cruise rate of descent of this capabilities bean in m/s
	 */
	public double getCruiseRateOfDescent() {
		return cruiseRateOfDescent;
	}

	/**
	 * Sets the cruise rate of descent of this capabilities bean.
	 * 
	 * @param cruiseRateOfDescent the cruise rate of descent in m/s
	 */
	public void setCruiseRateOfDescent(double cruiseRateOfDescent) {
		this.cruiseRateOfDescent = cruiseRateOfDescent;
	}

	/**
	 * Gets the approach rate of descent of this capabilities bean.
	 * 
	 * @return the approach rate of descent of this capabilities bean in m/s
	 */
	public double getApproachRateOfDescent() {
		return approachRateOfDescent;
	}

	/**
	 * Sets the approach rate of descent of this capabilities bean.
	 * 
	 * @param approachRateOfDescent the approach rate of descent in m/s
	 */
	public void setApproachRateOfDescent(double approachRateOfDescent) {
		this.approachRateOfDescent = approachRateOfDescent;
	}

	/**
	 * Gets the maximum rate of descent of this capabilities bean.
	 * 
	 * @return the maximum rate of descent of this capabilities bean in m/s
	 */
	public double getMaximumRateOfDescent() {
		return maximumRateOfDescent;
	}

	/**
	 * Sets the maximum rate of descent of this capabilities bean.
	 * 
	 * @param maximumRateOfDescent the maximum rate of descent in m/s
	 */
	public void setMaximumRateOfDescent(double maximumRateOfDescent) {
		this.maximumRateOfDescent = maximumRateOfDescent;
	}

	/**
	 * Gets the maximum angle of climb of this capabilities bean.
	 * 
	 * @return the maximum angle of climb of this capabilities bean in degrees
	 */
	public Angle getMaximumAngleOfClimb() {
		return maximumAngleOfClimb;
	}

	/**
	 * Sets the maximum angle of climb of this capabilities bean.
	 * 
	 * @param maximumAngleOfClimb the maximum angle of climb in degrees
	 */
	public void setMaximumAngleOfClimb(Angle maximumAngleOfClimb) {
		this.maximumAngleOfClimb = maximumAngleOfClimb;
	}

	/**
	 * Gets the estimated duration to travel a specified horizontal distance at
	 * cruise speed in still air.
	 * 
	 * @param distance the horizontal distance in meters
	 * 
	 * @return the estimated duration to travel the specified level distance at
	 *         cruise speed
	 */
	public Duration getEstimatedDuration(double distance) {
		return Duration.ofSeconds((long) (distance / this.cruiseSpeed));
	}

	/**
	 * Gets the estimated duration to travel directly from a start to a goal
	 * position at cruise speed including climbs and descents in still air.
	 * 
	 * @param start the start position
	 * @param goal the goal position
	 * @param globe the globe associated with the positions
	 * 
	 * @return the estimated duration to travel directly from the start to the goal
	 *         position at cruise speed including climbs and descents in still air
	 * 
	 * @throws IllegalArgumentException if aircraft is incapable of traveling from
	 *             start to goal
	 */
	public Duration getEstimatedDuration(Position start, Position goal, Globe globe) throws IllegalArgumentException {
		// Henrique changed from seconds to milliseconds
		Duration estimatedDuration = Duration.ZERO;
		double distance = LatLon.linearDistance(start, goal).getRadians() * globe.getRadius();
		double height = goal.getElevation() - start.getElevation();

		if (new PrecisionDouble(height).equals(new PrecisionDouble(0d))) {
			// horizontal movement
			estimatedDuration = this.getEstimatedDuration(distance);
		} else if (0 < height) {
			// climb
			// compute cruise climb duration
			// Duration climbDuration = Duration.ofSeconds((long) (height /
			// this.cruiseRateOfClimb));
			Duration climbDuration = Duration.ofMillis((long) (height * 1000 / this.cruiseRateOfClimb));
			// compute cruise slant distance in still air
			// double slantDistance = this.cruiseClimbSpeed * climbDuration.getSeconds();
			double slantDistance = this.cruiseClimbSpeed * climbDuration.toMillis() / 1000;

			// perform feasibility check
			double maxSlantDistance = Math.sqrt(Math.pow(distance, 2) + Math.pow(height, 2));
			if (-1 == new PrecisionDouble(maxSlantDistance).compareTo(new PrecisionDouble(slantDistance))) {
				// compute minimum climb duration
				// climbDuration = Duration.ofSeconds((long) (height /
				// this.maximumRateOfClimb));
				climbDuration = Duration.ofMillis((long) (height * 1000 / this.maximumRateOfClimb));
				// compute minimum slant distance in still air
				// slantDistance = this.maximumRateOfClimbSpeed * climbDuration.getSeconds();
				slantDistance = this.maximumRateOfClimbSpeed * climbDuration.toMillis() / 1000;

				if (-1 == new PrecisionDouble(maxSlantDistance).compareTo(new PrecisionDouble(slantDistance))) {
					throw new IllegalArgumentException("incapable of traveling directly from " + start + " to " + goal);
				}
			}

			// compute climb angle in still air
			double climbAngle = Math.asin(height / slantDistance);
			// compute ground distance during climb in still air
			double climbDistance = height / Math.tan(climbAngle);
			// compute ground distance during level in still air
			double levelDistance = distance - climbDistance;
			// compute complete estimated duration
			estimatedDuration = climbDuration.plus(this.getEstimatedDuration(levelDistance));
		} else {
			// descent
			height = Math.abs(height);
			// compute cruise descent duration
			// Duration descentDuration = Duration.ofSeconds((long) (height /
			// this.cruiseRateOfDescent));
			Duration descentDuration = Duration.ofMillis((long) (height * 1000 / this.cruiseRateOfDescent));
			// compute cruise slant distance in still air
			// double slantDistance = this.cruiseDescentSpeed *
			// descentDuration.getSeconds();
			double slantDistance = this.cruiseDescentSpeed * descentDuration.toMillis() / 1000;

			// perform feasibility check
			double maxSlantDistance = Math.sqrt(Math.pow(distance, 2) + Math.pow(height, 2));
			if (-1 == new PrecisionDouble(maxSlantDistance).compareTo(new PrecisionDouble(slantDistance))) {
				// compute minimum descent duration
				// descentDuration = Duration.ofSeconds((long) (height /
				// this.maximumRateOfDescent));
				descentDuration = Duration.ofMillis((long) (height * 1000 / this.maximumRateOfDescent));
				// compute minimum slant distance in still air
				// slantDistance = this.maximumRateOfDescentSpeed *
				// descentDuration.getSeconds();
				slantDistance = this.maximumRateOfDescentSpeed * descentDuration.toMillis() / 1000;

				if (-1 == new PrecisionDouble(maxSlantDistance).compareTo(new PrecisionDouble(slantDistance))) {
					throw new IllegalArgumentException("incapable of traveling directly from " + start + " to " + goal);
				}
			}

			// compute descent angle in still air
			double descentAngle = Math.asin(height / slantDistance);
			// compute ground distance during descent in still air
			double descentDistance = height / Math.tan(descentAngle);
			// compute ground distance during level in still air
			double levelDistance = distance - descentDistance;
			// compute complete estimated duration
			estimatedDuration = descentDuration.plus(this.getEstimatedDuration(levelDistance));
		}

		return estimatedDuration;
	}

	/**
	 * Gets the estimated duration to travel along a path of positions at cruise
	 * speed including climbs and descents in still air.
	 * 
	 * @param path the path of positions
	 * @param globe the globe associated with the positions
	 * 
	 * @return the estimated duration to travel along the path of positions at
	 *         cruise speed including climbs and descents in still air
	 */
	public Duration getEstimatedDuration(Path path, Globe globe) {
		Duration estimatedDuration = Duration.ZERO;
		@SuppressWarnings("unchecked")
		Iterator<Position> positions = (Iterator<Position>) path.getPositions().iterator();
		Position current = null;

		if (positions.hasNext()) {
			current = positions.next();
		}

		while (positions.hasNext()) {
			estimatedDuration = estimatedDuration.plus(this.getEstimatedDuration(current, positions.next(), globe));
		}

		return estimatedDuration;
	}

	/**
	 * Gets the estimated time of arrival after traveling a specified horizontal
	 * distance.
	 * 
	 * @param distance the horizontal distance in meters
	 * @param start the start time
	 * 
	 * @return the estimated time of arrival after traveling the specified
	 *         horizontal distance
	 */
	public ZonedDateTime getEstimatedTime(double distance, ZonedDateTime start) {
		return start.plus(this.getEstimatedDuration(distance));
	}

	/**
	 * Gets the estimated time of arrival after traveling along a specified path of
	 * positions.
	 * 
	 * @param path the path of positions
	 * @param globe the globe associated with the positions
	 * @param start the start time
	 * 
	 * @return the estimated time of arrival after traveling along the specified
	 *         path of positions
	 */
	public ZonedDateTime getEstimatedTime(Path path, Globe globe, ZonedDateTime start) {
		return start.plus(this.getEstimatedDuration(path, globe));
	}

	/**
	 * Indicates whether or not an aircraft can travel directly from a start to a
	 * goal position, considering maximum cruise, climb and descents speeds and
	 * rates.
	 * 
	 * @param start the start position
	 * @param goal the goal position
	 * @param globe the globe associated with the positions
	 * 
	 * @return true if the leg is flyable, false otherwise
	 */
	public boolean isFeasible(Position start, Position goal, Globe globe) {
		double distance = LatLon.linearDistance(start, goal).getRadians() * globe.getRadius();
		double height = goal.getElevation() - start.getElevation();

		if (0 < height) {
			// climb
			// compute minimum climb duration
			Duration climbDuration = Duration.ofMillis((long) (height * 1000 / this.maximumRateOfClimb));
			// compute minimum slant distance in still air
			double slantDistance = this.maximumRateOfClimbSpeed * climbDuration.toMillis() / 1000;
			// perform feasibility check
			double maxSlantDistance = Math.sqrt(Math.pow(distance, 2) + Math.pow(height, 2));
				if (-1 == new PrecisionDouble(maxSlantDistance).compareTo(new PrecisionDouble(slantDistance))) {
					return false;
				}
		} else if (0 > height) {
			// descent
			height = Math.abs(height);
			// compute minimum descent duration
			Duration descentDuration = Duration.ofMillis((long) (height * 1000 / this.maximumRateOfDescent));
			// compute minimum slant distance in still air
			double slantDistance = this.maximumRateOfDescentSpeed * descentDuration.toMillis() / 1000;

			// perform feasibility check
			double maxSlantDistance = Math.sqrt(Math.pow(distance, 2) + Math.pow(height, 2));
			if (-1 == new PrecisionDouble(maxSlantDistance).compareTo(new PrecisionDouble(slantDistance))) {
				return false;
			}

		}
		return true;
	}

}
