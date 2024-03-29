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
package com.cfar.swim.worldwind.aircraft;

import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.Objects;

import com.cfar.swim.worldwind.geom.precision.Precision;
import com.cfar.swim.worldwind.geom.precision.PrecisionDouble;

import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;
import gov.nasa.worldwind.util.measure.LengthMeasurer;

/**
 * Realizes the capabilities of an aircraft (or any moving agent).
 * 
 * @author Stephan Heinemann
 *
 */
public class Capabilities {
	
	/** the standard climb angle */
	public static final Angle STANDARD_CLIMB_ANGLE = Angle.fromDegrees(3d);
	
	/** the standard descent angle */
	public static final Angle STANDARD_DESCENT_ANGLE = Angle.fromDegrees(-3d);
	
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
	// TODO: possibly feature parameterization for air movement (horizontal / vertical)
	// TODO: possibly feature parameterization for air density (temperature / pressure / humidity)
	// TODO: maximum rate of turn
	// TODO: ensure maximum angles correspond to maximum speeds and rates
	// TODO: maximum and service ceilings
	// TODO: dynamic air data intervals (lambdas instead of constants)
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
	 * Gets the maximum of two durations.
	 * 
	 * @param d1 the first duration
	 * @param d2 the second duration
	 * 
	 * @return the maximum of the two durations
	 */
	public static Duration getMaximumDuration(Duration d1, Duration d2) {
		return (0 < d1.compareTo(d2)) ? d1 : d2;
	}
	
	/**
	 * Gets the estimated duration to travel a specified distance at a given
	 * speed in still air.
	 * 
	 * @param distance the distance in meters
	 * @param speed the speed in meters per second
	 * 
	 * @return the estimated duration to travel the specified distance at the
	 *         given speed
	 */
	public Duration getEstimatedDuration(double distance, double speed) {
		double seconds = Math.floor(distance / speed);
		double nanos = Math.floor(((distance / speed) - seconds) / Precision.UNIT_NANO);
		return Duration.ofSeconds((long) seconds, (long) nanos);
	}
	
	/**
	 * Gets the estimated duration to travel directly from a start to a goal
	 * position at cruise speed including climbs and descents in still air.
	 * 
	 * @param start the start position
	 * @param goal the goal position
	 * @param globe the globe associated with the positions
	 * 
	 * @return the estimated duration to travel directly from the start to the
	 *         goal position at cruise speed including climbs and descents in
	 *         still air
	 * 
	 * @throws CapabilitiesException if traveling directly from the start to
	 *         the goal position exceeds these capabilities
	 */
	public Duration getEstimatedDuration(Position start, Position goal, Globe globe) {
		Duration estimatedDuration = Duration.ZERO;
		
		// slant, vertical (height) and (horizontal) distance
		ArrayList<Position> positions = new ArrayList<Position>();
		positions.add(start);
		positions.add(goal);
		LengthMeasurer measurer = new LengthMeasurer(positions);
		measurer.setPathType(AVKey.LINEAR);
		measurer.setFollowTerrain(false);
		double maxSlantDistance = measurer.getLength(globe);
		
		// double distance = LatLon.linearDistance(start, goal).getRadians() * globe.getRadius();
		double height = goal.getElevation() - start.getElevation();
		double distance = Math.sqrt(Math.pow(maxSlantDistance, 2d) - Math.pow(height, 2d));
		
		// determine flight path angle
		Angle angle = Angle.ZERO;
		if (0d != distance) {
			angle = Angle.fromDegrees(Math.atan(height / distance));
		} else {
			if (0d < height) {
				angle = Angle.POS90;
			} else if (0d > height) {
				angle = Angle.NEG90;
			}
		}
		
		// determine durations without TOC and TOD
		if (0 <= angle.compareTo(Capabilities.STANDARD_CLIMB_ANGLE)) {
			// assume climb performance
			// compute cruise climb duration
			Duration climbDuration = Duration.ofMillis((long) (height / this.cruiseRateOfClimb / Precision.UNIT_MILLI));
			// compute cruise slant distance in still air
			double slantDistance = this.cruiseClimbSpeed * climbDuration.toMillis() * Precision.UNIT_MILLI;
			
			// perform feasibility check
			if (-1 == new PrecisionDouble(maxSlantDistance).compareTo(new PrecisionDouble(slantDistance))) {
				// compute minimum climb duration
				climbDuration = Duration.ofMillis((long) (height / this.maximumRateOfClimb / Precision.UNIT_MILLI));
				// compute minimum slant distance in still air
				slantDistance = this.maximumRateOfClimbSpeed * climbDuration.toMillis() * Precision.UNIT_MILLI;
				// apply maximum climb performance for horizontal speed
				estimatedDuration = this.getEstimatedDuration(maxSlantDistance, this.getMaximumRateOfClimbSpeed());
				//Duration levelDuration = this.getEstimatedDuration(distance, this.getMaximumRateOfClimbSpeed());
				//estimatedDuration = Capabilities.getMaximumDuration(climbDuration, levelDuration);
				
				if (-1 == new PrecisionDouble(maxSlantDistance).compareTo(new PrecisionDouble(slantDistance))) {
					throw new CapabilitiesException("incapable to travel directly from " + start + " to " + goal);
				}
			} else {
				// apply cruise climb performance for horizontal speed
				estimatedDuration = this.getEstimatedDuration(maxSlantDistance, this.getCruiseClimbSpeed());
				//Duration levelDuration = this.getEstimatedDuration(distance, this.getCruiseClimbSpeed());
				//estimatedDuration = Capabilities.getMaximumDuration(climbDuration, levelDuration);
			}
		} else if (0 >= angle.compareTo(Capabilities.STANDARD_DESCENT_ANGLE)) {
			// assume descent performance
			height = Math.abs(height);
			// compute cruise descent duration
			Duration descentDuration = Duration.ofMillis((long) (height / this.cruiseRateOfDescent / Precision.UNIT_MILLI));
			// compute cruise slant distance in still air
			double slantDistance = this.cruiseDescentSpeed * descentDuration.toMillis() * Precision.UNIT_MILLI;
			
			// perform feasibility check
			if (-1 == new PrecisionDouble(maxSlantDistance).compareTo(new PrecisionDouble(slantDistance))) {
				// compute minimum descent duration
				descentDuration = Duration.ofMillis((long) (height / this.maximumRateOfDescent / Precision.UNIT_MILLI));
				// compute minimum slant distance in still air
				slantDistance = this.maximumRateOfDescentSpeed * descentDuration.toMillis() * Precision.UNIT_MILLI;
				// apply maximum descent performance for horizontal speed
				estimatedDuration = this.getEstimatedDuration(maxSlantDistance, this.getMaximumRateOfDescentSpeed());
				//Duration levelDuration = this.getEstimatedDuration(distance, this.getMaximumRateOfDescentSpeed());
				//estimatedDuration = Capabilities.getMaximumDuration(descentDuration, levelDuration);
				
				if (-1 == new PrecisionDouble(maxSlantDistance).compareTo(new PrecisionDouble(slantDistance))) {
					throw new CapabilitiesException("incapable to travel directly from " + start + " to " + goal);
				}
			} else {
				// apply cruise descent performance for horizontal speed
				estimatedDuration = this.getEstimatedDuration(maxSlantDistance, this.getCruiseDescentSpeed());
				//Duration levelDuration = this.getEstimatedDuration(distance, this.getCruiseDescentSpeed());
				//estimatedDuration = Capabilities.getMaximumDuration(descentDuration, levelDuration);
				
			}
		} else if (0d != distance){
			// assume level performance
			estimatedDuration = this.getEstimatedDuration(maxSlantDistance, this.getCruiseSpeed());
			//estimatedDuration = this.getEstimatedDuration(distance, this.getCruiseSpeed());
		}
		
		// TODO: the following model assumes the automatic generation of TOC and TOD waypoints
		/*
		if (this.cruiseRateOfClimb <= height) {
			// climb for at least one second
			// compute cruise climb duration
			Duration climbDuration = Duration.ofMillis((long) (height / this.cruiseRateOfClimb / Precision.UNIT_MILLI));
			// compute cruise slant distance in still air
			double slantDistance = this.cruiseClimbSpeed * climbDuration.toMillis() * Precision.UNIT_MILLI;
			
			// perform feasibility check
			double maxSlantDistance = Math.sqrt(Math.pow(distance, 2) + Math.pow(height, 2));
			if (-1 == new PrecisionDouble(maxSlantDistance).compareTo(new PrecisionDouble(slantDistance))) {
				// compute minimum climb duration
				climbDuration = Duration.ofMillis((long) (height / this.maximumRateOfClimb / Precision.UNIT_MILLI));
				// compute minimum slant distance in still air
				slantDistance = this.maximumRateOfClimbSpeed * climbDuration.toMillis() * Precision.UNIT_MILLI;
				
				if (-1 == new PrecisionDouble(maxSlantDistance).compareTo(new PrecisionDouble(slantDistance))) {
					throw new CapabilitiesException("incapable to travel directly from " + start + " to " + goal);
				}
			}
			
			// compute climb angle in still air
			double climbAngle = Math.asin(Math.min(1d, (height / slantDistance))) * 180d / Math.PI;
			// compute ground distance during climb in still air
			double climbDistance = (90d > climbAngle) ? height / Math.tan(climbAngle) : 0d;
			// compute ground distance during level in still air
			double levelDistance = distance - climbDistance;
			// compute complete estimated duration
			estimatedDuration = climbDuration.plus(this.getEstimatedDuration(levelDistance, this.getCruiseSpeed()));
			
		} else if (this.cruiseRateOfDescent <= Math.abs(height)) {
			// descent for at least one second
			height = Math.abs(height);
			// compute cruise descent duration
			Duration descentDuration = Duration.ofMillis((long) (height / this.cruiseRateOfDescent / Precision.UNIT_MILLI));
			// compute cruise slant distance in still air
			double slantDistance = this.cruiseDescentSpeed * descentDuration.toMillis() * Precision.UNIT_MILLI;
			
			// perform feasibility check
			double maxSlantDistance = Math.sqrt(Math.pow(distance, 2) + Math.pow(height, 2));
			if (-1 == new PrecisionDouble(maxSlantDistance).compareTo(new PrecisionDouble(slantDistance))) {
				// compute minimum descent duration
				descentDuration = Duration.ofMillis((long) (height / this.maximumRateOfDescent / Precision.UNIT_MILLI));
				// compute minimum slant distance in still air
				slantDistance = this.maximumRateOfDescentSpeed * descentDuration.toMillis() * Precision.UNIT_MILLI;
				
				if (-1 == new PrecisionDouble(maxSlantDistance).compareTo(new PrecisionDouble(slantDistance))) {
					throw new CapabilitiesException("incapable to travel directly from " + start + " to " + goal);
				}
			}
			
			// compute descent angle in still air
			double descentAngle = Math.asin(Math.min(1d, (height / slantDistance))) * 180d / Math.PI;
			// compute ground distance during descent in still air
			double descentDistance = (90d > descentAngle) ? height / Math.tan(descentAngle) : 0d;
			// compute ground distance during level in still air
			double levelDistance = distance - descentDistance;
			// compute complete estimated duration
			estimatedDuration = descentDuration.plus(this.getEstimatedDuration(levelDistance, this.getCruiseSpeed()));
			
		} else {
			// horizontal movement
			estimatedDuration = this.getEstimatedDuration(distance, this.getCruiseSpeed());
		}
		*/
		
		return estimatedDuration;
	}
	
	/**
	 * Gets the estimated duration to travel along a path of positions at
	 * cruise speed including climbs and descents in still air.
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
	 * Gets the estimated time of arrival after traveling a specified
	 * horizontal distance.
	 * 
	 * @param distance the horizontal distance in meters 
	 * @param start the start time
	 * 
	 * @return the estimated time of arrival after traveling the specified
	 *         horizontal distance
	 */
	public ZonedDateTime getEstimatedTime(double distance, ZonedDateTime start) {
		return start.plus(this.getEstimatedDuration(distance, this.getCruiseSpeed()));
	}
	
	/**
	 * Gets the estimated time of arrival after traveling along a specified
	 * path of positions.
	 * 
	 * @param path the path of positions
	 * @param globe the globe associated with the positions
	 * @param start the start time
	 * 
	 * @return the estimated time of arrival after traveling along the
	 *         specified path of positions
	 */
	public ZonedDateTime getEstimatedTime(Path path, Globe globe, ZonedDateTime start) {
		return start.plus(this.getEstimatedDuration(path, globe));
	}
	
	/**
	 * Determines whether or not this capabilities bean equals another one
	 * based on their aggregated capabilities.
	 * 
	 * @param o the other capabilities bean
	 * 
	 * @return true if the aggregated capabilities of this capabilities bean
	 *         equal the aggregated capabilities of the other one,
	 *         false otherwise
	 * 
	 * @see Object#equals(Object)
	 */
	@Override
	public final boolean equals(Object o) {
		boolean equals = false;
		
		if (this == o) {
			equals = true;
		} else if ((null != o) && (o instanceof Capabilities)) {
			Capabilities c = (Capabilities) o;
			equals = (this.approachRateOfDescent == c.approachRateOfDescent)
					&& (this.approachSpeed == c.approachSpeed)
					&& (this.cruiseClimbSpeed == c.cruiseClimbSpeed)
					&& (this.cruiseDescentSpeed == c.cruiseDescentSpeed)
					&& (this.cruiseRateOfClimb == c.cruiseRateOfClimb)
					&& (this.cruiseRateOfDescent == c.cruiseRateOfDescent)
					&& (this.cruiseSpeed == c.cruiseSpeed)
					&& (this.maximumAngleOfClimb.equals(c.maximumAngleOfClimb))
					&& (this.maximumAngleOfClimbSpeed == c.maximumAngleOfClimbSpeed)
					&& (this.maximumGlideSpeed == c.maximumGlideSpeed)
					&& (this.maximumRateOfClimb == c.maximumRateOfClimb)
					&& (this.maximumRateOfClimbSpeed == c.maximumRateOfClimbSpeed)
					&& (this.maximumRateOfDescent == c.maximumRateOfDescent)
					&& (this.maximumRateOfDescentSpeed == c.maximumRateOfDescentSpeed)
					&& (this.maximumSpeed == c.maximumSpeed);
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this capabilities bean based on its aggregated
	 * capabilities.
	 * 
	 * @return the hash code of this capabilities bean based on its aggregated
	 *         capabilities
	 * 
	 * @see Object#hashCode()
	 */
	@Override
	public final int hashCode() {
		return Objects.hash(
				this.approachRateOfDescent,
				this.approachSpeed,
				this.cruiseClimbSpeed,
				this.cruiseDescentSpeed,
				this.cruiseRateOfClimb,
				this.cruiseRateOfDescent,
				this.cruiseSpeed,
				this.maximumAngleOfClimb,
				this.maximumAngleOfClimbSpeed,
				this.maximumGlideSpeed,
				this.maximumRateOfClimb,
				this.maximumRateOfClimbSpeed,
				this.maximumRateOfDescent,
				this.maximumRateOfDescentSpeed,
				this.maximumSpeed);
	}
	
}
