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

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;

public class Capabilities {

	// all capabilities are stored in SI units
	private double maximumAngleOfClimbSpeed = 0d; // m/s
	private double maximumRateOfClimbSpeed = 0d; // m/s
	private double cruiseClimbSpeed = 0d; // m/s
	private double cruiseSpeed = 0d; // m/s
	private double approachSpeed = 0d; // m/s
	private double maximumGlideSpeed = 0d; // m/s
	private double maximumSpeed = 0d; // m/s
	
	private double maximumRateOfClimb = 0d; // m/s
	private double cruiseRateOfClimb = 0d; // m/s
	private double approachRateOfDescent = 0d; // m/s
	private double maximumRateOfDescent = 0d; // m/s
	
	private Angle maximumAngleOfClimb = Angle.fromDegrees(0); // deg
	
	// TODO: climb-speed-distance performance
	// TODO: possibly feature parameterization for air movement (horizontal / vertical)
	// TODO: possibly feature parameterization for air density (temperature / pressure / humidity)
	// TODO: create a new AirDataInterval class which aggregates AirData class
	// TODO: environment would have to store air property intervals
	// TODO: average fuel consumption, endurance, range, radius of action...
	// TODO: equipment (de-icing, gear, floats...)
	
	public double getMaximumAngleOfClimbSpeed() {
		return maximumAngleOfClimbSpeed;
	}

	public void setMaximumAngleOfClimbSpeed(double maximumAngleOfClimbSpeed) {
		this.maximumAngleOfClimbSpeed = maximumAngleOfClimbSpeed;
	}
	
	public double getMaximumRateOfClimbSpeed() {
		return maximumRateOfClimbSpeed;
	}

	public void setMaximumRateOfClimbSpeed(double maximumRateOfClimbSpeed) {
		this.maximumRateOfClimbSpeed = maximumRateOfClimbSpeed;
	}
	
	public double getCruiseClimbSpeed() {
		return cruiseClimbSpeed;
	}

	public void setCruiseClimbSpeed(double cruiseClimbSpeed) {
		this.cruiseClimbSpeed = cruiseClimbSpeed;
	}
	
	public double getCruiseSpeed() {
		return this.cruiseSpeed;
	}
	
	public void setCruiseSpeed(double cruiseSpeed) {
		this.cruiseSpeed = cruiseSpeed;
	}
	
	public double getApproachSpeed() {
		return approachSpeed;
	}

	public void setApproachSpeed(double approachSpeed) {
		this.approachSpeed = approachSpeed;
	}
	
	public double getMaximumGlideSpeed() {
		return maximumGlideSpeed;
	}

	public void setMaximumGlideSpeed(double maximumGlideSpeed) {
		this.maximumGlideSpeed = maximumGlideSpeed;
	}
	
	public double getMaximumSpeed() {
		return maximumSpeed;
	}

	public void setMaximumSpeed(double maximumSpeed) {
		this.maximumSpeed = maximumSpeed;
	}

	public double getMaximumRateOfClimb() {
		return maximumRateOfClimb;
	}

	public void setMaximumRateOfClimb(double maximumRateOfClimb) {
		this.maximumRateOfClimb = maximumRateOfClimb;
	}
	
	public double getCruiseRateOfClimb() {
		return cruiseRateOfClimb;
	}

	public void setCruiseRateOfClimb(double cruiseRateOfClimb) {
		this.cruiseRateOfClimb = cruiseRateOfClimb;
	}
	
	public double getApproachRateOfDescent() {
		return approachRateOfDescent;
	}

	public void setApproachRateOfDescent(double approachRateOfDescent) {
		this.approachRateOfDescent = approachRateOfDescent;
	}
	
	public double getMaximumRateOfDescent() {
		return maximumRateOfDescent;
	}

	public void setMaximumRateOfDescent(double maximumDescentRate) {
		this.maximumRateOfDescent = maximumDescentRate;
	}
	
	public Angle getMaximumAngleOfClimb() {
		return maximumAngleOfClimb;
	}

	public void setMaximumAngleOfClimb(Angle maximumAngleOfClimb) {
		this.maximumAngleOfClimb = maximumAngleOfClimb;
	}
	
	/**
	 * Gets the estimated duration to travel a specified horizontal distance
	 * at cruise speed in still air.
	 * 
	 * @param distance the horizontal distance in meters
	 * 
	 * @return the estimated duration to travel the specified level distance
	 *         at cruise speed
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
	 * @return the estimated duration to travel directly from the start to the
	 *         goal position at cruise speed including climbs and descents in
	 *         still air
	 */
	public Duration getEstimatedDuration(Position start, Position goal, Globe globe) {
		Duration estimatedDuration = Duration.ZERO;
		double distance = LatLon.linearDistance(start, goal).getRadians() * globe.getRadius();
		double height = goal.getElevation() - start.getElevation();
		
		// use cruise rate of climb -> duration to cover height distance
		if (0 == height) {
			// horizontal movement
			estimatedDuration = this.getEstimatedDuration(distance);
		} else if (0 < height) {
			// climb
			// compute climb duration
			Duration climbDuration = Duration.ofSeconds((long) (height / this.cruiseRateOfClimb));
			// compute slant distance in still air
			double slantDistance = this.cruiseClimbSpeed * climbDuration.getSeconds();
			
			// perform feasibility check
			double maxSlantDistance = Math.sqrt(Math.pow(distance, 2) + Math.pow(height, 2));
			if (maxSlantDistance < slantDistance) {
				// TODO: try again with this.maximumRateOfClimb and use maxSlantDistance if feasible
				throw new IllegalArgumentException("incapable of traveling directly from " + start + " to " + goal);
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
			// compute descent duration
			Duration descentDuration = Duration.ofSeconds((long) (height / this.approachRateOfDescent));
			// compute slant distance in still air
			double slantDistance = this.approachSpeed * descentDuration.getSeconds();
			
			// perform feasibility check
			double maxSlantDistance = Math.sqrt(Math.pow(distance, 2) + Math.pow(height, 2));
			if (maxSlantDistance < slantDistance) {
				// TODO: try again with this.maximumRateOfDescent and use maxSlantDistance if feasible
				throw new IllegalArgumentException("incapable of traveling directly from " + start + " to " + goal);
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
		return start.plus(this.getEstimatedDuration(distance));
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
	
}
