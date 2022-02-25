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
package com.cfar.swim.worldwind.connections;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.tracks.AircraftTrackError;
import com.cfar.swim.worldwind.tracks.AircraftTrackPointError;

/**
 * Describes a datalink tracker following a datalink target.
 * 
 * @author Stephan Heinemann
 *
 */
public interface DatalinkTracker extends DatalinkCommunicator {
	
	/**
	 * Gets the aircraft this datalink tracker is tracking.
	 * 
	 * @return the aircraft this datalink tracker is tracking
	 */
	public Aircraft getAircraft();
	
	/**
	 * Gets the maximum acceptable track error of this datalink tracker to
	 * consider the aircraft on track.
	 * 
	 * @return the maximum acceptable track error of this datalink tracker to
	 *         consider the aircraft on track
	 */
	public AircraftTrackError getMaxTrackError();
	
	/**
	 * Sets the maximum acceptable track error of this datalink tracker to
	 * consider the aircraft on track.
	 * 
	 * @param maxTrackError the maximum acceptable track error to be set
	 */
	public void setMaxTrackError(AircraftTrackError maxTrackError);
	
	/**
	 * Determines whether or not the aircraft of this datalink tracker is on
	 * track.
	 * 
	 * @return true if the aircraft of this datalink tracker is on track,
	 *         false otherwise
	 */
	public boolean isOnTrack();
	
	/**
	 * Gets the maximum acceptable take-off error of this datalink tracker to
	 * consider the aircraft within the take-off regime.
	 * 
	 * @return the maximum acceptable take-off error of this datalink tracker
	 *         to consider the aircraft within the take-off regime
	 */
	public AircraftTrackPointError getMaxTakeOffError();
	
	/**
	 * Sets the maximum acceptable take-off error of this datalink tracker to
	 * consider the aircraft within the take-off regime.
	 * 
	 * @param maxTakeOffError the maximum acceptable take-off error to be set
	 */
	public void setMaxTakeOffError(AircraftTrackPointError maxTakeOffError);
	
	/**
	 * Determines whether or not the aircraft of this datalink tracker is
	 * within the take-off zone.
	 * 
	 * @return true if the aircraft of this datalink tracker is within the
	 *         take-off zone, false otherwise
	 */
	public boolean isInTakeOffZone();
	
	/**
	 * Determines whether or not the aircraft of this datalink tracker is
	 * within the take-off window.
	 * 
	 * @return true if the aircraft of this datalink tracker is within the
	 *         take-off window, false otherwise
	 */
	public boolean isInTakeOffWindow();
	
	/**
	 * Gets the maximum acceptable landing error of this datalink tracker to
	 * consider the aircraft within the landing regime.
	 * 
	 * @return the maximum acceptable landing error of this datalink tracker to
	 *         consider the aircraft within the landing regime
	 */
	public AircraftTrackPointError getMaxLandingError();
	
	/**
	 * Sets the maximum acceptable landing error of this datalink tracker to
	 * consider the aircraft within the landing regime.
	 * 
	 * @param maxLandingError the maximum acceptable landing error to be set
	 */
	public void setMaxLandingError(AircraftTrackPointError maxLandingError);
	
	/**
	 * Determines whether or not the aircraft of this datalink tracker is
	 * within the landing zone.
	 * 
	 * @return true if the aircraft of this datalink tracker is within the
	 *         landing zone, false otherwise
	 */
	public boolean isInLandingZone();
	
	/**
	 * Determines whether or not the aircraft of this datalink tracker is
	 * within the landing window.
	 * 
	 * @return true if the aircraft of this datalink tracker is within the
	 *         landing window, false otherwise
	 */
	public boolean isInLandingWindow();
	
	// TODO: consider maximum departure, arrival (terminal), and approach errors
	
}
