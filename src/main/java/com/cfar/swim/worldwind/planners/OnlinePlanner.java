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
package com.cfar.swim.worldwind.planners;

import com.cfar.swim.worldwind.connections.Communication;
import com.cfar.swim.worldwind.connections.Datalink;
import com.cfar.swim.worldwind.tracks.AircraftTrackError;

/**
 * Describes an online planner featuring a datalink connection to communicate
 * with a planned aircraft.
 * 
 * @author Stephan Heinemann
 *
 */
public interface OnlinePlanner extends Planner {

	/**
	 * Gets the datalink of this online planner.
	 * 
	 * @return the datalink of this online planner
	 */
	public Datalink getDatalink();
	
	/**
	 * Sets the datalink of this online planner.
	 * 
	 * @param datalink the datalink of this online planner
	 */
	public void setDatalink(Datalink datalink);
	
	/**
	 * Determines whether or not this online planner has a datalink.
	 * 
	 * @return true if this online planner has a datalink, false otherwise
	 */
	public boolean hasDatalink();
	
	/**
	 * Gets the datalink take-off communication of this online planner.
	 * 
	 * @return the datalink take-off communication of this online planner
	 */
	public Communication<Datalink> getTakeOff();
	
	/**
	 * Sets the datalink take-off communication of this online planner.
	 * 
	 * @param takeOff the datalink take-off communication to be set
	 */
	public void setTakeOff(Communication<Datalink> takeOff);
	
	/**
	 * Determines whether or not this online planner has a datalink take-off
	 * communication.
	 * 
	 * @return true if this online planner has a datalink take-off
	 *         communication, false otherwise
	 */
	public boolean hasTakeOff();
	
	/**
	 * Gets the datalink landing communication of this online planner.
	 * 
	 * @return the datalink landing communication of this online planner
	 */
	public Communication<Datalink> getLanding();
	
	/**
	 * Sets the datalink landing communication of this online planner.
	 * 
	 * @param landing the datalink landing communication to be set
	 */
	public void setLanding(Communication<Datalink> landing);
	
	/**
	 * Determines whether or not this online planner has a datalink landing
	 * communication.
	 * 
	 * @return true if this online planner has a datalink landing
	 *         communication, false otherwise
	 */
	public boolean hasLanding();
	
	/**
	 * Gets the datalink unplanned landing communication of this online
	 * planner.
	 * 
	 * @return the datalink unplanned landing communication of this online
	 *         planner
	 */
	public Communication<Datalink> getUnplannedLanding();
	
	/**
	 * Sets the datalink unplanned landing communication of this online
	 * planner.
	 * 
	 * @param unplannedLanding the datalink unplanned landing communication to
	 *                         be set
	 */
	public void setUnplannedLanding(Communication<Datalink> unplannedLanding);
	
	/**
	 * Determines whether or not this online planner has a datalink unplanned
	 * landing communication.
	 * 
	 * @return true if this online planner has a datalink unplanned landing
	 *         communication, false otherwise
	 */
	public boolean hasUnplannedLanding();
	
	/**
	 * Gets the establish datalink communication of this online planner.
	 * 
	 * @return the establish datalink communication of this online planner
	 */
	public Communication<Datalink> getEstablishDataLink();
	
	/**
	 * Sets the establish datalink communication of this online planner.
	 * 
	 * @param establishDatalink the establish datalink communication of this
	 *                          online planner
	 */
	public void setEstablishDatalink(Communication<Datalink> establishDatalink);
	
	/**
	 * Determines whether or not this online planner has an establish datalink
	 * communication.
	 * 
	 * @return true if this online planner has an establish datalink
	 *         communication, false otherwise
	 */
	public boolean hasEstablishDatalink();
	
	/**
	 * Gets the maximum acceptable track error of this online planner to
	 * consider the aircraft on track.
	 * 
	 * @return the maximum acceptable track error of this online planner to
	 *         consider the aircraft on track
	 */
	public AircraftTrackError getMaxTrackError();
	
	/**
	 * Sets the maximum acceptable track error of this online planner to
	 * consider the aircraft on track.
	 * 
	 * @param maxTrackError the maximum acceptable track error to be set
	 */
	public void setMaxTrackError(AircraftTrackError maxTrackError);
	
	/**
	 * Determines whether or not the aircraft of this online planner is on
	 * track.
	 * 
	 * @return true if the aircraft of this online planner is on track,
	 *         false otherwise
	 */
	public boolean isOnTrack();
	
}
