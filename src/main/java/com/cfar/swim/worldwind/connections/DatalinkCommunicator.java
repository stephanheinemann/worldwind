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

/**
 * Describes a datalink communicator featuring datalink communications.
 * 
 * @author Stephan Heinemann
 * 
 */
public interface DatalinkCommunicator {
	
	/**
	 * Gets the datalink of this datalink communicator.
	 * 
	 * @return the datalink of this datalink communicator
	 */
	public Datalink getDatalink();
	
	/**
	 * Sets the datalink of this datalink communicator.
	 * 
	 * @param datalink the datalink to be set
	 */
	public void setDatalink(Datalink datalink);
	
	/**
	 * Determines whether or not this datalink communicator has a datalink.
	 * 
	 * @return true if this datalink communicator has a datalink,
	 *         false otherwise
	 */
	public boolean hasDatalink();
	
	/**
	 * Gets the datalink take-off communication of this datalink communicator.
	 * 
	 * @return the datalink take-off communication of this datalink
	 *         communicator
	 */
	public Communication<Datalink> getTakeOff();
	
	/**
	 * Sets the datalink take-off communication of this datalink communicator.
	 * 
	 * @param takeOff the datalink take-off communication to be set
	 */
	public void setTakeOff(Communication<Datalink> takeOff);
	
	/**
	 * Determines whether or not this datalink communicator has a datalink
	 * take-off communication.
	 * 
	 * @return true if this datalink communicator has a datalink take-off
	 *         communication, false otherwise
	 */
	public boolean hasTakeOff();
	
	/**
	 * Gets the datalink landing communication of this datalink communicator.
	 * 
	 * @return the datalink landing communication of this datalink
	 *         communicator
	 */
	public Communication<Datalink> getLanding();
	
	/**
	 * Sets the datalink landing communication of this datalink communicator.
	 * 
	 * @param landing the datalink landing communication to be set
	 */
	public void setLanding(Communication<Datalink> landing);
	
	/**
	 * Determines whether or not this datalink communicator has a datalink
	 * landing communication.
	 * 
	 * @return true if this datalink communicator has a datalink landing
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
	 * Determines whether or not this datalink communicator has a datalink
	 * unplanned landing communication.
	 * 
	 * @return true if this datalink communicator has a datalink unplanned
	 *         landing communication, false otherwise
	 */
	public boolean hasUnplannedLanding();
	
	/**
	 * Gets the establish datalink communication of this datalink communicator.
	 * 
	 * @return the establish datalink communication of this datalink
	 *         communicator
	 */
	public Communication<Datalink> getEstablishDataLink();
	
	/**
	 * Sets the establish datalink communication of this datalink communicator.
	 * 
	 * @param establishDatalink the establish datalink communication to be set
	 */
	public void setEstablishDatalink(Communication<Datalink> establishDatalink);
	
	/**
	 * Determines whether or not this datalink communicator has an establish
	 * datalink communication.
	 * 
	 * @return true if this datalink communicator has an establish datalink
	 *         communication, false otherwise
	 */
	public boolean hasEstablishDatalink();
	
	// TODO: in-flight alternates, off-track, emergency situations
	// TODO: alternate and precautionary landing communications
	
}
