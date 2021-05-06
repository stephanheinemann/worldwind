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

import java.util.Iterator;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.stream.Collectors;

import gov.nasa.worldwind.tracks.Track;
import gov.nasa.worldwind.tracks.TrackSegment;

/**
 * Realizes a track of aircraft track points obtained via a datalink downlink.
 * 
 * @author Stephan Heinemann
 *
 */
public class AircraftTrack extends ConcurrentLinkedDeque<AircraftTrackPoint> implements Track {

	/** the default serial version identifier */
	private static final long serialVersionUID = 1L;
	
	/** the name of this aircraft track */
	private String name = "";
	
	/**
	 * Gets the segments of this aircraft track.
	 * 
	 * @return the segments of this aircraft track
	 * 
	 * @see Track#getSegments()
	 */
	@Override
	public List<TrackSegment> getSegments() {
		AircraftTrackSegment segment = new AircraftTrackSegment();
		// TODO: muli-part plans, segments between POIs
		segment.addAll(this);
		return segment.stream()
				.map(s -> (TrackSegment) s)
				.collect(Collectors.toUnmodifiableList());
	}
	
	/**
	 * Gets the name of this aircraft track.
	 * 
	 * @return the name of this aircraft track
	 * 
	 * @see Track#getName()
	 */
	@Override
	public String getName() {
		return this.name;
	}
	
	/**
	 * Sets the name of this aircraft track.
	 * 
	 * @param name the name to set
	 */
	public void setName(String name) {
		this.name = name;
	}
	
	/**
	 * Gets the number of track points of this aircraft track.
	 * 
	 * @return the number of track points of this aircraft track
	 * 
	 * @see Track#getNumPoints()
	 */
	@Override
	public int getNumPoints() {
		return this.size();
	}
	
	/**
	 * Gets the first track point of this aircraft track.
	 * 
	 * @return the first track point of this aircraft track
	 */
	public AircraftTrackPoint getFirstTrackPoint() {
		return this.peekFirst();
	}
	
	/**
	 * Gets the last track point of this aircraft track.
	 * 
	 * @return the last track point of this aircraft track
	 */
	public AircraftTrackPoint getLastTrackPoint() {
		return this.peekLast();
	}
	
	/**
	 * Gets a previous track point at a specified index of the reverse of this
	 * aircraft track.
	 * 
	 * @param index the index of the track point of the reverse of this
	 *              aircraft track
	 * 
	 * @return the previous track point at the specified index of the reverse
	 *         of this aircraft track
	 */
	public AircraftTrackPoint getPreviousTrackPoint(int index) {
		AircraftTrackPoint previousTrackPoint = null;
		Iterator<AircraftTrackPoint> reverseTrackIterator = this.descendingIterator();
		
		while (reverseTrackIterator.hasNext() && index > 0) {
			reverseTrackIterator.next();
			index--;
		}
		
		if (reverseTrackIterator.hasNext()) {
			previousTrackPoint = reverseTrackIterator.next();
		}
		
		return previousTrackPoint;
	}
	
}
