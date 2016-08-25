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
package com.cfar.swim.worldwind.iwxxm;

import java.time.ZonedDateTime;

import org.apache.commons.lang3.builder.EqualsBuilder;
import org.apache.commons.lang3.builder.HashCodeBuilder;

/**
 * Realizes a SIGMET reference (coordinates) which is used to refer to previous
 * SIGMET messages for cancellation. Unfortunately a SIGMET cancel message does
 * not refer to the SIGMET identifier directly. All attributes of this class
 * are immutable.
 * 
 * @author Stephan Heinemann
 *
 */
public class IwxxmSigmetReference {

	/** the airspace designator of this SIGMET reference */
	private String airspaceDesignator;
	
	/** the airspace name of this SIGMET reference */
	private String airspaceName;
	
	/** the sequence number of this SIGMET reference */
	private int sequenceNumber;
	
	/** the valid start time of this SIGMET reference */
	private ZonedDateTime validStart;
	
	/** the valid end time of this SIGMET reference */
	private ZonedDateTime validEnd;
	
	/**
	 * Constructs a SIGMET reference with a specified airspace designator,
	 * airspace name, sequence number, and valid period start and end times. 
	 * 
	 * @param airspaceDesignator the airspace designator of this SIGMET reference
	 * @param airspaceName the airspace name of this SIGMET reference
	 * @param sequenceNumber the sequence number of this SIGMET reference
	 * @param validStart the valid period start time of this SIGMET reference
	 * @param validEnd the valid period end time of this SIGMET reference
	 */
	public IwxxmSigmetReference(
			String airspaceDesignator,
			String airspaceName,
			int sequenceNumber,
			ZonedDateTime validStart,
			ZonedDateTime validEnd) {
		this.airspaceDesignator = airspaceDesignator;
		this.airspaceName = airspaceName;
		this.sequenceNumber = sequenceNumber;
		this.validStart = validStart;
		this.validEnd = validEnd;
	}
	
	/**
	 * Gets the airspace designator of this SIGMET reference.
	 * 
	 * @return the airspace designator of this SIGMET reference
	 */
	public String getAirspaceDesignator() {
		return airspaceDesignator;
	}

	/**
	 * Sets the airspace designator of this SIGMET reference.
	 * 
	 * @param airspaceDesignator the airspace designator of this SIGMET reference
	 */
	public void setAirspaceDesignator(String airspaceDesignator) {
		this.airspaceDesignator = airspaceDesignator;
	}

	/**
	 * Gets the airspace name of this SIGMET reference.
	 * 
	 * @return the airspace name of this SIGMET reference
	 */
	public String getAirspaceName() {
		return airspaceName;
	}

	/**
	 * Sets the airspace name of this SIGMET reference.
	 * 
	 * @param airspaceName the airspace name of this SIGMET reference
	 */
	public void setAirspaceName(String airspaceName) {
		this.airspaceName = airspaceName;
	}

	/**
	 * Gets the sequence number of this SIGMET reference.
	 * 
	 * @return the sequence number of this SIGMET reference
	 */
	public int getSequenceNumber() {
		return sequenceNumber;
	}

	/**
	 * Sets the sequence number of this SIGMET reference.
	 * 
	 * @param sequenceNumber the sequence number of this SIGMET reference
	 */
	public void setSequenceNumber(int sequenceNumber) {
		this.sequenceNumber = sequenceNumber;
	}

	/**
	 * Gets the valid period start time of this SIGMET reference.
	 * 
	 * @return the valid period start time of this SIGMET reference
	 */
	public ZonedDateTime getValidStart() {
		return validStart;
	}

	/**
	 * Sets the valid period start time of this SIGMET reference.
	 * 
	 * @param validStart the valid period start time of this SIGMET reference
	 */
	public void setValidStart(ZonedDateTime validStart) {
		this.validStart = validStart;
	}

	/**
	 * Gets the valid period end time of this SIGMET reference.
	 * 
	 * @return the valid period end time of this SIGMET reference
	 */
	public ZonedDateTime getValidEnd() {
		return validEnd;
	}

	/**
	 * Sets the valid period end time of this SIGMET reference.
	 * 
	 * @param validEnd the valid period end time of this SIGMET reference
	 */
	public void setValidEnd(ZonedDateTime validEnd) {
		this.validEnd = validEnd;
	}
	
	/**
	 * Determines whether or not all immutable attributes of this SIGMET
	 * reference equal the ones of another SIGMET reference.
	 * 
	 * @param object the other SIGMET reference
	 * 
	 * @return true if the other SIGMET reference equals this SIGMET reference,
	 *         false otherwise
	 * 
	 * @see Object#equals(Object)
	 */
	@Override
	public boolean equals(Object object) {
		boolean equals = false;
		
		if (object == this) {
			equals = true;
		} else if (object instanceof IwxxmSigmetReference) {
			IwxxmSigmetReference sigmetReference = (IwxxmSigmetReference) object;
			
			equals = new EqualsBuilder()
						.append(this.airspaceDesignator, sigmetReference.airspaceDesignator)
						.append(this.airspaceName, sigmetReference.airspaceName)
						.append(this.sequenceNumber, sigmetReference.sequenceNumber)
						.append(this.validStart, sigmetReference.validStart)
						.append(this.validEnd, sigmetReference.validEnd)
						.isEquals();
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this SIGMET reference.
	 * 
	 * @return the hash code of this SIGMET reference
	 * 
	 * @see Object#hashCode()
	 */
	@Override
	public int hashCode() {
		return new HashCodeBuilder(79, 83)
				.append(this.airspaceDesignator)
				.append(this.airspaceName)
				.append(this.sequenceNumber)
				.append(this.validStart)
				.append(this.validEnd)
				.toHashCode();
	}
	
}
