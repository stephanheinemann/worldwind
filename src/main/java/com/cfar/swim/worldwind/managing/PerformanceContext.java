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
package com.cfar.swim.worldwind.managing;

import java.io.Serializable;

import oshi.SystemInfo;

/**
 * Realizes a performance context featuring relevant system information.
 * 
 * @author Stephan Heinemann
 *
 */
public class PerformanceContext implements Serializable {

	/** the default serial identification of this performance context */
	private static final long serialVersionUID = 1L;

	/** the system information interface of this performance context */
	private static final SystemInfo systemInfo = new SystemInfo();
	
	/** the processor name of this performance context */
	private final String processorName;
	
	/** the physical processor count of this performance context */
	private final int physicalProcessorCount;
	
	/** the logical processor count of this performance context */
	private final int logicalProcessorCount;
	
	/** the maximum logical processor frequency of this performance context */
	private final long maxFreq;
	
	/**
	 * Constructs a new performance context.
	 */
	public PerformanceContext() {
		this.processorName = PerformanceContext.systemInfo
				.getHardware().getProcessor().getProcessorIdentifier().getName();
		this.physicalProcessorCount = PerformanceContext.systemInfo
				.getHardware().getProcessor().getPhysicalProcessorCount();
		this.logicalProcessorCount = PerformanceContext.systemInfo
				.getHardware().getProcessor().getLogicalProcessorCount();
		this.maxFreq = PerformanceContext.systemInfo
				.getHardware().getProcessor().getMaxFreq();
		// TODO: more context information
	}
	
	/**
	 * Gets the processor name of this performance context.
	 * 
	 * @return the processor name of this performance context
	 */
	public String getProcessorName() {
		return this.processorName;
	}
	
	/**
	 * Gets the physical processor count of this performance context.
	 * 
	 * @return the physical processor count of this performance context
	 */
	public int getPhysicalProcessorCount() {
		return this.physicalProcessorCount;
	}
	
	/**
	 * Gets the logical processor count of this performance context.
	 * 
	 * @return the logical processor count of this performance context
	 */
	public int getLogicalProcessorCount() {
		return this.logicalProcessorCount;
	}
	
	/**
	 * Gets the maximum logical processor frequency of this performance
	 * context.
	 * 
	 * @return the maximum logical processor frequency of this performance
	 *         context
	 */
	public long getMaxFreq() {
		return this.maxFreq;
	}
	
	/**
	 * Gets the string representation of this performance context.
	 * 
	 * @return the string representation of this performance context
	 * 
	 * @see Object#toString()
	 */
	@Override
	public String toString() {
		return "PN  = " + this.getProcessorName() + "\n"
				+ "PPC = " + this.getPhysicalProcessorCount() + "\n"
				+ "LPC = " + this.getLogicalProcessorCount() + "\n"
				+ "MLF = " + this.getMaxFreq();
	}
	
}
