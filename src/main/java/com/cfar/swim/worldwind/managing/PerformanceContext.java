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
import java.util.Objects;
import java.util.ResourceBundle;

import com.cfar.swim.worldwind.util.ResourceBundleLoader;

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
	
	/** the current performance context */
	public static final PerformanceContext CURRENT = new PerformanceContext();
	
	/** the dictionary of this performance context */
	protected transient final ResourceBundle dictionary = ResourceBundleLoader.getDictionaryBundle();
	
	/** the processor name property name of this performance context */
	public static final String HW_PROCESSOR_NAME = "performance.hardware.processor.name";
	
	/** the physical processor count property name of this performance context */
	public static final String HW_PROCESSOR_PHYSICAL_COUNT = "performance.hardware.processor.physical.count";
	
	/** the logical processor count property name of this performance context */
	public static final String HW_PROCESSOR_LOGICAL_COUNT = "performance.hardware.processor.logical.count";
	
	/** the maximum processor frequency property name of this performance context */
	public static final String HW_PROCESSOR_FREQUENCY_MAX = "performance.hardware.processor.frequency.max";
	
	/** the total memory property name of this performance context */
	public static final String HW_MEMORY_TOTAL = "performance.hardware.memory.total";
	
	/** the operating system platform property name of this performance context */
	public static final String OS_PLATFORM = "performance.operatingSystem.platform";
	
	/** the operating system manufacturer property name of this performance context */
	public static final String OS_MANUFACTURER = "performance.operatingSystem.manufacturer";
	
	/** the processor name of this performance context */
	private final String processorName;
	
	/** the physical processor count of this performance context */
	private final int physicalProcessorCount;
	
	/** the logical processor count of this performance context */
	private final int logicalProcessorCount;
	
	/** the maximum logical processor frequency of this performance context in Hz */
	private final long maxFrequency;
	
	/** the total memory of this performance context in bytes */
	private final long totalMemory;
	
	/** the operating system platform of this performance context */
	private final String osPlatform;
	
	/** the operating system manufacturer of this performance context */
	private final String osManufacturer;
	
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
		this.maxFrequency = PerformanceContext.systemInfo
				.getHardware().getProcessor().getMaxFreq();
		this.totalMemory = PerformanceContext.systemInfo
				.getHardware().getMemory().getTotal();
		this.osPlatform = SystemInfo.getCurrentPlatform().toString();
		this.osManufacturer = PerformanceContext.systemInfo
				.getOperatingSystem().getManufacturer();
		// TODO: more context information as required
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
	public long getMaxFrequency() {
		return this.maxFrequency;
	}
	
	/**
	 * Gets the total memory of this performance context.
	 * 
	 * @return the total memory of this performance context
	 */
	public long getTotalMemory() {
		return this.totalMemory;
	}
	
	/**
	 * Gets the operating system platform of this performance context.
	 * 
	 * @return the operating system platform of this performance context
	 */
	public String getOsPlatform() {
		return this.osPlatform;
	}
	
	/**
	 * Gets the operating system manufacturer of this performance context.
	 * 
	 * @return the operating system manufacturer of this performance context
	 */
	public String getOsManufacturer() {
		return this.osManufacturer;
	}
	
	/**
	 * Determines whether or not this performance context equals another
	 * performance context based on their captured system information.
	 * 
	 * @param o the other performance context
	 * 
	 * @return true, if the captured system information of this performance
	 *         context equals the captured system information of the other
	 *         performance context, false otherwise
	 * 
	 * @see Object#equals(Object)
	 */
	@Override
	public boolean equals(Object o) {
		boolean equals = false;
		
		if (this == o) {
			equals = true;
		} else if ((null != o) && (this.getClass() == o.getClass())) {
			PerformanceContext pc = (PerformanceContext) o;
			equals = (this.logicalProcessorCount == pc.logicalProcessorCount)
					&& (this.maxFrequency == pc.maxFrequency)
					&& (this.osManufacturer.equals(pc.osManufacturer))
					&& (this.osPlatform.equals(pc.osPlatform))
					&& (this.physicalProcessorCount == pc.physicalProcessorCount)
					&& (this.processorName.equals(pc.processorName))
					&& (this.totalMemory == pc.totalMemory);
		}
		
		return equals;
	}
	
	/**
	 * Gets the hash code of this performance context based on its captured
	 * system information.
	 * 
	 * @return the hash code of this performance context based on its captured
	 *         system information
	 * 
	 * @see Object#hashCode()
	 */
	@Override
	public int hashCode() {
		return Objects.hash(
				this.logicalProcessorCount,
				this.maxFrequency,
				this.osManufacturer,
				this.osPlatform,
				this.physicalProcessorCount,
				this.processorName,
				this.totalMemory);
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
		return this.dictionary.getString(PerformanceContext.HW_PROCESSOR_NAME)
				+ " = " + this.getProcessorName() + "\n"
				+ this.dictionary.getString(PerformanceContext.HW_PROCESSOR_PHYSICAL_COUNT)
				+ " = " + this.getPhysicalProcessorCount() + "\n"
				+ this.dictionary.getString(PerformanceContext.HW_PROCESSOR_LOGICAL_COUNT)
				+ " = " + this.getLogicalProcessorCount() + "\n"
				+ this.dictionary.getString(PerformanceContext.HW_PROCESSOR_FREQUENCY_MAX)
				+ " = " + this.getMaxFrequency() + "\n"
				+ this.dictionary.getString(PerformanceContext.HW_MEMORY_TOTAL)
				+ " = " + this.getTotalMemory() + "\n"
				+ this.dictionary.getString(PerformanceContext.OS_PLATFORM)
				+ " = " + this.getOsPlatform() + "\n"
				+ this.dictionary.getString(PerformanceContext.OS_MANUFACTURER)
				+ " = " + this.getOsManufacturer();
	}
	
}
