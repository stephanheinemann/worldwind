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
package com.cfar.swim.worldwind.geom.precision;

/**
 * Describes an numerical object possessing a specified precision.
 * 
 * @author Stephan Heinemann
 */
public interface Precision {
	
	/** the deci precision */
	public static final short DECI = 1;
	
	/** the centi precision */
	public static final short CENTI = 2;
	
	/** the milli precision */
	public static final short MILLI = 3;
	
	/** the hecto-micro precision */
	public static final short HECTO_MICRO = 4;
	
	/** the deca-micro precision */
	public static final short DECA_MICRO = 5;
	
	/** the micro precision */
	public static final short MICRO = 6;
	
	/** the hecto-nano precision */
	public static final short HECTO_NANO = 7;
	
	/** the deca-nano precision */
	public static final short DECA_NANO = 8;
	
	/** the nano precision */
	public static final short NANO = 9;
	
	/** the deci unit */
	public static final double UNIT_DECI = 1E-1d;
	
	/** the centi unit */
	public static final double UNIT_CENTI = 1E-2d;
	
	/** the milli unit */
	public static final double UNIT_MILLI = 1E-3d;
	
	/** the hecto-micro unit */
	public static final double UNIT_HECTO_MICRO = 1E-4d;
	
	/** the deca-micro unit */
	public static final double UNIT_DECA_MICRO = 1E-5d;
	
	/** the micro unit */
	public static final double UNIT_MICRO = 1E-6d;
	
	/** the hecto-nano unit */
	public static final double UNIT_HECTO_NANO = 1E-7d;
	
	/** the deca-nano unit */
	public static final double UNIT_DECA_NANO = 1E-8d;
	
	/** the nano unit */
	public static final double UNIT_NANO = 1E-9d;
	
	/**
	 * Gets the precision of this numerical object.
	 * 
	 * @return the precision of this numerical object.
	 */
	public int getPrecision();
	
	/**
	 * Sets the precision of this (potentially immutable) numerical object.
	 * 
	 * @param precision the precision
	 * 
	 * @return a new precision object based on the original numerical object
	 */
	public Precision setPrecision(int precision);
	
	/**
	 * Gets the original numerical object without any applied precision.
	 * 
	 * @return the original numerical object without any applied precision
	 */
	public Object getOriginal();

}
