package com.cfar.swim.worldwind.util;

/**
 * Describes a designatable object.
 * 
 * @author Stephan Heinemann
 *
 */
public interface Designatable {

	/**
	 * Gets the designator of this designatable.
	 * 
	 * @return the designator of this designatable
	 */
	public String getDesignator();
	
	/**
	 * Sets the designator of this designatable.
	 * 
	 * @param designator the designator of this designatable
	 */
	public void setDesignator(String designator);

}
