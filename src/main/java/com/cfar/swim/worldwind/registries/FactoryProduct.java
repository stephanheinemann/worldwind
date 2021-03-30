package com.cfar.swim.worldwind.registries;

/**
 * Describes the product of a factory.
 * 
 * @author Stephan Heinemann
 *
 * @see Factory
 * @see Registry
 * @see Specification
 */
public interface FactoryProduct {

	/**
	 * Determines whether or not a registered item matches a specification.
	 * 
	 * @param specification the specification to be matched
	 * 
	 * @return true if the this item matches the specification, false otherwise
	 */
	public boolean matches(Specification<? extends FactoryProduct> specification);
	
}
