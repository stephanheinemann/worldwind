package com.cfar.swim.worldwind.registries;

/**
 * Abstracts a factory of registered items. Factories can create items
 * such as environments and planners according to a customized specification.
 * 
 * @author Stephan Heinemann
 *
 * @param <Registree> the registered item
 * 
 * @see Registry
 * @see Specification
 */
public abstract class AbstractFactory<Registree> implements Factory<Registree> {

	/** the specification of this abstract factory */
	protected Specification<Registree> specification;
	
	/**
	 * Constructs a new abstract factory without a customized specification.
	 */
	public AbstractFactory() {
		this.specification = null;
	}
	
	/**
	 * Constructs a new abstract factory to create registered items according
	 * to a customized specification.
	 * 
	 * @param specification the specification describing the registered item
	 */
	public AbstractFactory(Specification<Registree> specification) {
		this.specification = specification;
	}
	
	/**
	 * Gets the specification of this abstract factory.
	 * 
	 * @return the specification of this abstract factory
	 * 
	 * @see Factory#getSpecification()
	 */
	@Override
	public Specification<Registree> getSpecification() {
		return this.specification;
	}
	
	/**
	 * Sets the specification of this abstract factory.
	 * 
	 * @param specification the specification to be set
	 * 
	 * @see Factory#setSpecification(Specification)
	 */
	@Override
	public void setSpecification(Specification<Registree> specification) {
		this.specification = specification;
	}
	
	/**
	 * Determines whether or not this abstract factory has a specification.
	 * 
	 * @return true if this abstract factory has a specification,
	 *         false otherwise
	 * 
	 * @see Factory#hasSpecification()
	 */
	@Override
	public boolean hasSpecification() {
		return (null != this.specification);
	}
	
	/**
	 * Creates an item according to the specification of this abstract factory.
	 * 
	 * @return the created item
	 * 
	 * @see Factory#createInstance()
	 */
	@Override
	public abstract Registree createInstance();

}
