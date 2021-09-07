package com.cfar.swim.worldwind.managing;

import com.cfar.swim.worldwind.registries.Specification;

/**
 * Abstracts a tuning of a tunable.
 * 
 * @author Stephan Heinemann
 *
 * @param <Tunable>
 */
public abstract class AbstractTuning<Tunable> implements Tuning<Tunable> {
	
	/** the specification of the tunable of this feature tuning */
	private Specification<Tunable> specification;
	
	/**
	 * Constructs a new abstract tuning based on a specification of the
	 * tunable.
	 * 
	 * @param specification the specification of the tunable
	 * 
	 * @throws IllegalArgumentException if the specification is invalid
	 */
	public AbstractTuning(Specification<Tunable> specification) {
		this.setSpecification(specification);
	}
	
	/**
	 * Gets the specification of the tunable of this abstract tuning.
	 * 
	 * @return the specification of the tunable of this abstract tuning
	 * 
	 * @see Tuning#getSpecification()
	 */
	@Override
	public Specification<Tunable> getSpecification() {
		return this.specification;
	}
	
	/**
	 * Sets the specification of the tunable of this abstract tuning.
	 * 
	 * @param specification the specification to be set
	 * 
	 * @throws IllegalArgumentException if the specification is invalid
	 *
	 * @see Tuning#setSpecification(Specification)
	 */
	@Override
	public void setSpecification(Specification<Tunable> specification) {
		if (null == specification) {
			throw new IllegalArgumentException("specification is invalid");
		}
		this.specification = specification;
	}
	
}
