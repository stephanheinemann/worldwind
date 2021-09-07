package com.cfar.swim.worldwind.managing;

import java.util.List;

import com.cfar.swim.worldwind.registries.Properties;
import com.cfar.swim.worldwind.registries.Specification;

/**
 * Abstracts a feature tuning of a tunable.
 * 
 * @author Stephan Heineamnn
 *
 * @param <Tunable> the tunable
 * 
 * @see AbstractTuning
 */
public abstract class FeatureTuning<Tunable> extends AbstractTuning<Tunable> {
	
	/** the features of this feature tuning */
	private Features features;
	
	/**
	 * Constructs a new feature tuning based on a specification and features.
	 * 
	 * @param specification the specification
	 * @param features the features
	 * 
	 * @throws IllegalArgumentException if the specification or features are
	 *         invalid
	 * 
	 * @see AbstractTuning#AbstractTuning(Specification)
	 */
	public FeatureTuning(
			Specification<Tunable> specification, Features features) {
		super(specification);
		this.setFeatures(features);
	}
	
	/**
	 * Gets the features of this feature tuning.
	 * 
	 * @return the features of this feature tuning
	 */
	public Features getFeatures() {
		return this.features;
	}
	
	/**
	 * Sets the features of this feature tuning.
	 * 
	 * @param features the features to be set
	 * 
	 * @throws IllegalArgumentException if the features are invalid
	 */
	public void setFeatures(Features features) {
		if (null == features) {
			throw new IllegalArgumentException("features are invalid");
		}
		this.features = features;
	}
	
	/**
	 * Tunes the specification of a tunable according to features.
	 * 
	 * @param specification the specification to be tuned.
	 * 
	 * @return the tuned candidate properties for the specification
	 */
	public abstract List<Properties<Tunable>> tune(
			Specification<Tunable> specification, Features features);
	
}
