package com.cfar.swim.worldwind.managing;

import java.util.List;

import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.registries.Properties;
import com.cfar.swim.worldwind.registries.Specification;

/**
 * Abstracts a feature tuning of a planner.
 * 
 * @author Stephan Heinemann
 *
 * @see FeatureTuning
 */
public abstract class PlannerTuning extends FeatureTuning<Planner> {
	
	/**
	 * Constructs a new planner tuning based on a planner specification and
	 * features.
	 * 
	 * @param specification the planner specification
	 * @param features the features
	 * 
	 * @throws IllegalArgumentException if the planner specification or
	 *         features are invalid
	 * 
	 * @see FeatureTuning#FeatureTuning(Specification, Features)
	 */
	public PlannerTuning(
			Specification<Planner> specification, Features features) {
		super(specification, features);
	}
	
	/**
	 * Tunes the specification of the planner of this tuning.
	 * 
	 * @return the tuned candidate properties for the specification
	 * 
	 * @see Tuning#tune()
	 */
	@Override
	public List<Properties<Planner>> tune() {
		return this.tune(this.getSpecification());
	}
	
	/**
	 * Tunes the specification of a planner according to the features
	 * of this planner tuning.
	 * 
	 * @param specification the specification to be tuned.
	 * 
	 * @return the tuned candidate properties for the specification
	 * 
	 * @see Tuning#tune(Specification)
	 */
	@Override
	public List<Properties<Planner>> tune(
			Specification<Planner> specfication) {
		return this.tune(specfication, this.getFeatures());
	}
	
	/**
	 * Tunes the specification of a planner according to features.
	 * 
	 * @param specification the specification to be tuned.
	 * 
	 * @return the tuned candidate properties for the specification
	 * 
	 * @see FeatureTuning#tune(Specification, Features)
	 */
	@Override
	public abstract List<Properties<Planner>> tune(
			Specification<Planner> specification, Features features);
	
}
