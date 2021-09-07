package com.cfar.swim.worldwind.managing;

import java.util.List;

import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.registries.Properties;
import com.cfar.swim.worldwind.registries.Specification;

/**
 * Realizes a heuristic planner tuning based on features.
 * 
 * @author Stephan Heinemann
 *
 * @see PlannerTuning
 */
public class HeuristicPlannerTuning extends PlannerTuning {
	
	/**
	 * Constructs a new heuristic planner tuning based on a planner
	 * specification and features.
	 * 
	 * @param specification the planner specification
	 * @param features the features
	 * 
	 * @throws IllegalArgumentException if the planner specification or
	 *         features are invalid
	 *
	 * @see PlannerTuning#PlannerTuning(Specification, Features)
	 */
	public HeuristicPlannerTuning(
			Specification<Planner> specification, Features features) {
		super(specification, features);
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
	public List<Properties<Planner>> tune(
			Specification<Planner> specification, Features features) {
		// TODO Auto-generated method stub
		return null;
	}

}
