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
package com.cfar.swim.worldwind.managers.smac;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

import com.cfar.swim.worldwind.javafx.TrajectoryStylist;
import com.cfar.swim.worldwind.managing.PlannerPerformance;
import com.cfar.swim.worldwind.planners.PlanRevisionListener;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.planners.managed.ManagedGoals;
import com.cfar.swim.worldwind.planners.managed.ManagedPlanner;
import com.cfar.swim.worldwind.planners.rrt.Extension;
import com.cfar.swim.worldwind.planners.rrt.Sampling;
import com.cfar.swim.worldwind.planners.rrt.Strategy;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.planners.cgs.OADStarProperties;
import com.cfar.swim.worldwind.registries.planners.rrt.OADRRTreeProperties;
import com.cfar.swim.worldwind.session.Scenario;
import com.cfar.swim.worldwind.session.Session;

import ca.ubc.cs.beta.aeatk.algorithmrunconfiguration.AlgorithmRunConfiguration;
import ca.ubc.cs.beta.aeatk.algorithmrunresult.AlgorithmRunResult;
import ca.ubc.cs.beta.aeatk.algorithmrunresult.ExistingAlgorithmRunResult;
import ca.ubc.cs.beta.aeatk.algorithmrunresult.RunStatus;
import ca.ubc.cs.beta.aeatk.parameterconfigurationspace.ParameterConfiguration;
import ca.ubc.cs.beta.aeatk.parameterconfigurationspace.ParameterConfiguration.ParameterStringFormat;
import ca.ubc.cs.beta.aeatk.targetalgorithmevaluator.AbstractSyncTargetAlgorithmEvaluator;
import ca.ubc.cs.beta.aeatk.targetalgorithmevaluator.AbstractTargetAlgorithmEvaluator;
import ca.ubc.cs.beta.aeatk.targetalgorithmevaluator.TargetAlgorithmEvaluator;
import ca.ubc.cs.beta.aeatk.targetalgorithmevaluator.TargetAlgorithmEvaluatorRunObserver;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.util.Logging;

/**
 * Realizes a SMAC planner evaluator.
 * 
 * @author Stephan Heinemann
 *
 */
public class SmacPlannerEvaluator extends AbstractSyncTargetAlgorithmEvaluator {
	
	/** the maximum termination delay of a SMAC planner evaluator */
	private static final Duration MAX_TERMINATION_DELAY = Duration.ofSeconds(10l);
	
	/** the managed session of this SMAC planner evaluator */
	private final Session managedSession;
	
	/** the executor service of this SMAC planner evaluator */
	// NOTE: super-class executor is private and final
	private ExecutorService executor = Executors.newSingleThreadExecutor();
	
	/**
	 * Constructs a new SMAC planner evaluator of a managed session.
	 * 
	 * @param managedSession the managed session
	 */
	public SmacPlannerEvaluator(Session managedSession) {
		this.managedSession = managedSession;
	}
	
	/**
	 * Determines whether or not SMAC planner evaluator runs are final.
	 * 
	 * @return true
	 * 
	 * @see TargetAlgorithmEvaluator#isRunFinal()
	 */
	@Override
	public boolean isRunFinal() {
		return true;
	}
	
	/**
	 * Determines whether or not SMAC planner evaluator runs are persisted.
	 * 
	 * @return false
	 * 
	 * @see TargetAlgorithmEvaluator#areRunsPersisted()
	 */
	@Override
	public boolean areRunsPersisted() {
		return false;
	}
	
	/**
	 * Determines whether or not SMAC planner evaluator runs are observable.
	 * 
	 * @return false
	 * 
	 * @see TargetAlgorithmEvaluator#areRunsObservable()
	 */
	@Override
	public boolean areRunsObservable() {
		return false;
	}
	
	/**
	 * Shuts down the executor of this SMAC planner evaluator.
	 */
	@Override
	protected void subtypeShutdown() {
		this.executor.shutdown();
	}
	
	/**
	 * Recycles the executor of this SMAC planner evaluator.
	 */
	protected void recycle() {
		this.executor.shutdownNow();
		this.executor = Executors.newSingleThreadExecutor();
	}
	
	/**
	 * Evaluates run configurations.
	 * 
	 * @see AbstractTargetAlgorithmEvaluator#evaluateRun(List, TargetAlgorithmEvaluatorRunObserver)
	 */
	@Override
	public List<AlgorithmRunResult> evaluateRun(List<AlgorithmRunConfiguration> runConfigs,
			TargetAlgorithmEvaluatorRunObserver runStatusObserver) {
		ArrayList<AlgorithmRunResult> results = new ArrayList<>();
		
		for (AlgorithmRunConfiguration arc : runConfigs) {
			Logging.logger().info("planner: " + arc.getAlgorithmExecutionConfiguration().getAlgorithmExecutable());
			Logging.logger().info("scenario: " + arc.getProblemInstanceSeedPair().getProblemInstance().getInstanceName());
			Logging.logger().info("tuning: " + arc.getParameterConfiguration().getFormattedParameterString(ParameterStringFormat.STATEFILE_SYNTAX));
			Logging.logger().info("cut-off: " + arc.getAlgorithmExecutionConfiguration().getAlgorithmMaximumCutoffTime());
			Logging.logger().info(arc.getProblemInstanceSeedPair().getProblemInstance().getFeatures().toString());
			
			// activate managed scenario (problem instance)
			Scenario managedScenario = this.managedSession.getScenario(
					arc.getProblemInstanceSeedPair().getProblemInstance().getInstanceName());
			managedScenario.clearTrajectory();
			this.managedSession.setActiveScenario(managedScenario);
			
			// create managed planner (target algorithm)
			Specification<Planner> plannerSpecification = this.createManagedPlannerSpecification(arc);
			this.managedSession.getPlannerFactory().setSpecification(plannerSpecification);
			ManagedPlanner managedPlanner = (ManagedPlanner) this.managedSession.getPlannerFactory().createInstance();
			
			managedPlanner.addPlanRevisionListener(new PlanRevisionListener() {
				@Override
				public void revisePlan(Trajectory trajectory) {
					TrajectoryStylist.styleTrajectory(trajectory);
					managedScenario.setTrajectory(trajectory);
					
					// TODO: first or specified revision could terminate
					if (null != runStatusObserver) {
						//TODO: runStatusObserver.currentStatus(results);
						if (runStatusObserver instanceof SmacPlannerEvaluatorObserver) {
							// TODO: report intermediate performance?
							((SmacPlannerEvaluatorObserver) runStatusObserver)
								.currentPerformance(managedPlanner.getPerformance());
						}
					}
				}
			});
			
			// setup managed planner
			managedScenario.setPlanner(managedPlanner);
			ManagedGoals managedGoals = new ManagedGoals();
			List<Position> pois = new ArrayList<Position>();
			pois.addAll(managedScenario.getWaypoints());
			managedGoals.setOrigin(pois.remove(0));
			managedGoals.setDestination(pois.remove(pois.size() - 1));
			managedGoals.addAllPois(pois);
			managedGoals.setEtd(managedScenario.getTime());
			managedPlanner.setGoals(managedGoals);
			
			// execute managed planner
			Future<Trajectory>  trajectory = this.executor.submit(managedPlanner);
			try {
				// execute managed planner until cut-off time
				trajectory.get((long) arc.getAlgorithmExecutionConfiguration().getAlgorithmMaximumCutoffTime(), TimeUnit.SECONDS);
			} catch (Exception coe) {
				// terminate managed planner
				managedPlanner.terminate();
				try {
					trajectory.get(SmacPlannerEvaluator.MAX_TERMINATION_DELAY.getSeconds(), TimeUnit.SECONDS);
				} catch (Exception tde) {
					tde.printStackTrace();
				}
			}
			
			// recycle executor if managed planner cannot be terminated in time
			if (!trajectory.isDone()) {
				trajectory.cancel(true);
				this.recycle();
			}
			
			// collect managed planner results
			int improvements = managedPlanner.getRevisions().size();
			Trajectory solution;
			if (0 < improvements) {
				solution = managedPlanner.getRevisions().get(improvements - 1);
			} else {
				solution = new Trajectory();
			}
			PlannerPerformance performance = managedPlanner.getPerformance();
			boolean satisfactory = solution.getPois().contains(managedGoals.getDestination());
			
			if (!satisfactory) {
				Logging.logger().info("unsatisfactory evaluation run");
			}
			
			// TODO: use intermediate results and quality improvements
			ExistingAlgorithmRunResult earr = new ExistingAlgorithmRunResult(
					arc,
					//!managedScenario.getTrajectory().isEmpty()
					satisfactory ? RunStatus.SAT : RunStatus.UNSAT,
					performance.getQuantity().get(), // runtime
					improvements, // number of quality improvements
					satisfactory ?
						//SmacPlannerEvaluator.toResultQuality(performance.getNormalized()) : 0d, // performance of solution
						SmacPlannerEvaluator.toResultQuality(performance.getQuality().getNormalized()) : 0d, // quality of solution
					1l, // problem instance seed (automatically generated instead)
					solution.toString()); // additional run-data
			results.add(earr);
			//runStatusObserver.currentStatus(results);
		}
		
		return results;
	}
	
	/**
	 * Creates a managed planner specification for an algorithm run
	 * configuration.
	 * 
	 * @param arc the algorithm run configuration
	 * 
	 * @return the managed planner specification for the algorithm run
	 *         configuration
	 */
	protected Specification<Planner> createManagedPlannerSpecification(AlgorithmRunConfiguration arc) {
		Specification<Planner> managedPlannerSpecification = null;
		
		if (Specification.PLANNER_MGP_ID.equals(arc.getAlgorithmExecutionConfiguration().getAlgorithmExecutable())) {
			managedPlannerSpecification = this.createManagedGridPlannerSpecification(arc.getParameterConfiguration());
		} else if (Specification.PLANNER_MTP_ID.equals(arc.getAlgorithmExecutionConfiguration().getAlgorithmExecutable())) {
			managedPlannerSpecification = this.createManagedTreePlannerSpecification(arc.getParameterConfiguration());
		} // TODO: managed roadmap planner specification
		
		return managedPlannerSpecification;
	}
	
	/**
	 * Creates a managed grid planner specification for a parameter
	 * configuration.
	 * 
	 * @param config the parameter configuration
	 * 
	 * @return the managed grid planner specification for the parameter
	 *         configuration
	 */
	public Specification<Planner> createManagedGridPlannerSpecification(ParameterConfiguration config) {
		OADStarProperties plannerProperties = new OADStarProperties();
		plannerProperties.setCostPolicy(this.managedSession.getManager().getCostPolicy());
		plannerProperties.setRiskPolicy(this.managedSession.getManager().getRiskPolicy());
		plannerProperties.setMinDeliberation(this.managedSession.getManager().getMinDeliberation().getSeconds());
		plannerProperties.setMaxDeliberation(this.managedSession.getManager().getMaxDeliberation().getSeconds());
		// TODO: review track error basic types (long versus double)
		plannerProperties.setMaxCrossTrackError((long) this.managedSession.getManager().getMaxTrackError().getCrossTrackError());
		plannerProperties.setMaxTimingError(this.managedSession.getManager().getMaxTrackError().getTimingError().toSeconds());
		// TODO: review track error basic types (long versus double)
		plannerProperties.setMaxTakeOffHorizontalError((long) this.managedSession.getManager().getMaxTakeOffError().getHorizontalError());
		plannerProperties.setMaxTakeOffTimingError(this.managedSession.getManager().getMaxTakeOffError().getTimingError().toSeconds());
		// TODO: review track error basic types (long versus double)
		plannerProperties.setMaxLandingHorizontalError((long) this.managedSession.getManager().getMaxLandingError().getHorizontalError());
		plannerProperties.setMaxLandingTimingError(this.managedSession.getManager().getMaxLandingError().getTimingError().toSeconds());
		// TODO: consider dynamic training
		plannerProperties.setMinimumQuality(Double.parseDouble(config.get("minimumQuality")));
		plannerProperties.setMaximumQuality(Double.parseDouble(config.get("maximumQuality")));
		plannerProperties.setQualityImprovement(Double.parseDouble(config.get("qualityImprovement")));
		plannerProperties.setSignificantChange(0.5d);
		return new Specification<Planner>(Specification.PLANNER_MGP_ID, plannerProperties);
	}
	
	/**
	 * Creates a managed tree planner specification for a parameter
	 * configuration.
	 * 
	 * @param config the parameter configuration
	 * 
	 * @return the managed tree planner specification for the parameter
	 *         configuration
	 */
	public Specification<Planner> createManagedTreePlannerSpecification(ParameterConfiguration config) {
		OADRRTreeProperties plannerProperties = new OADRRTreeProperties();
		plannerProperties.setCostPolicy(this.managedSession.getManager().getCostPolicy());
		plannerProperties.setRiskPolicy(this.managedSession.getManager().getRiskPolicy());
		plannerProperties.setMinDeliberation(this.managedSession.getManager().getMinDeliberation().getSeconds());
		plannerProperties.setMaxDeliberation(this.managedSession.getManager().getMaxDeliberation().getSeconds());
		// TODO: review track error basic types (long versus double)
		plannerProperties.setMaxCrossTrackError((long) this.managedSession.getManager().getMaxTrackError().getCrossTrackError());
		plannerProperties.setMaxTimingError(this.managedSession.getManager().getMaxTrackError().getTimingError().toSeconds());
		// TODO: review track error basic types (long versus double)
		plannerProperties.setMaxTakeOffHorizontalError((long) this.managedSession.getManager().getMaxTakeOffError().getHorizontalError());
		plannerProperties.setMaxTakeOffTimingError(this.managedSession.getManager().getMaxTakeOffError().getTimingError().toSeconds());
		// TODO: review track error basic types (long versus double)
		plannerProperties.setMaxLandingHorizontalError((long) this.managedSession.getManager().getMaxLandingError().getHorizontalError());
		plannerProperties.setMaxLandingTimingError(this.managedSession.getManager().getMaxLandingError().getTimingError().toSeconds());
		// TODO: consider dynamic training
		plannerProperties.setBias(Integer.parseInt(config.get("bias")));
		plannerProperties.setEpsilon(Double.parseDouble(config.get("epsilon")));
		plannerProperties.setExtension(Extension.FEASIBLE/*Extension.valueOf(config.get("extension"))*/);
		//plannerProperties.setGoalThreshold(Double.parseDouble(config.get("goalThreshold")));
		plannerProperties.setGoalThreshold(this.managedSession.getManager().getMaxLandingError().getHorizontalError() / 2d);
		plannerProperties.setMaxIterations(Integer.parseInt(config.get("maxIterations")));
		plannerProperties.setNeighborLimit(Integer.parseInt(config.get("neighborLimit")));
		plannerProperties.setSampling(Sampling.valueOf(config.get("sampling")));
		plannerProperties.setStrategy(Strategy.valueOf(config.get("strategy")));
		plannerProperties.setMinimumQuality(Double.parseDouble(config.get("initialCostBias")));
		plannerProperties.setMaximumQuality(Double.parseDouble(config.get("finalCostBias")));
		plannerProperties.setQualityImprovement(Double.parseDouble(config.get("improvementFactor")));
		plannerProperties.setSignificantChange(0.5d);
		return new Specification<Planner>(Specification.PLANNER_MTP_ID, plannerProperties);
	}
	
	/**
	 * Converts a quality value to be maximized into quality value to be
	 * minimized by this SMAC planner evaluator.
	 * 
	 * @param quality the quality value to be converted
	 * 
	 * @return the converted quality value
	 */
	public static double toResultQuality(double quality) {
		// NOTE: 1d / quality leads to SMAC exception (Double.POSITIVE_INFINITY, Double.MAX_VALUE)
		// (0d == quality) ? Double.MAX_VALUE : 1d / quality
		return -quality;
	}
	
}
