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

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import com.cfar.swim.worldwind.javafx.TrajectoryStylist;
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
import ca.ubc.cs.beta.aeatk.parameterconfigurationspace.ParameterConfiguration.ParameterStringFormat;
import ca.ubc.cs.beta.aeatk.targetalgorithmevaluator.AbstractSyncTargetAlgorithmEvaluator;
import ca.ubc.cs.beta.aeatk.targetalgorithmevaluator.TargetAlgorithmEvaluatorRunObserver;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.util.Logging;

public class SmacPlannerEvaluator extends AbstractSyncTargetAlgorithmEvaluator {
	
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

	@Override
	public boolean isRunFinal() {
		// TODO Auto-generated method stub
		//return false;
		return true;
	}

	@Override
	public boolean areRunsPersisted() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean areRunsObservable() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	protected void subtypeShutdown() {
		// TODO Auto-generated method stub

	}
	
	/**
	 * Evaluates run configurations.
	 * 
	 * @see AbstractSyncTargetAlgorithmEvaluator#evaluateRun(List, TargetAlgorithmEvaluatorRunObserver)
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
			Specification<Planner> plannerSpecification = this.createPlannerSpecification(arc);
			this.managedSession.getPlannerFactory().setSpecification(plannerSpecification);
			ManagedPlanner managedPlanner = (ManagedPlanner) this.managedSession.getPlannerFactory().createInstance();
			
			managedPlanner.addPlanRevisionListener(new PlanRevisionListener() {
				@Override
				public void revisePlan(Trajectory trajectory) {
					TrajectoryStylist.styleTrajectory(trajectory);
					managedScenario.setTrajectory(trajectory);
					// TODO: first or specified revision could terminate
					// TODO: report intermediate performance
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
			try {
				this.executor.submit(managedPlanner);
				this.executor.awaitTermination(
						(long) arc.getAlgorithmExecutionConfiguration().getAlgorithmMaximumCutoffTime(),
						TimeUnit.SECONDS);
				managedPlanner.terminate();
			} catch (Exception e) {
				e.printStackTrace();
			}
			
			// TODO: use intermediate results and quality improvements
			ExistingAlgorithmRunResult earr = new ExistingAlgorithmRunResult(
					arc,
					managedScenario.getTrajectory().isEmpty()
						? RunStatus.UNSAT : RunStatus.SAT, // TODO: versus TIMEOUT
					managedPlanner.getPerformance().getQuantity().get(), // runtime
					managedPlanner.getRevisions().size(), // number of quality improvements
					/*1d /  */1000d - managedPlanner.getPerformance().get(), // quality of solution
					//(0d == managedPlanner.getPerformance().get())
					//	? Double.MAX_VALUE : 1d / managedPlanner.getPerformance().get(),
					1l); // problem instance seed (automatically generated instead)
			// TODO: additional run-data: trajectory.toString()
			results.add(earr);
			//runStatusObserver.currentStatus(results);
		}
		
		return results;
	}
	
	/**
	 * Creates a planner specification for an algorithm run configuration.
	 * 
	 * @param arc the algorithm run configuration
	 * 
	 * @return the planner specification for the algorithm run configuration
	 */
	protected Specification<Planner> createPlannerSpecification(AlgorithmRunConfiguration arc) {
		Specification<Planner> plannerSpecification = null;
		
		if (Specification.PLANNER_MGP_ID.equals(arc.getAlgorithmExecutionConfiguration().getAlgorithmExecutable())) {
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
			plannerProperties.setMinimumQuality(Double.parseDouble(arc.getParameterConfiguration().get("minimumQuality")));
			plannerProperties.setMaximumQuality(1.0d/*Double.parseDouble(arc.getParameterConfiguration().get("maximumQuality"))*/);
			plannerProperties.setQualityImprovement(Double.parseDouble(arc.getParameterConfiguration().get("qualityImprovement")));
			plannerProperties.setSignificantChange(0.5d);
			plannerSpecification = new Specification<Planner>(Specification.PLANNER_MGP_ID, plannerProperties);
		} else if (Specification.PLANNER_MTP_ID.equals(arc.getAlgorithmExecutionConfiguration().getAlgorithmExecutable())) {
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
			plannerProperties.setBias(Integer.parseInt(arc.getParameterConfiguration().get("bias")));
			plannerProperties.setEpsilon(Double.parseDouble(arc.getParameterConfiguration().get("epsilon")));
			plannerProperties.setExtension(Extension.valueOf(arc.getParameterConfiguration().get("extension")));
			plannerProperties.setGoalThreshold(Double.parseDouble(arc.getParameterConfiguration().get("goalThreshold")));
			plannerProperties.setMaxIterations(Integer.parseInt(arc.getParameterConfiguration().get("maxIterations")));
			plannerProperties.setNeighborLimit(Integer.parseInt(arc.getParameterConfiguration().get("neighborLimit")));
			plannerProperties.setSampling(Sampling.valueOf(arc.getParameterConfiguration().get("sampling")));
			plannerProperties.setStrategy(Strategy.valueOf(arc.getParameterConfiguration().get("strategy")));
			plannerProperties.setMinimumQuality(Double.parseDouble(arc.getParameterConfiguration().get("initialCostBias")));
			plannerProperties.setMaximumQuality(1.0d/*Double.parseDouble(arc.getParameterConfiguration().get("finalCostBias"))*/);
			plannerProperties.setQualityImprovement(Double.parseDouble(arc.getParameterConfiguration().get("improvementFactor")));
			plannerProperties.setSignificantChange(0.5d);
			plannerSpecification = new Specification<Planner>(Specification.PLANNER_MTP_ID, plannerProperties);
		}
		
		return plannerSpecification;
	}
	
}
