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

import java.io.IOException;
import java.io.InputStream;
import java.net.URI;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.flight.FlightPhase;
import com.cfar.swim.worldwind.managers.AbstractAutonomicManager;
import com.cfar.swim.worldwind.managers.AutonomicManager;
import com.cfar.swim.worldwind.managers.heuristic.HeuristicPlannerTuning;
import com.cfar.swim.worldwind.managing.FeatureCategory;
import com.cfar.swim.worldwind.managing.Features;
import com.cfar.swim.worldwind.managing.NumericPerformance;
import com.cfar.swim.worldwind.managing.NumericQuality;
import com.cfar.swim.worldwind.managing.NumericQuantity;
import com.cfar.swim.worldwind.managing.PlannerTuning;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.planners.managed.ManagedGridPlanner;
import com.cfar.swim.worldwind.planners.managed.ManagedPlanner;
import com.cfar.swim.worldwind.planners.managed.ManagedTreePlanner;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.managers.SmacManagerProperties;
import com.cfar.swim.worldwind.registries.planners.cgs.OADStarProperties;
import com.cfar.swim.worldwind.registries.planners.rrt.OADRRTreeProperties;
import com.cfar.swim.worldwind.session.Scenario;
import com.cfar.swim.worldwind.session.Session;
import com.cfar.swim.worldwind.util.Identifiable;

import ca.ubc.cs.beta.aeatk.acquisitionfunctions.AcquisitionFunctions;
import ca.ubc.cs.beta.aeatk.algorithmexecutionconfiguration.AlgorithmExecutionConfiguration;
import ca.ubc.cs.beta.aeatk.logging.LogLevel;
import ca.ubc.cs.beta.aeatk.objectives.RunObjective;
import ca.ubc.cs.beta.aeatk.parameterconfigurationspace.ParameterConfiguration;
import ca.ubc.cs.beta.aeatk.parameterconfigurationspace.ParameterConfiguration.ParameterStringFormat;
import ca.ubc.cs.beta.aeatk.parameterconfigurationspace.ParameterConfigurationSpace;
import ca.ubc.cs.beta.aeatk.probleminstance.InstanceListWithSeeds;
import ca.ubc.cs.beta.aeatk.probleminstance.ProblemInstance;
import ca.ubc.cs.beta.aeatk.probleminstance.seedgenerator.InstanceSeedGenerator;
import ca.ubc.cs.beta.aeatk.probleminstance.seedgenerator.RandomInstanceSeedGenerator;
import ca.ubc.cs.beta.aeatk.random.SeedableRandomPool;
import ca.ubc.cs.beta.aeatk.smac.SMACOptions;
import ca.ubc.cs.beta.aeatk.targetalgorithmevaluator.decorators.functionality.OutstandingEvaluationsTargetAlgorithmEvaluatorDecorator;
import ca.ubc.cs.beta.smac.builder.SMACBuilder;
import ca.ubc.cs.beta.smac.configurator.SequentialModelBasedAlgorithmConfiguration;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.util.Logging;

/**
 * Realizes a sequential model-based algorithm configuration autonomic manager.
 *  
 * @author Stephan Heinemann
 *
 */
public class SmacAutonomicManager extends AbstractAutonomicManager implements AutonomicManager {
	
	/** the parameter configuration space resource of the managed grid planner */
	public static final String MGP_PCS_RESOURCE = "mgp.pcs";
	
	/** the parameter configuration space resource of the managed tree planner */
	public static final String MTP_PCS_RESOURCE = "mtp.pcs";
	
	/** the workspace resource of this SMAC autonomic manager */
	private URI workspaceResource = URI.create(SmacManagerProperties.WORKSPACE_RESOURCE);
	
	/** the training runs of this SMAC autonomic manager */
	private long trainingRuns = SmacManagerProperties.TRAINING_RUNS;
	
	/** the training run cut-off of this SMAC autonomic manager */
	private Duration trainingRunCutOff = Duration.ofSeconds(SmacManagerProperties.TRAINING_RUN_CUTOFF);
	
	/** the manager mode of this SMAC autonomic manager */
	private SmacManagerMode managerMode = SmacManagerMode.EXECUTING;
	
	/** the SMACs of this SMAC autonomic manager */
	private Map<String, SequentialModelBasedAlgorithmConfiguration> smacs = new HashMap<>();
	
	/** the SMAC planner evaluator of this SMAC autonomic manager */
	private SmacPlannerEvaluator plannerEvaluator;
	
	/**
	 * Gets the identifier of this SMAC autonomic manager.
	 * 
	 * @return the identifier of this SMAC autonomic manager
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return Specification.MANAGER_SMAC_ID;
	}
	
	/**
	 * Gets the workspace resource of this SMAC autonomic manager.
	 * 
	 * @return the workspace resource of this SMAC autonomic manager
	 */
	public URI getWorkspaceResource() {
		return this.workspaceResource;
	}
	
	/**
	 * Sets the workspace resource of this SMAC autonomic manager.
	 * 
	 * @param workspaceResource the workspace resource to be set
	 * 
	 * @throws IllegalArgumentException if the workspace resource is invalid
	 */
	public void setWorkspaceResource(URI workspaceResource) {
		if (null == workspaceResource) {
			throw new IllegalArgumentException("workspace resource is invalid");
		}
		this.workspaceResource = workspaceResource;
	}
	
	/**
	 * Gets the training runs of this SMAC autonomic manager.
	 * 
	 * @return the training runs of this SMAC autonomic manager
	 */
	public long getTrainingRuns() {
		return this.trainingRuns;
	}
	
	/**
	 * Sets the training runs of this SMAC autonomic manager.
	 * 
	 * @param trainingRuns the training runs to be set
	 * 
	 * @throws IllegalArgumentException if the training runs are less than 1
	 */
	public void setTrainingRuns(long trainingRuns) {
		if (1l > trainingRuns) {
			throw new IllegalArgumentException("training runs is invalid");
		}
		this.trainingRuns = trainingRuns;
	}
	
	/**
	 * Gets the training run cut-off of this SMAC autonomic manager.
	 * 
	 * @return the training run cut-off of this SMAC autonomic manager
	 */
	public Duration getTrainingRunCutOff() {
		return this.trainingRunCutOff;
	}
	
	/**
	 * Sets the training run cut-off of this SMAC autonomic manager.
	 * 
	 * @param trainingRunCutOff the training run cut-off to be set
	 * 
	 * @throws IllegalArgumentException if the training run cut-off is invalid
	 */
	public void setTrainingRunCutOff(Duration trainingRunCutOff) {
		if ((null == trainingRunCutOff) || trainingRunCutOff.isZero()
				|| trainingRunCutOff.isNegative()) {
			throw new IllegalArgumentException("training runs is invalid");
		}
		this.trainingRunCutOff = trainingRunCutOff;
	}
	
	/**
	 * Gets the manager mode of this SMAC autonomic manager.
	 * 
	 * @return the manager mode of this SMAC autonomic manager
	 */
	public SmacManagerMode getManagerMode() {
		return this.managerMode;
	}
	
	/**
	 * Sets the manager mode of this SMAC autonomic manager.
	 * 
	 * @param managerMode the manager mode to be set
	 */
	public void setManagerMode(SmacManagerMode managerMode) {
		this.managerMode = managerMode;
	}
	
	/**
	 * Creates a new SMAC planner tuning for this SMAC autonomic manager based
	 * on a planner specification and features.
	 * 
	 * @param specification the planner specification
	 * @param features the features
	 * 
	 * @return the created SMAC planner tuning
	 * 
	 * @see AbstractAutonomicManager#createPlannerTuning(Specification, Features)
	 */
	@Override
	public PlannerTuning createPlannerTuning(
			Specification<Planner> specification, Features features) {
		Set<FeatureCategory> categories = new HashSet<>();
		categories.add(FlightPhase.createCruise(features));
		categories.add(FlightPhase.createTransition(features));
		categories.add(FlightPhase.createTerminal(features));
		categories.add(FlightPhase.createUrgency(features));
		categories.add(FlightPhase.createEmergency(features));
		/*
		categories.add(Severity.createLow(features));
		categories.add(Severity.createModerate(features));
		categories.add(Severity.createSubstantial(features));
		categories.add(Severity.createSevere(features));
		categories.add(Severity.createCritical(features));
		categories.add(Severity.createFatal(features));
		*/
		/*
		categories.add(Difficulty.createLow(features));
		categories.add(Difficulty.createModerate(features));
		categories.add(Difficulty.createSubstantial(features));
		categories.add(Difficulty.createSevere(features));
		categories.add(Difficulty.createCritical(features));
		*/
		return new SmacPlannerTuning(specification, features,
				this.getKnowledgeBase(), categories);
	}
	
	/**
	 * Creates managed planner SMAC options from a managed planner
	 * specification.
	 * 
	 * @param specification the managed planner specification
	 * 
	 * @return the managed planner SMAC options if supported, null otherwise
	 */
	private SMACOptions createManagedPlannerOptions(
			Specification<Planner> specification) {
		SMACOptions smacOptions = null;
		
		if (Specification.PLANNER_MGP_ID.equals(specification.getId())) {
			smacOptions = this.createManagedGridPlannerOptions();
		} else if (Specification.PLANNER_MTP_ID.equals(specification.getId())) {
			smacOptions = this.createManagedTreePlannerOptions();
		} // TODO: managed roadmap planner
		
		return smacOptions;
	}
	
	/**
	 * Creates managed grid planner SMAC options.
	 * 
	 * @return the managed grid planner SMAC options
	 */
	private SMACOptions createManagedGridPlannerOptions() {
		SMACOptions smacOptions = new SMACOptions();
		
		String mgpWorkspace = this.getWorkspaceResource().getPath()
				+ "/" + Specification.PLANNER_MGP_ID;
		smacOptions.adaptiveCapping = false; // QUALITY
		smacOptions.alwaysRunInitialConfiguration = true;
		smacOptions.experimentDir = mgpWorkspace;
		smacOptions.expFunc = AcquisitionFunctions.EI;
		smacOptions.logOptions.consoleLogLevel = LogLevel.INFO;
		smacOptions.logOptions.logLevel = LogLevel.INFO;
		smacOptions.randomForestOptions.logModel = false; // QUALITY
		smacOptions.scenarioConfig._runObj = RunObjective.QUALITY;
		smacOptions.scenarioConfig.algoExecOptions.algoExec = Specification.PLANNER_MGP_ID;
		smacOptions.scenarioConfig.algoExecOptions.algoExecDir = mgpWorkspace;
		smacOptions.scenarioConfig.algoExecOptions.cutoffTime = this.getTrainingRunCutOff().toSeconds();
		smacOptions.scenarioConfig.algoExecOptions.deterministic = true;
		smacOptions.scenarioConfig.algoExecOptions.paramFileDelegate.paramFile =
				mgpWorkspace + "/" + SmacAutonomicManager.MGP_PCS_RESOURCE;
		smacOptions.scenarioConfig.limitOptions.totalNumRunsLimit = this.getTrainingRuns();
		smacOptions.scenarioConfig.outputDirectory = mgpWorkspace;
		smacOptions.stateOpts.restoreStateFrom = mgpWorkspace;
		/*
		smacOptions.warmStartOptions.restoreIteration = 32;
		smacOptions.warmStartOptions.warmStartStateFrom = mtpWorkspace;
		*/
		
		return smacOptions;
	}
	
	/**
	 * Creates managed tree planner SMAC options.
	 * 
	 * @return the managed tree planner SMAC options
	 */
	private SMACOptions createManagedTreePlannerOptions() {
		SMACOptions smacOptions = new SMACOptions();
		
		String mtpWorkspace = this.getWorkspaceResource().getPath()
				+ "/" + Specification.PLANNER_MTP_ID;
		smacOptions.adaptiveCapping = false; // QUALITY
		smacOptions.alwaysRunInitialConfiguration = true;
		smacOptions.experimentDir = mtpWorkspace;
		smacOptions.expFunc = AcquisitionFunctions.EI;
		smacOptions.logOptions.consoleLogLevel = LogLevel.INFO;
		smacOptions.logOptions.logLevel = LogLevel.INFO;
		smacOptions.randomForestOptions.logModel = false; // QUALITY
		smacOptions.scenarioConfig._runObj = RunObjective.QUALITY;
		smacOptions.scenarioConfig.algoExecOptions.algoExec = Specification.PLANNER_MTP_ID;
		smacOptions.scenarioConfig.algoExecOptions.algoExecDir = mtpWorkspace;
		smacOptions.scenarioConfig.algoExecOptions.cutoffTime = this.getTrainingRunCutOff().toSeconds();
		smacOptions.scenarioConfig.algoExecOptions.deterministic = false;
		smacOptions.scenarioConfig.algoExecOptions.paramFileDelegate.paramFile =
				mtpWorkspace + "/" + SmacAutonomicManager.MTP_PCS_RESOURCE;
		smacOptions.scenarioConfig.limitOptions.totalNumRunsLimit = this.getTrainingRuns();
		smacOptions.scenarioConfig.outputDirectory = mtpWorkspace;
		smacOptions.stateOpts.restoreStateFrom = mtpWorkspace;
		/*
		smacOptions.warmStartOptions.restoreIteration = 32;
		smacOptions.warmStartOptions.warmStartStateFrom = mtpWorkspace;
		*/
		
		return smacOptions;
	}
	
	/**
	 * Creates a managed planner algorithm execution configuration based on
	 * managed planner SMAC options and default features.
	 * 
	 * @param smacOptions the managed planner SMAC options
	 * @param defaultFeatures the default features
	 * 
	 * @return the managed planner algorithm execution configuration based on
	 *         the managed planner SMAC options and default features if
	 *         supported, null otherwise 
	 */
	private AlgorithmExecutionConfiguration createManagedPlannerAec(
			SMACOptions smacOptions,
			Features defaultFeatures) {
		AlgorithmExecutionConfiguration aec = null;
		
		if (Specification.PLANNER_MGP_ID.equals(smacOptions.scenarioConfig.algoExecOptions.algoExec)) {
			aec = this.createManagedGridPlannerAec(smacOptions, defaultFeatures);
		} else if (Specification.PLANNER_MTP_ID.equals(smacOptions.scenarioConfig.algoExecOptions.algoExec)) {
			aec = this.createManagedTreePlannerAec(smacOptions, defaultFeatures);
		} // TODO: managed roadmap planner
			
		return aec;
	}
	
	/**
	 * Creates a managed grid planner algorithm execution configuration based
	 * on managed grid planner SMAC options and default features.
	 * 
	 * @param mgpOptions the managed grid planner SMAC options
	 * @param defaultFeatures the default features
	 * 
	 * @return the managed grid planner algorithm execution configuration based
	 *         on the managed grid planner SMAC options and default features
	 */
	private AlgorithmExecutionConfiguration createManagedGridPlannerAec(
			SMACOptions mgpOptions,
			Features defaultFeatures) {
		// copy MGP PCS resource
		InputStream mpgPcsInputStream = this.getClass()
				.getResourceAsStream(SmacAutonomicManager.MGP_PCS_RESOURCE);
		try {
			Files.createDirectories(Path.of(mgpOptions.experimentDir));
			Files.copy(mpgPcsInputStream, Path.of(
					mgpOptions.scenarioConfig.algoExecOptions.paramFileDelegate.paramFile),
					StandardCopyOption.REPLACE_EXISTING);
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		// create MGP PCS
		ParameterConfigurationSpace mgpPcs = new ParameterConfigurationSpace(
				mgpOptions.scenarioConfig.algoExecOptions.paramFileDelegate.paramFile);
		
		// create MGP heuristic default configuration
		Specification<Planner> mgpSpec = new Specification<Planner>(Specification.PLANNER_MGP_ID, new OADStarProperties());
		HeuristicPlannerTuning mgpTuning = new HeuristicPlannerTuning(mgpSpec, defaultFeatures);
		OADStarProperties mgpProperties = (OADStarProperties) mgpTuning.tune().get(0);
		mgpOptions.initialIncumbent = "-minimumQuality '" + Double.toString(mgpProperties.getMinimumQuality()) + "' "
				+ "-maximumQuality '" + Double.toString(mgpProperties.getMaximumQuality()) + "' "
				+ "-qualityImprovement '" + Double.toString(mgpProperties.getQualityImprovement()) + "'";
		
		// create MGP AEC
		return new AlgorithmExecutionConfiguration(
				mgpOptions.scenarioConfig.algoExecOptions.algoExec, // executable
				mgpOptions.scenarioConfig.algoExecOptions.algoExecDir, // directory (irrelevant)
				mgpPcs, // from file resource
				false, // no cluster (irrelevant)
				mgpOptions.scenarioConfig.algoExecOptions.deterministic, // deterministic algorithm
				mgpOptions.scenarioConfig.algoExecOptions.cutoffTime); // terminate explicitly (cut-off)
	}
	
	/**
	 * Creates a managed tree planner algorithm execution configuration based
	 * on managed tree planner SMAC options and default features.
	 * 
	 * @param mtpOptions the managed tree planner SMAC options
	 * @param defaultFeatures the default features
	 * 
	 * @return the managed tree planner algorithm execution configuration based
	 *         on the managed tree planner SMAC options and default features
	 */
	private AlgorithmExecutionConfiguration createManagedTreePlannerAec(
			SMACOptions mtpOptions,
			Features defaultFeatures) {
		// copy MTP PCS resource
		InputStream mtpPcsInputStream = this.getClass()
				.getResourceAsStream(SmacAutonomicManager.MTP_PCS_RESOURCE);
		try {
			Files.createDirectories(Path.of(mtpOptions.experimentDir));
			Files.copy(mtpPcsInputStream, Path.of(
					mtpOptions.scenarioConfig.algoExecOptions.paramFileDelegate.paramFile),
					StandardCopyOption.REPLACE_EXISTING);
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		// create MTP PCS
		ParameterConfigurationSpace mtpPcs = new ParameterConfigurationSpace(
				mtpOptions.scenarioConfig.algoExecOptions.paramFileDelegate.paramFile);
		
		// create MTP heuristic default configuration
		Specification<Planner> mtpSpec = new Specification<Planner>(Specification.PLANNER_MTP_ID, new OADRRTreeProperties());
		HeuristicPlannerTuning mtpTuning = new HeuristicPlannerTuning(mtpSpec, defaultFeatures);
		OADRRTreeProperties mtpProperties = (OADRRTreeProperties) mtpTuning.tune().get(0);
		mtpOptions.initialIncumbent = "-sampling '" + mtpProperties.getSampling().name() + "' "
				+ "-strategy '" + mtpProperties.getStrategy().name() + "' "
				//+ "-extension '" + mtpProperties.getExtension().name() + "' "
				+ "-maxIterations '" + Integer.toString(mtpProperties.getMaxIterations()) + "' "
				+ "-epsilon '" + Double.toString(mtpProperties.getEpsilon()) + "' "
				+ "-bias '" + Double.toString(mtpProperties.getBias()) + "' "
				//+ "-goalThreshold '" + Double.toString(mtpProperties.getGoalThreshold()) + "' "
				+ "-neighborLimit '" + Double.toString(mtpProperties.getNeighborLimit()) + "' "
				+ "-initialCostBias '" + Double.toString(mtpProperties.getMinimumQuality()) + "' "
				+ "-finalCostBias '" + Double.toString(mtpProperties.getMaximumQuality()) + "' "
				+ "-improvementFactor '" + Double.toString(mtpProperties.getQualityImprovement()) + "'";
		
		// create MTP AEC
		return new AlgorithmExecutionConfiguration(
				mtpOptions.scenarioConfig.algoExecOptions.algoExec, // executable
				mtpOptions.scenarioConfig.algoExecOptions.algoExecDir, // directory (irrelevant)
				mtpPcs, // from file resource
				false, // no cluster (irrelevant)
				mtpOptions.scenarioConfig.algoExecOptions.deterministic, // non-deterministic algorithm
				mtpOptions.scenarioConfig.algoExecOptions.cutoffTime); // terminate explicitly (cut-off)
	}
	
	/**
	 * Gets the supported scenarios of a managed session for managed planner
	 * SMAC options.
	 * 
	 * @param managedSession the managed session
	 * @param options the managed planner SMAC options
	 * 
	 * @return the supported scenarios of the managed session for the managed
	 *         planner SMAC options
	 */
	private Set<Scenario> getSupportedScenarios(Session managedSession, SMACOptions options) {
		Set<Scenario> supportedScenarios = new HashSet<>();
		
		for (Scenario scenario : managedSession.getScenarios()) {
			ManagedPlanner managedPlanner = null;
			if (Specification.PLANNER_MGP_ID.equals(options.scenarioConfig.algoExecOptions.algoExec)) {
				managedPlanner = new ManagedGridPlanner(scenario.getAircraft(), scenario.getEnvironment());
			} else if (Specification.PLANNER_MTP_ID.equals(options.scenarioConfig.algoExecOptions.algoExec)) {
				managedPlanner = new ManagedTreePlanner(scenario.getAircraft(), scenario.getEnvironment());
			} // TODO: managed roadmap planner
			
			List<Position> pois = scenario.getWaypoints().stream()
					.map(w -> (Position) w)
					.collect(Collectors.toList());
			
			if ((null != managedPlanner)
					&& !scenario.getId().equals(Scenario.DEFAULT_SCENARIO_ID)
					&& managedPlanner.supports(scenario.getAircraft())
					&& managedPlanner.supports(scenario.getEnvironment())
					&& managedPlanner.supports(pois)
					&& (1 < pois.size())) {
				supportedScenarios.add(scenario);
			}
		}
		
		return supportedScenarios;
	}
	
	/**
	 * Creates problem instances for scenarios.
	 * 
	 * @param scenarios the scenarios to create problem instances for
	 * 
	 * @return the problem instances created from scenarios
	 */
	private List<ProblemInstance> createProblemInstances(Set<Scenario> scenarios) {
		List<ProblemInstance> problemInstances = new ArrayList<>();
		
		int instanceId = 1;
		for (Scenario scenario : scenarios) {
			ProblemInstance problemInstance = new ProblemInstance(
					scenario.getId(),
					instanceId++,
					new Features(scenario, this.getFeatureHorizon()));
			problemInstances.add(problemInstance);
		}
		
		return problemInstances;
	}
	
	/**
	 * Initializes this SMAC autonomic manager for a managed session.
	 * 
	 * @param managedSession the managed session
	 */
	@Override
	protected void initialize(Session managedSession) {
		if (SmacManagerMode.EXECUTING == this.getManagerMode()) {
			super.initialize(managedSession);
		} else if (SmacManagerMode.TRAINING == this.getManagerMode()){
			SMACBuilder builder = new SMACBuilder();
			
			// TODO: check TargetAlgorithmEvaluatorBuilder
			this.plannerEvaluator = new SmacPlannerEvaluator(managedSession);
			// TODO: register event handlers for run notifications?
			//EventManager eventManager = new EventManager();
			
			for (Specification<Planner> managedPlannerSpec :
				managedSession.getManagedPlannerSpecifications()) {
				
				// create managed planner SMAC context
				SMACOptions options = this.createManagedPlannerOptions(managedPlannerSpec);
				Set<Scenario> scenarios = this.getSupportedScenarios(managedSession, options);
				
				if (!scenarios.isEmpty()) {
					// extract default features
					Features defaultFeatures = new Features(
							scenarios.stream().findAny().get(),
							this.getFeatureHorizon());
					AlgorithmExecutionConfiguration aec = this.createManagedPlannerAec(
							options, defaultFeatures);
					List<ProblemInstance> problems = this.createProblemInstances(scenarios);
					SeedableRandomPool seeds = options.seedOptions.getSeedableRandomPool();
					InstanceSeedGenerator isg = new RandomInstanceSeedGenerator(problems, problems.hashCode());
					InstanceListWithSeeds ilws = new InstanceListWithSeeds(isg, problems);
					Logging.logger().info(aec.toString());
					Logging.logger().info(options.initialIncumbent);
				
					// create managed planner SMAC
					this.smacs.put(managedPlannerSpec.getId(),
							(SequentialModelBasedAlgorithmConfiguration) builder.getAutomaticConfigurator(
									aec,
									ilws,
									options,
									options.scenarioConfig.algoExecOptions.taeOpts.getAvailableTargetAlgorithmEvaluators(),
									options.scenarioConfig.outputDirectory,
									seeds,
									new OutstandingEvaluationsTargetAlgorithmEvaluatorDecorator(this.plannerEvaluator),
									null)); // run history created internally
				}
			}
			
			if (!this.smacs.isEmpty()) {
				// load knowledge base
				this.getKnowledgeBase().load(this.getKnowledgeBaseResource());
				Logging.logger().info("loaded knowledge base\n" + this.getKnowledgeBase());
			}
		}
	}
	
	/**
	 * Runs this SMAC autonomic manager for a managed session.
	 * 
	 * @param managedSession the managed session
	 */
	@Override
	protected void run(Session managedSession) {
		if (SmacManagerMode.EXECUTING == this.getManagerMode()) {
			super.run(managedSession);
		} else if (SmacManagerMode.TRAINING == this.getManagerMode()) {
			// run training session
			for (String plannerId : this.smacs.keySet()) {
				SequentialModelBasedAlgorithmConfiguration smac = this.smacs.get(plannerId);
				// run SMAC
				smac.run();
				this.plannerEvaluator.recycle();
				ParameterConfiguration incumbent = smac.getIncumbent();
				Logging.logger().info("incumbent: " + incumbent.getFormattedParameterString(ParameterStringFormat.STATEFILE_SYNTAX));
				Logging.logger().info("incumbent performance: " + smac.getEmpericalPerformance(incumbent));
				
				// extend knowledge base
				Set<ProblemInstance> problems = smac.getRunHistory().getUniqueInstancesRan();
				
				for (ProblemInstance problem : problems) {
					Specification<Planner> managedPlannerSpecification = null;
					if (Specification.PLANNER_MGP_ID.equals(plannerId)) {
						managedPlannerSpecification = this.plannerEvaluator
								.createManagedGridPlannerSpecification(incumbent);
					} else if (Specification.PLANNER_MTP_ID.equals(plannerId)) {
						managedPlannerSpecification = this.plannerEvaluator
								.createManagedTreePlannerSpecification(incumbent);
					} // TODO: managed roadmap planner specification
					
					PlannerTuning tuning = this.createPlannerTuning(
							managedPlannerSpecification, (Features) problem.getFeatures());
					double cost = smac.getRunHistory().getEmpiricalCost(
							incumbent,
							Collections.singleton(problem),
							this.getTrainingRunCutOff().toSeconds());
					NumericPerformance performance = new NumericPerformance(
							ZonedDateTime.now(),
							new NumericQuality(SmacPlannerEvaluator.toResultQuality(cost)),
							new NumericQuantity(1));
					this.getKnowledgeBase().getReputation().addTuningPerformance(tuning, performance);
					Logging.logger().info("added performance " + performance.toString());
				}
			}
		}
	}
	
	/**
	 * Cleans up this SMAC autonomic manager.
	 * 
	 * @param managedSession the managed session
	 */
	@Override
	protected void cleanup(Session managedSession) {
		if (SmacManagerMode.EXECUTING == this.getManagerMode()) {
			super.cleanup(managedSession);
		} else if (SmacManagerMode.TRAINING == this.getManagerMode()) {
			if (!this.smacs.isEmpty()) {
				// save knowledge base
				this.getKnowledgeBase().save(this.getKnowledgeBaseResource());
				Logging.logger().info("saved knowledge base\n" + this.getKnowledgeBase());
				this.smacs.clear();
			}
		}
	}
	
	/**
	 * Determines whether or not this SMAC autonomic manager matches a
	 * specification.
	 * 
	 * @param specification the specification to be matched
	 * 
	 * @return true if the this SMAC autonomic manager matches the
	 *         specification, false otherwise
	 * 
	 * @see AbstractAutonomicManager#matches(Specification)
	 */
	@Override
	public synchronized boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = super.matches(specification);
		
		if (matches && (specification.getProperties() instanceof SmacManagerProperties)) {
			SmacManagerProperties properties = (SmacManagerProperties) specification.getProperties();
			matches = (this.getManagerMode() == properties.getManagerMode())
					&& (this.getTrainingRuns() == properties.getTrainingRuns())
					&& (this.getTrainingRunCutOff().equals(Duration.ofSeconds(properties.getTrainingRunCutOff())))
					&& (this.getWorkspaceResource().equals(URI.create(properties.getWorkspaceResource())));
		}
		
		return matches;
	}
	
	/**
	 * Updates this SMAC autonomic manager according to a specification.
	 * 
	 * @param specification the specification to be used for the update
	 * 
	 * @return true if this SMAC autonomic manager has been updated,
	 *         false otherwise
	 * 
	 * @see AbstractAutonomicManager#update(Specification)
	 */
	@Override
	public synchronized boolean update(Specification<? extends FactoryProduct> specification) {
		boolean updated = super.update(specification);
		
		if (updated && (specification.getProperties() instanceof SmacManagerProperties)) {
			SmacManagerProperties properties = (SmacManagerProperties) specification.getProperties();
			this.setManagerMode(properties.getManagerMode());
			this.setTrainingRuns(properties.getTrainingRuns());
			this.setTrainingRunCutOff(Duration.ofSeconds(properties.getTrainingRunCutOff()));
			this.setWorkspaceResource(URI.create(properties.getWorkspaceResource()));
		}
		
		return updated;
	}
	
}
