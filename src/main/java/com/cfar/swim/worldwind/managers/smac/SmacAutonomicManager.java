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
import java.util.ArrayList;

import com.cfar.swim.worldwind.managers.AbstractAutonomicManager;
import com.cfar.swim.worldwind.managers.AutonomicManager;
import com.cfar.swim.worldwind.managers.heuristic.HeuristicPlannerTuning;
import com.cfar.swim.worldwind.managing.Features;
import com.cfar.swim.worldwind.managing.PlannerTuning;
import com.cfar.swim.worldwind.planners.Planner;
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
import ca.ubc.cs.beta.aeatk.eventsystem.EventManager;
import ca.ubc.cs.beta.aeatk.logging.LogLevel;
import ca.ubc.cs.beta.aeatk.misc.options.OptionLevel;
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
	
	/** the manager mode of this SMAC autonomic manager */
	private SmacManagerMode managerMode = SmacManagerMode.EXECUTING;
	
	/** the SMAC of this SMAC autonomic manager */
	private SequentialModelBasedAlgorithmConfiguration smac = null;
	
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
		return new SmacPlannerTuning(specification, features);
	}
	
	/**
	 * Gets the managed grid planner algorithm execution configuration.
	 * 
	 * @param smacOptions the SMAC options
	 * @param defaultFeatures the default features
	 * 
	 * @return the managed grid planner algorithm execution configuration
	 */
	private AlgorithmExecutionConfiguration getManagedGridPlannerAec(
			SMACOptions smacOptions,
			Features defaultFeatures) {
		// copy MGP PCS resource
		InputStream mpgPcsInputStream = this.getClass()
				.getResourceAsStream(SmacAutonomicManager.MGP_PCS_RESOURCE);
		try {
			Files.createDirectories(Path.of(
					this.getWorkspaceResource().getPath() + "/"
							+ Specification.PLANNER_MGP_ID));
			Files.copy(mpgPcsInputStream, Path.of(
					this.getWorkspaceResource().getPath() + "/"
							+ Specification.PLANNER_MGP_ID + "/"
							+ SmacAutonomicManager.MGP_PCS_RESOURCE),
					StandardCopyOption.REPLACE_EXISTING);
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		// create MGP PCS
		ParameterConfigurationSpace mgpPcs = new ParameterConfigurationSpace(
				this.getWorkspaceResource().getPath() + "/"
						+ Specification.PLANNER_MGP_ID + "/"
						+ SmacAutonomicManager.MGP_PCS_RESOURCE);
		
		// create MGP heuristic default configuration
		Specification<Planner> mgpSpec = new Specification<Planner>(Specification.PLANNER_MGP_ID, new OADStarProperties());
		HeuristicPlannerTuning mgpTuning = new HeuristicPlannerTuning(mgpSpec, defaultFeatures);
		OADStarProperties mgpProperties = (OADStarProperties) mgpTuning.tune().get(0);
		smacOptions.initialIncumbent = "-minimumQuality '" + Double.toString(mgpProperties.getMinimumQuality()) + "' "
				+ "-qualityImprovement '" + Double.toString(mgpProperties.getQualityImprovement()) + "'";
		
		// create MGP AEC
		return new AlgorithmExecutionConfiguration(
				Specification.PLANNER_MGP_ID, // executable
				this.getWorkspaceResource().getPath(), // directory (irrelevant)
				mgpPcs, // from file resource
				false, // no cluster (irrelevant)
				true, // deterministic algorithm
				// terminate explicitly (cut-off)
				smacOptions.scenarioConfig.algoExecOptions.cutoffTime);
	}
	
	/**
	 * Gets the managed tree planner algorithm execution configuration.
	 * 
	 * @param smacOptions the SMAC options
	 * @param defaultFeatures the default features
	 * 
	 * @return the managed tree planner algorithm execution configuration
	 */
	private AlgorithmExecutionConfiguration getManagedTreePlannerAec(
			SMACOptions smacOptions,
			Features defaultFeatures) {
		// copy MTP PCS resource
		InputStream mtpPcsInputStream = this.getClass()
				.getResourceAsStream(SmacAutonomicManager.MTP_PCS_RESOURCE);
		try {
			Files.createDirectories(Path.of(
					this.getWorkspaceResource().getPath() + "/"
							+ Specification.PLANNER_MTP_ID));
			Files.copy(mtpPcsInputStream, Path.of(
					this.getWorkspaceResource().getPath() + "/"
							+ Specification.PLANNER_MTP_ID + "/"
							+ SmacAutonomicManager.MTP_PCS_RESOURCE),
					StandardCopyOption.REPLACE_EXISTING);
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		// create MTP PCS
		ParameterConfigurationSpace mtpPcs = new ParameterConfigurationSpace(
				this.getWorkspaceResource().getPath() + "/"
						+ Specification.PLANNER_MTP_ID + "/"
						+ SmacAutonomicManager.MTP_PCS_RESOURCE);
		
		// create MTP heuristic default configuration
		Specification<Planner> mtpSpec = new Specification<Planner>(Specification.PLANNER_MTP_ID, new OADRRTreeProperties());
		HeuristicPlannerTuning mtpTuning = new HeuristicPlannerTuning(mtpSpec, defaultFeatures);
		OADRRTreeProperties mtpProperties = (OADRRTreeProperties) mtpTuning.tune().get(0);
		smacOptions.initialIncumbent = "-sampling '" + mtpProperties.getSampling().name() + "' "
				+ "-strategy '" + mtpProperties.getStrategy().name() + "' "
				+ "-extension '" + mtpProperties.getExtension().name() + "' "
				+ "-maxIterations '" + Integer.toString(mtpProperties.getMaxIterations()) + "' "
				+ "-epsilon '" + Double.toString(mtpProperties.getEpsilon()) + "' "
				+ "-bias '" + Double.toString(mtpProperties.getBias()) + "' "
				+ "-goalThreshold '" + Double.toString(mtpProperties.getGoalThreshold()) + "' "
				+ "-neighborLimit '" + Double.toString(mtpProperties.getNeighborLimit()) + "' "
				+ "-initialCostBias '" + Double.toString(mtpProperties.getMinimumQuality()) + "' "
				+ "-improvementFactor '" + Double.toString(mtpProperties.getQualityImprovement()) + "'";
		
		// create MTP AEC
		return new AlgorithmExecutionConfiguration(
				Specification.PLANNER_MTP_ID, // executable
				this.getWorkspaceResource().getPath(), // directory (irrelevant)
				mtpPcs, // from file resource
				false, // no cluster (irrelevant)
				false, // non-deterministic algorithm
				// terminate explicitly (cut-off)
				smacOptions.scenarioConfig.algoExecOptions.cutoffTime);
	}
	
	@Override
	protected void initialize(Session managedSession) {
		if (SmacManagerMode.EXECUTING == this.getManagerMode()) {
			super.initialize(managedSession);
		} else if (SmacManagerMode.TRAINING == this.getManagerMode()){
			// TODO: initialize training session
			
			//ParameterConfigurationSpace mgpPcs = this.getManagedGridPlannerPcs();
			//ParameterConfigurationSpace mtpPcs = this.getManagedTreePlannerPcs();
			
			/*
			BufferedReader br = new BufferedReader(new InputStreamReader(is));
			ParameterConfigurationSpace pcs = new ParameterConfigurationSpace(br);
			ParameterConfiguration pc = pcs.getRandomParameterConfiguration(new Random());
			for (String parameter : pc.keySet()) {
				System.out.println(parameter + " = " + pc.get(parameter));
			}
			*/
			
			SMACOptions smacOptions = new SMACOptions();
			smacOptions.adaptiveCapping = false; // QUALITY
			smacOptions.alwaysRunInitialConfiguration = true;
			//smacOptions.capAddSlack = 1.0d;
			//smacOptions.capSlack = 1.3d;
			//smacOptions.classicInitModeOpts.initialIncumbentRuns = 1;
			//smacOptions.dciModeOpts.numberOfChallengers = 2; // double capping
			//smacOptions.dciModeOpts.numberOfRunsPerChallenger = 2; // double capping
			//smacOptions.defaultHandler = SharedModelModeDefaultHandling.USE_ALL;
			//smacOptions.deterministicInstanceOrdering = false;
			//smacOptions.doValidation = true;
			//smacOptions.execMode = ExecutionMode.SMAC;
			smacOptions.experimentDir = this.getWorkspaceResource().getPath();
			smacOptions.expFunc = AcquisitionFunctions.EI;
			//smacOptions.help.helpDefaults = null;
			smacOptions.help.helpLevel = OptionLevel.ADVANCED;
			//smacOptions.initialChallengers = new ArrayList<>();
			//smacOptions.initialChallengersIntensificationTime = 2147483647;
			//smacOptions.initialChallengeRuns = 1;
			//smacOptions.initialIncumbent = "DEFAULT"; // from heuristic tuner
			//smacOptions.initializationMode = InitializationMode.CLASSIC;
			//smacOptions.intensificationPercentage = 0.5d;
			//smacOptions.intermediarySaves = true;
			//smacOptions.iterativeCappingBreakOnFirstCompletion = false;
			//smacOptions.iterativeCappingK = 1;
			
			smacOptions.logOptions.consoleLogLevel = LogLevel.INFO;
			smacOptions.logOptions.logLevel = LogLevel.INFO;
			
			// ------------------------ model building ------------------------
			/*
			smacOptions.mbOptions.imputationIterations = 2;
			smacOptions.mbOptions.maskCensoredDataAsKappaMax = false;
			smacOptions.mbOptions.maskCensoredDataAsUncensored = false;
			smacOptions.mbOptions.maskInactiveConditionalParametersAsDefaultValue = true;
			*/
			
			//smacOptions.maxIncumbentRuns = 2000;
			//smacOptions.numberOfChallengers = 10;
			//smacOptions.numberOfRandomConfigsInEI = 10000;
			//smacOptions.numberOfRandomConfigsUsedForLocalSearch = 0;
			//smacOptions.numPCA = 7;
			//smacOptions.optionFile = null;
			//smacOptions.optionFile2 = null;
			
			// ------------------------ random forest -------------------------
			/*
			smacOptions.randomForestOptions.brokenVarianceCalculation = false;
			smacOptions.randomForestOptions.freeMemoryPercentageToSubsample = 0.25d;
			smacOptions.randomForestOptions.fullTreeBootstrap = false;
			smacOptions.randomForestOptions.ignoreConditionality = false;
			smacOptions.randomForestOptions.imputeMean = false;
			*/
			smacOptions.randomForestOptions.logModel = false; // QUALITY
			/*
			smacOptions.randomForestOptions.minVariance = 1E-14d;
			smacOptions.randomForestOptions.numTrees = 10;
			smacOptions.randomForestOptions.penalizeImputedValues = false;
			smacOptions.randomForestOptions.preprocessMarginal = true;
			smacOptions.randomForestOptions.ratioFeatures = 0.83d;
			smacOptions.randomForestOptions.shuffleImputedValues = false;
			smacOptions.randomForestOptions.splitMin = 10;
			smacOptions.randomForestOptions.storeDataInLeaves = false;
			smacOptions.randomForestOptions.subsamplePercentage = 0.9d;
			smacOptions.randomForestOptions.subsampleValuesWhenLowMemory = false;
			*/
			
			// ------------------------ run groups ----------------------------
			// TODO: run groups for flight phases
			/*
			smacOptions.runGroupOptions.replacementChar = null;
			smacOptions.runGroupOptions.runGroupName = null;
			smacOptions.runGroupOptions.runGroupExit = false;
			*/
			
			//smacOptions.saveRunsEveryIteration = false;
			
			// ------------------------ scenarios -----------------------------
			smacOptions.scenarioConfig._runObj = RunObjective.QUALITY;
			
			smacOptions.scenarioConfig.algoExecOptions.algoExec = Specification.PLANNER_MGP_ID;
			//smacOptions.scenarioConfig.algoExecOptions.algoExecDir = this.getClass().getResource(MGP_PCS_RESOURCE).getPath().;
			
			//smacOptions.scenarioConfig.algoExecOptions.cutoffLength = Double.POSITIVE_INFINITY;
			smacOptions.scenarioConfig.algoExecOptions.cutoffTime = 5d; // seconds
			/*
			smacOptions.scenarioConfig.algoExecOptions.deterministic = true;
			smacOptions.scenarioConfig.algoExecOptions.paramFileDelegate.continuousNeighbours = 4;
			smacOptions.scenarioConfig.algoExecOptions.paramFileDelegate.paramFile = SmacAutonomicManager.MGP_PCS_RESOURCE;
			smacOptions.scenarioConfig.algoExecOptions.paramFileDelegate.searchSubspace = null;
			smacOptions.scenarioConfig.algoExecOptions.paramFileDelegate.searchSubspaceFile = null;
			*/
			
			// ------------------------ algorithm evaluator -------------------
			/*
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.abortOnCrash = true;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.abortOnFirstRunCrash = true;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.boundRuns = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.cacheDebug = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.cacheRuns = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.callObserverBeforeCompletion = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.checkResultOrderConsistent = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.checkRunConfigsUnique = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.checkRunConfigsUniqueException = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.checkSATConsistency = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.checkSATConsistencyException = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.exitOnFailure = true;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.filecache = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.fileCacheCrashOnMiss = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.fileCacheOutput = null;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.fileCacheSource = null;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.fileToWatch = null;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.filterZeroCutoffRuns = true;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.killCaptimeExceedingRun = true;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.killCaptimeExceedingRunFactor = 0d;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.leakMemory = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.leakMemoryAmount = 0;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.logRequestResponses = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.logRequestResponsesRCOnly = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.maxConcurrentAlgoExecs = 0;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.observeWalltimeDelay = 0d;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.observeWalltimeIfNoRuntime = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.observeWalltimeScale = 0d;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.prePostOptions.directory = null;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.prePostOptions.exceptionOnError = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.prePostOptions.logOutput = true;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.prePostOptions.postCommand = null;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.prePostOptions.preCommand = null;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.reportStrictlyIncreasingRuntimes = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.retryCount = 0;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.runHashCodeFile = null;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.skipOutstandingEvaluationsTAE = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.synchronousObserver = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.taeDefaults = null;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.taeStopProcessingOnShutdown = true;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.targetAlgorithmEvaluator = null;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.tForkOptions.forkToTAE = null;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.tForkOptions.fPolicyOptions.duplicateOnSlaveQuickTimeout = 0;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.tForkOptions.fPolicyOptions.fPolicy = null;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.trackRunsScheduled = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.trackRunsScheduledResolution = 0d;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.transformCrashedQuality = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.transformCrashedQualityValue = 0d;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.ttaedo.other_quality_transform = null;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.ttaedo.other_runtime_transform = null;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.ttaedo.SAT_quality_transform = null;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.ttaedo.SAT_runtime_transform = null;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.ttaedo.TIMEOUT_quality_transform = null;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.ttaedo.TIMEOUT_runtime_transform = null;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.ttaedo.transform = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.ttaedo.transformValidValuesOnly = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.ttaedo.UNSAT_quality_transform = null;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.ttaedo.UNSAT_runtime_transform = null;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.uncleanShutdownCheck = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.useDynamicCappingExclusively = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.verifySAT = false;
			smacOptions.scenarioConfig.algoExecOptions.taeOpts.warnIfNoResponseFromTAE = 0;
			*/
			
			// ------------------------ problem instances ---------------------
			/*
			smacOptions.scenarioConfig.instanceOptions.checkInstanceFilesExist = false;
			smacOptions.scenarioConfig.instanceOptions.ignoreFeatures = false;
			smacOptions.scenarioConfig.instanceOptions.instanceFeatureFile = null;
			smacOptions.scenarioConfig.instanceOptions.instanceFile = null;
			smacOptions.scenarioConfig.instanceOptions.instanceSuffix = "csv";
			smacOptions.scenarioConfig.instanceOptions.testInstanceFile = null;
			smacOptions.scenarioConfig.instanceOptions.testInstanceSuffix = "csv";
			smacOptions.scenarioConfig.instanceOptions.useInstances = true;
			
			smacOptions.scenarioConfig.interInstanceObj = OverallObjective.MEAN;
			smacOptions.scenarioConfig.intraInstanceObj = OverallObjective.MEAN;
			smacOptions.scenarioConfig.invalidScenarioReason = null;
			*/
			
			// ------------------------ limitations ---------------------------
			/*
			smacOptions.scenarioConfig.limitOptions.challengeIncumbentAttempts = 5;
			smacOptions.scenarioConfig.limitOptions.countSMACTimeAsTunerTime = false;
			smacOptions.scenarioConfig.limitOptions.fileToWatch = null;
			*/
			//smacOptions.scenarioConfig.limitOptions.numIterations = 500;
			//smacOptions.scenarioConfig.limitOptions.runtimeLimit = 2147483647;
			smacOptions.scenarioConfig.limitOptions.totalNumRunsLimit = this.getTrainingRuns();
			//smacOptions.scenarioConfig.limitOptions.tunerTimeout = Integer.MAX_VALUE;
			smacOptions.scenarioConfig.outputDirectory = this.getWorkspaceResource().getPath();
			//smacOptions.scenarioConfig.scenarioFile = null;
			
			// ------------------------ seed options --------------------------
			/*
			smacOptions.seedOptions.initialSeedMap = null;
			smacOptions.seedOptions.numRun = 0;
			smacOptions.seedOptions.seedOffset = 0;
			*/
			
			// ------------------------ shared models -------------------------
			/*
			smacOptions.sharedModeModeAssymetricMode = false;
			smacOptions.shareModelMode = false;
			smacOptions.shareModeModeTAE = true;
			smacOptions.shareRunDataFrequency = 300;
			*/
			
			// ------------------------ state handling ------------------------
			//smacOptions.stateOpts.cleanOldStatesOnSuccess = true;
			//smacOptions.stateOpts.restoreIteration = 0;
			//smacOptions.stateOpts.restoreScenario = null;
			smacOptions.stateOpts.restoreStateFrom = this.getWorkspaceResource().getPath();
			//smacOptions.stateOpts.saveContextWithState = true;
			//smacOptions.stateOpts.statedeSerializer = StateSerializers.LEGACY
			//smacOptions.stateOpts.statedeSerializer = StateSerializers.LEGACY;
			
			//smacOptions.stateQuickSaves = true;
			//smacOptions.validationCores = 0;
			//smacOptions.validationSeed = 0;
			
			// ------------------------ warm starting -------------------------
			//smacOptions.warmStartOptions.restoreIteration = 32;
			//smacOptions.warmStartOptions.warmStartStateFrom = this.getWorkspaceResource().getPath();
			
			/*
			AlgorithmExecutionConfiguration mgpAec = new AlgorithmExecutionConfiguration(
					Specification.PLANNER_MGP_ID, // smacOptions.getAlgorithmExecutionConfig().getAlgorithmExecutable(), // executable
					this.getWorkspaceResource().getPath(), //smacOptions.getAlgorithmExecutionConfig().getAlgorithmExecutionDirectory(), // irrelevant (directory)
					mgpPcs, // from file resource
					false, // irrelevant (cluster)
					true, // deterministic
					smacOptions.scenarioConfig.algoExecOptions.cutoffTime); // terminate explicitly (cut-off), or criticality
			*/
			
			Features defaultFeatures = new Features();
			defaultFeatures.extractFeatures(managedSession.getActiveScenario());
			//AlgorithmExecutionConfiguration mgpAec = this.getManagedGridPlannerAec(smacOptions, defaultFeatures);
			//Logging.logger().info(mgpAec.toString());
			AlgorithmExecutionConfiguration mtpAec = this.getManagedTreePlannerAec(smacOptions, defaultFeatures);
			Logging.logger().info(mtpAec.toString());
			
			ArrayList<ProblemInstance> problemInstances = new ArrayList<>();
			int instanceId = 1;
			for (Scenario trainingScenario : managedSession.getScenarios()) {
				Features trainingFeatures = new Features();
				trainingFeatures.extractFeatures(trainingScenario);
				ProblemInstance problemInstance = new ProblemInstance(
						trainingScenario.getId(),
						instanceId++,
						trainingFeatures);
				problemInstances.add(problemInstance);
			}
			
			// ProblemInstanceSeedPair is build by SMAC and combines instance and features with seed
			// to be used in the AlgorithmRunConfiguration
			
			// TODO: check TargetAlgorithmEvaluatorBuilder
			SmacPlannerEvaluator plannerEvaluator = new SmacPlannerEvaluator(managedSession);
			
			SeedableRandomPool pool = smacOptions.seedOptions.getSeedableRandomPool();
			/*
			TrainTestInstances tti;
			InstanceListWithSeeds trainingInstances = null; // create instances from features
			InstanceListWithSeeds testingInstances = null;
			try {
				tti = smacOptions.getTrainingAndTestProblemInstances(
						pool, new SeedableRandomPool(smacOptions.validationSeed
								+ smacOptions.seedOptions.seedOffset,
								pool.getInitialSeeds()));
				trainingInstances = tti.getTrainingInstances();
				testingInstances = tti.getTestInstances();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			*/
			InstanceSeedGenerator isg = new RandomInstanceSeedGenerator(problemInstances, problemInstances.hashCode());
			InstanceListWithSeeds trainingInstances = new InstanceListWithSeeds(isg, problemInstances);
			
			EventManager eventManager = new EventManager();
			// TODO: register event handlers
			
			Logging.logger().info(smacOptions.initialIncumbent);
			// TODO: possibly use builder and simply replace plannerEvaluator
			SMACBuilder builder = new SMACBuilder();
			this.smac = (SequentialModelBasedAlgorithmConfiguration) builder.getAutomaticConfigurator(
					mtpAec,
					trainingInstances,
					smacOptions,
					// TODO: not really relevant since the evaluator is provided below
					smacOptions.scenarioConfig.algoExecOptions.taeOpts.getAvailableTargetAlgorithmEvaluators(),
					smacOptions.scenarioConfig.outputDirectory,
					pool,
					new OutstandingEvaluationsTargetAlgorithmEvaluatorDecorator(plannerEvaluator),
					null); // run history created internally
			Logging.logger().info(this.smac.getInitialIncumbent().getFormattedParameterString(ParameterStringFormat.NODB_SYNTAX));
			
			/*
			Specification<Planner> mgpSpec = new Specification<Planner>(Specification.PLANNER_MGP_ID, new OADStarProperties());
			HeuristicPlannerTuning mgpTuning = new HeuristicPlannerTuning(mgpSpec, defaultFeatures);
			OADStarProperties mgpProperties = (OADStarProperties) mgpTuning.tune().get(0);
			this.smac.getInitialIncumbent().put("minimumQuality", Double.toString(mgpProperties.getMinimumQuality()));
			this.smac.getInitialIncumbent().put("qualityImprovement", Double.toString(mgpProperties.getQualityImprovement()));
			for (String parameter : this.smac.getInitialIncumbent().keySet()) {
				Logging.logger().info(parameter + " = " + this.smac.getInitialIncumbent().get(parameter));
			}
			*/
			
			/*
			SequentialModelBasedPlannerConfiguration smpc = new
					SequentialModelBasedPlannerConfiguration(
							smacOptions,
							algorithmExecutionConfiguration,
							problemInstances,
							plannerEvaluator,
							AcquisitionFunctions.EI,
							smacOptions.getRestoreStateFactory("/var/tmp/smac/state"),
							pcs,
							trainingILWS.getSeedGen(),
							pc,
							new ArrayList<>(), // initial challengers (from heuristic planner knowledge base)
							eventManager,
							null,
							null,
							null,
							null,
							null,
							null,
							null);
			*/
		}
	}
	
	@Override
	protected void run(Session managedSession) {
		if (SmacManagerMode.EXECUTING == this.getManagerMode()) {
			super.run(managedSession);
		} else if (SmacManagerMode.TRAINING == this.getManagerMode()) {
			// TODO: run training session
			// TODO: create and run SMAC on available problem instances (scenarios)
			this.smac.run();
			ParameterConfiguration incumbent = this.smac.getIncumbent();
			for (String parameter : incumbent.getActiveParameters()) {
				Logging.logger().info(parameter + " = " + incumbent.get(parameter));
			}
			Logging.logger().info("performance = " + this.smac.getEmpericalPerformance(incumbent));
		}
	}
	
	@Override
	protected void cleanup(Session managedSession) {
		if (SmacManagerMode.EXECUTING == this.getManagerMode()) {
			super.cleanup(managedSession);
		} else if (SmacManagerMode.TRAINING == this.getManagerMode()) {
			// TODO: cleanup training session
			this.smac = null;
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
			this.setWorkspaceResource(URI.create(properties.getWorkspaceResource()));
		}
		
		return updated;
	}
	
}
