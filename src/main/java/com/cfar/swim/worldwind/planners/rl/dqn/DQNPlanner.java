package com.cfar.swim.worldwind.planners.rl.dqn;

import java.util.ArrayList;


import java.util.Arrays;
import java.awt.Color;
import java.time.ZonedDateTime;
import java.time.temporal.ChronoUnit;
import java.time.temporal.TemporalUnit;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.Random;
import java.util.Set;
import java.util.TreeSet;
import java.util.Map;
import java.util.TreeMap;
import java.util.stream.Collectors;
import java.nio.Buffer;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.environments.PlanningContinuum;
import com.cfar.swim.worldwind.environments.PlanningGrid;
import com.cfar.swim.worldwind.environments.PlanningRoadmap;
import com.cfar.swim.worldwind.geom.precision.PrecisionPosition;
import com.cfar.swim.worldwind.planners.AbstractPlanner;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.planners.rl.Plot;
import com.cfar.swim.worldwind.planners.rl.RLWaypoint;
import com.cfar.swim.worldwind.planners.rl.State;
import com.cfar.swim.worldwind.planners.rl.dqn.Memory;
import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.render.Obstacle;
//import com.cfar.swim.worldwind.tests.Plot;
import com.cfar.swim.worldwind.util.Identifiable;
import com.cfar.swim.worldwind.render.airspaces.*;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;
import gov.nasa.worldwind.geom.LatLon;
import ai.djl.Model;
import ai.djl.engine.Engine;
import ai.djl.inference.Predictor;
import ai.djl.ndarray.*;
import ai.djl.nn.Parameter;
import ai.djl.training.GradientCollector;
import ai.djl.training.loss.L2Loss;
import ai.djl.training.loss.Loss;
import ai.djl.training.optimizer.Optimizer;
import ai.djl.training.tracker.Tracker;
import ai.djl.translate.TranslateException;
import ai.djl.util.Pair;
import ai.djl.translate.NoopTranslator;

/**
* Realizes a deep reinforcement learning planner, using a Deep Q-Network, that plans a trajectory of an aircraft
* in an environment considering a local cost and risk policy.
* 
* @author Rafaela Seguro
*
*/

public class DQNPlanner extends AbstractPlanner {
	
	/** the minimum exploration rate used during training */
	private static final float MIN_EXPLORE_RATE = 0.1f; 
	
//	/** the rate at which the exploration rate decays over time */
//	private static final float DECAY_EXPLORE_RATE = 0.9f;
	
	/** the initial value of epsilon */
	private static final float INITIAL_EPSILON = 1.0f;
	
	/** number of episodes for the global training */
	private static final int NUM_GLOBAL_EPS = 1000;
	
	/** maximum number of steps per episode */
	private static final int MAX_STEPS = 100;
	
	/** in how many steps to divide from start to goal */
	private static final int STEP_DIVISION = 10;
	
	/** random number */
	private final Random rand = new Random();
	
	/** replay memory */
	private final Memory memory = new Memory(4096);
	
	/** dimension of a state ID (input of the neural network) */
	private int dimOfState;
	
	/** the number of available actions in each state */
	private final int numOfActions = 7;
	
	/** the number of hidden units (neurons) in the neural network */
	private final int[] hiddenSize = {64, 128, 256, 128, 64};
	
	/** learning rate used by the optimizer during training */
	private final float learningRate = 0.001f;
	
	/** the size of the mini-batch of transitions used for training */
	protected final int batchSize = 32;
	
	/** the number of iterations between each train of the policy network */
	protected final int trainNetInterval = 1;
	
	/** the number of iterations between each sync of the target and policy networks */
	protected final int syncNetInterval = 200;
	
	/** gamma factor for the Bellman equation */
	protected final float gamma = 0.9f;
	
	/** the optimizer used for updating the network parameters during training */
	private Optimizer optimizer;
	
	/** the policy network, which predicts the Q-values */
	private Model policyNet;
	
	/** the target network, used to stabilize training in DQN */
	private Model targetNet;
	
	/** used for managing NDArrays within the class */
	protected NDManager manager;
	
	/** predictor for the policy network */
	protected Predictor<NDList, NDList> policyPredictor;
	
	/** predictor for the target network */
	protected Predictor<NDList, NDList> targetPredictor;
	
	/** keeps track of the number of iterations the agent has experienced */
	private int iteration = 0;
	
	/** the current epsilon value to implement decaying e-greedy */
	private float epsilon = INITIAL_EPSILON; 
	
	/** the loss function */
	private final Loss lossFunc = Loss.l2Loss();
	
	/** the planner's start state */
	private State start = null;
	
	/** the planner's goal position */
	private PrecisionPosition goalPosition = null;
	
	/** the current state */
	private State state = null;
	
	/** the next state */
	private State nextState = null;
	
	/** the index of the chosen action */
	private int action = 0;
	
	/** the received reward */
	private double reward = 0.0;
	
	/** indicates if it reached goal or not, true if it has */
	private boolean done = false;
	
	/** indicates if it reached goal or not, true if it has */
	private boolean failure = false;
	
	/** saves the most recent waypoint added to the trajectory */
	private RLWaypoint newestWaypoint = null;
	
	/** stores the plan's ETD */
	private ZonedDateTime etd = ZonedDateTime.now();
	
	/** stores the step size of the planner */
	private double stepSize = 0.0;
	
	/** set of random obstacles for training */
	private HashSet<Obstacle> trainingObstacles = new HashSet<>();
	
	/** saves the actual cost policy for when training ends*/
	private CostPolicy actualCostPolicy = null;
	
	/** saves the actual risk policy for when training ends*/
	private RiskPolicy actualRiskPolicy = null;
	
	/** Stores episode results */
	public boolean[] episodeResults = new boolean[NUM_GLOBAL_EPS+1];
	
	
	/** Constructs a planner trained by a Deep Q-Network for a specified aircraft and
	 * environment using default local cost and risk policies.
	 * 
	 * @param the aircraft
	 * @param the environment
	 */
	public DQNPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		this.dimOfState = State.ID_SIZE;
//		this.listOfActions = Helper.listOfActions();
//		this.numOfActions = listOfActions.size();
		this.actualCostPolicy = this.getCostPolicy();
		this.actualRiskPolicy = this.getRiskPolicy();
		
		resetAgent();
		trainGlobal();
		syncNetworks();
	}
	
	/**
	 * Gets the identifier of this DQN planner.
	 * 
	 * @return the identifier of this DQN planner
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return Specification.PLANNER_DQN_ID;
	}
	
	/**
	 * Gets the planning continuum of this DQN planner.
	 * 
	 * @return the planning continuum of this DQN planner
	 * 
	 * @see AbstractPlanner#getEnvironment()
	 */
	public PlanningContinuum getPlanningContinuum() {
		return (PlanningContinuum) super.getEnvironment();
	}
	
	/**
	 * Gets the size of a state's ID, which is the size of the network input.
	 * 
	 * @return the size of a state's ID
	 */
	protected int getDimOfState() {
		return this.dimOfState;
	}
	
	/**
	 * Gets the number of available actions.
	 * 
	 * @return the number of available actions
	 */
	protected int getNumOfActions() {
		return this.numOfActions;
	}
	
	/**
	 * Gets the start state of this  planner.
	 * 
	 * @return the start state of this planner
	 */
	protected State getStart() {
		return this.start;
	}
	
	/**
	 * Sets the start state of this  planner.
	 * 
	 * @param start the start state of this planner
	 */
	protected void setStart(State start) {
		this.start = start;
	}

	/**
	 * Gets the goal position of this planner.
	 * 
	 * @return the goal position of this planner
	 */
	protected PrecisionPosition getGoalPosition() {
		return this.goalPosition;
	}
	
	/**
	 * Sets the goal position of this planner.
	 * 
	 * @param goal the goal position of this planner
	 */
	protected void setGoalPosition(PrecisionPosition goal) {
		this.goalPosition = goal;
	}
	
	/**
	 * Gets the current state.
	 * 
	 * @return the state 
	 */
	protected State getState() {
		return this.state;
	}
	
	/**
	 * Sets the current state.
	 * 
	 * @param state the current state
	 */
	protected void setState(State state) {
		this.state = state;
	}
	
	/**
	 * Gets the next state.
	 * 
	 * @return the next state
	 */
	protected State getNextState() {
		return this.nextState;
	}

	/**
	 * Sets the next state of this planner.
	 * 
	 * @param nextState the next state of this planner
	 */
	protected void setNextState(State nextState) {
		this.nextState = nextState;
	}
	
	/**
	 * Gets the chosen action.
	 * 
	 * @return the chosen action
	 */
	protected int getAction() {
		return this.action;
	}
	
	/**
	 * Sets the chosen action.
	 * 
	 * @param action the chosen action
	 */
	protected void setAction(int action) {
		this.action = action;
	}
	
	/**
	 * Gets the reward.
	 * 
	 * @return the reward
	 */
	protected double getReward() {
		return this.reward;
	}
	
	/**
	 * Sets the reward.
	 * 
	 * @param the reward
	 */
	protected void setReward(double reward) {
		this.reward = reward;
	}
	
	/**
	 * Checks if the planner has reached the goal
	 * 
	 * @return true if it reached the goal, false otherwise
	 */
	protected boolean isDone() {
		return this.done;
	}
	
	/**
	 * Sets the boolean that indicates if the planner has reached the goal.
	 * 
	 * @param true if it reached the goal, false otherwise
	 */
	protected void setDone(boolean done) {
		this.done = done;
	}
	
	/**
	 * Checks if the planner has failed
	 * 
	 * @return true if it failed, false otherwise
	 */
	protected boolean failed() {
		return this.failure;
	}
	
	/**
	 * Sets the boolean that indicates if the planner has failed.
	 * 
	 * @param true if it failed, false otherwise
	 */
	protected void setFailure(boolean failure) {
		this.failure = failure;
	}
	
	/**
	 * Gets the newest RL waypoint added to the trajectory.
	 * 
	 * @return the newest RL waypoint added to the trajectory
	 */
	protected RLWaypoint getNewestWaypoint() {
		return this.newestWaypoint;
	}
	
	/**
	 * Sets the newest RL waypoint added to the trajectory.
	 * 
	 * @param newestWaypoint the newest RL waypoint added to the trajectory
	 */
	protected void setNewestWaypoint(RLWaypoint newestWaypoint) {
		this.newestWaypoint = newestWaypoint;
	}
	
	/**
	 * Gets the plan's ETD.
	 * 
	 * @return the plan's ETD
	 */
	protected ZonedDateTime getEtd() {
		return this.etd;
	}
	
	/**
	 * Sets the plan's ETD.
	 * 
	 * @param etd this plan's ETD
	 */
	protected void setEtd(ZonedDateTime etd) {
		this.etd = etd;
	}
	
	/**
	 * Gets the planner's step size.
	 * 
	 * @return the planner's step size
	 */
	protected double getStepSize() {
		return this.stepSize;
	}
	
	/**
	 * Sets the planner's step size.
	 * 
	 * @param stepSize the planner's step size
	 */
	protected void setStepSize(double stepSize) {
		this.stepSize = stepSize;
	}

	/**
	 * Gets the planner's step size.
	 * 
	 * @return the planner's step size
	 */
	protected HashSet<Obstacle> getTrainingObstacles() {
		return this.trainingObstacles;
	}
	
	/**
	 * Sets the planner's step size.
	 * 
	 * @param stepSize the planner's step size
	 */
	protected void setTrainingObstacles(HashSet<Obstacle> obs) {
		this.trainingObstacles = obs;
	}

	
	/** Resets the DQN agent before training
	 */
	protected void resetAgent() {
		optimizer = Optimizer.adam().optLearningRateTracker(Tracker.fixed(learningRate)).build();
		
		if (manager != null) {
			manager.close();
		}
		manager = NDManager.newBaseManager();
		
		policyNet = NetworkModel.newModel(manager, dimOfState, hiddenSize, numOfActions);
		//Sets require gradient to true for the policy network's parameters
		for (Pair<String, Parameter> params : policyNet.getBlock().getParameters()) {
			params.getValue().getArray().setRequiresGradient(true);
		}
		targetNet = NetworkModel.newModel(manager, dimOfState, hiddenSize, numOfActions);
		
		policyPredictor = policyNet.newPredictor(new NoopTranslator());
		syncNetworks();
	}

	/**
	 * Runs the training of the Deep Q-Network for random environment configurations
	 */
	protected void trainGlobal() {
		
		double decay = 0;
		int episode = -1;
		int step;
		double agentPerformance = 0;
		double successfulEpisodes = 0;
		//|| agentPerformance < 0.9
		
		// For each episode
		while (episode < NUM_GLOBAL_EPS ) {
			// Reset environment 
			episode++;
			step = 0;
			
			resetEnvironment();
		
			// For each time step
			while (!this.isDone() && !this.failed()) {
				// Saves state in memory; Updates the network; Selects the next action and saves it in memory
				this.react();
				System.out.println("STATE " + (step + 1));
				System.out.println("Cost Policy: " + state.getId()[0] + "; Step size: " + state.getId()[1]);
				System.out.println("Distance to axes: " + state.getId()[2] + "; " + state.getId()[3] + "; " + state.getId()[4] 
						+ "; " + state.getId()[5] + "; " + state.getId()[6] + "; " + state.getId()[7]);
				System.out.println("Relative to goal: " + state.getId()[8] + "; " + state.getId()[9] + "; " + state.getId()[10] + "; " + state.getId()[11]);
				System.out.println("Obstacle 1: " + state.getId()[12] + "; " + state.getId()[13] + "; " + state.getId()[14] + "; " + state.getId()[15]);
				System.out.println("Obstacle 2: " + state.getId()[16] + "; " + state.getId()[17] + "; " + state.getId()[18] + "; " + state.getId()[19]);
				System.out.println("Obstacle 3: " + state.getId()[20] + "; " + state.getId()[21] + "; " + state.getId()[22] + "; " + state.getId()[23]);
				System.out.println("Action: " + action);
				// Execute action and get next state and reward; Checks if the goal has been reached
				this.step();
				// Reaching maximum number of steps counts as failure
				if (step >= MAX_STEPS)
					this.setFailure(true);
				// Stores the reward and the "done" boolean in memory
				memory.setReward(this.getReward(), this.isDone(), this.failed());
				// Sets the state as the next state
				this.setState(this.getNextState());
				step++;
			}
			// Updates epsilon for next episode
			decay = Math.exp(-episode * 1.0 / (NUM_GLOBAL_EPS));
			epsilon = (float) (MIN_EXPLORE_RATE + (INITIAL_EPSILON - MIN_EXPLORE_RATE) * decay);
			
//			if(this.isDone())
//				successfulEpisodes ++;
//			agentPerformance = successfulEpisodes / episode;
			
			episodeResults[episode] = this.isDone();
			successfulEpisodes = 0;
			if (episode > 1000) {
				for (int i=0; i<1000; i++) {
					if (episodeResults[episode - 1000 + i])
						successfulEpisodes++;
				}
				agentPerformance = successfulEpisodes / 1000;
			} else {
				for (int i=0; i<episode; i++) {
					if (episodeResults[i])
						successfulEpisodes++;
				}
				agentPerformance = successfulEpisodes / episode;
			}
			
			if (this.isDone()) {
				System.out.println("Episode " + episode + " -----------------------> GOAL");
			} else if (this.failed()) {
				if (step >= MAX_STEPS ) {
					System.out.println("Episode " + episode + " did too many steps");
				} else if (this.getReward()==-100){
					System.out.println("Episode " + episode + " hit an OBSTACLE");
				} else if (this.getReward()==-50){
					System.out.println("Episode " + episode + " left environment");
				}
			}
			System.out.println("Performance " + agentPerformance);
		}
	}
	
	
//	/**
//	 * Runs the training of the Deep Q-Network for a specific start and goal
//	 */
//	protected void trainLocal(int numEpisodes, Position origin, Position destination) {
//		
//		double decay = 0;
//		int episode = 0;
//		int step;
//		double score;
//		
//		initialize(origin, destination, etd);
//		
//		// For each episode
//		while (episode < numEpisodes) {
//			
//			episode++;
//			step = 1;
//			score = 0;
//			// Reset environment 
//			this.setDone(false);
//			this.setFailure(false);
//			this.setState(this.getStart());
//		
//			// For each time step, until it reaches goal or failure
//			while (!this.isDone() && !this.failed()) {
//				// Saves state in memory; Updates the network; Selects the next action and saves it in memory
//				this.react();
//				// Execute action and get next state and reward; Checks if the goal has been reached
//				this.step();
//				// Reaching maximum number of steps counts as failure
//				if (step >= MAX_STEPS)
//					this.setFailure(true);
//				// Stores the reward and the "done" boolean in memory
//				memory.setReward(this.getReward(), this.isDone(), this.failed());
//				score += this.getReward();
//				// Sets the state as the next state
//				this.setState(this.getNextState());
//				step++;
//			}
//			// Update epsilon for next episode
//			decay = Math.exp(-episode * 1.0 / (NUM_GLOBAL_EPS-100));
//			epsilon = (float) (INITIAL_EPSILON - (INITIAL_EPSILON - MIN_EXPLORE_RATE) * decay);
//			
//			if (this.isDone()) {
//				System.out.println("Episode " + episode + " had score " + score + " and reached GOAL after " + step + " steps");
//			} else if (this.failed()) {
//				if (step >= MAX_STEPS ) {
//					System.out.println("Episode " + episode + " had score " + score + "  but did too many steps");
//				} else if (this.getReward()==-100){
//					System.out.println("Episode " + episode + " had score " + score + "  but hit an obstacle");
//				} else {
//					System.out.println("Episode " + episode + " had score " + score + "  but left environment");
//				}
//			} 
//		}
//	}

	/** 
	 * Resets the environment with a new random start and goal
	 */
	protected void resetEnvironment() {
		
		// Starts by sampling random positions for the start and goal, following a uniform distribution
		PrecisionPosition startPosition = new PrecisionPosition(this.getPlanningContinuum().sampleRandomUniformPosition());
		PrecisionPosition goalPosition = startPosition;
		// Makes sure the start and goal are different
		while (goalPosition == startPosition)
			goalPosition = new PrecisionPosition(this.getPlanningContinuum().sampleRandomUniformPosition());
		
		// Sets the step size
		double distance = this.getPlanningContinuum().getGlobe().computePointFromPosition(goalPosition).
				subtract3(this.getPlanningContinuum().getGlobe().computePointFromPosition(startPosition)).getLength3();
		this.setStepSize(distance / STEP_DIVISION);
		
		// Adds obstacles to the environment
		this.createRandomObstacles(startPosition, goalPosition);
		
		// Creates the start state
		TreeSet<DQNObstacle> obstacles = Helper.getCloseObstacles(startPosition, this.getPlanningContinuum(), this.getTrainingObstacles(), this.getRiskPolicy());
		this.setStart(new State(startPosition, goalPosition, this.getPlanningContinuum(), obstacles, this.getCostPolicy(), 
				this.getEtd(), this.getStepSize() , null));
		
		// Sets the goal position
		this.setGoalPosition(goalPosition);
		
		// Sets the state as the start
		this.setState(this.getStart());
		
		// Sets the "done" and "failure" booleans to false
		this.setDone(false);
		this.setFailure(false);
		
	}
	
	
	/** 
	 * Creates random obstacles and adds them to the planning continuum. Also sets a random cost and risk policy
	 */
	protected void createRandomObstacles(Position start, Position goal) {
		
		this.getTrainingObstacles().clear();
		
		// TODO: maybe add dynamic obstacles but for now they are static
		ZonedDateTime t1 = this.getEtd().minusYears(1);
		ZonedDateTime t2 = this.getEtd().plusYears(1);
		
		int numberOfObstacles = rand.nextInt(State.CONSIDERED_OBSTACLES);
		
		// Creates random obstacles and adds them to set
		for (int i=0; i<numberOfObstacles; i++) {
			
			Position center = this.getPlanningContinuum().sampleRandomUniformPosition();
			double radius = rand.nextDouble() * (this.getPlanningContinuum().getRadius() / 5);
			// TODO: add other types of obstacles, for now it is only sphere since it is the simplest to create
			ObstacleSphere obstacle = new ObstacleSphere(center, radius);
			
			double cost = rand.nextDouble() * 100;
			obstacle.setCostInterval(new CostInterval("ci", t1, t2, cost));
			
			this.getTrainingObstacles().add(obstacle);
		}
		
		// 1/3 of the times, there is an obstacle right between start and goal, to make sure it learns to go around
//		if (true) {
//			
//			Globe globe = this.getEnvironment().getGlobe();
//			Vec4 startPoint = this.getPlanningContinuum().transformModelToBoxOrigin(globe.computePointFromPosition(start));
//			Vec4 goalPoint = this.getPlanningContinuum().transformModelToBoxOrigin(globe.computePointFromPosition(goal));
//			Vec4 halfStartToGoal = goalPoint.subtract3(startPoint).divide3(2d);
//			Vec4 e = startPoint.add3(halfStartToGoal);
//			Position center = globe.computePositionFromPoint(this.getPlanningContinuum().transformBoxOriginToModel(startPoint.add3(halfStartToGoal)));
//			//Vec4 obstaclePoint = t.transformModelToBoxOrigin(env.getGlobe().computePointFromPosition(obstacle.getCenter()));
//
//			double radius = rand.nextDouble() * (this.getPlanningContinuum().getRadius() / 5);
//			double height = rand.nextDouble() * (this.getPlanningContinuum().getTLength());
//			ObstacleCylinder obstacle = new ObstacleCylinder(center, 0, height, radius);
//			
//			double cost = rand.nextDouble() * 100;
//			obstacle.setCostInterval(new CostInterval("ci", t1, t2, cost));
//			
//			this.getTrainingObstacles().add(obstacle);
//			
//		}
		
		// Sets random cost policy
		int costPolicy = rand.nextInt(2);
		this.setCostPolicy(CostPolicy.values()[costPolicy]);
		
		// Sets random risk policy
		int riskPolicy = rand.nextInt(4);
		//this.setRiskPolicy(RiskPolicy.values()[riskPolicy]);
		this.setRiskPolicy(RiskPolicy.AVOIDANCE);
	}


	/** 
	 * Reacts to the current state, updating the memory and choosing the next action
	 */
	protected void react() {
		
		iteration++;
		try (NDManager submanager = manager.newSubManager()) {
			
			memory.setState(this.getState().getId());
			
			// Trains every n iterations only
			if (iteration % trainNetInterval == 0  && memory.getSize()>batchSize)
				updateModel(submanager);
			
			// Syncs networks and updates every n iterations
			if (iteration % syncNetInterval == 0) {
				syncNetworks();
			}
			
			this.setAction(chooseAction(submanager, this.getState().getId())) ;
			
			memory.setAction(this.getAction());
			
		} catch (TranslateException e) {
			throw new IllegalStateException(e);
		}
	}

	/** 
	 * Chooses the action to perform using the epsilon greedy policy
	 * 
	 * @param manager the memory space manager
	 * @param state the current state
	 * 
	 * @return the chosen action
	 */
	protected int chooseAction(NDManager manager, float[] state) throws TranslateException {
		
		// Gets the predicted Q-values from the main network (Double DQN)
		NDArray qValues = policyPredictor.predict(new NDList(manager.create(state))).singletonOrThrow();
		// Chooses the action using the epsilon greedy policy and the predicted Q-values
		return ActionSampler.epsilonGreedy(qValues, rand, Math.max(MIN_EXPLORE_RATE, epsilon));
	}

	
	/** 
	 * Determines the next state based on the chosen action
	 */
	protected void step() {
		
		int action = this.getAction();
		//double stepSize = this.getStart().getdistan;
		
		// Determines the movement from the current state to the next based on action
		Vec4 movVector = Helper.getNewMoveVector(this.getState().getNormalizedRelativeGoal().getNegative3(), action);
				
//		if (action == 0) {
//			// If the action index is 0, the action corresponds to just flying in the direction of the goal
//			movVector = this.getState().getNormalizedRelativeToGoal().getNegative3().multiply3(this.getStepSize());
//		} else {
//			movVector = listOfActions.get(action).multiply3(this.getStepSize());
//		}
		
		// Computes the next state's position
		Vec4 boxNextStatePoint = this.getPlanningContinuum().transformModelToBoxOrigin(this.getState().getStatePoint().add3(movVector.multiply3(this.getStepSize())));
		Vec4 nextStatePoint = this.getPlanningContinuum().transformBoxOriginToModel(boxNextStatePoint);
		PrecisionPosition nextStatePosition = new PrecisionPosition(this.getEnvironment().getGlobe().computePositionFromPoint(nextStatePoint));
		
		// If the next state is outside the environment, stays in place, and fails
		if (!this.getEnvironment().contains(nextStatePosition)) {
			nextStatePosition = this.getState().getPosition();
			this.setFailure(true);
		}
		
		// Gets the set of surrounding obstacles of the next state
		TreeSet<DQNObstacle> obstacles = Helper.getCloseObstacles(nextStatePosition, this.getPlanningContinuum(), this.getTrainingObstacles(), this.getRiskPolicy());
		
		// Computes the next state's ETO
		Path leg = new Path(this.getState().getPosition(), nextStatePosition);
		ZonedDateTime eto = this.getAircraft().getCapabilities().getEstimatedTime(leg, this.getEnvironment().getGlobe(), this.getState().getEto());
		
		// Creates the next state and adds it to the map
		this.setNextState(new State(nextStatePosition, this.getGoalPosition(), this.getPlanningContinuum(), obstacles, 
				this.getCostPolicy(), eto, this.getStepSize(), movVector));
		
		// Checks if it has reached goal
		if (this.getNextState().getDistanceToGoal() <= this.getStepSize())
			this.setDone(true);
		
		// Calculates reward
		this.calculateReward();
	}

	/**
	 * Calculates the reward based on the next state
	 */
	protected void calculateReward() {
		
		double reward = 0;
		
		// If the goal has been reached
		if (this.isDone()) {
			this.setReward(100);
			return;
		}
		
		// If it failed for leaving the environment
		if (this.failed()){
			this.setReward(-50);
			return;
		}
		
		// If the cost is exceeded
		double cost = this.getEnvironment().getLegCost(this.getState().getPosition(), this.getNextState().getPosition(), this.getState().getEto(), 
				this.getNextState().getEto(), getCostPolicy(), getRiskPolicy());
		if (Double.POSITIVE_INFINITY == cost) {
			this.setFailure(true);
			this.setReward(-100);
			return;
		}
		
		// If it gets too close to obstacle
		Iterator<DQNObstacle> itr = this.getNextState().getObstacles().iterator();
		DQNObstacle current = null;
		while(itr.hasNext()) {
			current =itr.next();
			if (current.getDistanceToState() <= this.getState().getStepSize()) {
				reward += -40;
				break;
			}
		}
		
		// Otherwise: step penalty +  distance to goal penalty/reward 
		reward += -5;
		reward += this.getState().getDistanceToGoal() - this.getNextState().getDistanceToGoal();
		
		this.setReward(reward);;
	
	}


	/** 
	 * Gets a batch of transitions from memory, calculates the loss based on the predicted Q-values and the actual rewards and 
	 * performs the gradient update through backpropagation
	 * 
	 * @param manager the memory space manager
	 */
	protected void updateModel(NDManager manager) throws TranslateException {
		
		try (GradientCollector collector = Engine.getInstance().newGradientCollector()) {
			
			MemoryBatch batch = memory.sampleBatch(batchSize, manager);
			
			// Predicts the policy for the states in the batch
			NDArray policy = policyPredictor.predict(new NDList(batch.getStates())).singletonOrThrow();
			// Gather the predicted Q-values for the selected actions in the batch
			NDArray expectedReturns = Helper.gather(policy, batch.getActions().toIntArray());
			
			// Predicts the target Q-values for next the states in the batch
			NDArray target = targetPredictor.predict(new NDList(batch.getNextStates())).singletonOrThrow().duplicate();
			// Calculates the target Q-values for the current states using the Bellman equation
			NDArray nextReturns = batch.getRewards().add(target.max(new int[] { 1 }).mul(batch.getDones().logicalNot()).mul(gamma));
			
			// Calculates the loss (mean squared error)
			NDArray loss = lossFunc.evaluate(new NDList(expectedReturns), new NDList(nextReturns));
			loss.setRequiresGradient(true);
			
			//Performs the backpropagation and calculates the gradients
			collector.backward(loss);

			// Updates the policy network's parameters
			for (Pair<String, Parameter> params : policyNet.getBlock().getParameters()) {
				NDArray paramsArr = params.getValue().getArray();
				optimizer.update(params.getKey(), paramsArr, paramsArr.getGradient());
			}
		}
	}


	/** 
	 * Copies the weights from the main network to the target network
	 */
	protected void syncNetworks() {
	
		for (Pair<String, Parameter> params : policyNet.getBlock().getParameters()) {
			NDArray targetArray = targetNet.getBlock().getParameters().get(params.getKey()).getArray();
			params.getValue().getArray().copyTo(targetArray);
		}
		targetPredictor = targetNet.newPredictor(new NoopTranslator());
	}
	

	/**
	 * Initializes the DQN planner to plan from an origin to a destination at a
	 * specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 */
	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {
		
		this.setEtd(etd);
		this.setCostPolicy(actualCostPolicy);
		this.setRiskPolicy(actualRiskPolicy);
		
		PrecisionPosition pOrigin = new PrecisionPosition(origin);
		PrecisionPosition pDestination = new PrecisionPosition(destination);
		
		// Clears the trainingObstacles set 
		this.getTrainingObstacles().clear();
		
		// Creates the start waypoint 
		this.setNewestWaypoint(this.createWaypoint(origin));
		this.getNewestWaypoint().setEto(etd);
		this.getNewestWaypoint().setPoi(true);
		
		// Sets the step size
		double distance = this.getPlanningContinuum().getGlobe().computePointFromPosition(destination).
				subtract3(this.getPlanningContinuum().getGlobe().computePointFromPosition(origin)).getLength3();
		this.setStepSize(distance / STEP_DIVISION);
		
		// Creates the start and goal states 
		TreeSet<DQNObstacle> obstacles = Helper.getCloseObstacles(origin, this.getPlanningContinuum(), this.getTrainingObstacles(), this.getRiskPolicy());
		this.setStart(new State(pOrigin, pDestination, this.getPlanningContinuum(), obstacles, this.getCostPolicy(), etd, this.getStepSize(), null));
		this.setGoalPosition(pDestination);
		
		// Sets the state as the start
		this.setState(this.getStart());
		
		// Sets the "done" and "failure" booleans to false
		this.setDone(false);
		this.setFailure(false);
	
	}

	/**
	 * Computes a plan according to the learned policy in the DQN
	 * @throws TranslateException 
	 */
	protected void compute() {
		
		// Adds the start waypoint to the trajectory if this is the first part
		if (this.getWaypoints().isEmpty()) {
			this.getWaypoints().addLast(this.getNewestWaypoint());
		}
		
		int step = 0;
		
		// Until the goal is reached, the planner fails, or does the max steps
		while(!this.isDone() && !this.failed() && step < MAX_STEPS) {
			
			// Chooses the action
			try (NDManager submanager = manager.newSubManager()) {
				
				NDArray qValues = targetPredictor.predict(new NDList(submanager.create(this.getState().getId()))).singletonOrThrow();
				// DEBUG
				System.out.print("Q VALUES: ");
				for (int i=0; i<qValues.size(); i++) {
					System.out.print(qValues.getFloat(i) + "; ");
				}
				System.out.println();
				this.setAction(ActionSampler.greedy(qValues));
				
			} catch (TranslateException e) {
				throw new IllegalStateException(e);
			}
			
			System.out.println("STATE " + (step + 1));
			System.out.println("Cost Policy: " + state.getId()[0] + "; Step size: " + state.getId()[1]);
			System.out.println("Distance to axes: " + state.getId()[2] + "; " + state.getId()[3] + "; " + state.getId()[4] 
					+ "; " + state.getId()[5] + "; " + state.getId()[6] + "; " + state.getId()[7]);
			System.out.println("Relative to goal: " + state.getId()[8] + "; " + state.getId()[9] + "; " + state.getId()[10] + "; " + state.getId()[11]);
			System.out.println("Obstacle 1: " + state.getId()[12] + "; " + state.getId()[13] + "; " + state.getId()[14] + "; " + state.getId()[15]);
			System.out.println("Obstacle 2: " + state.getId()[16] + "; " + state.getId()[17] + "; " + state.getId()[18] + "; " + state.getId()[19]);
			System.out.println("Obstacle 3: " + state.getId()[20] + "; " + state.getId()[21] + "; " + state.getId()[22] + "; " + state.getId()[23]);
			System.out.println("Action: " + action);
			
			// Execute action and get next state and reward; Checks if the goal has been reached
			this.step();
			
			// Adds the next waypoint to the trajectory
			this.getWaypoints().addLast(this.createWaypoint(this.getNextState().getPosition()));
			
			// Sets the state as the next state
			this.setState(this.getNextState());
			
			step++;
		}
		
		// If it finished without reaching the goal (because it reached the maximum number of steps
		// or failed) returns an empty trajectory
		if (!this.isDone()) {
			this.clearWaypoints();
			if (step >= MAX_STEPS ) {
				System.out.println("Did too many steps");
			} else if (this.getReward()==-100){
				System.out.println("Hit an obstacle");
			} else {
				System.out.println("Left environment");
			}
			return;
		}
		
		// Adds the goal to the trajectory if it is not already there
		RLWaypoint goalWaypoint = this.createWaypoint(this.getGoalPosition());
		if(this.getWaypoints().getLast() != goalWaypoint) {
			this.getWaypoints().addLast(goalWaypoint);
		}
		
	    // Sets ETOs correctly
		this.getWaypoints().getFirst().setEto(getEtd());
		for (int i=1; i< this.getWaypoints().size(); ++i) {
			computeEto(this.getWaypoints().get(i-1), this.getWaypoints().get(i));
		}
	}
	
	
	/**
	 * Creates a waypoint at a specified position
	 * 
	 * @param position the position
	 * 
	 * @return the waypoint at the specified position
	 */
	protected RLWaypoint createWaypoint(Position position) {
		
		RLWaypoint wp = new RLWaypoint(position);
		
		// If it is not the start
		if(this.getNewestWaypoint() != null) {
			computeEto(this.getNewestWaypoint(), wp);
			wp.setPoi(true);
		}
		
		return wp;

	}
	
	
	/**
	 * Plans a part of a trajectory.
	 * 
	 * @param partIndex the index of the part
	 * 
	 * @return the planned part of a trajectory
	 * 
	 */
	protected Trajectory planPart(int partIndex) {
		this.compute();
		System.out.println("Computed trajectory:" + this.getWaypoints());
		return this.createTrajectory();
	}
	
	

	/**
	 * Plans a trajectory from an origin to a destination at a specified
	 * estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @return the planned trajectory from the origin to the destination with
	 *         the estimated time of departure
	 * 
	 * @see Planner#plan(Position, Position, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, ZonedDateTime etd) {
		
		this.clearWaypoints();
		
		this.syncNetworks();
		
		actualRiskPolicy = this.getRiskPolicy();
		actualCostPolicy = this.getCostPolicy();
		
		this.initialize(origin, destination, etd);
		
		Trajectory trajectory = this.planPart(0);
		
		this.revisePlan(trajectory);
		return trajectory;		
	}
	
	/**
	 * Plans a trajectory from an origin to a destination along waypoints at a
	 * specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param waypoints the waypoints in globe coordinates
	 * @param etd the estimated time of departure
	 * 
	 * @return the planned trajectory from the origin to the destination along
	 *         the waypoints with the estimated time of departure
	 * 
	 * @see Planner#plan(Position, Position, List, ZonedDateTime)
	 */
	@Override
	public Trajectory plan(Position origin, Position destination, List<Position> waypoints, ZonedDateTime etd) {
		
		this.clearWaypoints();
		
		this.syncNetworks();
		
		actualRiskPolicy = this.getRiskPolicy();
		actualCostPolicy = this.getCostPolicy();
		
		Position currentOrigin = origin;
		ZonedDateTime currentEtd = etd;
		
//		// collect intermediate destinations
//		ArrayList<RLWaypoint> destinations = waypoints.stream()
//				.map(RLWaypoint::new)
//				.collect(Collectors.toCollection(ArrayList::new));
//		destinations.add(new RLWaypoint(destination));
		
		waypoints.add(destination);

		
		// plan and concatenate partial trajectories
		for (int partIndex = 0; partIndex < waypoints.size(); partIndex++) {
			Position currentDestination = waypoints.get(partIndex);
			if (!(currentOrigin.equals(currentDestination))) {
				
				/* 
				 * Each part of a multi-part plan has to be computed completely
				 * in order to finalize the ETO of the goal waypoint which
				 * becomes the start waypoint of the next part and the basis
				 * for any subsequent plan revisions. A possible repair of one
				 * part requires the re-computation of all subsequent parts.
				 * This cost-greedy solution does not necessarily result in
				 * optimality with respect to the overall cost of the computed
				 * multi-part plan.
				 * 
				 * https://github.com/stephanheinemann/worldwind/issues/24
				 */
				
				// plan partial trajectory
				this.initialize(currentOrigin, currentDestination, currentEtd);
				this.planPart(partIndex);
				
				if ((!this.hasWaypoints()) || !(this.getWaypoints().getLast().equals(new RLWaypoint(currentDestination)))) {
					// if no plan could be found, return an empty trajectory
					Trajectory empty = new Trajectory();
					this.revisePlan(empty);
					return empty;
				} else {
					// revise growing trajectory for each part
					this.revisePlan(this.createTrajectory());
					currentOrigin = currentDestination;
					currentEtd = this.getWaypoints().getLast().getEto();
				}
			}
		}
		return this.createTrajectory();
		
	}
	
	/**
	 * Determines whether or not this DQN planner supports a specified
	 * environment.
	 * 
	 * @param environment the environment
	 * 
	 * @return true if the environment is a planning continuum, false otherwise
	 * 
	 * @see PlanningContinuum
	 */
	@Override
	public boolean supports(Environment environment) {
		boolean supports = super.supports(environment);
		
		if (supports) {
			supports = (environment instanceof PlanningContinuum);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
		}
		
		return supports;
	}





	
}

