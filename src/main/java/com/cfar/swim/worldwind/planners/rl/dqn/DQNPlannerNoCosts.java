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
import com.cfar.swim.worldwind.planners.rl.StateNoCosts;
import com.cfar.swim.worldwind.planners.rl.dqn.Memory;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.registries.Specification;
//import com.cfar.swim.worldwind.tests.Plot;
import com.cfar.swim.worldwind.util.Identifiable;

import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;
import gov.nasa.worldwind.render.Path;
import ai.djl.Model;
import ai.djl.engine.Engine;
import ai.djl.inference.Predictor;
import ai.djl.ndarray.*;
import ai.djl.nn.Parameter;
import ai.djl.training.GradientCollector;
import ai.djl.training.loss.L2Loss;
import ai.djl.training.optimizer.Optimizer;
import ai.djl.training.tracker.Tracker;
import ai.djl.translate.TranslateException;
import ai.djl.util.Pair;
import ai.djl.translate.NoopTranslator;

/**
* Realizes a deep reinforcement learning planner, using a Deep Q-Network, that plans a trajectory of an aircraft
* in an environment without considering the local cost and risk policy.
* 
* @author Rafaela Seguro
*
*/

public class DQNPlannerNoCosts extends AbstractPlanner {

	/** the minimum exploration rate used during training */
	private static final float MIN_EXPLORE_RATE = 0.1f; 
	
	/** the rate at which the exploration rate decays over time */
	private static final float DECAY_EXPLORE_RATE = 0.99f;
	
	/** the initial value of epsilon */
	private static final float INITIAL_EPSILON = 0.7f;
	
	/** number of episodes for the global training */
	private static final int NUM_GLOBAL_EPS = 5000;
	
	/** maximum number of steps per episode */
	private static final int MAX_STEPS = 100;
	
	/** indicates if the DQN has been trained or not */
	private boolean trained = false;
	
	/** random number */
	private final Random rand = new Random();
	
	/** replay memory */
	private final Memory memory = new Memory(4096);
	
	/** maps states to their IDs */
	protected Map<float[], StateNoCosts> stateMap = new TreeMap<>(new FloatArrayComparator());
	
	/** dimension of a space ID (input of the neural network) */
	private int dimOfSpace;
	
	/** the list of available actions in each state */
	private final ArrayList<Vec4> listOfActions;
	
	/** the number of available actions in each state */
	private final int numOfActions;
	
	/** the number of hidden units (neurons) in the neural network */
	private final int[] hiddenSize = {64, 128, 128, 64};
	
	/** learning rate used by the optimizer during training */
	private final float learningRate = 0.0025f;
	
	/** the size of the mini-batch of transitions used for training */
	protected final int batchSize = 32;
	
	/** the number of iterations between each train of the policy network */
	protected final int trainNetInterval = 2;
	
	/** the number of iterations between each sync of the target and policy networks */
	protected final int syncNetInterval = 50;
	
	/** gamma factor for the Bellman equation */
	protected final float gamma = 0.99f;
	
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
	
	/** the minimum, maximum and current epsilon value to implement decaying e-greedy */
//	private float minEpsilon = 0.0f; 
//	private float maxEpsilon = 1.0f; 
	private float epsilon = INITIAL_EPSILON; 
	
	/** the loss function */
	private final L2Loss lossFunc = new L2Loss();
	
	/** the planner's start state */
	private StateNoCosts start = null;
	
	/** the planner's goal state */
	private StateNoCosts goal = null;
	
	/** the current state */
	private StateNoCosts state = null;
	
	/** the next state */
	private StateNoCosts nextState = null;
	
	/** the index of the chosen action */
	private int action = 0;
	
	/** the received reward */
	private double reward = 0.0;
	
	/** indicates if it reached goal or not, true if it has */
	private boolean done = false;
	
	/** indicates if it reached goal or not, true if it has */
	private boolean failure = false;
	
	/** indicates the result of a training episode: -1 if failure, 0 if ran out of steps, 1 if reached goal */
	private int episodeResult = 0;
	
	/** the radius of the sphere defining the goal region of this DQN planner */
	private double goalThreshold = 10d; // (0, Double.MAX_Value]
	
	/** saves the most recent waypoint added to the trajectory */
	private RLWaypoint newestWaypoint = null;
	
	/** saves the current goal as a point */
	private Vec4 goalPoint = null;
	
	/** saves the current start position */
	private Position startPosition = null;
	
	/** saves the current goal position */
	private Position goalPosition = null;
	
	/** stores the plan's ETD */
	private ZonedDateTime etd;
	
	/** times */
	long reactTime = 0;
	long stepTime = 0;
	long chooseActionTime = 0;
	long updateModelTime = 0;
	long gradientUpdateTime = 0;
	long resetEnvironmentTime = 0;
	long syncNetworksTime = 0;
	long addStateTime = 0;
	
	
	/** Constructs a planner trained by a Deep Q-Network for a specified aircraft and
	 * environment using default local cost and risk policies.
	 * 
	 * @param the aircraft
	 * @param the environment
	 */
	public DQNPlannerNoCosts(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		this.dimOfSpace = StateNoCosts.ID_SIZE;
		this.listOfActions = Helper.listOfActions();
		this.numOfActions = listOfActions.size();
		
		resetAgent();
		trainGlobal();
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
		return Specification.PLANNER_DQN_NOCOSTS_ID;
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
	 * Sets the boolean that indicates if the DQN has been trained.
	 * 
	 * @param true if the DQN has been trained, false otherwise
	 */
	protected void setTrained(boolean trained) {
		this.trained = trained;
	}
	
	/**
	 * Checks if the DQN has been trained.
	 * 
	 * @return true if the DQN has been trained, false otherwise
	 */
	protected boolean isTrained() {
		return trained;
	}
	
	/**
	 * Gets the size of a state's ID, which is the size of the network input.
	 * 
	 * @return the size of a state's ID
	 */
	protected int getDimOfSpace() {
		return this.dimOfSpace;
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
	protected StateNoCosts getStart() {
		return this.start;
	}
	
	/**
	 * Sets the start state of this  planner.
	 * 
	 * @param start the start state of this planner
	 */
	protected void setStart(StateNoCosts start) {
		this.start = start;
	}

	/**
	 * Gets the goal state of this planner.
	 * 
	 * @return the goal state of this planner
	 */
	protected StateNoCosts getGoal() {
		return this.goal;
	}
	
	/**
	 * Sets the goal state of this planner.
	 * 
	 * @param goal the goal state of this planner
	 */
	protected void setGoal(StateNoCosts goal) {
		this.goal = goal;
	}
	
	/**
	 * Gets the current state.
	 * 
	 * @return the state 
	 */
	protected StateNoCosts getState() {
		return this.state;
	}
	
	/**
	 * Sets the current state.
	 * 
	 * @param state the current state
	 */
	protected void setState(StateNoCosts state) {
		this.state = state;
	}
	
	/**
	 * Gets the next state.
	 * 
	 * @return the next state
	 */
	protected StateNoCosts getNextState() {
		return this.nextState;
	}

	/**
	 * Sets the next state of this planner.
	 * 
	 * @param nextState the next state of this planner
	 */
	protected void setNextState(StateNoCosts nextState) {
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
	 * @param d the reward
	 */
	protected void setReward(double reward) {
		this.reward = reward;
	}
	
	/**
	 * Checks if the planner has reached the goal
	 * 
	 * @return the reward
	 */
	protected boolean isDone() {
		return this.done;
	}
	
	/**
	 * Sets the boolean that indicates if the planner has reached the goal.
	 * 
	 * @param the boolean value
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
	 * Gets the result of the current training episode
	 * 
	 * @return the result of the episode: -1 if failure, 0 if ran out of steps, 1 if reached goal
	 */
	protected int getEpisodeResult() {
		return this.episodeResult;
	}
	
	/**
	 * Sets the result of the current training episode.
	 * 
	 * @param the result: -1 if failure, 0 if ran out of steps, 1 if reached goal
	 */
	protected void setEpisodeResult(int result) {
		this.episodeResult = result;
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
	 * Gets the current goal in point format.
	 * 
	 * @return the current goal in point format
	 */
	protected Vec4 getGoalPoint() {
		return this.goalPoint;
	}
	
	/**
	 * Sets the current goal in point format.
	 * 
	 * @param the current goal in point format
	 */
	protected void setGoalPoint(Vec4 goalPoint) {
		this.goalPoint = goalPoint;
	}
	
	/**
	 * Gets the current start position.
	 * 
	 * @return the current start position
	 */
	protected Position getStartPosition() {
		return this.startPosition;
	}
	
	/**
	 * Sets the current start position.
	 * 
	 * @param the current start position
	 */
	protected void setStartPosition(Position startPosition) {
		this.startPosition = startPosition;
	}
	
	/**
	 * Gets the current goal position.
	 * 
	 * @return the current goal position
	 */
	protected Position getGoalPosition() {
		return this.goalPosition;
	}
	
	/**
	 * Sets the current goal position.
	 * 
	 * @param the current goal position
	 */
	protected void setGoalPosition(Position goalPosition) {
		this.goalPosition = goalPosition;
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
	


	
	/** Resets the DQN agent before training
	 */
	protected void resetAgent() {
		optimizer = Optimizer.adam().optLearningRateTracker(Tracker.fixed(learningRate)).build();
		
		if (manager != null) {
			manager.close();
		}
		
		manager = NDManager.newBaseManager();
		policyNet = NetworkModel.newModel(manager, dimOfSpace, hiddenSize, numOfActions);
		targetNet = NetworkModel.newModel(manager, dimOfSpace, hiddenSize, numOfActions);
		
		policyPredictor = policyNet.newPredictor(new NoopTranslator());
		syncNetworks();
	}

	/**
	 * Runs the training of the Deep Q-Network for random environment configurations
	 */
	protected void trainGlobal() {
		
		long start = System.nanoTime() / 100000;
		
		System.out.println("TRAINING GLOBAL");
		System.out.println();
		
		// Sets epsilon to one, for zero exploitation
		this.epsilon = 1.0f;
		
		double decay = 0.000;
		int episode = 0;
		int step;
		double score;
		
		// For each episode
		while (episode < NUM_GLOBAL_EPS) {
			// Reset environment 
			episode++;
			step = 0;
			score = 0;
			resetEnvironment();
			resetVisitedStates();
			
//			System.out.print("Episode " + episode + "\r");
			long startEpisode = System.nanoTime() / 100000;
			
			// For each time step
			while (!this.isDone() && !this.failed() && step < MAX_STEPS) {
				// Sets the state to visited
				//this.stateMap.get(this.getState().getId()).setVisited(true);
				// Saves state in memory; Updates the network; Selects the next action and saves it in memory
				this.react();
				// Execute action and get next state and reward; Checks if the goal has been reached
				this.step();
				//System.out.println("State: " + this.getState().getDistanceToGoal() + " Action: " + this.getAction() + " Reward:" + this.getReward());
				// Stores the reward and the "done" boolean in memory
				this.collect();
				score += this.getReward();
				// Sets the state as the next state
				this.setState(this.getNextState());
				step++;
			}
			// Update epsilon for next episode
			decay = Math.min(((double) episode / NUM_GLOBAL_EPS), 1.0);
			epsilon = (float) (INITIAL_EPSILON - (INITIAL_EPSILON - MIN_EXPLORE_RATE) * decay);
			long endEpisode = System.nanoTime() / 100000;
			long episodeTime = endEpisode - startEpisode;
			if (this.isDone()) {
				//System.out.println("-------------------------------------------------------------------------------------");
				System.out.println("Episode " + episode + " had score " + score + " and reached GOAL after " + step + " steps");
			} else if (this.failed()) {
				System.out.println("Episode " + episode + " had score " + score + "  and failed");
			} else {
				System.out.println("Episode " + episode + " had score " + score + "  and did not reach goal");
			} 
//			System.out.println("Total episode time: " + episodeTime);
//			System.out.println("Time in react: " + reactTime);
//			System.out.println("Time in step: " + stepTime);
//			System.out.println("Time in chooseAction: " + chooseActionTime);
//			System.out.println("Time in updateModel: " + updateModelTime);
//			System.out.println("Time in gradientUpdate: " + gradientUpdateTime);
//			System.out.println("Time in resetEnvironment: " + resetEnvironmentTime);
//			System.out.println("Time in syncNetworks: " + syncNetworksTime);
//			System.out.println("Time in addState: " + addStateTime);
			reactTime = 0;
			stepTime = 0;
			chooseActionTime = 0;
			updateModelTime = 0;
			gradientUpdateTime = 0;
			resetEnvironmentTime = 0;
			syncNetworksTime = 0;
			addStateTime = 0;
			
		}
		
		long end = System.nanoTime() / 100000;
		long trainTime = (end - start);
		System.out.println("Total training time: " + trainTime);
	}
	
	
	/**
	 * Runs the training of the Deep Q-Network for a specific start and goal
	 */
	protected void trainLocal(int numEpisodes, Position origin, Position destination) {
		
		System.out.println("TRAINING LOCAL");
		
		// Sets epsilon to 0.1
		this.epsilon = 1.0f;
		
		int episode = 0;
		int step;
		
		// Creates the start and goal states and adds them to the state map
		this.setStart(this.addState(new StateNoCosts(origin, destination, this.getPlanningContinuum().getCorners(), this.getEnvironment().getGlobe())));
		this.setGoal(this.addState(new StateNoCosts(destination, destination, this.getPlanningContinuum().getCorners(), this.getEnvironment().getGlobe())));

		
		// For each episode
		while (episode < numEpisodes) {
			// Reset environment 
			this.setDone(false);
			resetVisitedStates();
			episode++;
			step = 0;
			this.setState(this.getStart());
			
			System.out.print("Episode " + episode + "\r");
		
			// For each time step
			while (!this.isDone() && step < MAX_STEPS) {
				// Sets the state to visited
				this.stateMap.get(Helper.flattenArray(this.getState().getId())).setVisited(true);
				// Saves state in memory; Updates the network; Selects the next action and saves it in memory
				this.react();
				// Execute action and get next state and reward; Checks if the goal has been reached
				this.step();
				// Stores the reward and the "done" boolean in memory
				this.collect();
				// Sets the state as the next state
				this.setState(this.getNextState());
				step++;
			}
//			if (this.isDone()) {
//				System.out.println("Episode " + episode + " reached goal after " + step + " steps");
//			} else {
//				System.out.println("Episode " + episode + " did not reach goal");
//			}
		}
	}

	/** 
	 * Resets the environment with a new random start and goal
	 */
	protected void resetEnvironment() {
		
		long start = System.nanoTime() / 100000;
		
		// Starts by sampling random positions for the start and goal, following a uniform distribution
		Position startPosition = this.getPlanningContinuum().sampleRandomUniformPosition();
		Position goalPosition = startPosition;
		// Makes sure the start and goal are different
		while (goalPosition == startPosition)
			goalPosition = this.getPlanningContinuum().sampleRandomUniformPosition();
		
		// Creates the start and goal states and adds them to the state map
		this.setStart(this.addState(new StateNoCosts(startPosition, goalPosition, this.getPlanningContinuum().getCorners(), this.getEnvironment().getGlobe())));
		this.setGoal(this.addState(new StateNoCosts(goalPosition, goalPosition, this.getPlanningContinuum().getCorners(), this.getEnvironment().getGlobe())));
		
		// Sets the start and goal position and goal point
		this.setStartPosition(startPosition);
		this.setGoalPosition(goalPosition);
		this.setGoalPoint(this.getEnvironment().getGlobe().computePointFromPosition(goalPosition));	
		
		// Sets the state as the start
		this.setState(this.getStart());
		
		// Sets the "done" boolean to false
		this.setDone(false);
		this.setFailure(false);
		
		long end = System.nanoTime() / 100000;
		resetEnvironmentTime += (end - start);
		
		//TODO: adicionar obstaculos e treinar para policies diferentes -> tem que ser adicionados tambem na definicao do estado e na comparacao
	}

	/**
	 * Resets all states saved in the state map to not visited 
	 */
	protected void resetVisitedStates() {
		
		if (stateMap.isEmpty())
			return;
		for (Map.Entry<float[], StateNoCosts> s : stateMap.entrySet()) {
			s.getValue().setVisited(false);
		}
	
	}

	/** 
	 * Adds a new state to the state map if it still doesn't exist
	 * 
	 * @param state the state to be added
	 * 
	 * @return the state
	 */
	protected StateNoCosts addState(StateNoCosts state) {
//		boolean exists = false;
//		
//		for (Map.Entry<float[], StateNoCosts> s : stateMap.entrySet()) {
//			if (s.getValue().equals(state)) {
//				exists = true;
//				return s.getValue();
//			}
//		}
//		
//		if (!exists) {
//			stateMap.put(state.getId(), state);
//		}
		stateMap.put(Helper.flattenArray(state.getId()), state);
		
		return state;
		
	}

	/** 
	 * Reacts to the current state, updating the memory and choosing the next action
	 */
	protected void react() {
		
		long start = System.nanoTime() / 100000;
		
		int action;
		try (NDManager submanager = manager.newSubManager()) {
			
			memory.setState(Helper.flattenArray(this.getState().getId()));
			
			// Trains every n iterations only
			if (iteration++ % trainNetInterval == 0  && memory.getSize()>batchSize)
				updateModel(submanager);
			
			// Syncs networks and updates every n iterations
			if (iteration % syncNetInterval == 0) {
//				epsilon *= DECAY_EXPLORE_RATE;
				syncNetworks();
			}
			
			action = chooseAction(submanager, Helper.flattenArray(this.getState().getId()));
			//System.out.println("Action is " + action);
			
			memory.setAction(action);
			
		} catch (TranslateException e) {
			throw new IllegalStateException(e);
		}
		
		this.setAction(action);
		
		long end = System.nanoTime() / 100000;
		reactTime += (end - start);
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
				
		// Gets the predicted Q-values from the target network
		NDArray qValues = targetPredictor.predict(new NDList(manager.create(state))).singletonOrThrow();
		//System.out.println("Q-values: " + qValues);
		// Chooses the action using the epsilon greedy policy and the predicted Q-values
		return ActionSampler.epsilonGreedy(qValues, rand, Math.max(MIN_EXPLORE_RATE, epsilon));
		
	}

	/** 
	 * Determines the next state based on the chosen action
	 */
	protected void step() {
		long start = System.nanoTime() / 100000;
		
		int action = this.getAction();
		double stepSize = 2 * this.getAircraft().getCapabilities().getCruiseSpeed();
		
		// Determines the movement from the current state to the next based on action
		Vec4 movVector;
		if (action == 0) {
			// If the action index is 0, the action corresponds to just flying in the direction of the goal
			movVector = this.getState().getRelativeToGoal().normalize3().multiply3(stepSize);
		} else {
			movVector = listOfActions.get(action).multiply3(stepSize);
		}
		
		// Computes the next state's position
		Position nextStatePosition = this.getEnvironment().getGlobe().computePositionFromPoint(this.getGoalPoint()
				.subtract3(this.getState().getRelativeToGoal().subtract3(movVector)));
		
		// If the next state is outside the environment, stays in place
		if (!this.getEnvironment().contains(nextStatePosition))
			nextStatePosition = this.getState().getPosition();
		
		// Faria sentido mas so se o estado guardasse informacao do terreno tambem
//		// If the next state collides with terrain, it stays in place
//		if (this.getEnvironment().collidesTerrain(this.getState().getPosition(), nextStatePosition))
//			nextStatePosition = this.getState().getPosition();
		
		// Creates the next state and adds it to the map
		StateNoCosts nextState = new StateNoCosts(nextStatePosition, this.getGoalPosition(), this.getPlanningContinuum().getCorners(), this.getEnvironment().getGlobe());
		this.addState(nextState);
		this.setNextState(nextState);
		
		// Checks if it has reached goal
		if (this.getNextState().getDistanceToGoal() <= stepSize)
			this.setDone(true);
		
//		// If the next state is outside the environment, it fails
//		if (!this.getEnvironment().contains(nextStatePosition))
//			this.setFailure(true);
		
		// Calculates reward
		this.calculateReward();
		
		long end = System.nanoTime() / 100000;
		stepTime += (end-start);
	}

	/**
	 * Calculates the reward based on the next state
	 */
	protected void calculateReward() {
		
		double reward = 0;
		
		// If the goal has been reached
		if (this.isDone()) {
			this.setReward(1000);
			this.setEpisodeResult(1);
			return;
		}
		
		// If it failed
		if (this.failed()){
			this.setReward(-50);
			return;
		}
		
		// If the cost is exceeded
		
//		// If the state is repeated -100
//		if (this.getNextState().isVisited())
//			reward += -50;
		
		// Otherwise: step penalty +  distance to goal penalty/reward 
		reward += -1;
		reward += 10 * (this.getState().getDistanceToGoal() - this.getNextState().getDistanceToGoal()) / this.getState().getDistanceToGoal();
		
		this.setReward(reward);
		//System.out.println("Reward is " + reward);
	
	}

	/** 
	 * Updates the memory with the received reward and the 'done' boolean
	 */
	protected void collect() {
		if(!isTrained())
			memory.setReward(this.getReward()*10, this.isDone(), this.failed());
	}

	/** 
	 * Gets a batch of transitions from memory, calculates the loss based on the predicted Q-values and the actual rewards and 
	 * performs the gradient update
	 * 
	 * @param manager the memory space manager
	 */
	protected void updateModel(NDManager manager) throws TranslateException {
		
		long start = System.nanoTime() / 100000;
		
		// Samples a batch of transitions from memory
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
		
		gradientUpdate(loss);
		
		long end = System.nanoTime() / 100000;
		updateModelTime += (end-start);
	}

	/** 
	 * Performs the gradient update through backpropagation and, each "syncNetInterval" iterations, it syncs the policy 
	 * and target networks and updates the epsilon value for the epsilon greedy policy
	 * 
	 * @param loss the loss
	 */
	protected void gradientUpdate(NDArray loss) {
		
		long start = System.nanoTime() / 100000;
		
		try (GradientCollector collector = Engine.getInstance().newGradientCollector()) {
			collector.backward(loss);
			for (Pair<String, Parameter> params : policyNet.getBlock().getParameters()) {
				NDArray paramsArr = params.getValue().getArray();
				optimizer.update(params.getKey(), paramsArr, paramsArr.getGradient());
			}
		}
//		if (iteration % syncNetInterval == 0) {
//			epsilon *= DECAY_EXPLORE_RATE;
//			syncNetworks();
//		}
		
		long end = System.nanoTime() / 100000;
		gradientUpdateTime += (end-start);
	}

	/** 
	 * Copies the weights from the main network to the target network
	 */
	protected void syncNetworks() {
		
		long start = System.nanoTime() / 100000;
	
		for (Pair<String, Parameter> params : policyNet.getBlock().getParameters()) {
			NDArray policyArray = params.getValue().getArray();
			NDArray targetArray = targetNet.getBlock().getParameters().get(params.getKey()).getArray();
			
			policyArray.copyTo(targetArray);
			
		}

		targetPredictor = targetNet.newPredictor(new NoopTranslator());
		
		long end = System.nanoTime() / 100000;
		syncNetworksTime += (end-start);
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
		
//		this.setNewestWaypoint(null);
		this.setEtd(etd);
		
		// Sets the start and goal positions
		this.setStartPosition(origin);
		this.setGoalPosition(destination);
		
		// Creates the start waypoint 
		this.setNewestWaypoint(this.createWaypoint(origin));
		this.getNewestWaypoint().setEto(etd);
		this.getNewestWaypoint().setPoi(true);
		
		// Sets the goal point
		this.setGoalPoint(this.getEnvironment().getGlobe().computePointFromPosition(destination));
		
		// Creates the start and goal states and adds them to the state map
		this.setStart(this.addState(new StateNoCosts(origin, destination, this.getPlanningContinuum().getCorners(), this.getEnvironment().getGlobe())));
		this.setGoal(this.addState(new StateNoCosts(destination, destination, this.getPlanningContinuum().getCorners(), this.getEnvironment().getGlobe())));
		
		// Sets the state as the start
		this.setState(this.getStart());
		
		// Sets the "done" boolean to false
		this.setDone(false);
		this.setFailure(false);
	
	}

	/**
	 * Computes a plan according to the learned policy in the DQN
	 * @throws TranslateException 
	 */
	protected void compute() {
		
		Globe globe = this.getEnvironment().getGlobe();
		
		// Adds the start waypoint to the trajectory if this is the first part
		if (this.getWaypoints().isEmpty()) {
			this.getWaypoints().addLast(this.getNewestWaypoint());
		}
		
		int step = 0;
		
		// Until the goal is reached
		while(!this.isDone() && !this.failed() && step < MAX_STEPS) {
			
			// Chooses the action
			try (NDManager submanager = manager.newSubManager()) {
				
				NDArray qValues = targetPredictor.predict(new NDList(submanager.create(Helper.flattenArray(this.getState().getId())))).singletonOrThrow();
				this.setAction(ActionSampler.greedy(qValues));
				
			} catch (TranslateException e) {
				throw new IllegalStateException(e);
			}
			
			System.out.println("State: " + this.getState().getDistanceToGoal() + ": Action:" + this.getAction());
			
			// Execute action and get next state and reward; Checks if the goal has been reached
			this.step();
			
			// Adds the next waypoint to the trajectory
			// Creates waypoint by subtracting the next state's relative vector to the goal
			this.getWaypoints().addLast(this.createWaypoint(globe.computePositionFromPoint(this.getGoalPoint().subtract3(this.getNextState().getRelativeToGoal()))));
			
			// Sets the state as the next state
			this.setState(this.getNextState());
			
			step++;
		}
		
		// If it finished without reaching the goal (because it reached the maximum number of steps
		// or failed) returns an empty trajectory
//		if (!this.isDone()) {
//			this.clearWaypoints();
//			return;
//		}
		
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
		
		//TODO: find metric to decide number of episodes for local training depending on case
		//this.trainLocal(500, origin, destination);
		
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
		
		//TODO: find metric to decide number of episodes for local training depending on case; should I train for each part?
		//this.trainLocal(100, origin, destination);
		
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

