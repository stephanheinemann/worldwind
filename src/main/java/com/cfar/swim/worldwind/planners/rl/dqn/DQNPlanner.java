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
import com.cfar.swim.worldwind.planners.rl.dqn.Memory;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.render.Obstacle;
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
* in an environment considering a local cost and risk policy.
* 
* @author Rafaela Seguro
*
*/

public class DQNPlanner extends AbstractPlanner {
	
	/** the minimum exploration rate used during training */
	private static final float MIN_EXPLORE_RATE = 0.0f; 
	
	/** the rate at which the exploration rate decays over time */
	private static final float DECAY_EXPLORE_RATE = 0.9f;
	
	/** number of episodes for the global training */
	private static final int NUM_GLOBAL_EPS = 1000;
	
	/** maximum number of steps per episode */
	private static final int MAX_STEPS = 200;
	
	/** indicates if the DQN has been trained or not */
	private boolean trained = false;
	
	/** random number */
	private final Random rand = new Random();
	
	/** replay memory */
	private final Memory memory = new Memory(4096);
	
	/** maps states to their IDs */
	protected Map<float[], State> stateMap = new TreeMap<>(new FloatArrayComparator());
	
	/** dimension of a space ID (input of the neural network) */
	private int dimOfSpace;
	
	/** the list of available actions in each state */
	private final ArrayList<Vec4> listOfActions;
	
	/** the number of available actions in each state */
	private final int numOfActions;
	
	/** the number of hidden units (neurons) in the neural network */
	private final int[] hiddenSize = {64, 128, 128, 64};
	
	/** learning rate used by the optimizer during training */
	private final float learningRate = 0.001f;
	
	/** the size of the mini-batch of transitions used for training */
	protected final int batchSize = 32;
	
	/** the number of iterations between each sync of the target and policy networks */
	protected final int syncNetInterval = 10;
	
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
	private float epsilon = 1.0f; 
	
	/** the loss function */
	private final L2Loss lossFunc = new L2Loss();
	
	/** the planner's start state */
	private State start = null;
	
	/** the planner's goal state */
	private State goal = null;
	
	/** the current state */
	private State state = null;
	
	/** the next state */
	private State nextState = null;
	
	/** the index of the chosen action */
	private int action = 0;
	
	/** the received reward */
	private double reward = 0;
	
	/** indicates if it reached goal or not, true if it has */
	private boolean done = false;
	
	/** indicates if it reached goal or not, true if it has */
	private boolean failure = false;
	
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
	private ZonedDateTime etd = ZonedDateTime.now();
	
	
	/** Constructs a planner trained by a Deep Q-Network for a specified aircraft and
	 * environment using default local cost and risk policies.
	 * 
	 * @param the aircraft
	 * @param the environment
	 */
	public DQNPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		this.dimOfSpace = State.ID_SIZE;
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
	 * Gets the goal state of this planner.
	 * 
	 * @return the goal state of this planner
	 */
	protected State getGoal() {
		return this.goal;
	}
	
	/**
	 * Sets the goal state of this planner.
	 * 
	 * @param goal the goal state of this planner
	 */
	protected void setGoal(State goal) {
		this.goal = goal;
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
		
		System.out.println("TRAINING GLOBAL");
		
		// Sets epsilon to one, for zero exploitation
		this.epsilon = 1.0f;
		
		int episode = 0;
		int step;
		
		// For each episode
		while (episode < NUM_GLOBAL_EPS) {
			// Reset environment 
			episode++;
			step = 0;
			resetEnvironment();
			resetVisitedStates();
			
			System.out.print("Episode " + episode + "\r");
		
			// For each time step
			while (!this.isDone() && step < MAX_STEPS) {
				// Sets the state to visited
				this.stateMap.get(this.getState().getId()).setVisited(true);
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
			// Updates epsilon for next episode
//			if (this.isDone()) {
//				System.out.println("Episode " + episode + " reached goal after " + step + " steps");
//			} else {
//				System.out.println("Episode " + episode + " did not reach goal");
//			}
		}
	}
	
	
	/**
	 * Runs the training of the Deep Q-Network for a specific start and goal
	 */
	protected void trainLocal(int numEpisodes, Position origin, Position destination, ZonedDateTime etd) {
		
		System.out.println("TRAINING LOCAL");
		
		// Sets epsilon to 0.1
		this.epsilon = 1.0f;
		
		int episode = 0;
		int step;
		
		// Creates the start and goal states and adds them to the state map
		Set<DQNObstacle> obstacles = Helper.getCloseObstacles(origin, this.getAircraft().getRadius(), this.getEnvironment(), this.getRiskPolicy());
		this.setStart(this.addState(new State(origin, destination, obstacles, this.getCostPolicy(), etd, this.getEnvironment().getGlobe())));
		obstacles = Helper.getCloseObstacles(destination, this.getAircraft().getRadius(), this.getEnvironment(), this.getRiskPolicy());
		this.setGoal(this.addState(new State(destination, destination, obstacles, this.getCostPolicy(), null, this.getEnvironment().getGlobe())));
		
		System.out.print("Episode " + episode + "\r");
		
		// For each episode
		while (episode < numEpisodes) {
			// Reset environment 
			this.setDone(false);
			resetVisitedStates();
			episode++;
			step = 0;
			this.setState(this.getStart());
		
			// For each time step
			while (!this.isDone() && step < MAX_STEPS) {
				// Sets the state to visited
				this.stateMap.get(this.getState().getId()).setVisited(true);
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
		
		// Starts by sampling random positions for the start and goal, following a uniform distribution
		Position startPosition = this.getPlanningContinuum().sampleRandomUniformPosition();
		Position goalPosition = startPosition;
		// Makes sure the start and goal are different
		while (goalPosition == startPosition)
			goalPosition = this.getPlanningContinuum().sampleRandomUniformPosition();
		
		// Adds obstacles to the environment
		//TODO: create function to randomize obstacles and policies and add them to the environment
		
		// Creates the start and goal states and adds them to the state map
		Set<DQNObstacle> obstacles = Helper.getCloseObstacles(startPosition, this.getAircraft().getRadius(), this.getEnvironment(), this.getRiskPolicy());
		this.setStart(this.addState(new State(startPosition, goalPosition, obstacles, this.getCostPolicy(), this.getEtd(), this.getEnvironment().getGlobe())));
		obstacles = Helper.getCloseObstacles(goalPosition, this.getAircraft().getRadius(), this.getEnvironment(), this.getRiskPolicy());
		this.setGoal(this.addState(new State(goalPosition, goalPosition, obstacles, this.getCostPolicy(), null, this.getEnvironment().getGlobe())));
		
		// Sets the start and goal position and goal point
		this.setStartPosition(startPosition);
		this.setGoalPosition(goalPosition);
		this.setGoalPoint(this.getEnvironment().getGlobe().computePointFromPosition(goalPosition));
		
		// Sets the state as the start
		this.setState(this.getStart());
		
		// Sets the "done" boolean to false
		this.setDone(false);
		
	}

	/**
	 * Resets all states saved in the state map to not visited 
	 */
	protected void resetVisitedStates() {
		
		if (stateMap.isEmpty())
			return;
		for (Map.Entry<float[], State> s : stateMap.entrySet()) {
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
	protected State addState(State state) {
//		boolean exists = false;
//		
//		for (Map.Entry<float[], State> s : stateMap.entrySet()) {
//			if (s.getValue().equals(state)) {
//				exists = true;
//				// Stores most recent state
//				s.getKey().;
//				return s.getValue();
//			}
//		}
//		
//		if (!exists) {
//			stateMap.put(state.getId(), state);
//		}
		
		// Should be enough since it is a tree map, already does this check and stores the most recent one
		stateMap.put(state.getId(), state);
		
		return state;
		
	}

	/** 
	 * Reacts to the current state, updating the memory and choosing the next action
	 */
	protected void react() {
		int action;
		try (NDManager submanager = manager.newSubManager()) {
			
			memory.setState(this.getState().getId());
			
			if (memory.getSize() > batchSize)
				updateModel(submanager);
			
			action = chooseAction(submanager, this.getState().getId());
			
			memory.setAction(action);
			
		} catch (TranslateException e) {
			throw new IllegalStateException(e);
		}
		
		this.setAction(action);;
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
		// Chooses the action using the epsilon greedy policy and the predicted Q-values
		return ActionSampler.epsilonGreedy(qValues, rand, Math.max(MIN_EXPLORE_RATE, epsilon));
	}

	/** 
	 * Determines the next state based on the chosen action
	 */
	protected void step() {
		
		int action = this.getAction();
		double stepSize = 2 * this.getAircraft().getCapabilities().getCruiseSpeed();
		
		// Determines the movement from the current state to the next based on action
		Vec4 movVector;
		if (action == 0) {
			// If the action index is 0, the action corresponds to just flying in the direction of the goal
			movVector = this.getState().getrelativeToGoal().normalize3().multiply3(stepSize);
		} else {
			movVector = listOfActions.get(action).multiply3(stepSize);
		}
		
		// Computes the next state's position
		Position nextStatePosition = this.getEnvironment().getGlobe().computePositionFromPoint(this.getGoalPoint()
				.subtract3(this.getState().getrelativeToGoal().subtract3(movVector)));
		
		// Gets the set of surrounding obstacles of the next state
		Set<DQNObstacle> obstacles = Helper.getCloseObstacles(nextStatePosition, this.getAircraft().getRadius(), this.getEnvironment(), this.getRiskPolicy());
		
		// Computes the next state's ETO
		Path leg = new Path(this.getState().getPosition(), nextStatePosition);
		ZonedDateTime eto = this.getAircraft().getCapabilities().getEstimatedTime(leg, this.getEnvironment().getGlobe(), this.getState().getEto());
		
		// Creates the next state and adds it to the map
		State nextState = new State(nextStatePosition, this.getGoalPosition(), obstacles, this.getCostPolicy(), eto, this.getEnvironment().getGlobe());
		this.addState(nextState);
		this.setNextState(nextState);
		
		// Checks if it has reached goal
		if (this.getNextState().getDistanceToGoal() <= stepSize)
			this.setDone(true);
		
		// Calculates reward
		this.calculateReward();
	}

	/**
	 * Calculates the reward based on the next state
	 */
	protected void calculateReward() {
		
		int reward = 0;
		
		// If the goal has been reached
		if (done) {
			this.setReward(1000);
			return;
		}
		
		// If the cost is exceeded
		double cost = this.getEnvironment().getStepCost(this.getState().getPosition(), this.getNextState().getPosition(), this.getState().getEto(), 
				this.getNextState().getEto(), getCostPolicy(), getRiskPolicy());
		if (Double.POSITIVE_INFINITY == cost) {
			this.setReward(-1000);
			this.setFailure(true);
			return;
		}
		
		// If the state is repeated -100
		if (this.getNextState().isVisited())
			reward += -50;
		
		// Otherwise: -10 for step + 10 * distance difference
		reward += -10;
		reward += 10 * (this.getState().getDistanceToGoal() - this.getNextState().getDistanceToGoal());
		
		this.setReward(reward);
	
	}

	/** 
	 * Updates the memory with the received reward and the 'done' boolean
	 */
	protected void collect() {
		if(!isTrained())
			memory.setReward(this.getReward()*100, this.isDone(), this.failed());
	}

	/** 
	 * Gets a batch of transitions from memory, calculates the loss based on the predicted Q-values and the actual rewards and 
	 * performs the gradient update
	 * 
	 * @param manager the memory space manager
	 */
	protected void updateModel(NDManager manager) throws TranslateException {
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
	}

	/** 
	 * Performs the gradient update through backpropagation and, each "syncNetInterval" iterations, it syncs the policy 
	 * and target networks and updates the epsilon value for the epsilon greedy policy
	 * 
	 * @param loss the loss
	 */
	protected void gradientUpdate(NDArray loss) {
		try (GradientCollector collector = Engine.getInstance().newGradientCollector()) {
			collector.backward(loss);
			for (Pair<String, Parameter> params : policyNet.getBlock().getParameters()) {
				NDArray paramsArr = params.getValue().getArray();
				optimizer.update(params.getKey(), paramsArr, paramsArr.getGradient());
			}
		}
		if (iteration++ % syncNetInterval == 0) {
			epsilon *= DECAY_EXPLORE_RATE;
			syncNetworks();
		}
	}

	/** 
	 * Copies the weights from the main network to the target network
	 */
	protected void syncNetworks() {
	
		for (Pair<String, Parameter> params : policyNet.getBlock().getParameters()) {
			NDArray policyArray = params.getValue().getArray();
			NDArray targetArray = targetNet.getBlock().getParameters().get(params.getKey()).getArray();
			
			policyArray.copyTo(targetArray);
			
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
		Set<DQNObstacle> obstacles = Helper.getCloseObstacles(origin, this.getAircraft().getRadius(), this.getEnvironment(), this.getRiskPolicy());
		this.setStart(this.addState(new State(origin, destination, obstacles, this.getCostPolicy(), etd, this.getEnvironment().getGlobe())));
		obstacles = Helper.getCloseObstacles(destination, this.getAircraft().getRadius(), this.getEnvironment(), this.getRiskPolicy());
		this.setGoal(this.addState(new State(destination, destination, obstacles, this.getCostPolicy(), null, this.getEnvironment().getGlobe())));
		
		// Sets the state as the start
		this.setState(this.getStart());
		
		// Sets the "done" boolean to false
		this.setDone(false);
	
	}

	/**
	 * Computes a plan according to the learned policy in the DQN
	 * @throws TranslateException 
	 */
	protected void compute() {
		
		Globe globe = this.getEnvironment().getGlobe();
		
		// Sets epsilon to zero, for zero exploration
		this.epsilon = 0f;
		
		// Adds the start waypoint to the trajectory if this is the first part
		if (this.getWaypoints().isEmpty()) {
			this.getWaypoints().addLast(this.getNewestWaypoint());
		}
		
		// Until the goal is reached
		while(!this.isDone()) {
			
			// Chooses the action
			try (NDManager submanager = manager.newSubManager()) {
				
				this.setAction(chooseAction(submanager, this.getState().getId()));
				
			} catch (TranslateException e) {
				throw new IllegalStateException(e);
			}
			
			System.out.println("State: " + this.getState().getDistanceToGoal() + ": Action:" + this.getAction());
			
			// Execute action and get next state and reward; Checks if the goal has been reached
			this.step();
			
			// Adds the next waypoint to the trajectory
			// Creates waypoint by subtracting the next state's relative vector to the goal
			this.getWaypoints().addLast(this.createWaypoint(globe.computePositionFromPoint(this.getGoalPoint().subtract3(this.getNextState().getrelativeToGoal()))));
			
			// Sets the state as the next state
			this.setState(this.getNextState());
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
		
		//TODO: find metric to decide number of episodes for local training depending on case
		//this.trainLocal(200, origin, destination);
		
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

