package com.cfar.swim.worldwind.planners.rl.dqn;

import java.util.ArrayList;





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

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.environments.PlanningContinuum;
import com.cfar.swim.worldwind.environments.PlanningGrid;
import com.cfar.swim.worldwind.environments.PlanningRoadmap;
import com.cfar.swim.worldwind.geom.precision.PrecisionPosition;
import com.cfar.swim.worldwind.planners.AbstractPlanner;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.planners.cgs.astar.AStarWaypoint;
import com.cfar.swim.worldwind.planners.rl.Plot;
import com.cfar.swim.worldwind.planners.rl.QLine;
import com.cfar.swim.worldwind.planners.rl.RLWaypoint;
import com.cfar.swim.worldwind.planners.rl.State;
import com.cfar.swim.worldwind.planners.rl.Action;
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
* Realizes a reinforcement learning planner, using a Deep Q-Network that plans a trajectory of an aircraft
* in an environment considering a local cost and risk policy.
* 
* @author Rafaela Seguro
*
*/

public class DQNPlanner extends AbstractPlanner {
	
	/** the exploration rate */
	private static final float MIN_EXPLORE_RATE = 0.1f; 
	
	/** the decay rate */
	private static final float DECAY_EXPLORE_RATE = 0.99f;
	
	/** indicates if the DQN has been trained or not */
	private boolean trained = false;
	
	/** random number */
	private final Random rand = new Random();
	
	/** replay memory */
	private final Memory memory = new Memory(4096);
	
	/** maps states to their IDs */
	protected Map<Integer, State> stateMap = new TreeMap<>();
	
	/** dimension of the state space */
	private int dimStateSpace;
	
	/** the list of available actions in each state */
	private final ArrayList<Vec4> listOfActions;
	
	/** the number of available actions in each state */
	private final int numOfActions;
	
	/**  */
	private final int hiddenSize = 64;
	
	/**  */
	private final float learningRate = 0.001f;
	
	/**  */
	protected final int batchSize = 32;
	
	/**  */
	protected final int syncNetInterval = 32;
	
	/**  */
	protected final float gamma = 0.99f;
	
	/**  */
	private Optimizer optimizer;
	
	/**  */
	private Model policyNet;
	
	/**  */
	private Model targetNet;
	
	/**  */
	protected NDManager manager;
	
	/**  */
	protected Predictor<NDList, NDList> policyPredictor;
	
	/**  */
	protected Predictor<NDList, NDList> targetPredictor;
	
	/**  */
	private int iteration = 0;
	
	/**  */
	private float epsilon = 1.0f;
	
	/**  */
	private final L2Loss lossFunc = new L2Loss();
	
	/** The planner's start state */
	private State start = null;
	
	/** The planner's goal state */
	private State goal = null;
	
	/** The current state */
	private State state = null;
	
	/** The next state */
	private State nextState = null;
	
	/** The index of the chosen action */
	private int action = 0;
	
	/** The received reward */
	private double reward = 0;
	
	/** indicates if it reached goal or not, true if it has */
	private boolean done = false;
	
	/** Stores the current transition */
	private Transition transition;
	
	/** The received reward */
	private int stateCounter = 0;
	
	/** the radius of the sphere defining the goal region of this DQN planner */
	private double goalThreshold = 5d; // (0, Double.MAX_Value]
	
	
	/** Constructs a planner trained by a Deep Q-Network for a specified aircraft and
	 * environment using default local cost and risk policies.
	 * 
	 * @param the aircraft
	 * @param the environment
	 */
	public DQNPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		this.dimStateSpace = stateMap.size();
		this.listOfActions = Helper.listOfActions();
		this.numOfActions = listOfActions.size();
		
		resetAgent();
	}
	
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
	 * Gets the current size of the state space.
	 * 
	 * @return the current size of the state space
	 */
	protected int getDimOfStateSpace() {
		return this.stateMap.size();
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
	 * Creates a waypoint at a specified position
	 * 
	 * @param position the position
	 * 
	 * @return the waypoint at the specified position
	 */
	protected RLWaypoint createWaypoint(Position position) {
		return null;

	}
	
	
	/**
	 * Initializes this Q-Learning planner to plan from an origin to a destination at a
	 * specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 */
	protected void initialize(Position origin, Position destination, ZonedDateTime etd) {

	}
	
	/** Adds a new state to the state map if it still doesn't exist
	 * 
	 * @param state the state to be added
	 * 
	 * @return the state
	 */
	public State addState(State state) {
		boolean exists = false;
		
		for (Map.Entry<Integer, State> s : stateMap.entrySet()) {
			if (s.getValue().equals(state)) {
				exists = true;
				return s.getValue();
			}
		}
		
		if (!exists) {
			stateMap.put(stateCounter, state);
			state.setId(stateCounter);
			stateCounter++;
		}
		
		return state;
		
	}
	
	
	/** Resets the environment with a new random start and goal
	 */
	public void resetEnvironment() {
		
		// Starts by sampling random positions for the start and goal, following a uniform distribution
		Position startPosition = this.getPlanningContinuum().sampleRandomUniformPosition();
		Position goalPosition = startPosition;
		// Makes sure the start and goal are different
		while (goalPosition == startPosition)
			goalPosition = this.getPlanningContinuum().sampleRandomUniformPosition();
		
		// Creates the start and goal states and adds them to the state map
		this.setStart(this.addState(new State(startPosition, goalPosition, this.getEnvironment().getGlobe())));
		this.setGoal(this.addState(new State(goalPosition, goalPosition, this.getEnvironment().getGlobe())));
		
		// Sets the state as the start
		this.setState(this.getStart());
		
		// Sets the "done" boolean to false
		this.setDone(false);
		
		//TODO: adicionar obstaculos e treinar para policies diferentes -> tem que ser adicionados tambem na definicao do estado e na comparacao
		
	}
	
	
	/** Determines the next state based on the chosen action
	 * 
	 * @param the chosen action -> maybe nem e preciso posso so guardar na variavel action, mas tenho que editar outras funcoes
	 * 
	 * @return the next state
	 */
	public void step(int action) {
		
		// Determines next state based on action
		// It is considered that one time step corresponds to 1s
		Vec4 movVector;
		if (action == 0) {
			// If the action index is 0, the action corresponds to just flying in the direction of the goal
			movVector = this.getState().getRelativeVector().normalize3().multiply3(this.getAircraft().getCapabilities().getCruiseSpeed());
		} else {
			movVector = listOfActions.get(action).multiply3(this.getAircraft().getCapabilities().getCruiseSpeed());
		}
		Vec4 relativeVector = this.getState().getRelativeVector().subtract3(movVector);
		State nextState = new State(relativeVector);
		this.addState(nextState);
		
		// Checks if it has reached goal
		if (this.getNextState().getDistanceToGoal() <= this.goalThreshold)
			this.setDone(true);
		
		// Calculates reward
		this.calculateReward();
	}
	
	
	
	/** Reacts to the current state, updating the memory and choosing the next action
	 * 
	 * @param the current state
	 * @return the chosen action
	 */
	public int react(int state) {
		int action;
		try (NDManager submanager = manager.newSubManager()) {
			
			if(!isTrained()) {
				memory.setState(state);
				
				if (memory.getSize() > batchSize)
					updateModel(submanager);
			}
			
			action = chooseAction(submanager, state);
			
			if(!isTrained())
				memory.setAction(action);
			
		} catch (TranslateException e) {
			throw new IllegalStateException(e);
		}
		return action;
	}
	
	
	/** Updates the memory with the received reward
	 * 
	 * @param the reward
	 * @param indicates if it has reached goal or not
	 */
	public void collect(double reward, boolean done) {
		if(!isTrained())
			memory.setReward(reward, done);
	}
	
	
	/** Resets the DQN agent before training
	 */
	public void resetAgent() {
		optimizer = Optimizer.adam().optLearningRateTracker(Tracker.fixed(learningRate)).build();
		
		if (manager != null) {
			manager.close();
		}
		
		manager = NDManager.newBaseManager();
		policyNet = NetworkModel.newModel(manager, dimStateSpace, hiddenSize, numOfActions);
		targetNet = NetworkModel.newModel(manager, dimStateSpace, hiddenSize, numOfActions);
		
		policyPredictor = policyNet.newPredictor(new NoopTranslator());
		syncNetworks();
	}
	
	
	/** 
	 * Copies the weights from the main network to the target network
	 */
	public void syncNetworks() {
		for (Pair<String, Parameter> params : policyNet.getBlock().getParameters()) {
			targetNet.getBlock().getParameters().get(params.getKey()).setArray(params.getValue().getArray().duplicate());
		}
		targetPredictor = targetNet.newPredictor(new NoopTranslator());
	}
	
	
	/** 
	 * Performs the gradient update through backpropagation and, each "syncNetInterval" iterations, it syncs the policy 
	 * and target networks and updates the epsilon value for the epsilon greedy policy
	 * 
	 * @param loss the loss
	 */
	public void gradientUpdate(NDArray loss) {
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
	 * Chooses the action to perform using the epsilon greedy policy
	 * 
	 * @param manager the memory space manager
	 * @param state the current state
	 * 
	 * @return the chosen action
	 */
	protected int chooseAction(NDManager manager, int state) throws TranslateException {
		// Gets the predicted Q-values from the target network
		NDArray qValues = targetPredictor.predict(new NDList(manager.create(state))).singletonOrThrow();
		// Chooses the action using the epsilon greedy policy and the predicted Q-values
		return ActionSampler.epsilonGreedy(qValues, rand, Math.max(MIN_EXPLORE_RATE, epsilon));
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
		
		gradientUpdate(loss);
	}
	
	
	
	/**
	 * Calculates the reward based on the next state
	 */
	public void calculateReward() {
		
		// If the goal has been reached
		if (done) {
			this.setReward(1000);
			return;
		}
		
		// If the cost is exceeded
		
		// Otherwise: -10 for step + 10 * distance difference
		this.setReward(-10 + 10 * (this.getState().getDistanceToGoal() - this.getNextState().getDistanceToGoal()));
		
		

	}
	
	
	/**
	 * Runs the training to learn the Q-values, for a single part trajectory
	 */
	public void train() {
		
		int episode = 0;
		
		// For each episode
		while (episode < 5000) {
			// Reset environment 
			episode++;
			resetEnvironment();
		
			// For each time step
			while (!this.isDone()) {
				// Select action
				this.setAction(react(this.getState().getId()));
				// Execute action and get next state and reward
				step(this.getAction());
				// Stores the reward in memory
				collect(this.getReward(), this.isDone());
			}
		}
	}
	
	/**
	 * Computes a plan according to the trained Q-Table.
	 */
	protected void compute() {
		
		// Go waypoint by waypoint, find the corresponding state and choose the best action based on the predicted
		// value from the target network

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
		this.resetAgent();
		this.train();
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
		return null;
		
	}
	
	/**
	 * Indicates whether or not this forward A* planner supports a specified
	 * environment.
	 * 
	 * @param environment the environment
	 * 
	 * @return true if the environment is a planning grid or roadmap,
	 *         false otherwise
	 *         
	 * @see PlanningGrid
	 * @see PlanningRoadmap
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

