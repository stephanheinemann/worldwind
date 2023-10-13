package com.cfar.swim.worldwind.planners.rl.dqn;

import java.util.ArrayList;


import java.util.Arrays;

import static org.assertj.core.api.Assertions.assertThatIllegalStateException;

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
	private static final float INITIAL_EPSILON = 1.0f;
	
	/** number of episodes for the global training */
	private static final int NUM_GLOBAL_EPS = 3000;
	
	/** maximum number of steps per episode */
	private static final int MAX_STEPS = 200;
	
	/** random number */
	private final Random rand = new Random();
	
	/** replay memory */
	private final Memory memory = new Memory(2500);
	/** maps states to their IDs */
	//protected Map<float[], StateNoCosts> stateMap = new TreeMap<>(new FloatArrayComparator());
	
	/** dimension of a space ID (input of the neural network) */
	private int dimOfSpace;
	
	/** the list of available actions in each state */
	private final ArrayList<Vec4> listOfActions;
	
	/** the number of available actions in each state */
	private final int numOfActions;
	
	/** the number of hidden units (neurons) in the neural network */
	private final int[] hiddenSize = {64};
	
	/** learning rate used by the optimizer during training */
	private final float learningRate = 0.1f;
	
	/** the size of the mini-batch of transitions used for training */
	protected final int batchSize = 32;
	
	/** the number of iterations between each train of the policy network */
	protected final int trainNetInterval = 1;
	
	/** the number of iterations between each sync of the target and policy networks */
	protected final int syncNetInterval = 200;
	
	/** gamma factor for the Bellman equation */
	protected final float gamma = 0.95f;
	
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
	
	/** saves the most recent waypoint added to the trajectory */
	private RLWaypoint newestWaypoint = null;
	
	/** stores the plan's ETD */
	private ZonedDateTime etd;
	
	
	//DEBUG
	public List<Double> rewards0 = new ArrayList<>();
	public List<Double> rewards1 = new ArrayList<>();
	public List<Double> rewards2 = new ArrayList<>();
	public List<Double> rewards3 = new ArrayList<>();
	public List<Double> rewards4 = new ArrayList<>();
	public List<Double> rewards5 = new ArrayList<>();
	public List<Double> rewards6 = new ArrayList<>();
	
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
	 * @param the reward
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
		
		double decay = 0;
		int episode = 0;
		int step;
		double score;
		
		// For each episode
		while (episode < NUM_GLOBAL_EPS) {
			// Reset environment 
			episode++;
			step = 1;
			score = 0;
			
			resetEnvironment();
			
			this.setState(this.getStart());
			this.setDone(false);
			this.setFailure(false);
			
			// For each time step, until it reaches goal or failure
			while (!this.isDone() && !this.failed()) {
				// Saves state in memory; Updates the network; Selects the next action and saves it in memory
				this.react();
				// Execute action and get next state and reward; Checks if the goal has been reached
				this.step();
				//System.out.println("State: " + this.getState().getDistanceToGoal() + " Action: " + this.getAction() + " Reward:" + this.getReward());
				// Reaching maximum number of steps counts as failure
				if (step >= MAX_STEPS)
					this.setFailure(true);
				// Stores the reward and the "done" boolean in memory
				memory.setReward(0.1*this.getReward(), this.isDone(), this.failed());
				score += this.getReward();
				// Sets the state as the next state
				this.setState(this.getNextState());
				step++;
			}
			// Update epsilon for next episode
			decay = Math.exp(-episode * 1.0 / NUM_GLOBAL_EPS);
			epsilon = (float) (MIN_EXPLORE_RATE + (INITIAL_EPSILON - MIN_EXPLORE_RATE) * decay);
			
			
			if (this.isDone()) {
				System.out.println("Episode " + episode + " had score " + score + " and reached GOAL after " + (step-1) + " steps");
			} else if (this.failed()) {
				if (step >= MAX_STEPS ) {
					System.out.println("Episode " + episode + " had score " + score + "  but did too many steps");
				} else {
					System.out.println("Episode " + episode + " had score " + score + "  but left environment");
				}
			} 	
		}
	}
	
	
//	/**
//	 * Runs the training of the Deep Q-Network for a specific start and goal
//	 */
//	protected void trainLocal(int numEpisodes, Position origin, Position destination) {
//		
//		System.out.println("TRAINING LOCAL");
//		
//		double decay = 0;
//		int episode = 0;
//		int step;
//		double score;
//		
//		// Creates the start and goal states
//		this.setStart(new StateNoCosts(origin, destination, this.getPlanningContinuum()));
//		this.setGoal(new StateNoCosts(destination, destination, this.getPlanningContinuum()));
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
//			decay = Math.min(((double) episode / NUM_GLOBAL_EPS), 1.0);
//			epsilon = (float) (INITIAL_EPSILON - (INITIAL_EPSILON - MIN_EXPLORE_RATE) * decay);
//			
//			if (this.isDone()) {
//				System.out.println("Episode " + episode + " had score " + score + " and reached GOAL after " + step + " steps");
//			} else if (this.failed()) {
//				if (step >= MAX_STEPS ) {
//					System.out.println("Episode " + episode + " had score " + score + "  but did too many steps");
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
		Position startPosition = this.getPlanningContinuum().sampleRandomUniformPosition();
		Position goalPosition = startPosition;
		// Makes sure the start and goal are different
		while (goalPosition == startPosition)
			goalPosition = this.getPlanningContinuum().sampleRandomUniformPosition();
		
		// Creates the start and goal states
		this.setStart(new StateNoCosts(startPosition, goalPosition, this.getPlanningContinuum()));
		this.setGoal(new StateNoCosts(goalPosition, goalPosition, this.getPlanningContinuum()));
		
		//TODO: adicionar obstaculos e treinar para policies diferentes -> tem que ser adicionados tambem na definicao do estado e na comparacao
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
				
		// Gets the predicted Q-values from the target network
		NDArray qValues = policyPredictor.predict(new NDList(manager.create(state))).singletonOrThrow();
		// Chooses the action using the epsilon greedy policy and the predicted Q-values
		return ActionSampler.epsilonGreedy(qValues, rand, Math.max(MIN_EXPLORE_RATE, epsilon));
		
	}
	

	/** 
	 * Determines the next state based on the chosen action
	 */
	protected void step() {
		
		int action = this.getAction();
		//double stepSize = 2 * this.getAircraft().getCapabilities().getCruiseSpeed();
		double stepSize = this.getStart().getDistanceToGoal() / 4;
		
		// Determines the movement from the current state to the next based on action
		Vec4 movVector;
		if (action == 0) {
			// If the action index is 0, the action corresponds to just flying in the direction of the goal
			movVector = this.getState().getNormalizedRelativeVector().multiply3(stepSize);
		} else {
			movVector = listOfActions.get(action).multiply3(stepSize);
		}
		
		// Computes the next state's position
		Vec4 nextStateBoxPoint = this.getState().getBoxStatePoint().add3(movVector);
		Vec4 nextStatePoint = this.getPlanningContinuum().transformBoxOriginToModel(nextStateBoxPoint);
		Position nextStatePosition = this.getEnvironment().getGlobe().computePositionFromPoint(nextStatePoint);
		
		// If the next state is outside the environment, stays in place, and fails
		if (!this.getEnvironment().contains(nextStatePosition)) {
			nextStatePosition = this.getState().getPosition();
			this.setFailure(true);
		}
		
		// Creates the next state
		this.setNextState(new StateNoCosts(nextStatePosition, this.getGoal().getPosition(), this.getPlanningContinuum()));
		
		// Checks if it has reached goal
		if (this.getNextState().getDistanceToGoal() <= stepSize)
			this.setDone(true);
		
		// Calculates reward
		this.calculateReward();
		
		// DEBUG
		if(action == 0)
			rewards0.add(this.getReward());
		if(action == 1)
			rewards1.add(this.getReward());
		if(action == 2)
			rewards2.add(this.getReward());
		if(action == 3)
			rewards3.add(this.getReward());
		if(action == 4)
			rewards4.add(this.getReward());
		if(action == 5)
			rewards5.add(this.getReward());
		if(action == 6)
			rewards6.add(this.getReward());
		
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
		
		// If it failed
		if (this.failed()){
			this.setReward(-50);
			return;
		}
		
		// If the cost is exceeded
		
		// Otherwise: step penalty +  distance to goal penalty/reward 
		reward += -5;
		reward += this.getState().getDistanceToGoal() - this.getNextState().getDistanceToGoal();
		
		this.setReward(reward);
		//System.out.println("Reward is " + reward);
	
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
		
		float[] ex = expectedReturns.toFloatArray();
		Number[] ne = nextReturns.toArray();
		// Calculates the loss (mean squared error)
		NDArray loss = lossFunc.evaluate(new NDList(expectedReturns), new NDList(nextReturns));
		Number[] l = loss.toArray();
		
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
			for (Pair<String, Parameter> params : policyNet.getBlock().getParameters()) {
				params.getValue().getArray().setRequiresGradient(true);
			}
			collector.backward(loss);
			for (Pair<String, Parameter> params : policyNet.getBlock().getParameters()) {
				NDArray paramsArr = params.getValue().getArray();
				optimizer.update(params.getKey(), paramsArr, paramsArr.getGradient());
				
				float[] gradient = paramsArr.getGradient().toFloatArray();
				float[] p = paramsArr.toFloatArray();
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
		
//		this.setNewestWaypoint(null);
		this.setEtd(etd);
		
		// Creates the start waypoint 
		this.setNewestWaypoint(this.createWaypoint(origin));
		this.getNewestWaypoint().setEto(etd);
		this.getNewestWaypoint().setPoi(true);
		
		// Creates the start and goal states and adds them to the state map
		this.setStart(new StateNoCosts(origin, destination, this.getPlanningContinuum()));
		this.setGoal(new StateNoCosts(destination, destination, this.getPlanningContinuum()));
		
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
		
		// Adds the start waypoint to the trajectory if this is the first part
		if (this.getWaypoints().isEmpty()) {
			this.getWaypoints().addLast(this.getNewestWaypoint());
		}
		
		// DEBUG
		double sum = 0;
		for (Double n : rewards0)
			sum += n;
		System.out.println("Average rewards of action 0:" + (sum/rewards0.size()));
		sum = 0;
		for (Double n : rewards1)
			sum += n;
		System.out.println("Average rewards of action 1:" + (sum/rewards1.size()));
		sum = 0;
		for (Double n : rewards2)
			sum += n;
		System.out.println("Average rewards of action 2:" + (sum/rewards2.size()));
		sum = 0;
		for (Double n : rewards3)
			sum += n;
		System.out.println("Average rewards of action 3:" + (sum/rewards3.size()));
		sum = 0;
		for (Double n : rewards4)
			sum += n;
		System.out.println("Average rewards of action 4:" + (sum/rewards4.size()));
		sum = 0;
		for (Double n : rewards5)
			sum += n;
		System.out.println("Average rewards of action 5:" + (sum/rewards5.size()));
		sum = 0;
		for (Double n : rewards6)
			sum += n;
		System.out.println("Average rewards of action 6:" + (sum/rewards6.size()));
		
		int step = 0;
		
		// Until the goal is reached
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
			
			System.out.println("State: (" + this.getState().getId()[0] + "; " + this.getState().getId()[1] + "; " +
					this.getState().getId()[2] + "; " + this.getState().getId()[3] + "; " + this.getState().getId()[4] + "; " +
					this.getState().getId()[5] + "; " + this.getState().getId()[6] + "; " + this.getState().getId()[7] + "; " +
					this.getState().getId()[8] + "; " + this.getState().getId()[9] + ") " + ": Action:" + this.getAction());
			
			// Execute action and get next state and reward; Checks if the goal has been reached
			this.step();
			
			// Adds the next waypoint to the trajectory
			// Creates waypoint by subtracting the next state's relative vector to the goal
			this.getWaypoints().addLast(this.createWaypoint(this.getNextState().getPosition()));
			
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
		RLWaypoint goalWaypoint = this.createWaypoint(this.getGoal().getPosition());
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
		
//		this.trainLocal(3000, origin, destination);
		
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

