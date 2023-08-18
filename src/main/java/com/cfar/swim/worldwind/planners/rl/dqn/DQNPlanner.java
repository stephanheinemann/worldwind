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
	private final int dimStateSpace;
	
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
	
	
	/** Constructs a planner trained by a Deep Q-Network for a specified aircraft and
	 * environment using default local cost and risk policies.
	 * 
	 * @param the aircraft
	 * @param the environment
	 */
	public DQNPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		// Calculate based on environment
		this.dimStateSpace = 1000;
		this.numOfActions = Action.values().length;
	}
	
	@Override
	public String getId() {
		// TODO Auto-generated method stub
		return Specification.PLANNER_DQN_ID;
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
			
			action = getAction(submanager, state);
			
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
	public void collect(float reward, boolean done) {
		if(!isTrained())
			memory.setReward(reward, done);
	}
	
	
	/** Resets the DQN agent to train
	 */
	public void reset() {
		optimizer = Optimizer.adam().optLearningRateTracker(Tracker.fixed(learningRate)).build();
		
		if (manager != null) {
			manager.close();
		}
		
		manager = NDManager.newBaseManager();
		policyNet = ScoreModel.newModel(manager, dimStateSpace, hiddenSize, numOfActions);
		targetNet = ScoreModel.newModel(manager, dimStateSpace, hiddenSize, numOfActions);
		
		policyPredictor = policyNet.newPredictor(new NoopTranslator());
		syncNetworks();
	}
	
	
	/** Copies the weights from the main network to the target network
	 */
	public void syncNetworks() {
		for (Pair<String, Parameter> params : policyNet.getBlock().getParameters()) {
			targetNet.getBlock().getParameters().get(params.getKey()).setArray(params.getValue().getArray().duplicate());
		}
		targetPredictor = targetNet.newPredictor(new NoopTranslator());
	}
	
	
	/** Performs the gradient update
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
	
	
	/** Chooses the action to perform
	 * 
	 * @param the manager
	 * @param the current state
	 * 
	 * @return the chosen action
	 */
	protected int getAction(NDManager manager, int state) throws TranslateException {
		NDArray score = targetPredictor.predict(new NDList(manager.create(state))).singletonOrThrow();
		return ActionSampler.epsilonGreedy(score, rand, Math.max(MIN_EXPLORE_RATE, epsilon));
	}
	
	
	/** Performs the gradient update
	 */
	protected void updateModel(NDManager manager) throws TranslateException {
		MemoryBatch batch = memory.sampleBatch(batchSize, manager);
		
		NDArray policy = policyPredictor.predict(new NDList(batch.getStates())).singletonOrThrow();
		NDArray target = targetPredictor.predict(new NDList(batch.getNextStates())).singletonOrThrow().duplicate();
		NDArray expectedReturns = Helper.gather(policy, batch.getActions().toIntArray());
		NDArray nextReturns = batch.getRewards().add(target.max(new int[] { 1 }).mul(batch.getDones().logicalNot()).mul(gamma));
		
		NDArray loss = lossFunc.evaluate(new NDList(expectedReturns), new NDList(nextReturns));
		
		gradientUpdate(loss);
	}
	
	
	
	/**
	 * Calculates the reward based on the next state 
	 */
	public void calculateReward() {

	}
	
	
	/**
	 * Runs the training to learn the Q-values, for a single part trajectory
	 */
	public void train() {
		// Initialize replay memory D
		
		// Initialize action-value function Q
		
		// For each episode
			// Reset environment 
		
			// For each time step
				// Select action, execute it and observe next state and reward
		
				// Store transition in D
		
				// Sample a minibatch of transitions from D
		
				// For each tuple j
					// Calculate target
		
				// Do a gradient descent step
		
				// Update the target network each N steps
	}
	
	/**
	 * Computes a plan according to the trained Q-Table.
	 */
	protected void compute() {

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

