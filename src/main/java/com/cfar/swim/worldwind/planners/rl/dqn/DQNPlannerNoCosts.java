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

import org.apache.commons.collections4.queue.CircularFifoQueue;

import java.nio.Buffer;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.aircraft.Capabilities;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.environments.PlanningContinuum;
import com.cfar.swim.worldwind.environments.PlanningGrid;
import com.cfar.swim.worldwind.environments.PlanningRoadmap;
import com.cfar.swim.worldwind.environments.RLEnvironment;
import com.cfar.swim.worldwind.geom.precision.PrecisionPosition;
import com.cfar.swim.worldwind.planners.AbstractPlanner;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.planners.rl.Plot;
import com.cfar.swim.worldwind.planners.rl.State;
import com.cfar.swim.worldwind.planners.rl.ActionSampler;
import com.cfar.swim.worldwind.planners.rl.Helper;
import com.cfar.swim.worldwind.planners.rl.Memory;
import com.cfar.swim.worldwind.planners.rl.MemoryBatch;
import com.cfar.swim.worldwind.planning.RiskPolicy;
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
import ai.djl.ndarray.types.DataType;
import ai.djl.nn.Parameter;
import ai.djl.training.GradientCollector;
import ai.djl.training.Trainer;
import ai.djl.training.loss.L2Loss;
import ai.djl.training.loss.Loss;
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
	
//	/** the rate at which the exploration rate decays over time */
//	private static final float DECAY_EXPLORE_RATE = 0.99f;
	
	/** the initial value of epsilon */
	private static final float INITIAL_EPSILON = 1.0f;
	
	/** number of episodes for the global training */
	private static final int NUM_GLOBAL_EPS = 10;
	
	/** maximum number of steps per episode */
	private static final int MAX_STEPS = 200;
	
	/** in how many steps to divide from start to goal */
	private static final int STEP_DIVISION = 20;
	
	/** random number */
	private final Random rand = new Random();
	
	/** replay memory */
	private final Memory memory = new Memory(4096);
	
	/** dimension of a space ID (input of the neural network) */
	private int dimOfSpace;
	
//	/** the list of available actions in each state */
//	private final ArrayList<Vec4> listOfActions;
	
	/** the number of available actions in each state */
	private final int numOfActions = 7;
	
	/** the number of hidden units (neurons) in the neural network */
	private final int[] hiddenSize = {64, 128, 128, 64};
	
	/** learning rate used by the optimizer during training */
	private final float learningRate = 0.08f;
	
	/** the size of the mini-batch of transitions used for training */
	protected final int batchSize = 32;
	
	/** the number of iterations between each train of the policy network */
	protected final int trainNetInterval = 1;
	
	/** the number of iterations between each sync of the target and policy networks */
	protected final int syncNetInterval = 200;
	
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
	
	/** the current epsilon value to implement decaying e-greedy */
	private float epsilon = INITIAL_EPSILON; 
	
	/** the loss function */
	private final Loss lossFunc = Loss.l2Loss();
	
	/** the planner's start state */
	private StateNoCosts start = null;
	
	/** the planner's goal position */
	private Position goalPosition = null;
	
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
	
	/** stores the step size of the planner */
	private double stepSize = 0.0;
	
	/** saves the most recent waypoint added to the trajectory */
	private Waypoint newestWaypoint = null;
	
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
	
	/** Stores episode results */
	public CircularFifoQueue<Integer> episodeResults = new CircularFifoQueue<Integer>(1000);
	
	/** Constructs a planner trained by a Deep Q-Network for a specified aircraft and
	 * environment using default local cost and risk policies.
	 * 
	 * @param the aircraft
	 * @param the environment
	 */
	public DQNPlannerNoCosts(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		this.dimOfSpace = StateNoCosts.ID_SIZE;
//		this.listOfActions = Helper.listOfActions();
//		this.numOfActions = listOfActions.size();
		
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
	 * Gets the goal position of this planner.
	 * 
	 * @return the goal position of this planner
	 */
	protected Position getGoalPosition() {
		return this.goalPosition;
	}
	
	/**
	 * Sets the goal position of this planner.
	 * 
	 * @param goal the goal position of this planner
	 */
	protected void setGoalPosition(Position goal) {
		this.goalPosition = goal;
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
	 * Gets the newest RL waypoint added to the trajectory.
	 * 
	 * @return the newest RL waypoint added to the trajectory
	 */
	protected Waypoint getNewestWaypoint() {
		return this.newestWaypoint;
	}
	
	/**
	 * Sets the newest RL waypoint added to the trajectory.
	 * 
	 * @param newestWaypoint the newest RL waypoint added to the trajectory
	 */
	protected void setNewestWaypoint(Waypoint newestWaypoint) {
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
		//Sets require gradient to true for the policy network's parameters
		for (Pair<String, Parameter> params : policyNet.getBlock().getParameters()) {
			params.getValue().getArray().setRequiresGradient(true);
		}
		targetNet = NetworkModel.newModel(manager, dimOfSpace, hiddenSize, numOfActions);
		
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
		double score;
		double agentPerformance = 0;
		double successfulEpisodes = 0;
		
		// For each episode
		while (episode < NUM_GLOBAL_EPS) {
			// Reset environment 
			episode++;
			step = 1;
			score = 0;
			
			resetEnvironment();
			
			// For each time step, until it reaches goal or failure
			while (!this.isDone() && !this.failed()) {
				// Saves state in memory; Updates the network; Selects the next action and saves it in memory
				this.react();
				// Execute action and get next state and reward; Checks if the goal has been reached
				this.step();
				//System.out.println("State: " + this.getState().getDistanceToGoal() + " Action: " + this.getAction() + " Reward:" + this.getReward());
//				System.out.println("State: (" + this.getState().getId()[0] + "; " + this.getState().getId()[1] + "; " +
//						this.getState().getId()[2] + "; " + this.getState().getId()[3] + "; " + this.getState().getId()[4] + "; " +
//						this.getState().getId()[5] + "; " + this.getState().getId()[6] + "; " + this.getState().getId()[7] + "; " +
//						this.getState().getId()[8] + "; " + this.getState().getId()[9] + "; " + this.getState().getId()[10] + ") " + ": Action:" + this.getAction());
				// Reaching maximum number of steps counts as failure
				if (step >= MAX_STEPS)
					this.setFailure(true);
				// Stores the reward and the "done" boolean in memory
				memory.setReward(0.01*this.getReward(), this.isDone(), this.failed());
				score += this.getReward();
				// Sets the state as the next state
				this.setState(this.getNextState());
				step++;
			}
			// Update epsilon for next episode
			decay = Math.exp(-episode * 1.0 / (NUM_GLOBAL_EPS * 0.9));
			epsilon = (float) (MIN_EXPLORE_RATE + (INITIAL_EPSILON - MIN_EXPLORE_RATE) * decay);
			
			if(this.isDone()) {
				episodeResults.add(1);
			} else {
				episodeResults.add(0);
			}
			double sum = (episodeResults.stream().mapToInt(Integer::intValue).sum());
			double size = episodeResults.size();
			agentPerformance =  sum / size;
			double e = (episodeResults.stream().mapToInt(Integer::intValue).sum()) / (episodeResults.size());
//			agentPerformance = successfulEpisodes / episode;
//			episodeResults[episode] = this.isDone();
//			successfulEpisodes = 0;
//			if (episode > 1000) {
//				for (int i=0; i<1000; i++) {
//					if (episodeResults[episode - 1000 + i])
//						successfulEpisodes++;
//				}
//				agentPerformance = successfulEpisodes / 1000;
//			} else {
//				for (int i=0; i<episode; i++) {
//					if (episodeResults[i])
//						successfulEpisodes++;
//				}
//				agentPerformance = successfulEpisodes / (episode+1);
//			}
			
			if (this.isDone()) {
				System.out.println("Episode " + episode + " -----------------------> GOAL");
			} else if (this.failed()) {
				if (step >= MAX_STEPS ) {
					System.out.println("Episode " + episode + " did too many steps");
				} else {
					System.out.println("Episode " + episode + " left environment");
				}
			}
			System.out.printf("Performance %,.3f %n", agentPerformance);
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
		
		// Sets the step size
		Vec4 startBoxPoint = this.getPlanningContinuum().transformModelToBoxOrigin(this.getPlanningContinuum().getGlobe()
				.computePointFromPosition(startPosition));
		Vec4 goalBoxPoint = this.getPlanningContinuum().transformModelToBoxOrigin(this.getPlanningContinuum().getGlobe()
				.computePointFromPosition(goalPosition));
		double distance = startBoxPoint.subtract3(goalBoxPoint).getLength3();
		this.setStepSize(distance / STEP_DIVISION);
		
		// Creates the start and goal states
		this.setStart(new StateNoCosts(startPosition, goalPosition, this.getPlanningContinuum(), null, this.getStepSize()));
		this.setGoalPosition(goalPosition);
		
		this.setState(this.getStart());
		this.setDone(false);
		this.setFailure(false);
		
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
				
		// Gets the predicted Q-values from the main network
		NDArray qValues = policyPredictor.predict(new NDList(manager.create(state))).singletonOrThrow();
		// Chooses the action using the epsilon greedy policy and the predicted Q-values
		return ActionSampler.epsilonGreedy(qValues, rand, Math.max(MIN_EXPLORE_RATE, epsilon));
		
	}
	
	/** 
	 * Calculates the new movVector depending on the chosen action
	 * 
	 * @param the original move vector
	 * @param the action
	 * 
	 * @return the new movVector (normalized)
	 */
	public Vec4 getNewMoveVector (Vec4 originalVector, Vec4 relativeGoal, int action) {
		
		// TODO: create movements considering aircraft capabilities and not for 45 and 60 fixed angles

		double x = originalVector.x;
		double y = originalVector.y;
		double z = originalVector.z;
		double newX = x;
		double newY = y;
		double newZ = z;
		
		switch(action) {
			// Go in direction of goal 
			case 0: 
				newX = relativeGoal.x;
				newY = relativeGoal.y;
				newZ = relativeGoal.z;
				break;
			// Turn right 45 degrees
			case 1: 
				newX = x * Math.cos(Math.toRadians(45)) + y * Math.sin(Math.toRadians(45));
				newY = -x * Math.sin(Math.toRadians(45)) + y * Math.cos(Math.toRadians(45));
//				newX = 1;
//				newY = 0;
//				newZ = 0;
				break;
			// Turn right 60 degrees
			case 2: 
				newX = x * Math.cos(Math.toRadians(60)) + y * Math.sin(Math.toRadians(60));
				newY = -x * Math.sin(Math.toRadians(60)) + y * Math.cos(Math.toRadians(60));
//				newX = -1;
//				newY = 0;
//				newZ = 0;
				break;
			// Turn left 45 degrees
			case 3: 
				newX = x * Math.cos(Math.toRadians(45)) - y * Math.sin(Math.toRadians(45));
				newY = x * Math.sin(Math.toRadians(45)) + y * Math.cos(Math.toRadians(45));
//				newX = 0;
//				newY = 1;
//				newZ = 0;
				break;
			// Turn left 60 degrees
			case 4: 
				newX = x * Math.cos(Math.toRadians(60)) - y * Math.sin(Math.toRadians(60));
				newY = x * Math.sin(Math.toRadians(60)) + y * Math.cos(Math.toRadians(60));
//				newX = 0;
//				newY = -1;
//				newZ = 0;
				break;
			 // Climb 45 degrees
			case 5: 
				newZ = z + Math.tan(Math.toRadians(45));
//				newX = 0;
//				newY = 0;
//				newZ = 1;
				break;
			// Descend 45 degrees
			case 6:
				newZ = z - Math.tan(Math.toRadians(45));
//				newX = 0;
//				newY = 0;
//				newZ = -1;
				break;
			default:
		}
		
		Vec4 newVector = new Vec4(newX, newY, newZ);
		
		return newVector.normalize3();
	}
	

	/** 
	 * Determines the next state based on the chosen action
	 */
	protected void step() {
		
		int action = this.getAction();
		//double stepSize = 2 * this.getAircraft().getCapabilities().getCruiseSpeed();
		//double stepSize = this.getStart().getDistanceToGoal() / 10;
		
		Vec4 movVector = this.getNewMoveVector(this.getState().getMovVector(), this.getState().getNormalizedRelativeGoal().getNegative3(), action);
		
//		if (action == 0) {
//			// If the action index is 0, the action corresponds to just flying in the direction of the goal
//			movVector = this.getState().getNormalizedRelativeVector().getNegative3().multiply3(stepSize);
//		} else {
//			movVector = listOfActions.get(action).multiply3(stepSize);
//		}
		
		// Computes the next state's position
		Vec4 boxNextStatePoint = this.getState().getBoxStatePoint().add3(movVector.multiply3(this.getStepSize()));
		Vec4 nextStatePoint = this.getPlanningContinuum().transformBoxOriginToModel(boxNextStatePoint);
		Position nextStatePosition = this.getEnvironment().getGlobe().computePositionFromPoint(nextStatePoint);
				
		// If the next state is outside the environment, stays in place, and fails
		if (!this.getEnvironment().contains(nextStatePosition)) {
			nextStatePosition = this.getState().getPosition();
			this.setFailure(true);
		}
		
		// Creates the next state
		this.setNextState(new StateNoCosts(nextStatePosition, this.getGoalPosition(), this.getPlanningContinuum(), movVector, this.getStepSize()));
		
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
		
//		// If it gets too close to environment limits
//		for (int i=0; i<6; i++) {
//			if (this.getNextState().getDistanceToEnv()[i] < this.getNextState().getStepSize()) {
//				reward += -10;
//				break;
//			}
//		}
		
		// Otherwise: step penalty +  distance to goal penalty/reward 
		reward += -5;
		reward += this.getState().getDistanceToGoal() - this.getNextState().getDistanceToGoal();
		
		this.setReward(reward);
		//System.out.println("Reward is " + reward);
	
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
		
//		this.setNewestWaypoint(null);
		this.setEtd(etd);
		
		// Creates the start waypoint 
		this.setNewestWaypoint(this.createWaypoint(origin));
		this.getNewestWaypoint().setEto(etd);
		this.getNewestWaypoint().setPoi(true);
		
		// Sets the step size
		Vec4 startBoxPoint = this.getPlanningContinuum().transformModelToBoxOrigin(this.getPlanningContinuum().getGlobe()
				.computePointFromPosition(origin));
		Vec4 goalBoxPoint = this.getPlanningContinuum().transformModelToBoxOrigin(this.getPlanningContinuum().getGlobe()
				.computePointFromPosition(destination));
		double distance = startBoxPoint.subtract3(goalBoxPoint).getLength3();
		this.setStepSize(distance / STEP_DIVISION);
		
		// Creates the start and goal states 
		this.setStart(new StateNoCosts(origin, destination, this.getPlanningContinuum(), null, this.getStepSize()));
		this.setGoalPosition(destination);
		
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
		
		int retry = 0;
		
		// DEBUG
		double sum = 0;
//		for (Double n : rewards0)
//			sum += n;
//		System.out.println("Average rewards of action 0:" + (sum/rewards0.size()));
//		sum = 0;
//		for (Double n : rewards1)
//			sum += n;
//		System.out.println("Average rewards of action 1:" + (sum/rewards1.size()));
//		sum = 0;
//		for (Double n : rewards2)
//			sum += n;
//		System.out.println("Average rewards of action 2:" + (sum/rewards2.size()));
//		sum = 0;
//		for (Double n : rewards3)
//			sum += n;
//		System.out.println("Average rewards of action 3:" + (sum/rewards3.size()));
//		sum = 0;
//		for (Double n : rewards4)
//			sum += n;
//		System.out.println("Average rewards of action 4:" + (sum/rewards4.size()));
//		sum = 0;
//		for (Double n : rewards5)
//			sum += n;
//		System.out.println("Average rewards of action 5:" + (sum/rewards5.size()));
//		sum = 0;
//		for (Double n : rewards6)
//			sum += n;
//		System.out.println("Average rewards of action 6:" + (sum/rewards6.size()));
		
		int step = 0;
		
		// Until the goal is reached
		while(!this.isDone() && !this.failed() && step < MAX_STEPS) {
			
			// Chooses the action
			try (NDManager submanager = manager.newSubManager()) {
				
				NDArray qValues = targetPredictor.predict(new NDList(submanager.create(this.getState().getId()))).singletonOrThrow();
				// DEBUG
//				System.out.print("Q VALUES: ");
//				for (int i=0; i<qValues.size(); i++) {
//					System.out.print(qValues.getFloat(i) + "; ");
//				}
//				System.out.println();
				this.setAction(ActionSampler.greedy(qValues));
				
			} catch (TranslateException e) {
				throw new IllegalStateException(e);
			}
			
//			System.out.println("State: (" + this.getState().getId()[0] + "; " + this.getState().getId()[1] + "; " +
//					this.getState().getId()[2] + "; " + this.getState().getId()[3] + ")" + ": Action:" + this.getAction());
			
			// Execute action and get next state and reward; Checks if the goal has been reached
			this.step();
			
			System.out.println("State: (" + this.getState().getId()[0] + "; " + this.getState().getId()[1] + "; " +
					this.getState().getId()[2] + "; " + this.getState().getId()[3] + "; " + this.getState().getId()[4] + "; " +
					this.getState().getId()[5] + "; " + this.getState().getId()[6] + "; " + this.getState().getId()[7] + "; " +
					this.getState().getId()[8] + "; " + this.getState().getId()[9] + "; " + this.getState().getId()[10] + ") " + ": Action:" + this.getAction());
			
			// Adds the next waypoint to the trajectory
			this.getWaypoints().addLast(this.createWaypoint(this.getNextState().getPosition()));
			
			// Sets the state as the next state
			this.setState(this.getNextState());
			
			step++;
		}
		
		// If it finished without reaching the goal (because it reached the maximum number of steps
		// or failed) returns an empty trajectory
		if (!this.isDone()) {
			//this.clearWaypoints();
			return;
		}
		
		// Adds the goal to the trajectory if it is not already there
		Waypoint goalWaypoint = this.createWaypoint(this.getGoalPosition());
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
	protected Waypoint createWaypoint(Position position) {
		
		Waypoint wp = new Waypoint(position);
		
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
				
				if ((!this.hasWaypoints()) || !(this.getWaypoints().getLast().equals(new Waypoint(currentDestination)))) {
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

