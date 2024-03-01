package com.cfar.swim.worldwind.planners.rl.dqn;

import java.time.ZonedDateTime;



import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.LinkedList;
import java.io.*;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.environments.PlanningContinuum;
import com.cfar.swim.worldwind.environments.RLEnvironment;
import com.cfar.swim.worldwind.planners.AbstractPlanner;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.planners.rl.ActionSampler;
import com.cfar.swim.worldwind.planners.rl.Helper;
import com.cfar.swim.worldwind.planners.rl.Memory;
import com.cfar.swim.worldwind.planners.rl.MemoryBatch;
import com.cfar.swim.worldwind.planners.rl.NetworkModel;
import com.cfar.swim.worldwind.planners.rl.Snapshot;
import com.cfar.swim.worldwind.planners.rl.State;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.render.*;
import com.cfar.swim.worldwind.util.Identifiable;
import gov.nasa.worldwind.geom.Position;
import ai.djl.Model;
import ai.djl.engine.Engine;
import ai.djl.inference.Predictor;
import ai.djl.ndarray.*;
import ai.djl.nn.Parameter;
import ai.djl.training.GradientCollector;
import ai.djl.training.loss.Loss;
import ai.djl.training.optimizer.Optimizer;
import ai.djl.training.tracker.Tracker;
import ai.djl.translate.TranslateException;
import ai.djl.util.Pair;
import ai.djl.translate.NoopTranslator;

/**
* Realizes a deep reinforcement learning planner, using a Double Deep Q-Network, that plans a trajectory 
* of an aircraft in an environment considering a local cost and risk policy.
* 
* @author Rafaela Seguro
*
*/

public class DQNPlanner extends AbstractPlanner {
	
	/** the initial value of epsilon */
	private static final float INITIAL_EPSILON = 1.0f;
	
	/** the minimum value of epsilon during training */
	private static final float MIN_EPSILON = 0.01f; 
	
	/** number of total training episodes */
	private static final int NUM_GLOBAL_EPS = 10000;
	
	/** number of extra training episodes for when trajectory computation fails */
	private static final int NUM_EXTRA_EPS = 5;
	
	/** number of episodes over which epsilon decays */
	private static final int EPSILON_DECAY_EPS = 300;
	
	/** maximum number of steps per episode */
	private static final int MAX_STEPS = 500;
	
	/** random number */
	private final Random rand = new Random();
	
	/** replay memory */
	private final Memory memory = new Memory(4096);
	
	/** the number of hidden units (neurons) in the neural network */
	private final int[] hiddenSize = {128, 256, 128};
	
	/** learning rate used by the optimizer during training */
	private final float learningRate = 0.00025f;
	
	/** the size of the batch of transitions used for training */
	protected final int batchSize = 32;
	
	/** the number of iterations between each train of the main network */
	protected final int trainNetInterval = 1;
	
	/** the number of iterations between each sync of the target and main networks */
	protected final int syncNetInterval = 200;
	
	/** gamma factor for the Bellman equation */
	protected final float gamma = 0.99f;
	
	/** the optimizer used for updating the network parameters during training */
	private Optimizer optimizer;
	
	/** the main network, which predicts the Q-values */
	private Model mainNet;
	
	/** the target network, used to stabilize training in DQN */
	private Model targetNet;
	
	/** used for managing NDArrays within the class */
	protected NDManager manager;
	
	/** predictor for the main network */
	protected Predictor<NDList, NDList> mainPredictor;
	
	/** predictor for the target network */
	protected Predictor<NDList, NDList> targetPredictor;
	
	/** keeps track of the number of iterations the agent has experienced */
	private int iteration = 0;
	
	/** the current epsilon value to implement decaying e-greedy */
	private float epsilon = INITIAL_EPSILON; 
	
	/** the loss function */
	private final Loss lossFunc = Loss.l2Loss();
	
	/** saves the start waypoint */
	private Waypoint startWaypoint = null;
	
	/** stores the plan's ETD */
	private ZonedDateTime etd;
	
	/** list to store loss value over an episode*/
	List<Number> lossArray = new LinkedList<>();
	
	
	/** Constructs a planner trained by a Double Deep Q-Network for a specified aircraft and
	 * environment using default local cost and risk policies.
	 * 
	 * @param the aircraft
	 * @param the environment
	 */
	public DQNPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		this.etd = environment.getTime();
		
		resetAgent();
		train(0);
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
	 * Gets the RL environment of this DQN planner.
	 * 
	 * @return the RL environment of this DQN planner
	 * 
	 * @see AbstractPlanner#getEnvironment()
	 */
	public RLEnvironment getRLEnvironment() {
		return (RLEnvironment) super.getEnvironment();
	}
	
	/**
	 * Gets the start waypoint.
	 * 
	 * @return the start waypoint
	 */
	protected Waypoint getStartWaypoint() {
		return this.startWaypoint;
	}
	
	/**
	 * Sets the start waypoint.
	 * 
	 * @param the start waypoint
	 */
	protected void setStartWaypoint(Waypoint startWaypoint) {
		this.startWaypoint = startWaypoint;
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
		
		mainNet = NetworkModel.newModel(manager, this.getRLEnvironment().getDimOfState(), hiddenSize, this.getRLEnvironment().getNumOfActions());
		
		for (Pair<String, Parameter> params : mainNet.getBlock().getParameters()) {
			params.getValue().getArray().setRequiresGradient(true);
		}
		targetNet = NetworkModel.newModel(manager, this.getRLEnvironment().getDimOfState(), hiddenSize, this.getRLEnvironment().getNumOfActions());
		
		mainPredictor = mainNet.newPredictor(new NoopTranslator());
		syncNetworks();
	}

	
	/**
	 * Runs the training of the Deep Q-Network for random environment configurations
	 */
	protected void train(int test) {
		
		// The training results are stored in a file
		PrintWriter outputFile = null;
		String name = "DQN_training_results";
		try {
			outputFile = new PrintWriter(name);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		outputFile.println("Episode, Cumulative Reward, Loss, Steps, Result, Agent Performance");
		
		double decay = 0;
		int episode = 0;
		int step = 0;
		double agentPerformance = 0;
		double cumulativeReward = 0.0;
		double lossAverage = 0.0;
		String result = " ";
		int successCount = 0;
		Set<Obstacle> obstacles = new HashSet<>();
		
		// Unembeds environment's obstacles for training with random configurations
		if(this.getRLEnvironment().getObstacles()!=null) {
			obstacles.addAll(this.getRLEnvironment().getObstacles());
			for (Obstacle o : obstacles) {
				this.getRLEnvironment().unembed(o);
			}
		}
		
		// For each episode
		while (episode < NUM_GLOBAL_EPS ) {
			
			// Reset environment and variables
			episode++;
			step = 0;
			cumulativeReward = 0.0;
			lossArray.clear();
			this.getRLEnvironment().resetRandom();
			State state = this.getRLEnvironment().getStart();
			int action = 0;
			Snapshot snapshot = null;
		
			// For each time step
			while (!this.getRLEnvironment().isDone() && !this.getRLEnvironment().failed() && step < MAX_STEPS) {
				// Saves state in memory; Updates the network; Selects the next action and saves it in memory
				action = this.react(state);
				// Execute action and get next state and reward; Checks if the goal has been reached
				snapshot = this.getRLEnvironment().step(action, this.getAircraft());
				// Stores the reward and the "done" boolean in memory; too many steps counts as failure
				if (step >= MAX_STEPS) {
					memory.setReward(snapshot.getReward()-50, false, true);
					cumulativeReward += snapshot.getReward()-50;
				} else {
					memory.setReward(snapshot.getReward(), snapshot.isDone(), snapshot.failed());
					cumulativeReward += snapshot.getReward();
				}
				// Sets the state as the next state
				state = snapshot.getState();
				
				step++;
			}
			// Updates epsilon for next episode
			if (episode < EPSILON_DECAY_EPS) {
				decay = ((double) episode) / ((double) EPSILON_DECAY_EPS);
				epsilon = (float) (INITIAL_EPSILON - ((INITIAL_EPSILON - MIN_EPSILON) * decay));
			}
			
			// Training results calculation 
			if (snapshot.isDone()) {
				result = "done";
				successCount++;
			} else if (snapshot.failed()) {
				result = "obstacle";
			} else if (step >= MAX_STEPS) {
				result = "steps";
			}
			lossAverage = lossArray.stream().mapToDouble(a -> (double) a).average().orElse(0);
			agentPerformance = (double) successCount / episode;
			System.out.println(episode + ", " + cumulativeReward + ", " + lossAverage + ", " + step + ", " + result + ", " + agentPerformance);
			outputFile.println(episode + ", " + cumulativeReward + ", " + lossAverage + ", " + step + ", " + result + ", " + agentPerformance);
		}
		outputFile.close();

		// Embeds the original environment obstacles at the end of training
		if(obstacles!=null) {
			for (Obstacle o : obstacles) {
				this.getRLEnvironment().embed(o);
			}
		}
	}
	
	
	/**
	 * Runs the training of the Deep Q-Network for a given environment configuration
	 */
	protected void trainFixed() {
		
		int episode = 0;
		int step = 0;
		
		// For each episode
		while (episode < NUM_EXTRA_EPS ) {
			
			// Reset environment 
			episode++;
			step = 0;
			lossArray.clear();
			
			State state = this.getRLEnvironment().getStart();
			this.getRLEnvironment().setDone(false);
			this.getRLEnvironment().setFailure(false);
			this.getRLEnvironment().setStateToStart();
			
			int action = 0;
			Snapshot snapshot = null;
		
			// For each time step
			while (!this.getRLEnvironment().isDone() && !this.getRLEnvironment().failed() && step < MAX_STEPS) {
				// Saves state in memory; Updates the network; Selects the next action and saves it in memory
				action = this.react(state);
				// Execute action and get next state and reward; Checks if the goal has been reached
				snapshot = this.getRLEnvironment().step(action, this.getAircraft());
				// Stores the reward and the "done" boolean in memory; too many steps counts as failure
				if (step >= MAX_STEPS) {
					memory.setReward(snapshot.getReward()-50, false, true);
				} else {
					memory.setReward(snapshot.getReward(), snapshot.isDone(), snapshot.failed());
				}
				// Sets the state as the next state
				state = snapshot.getState();
				
				step++;
			}
		}
	}



	/** 
	 * Reacts to the current state, updating the memory and choosing the next action. Also responsible for
	 * the update of the main and target networks.
	 * 
	 * @param the state
	 * 
	 * @return the action
	 */
	protected int react(State state) {
		
		int action;
		iteration++;
		try (NDManager submanager = manager.newSubManager()) {
			
			memory.setState(state.getId());
			
			// Updates main network 
			if (iteration % trainNetInterval == 0  && memory.getSize()>batchSize)
				updateModel(submanager);
			
			// Syncs networks and updates every n iterations
			if (iteration % syncNetInterval == 0) {
				syncNetworks();
			}
			
			action = chooseAction(submanager, state.getId());
			
			memory.setAction(action);
			
		} catch (TranslateException e) {
			throw new IllegalStateException(e);
		}
		
		return action;
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
		NDArray qValues = mainPredictor.predict(new NDList(manager.create(state))).singletonOrThrow();
		// Chooses the action using the epsilon greedy policy and the predicted Q-values
		return ActionSampler.epsilonGreedy(qValues, rand, epsilon);
	}


	/** 
	 * Gets a batch of transitions from memory, calculates the loss based on the predicted Q-values and the actual rewards and 
	 * performs the gradient update through backpropagation. s
	 * 
	 * @param manager the memory space manager
	 */
	protected void updateModel(NDManager manager) throws TranslateException {
		
		try (GradientCollector collector = Engine.getInstance().newGradientCollector()) {
			
			MemoryBatch batch = memory.sampleBatch(batchSize, manager);
			
			// Predicts the policy for the states in the batch
			NDArray policy = mainPredictor.predict(new NDList(batch.getStates())).singletonOrThrow();
			// Gather the predicted Q-values for the selected actions in the batch
			NDArray expectedReturns = Helper.gather(policy, batch.getActions().toIntArray());
			
			//TODO: Revise difference between DQN and Double DQN
			// Predicts the target Q-values for next the states in the batch
			NDArray target = targetPredictor.predict(new NDList(batch.getNextStates())).singletonOrThrow().duplicate();
			// Calculates the target Q-values for the current states using the Bellman equation
			NDArray nextReturns = batch.getRewards().add(target.max(new int[] { 1 }).mul(batch.getDones().logicalNot()).mul(gamma));
						
			// Calculates the loss (mean squared error)
			NDArray loss = lossFunc.evaluate(new NDList(expectedReturns), new NDList(nextReturns));
			lossArray.add(loss.toArray()[0]);
			loss.setRequiresGradient(true);
			
			//Performs the backpropagation and calculates the gradients
			collector.backward(loss);

			// Updates the main network's parameters
			for (Pair<String, Parameter> params : mainNet.getBlock().getParameters()) {
				NDArray paramsArr = params.getValue().getArray();
				optimizer.update(params.getKey(), paramsArr, paramsArr.getGradient());
			}
		}
	}


	/** 
	 * Copies the weights from the main network to the target network
	 */
	protected void syncNetworks() {
	
		for (Pair<String, Parameter> params : mainNet.getBlock().getParameters()) {
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
		
		// Creates the start waypoint 
		this.setStartWaypoint(this.createWaypoint(origin));
		this.getStartWaypoint().setEto(etd);
		this.getStartWaypoint().setPoi(true);
		
		// Initializes the RLEnvironment
		this.getRLEnvironment().initializeEnvironment(origin, destination, this.getRiskPolicy(), this.getCostPolicy(), etd);
	
	}

	/**
	 * Computes a plan according to the policy learned by the DQN agent
	 * @throws TranslateException 
	 */
	protected void compute() {
		
		// Adds the start waypoint to the trajectory if this is the first part
		if (this.getWaypoints().isEmpty()) {
			this.getWaypoints().addLast(this.getStartWaypoint());
		}
		
		int step = 0;
		State state = this.getRLEnvironment().getStart();
		int action = 0;
		Snapshot snapshot = null;
		
		// Until the goal is reached, the planner fails, or does the max steps
		while(!this.getRLEnvironment().isDone() && !this.getRLEnvironment().failed() && step < MAX_STEPS) {
			
			// Chooses the action
			try (NDManager submanager = manager.newSubManager()) {
				
				NDArray qValues = targetPredictor.predict(new NDList(submanager.create(state.getId()))).singletonOrThrow();				
				action = ActionSampler.greedy(qValues);
				
			} catch (TranslateException e) {
				throw new IllegalStateException(e);
			}
			
			// Execute action and get next state and reward; Checks if the goal has been reached
			snapshot = this.getRLEnvironment().step(action, this.getAircraft());
			
			// Adds the next waypoint to the trajectory
			if (!this.getRLEnvironment().failed()) {
				this.getWaypoints().addLast(this.createWaypoint(snapshot.getState().getPosition()));
			}
			
			// Sets the state as the next state
			state = snapshot.getState();
			
			step++;
		}
		
		// If it finished without reaching the goal (because it reached the maximum number of steps
		// or failed) returns an empty trajectory 
		if (!this.getRLEnvironment().isDone()) {
			this.clearWaypoints();
			if (step >= MAX_STEPS ) {
				System.out.println("Did too many steps");
			} else if (snapshot.getReward()==-200){
				System.out.println("Hit an OBSTACLE");
			} 
		} else {
		
			// Adds the goal to the trajectory if it is not already there
			Waypoint goalWaypoint = this.createWaypoint(this.getRLEnvironment().getGoalPosition());
			if(this.getWaypoints().getLast() != goalWaypoint) {
				this.getWaypoints().addLast(goalWaypoint);
			}
			
		    // Sets ETOs and costs correctly
			this.getWaypoints().getFirst().setEto(getEtd());
			for (int i=1; i< this.getWaypoints().size(); ++i) {
				computeEto(this.getWaypoints().get(i-1), this.getWaypoints().get(i));
				computeCost(this.getWaypoints().get(i-1), this.getWaypoints().get(i));
			}
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
		wp.setCost(0);
		
		// If it is not the start
		if(!this.getWaypoints().isEmpty()) {
			computeEto(this.getWaypoints().getLast(), wp);
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
		
		int i = 0;
		
		// If compute returned an empty trajectory, retrains for the fixed environment and computes again
		while(this.getWaypoints().isEmpty() && i<5) {
			
			System.out.printf("Failed to compute, retraining (Attempt %d) %n", i+1);
			
			this.trainFixed();
			this.syncNetworks();
			
			this.getRLEnvironment().setDone(false);
			this.getRLEnvironment().setFailure(false);
			this.getRLEnvironment().setStateToStart();
			this.compute();
			
			System.out.println("Computed trajectory:" + this.getWaypoints());
			
			i++;
		}
		
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
		
		Position currentOrigin = origin;
		ZonedDateTime currentEtd = etd;
		
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
			supports = (environment instanceof RLEnvironment);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
		}
		
		return supports;
	}

	
}

