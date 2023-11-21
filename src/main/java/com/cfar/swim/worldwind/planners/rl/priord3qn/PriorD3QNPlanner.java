package com.cfar.swim.worldwind.planners.rl.priord3qn;

import java.time.ZonedDateTime;


import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;

import org.apache.commons.collections4.queue.CircularFifoQueue;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.environments.PlanningContinuum;
import com.cfar.swim.worldwind.environments.RLEnvironment;
import com.cfar.swim.worldwind.planners.AbstractPlanner;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.planners.rl.ActionSampler;
import com.cfar.swim.worldwind.planners.rl.Helper;
import com.cfar.swim.worldwind.planners.rl.MemoryBatch;
import com.cfar.swim.worldwind.planners.rl.NetworkModel;
import com.cfar.swim.worldwind.planners.rl.Snapshot;
import com.cfar.swim.worldwind.planners.rl.State;
import com.cfar.swim.worldwind.planners.rl.d3qn.DuelingNetworkModel;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.planners.rl.priordqn.*;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.render.*;
//import com.cfar.swim.worldwind.tests.Plot;
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
* Realizes a deep reinforcement learning planner, using a Double Deep Q-Network with Prioritized Experience Replay, 
* that plans a trajectory of an aircraft in an environment considering a local cost and risk policy.
* 
* @author Rafaela Seguro
*
*/

public class PriorD3QNPlanner extends AbstractPlanner {
	
	/** the initial value of epsilon */
	private static final float INITIAL_EPSILON = 0.8f;
	
	/** the minimum value of epsilon during training */
	private static final float MIN_EPSILON = 0.1f; 
	
	/** the initial value of beta */
	private static final double INITIAL_BETA = 0.4;
	
	/** number of total training episodes */
	private static final int NUM_GLOBAL_EPS = 3000;
	
	/** number of episodes it trains without obstacles first */
	private static final int NO_OBS_EPS = 3000;
	
	/** maximum number of steps per episode */
	private static final int MAX_STEPS = 150;
	
	/** random number */
	private final Random rand = new Random();
	
	/** the number of hidden units (neurons) in the neural network */
	private final int[] hiddenSize = {256, 512, 256};
	
	/** learning rate used by the optimizer during training */
	private final float learningRate = 0.0007f;
	
	/** the size of the mini-batch of transitions used for training */
	protected final int batchSize = 32;
	
	/** the number of iterations between each train of the policy network */
	protected final int trainNetInterval = 1;
	
	/** the number of iterations between each sync of the target and policy networks */
	protected final int syncNetInterval = 200;
	
	/** gamma factor for the Bellman equation */
	protected final float gamma = 0.99f;
	
	/** parameter that determines how much prioritization is used in prioritized experience replay*/
	protected final double alpha = 0.6;
	
	/** parameter to balance importance sampling*/
	private double beta = INITIAL_BETA;
	
	/** replay memory */
	private final PrioritizedMemory memory = new PrioritizedMemory(4096, alpha);
	
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
	
	/** saves the most recent waypoint added to the trajectory */
	private Waypoint newestWaypoint = null;
	
	/** stores the plan's ETD */
	private ZonedDateTime etd;
	
	/** Stores episode results */
	private CircularFifoQueue<Integer> episodeResults = new CircularFifoQueue<Integer>(1000);
	
	/** Stores the environment for which the training was performed*/
	private RLEnvironment trainEnvironment = null;
	
	
	/** Constructs a planner trained by a Double Deep Q-Network with Prioritized Experience Replay 
	 * for a specified aircraft and environment using default local cost and risk policies.
	 * 
	 * @param the aircraft
	 * @param the environment
	 */
	public PriorD3QNPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		this.etd = environment.getTime();
		
		resetAgent();
		this.trainEnvironment = this.getRLEnvironment();
		train();
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
		return Specification.PLANNER_PRIORD3QN_ID;
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
		
		policyNet = DuelingNetworkModel.newModel(manager, this.getRLEnvironment().getDimOfState(), hiddenSize, this.getRLEnvironment().getNumOfActions());
		//Sets require gradient to true for the policy network's parameters
		for (Pair<String, Parameter> params : policyNet.getBlock().getParameters()) {
			params.getValue().getArray().setRequiresGradient(true);
		}
		targetNet = DuelingNetworkModel.newModel(manager, this.getRLEnvironment().getDimOfState(), hiddenSize, this.getRLEnvironment().getNumOfActions());
		
		policyPredictor = policyNet.newPredictor(new NoopTranslator());
		syncNetworks();
	}

	/**
	 * Runs the training of the Deep Q-Network for random environment configurations
	 */
	protected void train() {
		
		double decay = 0;
		int episode = -1;
		int step;
		int numOfEpisodes = NO_OBS_EPS;
		double agentPerformance = 0;
		boolean addObstacles = false;
		Set<Obstacle> obstacles = new HashSet<>();
		//|| agentPerformance < 0.9
		
		if(this.getRLEnvironment().getObstacles()!=null) {
			obstacles.addAll(this.getRLEnvironment().getObstacles());
			for (Obstacle o : obstacles) {
				this.getRLEnvironment().unembed(o);
			}
		}
		
		// For each episode
		while (episode < NUM_GLOBAL_EPS ) {
			
			// Adds obstacles to the training 
			if(episode == NO_OBS_EPS) {
				addObstacles = true;
				epsilon = INITIAL_EPSILON;
				numOfEpisodes = NUM_GLOBAL_EPS - NO_OBS_EPS;
//				if(obstacles!=null) {
//					for (Obstacle o : obstacles) {
//						this.getRLEnvironment().embed(o);
//					}
//				}
			} 
			// Reset environment 
			episode++;
			step = 0;
			
			this.getRLEnvironment().resetRandom(addObstacles);
//			this.getRLEnvironment().initializeEnvironment(origin, destination, getRiskPolicy(), getCostPolicy(), etd);
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
					memory.setReward((-50), false, true);
				} else {
					memory.setReward(snapshot.getReward(), snapshot.isDone(), snapshot.failed());
				}
				// Sets the state as the next state
				state = snapshot.getState();

//				System.out.println("STATE " + (step + 1));
//				System.out.println("Cost Policy: " + state.getId()[0] + "; Step size: " + state.getId()[1]);
//				System.out.println("Distance to axes: " + state.getId()[2] + "; " + state.getId()[3] + "; " + state.getId()[4] 
//						+ "; " + state.getId()[5] + "; " + state.getId()[6] + "; " + state.getId()[7]);
//				System.out.println("Relative goal: " + state.getId()[8] + "; " + state.getId()[9] + "; " + state.getId()[10] + "; " + state.getId()[11]);
//				System.out.println("Obstacle 1: " + state.getId()[12] + "; " + state.getId()[13] + "; " + state.getId()[14] + "; " + state.getId()[15]);
//				System.out.println("Obstacle 2: " + state.getId()[16] + "; " + state.getId()[17] + "; " + state.getId()[18] + "; " + state.getId()[19]);
//				System.out.println("Obstacle 3: " + state.getId()[20] + "; " + state.getId()[21] + "; " + state.getId()[22] + "; " + state.getId()[23]);
//				System.out.println("Action: " + action);
//				System.out.println("Reward " + snapshot.getReward());

				step++;
			}
			// Updates beta and epsilon for next episode
			decay = episode / (numOfEpisodes);
			beta = (INITIAL_BETA + ((1 - INITIAL_BETA) * decay));
			decay = Math.exp(-episode * 1.0 / numOfEpisodes);
			epsilon = (float) (MIN_EPSILON + (INITIAL_EPSILON - MIN_EPSILON) * decay);
			
			if(snapshot.isDone()) {
				episodeResults.add(1);
			} else {
				episodeResults.add(0);
			}
			double sum = (episodeResults.stream().mapToInt(Integer::intValue).sum());
			double size = episodeResults.size();
			agentPerformance =  sum / size;
			
			if (snapshot.isDone()) {
				System.out.println("Episode " + episode + " -----------------------> GOAL in " + step + " steps");
			} else if (snapshot.failed()) {
				System.out.println("Episode " + episode + " hit an OBSTACLE");
			} else if (step >= MAX_STEPS) {
				System.out.println("Episode " + episode + " did too many steps");
			}
			System.out.printf("Performance %,.3f %n", agentPerformance);
		}
		//this.getRLEnvironment().embed(obstacle);
		if(obstacles!=null) {
			for (Obstacle o : obstacles) {
				this.getRLEnvironment().embed(o);
			}
		}
	}


	/** 
	 * Reacts to the current state, updating the memory and choosing the next action
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
			
			// Trains every n iterations only
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
		
		// Gets the predicted Q-values from the main network (Double DQN)
		NDArray qValues = policyPredictor.predict(new NDList(manager.create(state))).singletonOrThrow();
		// Chooses the action using the epsilon greedy policy and the predicted Q-values
		return ActionSampler.epsilonGreedy(qValues, rand, epsilon);
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
			
			// Computes new priorities from the TD errors
			NDArray tdErrors = nextReturns.sub(expectedReturns);
			NDArray newPriorities = tdErrors.abs().add(0.001);
			
			// Updates transition data in memory and gets the importance sampling weights
			 NDArray importanceWeights = manager.create(memory.updateTransitionData(batch.getIndices().toDoubleArray(), newPriorities.toDoubleArray(), beta));
			
			// Calculates the loss (mean squared error)
			NDArray weightedLoss = lossFunc.evaluate(new NDList(expectedReturns), new NDList(nextReturns)).mul(importanceWeights);
			weightedLoss.setRequiresGradient(true);
			
			//Performs the backpropagation and calculates the gradients
			collector.backward(weightedLoss);

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
//		this.setCostPolicy(actualCostPolicy);
//		this.setRiskPolicy(actualRiskPolicy);
		
		// Creates the start waypoint 
		this.setNewestWaypoint(this.createWaypoint(origin));
		this.getNewestWaypoint().setEto(etd);
		this.getNewestWaypoint().setPoi(true);
		
		// Initializes the RLEnvironment
		this.getRLEnvironment().initializeEnvironment(origin, destination, this.getRiskPolicy(), this.getCostPolicy(), etd);
	
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
		State state = this.getRLEnvironment().getStart();
		int action = 0;
		Snapshot snapshot = null;
		
		// Until the goal is reached, the planner fails, or does the max steps
		while(!this.getRLEnvironment().isDone() && !this.getRLEnvironment().failed() && step < MAX_STEPS) {
			
			// Chooses the action
			try (NDManager submanager = manager.newSubManager()) {
				
				NDArray qValues = targetPredictor.predict(new NDList(submanager.create(state.getId()))).singletonOrThrow();
				// DEBUG
//				System.out.print("Q VALUES: ");
//				for (int i=0; i<qValues.size(); i++) {
//					System.out.print(qValues.getFloat(i) + "; ");
//				}
//				System.out.println();
				
				action = ActionSampler.greedy(qValues);
				
			} catch (TranslateException e) {
				throw new IllegalStateException(e);
			}
			
			System.out.println("STATE " + (step + 1));
			System.out.println("Cost Policy: " + state.getId()[0] + "; Step size: " + state.getId()[1]);
			System.out.println("Move vector: " + state.getId()[2] + "; " + state.getId()[3] + "; " + state.getId()[4]);
			System.out.println("Distance to axes: " + state.getId()[5] + "; " + state.getId()[6] + "; " + state.getId()[7] 
					+ "; " + state.getId()[8] + "; " + state.getId()[9] + "; " + state.getId()[10]);
			System.out.println("Relative goal: " + state.getId()[11] + "; " + state.getId()[12] + "; " + state.getId()[13] + "; " + state.getId()[14]);
			System.out.println("Obstacle 1: " + state.getId()[15] + "; " + state.getId()[16] + "; " + state.getId()[17] + "; " + state.getId()[18]);
			System.out.println("Obstacle 2: " + state.getId()[19] + "; " + state.getId()[20] + "; " + state.getId()[21] + "; " + state.getId()[22]);
			System.out.println("Obstacle 3: " + state.getId()[23] + "; " + state.getId()[24] + "; " + state.getId()[25] + "; " + state.getId()[26]);
			System.out.println("Action: " + action);
			
			// Execute action and get next state and reward; Checks if the goal has been reached
			snapshot = this.getRLEnvironment().step(action, this.getAircraft());
			
			// Adds the next waypoint to the trajectory
			if (!this.getRLEnvironment().failed()) {
				// Only adds if the position is different to the previous one
				//if (this.getWaypoints().getLast().getPrecisionPosition() != new PrecisionPosition(snapshot.getState().getPosition()))
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
			} else if (snapshot.getReward()==-150){
				System.out.println("Hit an OBSTACLE");
			} else if (snapshot.getReward()==-50){
				System.out.println("Left environment");
			}
			
		} else {
		
			// Adds the goal to the trajectory if it is not already there
			Waypoint goalWaypoint = this.createWaypoint(this.getRLEnvironment().getGoalPosition());
			if(this.getWaypoints().getLast() != goalWaypoint) {
				this.getWaypoints().addLast(goalWaypoint);
			}
			
		    // Sets ETOs correctly
			this.getWaypoints().getFirst().setEto(getEtd());
			for (int i=1; i< this.getWaypoints().size(); ++i) {
				computeEto(this.getWaypoints().get(i-1), this.getWaypoints().get(i));
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

