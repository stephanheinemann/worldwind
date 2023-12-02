package com.cfar.swim.worldwind.planners.rl.d3qn;

import java.util.List;
import java.util.ArrayList;

import ai.djl.Model;
import ai.djl.ndarray.NDArray;
import ai.djl.ndarray.NDList;
import ai.djl.ndarray.NDManager;
import ai.djl.ndarray.types.DataType;
import ai.djl.ndarray.types.Shape;
import ai.djl.nn.AbstractBlock;
import ai.djl.nn.Activation;
import ai.djl.nn.Block;
import ai.djl.nn.Parameter;
import ai.djl.nn.core.Linear;
import ai.djl.training.ParameterStore;
import ai.djl.training.initializer.XavierInitializer;
import ai.djl.util.PairList;

/**
 * Realizes a model for the neural network that is used to train the Deep Reinforcement Learning Planner
 * 
 * @author Rafaela Seguro
 *
 */

public class DuelingNetworkModel extends AbstractBlock {
	
	/** the version of the block, used to track changes to the architecture or behavior */
	private static final byte VERSION = 1;
	
	/** NDArray manager */
	private final NDManager manager;

	/** input layer block */
	private final Block linearInput;
	
	/** stream to estimate advantage */
	private final Block linearAdvantage;
	
	/** stream to estimate state value */
	private final Block linearValue;
	
	/** output layer block */
	private final Block linearOutput;
	
	/** hidden layers list of blocks */
	private final List<Block> hiddenLayers;
	
	/** the number of hidden units in the neural network */
	private final int[] hiddenSize;
	
	/** the number of output units in the neural network */
	private final int outputSize;
	
	
	/** 
	 * Constructs a neural network model
	 * 
	 * @param the manager for NDArray operations
	 * @param the number of hidden units in the neural network
	 * @param the number of output units in the neural network
	 */
	public DuelingNetworkModel(NDManager manager, int[] hiddenSize, int outputSize) {

		super(VERSION);
		
		this.manager = manager;
		this.hiddenSize = hiddenSize;
		this.hiddenLayers = new ArrayList<>();  // Initialize the list of hidden layers
		this.outputSize = outputSize;
		
		// Create and add the input layer
		linearInput = Linear.builder().setUnits(hiddenSize[0]).build();
		this.hiddenLayers.add(addChildBlock("linear_input", linearInput));
		
		// Create and add the hidden layers for the advantage stream
		for (int i = 1; i < hiddenSize.length; i++) {
			Block hiddenLayer = Linear.builder().setUnits(hiddenSize[i]).build();
			this.hiddenLayers.add(addChildBlock("advantage_hidden_" + i, hiddenLayer));
		}
		linearAdvantage = Linear.builder().setUnits(this.outputSize).build();
		this.hiddenLayers.add(addChildBlock("advantage_output", linearAdvantage));
		
		// Create and add the hidden layers for the state value stream
		for (int i = 1; i < hiddenSize.length; i++) {
			Block hiddenLayer = Linear.builder().setUnits(hiddenSize[i]).build();
			this.hiddenLayers.add(addChildBlock("value_hidden_" + i, hiddenLayer));
		}
		linearValue = Linear.builder().setUnits(1).build();
		this.hiddenLayers.add(addChildBlock("value_output", linearValue));
		
//		// Create and add the hidden layers 
//		for (int i = 1; i < (hiddenSize.length); i++) {
//			Block hiddenLayer = Linear.builder().setUnits(hiddenSize[i]).build();
//			this.hiddenLayers.add(addChildBlock("hidden_layer_" + i, hiddenLayer));
//		}	
		
//		// The last hidden layer is separated into the value and advantage stream
////		Block hiddenLayer = Linear.builder().setUnits(hiddenSize[hiddenSize.length - 1]).build();
////		this.hiddenLayers.add(addChildBlock("advantage_hidden_" , hiddenLayer));
//		linearAdvantage = Linear.builder().setUnits(this.outputSize).build();
//		this.hiddenLayers.add(addChildBlock("advantage_stream", linearAdvantage));
//		
////		hiddenLayer = Linear.builder().setUnits(hiddenSize[hiddenSize.length - 1]).build();
////		this.hiddenLayers.add(addChildBlock("value_hidden_" , hiddenLayer));
//		linearValue = Linear.builder().setUnits(1).build();
//		this.hiddenLayers.add(addChildBlock("value_stream", linearValue));
		
		// Create and add the output layer
		linearOutput = Linear.builder().setUnits(this.outputSize).build();
		this.hiddenLayers.add(addChildBlock("linear_output", linearOutput));
		
	}
	
	
	/** 
	 * Gets the manager for this network
	 * 
	 * @return the manager
	 */
	public NDManager getManager() {
		return manager;
	}
	
	
	/** 
	 * Method to create a new neural network model
	 * 
	 * @param the manager for NDArray operations
	 * @param the size of the input data
	 * @param the number of hidden units in the neural network
	 * @param the number of output units in the neural network
	 * 
	 * @return a new NetowrkModel instance
	 */
	public static Model newModel(NDManager manager, int inputSize, int[] hiddenSize, int outputSize) {
		
		Model model = Model.newInstance("NetworkModel");
		DuelingNetworkModel network = new DuelingNetworkModel(manager, hiddenSize, outputSize);
		network.initialize(network.getManager(), DataType.FLOAT32, new Shape(inputSize));
		model.setBlock(network);
		
		return model;
	}
	
	
	/** 
	 * Forward pass of the neural network
	 * 
	 * @param the parameter store for the model
	 * @param a list of input NDArrays
	 * @param boolean that indicates if model is in training mode
	 * @param a list of additional parameters
	 * 
	 * @return the output of the forward pass
	 */
	@Override
	public NDList forwardInternal(ParameterStore parameterStore, NDList inputs, boolean training, PairList<String, Object> params) {

		NDList initial = inputs;
		
		//Forward pass through input layer
		initial = hiddenLayers.get(0).forward(parameterStore, initial, true);
        initial = new NDList(Activation.relu(initial.singletonOrThrow()));
        
        NDList current = initial;
	    
	    // Forward pass through advantage layers
	    for (int i = 1; i < (hiddenSize.length); i++) {
	    	current = hiddenLayers.get(i).forward(parameterStore, current, true);
	        current = new NDList(Activation.relu(current.singletonOrThrow()));
	    }
	    NDArray advantageStream = linearAdvantage.forward(parameterStore, current, training).singletonOrThrow();
	    
	    current = initial;

	    // Forward pass through value layers
	    for (int i = 1; i < (hiddenSize.length); i++) {
	    	current = hiddenLayers.get(i+hiddenSize.length).forward(parameterStore, current, true);
	        current = new NDList(Activation.relu(current.singletonOrThrow()));
	    }
	    NDArray valueStream = linearValue.forward(parameterStore, current, training).singletonOrThrow();
	    
        // Output calculation
	    NDArray qValues = valueStream.add(advantageStream.sub(advantageStream.mean(new int[]{0}, false)));
	    
	    current = linearOutput.forward(parameterStore, new NDList(qValues), true);
	    
	    return current;
	}
	
	/** 
	 * Gets the output shapes of the neural network
	 * 
	 * @param the manager for NDArray operations
	 * @param an array of input shapes
	 * 
	 * @return array with the output shapes
	 */
	@Override
	public Shape[] getOutputShapes(Shape[] inputShape) {
		Shape[] current = inputShape;
		for (Block block : children.values()) {
			current = block.getOutputShapes(current);
		}
		return current;
	}
	
	/** 
	 * Initializes the child blocks (layer or sequence of layers) of the neural network
	 * 
	 * @param the manager for NDArray operations
	 * @param the data type to use for initialization
	 * @param an array of input shapes
	 */
	@Override
	public void initializeChildBlocks(NDManager manager, DataType dataType, Shape... inputShapes) {
	    setInitializer(new XavierInitializer(), Parameter.Type.WEIGHT);

	    // Initialize input and output layers
 		hiddenLayers.get(0).initialize(manager, dataType, inputShapes[0]);
 		linearOutput.initialize(manager, dataType, new Shape(outputSize));
	 	
//	 	// Initialize hidden layers 
//	    for (int i = 1; i < (hiddenSize.length); i++) {
//            hiddenLayers.get(i).initialize(manager, dataType, new Shape(hiddenSize[i - 1]));
//	    }
//		   
//		// Initialize layers for the advantage stream
//	    hiddenLayers.get(hiddenSize.length).initialize(manager, dataType, new Shape(hiddenSize[hiddenSize.length - 1]));
//	    
//	    // Initialize layers for the value stream
//	    hiddenLayers.get(hiddenSize.length+1).initialize(manager, dataType, new Shape(hiddenSize[hiddenSize.length - 1]));
	    
	    // Initialize hidden layers for the advantage stream
	    for (int i = 1; i < (hiddenSize.length+1); i++) {
            // Initialize subsequent hidden layers
            hiddenLayers.get(i).initialize(manager, dataType, new Shape(hiddenSize[i - 1]));
	    }
	    // Initialize hidden layers for the advantage stream
	    for (int i = 1; i < (hiddenSize.length+1); i++) {
            // Initialize subsequent hidden layers
            hiddenLayers.get(i+hiddenSize.length).initialize(manager, dataType, new Shape(hiddenSize[i - 1]));
	    }
	}


}
