package com.cfar.swim.worldwind.planners.rl.dqn;

import ai.djl.Model;
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
 * Realizes a Score Model
 * 
 * @author Rafaela Seguro
 *
 */

public class ScoreModel extends AbstractBlock {
	
	/**  */
	private static final byte VERSION = 2;
	
	/**  */
	private final NDManager manager;
	
	/**  */
	private final Block linearInput;
	
	/**  */
	private final Block linearOutput;
	
	/**  */
	private final int hiddenSize;
	
	/**  */
	private final int outputSize;
	
	
	/** Constructs a score model
	 * 
	 * @param the manager
	 * @param the hidden size
	 * @param the output size
	 */
	public ScoreModel(NDManager manager, int hiddenSize, int outputSize) {
		super(VERSION);
		this.manager = manager;
		
		this.linearInput = addChildBlock("linear_input", Linear.builder().setUnits(hiddenSize).build());
		this.linearOutput = addChildBlock("linear_output", Linear.builder().setUnits(outputSize).build());
		
		this.hiddenSize = hiddenSize;
		this.outputSize = outputSize;
	}
	
	
	/** Gets this model's manager
	 * 
	 * @return the manager
	 */
	public NDManager getManager() {
		return manager;
	}
	
	
	/** 
	 * 
	 * @param the manager
	 * @param the input size
	 * @param the hidden size
	 * @param the output size
	 */
	public static Model newModel(NDManager manager, int inputSize, int hiddenSize, int outputSize) {
		
		Model model = Model.newInstance("ScoreModel");
		ScoreModel net = new ScoreModel(manager, hiddenSize, outputSize);
		net.initialize(net.getManager(), DataType.FLOAT32, new Shape(inputSize));
		model.setBlock(net);
		
		return model;
	}
	
	
	/** 
	 * 
	 * @param the parameter store
	 * @param the inputs list
	 * @param boolean that indicates if model is training
	 * @param 
	 */
	public NDList forward2(ParameterStore parameterStore, NDList inputs, boolean training, PairList<String, Object> params) {
		
		NDList hidden = new NDList(Activation.relu(linearInput.forward(parameterStore, inputs, training).singletonOrThrow()));
		
		return linearOutput.forward(parameterStore, inputs, training);
	}
	
	/** 
	 * 
	 * @param the manager
	 * @param the input shape
	 * 
	 * @return array with the output shapes
	 */
	public Shape[] getOutputShapes(Shape[] inputShape) {
		
		return new Shape[] { new Shape(outputSize) };
	}
	
	/** 
	 * 
	 * @param the manager
	 * @param the data type
	 * @param the input shapes
	 */
	public void initializeChildBlocks(NDManager manager, DataType dataType, Shape... inputShapes) {
		setInitializer(new XavierInitializer(), Parameter.Type.WEIGHT);
		linearInput.initialize(manager, dataType, inputShapes[0]);
		linearOutput.initialize(manager, dataType, new Shape(hiddenSize));
	}


	@Override
	protected NDList forwardInternal(ParameterStore parameterStore, NDList inputs, boolean training,
			PairList<String, Object> params) {
		// TODO Auto-generated method stub
		return null;
	}

}
