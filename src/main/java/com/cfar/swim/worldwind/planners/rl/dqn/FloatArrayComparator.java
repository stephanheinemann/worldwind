package com.cfar.swim.worldwind.planners.rl.dqn;

import java.util.Arrays;
import java.util.Comparator;

/**
 * Realizes a comparator of arrays of doubles, to compare state IDs.
 * 
 * @author Rafaela Seguro
 *
 */

public class FloatArrayComparator implements Comparator<float[]>{
	
	public int compare(float[] array1, float[] array2) {
		
		return Arrays.compare(array1, array2);
	}

}
