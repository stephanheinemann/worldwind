package com.cfar.swim.worldwind.geom.precision;

public interface Precision {
	
	/**
	 * the default epsilon used to compensate for numerical inaccuracies
	 */
	public static final int PRECISION = 7;
	public static final double EPSILON = 1E-7;
	
	public int getPrecision();
	public Precision setPrecision(int precision);
	public Object getOriginal();

}
