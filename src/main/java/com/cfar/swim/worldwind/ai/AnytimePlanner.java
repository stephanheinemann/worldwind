package com.cfar.swim.worldwind.ai;

import java.beans.PropertyChangeListener;

public interface AnytimePlanner extends Planner {
	
	public double getInflationFactor();
	public void setInflationFactor(double inflationFactor);
	
	public double getDeflationAmount();
	public void setDeflationAmount(double deflationAmount);
	
	public void addPlanChangeListener(PropertyChangeListener listener);
}
