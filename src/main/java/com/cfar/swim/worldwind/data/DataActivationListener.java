package com.cfar.swim.worldwind.data;

public interface DataActivationListener {

	public String getIdentifier();
	public void activate();
	public void deactivate();
	
}
