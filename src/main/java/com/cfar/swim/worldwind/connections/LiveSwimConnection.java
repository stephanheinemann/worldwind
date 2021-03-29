package com.cfar.swim.worldwind.connections;

import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;

public class LiveSwimConnection extends SwimConnection {

	@Override
	public void connect() {
	}

	@Override
	public void disconnect() {
	}

	@Override
	public boolean isConnected() {
		return false;
	}

	@Override
	public final boolean matches(Specification<? extends FactoryProduct> specification) {
		return false;
	}

}
