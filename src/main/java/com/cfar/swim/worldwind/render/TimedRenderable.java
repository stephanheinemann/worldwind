package com.cfar.swim.worldwind.render;

import java.time.ZonedDateTime;

import gov.nasa.worldwind.render.Renderable;

public interface TimedRenderable extends Renderable {

	public ZonedDateTime getTime();

	public void setTime(ZonedDateTime time);

}
