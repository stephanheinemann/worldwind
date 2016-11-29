package com.cfar.swim.worldwind.data;

import java.util.Set;

import javax.xml.bind.JAXBException;

import org.xml.sax.InputSource;

import com.cfar.swim.worldwind.render.Obstacle;

public interface ObstacleLoader {

	public Set<Obstacle> load(InputSource source) throws JAXBException;
	
}
