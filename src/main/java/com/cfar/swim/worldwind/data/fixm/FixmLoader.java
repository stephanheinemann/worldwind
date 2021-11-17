/**
 * Copyright (c) 2021, Stephan Heinemann (UVic Center for Aerospace Research)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.cfar.swim.worldwind.data.fixm;

import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

import javax.xml.bind.JAXBElement;
import javax.xml.bind.JAXBException;

import org.xml.sax.InputSource;

import com.cfar.swim.fixm.bind.FixmUnmarshaller;
import com.cfar.swim.worldwind.aircraft.A320;
import com.cfar.swim.worldwind.aircraft.CombatIdentification;
import com.cfar.swim.worldwind.aircraft.H135;
import com.cfar.swim.worldwind.aircraft.Iris;
import com.cfar.swim.worldwind.data.Conversions;
import com.cfar.swim.worldwind.data.ObstacleLoader;
import com.cfar.swim.worldwind.data.SwimProtocol;
import com.cfar.swim.worldwind.data.SwimResource;
import com.cfar.swim.worldwind.planning.CostInterval;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.airspaces.ObstacleSphere;
import com.cfar.swim.worldwind.render.annotations.DepictionAnnotation;

import aero.fixm.base.AltitudeType;
import aero.fixm.base.GeographicalPositionType;
import aero.fixm.flight.AircraftType;
import aero.fixm.flight.AircraftTypeType;
import aero.fixm.flight.FlightIdentificationType;
import aero.fixm.flight.FlightType;
import aero.fixm.flight.Point4DTimeChoiceType;
import aero.fixm.flight.RouteTrajectoryElementType;
import aero.fixm.flight.RouteTrajectoryGroupContainerType;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.globes.Earth;

/**
 * Realizes a FIXM obstacle loader.
 * 
 * @author Stephan Heinemann
 *
 */
public class FixmLoader implements ObstacleLoader {
	
	// NOTE: this is only a very simplified, specialized and rudimentary loader
	
	// TODO: cost and separation could be part of FIXM:
	// Dangerous Goods, Special Handling, ATC requirements, Aircraft Cost Map
	
	/** the default cost of an aircraft obstacle */
	private static final double DEFAULT_AIRCRAFT_COST = 100d;
	
	/** the default separation of aircraft obstacles (determines sphere radius) */
	private static final Duration DEFAULT_AIRCRAFT_SEPARATION = Duration.ofSeconds(10l);
	
	/** the FIXM unmarshaller of this FIXM loader */
	private FixmUnmarshaller unmarshaller;
	
	/**
	 * Constructs a new FIXM obstacle loader.
	 * 
	 * @throws JAXBException if the FIXM obstacle loader cannot be constructed
	 */
	public FixmLoader() throws JAXBException {
		this.unmarshaller = new FixmUnmarshaller();
	}
	
	/**
	 * Loads FIXM obstacles from a SWIM resource.
	 * 
	 * @param resource the SWIM resource
	 * 
	 * @return the loaded obstacles
	 * 
	 * @see ObstacleLoader#load(SwimResource)
	 */
	@Override
	public Set<Obstacle> load(SwimResource resource) {
		Set<Obstacle> obstacles = new HashSet<>();
		
		if (resource.getProtocol().isEmpty()
				|| (resource.getProtocol().isPresent()
				&& resource.getProtocol().get().equals(SwimProtocol.FIXM))) {
			try {
				obstacles = this.load(new InputSource(resource.getResource().toURL().openStream()));
			} catch (Exception e) {
				//e.printStackTrace();
			}
		}
		
		return obstacles;
	}
	
	/**
	 * Loads FIXM obstacles from an input source.
	 * 
	 * @param source the input source
	 * 
	 * @return the loaded FIXM obstacles
	 * 
	 * @throws JAXBException if the FIXM obstacles cannot be loaded
	 */
	public Set<Obstacle> load(InputSource source) throws JAXBException {
		Set<Obstacle> obstacles = new HashSet<>();
		JAXBElement<?> fixmElement = (JAXBElement<?>) this.unmarshaller.unmarshal(source);
		
		if (fixmElement.getValue() instanceof FlightType) {
			obstacles = this.loadFlight((FlightType) fixmElement.getValue());
		}
		
		return obstacles;
	}
	
	/**
	 * Loads FIXM flight obstacles.
	 * 
	 * @param flight the FIXM flight
	 * 
	 * @return the loaded FIXM flight obstacles
	 */
	private Set<Obstacle> loadFlight(FlightType flight) {
		Set<Obstacle> obstacles = new HashSet<>();
		
		String flightId = this.loadFlightId(flight.getFlightIdentification());
		Trajectory trajectory = this.loadTrajectory(flight.getRouteTrajectoryGroup());
		
		if (!flightId.equals("") && !trajectory.isEmpty()) {
			Iterator<? extends Waypoint> wpi = trajectory.getWaypoints().iterator();
			Waypoint current = wpi.next();
			while (wpi.hasNext()) {
				ObstacleSphere aircraft = this.loadAircraft(flight.getAircraft());
				aircraft.moveTo(current);
				aircraft.setCostInterval(new CostInterval(flightId, current.getEto(), current.getEto(), FixmLoader.DEFAULT_AIRCRAFT_COST));
				
				Waypoint next = wpi.next();
				ObstacleSphere nextAircraft = this.loadAircraft(flight.getAircraft());
				nextAircraft.moveTo(next);
				nextAircraft.setCostInterval(new CostInterval(flightId, next.getEto(), next.getEto(), FixmLoader.DEFAULT_AIRCRAFT_COST));
				
				Duration duration = Duration.between(current.getEto(), next.getEto());
				double distance = Position.ellipsoidalDistance(current, next, Earth.WGS84_EQUATORIAL_RADIUS, Earth.WGS84_POLAR_RADIUS);
				double speed = 0d;
				if (!duration.isZero() && !duration.isNegative()) {
					speed = distance / duration.getSeconds();
				}
				
				// aircraft safety radius
				aircraft.setRadius(FixmLoader.DEFAULT_AIRCRAFT_SEPARATION.toSeconds() * speed);
				nextAircraft.setRadius(FixmLoader.DEFAULT_AIRCRAFT_SEPARATION.toSeconds() * speed);
				// TODO: potential memory issue for long distances or small radii
				int steps = (int) Math.round((Math.log(distance / aircraft.getRadius()) / Math.log(2d)));
				steps = Math.max(steps, 1);
				
				// current aircraft obstacle sphere (to be modified by interpolation)
				obstacles.add(aircraft);
				// interpolate aircraft obstacle sphere
				obstacles.addAll(aircraft.interpolate(nextAircraft, steps));
				
				if (wpi.hasNext()) {
					current = next;
				} else {
					// last aircraft obstacle sphere
					obstacles.add(nextAircraft);
				}
			}
			obstacles.stream().forEach(o -> o.getDepiction().setAnnotation(
					new DepictionAnnotation(flightId, o.getCenter())));
		}
		
		return obstacles;
	}
	
	/**
	 * Loads the FIXM flight identification.
	 * 
	 * @param flightIdElement the FIXM flight identification element
	 * 
	 * @return the loaded FIXM flight identification
	 */
	private String loadFlightId(JAXBElement<FlightIdentificationType> flightIdElement) {
		String flightId = "";
		
		if (null != flightIdElement) {
			if (null != flightIdElement.getValue()) {
				if (null != flightIdElement.getValue().getAircraftIdentification()) {
					if (null != flightIdElement.getValue().getAircraftIdentification().getValue()) {
						flightId = flightIdElement.getValue().getAircraftIdentification().getValue();
					}
				}
			}
		}
		
		return flightId;
	}
	
	/**
	 * Loads a FIXM aircraft.
	 * 
	 * @param aircraftElement the FIXM aircraft
	 * 
	 * @return the loaded FIXM aircraft
	 */
	private ObstacleSphere loadAircraft(JAXBElement<AircraftType> aircraftElement) {
		ObstacleSphere aircraft = new ObstacleSphere(Position.ZERO, 1000d);
		
		if (null != aircraftElement) {
			if (null != aircraftElement.getValue()) {
				if (null != aircraftElement.getValue().getAircraftType()) {
					if (!aircraftElement.getValue().getAircraftType().isEmpty()) {
						AircraftTypeType aircraftType = aircraftElement.getValue().getAircraftType().get(0);
						if (null != aircraftType.getType()) {
							if (null != aircraftType.getType().getValue().getIcaoAircraftTypeDesignator()) {
								String designator = aircraftType.getType().getValue().getIcaoAircraftTypeDesignator();
								if (Specification.AIRCRAFT_A320_ID.equals(designator)) {
									aircraft = new A320(Position.ZERO, A320.SEPARATION_RADIUS, CombatIdentification.NEUTRAL);
								}
								if (Specification.AIRCRAFT_H135_ID.equals(designator)) {
									aircraft = new H135(Position.ZERO, H135.SEPARATION_RADIUS, CombatIdentification.NEUTRAL);
								}
								if (Specification.AIRCRAFT_IRIS_ID.equals(designator)) {
									aircraft = new Iris(Position.ZERO, Iris.SEPARATION_RADIUS, CombatIdentification.NEUTRAL);
								}
							}
						}
					}
				}
			}
		}
		
		return aircraft;
	}
	
	/**
	 * Loads a FIXM trajectory
	 * 
	 * @param trajectoryElement the FIXM trajectory
	 * 
	 * @return the loaded FIXM trajectory
	 */
	private Trajectory loadTrajectory(JAXBElement<RouteTrajectoryGroupContainerType> trajectoryElement) {
		Trajectory trajectory = new Trajectory();
		
		if (null != trajectoryElement) {
			if (null != trajectoryElement.getValue()) {
				if (null != trajectoryElement.getValue().getCurrent()) {
					if (null != trajectoryElement.getValue().getCurrent().getValue()) {
						if (null != trajectoryElement.getValue().getCurrent().getValue().getElement()) {
							if (!trajectoryElement.getValue().getCurrent().getValue().getElement().isEmpty()) {
								ArrayList<Waypoint> waypoints = new ArrayList<>();
								
								for (RouteTrajectoryElementType rte : trajectoryElement.getValue().getCurrent().getValue().getElement()) {
									if (null != rte.getPoint4D()) {
										if (null != rte.getPoint4D().getValue()) {
											if ((null != rte.getPoint4D().getValue().getPosition())
													&& (null != rte.getPoint4D().getValue().getTime())) {
												if ((null != rte.getPoint4D().getValue().getPosition().getValue())
														&& (null != rte.getPoint4D().getValue().getTime().getValue())) {
													GeographicalPositionType gp = rte.getPoint4D().getValue().getPosition().getValue();
													Point4DTimeChoiceType tc = rte.getPoint4D().getValue().getTime().getValue();
													double elevation = 0d;
													
													if (null != rte.getPoint4D().getValue().getLevel()) {
														if (null != rte.getPoint4D().getValue().getLevel().getValue()) {
															if (null != rte.getPoint4D().getValue().getLevel().getValue().getAltitude()) {
																AltitudeType altitude = rte.getPoint4D().getValue().getLevel().getValue().getAltitude();
																// TODO: altitude reference AGL versus ASL and altimeter
																elevation = Conversions.toMeters(altitude.getValue(), altitude.getUom().toString());
															}
														}
													}
													
													Waypoint waypoint = new Waypoint(Position.fromDegrees(
															gp.getPos().get(0), gp.getPos().get(1), elevation));
													
													if (null != tc.getAbsoluteTime()) {
														waypoint.setEto(ZonedDateTime.parse(tc.getAbsoluteTime().toString()));
														waypoints.add(waypoint);
													}
												}
											}
										}
									}
								}
								
								trajectory = new Trajectory(waypoints);
							}
						}
					}
				}
			}
		}
		
		return trajectory;
	}
	
}
