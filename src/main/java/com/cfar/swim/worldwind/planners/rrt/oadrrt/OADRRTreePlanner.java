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
package com.cfar.swim.worldwind.planners.rrt.oadrrt;

import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import java.time.Duration;
import java.time.ZonedDateTime;
import java.time.temporal.ChronoUnit;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.connections.Communication;
import com.cfar.swim.worldwind.connections.Datalink;
import com.cfar.swim.worldwind.connections.DatalinkCommunicator;
import com.cfar.swim.worldwind.connections.DatalinkTracker;
import com.cfar.swim.worldwind.environments.DirectedEdge;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.planners.MissionLoader;
import com.cfar.swim.worldwind.planners.OnlinePlanner;
import com.cfar.swim.worldwind.planners.rrt.adrrt.ADRRTreePlanner;
import com.cfar.swim.worldwind.planners.rrt.adrrt.ADRRTreeWaypoint;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.planning.TimeInterval;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.planners.rrt.OADRRTreeProperties;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.tracks.AircraftTrack;
import com.cfar.swim.worldwind.tracks.AircraftTrackError;
import com.cfar.swim.worldwind.tracks.AircraftTrackPoint;
import com.cfar.swim.worldwind.tracks.AircraftTrackPointError;
import com.cfar.swim.worldwind.util.Identifiable;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.util.Logging;

/**
 * Realizes an online anytime dynamic RRT planner that features a datalink
 * connection to communicate with a planned aircraft.
 * 
 * @author Stephan Heinemann
 *
 */
public class OADRRTreePlanner extends ADRRTreePlanner implements OnlinePlanner {
	
	/** the active leg of this OADRRT planner */
	private Optional<DirectedEdge> activeLeg = Optional.empty();
	
	/** the active part of this OADRRT planner */
	private int activePart = 0;
	
	/** the datalink of this OADRRT planner */
	private Datalink datalink = null;
	
	/** the datalink take-off communication of this OADRRT planner */
	private Communication<Datalink> takeOff = null;
	
	/** the datalink landing communication of this OADRRT planner */
	private Communication<Datalink> landing = null;
	
	/** the datalink unplanned landing communication of this OADRRT planner */
	private Communication<Datalink> unplannedLanding = null;
	
	/** the establish datalink communication of this OADRRT planner */
	private Communication<Datalink> establishDatalink = null;
	
	/** the minimum deliberation duration of this OADRRT planner */
	private Duration minDeliberation = Duration.ofSeconds(10l);
	
	/** the maximum deliberation duration of this OADRRT planner */
	private Duration maxDeliberation = Duration.ofSeconds(60l);
	
	/** the deliberation start of this OADRRT planner */
	private ZonedDateTime deliberationStart = null;
	
	/** the deliberation end of this OADRRT planner */
	private ZonedDateTime deliberationEnd = null;
	
	/** the maximum acceptable aircraft take-off error of this OADRRT planner */
	private AircraftTrackPointError maxTakeOffError = AircraftTrackPointError.ZERO;
	
	/** the maximum acceptable aircraft landing error of this OADRRT planner */
	private AircraftTrackPointError maxLandingError = AircraftTrackPointError.ZERO;
	
	/** the maximum acceptable aircraft track error of this OADRRT planner */
	private AircraftTrackError maxTrackError = AircraftTrackError.ZERO;
	
	/** the track change listener of this OADRRT planner */
	private final TrackChangeListener trackCl = new TrackChangeListener();
	
	/** the mission loader of this OADRRT planner */
	private final MissionLoader missionLoader = new MissionLoader(this);
	
	/** the off-track counter of this OADRRT planner */
	private int offTrackCount = 0;
	
	/**
	 * Constructs a OADRRT planner for a specified aircraft and environment
	 * using default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 */
	public OADRRTreePlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		this.addPlanRevisionListener(this.getMissionLoader());
	}
	
	/**
	 * Gets the identifier of this OADRRT planner.
	 * 
	 * @return the identifier of this OADRRT planner
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return Specification.PLANNER_OADRRT_ID;
	}
	
	/**
	 * Gets the active leg of this OADRRT planner.
	 * 
	 * @return the active leg of this OADRRT planner
	 */
	protected synchronized Optional<DirectedEdge> getActiveLeg() {
		return this.activeLeg;
	}
	
	/**
	 * Sets the active leg of this OADRRT planner.
	 * 
	 * @param activeLeg the active leg to be set
	 */
	protected synchronized void setActiveLeg(Optional<DirectedEdge> activeLeg) {
		this.activeLeg = activeLeg;
	}
	
	/**
	 * Gets the previous waypoint of this OADRRT planner.
	 * 
	 * @return the previous waypoint of this OADRRT planner
	 * 
	 * @see OnlinePlanner#getPreviousWaypoint()
	 */
	@Override
	public synchronized Waypoint getPreviousWaypoint() {
		return this.getActiveLeg().isEmpty() ? null :
			(ADRRTreeWaypoint) this.getActiveLeg().get().getFirstPosition();
	}
	
	/**
	 * Determines whether or not this OADRRT planner has a previous waypoint.
	 * 
	 * @return true if this OADRRT planner has a previous waypoint,
	 *         false otherwise
	 * 
	 * @see OnlinePlanner#hasPreviousWaypoint()
	 */
	@Override
	public synchronized boolean hasPreviousWaypoint() {
		return this.getActiveLeg().isPresent();
	}
	
	/**
	 * Gets the next waypoint of this OADRRT planner.
	 * 
	 * @return the next waypoint of this OADRRT planner
	 * 
	 * @see OnlinePlanner#getNextWaypoint()
	 */
	@Override
	public synchronized ADRRTreeWaypoint getNextWaypoint() {
		return this.getActiveLeg().isEmpty() ? null :
			(ADRRTreeWaypoint) this.getActiveLeg().get().getSecondPosition();
	}
	
	/**
	 * Determines whether or not this OADRRT planner has a next waypoint.
	 * 
	 * @return true if this OADRRT planner has a next waypoint, false otherwise
	 * 
	 * @see OnlinePlanner#hasNextWaypoint()
	 */
	@Override
	public synchronized boolean hasNextWaypoint() {
		return this.getActiveLeg().isPresent();
	}
	
	/**
	 * Gets the active part of this OADRRT planner.
	 * 
	 * @return the active part of this OADRRT planner
	 * 
	 * @see OnlinePlanner#getActivePart()
	 */
	@Override
	public synchronized int getActivePart() {
		return this.activePart;
	}
	
	/**
	 * Sets the active part of this OADRRT planner.
	 * 
	 * @param activePart the active part to be set
	 */
	protected synchronized void setActivePart(int activePart) {
		this.activePart = activePart;
	}
	
	/**
	 * Gets the datalink of this OADRRT planner.
	 * 
	 * @return the datalink of this OADRRT planner
	 * 
	 * @see DatalinkCommunicator#getDatalink()
	 */
	@Override
	public Datalink getDatalink() {
		return this.datalink;
	}
	
	/**
	 * Sets the datalink of this OADRRT planner.
	 * 
	 * @param datalink the datalink to be set
	 * 
	 * @see DatalinkCommunicator#setDatalink(Datalink)
	 */
	@Override
	public void setDatalink(Datalink datalink) {
		if (this.hasDatalink() && !this.hasTerminated()) {
			this.getDatalink().removePropertyChangeListener(this.trackCl);
		}
		this.datalink = datalink;
		if (this.hasDatalink() && !this.hasTerminated()) {
			this.getDatalink().addTrackChangeListener(this.trackCl);
		}
	}
	
	/**
	 * Determines whether or not this OADRRT planner has a datalink.
	 * 
	 * @return true if this OADRRT planner has a datalink, false otherwise
	 * 
	 * @see DatalinkCommunicator#hasDatalink()
	 */
	@Override
	public boolean hasDatalink() {
		return (null != this.datalink);
	}
	
	/**
	 * Gets the minimum deliberation duration of this OADRRT planner.
	 * 
	 * @return the minimum deliberation duration of this OADRRT planner
	 * 
	 * @see OnlinePlanner#getMinDeliberation()
	 */
	@Override
	public synchronized Duration getMinDeliberation() {
		return this.minDeliberation;
	}
	
	/**
	 * Sets the minimum deliberation duration of this OADRRT planner.
	 * 
	 * @param minDeliberation the minimum deliberation duration to be set
	 * 
	 * @throws IllegalArgumentException if the minimum deliberation is invalid
	 * 
	 * @see OnlinePlanner#setMinDeliberation(Duration)
	 */
	@Override
	public synchronized void setMinDeliberation(Duration minDeliberation) {
		if (null == minDeliberation) {
			throw new IllegalArgumentException();
		}
		this.minDeliberation = minDeliberation;
	}
	
	/**
	 * Gets the maximum deliberation duration of this OADRRT planner.
	 * 
	 * @return the maximum deliberation duration of this OADRRT planner
	 * 
	 * @see OnlinePlanner#getMaxDeliberation()
	 */
	@Override
	public synchronized Duration getMaxDeliberation() {
		return this.maxDeliberation;
	}
	
	/**
	 * Sets the maximum deliberation duration of this OADRRT planner.
	 * 
	 * @param maxDeliberation the maximum deliberation duration to be set
	 * 
	 * @throws IllegalArgumentException if the maximum deliberation is invalid
	 * 
	 * @see OnlinePlanner#setMaxDeliberation(Duration)
	 */
	@Override
	public synchronized void setMaxDeliberation(Duration maxDeliberation) {
		if (null == maxDeliberation) {
			throw new IllegalArgumentException();
		}
		this.maxDeliberation = maxDeliberation;
	}
	
	/**
	 * Gets the deliberation start of this OADRRT planner.
	 * 
	 * @return the deliberation start of this OADRRT planner
	 */
	protected ZonedDateTime getDeliberationStart() {
		return this.deliberationStart;
	}
	
	/**
	 * Sets the deliberation start of this OADRRT planner.
	 * 
	 * @param deliberationStart the deliberation start to be set
	 */
	protected void setDeliberationStart(ZonedDateTime deliberationStart) {
		this.deliberationStart = deliberationStart;
	}
	
	/**
	 * Gets the deliberation end of this OADRRT planner.
	 * 
	 * @return the deliberation end of this OADRRT planner
	 */
	protected ZonedDateTime getDeliberationEnd() {
		return this.deliberationEnd;
	}
	
	/**
	 * Sets the deliberation end of this OADRRT planner.
	 * 
	 * @param deliberationEnd the deliberation end to be set
	 */
	protected void setDeliberationEnd(ZonedDateTime deliberationEnd) {
		this.deliberationEnd = deliberationEnd;
	}
	
	/**
	 * Gets the available deliberation duration of this OADRRT planner.
	 * 
	 * @return the available deliberation duration of this OADRRT planner
	 */
	protected Duration getDeliberation() {
		Duration deliberation = this.getMaxDeliberation();
		
		// determine critical plan waypoint affected by dynamic obstacles
		Set<ADRRTreeWaypoint> affectedWaypoints = new HashSet<ADRRTreeWaypoint>();
		for (Obstacle obstacle : this.getDynamicObstacles()) {
			affectedWaypoints.addAll(this.getPlanningContinuum()
					.findAffectedEdges(obstacle).stream()
					.filter(edge -> obstacle.getCostInterval()
					.intersects(new TimeInterval(
						((ADRRTreeWaypoint) edge.getFirstPosition()).getEto(),
						((ADRRTreeWaypoint) edge.getSecondPosition()).getEto())))
					.map(edge -> (ADRRTreeWaypoint) edge.getSecondPosition())
					.collect(Collectors.toSet()));
		}
		
		Optional<ADRRTreeWaypoint> criticalWaypoint = affectedWaypoints.stream()
				.filter(w -> this.getWaypoints().contains(w))
				.min(new Comparator<ADRRTreeWaypoint>() {
					@Override
					public int compare(ADRRTreeWaypoint w1, ADRRTreeWaypoint w2) {
						return w1.getEto().compareTo(w2.getEto());
					}
				});
		
		// determine if critical waypoint features higher cost
		if (criticalWaypoint.isPresent()) {
			ADRRTreeWaypoint target = criticalWaypoint.get();
			if (target.hasParent()) {
				ADRRTreeWaypoint source = target.getParent();
				double cost = this.getEnvironment().getLegCost(
						source, target, source.getEto(), target.getEto(),
						this.getCostPolicy(), this.getRiskPolicy());
				if ((source.getCost() + cost) > target.getCost()) {
					// compute available deliberation time
					ZonedDateTime minDeliberationEnd = this.getDeliberationStart()
							.plus(this.getMinDeliberation()
							.plus(this.getMaxTrackError().getTimingError()
							.plus(Duration.ofSeconds(
									Math.round(this.getAircraft().getRadius() /
									this.getAircraft().getCapabilities().getCruiseSpeed())))));
					ZonedDateTime maxDeliberationEnd = this.getDeliberationStart()
							.plus(this.getMaxDeliberation()
							.plus(this.getMaxTrackError().getTimingError()
							.plus(Duration.ofSeconds(
									Math.round(this.getAircraft().getRadius() /
									this.getAircraft().getCapabilities().getCruiseSpeed())))));
				
					ZonedDateTime criticalDeliberationEnd = criticalWaypoint.get().getParent().getEto();
					if (criticalDeliberationEnd.isBefore(minDeliberationEnd)) {
						deliberation = this.getMinDeliberation();
					} else if (criticalDeliberationEnd.isBefore(maxDeliberationEnd)) {
						deliberation = Duration.between(
								this.getDeliberationStart(),
								criticalDeliberationEnd);
					}
				}
			}
		}
		
		Logging.logger().info("deliberation duration = " + deliberation);
		return deliberation;
	}
	
	/**
	 * Determines if this OADRRT planner can deliberate.
	 * 
	 * @return true if this OADRRT planner can deliberate, false otherwise
	 */
	protected boolean canDeliberate() {
		boolean canDeliberate = true;
		
		if (null != this.getDeliberationEnd()) {
			return (0 < this.getDeliberationEnd().compareTo(ZonedDateTime.now()));
		}
		
		return canDeliberate;
	}
	
	/**
	 * Gets the maximum acceptable take-off error of this OADRRT planner to
	 * consider the aircraft within the take-off regime.
	 * 
	 * @return the maximum acceptable take-off error of this OADRRT planner to
	 *         consider the aircraft within the take-off regime
	 *
	 * @see DatalinkTracker#getMaxTakeOffError()
	 */
	@Override
	public synchronized AircraftTrackPointError getMaxTakeOffError() {
		return this.maxTakeOffError;
	}
	
	/**
	 * Sets the maximum acceptable take-off error of this OADRRT planner to
	 * consider the aircraft within the take-off regime.
	 * 
	 * @param maxTakeOffError the maximum acceptable take-off error to be set
	 *
	 * @throws IllegalArgumentException if the maximum take-off error is null
	 *
	 * @see DatalinkTracker#setMaxTakeOffError(AircraftTrackPointError)
	 */
	@Override
	public synchronized void setMaxTakeOffError(AircraftTrackPointError maxTakeOffError) {
		if (null == maxTakeOffError) {
			throw new IllegalArgumentException();
		}
		this.maxTakeOffError = maxTakeOffError;
	}
	
	/**
	 * Determines whether or not the aircraft of this OADRRT planner is within
	 * the take-off zone.
	 * 
	 * @return true if the aircraft of this OADRRT planner is within the
	 *         take-off zone, false otherwise
	 *
	 * @see DatalinkTracker#isInTakeOffZone()
	 */
	@Override
	public boolean isInTakeOffZone() {
		boolean isInTakeOffZone = false;
		
		if (this.hasDatalink() && this.getDatalink().isConnected()
				&& this.getDatalink().isMonitoring()) {
			AircraftTrack track = this.getDatalink().getAircraftTrack();
			
			if (!track.isEmpty()) {
				AircraftTrackPoint last = track.getLastTrackPoint();
				Position start = this.getStart().getPrecisionPosition().getOriginal();
				
				// horizontal error
				Position spos = new Position(start, 0d);
				Position lpos = new Position(last.getPosition(), 0d);
				double dhor = this.getEnvironment().getGreatCircleDistance(spos, lpos);
				
				// vertical error
				double dver = Math.abs(start.getElevation()
						- last.getPosition().getElevation());
				
				// position error
				Angle dlat = start.getLatitude()
						.angularDistanceTo(last.getPosition().getLatitude());
				Angle dlon = start.getLongitude()
						.angularDistanceTo(last.getPosition().getLongitude());
				
				isInTakeOffZone = (this.getMaxTakeOffError().getHorizontalError() >= dhor)
						&& (this.getMaxTakeOffError().getVerticalError() >= dver)
						&& (this.getMaxTakeOffError().getPositionError().getElevation() >= dver)
						&& (0 >= dlat.compareTo(this.getMaxTakeOffError().getPositionError().getLatitude()))
						&& (0 >= dlon.compareTo(this.getMaxTakeOffError().getPositionError().getLongitude()));
			}
		}
		
		return isInTakeOffZone;
	}
	
	/**
	 * Determines whether or not the aircraft of this OADRRT planner is within
	 * the take-off window.
	 * 
	 * @return true if the aircraft of this OADRRT planner is within the
	 *         take-off window, false otherwise
	 *
	 * @see DatalinkTracker#isInTakeOffWindow()
	 */
	@Override
	public boolean isInTakeOffWindow() {
		boolean isInTakeOffWindow = false;
		
		if (this.hasDatalink() && this.getDatalink().isConnected()
				&& this.getDatalink().isMonitoring()) {
			AircraftTrack track = this.getDatalink().getAircraftTrack();
			
			if (!track.isEmpty()) {
				AircraftTrackPoint last = track.getLastTrackPoint();
				
				// calculate take-off window with take-off error margin
				ZonedDateTime takeOffTime = this.getStart().getEto();
				TimeInterval takeOffWindow = new TimeInterval(
						takeOffTime.minus(this.getMaxTakeOffError()
								.getTimingError().getSeconds(), ChronoUnit.SECONDS),
						takeOffTime.plus(this.getMaxTakeOffError()
								.getTimingError().getSeconds(), ChronoUnit.SECONDS));
				
				isInTakeOffWindow = takeOffWindow.contains(last.getAto());
			}
		}
		
		return isInTakeOffWindow;
	}
	
	/**
	 * Gets the maximum acceptable landing error of this OADRRT planner to
	 * consider the aircraft within the landing regime.
	 * 
	 * @return the maximum acceptable landing error of this OADRRT planner to
	 *         consider the aircraft within the landing regime
	 *
	 * @see DatalinkTracker#getMaxLandingError()
	 */
	@Override
	public synchronized AircraftTrackPointError getMaxLandingError() {
		return this.maxLandingError;
	}
	
	/**
	 * Sets the maximum acceptable landing error of this OADRRT planner to
	 * consider the aircraft within the landing regime.
	 * 
	 * @param maxLandingError the maximum acceptable landing error to be set
	 *
	 * @throws IllegalArgumentException if the maximum landing error is null
	 *
	 * @see DatalinkTracker#setMaxLandingError(AircraftTrackPointError)
	 */
	@Override
	public synchronized void setMaxLandingError(AircraftTrackPointError maxLandingError) {
		if (null == maxLandingError) {
			throw new IllegalArgumentException();
		}
		this.maxLandingError = maxLandingError;
	}
	
	/**
	 * Determines whether or not the aircraft of this OADRRT planner is within
	 * the landing zone.
	 * 
	 * @return true if the aircraft of this OADRRT planner is within the
	 *         landing zone, false otherwise
	 *
	 * @see DatalinkTracker#isInLandingZone()
	 */
	@Override
	public boolean isInLandingZone() {
		boolean isInLandingZone = false;
		
		if (this.hasDatalink() && this.getDatalink().isConnected()
				&& this.getDatalink().isMonitoring()) {
			AircraftTrack track = this.getDatalink().getAircraftTrack();
			
			if (!track.isEmpty()) {
				AircraftTrackPoint last = track.getLastTrackPoint();
				Position goal = this.getGoal().getPrecisionPosition().getOriginal();
				
				// horizontal error
				Position gpos = new Position(goal, 0d);
				Position lpos = new Position(last.getPosition(), 0d);
				double dhor = this.getEnvironment().getGreatCircleDistance(gpos, lpos);
				
				// vertical error
				double dver = Math.abs(goal.getElevation()
						- last.getPosition().getElevation());
				
				// position error
				Angle dlat = goal.getLatitude()
						.angularDistanceTo(last.getPosition().getLatitude());
				Angle dlon = goal.getLongitude()
						.angularDistanceTo(last.getPosition().getLongitude());
				
				isInLandingZone = (this.getMaxLandingError().getHorizontalError() >= dhor)
						&& (this.getMaxLandingError().getVerticalError() >= dver)
						&& (this.getMaxLandingError().getPositionError().getElevation() >= dver)
						&& (0 >= dlat.compareTo(this.getMaxLandingError().getPositionError().getLatitude()))
						&& (0 >= dlon.compareTo(this.getMaxLandingError().getPositionError().getLongitude()));
			}
		}
		
		return isInLandingZone;
	}
	
	/**
	 * Determines whether or not the aircraft of this OADRRT planner is within
	 * the landing window.
	 * 
	 * @return true if the aircraft of this OADRRT planner is within the
	 *         landing window, false otherwise
	 *
	 * @see DatalinkTracker#isInLandingWindow()
	 */
	@Override
	public boolean isInLandingWindow() {
		boolean isInLandingWindow = false;
		
		if (this.hasDatalink() && this.getDatalink().isConnected()
				&& this.getDatalink().isMonitoring()) {
			AircraftTrack track = this.getDatalink().getAircraftTrack();
			
			if (!track.isEmpty()) {
				AircraftTrackPoint last = track.getLastTrackPoint();
			
				// calculate landing window with landing error margin
				ZonedDateTime landingTime = this.getGoal().getEto();
				TimeInterval landingWindow = new TimeInterval(
						landingTime.minus(this.getMaxLandingError()
								.getTimingError().getSeconds(), ChronoUnit.SECONDS),
						landingTime.plus(this.getMaxLandingError()
								.getTimingError().getSeconds(), ChronoUnit.SECONDS));
				isInLandingWindow = landingWindow.contains(last.getAto());
			}
		}
		
		return isInLandingWindow;
	}
	
	/**
	 * Gets the maximum acceptable track error of this OADRRT planner to
	 * consider the aircraft on track.
	 * 
	 * @return the maximum acceptable track error of this OADRRT planner to
	 *         consider the aircraft on track
	 * 
	 * @see DatalinkTracker#getMaxTrackError()
	 */
	@Override
	public synchronized AircraftTrackError getMaxTrackError() {
		return this.maxTrackError;
	}
	
	/**
	 * Sets the maximum acceptable track error of this OADRRT planner to
	 * consider the aircraft on track.
	 * 
	 * @param maxTrackError the maximum acceptable track error to be set
	 * 
	 * @see DatalinkTracker#setMaxTrackError(AircraftTrackError)
	 */
	@Override
	public synchronized void setMaxTrackError(AircraftTrackError maxTrackError) {
		if (null == maxTrackError) {
			throw new IllegalArgumentException();
		}
		this.maxTrackError = maxTrackError;
	}
	
	/**
	 * Realizes a track change listener.
	 * 
	 * @author Stephan Heinemann
	 *
	 */
	private class TrackChangeListener implements PropertyChangeListener {
		
		/**
		 * Notifies the OADRRT planner about a track change.
		 * 
		 * @param evt the property change event
		 * 
		 * @see PropertyChangeListener#propertyChange(PropertyChangeEvent)
		 */
		@Override
		public void propertyChange(PropertyChangeEvent evt) {
			notifyProgress();
		}
	}
	
	/**
	 * Determines whether or not the aircraft of this OADRRT planner is on
	 * track.
	 * 
	 * @return true if the aircraft of this OADRRT planner is on track,
	 *         false otherwise
	 * 
	 * @see DatalinkTracker#isOnTrack()
	 */
	@Override
	public boolean isOnTrack() {
		boolean isOnTrack = true;
		
		if (this.getActiveLeg().isPresent()
				&& this.hasDatalink() && this.getDatalink().isConnected()
				&& this.getDatalink().isMonitoring()
				&& this.getDatalink().isAirborne()) {
			
			AircraftTrack track = this.getDatalink().getAircraftTrack();
			
			if (!track.isEmpty()) {
				AircraftTrackPoint last = track.getLastTrackPoint();
				AircraftTrackPoint first = track.getFirstTrackPoint();
				
				// ensure sufficient track points for on-track assessment
				long maxAge = last.getMaxAge().toMillis();
				long period = this.getDatalink().getDownlinkPeriod().toMillis();
				long maxTrackPoints = maxAge / period;
				
				if (track.size() >= (maxTrackPoints - 1)) {
					DirectedEdge leg = this.getActiveLeg().get();
					ADRRTreeWaypoint next = (ADRRTreeWaypoint) leg.getSecondPosition();
					
					// cross track check
					ADRRTreeWaypoint lastPosition = new ADRRTreeWaypoint(last.getPosition());
					double hced = leg.getHorizontalCrossEdgeDistance(
							lastPosition.getPrecisionPosition().getOriginal());
					double vced = leg.getVerticalCrossEdgeDistance(
							lastPosition.getPrecisionPosition().getOriginal());
					Angle ob = leg.getOpeningBearing(
							lastPosition.getPrecisionPosition().getOriginal());
					Angle cb = leg.getClosingBearing(
							lastPosition.getPrecisionPosition().getOriginal());
					Logging.logger().info("xtd = " + hced + ", ob = " + ob + ", cb = " + cb);
					
					// ground speed check
					Duration dt = Duration.between(first.getAto(), last.getAto());
					double s = this.getEnvironment().getDistance(track.getPositions());
					double gs = s / dt.getSeconds();
					Logging.logger().info("dt = " + dt + ", s = " + s + ", gs =" + gs);
					
					// ETO update
					Duration deto = Duration.ZERO;
					if (0 != gs) {
						// TODO: ground speed could be averaged along the leg
						double dtg = this.getEnvironment().getDistance(
								lastPosition.getPrecisionPosition().getOriginal(),
								next.getPrecisionPosition().getOriginal());
						Duration ttg = Duration.ofSeconds((long) (dtg / gs));
						Logging.logger().info("dtg = " + dtg + ", ttg = " + ttg);
						ZonedDateTime eto = last.getAto().plus(ttg);
						deto = Duration.between(next.getEto(), eto);//.abs();
						Logging.logger().info("eto = " + eto + ", deto = " + deto);
					} else {
						deto = this.getMaxTrackError().getTimingError().plusSeconds(1);
					}
					
					isOnTrack = (this.getMaxTrackError().getCrossTrackError() >= hced)
							&& (this.getMaxTrackError().getAltitudeError() >= vced)
							&& (0 <= this.getMaxTrackError().getOpeningBearingError().compareTo(ob))
							&& (0 <= this.getMaxTrackError().getClosingBearingError().compareTo(cb))
							&& (0 <= this.getMaxTrackError().getTimingError().compareTo(deto));
					
					// determine off-track situation based on all relevant track points
					if (isOnTrack) {
						this.offTrackCount = 0;
					} else {
						this.offTrackCount++;
						if (maxTrackPoints > this.offTrackCount) {
							isOnTrack = true;
						} else {
							Logging.logger().warning("the aircraft is off-track...");
						}
					}
					Logging.logger().info("off-track count is " + this.offTrackCount + " of " + maxTrackPoints);
					
				} else {
					Logging.logger().info("insufficient datalink track points = " + track.size());
				}
			} else {
				Logging.logger().warning("empty datalink track...");
			}
		}
		
		return isOnTrack;
	}
	
	/**
	 * Initializes this OADRRT planner to plan from an origin to a destination at
	 * a specified estimated time of departure.
	 * 
	 * @param origin the origin in globe coordinates
	 * @param destination the destination in globe coordinates
	 * @param etd the estimated time of departure
	 */
	@Override
	protected synchronized void initialize(Position origin, Position destination, ZonedDateTime etd) {
		super.initialize(origin, destination, etd);
		this.offTrackCount = 0;
		if (!this.hasDatalink() || !this.getDatalink().isAirborne()) {
			this.setActiveLeg(Optional.empty());
			this.setActivePart(0);
		}
	}
	
	/**
	 * Roots a branch of the tree generated by this OADRRT planner by removing
	 * all parents and siblings.
	 * 
	 * @param root the root of the new tree
	 */
	protected void root(ADRRTreeWaypoint root) {
		this.setStart(root);
		
		if (root.hasParent()) {
			ADRRTreeWaypoint parent = root.getParent();
			parent.removeChild(root);
			root.removeParent();
			while (parent.hasParent()) {
				parent = parent.getParent();
			}
			this.trim(parent);
		}
	}
	
	/**
	 * Progresses the plan of this OADRRT planner by rooting its generated tree
	 * at the next mission position.
	 * 
	 * @param partIndex the index of the part to be progressed
	 */
	protected synchronized void progress(int partIndex) {
		if (this.hasStart()
				&& this.hasDatalink() && this.getDatalink().isConnected()
				&& this.getDatalink().isMonitoring()
				&& this.getDatalink().hasMission(true)) {
			// TODO: has uploaded planner mission versus just any mission
			
			if (this.getActivePart() < partIndex) {
				this.backup(partIndex);
				this.restore(partIndex - 1);
				this.progress(partIndex - 1);
				this.backup(partIndex - 1);
				this.restore(partIndex);
			} else {
				ADRRTreeWaypoint nextDatalinkWaypoint = new ADRRTreeWaypoint(
						this.getDatalink().getNextMissionPosition());
				
				// progress planner tree
				while (!this.getStart().equals(nextDatalinkWaypoint) &&
						(this.getWaypoints().contains(nextDatalinkWaypoint)
								|| (1 == this.getWaypoints().size()))) {
					Logging.logger().info("progressing from " + this.getStart()
					+ " to " + nextDatalinkWaypoint);
					ADRRTreeWaypoint previous = (ADRRTreeWaypoint) this.getWaypoints().removeFirst();
					if (this.hasWaypoints()) {
						this.setActiveLeg(Optional.of(new DirectedEdge(
								this.getEnvironment(), previous, this.getWaypoints().getFirst())));
						this.setActivePart(partIndex);
						this.root((ADRRTreeWaypoint) this.getWaypoints().getFirst());
						Logging.logger().info("new active leg is "
								+ this.getActiveLeg().get().toString());
					} else {
						this.setActivePart(partIndex + 1);
						Logging.logger().info(this.getId() + " has an empty part " + partIndex);
					}
				}
				
				if (this.hasWaypoints()) {
					Logging.logger().info("part " + partIndex + ": " + this.getWaypoints());
				} else {
					Logging.logger().warning("part " + partIndex + " has an empty plan");
				}
				
				// check take-off and landing conditions
				if (this.isLastIndex(partIndex)
						&& this.getDatalink().isAirborne()
						&& this.isInLandingZone()) {
					// TODO: landing versus approach window for fixed wing
					// aircraft with approach and landing procedure
					if (this.isInLandingWindow()) {
						this.performLanding();
					} else {
						this.performUnplannedLanding();
					}
				} else if ((0 == partIndex)
						&& !this.getDatalink().isAirborne()
						&& this.isInTakeOffZone()
						&& this.hasWaypoints()) {
					if (this.isInTakeOffWindow()) {
						this.performTakeOff();
					}
				}
			}
		} else if (!this.hasDatalink()
				|| !this.getDatalink().isConnected()
				|| !this.getDatalink().isMonitoring()) {
			this.establishDatalink();
		}
	}
	
	/**
	 * Re-plans an off-track aircraft of this OADRRT planner.
	 * 
	 * @param partIndex the of the plan to be re-planned
	 */
	protected void replan(int partIndex) {
		// TODO: possibly always re-plan from scratch based on track point
		ADRRTreeWaypoint partStart = (ADRRTreeWaypoint) this.getStart().clone();
		
		if (this.getActivePart() < partIndex) {
			this.backup(partIndex);
			this.restore(partIndex - 1);
			this.replan(partIndex - 1);
			partStart.setCost(this.getGoal().getCost());
			partStart.setEto(this.getGoal().getEto());
			partStart.setParent(this.getGoal().getParent());
			this.backup(partIndex - 1);
			this.restore(partIndex);
		} else {
			// naively re-plan based on aircraft capabilities
			AircraftTrack track = this.getDatalink().getAircraftTrack();
			
			if (!track.isEmpty()) {
				AircraftTrackPoint trackPoint = track.getLastTrackPoint();
				ADRRTreeWaypoint offTrackWaypoint = new ADRRTreeWaypoint(trackPoint.getPosition());
				
				if (!this.getStart().equals(offTrackWaypoint)) {
					offTrackWaypoint.setEto(trackPoint.getAto());
					this.computeEto(offTrackWaypoint, this.getStart());
					this.setActiveLeg(Optional.of(new DirectedEdge(
							this.getEnvironment(), offTrackWaypoint, this.getStart())));
				} else {
					this.setActiveLeg(Optional.empty());
				}
			} else {
				Logging.logger().warning("empty datalink track...");
				this.setActiveLeg(Optional.empty());
			}
			
			// plan active part from scratch
			this.initialize(this.getStart(), this.getGoal(), this.getStart().getEto());
			this.getStart().setPoi(partStart.isPoi());
			this.getStart().setCost(partStart.getCost());
			this.backups.get(partIndex).clear();
			this.planPart(partIndex);
			partStart.setEto(this.getStart().getEto());
			Logging.logger().info("replanned part " + partIndex + ": " + this.getWaypoints());
		}
		
		if (partStart.hasInfiniteCost()) {
			// invalid previous part
			this.getGoal().setInfiniteCost();
			Logging.logger().warning("replanning part " + partIndex + " was unsuccessful");
		} else if (!this.getStart().getEto().equals(partStart.getEto())) {
			// TODO: slots versus permissible aircraft track point error
			// plan current part from scratch if start ETO has changed
			this.initialize(this.getStart(), this.getGoal(), partStart.getEto());
			this.getStart().setCost(partStart.getCost());
			if (partStart.hasParent()) {
				this.getStart().setParent(partStart.getParent());
			}
			this.backups.get(partIndex).clear();
			this.planPart(partIndex);
			Logging.logger().info("replanned part " + partIndex + ": " + this.getWaypoints());
		} else {
			// propagate potential cost change
			double deltaCost = partStart.getCost() - this.getStart().getCost();
			if (0 != deltaCost) {
				for (Position vertex : this.getPlanningContinuum().getVertices()) {
					ADRRTreeWaypoint waypoint = (ADRRTreeWaypoint) vertex;
					waypoint.setCost(waypoint.getCost() + deltaCost);
				}
			}
			
			if (partStart.hasParent()) {
				this.getStart().setParent(partStart.getParent());
			}
			Logging.logger().info("adjusted part " + partIndex + ": " + this.getWaypoints());
		}
	}
	
	/**
	 * Elaborates an OADRRT plan.
	 * 
	 * @param partIndex the index of the plan to be elaborated
	 */
	@Override
	protected void elaborate(int partIndex) {
		// limit deliberation for active part
		if (partIndex == this.getActivePart()) {
			this.setDeliberationStart(ZonedDateTime.now());
			this.setDeliberationEnd(this.getDeliberationStart().plus(this.getDeliberation()));
		} else {
			this.setDeliberationEnd(null);
		}
		
		// TODO: off-track elaboration: re-plan or control?
		// (1) re-planning from next mission or track point position
		// based on static or dynamic performance
		// (2) control adjusting speed to recover track
		// TODO: consider adding off-track notification callback
		if (!this.isOnTrack()) {
			this.offTrackCount = 0;
			this.replan(partIndex);
		}
		
		// do not elaborate an exceeded risk policy solution beyond limit
		int riskyProbes = (this.getGoal().hasInfiniteCost()) ? 1 : 0;
		
		// proceed to next part only if fully improved and not in need of repair
		while (this.canDeliberate() && !this.hasTerminated()
				&& ((!this.hasMaximumQuality() && (this.getMaxRiskyProbes() > riskyProbes))
				|| this.needsRepair())) {
			// TODO: override repair to start at deliberation waypoint limit
			this.repair(partIndex);
			// TODO: proceed with alternate or unplanned landing if repair was unsuccessful
			this.improve(partIndex);
			this.progress(partIndex);
			if (this.getGoal().hasInfiniteCost()) {
				riskyProbes++;
			} else {
				riskyProbes = 0;
			}
			Thread.yield();
			this.updateDynamicObstacles();
			// TODO: update deliberation times?
		}
		// backup after elaboration
		this.backup(partIndex);
	}
	
	/**
	 * Computes an OADRRT plan applying the current cost and goal biases by
	 * growing a tree until the goal region is reached. The risk policy will
	 * be boosted if necessary when airborne to continue temporarily under
	 * higher risk.
	 * 
	 * @return true if the goal region was reached, false otherwise
	 */
	@Override
	protected boolean compute() {
		if (this.getStart().equals(this.getGoal())) {
			this.connectPlan(this.getStart());
			return true;
		}
		
		RiskPolicy riskPolicy = this.getRiskPolicy();
		
		// boost risk policy temporarily in flight if necessary
		if (this.hasDatalink()
				&& this.getDatalink().isConnected()
				&& this.getDatalink().isMonitoring()
				&& this.getDatalink().isAirborne()) {
			
			// permit escaping local in-flight hazards with boosted risk policy
			double distance = this.getEnvironment().getGreatCircleDistance(
					this.getStart(), this.getGoal());
			double radius = this.getAircraft().getRadius();
			
			if (0d < distance) {
				double ratio  = radius / distance;
				ADRRTreeWaypoint end = new ADRRTreeWaypoint(
						Position.interpolateGreatCircle(
								ratio, this.getStart(), this.getGoal()));
				this.computeEto(this.getStart(), end);
				DirectedEdge leg = new DirectedEdge(
						this.getEnvironment(), this.getStart(), end);
				double cost = leg.getCost(
						this.getStart().getEto(), end.getEto(),
						CostPolicy.MAXIMUM, RiskPolicy.INSANITY);
				if (!this.getRiskPolicy().satisfies(cost)) {
					this.setRiskPolicy(RiskPolicy.adjustTo(cost));
					Logging.logger().warning("the risk policy has been boosted...");
					// TODO: consider risk policy boost notification callback
					// TODO: boosting may lead to higher risk exposure beyond threat
				}
			}
		}
		
		boolean isInGoalRegion = super.compute();
		
		// TODO: plan for in-flight alternates and / or issue warning if no solution
		// TODO: alternate and precautionary landing communications
		
		this.setRiskPolicy(riskPolicy);
		return isInGoalRegion;
	}
	
	/**
	 * Shares dynamic obstacles among all active OADRRT backups for dynamic repair.
	 * 
	 * @param dynamicObstacles the dynamic obstacles to be shared
	 */
	@Override
	protected void shareDynamicObstacles(Set<Obstacle> dynamicObstacles) {
		for (int backupIndex = this.getActivePart(); backupIndex < this.backups.size(); backupIndex++) {
			if (this.hasBackup(backupIndex)) {
				Backup backup = (Backup) this.backups.get(backupIndex);
				backup.dynamicObstacles.addAll(dynamicObstacles);
			}
		}
	}
	
	/**
	 * Suspends this OADRRT planner until termination, context changes or
	 * off-track situations occur.
	 */
	@Override
	protected synchronized void suspend() {
		try {
			// wait for termination, dynamic changes, or off-track situations
			while (!this.hasTerminated() && !this.needsRepair() && this.isOnTrack()) {
				this.progress(this.backups.size() - 1);
				this.wait();
				this.updateDynamicObstacles();
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * Notifies this OADRRT planner about progress of the aircraft.
	 */
	protected synchronized void notifyProgress() {
		this.notifyAll();
	}
	
	/**
	 * Terminates this OADRRT planner and removes its downlink listener.
	 * 
	 * @see ADRRTreePlanner#terminate()
	 */
	@Override
	public synchronized void terminate() {
		super.terminate();
		if (this.hasDatalink()) {
			this.getDatalink().removePropertyChangeListener(this.trackCl);
		}
	}
	
	/**
	 * Recycles this OADRRT planner and adds its downlink listener.
	 * 
	 * @see ADRRTreePlanner#recycle()
	 */
	@Override
	public synchronized void recycle() {
		super.recycle();
		if (this.hasDatalink()) {
			this.getDatalink().addTrackChangeListener(this.trackCl);
		}
	}
	
	/**
	 * Gets the mission loader of this OADRRT planner.
	 * 
	 * @return the mission loader of this OADRRT planner
	 * 
	 * @see OnlinePlanner#getMissionLoader()
	 */
	@Override
	public MissionLoader getMissionLoader() {
		return this.missionLoader;
	}
	
	/**
	 * Gets the datalink take-off communication of this OADRRT planner.
	 * 
	 * @return the datalink take-off communication of this OADRRT planner
	 * 
	 * @see DatalinkCommunicator#getTakeOff()
	 */
	@Override
	public Communication<Datalink> getTakeOff() {
		return this.takeOff;
	}
	
	/**
	 * Sets the datalink take-off communication of this OADRRT planner.
	 * 
	 * @param takeOff the datalink take-off communication to be set
	 *
	 * @see DatalinkCommunicator#setTakeOff(Communication)
	 */
	@Override
	public void setTakeOff(Communication<Datalink> takeOff) {
		this.takeOff = takeOff;
	}
	
	/**
	 * Determines whether or not this OADRRT planner has a datalink take-off
	 * communication.
	 * 
	 * @return true if this OADRRT planner has a datalink take-off
	 *         communication, false otherwise
	 *
	 * @see DatalinkCommunicator#hasTakeOff()
	 */
	@Override
	public boolean hasTakeOff() {
		return (null != this.takeOff);
	}
	
	/**
	 * Gets the datalink landing communication of this OADRRT planner.
	 * 
	 * @return the datalink landing communication of this OADRRT planner
	 *
	 * @see DatalinkCommunicator#getLanding()
	 */
	@Override
	public Communication<Datalink> getLanding() {
		return this.landing;
	}
	
	/**
	 * Sets the datalink landing communication of this OADRRT planner.
	 * 
	 * @param landing the datalink landing communication to be set
	 *
	 * @see DatalinkCommunicator#setLanding(Communication)
	 */
	@Override
	public void setLanding(Communication<Datalink> landing) {
		this.landing = landing;
	}
	
	/**
	 * Determines whether or not this OADRRT planner has a datalink landing
	 * communication.
	 * 
	 * @return true if this OADRRT planner has a datalink landing
	 *         communication, false otherwise
	 *
	 * @see DatalinkCommunicator#hasLanding() 
	 */
	@Override
	public boolean hasLanding() {
		return (null != this.landing);
	}
	
	/**
	 * Gets the datalink unplanned landing communication of this OADRRT
	 * planner.
	 * 
	 * @return the datalink unplanned landing communication of this OADRRT
	 *         planner
	 *
	 * @see DatalinkCommunicator#getUnplannedLanding()
	 */
	@Override
	public Communication<Datalink> getUnplannedLanding() {
		return this.unplannedLanding;
	}
	
	/**
	 * Sets the datalink unplanned landing communication of this OADRRT
	 * planner.
	 * 
	 * @param unplannedLanding the datalink unplanned landing communication to
	 *                         be set
	 *
	 * @see DatalinkCommunicator#setUnplannedLanding(Communication)
	 */
	@Override
	public void setUnplannedLanding(Communication<Datalink> unplannedLanding) {
		this.unplannedLanding = unplannedLanding;
	}
	
	/**
	 * Determines whether or not this OADRRT planner has a datalink unplanned
	 * landing communication.
	 * 
	 * @return true if this OADRRT planner has a datalink unplanned landing
	 *         communication, false otherwise
	 *
	 * @see DatalinkCommunicator#hasUnplannedLanding()
	 */
	@Override
	public boolean hasUnplannedLanding() {
		return (null != this.unplannedLanding);
	}
	
	/**
	 * Gets the establish datalink communication of this OADRRT planner.
	 * 
	 * @return the establish datalink communication of this OADRRT planner
	 *
	 * @see DatalinkCommunicator#getEstablishDataLink()
	 */
	@Override
	public Communication<Datalink> getEstablishDataLink() {
		return this.establishDatalink;
	}
	
	/**
	 * Sets the establish datalink communication of this OADRRT planner.
	 * 
	 * @param establishDatalink the establish datalink communication to be set
	 *
	 * @see DatalinkCommunicator#setEstablishDatalink(Communication)
	 */
	@Override
	public void setEstablishDatalink(Communication<Datalink> establishDatalink) {
		this.establishDatalink = establishDatalink;
	}
	
	/**
	 * Determines whether or not this OADRRT planner has an establish datalink
	 * communication.
	 * 
	 * @return true if this OADRRT planner has an establish datalink
	 *         communication, false otherwise
	 *
	 * @see DatalinkCommunicator#hasEstablishDatalink()
	 */
	@Override
	public boolean hasEstablishDatalink() {
		return (null != this.establishDatalink);
	}
	
	/**
	 * Performs a take-off via the datalink communication of this OADRRT
	 * planner.
	 */
	protected void performTakeOff() {
		if (this.hasTakeOff()) {
			this.takeOff.perform();
		}
	}
	
	/**
	 * Performs a landing via the datalink communication of this OADRRT
	 * planner.
	 */
	protected void performLanding() {
		if (this.hasLanding()) {
			this.landing.perform();
		}
	}
	
	/**
	 * Performs an unplanned landing via the datalink communication of this
	 * OADRRT planner.
	 */
	protected void performUnplannedLanding() {
		if (this.hasUnplannedLanding()) {
			this.unplannedLanding.perform();
		}
	}
	
	/**
	 * Establishes the datalink communication of this OADRRT planner.
	 */
	protected void establishDatalink() {
		if (this.hasEstablishDatalink()) {
			this.establishDatalink.perform();
		}
	}
	
	/**
	 * Determines whether or not this OADRRT planner matches a specification.
	 * 
	 * @param specification the specification to be matched
	 * 
	 * @return true if the this OADRRT planner matches the specification,
	 *         false otherwise
	 * 
	 * @see ADRRTreePlanner#matches(Specification)
	 */
	@Override
	public synchronized boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = super.matches(specification);
		
		if (matches && (specification.getProperties() instanceof OADRRTreeProperties)) {
			OADRRTreeProperties properties = (OADRRTreeProperties) specification.getProperties();
			matches = (this.getMinDeliberation().equals(Duration.ofSeconds(properties.getMinDeliberation())))
					&& (this.getMaxDeliberation().equals(Duration.ofSeconds(properties.getMaxDeliberation())))
					&& (this.getMaxTrackError().getCrossTrackError() == properties.getMaxCrossTrackError())
					&& (this.getMaxTrackError().getTimingError().equals(Duration.ofSeconds(properties.getMaxTimingError())))
					&& (this.getMaxTakeOffError().getHorizontalError() == properties.getMaxTakeOffHorizontalError())
					&& (this.getMaxTakeOffError().getTimingError().equals(Duration.ofSeconds(properties.getMaxTakeOffTimingError())))
					&& (this.getMaxLandingError().getHorizontalError() == properties.getMaxLandingHorizontalError())
					&& (this.getMaxLandingError().getTimingError().equals(Duration.ofSeconds(properties.getMaxLandingTimingError())));
		}
		
		return matches;
	}
	
	/**
	 * Updates this OADRRT planner according to a specification.
	 * 
	 * @param specification the specification to be used for the update
	 * 
	 * @return true if this OADRRT planner has been updated, false otherwise
	 * 
	 * @see ADRRTreePlanner#update(Specification)
	 */
	@Override
	public synchronized boolean update(Specification<? extends FactoryProduct> specification) {
		boolean updated = super.update(specification);
		
		if (updated && (specification.getProperties() instanceof OADRRTreeProperties)) {
			OADRRTreeProperties properties = (OADRRTreeProperties) specification.getProperties();
			this.setMinDeliberation(Duration.ofSeconds(properties.getMinDeliberation()));
			this.setMaxDeliberation(Duration.ofSeconds(properties.getMaxDeliberation()));
			this.getMaxTrackError().setCrossTrackError(properties.getMaxCrossTrackError());
			this.getMaxTrackError().setTimingError(Duration.ofSeconds(properties.getMaxTimingError()));
			this.getMaxTakeOffError().setHorizontalError(properties.getMaxTakeOffHorizontalError());
			this.getMaxTakeOffError().setTimingError(Duration.ofSeconds(properties.getMaxTakeOffTimingError()));
			this.getMaxLandingError().setHorizontalError(properties.getMaxLandingHorizontalError());
			this.getMaxLandingError().setTimingError(Duration.ofSeconds(properties.getMaxLandingTimingError()));
		}
		
		return updated;
	}
	
}
