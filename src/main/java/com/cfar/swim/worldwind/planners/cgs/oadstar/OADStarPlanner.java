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
package com.cfar.swim.worldwind.planners.cgs.oadstar;

import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import java.time.Duration;
import java.time.ZonedDateTime;
import java.time.temporal.ChronoUnit;
import java.util.Optional;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.connections.Communication;
import com.cfar.swim.worldwind.connections.Datalink;
import com.cfar.swim.worldwind.connections.DatalinkCommunicator;
import com.cfar.swim.worldwind.connections.DatalinkTracker;
import com.cfar.swim.worldwind.environments.DirectedEdge;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.planners.MissionLoader;
import com.cfar.swim.worldwind.planners.OnlinePlanner;
import com.cfar.swim.worldwind.planners.cgs.adstar.ADStarPlanner;
import com.cfar.swim.worldwind.planners.cgs.adstar.ADStarWaypoint;
import com.cfar.swim.worldwind.planners.cgs.astar.AStarWaypoint;
import com.cfar.swim.worldwind.planning.CostPolicy;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.planning.TimeInterval;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.planners.cgs.OADStarProperties;
import com.cfar.swim.worldwind.tracks.AircraftTrack;
import com.cfar.swim.worldwind.tracks.AircraftTrackError;
import com.cfar.swim.worldwind.tracks.AircraftTrackPoint;
import com.cfar.swim.worldwind.tracks.AircraftTrackPointError;
import com.cfar.swim.worldwind.util.Identifiable;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.util.Logging;

/**
 * Realizes an online anytime dynamic A* planner that features a datalink
 * connection to communicate with a planned aircraft.
 * 
 * @author Stephan Heinemann
 *
 */
public class OADStarPlanner extends ADStarPlanner implements OnlinePlanner {
	
	/** the active leg of this OAD* planner */
	private Optional<DirectedEdge> activeLeg = Optional.empty();
	
	/** the active part of this OAD* planner */
	private int activePart = 0;
	
	/** the datalink of this OAD* planner */
	private Datalink datalink = null;
	
	/** the datalink take-off communication of this OAD* planner */
	private Communication<Datalink> takeOff = null;
	
	/** the datalink landing communication of this OAD* planner */
	private Communication<Datalink> landing = null;
	
	/** the datalink unplanned landing communication of this OAD* planner */
	private Communication<Datalink> unplannedLanding = null;
	
	/** the establish datalink communication of this OAD* planner */
	private Communication<Datalink> establishDatalink = null;
	
	/** the maximum acceptable aircraft take-off error of this OAD* planner */
	private AircraftTrackPointError maxTakeOffError = AircraftTrackPointError.ZERO;
	
	/** the maximum acceptable aircraft landing error of this OAD* planner */
	private AircraftTrackPointError maxLandingError = AircraftTrackPointError.ZERO;
	
	/** the maximum acceptable aircraft track error of this OAD* planner */
	private AircraftTrackError maxTrackError = AircraftTrackError.ZERO;
	
	/** the track change listener of this OAD* planner */
	private final TrackChangeListener trackCl = new TrackChangeListener();
	
	/** the mission loader of this OAD* planner */
	private final MissionLoader missionLoader = new MissionLoader(this);
	
	/** the off-track counter of this OAD* planner */
	private int offTrackCount = 0;
	
	/**
	 * Constructs a OAD* planner for a specified aircraft and environment
	 * using default local cost and risk policies.
	 * 
	 * @param aircraft the aircraft
	 * @param environment the environment
	 */
	public OADStarPlanner(Aircraft aircraft, Environment environment) {
		super(aircraft, environment);
		this.addPlanRevisionListener(this.getMissionLoader());
	}
	
	/**
	 * Gets the identifier of this OAD* planner.
	 * 
	 * @return the identifier of this OAD* planner
	 * 
	 * @see Identifiable#getId()
	 */
	@Override
	public String getId() {
		return Specification.PLANNER_OADS_ID;
	}
	
	/**
	 * Gets the active leg of this OAD* planner.
	 * 
	 * @return the active leg of this OAD* planner
	 */
	protected synchronized Optional<DirectedEdge> getActiveLeg() {
		return this.activeLeg;
	}
	
	/**
	 * Sets the active leg of this OAD* planner.
	 * 
	 * @param activeLeg the active leg to be set
	 */
	protected synchronized void setActiveLeg(Optional<DirectedEdge> activeLeg) {
		this.activeLeg = activeLeg;
	}
	
	/**
	 * Gets the previous waypoint of this OAD* planner.
	 * 
	 * @return the previous waypoint of this OAD* planner if any,
	 *         null otherwise
	 * 
	 * @see OnlinePlanner#getPreviousWaypoint()
	 */
	@Override
	public synchronized Waypoint getPreviousWaypoint() {
		return this.getActiveLeg().isEmpty() ? null :
			(ADStarWaypoint) this.getActiveLeg().get().getFirstPosition();
	}
	
	/**
	 * Determines whether or not this OAD* planner has a previous waypoint.
	 * 
	 * @return true if this OAD* planner has a previous waypoint,
	 *         false otherwise
	 * 
	 * @see OnlinePlanner#hasPreviousWaypoint()
	 */
	@Override
	public synchronized boolean hasPreviousWaypoint() {
		return this.getActiveLeg().isPresent();
	}
	
	/**
	 * Gets the next waypoint of this OAD* planner.
	 * 
	 * @return the next waypoint of this OAD* planner if any, null otherwise
	 * 
	 * @see OnlinePlanner#getNextWaypoint()
	 */
	@Override
	public synchronized ADStarWaypoint getNextWaypoint() {
		return this.getActiveLeg().isEmpty() ? null :
			(ADStarWaypoint) this.getActiveLeg().get().getSecondPosition();
	}
	
	/**
	 * Determines whether or not this OAD* planner has a next waypoint.
	 * 
	 * @return true if this OAD* planner has a next waypoint, false otherwise
	 * 
	 * @see OnlinePlanner#hasNextWaypoint()
	 */
	@Override
	public synchronized boolean hasNextWaypoint() {
		return this.getActiveLeg().isPresent();
	}
	
	/**
	 * Gets the active part of this OAD* planner.
	 * 
	 * @return the active part of this OAD* planner
	 * 
	 * @see OnlinePlanner#getActivePart()
	 */
	@Override
	public synchronized int getActivePart() {
		return this.activePart;
	}
	
	/**
	 * Sets the active part of this OAD* planner.
	 * 
	 * @param activePart the active part to be set
	 */
	protected synchronized void setActivePart(int activePart) {
		this.activePart = activePart;
	}
	
	/**
	 * Gets the datalink of this OAD* planner.
	 * 
	 * @return the datalink of this OAD* planner
	 * 
	 * @see DatalinkCommunicator#getDatalink()
	 */
	@Override
	public Datalink getDatalink() {
		return this.datalink;
	}
	
	/**
	 * Sets the datalink of this OAD* planner.
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
	 * Determines whether or not this OAD* planner has a datalink.
	 * 
	 * @return true if this OAD* planner has a datalink, false otherwise
	 * 
	 * @see DatalinkCommunicator#hasDatalink()
	 */
	@Override
	public boolean hasDatalink() {
		return (null != this.datalink);
	}
	
	/**
	 * Gets the maximum acceptable take-off error of this OAD* planner to
	 * consider the aircraft within the take-off regime.
	 * 
	 * @return the maximum acceptable take-off error of this OAD* planner to
	 *         consider the aircraft within the take-off regime
	 *
	 * @see DatalinkTracker#getMaxTakeOffError()
	 */
	@Override
	public synchronized AircraftTrackPointError getMaxTakeOffError() {
		return this.maxTakeOffError;
	}
	
	/**
	 * Sets the maximum acceptable take-off error of this OAD* planner to
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
	 * Determines whether or not the aircraft of this OAD* planner is within
	 * the take-off zone.
	 * 
	 * @return true if the aircraft of this OAD* planner is within the
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
		
		return isInTakeOffZone;
	}
	
	/**
	 * Determines whether or not the aircraft of this OAD* planner is within
	 * the take-off window.
	 * 
	 * @return true if the aircraft of this OAD* planner is within the
	 *         take-off window, false otherwise
	 *
	 * @see DatalinkTracker#isInTakeOffWindow()
	 */
	@Override
	public boolean isInTakeOffWindow() {
		boolean isInTakeOffWindow = false;
		
		if (this.hasDatalink() && this.getDatalink().isConnected()
				&& this.getDatalink().isMonitoring()
				&& this.getStart().hasEto()) {
			AircraftTrack track = this.getDatalink().getAircraftTrack();
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
		
		return isInTakeOffWindow;
	}
	
	/**
	 * Gets the maximum acceptable landing error of this OAD* planner to
	 * consider the aircraft within the landing regime.
	 * 
	 * @return the maximum acceptable landing error of this OAD* planner to
	 *         consider the aircraft within the landing regime
	 *
	 * @see DatalinkTracker#getMaxLandingError()
	 */
	@Override
	public synchronized AircraftTrackPointError getMaxLandingError() {
		return this.maxLandingError;
	}
	
	/**
	 * Sets the maximum acceptable landing error of this OAD* planner to
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
	 * Determines whether or not the aircraft of this OAD* planner is within
	 * the landing zone.
	 * 
	 * @return true if the aircraft of this OAD* planner is within the
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
		
		return isInLandingZone;
	}
	
	/**
	 * Determines whether or not the aircraft of this OAD* planner is within
	 * the landing window.
	 * 
	 * @return true if the aircraft of this OAD* planner is within the
	 *         landing window, false otherwise
	 *
	 * @see DatalinkTracker#isInLandingWindow()
	 */
	@Override
	public boolean isInLandingWindow() {
		boolean isInLandingWindow = false;
		
		if (this.hasDatalink() && this.getDatalink().isConnected()
				&& this.getDatalink().isMonitoring()
				&& this.getGoal().hasEto()) {
			AircraftTrack track = this.getDatalink().getAircraftTrack();
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
		
		return isInLandingWindow;
	}
	
	/**
	 * Gets the maximum acceptable track error of this OAD* planner to
	 * consider the aircraft on track.
	 * 
	 * @return the maximum acceptable track error of this OAD* planner to
	 *         consider the aircraft on track
	 * 
	 * @see DatalinkTracker#getMaxTrackError()
	 */
	@Override
	public synchronized AircraftTrackError getMaxTrackError() {
		return this.maxTrackError;
	}
	
	/**
	 * Sets the maximum acceptable track error of this OAD* planner to
	 * consider the aircraft on track.
	 * 
	 * @param maxTrackError the maximum acceptable track error to be set
	 * 
	 * @throws IllegalArgumentException if the maximum track error is null
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
		 * Notifies the OAD* planner about a track change.
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
	 * Determines whether or not the aircraft of this OAD* planner is on
	 * track.
	 * 
	 * @return true if the aircraft of this OAD* planner is on track,
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
			AircraftTrackPoint last = track.getLastTrackPoint();
			AircraftTrackPoint first = track.getFirstTrackPoint();
			
			// ensure sufficient track points for on-track assessment
			long maxAge = last.getMaxAge().toMillis();
			long period = this.getDatalink().getDownlinkPeriod();
			long maxTrackPoints = maxAge / period;
			
			if (track.size() >= (maxTrackPoints - 1)) {
				DirectedEdge leg = this.getActiveLeg().get();
				ADStarWaypoint next = (ADStarWaypoint) leg.getSecondPosition();
				
				// cross track check
				ADStarWaypoint lastPosition = this.createWaypoint(last.getPosition());
				double hced = leg.getHorizontalCrossEdgeDistance(lastPosition);
				double vced = leg.getVerticalCrossEdgeDistance(lastPosition);
				Angle ob = leg.getOpeningBearing(lastPosition);
				Angle cb = leg.getClosingBearing(lastPosition);
				Logging.logger().info("xtd = " + hced + ", ob = " + ob + ", cb = " + cb);
				
				// ground speed check
				Duration dt = Duration.between(first.getAto(), last.getAto());
				double s = this.getEnvironment().getDistance(first.getPosition(), last.getPosition());
				double gs = s / dt.getSeconds();
				Logging.logger().info("dt = " + dt + ", s = " + s + ", gs =" + gs);
				
				// ETO update
				Duration deto = Duration.ZERO;
				if (0 != gs) {
					double dtg = this.getEnvironment().getDistance(lastPosition, next);
					Duration ttg = Duration.ofSeconds((long) (dtg / gs));
					Logging.logger().info("dtg = " + dtg + ", ttg = " + ttg);
					ZonedDateTime eto = last.getAto().plus(ttg);
					deto = Duration.between(next.getEto(), eto).abs();
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
				
			} else {
				Logging.logger().info("insufficient track points = " + track.size());
			}
		}
		
		return isOnTrack;
	}
	
	/**
	 * Roots a graph in the grid generated by this OAD* planner by making
	 * the previous root and its dependent children under-consistent.
	 * 
	 * @param root the root of the new graph
	 */
	protected void root(ADStarWaypoint root) {
		this.setStart(root);
		if (root.hasParent()) {
			ADStarWaypoint parent = root.getParent();
			root.resetParents();
			// make parent under-consistent and propagate under-consistency
			this.repair(parent);
			this.updateSets(parent);
		}
	}
	
	/**
	 * Progresses the plan of this OAD* planner by rooting its generated graph
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
			
			if (this.hasWaypoints(partIndex - 1)) {
				this.backup(partIndex);
				this.restore(partIndex - 1);
				this.progress(partIndex - 1);
				this.backup(partIndex - 1);
				this.restore(partIndex);
			} else {
				// progress planner graph
				while (!this.getStart().equals(this.createWaypoint(
						this.getDatalink().getNextMissionPosition()))
						&& this.hasWaypoints()) {
					ADStarWaypoint previous = (ADStarWaypoint) this.getWaypoints().removeFirst();
					if (this.hasWaypoints()) {
						this.setActiveLeg(Optional.of(new DirectedEdge(
								this.getEnvironment(), previous, this.getWaypoints().getFirst())));
						this.setActivePart(partIndex);
						this.root((ADStarWaypoint) this.getWaypoints().getFirst());
					} else {
						this.setActiveLeg(Optional.empty());
						Logging.logger().info(this.getId() + " has an empty part " + partIndex);
					}
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
	 * Re-plans an off-track aircraft of this OAD* planner.
	 * 
	 * @param partIndex the of the plan to be re-planned
	 */
	protected synchronized void replan(int partIndex) {
		// TODO: possibly always re-plan from scratch based on track point
		ADStarWaypoint partStart = (ADStarWaypoint) this.getStart().clone();
		
		if (this.hasWaypoints(partIndex - 1)) {
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
			AircraftTrackPoint trackPoint = this.getDatalink().getAircraftTrack().getLastTrackPoint();
			ADStarWaypoint offTrackWaypoint = this.createWaypoint(trackPoint.getPosition());
			
			if (!this.getStart().equals(offTrackWaypoint)) {
				offTrackWaypoint.setEto(trackPoint.getAto());
				this.computeEto(offTrackWaypoint, this.getStart());
				this.setActiveLeg(Optional.of(new DirectedEdge(
						this.getEnvironment(), offTrackWaypoint, this.getStart())));
			} else {
				this.setActiveLeg(Optional.empty());
			}
			
			// plan active part from scratch
			this.initialize(this.getStart(), this.getGoal(), this.getStart().getEto());
			this.getStart().setCost(partStart.getCost());
			this.backups.get(partIndex).clear();
			this.planPart(partIndex);
			partStart.setEto(this.getStart().getEto());
		}
		
		if (partStart.hasInfiniteCost()) {
			// invalid previous part
			this.getGoal().setInfiniteCost();
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
		} else {
			// propagate potential cost change
			double deltaCost = partStart.getCost() - this.getStart().getCost();
			if (0 != deltaCost) {
				for (AStarWaypoint waypoint : this.visited) {
					ADStarWaypoint adswp = (ADStarWaypoint) waypoint;
					adswp.setG(adswp.getG() + deltaCost);
					adswp.setV(adswp.getV() + deltaCost);
				}
			}
			
			if (partStart.hasParent()) {
				this.getStart().setParent(partStart.getParent());
			}
		}
	}
	
	/**
	 * Elaborates an OAD* plan.
	 * 
	 * @param partIndex the index of the plan to be elaborated
	 */
	@Override
	protected void elaborate(int partIndex) {
		// TODO: off-track elaboration: re-plan or control?
		// (1) re-planning from next mission or track point position
		// based on static or dynamic performance
		// (2) control adjusting speed to recover track
		// TODO: consider adding off-track notification callback
		if (!this.isOnTrack()) {
			this.offTrackCount = 0;
			this.replan(partIndex);
		}
		
		// proceed to next part only if fully improved and not in need of repair
		while ((!this.hasMaximumQuality() || this.needsRepair()) && !this.hasTerminated()) {
			this.repair(partIndex);
			// TODO: proceed with alternate or unplanned landing if repair was unsuccessful
			this.progress(partIndex);
			this.improve(partIndex);
			Thread.yield();
		}
		// backup after elaboration
		this.backup(partIndex);
	}
	
	/**
	 * Computes an OAD* plan. The risk policy will be boosted if necessary when
	 * airborne to continue temporarily under higher risk.
	 */
	@Override
	protected void compute() {
		if (this.getStart().equals(this.getGoal())) {
			this.connectPlan(this.getStart());
			return;
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
				ADStarWaypoint end = this.createWaypoint(
						Position.interpolateGreatCircle(
								ratio, this.getStart(), this.getGoal()));
				this.computeEto(this.getStart(), end);
				double cost = this.getEnvironment().getStepCost(
						this.getStart(), end,
						this.getStart().getEto(), end.getEto(),
						CostPolicy.MAXIMUM, RiskPolicy.INSANITY);
				
				if (!this.getRiskPolicy().satisfies(cost)) {
					this.setRiskPolicy(RiskPolicy.adjustTo(cost));
					Logging.logger().warning("the risk policy has been boosted...");
					// TODO: consider risk policy boost notification callback
				}
			}
		}
		
		super.compute();
		
		// TODO: plan for in-flight alternates and / or issue warning if no solution
		// TODO: alternate and precautionary landing communications
		
		this.setRiskPolicy(riskPolicy);
	}
	
	/**
	 * Suspends this OAD* planner until termination, context changes or
	 * off-track situations occur.
	 */
	@Override
	protected synchronized void suspend() {
		try {
			// wait for termination, dynamic changes, or off-track situations
			while (!this.hasTerminated() && !this.needsRepair() && this.isOnTrack()) {
				this.progress(this.backups.size() - 1);
				this.wait();
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * Notifies this OAD* planner about progress of the aircraft.
	 */
	protected synchronized void notifyProgress() {
		this.notifyAll();
	}
	
	/**
	 * Terminates this OAD* planner and removes its downlink listener.
	 * 
	 * @see ADStarPlanner#terminate()
	 */
	@Override
	public synchronized void terminate() {
		super.terminate();
		if (this.hasDatalink()) {
			this.getDatalink().removePropertyChangeListener(this.trackCl);
		}
	}
	
	/**
	 * Recycles this OAD* planner and adds its downlink listener.
	 * 
	 * @see ADStarPlanner#recycle()
	 */
	@Override
	public synchronized void recycle() {
		super.recycle();
		if (this.hasDatalink()) {
			this.getDatalink().addTrackChangeListener(this.trackCl);
		}
	}
	
	/**
	 * Gets the mission loader of this OAD* planner.
	 * 
	 * @return the mission loader of this OAD* planner
	 * 
	 * @see OnlinePlanner#getMissionLoader()
	 */
	@Override
	public MissionLoader getMissionLoader() {
		return this.missionLoader;
	}
	
	/**
	 * Gets the datalink take-off communication of this OAD* planner.
	 * 
	 * @return the datalink take-off communication of this OAD* planner
	 * 
	 * @see DatalinkCommunicator#getTakeOff()
	 */
	@Override
	public Communication<Datalink> getTakeOff() {
		return this.takeOff;
	}
	
	/**
	 * Sets the datalink take-off communication of this OAD* planner.
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
	 * Determines whether or not this OAD* planner has a datalink take-off
	 * communication.
	 * 
	 * @return true if this OAD* planner has a datalink take-off
	 *         communication, false otherwise
	 *
	 * @see DatalinkCommunicator#hasTakeOff()
	 */
	@Override
	public boolean hasTakeOff() {
		return (null != this.takeOff);
	}
	
	/**
	 * Gets the datalink landing communication of this OAD* planner.
	 * 
	 * @return the datalink landing communication of this OAD* planner
	 *
	 * @see DatalinkCommunicator#getLanding()
	 */
	@Override
	public Communication<Datalink> getLanding() {
		return this.landing;
	}
	
	/**
	 * Sets the datalink landing communication of this OAD* planner.
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
	 * Determines whether or not this OAD* planner has a datalink landing
	 * communication.
	 * 
	 * @return true if this OAD* planner has a datalink landing
	 *         communication, false otherwise
	 *
	 * @see DatalinkCommunicator#hasLanding() 
	 */
	@Override
	public boolean hasLanding() {
		return (null != this.landing);
	}
	
	/**
	 * Gets the datalink unplanned landing communication of this OAD* planner.
	 * 
	 * @return the datalink unplanned landing communication of this OAD*
	 *         planner
	 *
	 * @see DatalinkCommunicator#getUnplannedLanding()
	 */
	@Override
	public Communication<Datalink> getUnplannedLanding() {
		return this.unplannedLanding;
	}
	
	/**
	 * Sets the datalink unplanned landing communication of this OAD* planner.
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
	 * Determines whether or not this OAD* planner has a datalink unplanned
	 * landing communication.
	 * 
	 * @return true if this OAD* planner has a datalink unplanned landing
	 *         communication, false otherwise
	 *
	 * @see DatalinkCommunicator#hasUnplannedLanding()
	 */
	@Override
	public boolean hasUnplannedLanding() {
		return (null != this.unplannedLanding);
	}
	
	/**
	 * Gets the establish datalink communication of this OAD* planner.
	 * 
	 * @return the establish datalink communication of this OAD* planner
	 *
	 * @see DatalinkCommunicator#getEstablishDataLink()
	 */
	@Override
	public Communication<Datalink> getEstablishDataLink() {
		return this.establishDatalink;
	}
	
	/**
	 * Sets the establish datalink communication of this OAD* planner.
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
	 * Determines whether or not this OAD* planner has an establish datalink
	 * communication.
	 * 
	 * @return true if this OAD* planner has an establish datalink
	 *         communication, false otherwise
	 *
	 * @see DatalinkCommunicator#hasEstablishDatalink()
	 */
	@Override
	public boolean hasEstablishDatalink() {
		return (null != this.establishDatalink);
	}
	
	/**
	 * Performs a take-off via the datalink communication of this OAD* planner.
	 */
	protected void performTakeOff() {
		if (this.hasTakeOff()) {
			this.takeOff.perform();
		}
	}
	
	/**
	 * Performs a landing via the datalink communication of this OAD* planner.
	 */
	protected void performLanding() {
		if (this.hasLanding()) {
			this.landing.perform();
		}
	}
	
	/**
	 * Performs an unplanned landing via the datalink communication of this
	 * OAD* planner.
	 */
	protected void performUnplannedLanding() {
		if (this.hasUnplannedLanding()) {
			this.unplannedLanding.perform();
		}
	}
	
	/**
	 * Establishes the datalink communication of this OAD* planner.
	 */
	protected void establishDatalink() {
		if (this.hasEstablishDatalink()) {
			this.establishDatalink.perform();
		}
	}
	
	/**
	 * Determines whether or not this OAD* planner matches a specification.
	 * 
	 * @param specification the specification to be matched
	 * 
	 * @return true if the this OAD* planner matches the specification,
	 *         false otherwise
	 * 
	 * @see ADStarPlanner#matches(Specification)
	 */
	@Override
	public synchronized boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = super.matches(specification);
		
		if (matches && (specification.getProperties() instanceof OADStarProperties)) {
			OADStarProperties properties = (OADStarProperties) specification.getProperties();
			matches = (this.getMaxTrackError().getCrossTrackError() == properties.getMaxCrossTrackError())
					&& (this.getMaxTrackError().getTimingError().equals(Duration.ofSeconds(properties.getMaxTimingError())))
					&& (this.getMaxTakeOffError().getHorizontalError() == properties.getMaxTakeOffHorizontalError())
					&& (this.getMaxTakeOffError().getTimingError().equals(Duration.ofSeconds(properties.getMaxTakeOffTimingError())))
					&& (this.getMaxLandingError().getHorizontalError() == properties.getMaxLandingHorizontalError())
					&& (this.getMaxLandingError().getTimingError().equals(Duration.ofSeconds(properties.getMaxLandingTimingError())));
		}
		
		return matches;
	}
	
	/**
	 * Updates this OAD* planner according to a specification.
	 * 
	 * @param specification the specification to be used for the update
	 * 
	 * @return true if this OAD* planner has been updated, false otherwise
	 * 
	 * @see ADStarPlanner#update(Specification)
	 */
	@Override
	public synchronized boolean update(Specification<? extends FactoryProduct> specification) {
		boolean updated = super.update(specification);
		
		if (updated && (specification.getProperties() instanceof OADStarProperties)) {
			OADStarProperties properties = (OADStarProperties) specification.getProperties();
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
