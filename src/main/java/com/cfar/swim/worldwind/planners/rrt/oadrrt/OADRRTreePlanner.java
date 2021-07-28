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
import java.util.Optional;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.connections.Communication;
import com.cfar.swim.worldwind.connections.Datalink;
import com.cfar.swim.worldwind.environments.DirectedEdge;
import com.cfar.swim.worldwind.environments.Edge;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.planners.AbstractPlanner;
import com.cfar.swim.worldwind.planners.OnlinePlanner;
import com.cfar.swim.worldwind.planners.PlanRevisionListener;
import com.cfar.swim.worldwind.planners.rrt.adrrt.ADRRTreePlanner;
import com.cfar.swim.worldwind.planners.rrt.adrrt.ADRRTreeWaypoint;
import com.cfar.swim.worldwind.planning.RiskPolicy;
import com.cfar.swim.worldwind.planning.TimeInterval;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.planners.rrt.OADRRTreeProperties;
import com.cfar.swim.worldwind.tracks.AircraftTrack;
import com.cfar.swim.worldwind.tracks.AircraftTrackError;
import com.cfar.swim.worldwind.tracks.AircraftTrackPoint;
import com.cfar.swim.worldwind.tracks.AircraftTrackPointError;

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
	
	/** the current leg of the aircraft planned by this OADRRT planner */
	private Optional<Edge> currentLeg = Optional.empty();
	
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
	
	/** the maximum acceptable aircraft take-off error of this OADRRT planner */
	private AircraftTrackPointError maxTakeOffError = AircraftTrackPointError.ZERO;
	
	/** the maximum acceptable aircraft landing error of this OADRRT planner */
	private AircraftTrackPointError maxLandingError = AircraftTrackPointError.ZERO;
	
	/** the maximum acceptable aircraft track error of this OADRRT planner */
	private AircraftTrackError maxTrackError = AircraftTrackError.ZERO;
	
	/** the track change listener of this OADRRT planner */
	private final TrackChangeListener trackCl = new TrackChangeListener();
	
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
		this.addPlanRevisionListener(new MissionLoader());
	}
	
	/**
	 * Realizes a mission loader to upload revised trajectories via the
	 * datalink of this OADRRT planner.
	 * 
	 * @author Stephan Heinemann
	 *
	 */
	protected class MissionLoader implements PlanRevisionListener {
		
		/**
		 * Uploads a revised trajectory via the datalink of this OADRRT planner.
		 * 
		 * @param trajectory the revised trajectory
		 */
		@Override
		public void revisePlan(Trajectory trajectory) {
			if (hasDatalink() && getDatalink().isConnected()) {
				// warn if mission is obsolete
				if (!getStart().equals(createWaypoint(getDatalink().getNextMissionPosition()))) {
					Logging.logger().warning("start is not the next mission position...");
				}
				// do not upload an empty trajectory
				if (!trajectory.isEmpty()) {
					Logging.logger().info("uploading mission: " + trajectory);
					getDatalink().uploadMission(trajectory);
					// confirm the consistent upload
					if (!getDatalink().hasMission(trajectory, false)) {
						Logging.logger().severe("mission is not consistent...");
					}
				} else {
					Logging.logger().warning("not uploading an empty trajectory...");
				}
			}
		}
	}
	
	/**
	 * Gets the datalink of this OADRRT planner.
	 * 
	 * @return the datalink of this OADRRT planner
	 * 
	 * @see OnlinePlanner#getDatalink()
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
	 * @see OnlinePlanner#setDatalink(Datalink)
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
	 * @see OnlinePlanner#hasDatalink()
	 */
	@Override
	public boolean hasDatalink() {
		return (null != this.datalink);
	}
	
	/**
	 * Gets the maximum acceptable take-off error of this OADRRT planner to
	 * consider the aircraft within the take-off regime.
	 * 
	 * @return the maximum acceptable take-off error of this OADRRT planner to
	 *         consider the aircraft within the take-off regime
	 *
	 * @see OnlinePlanner#getMaxTakeOffError()
	 */
	@Override
	public AircraftTrackPointError getMaxTakeOffError() {
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
	 * @see OnlinePlanner#setMaxTakeOffError(AircraftTrackPointError)
	 */
	@Override
	public void setMaxTakeOffError(AircraftTrackPointError maxTakeOffError) {
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
	 * @see OnlinePlanner#isInTakeOffZone()
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
	 * Determines whether or not the aircraft of this OADRRT planner is within
	 * the take-off window.
	 * 
	 * @return true if the aircraft of this OADRRT planner is within the
	 *         take-off window, false otherwise
	 *
	 * @see OnlinePlanner#isInTakeOffWindow()
	 */
	@Override
	public boolean isInTakeOffWindow() {
		boolean isInTakeOffWindow = false;
		
		if (this.hasDatalink() && this.getDatalink().isConnected()
				&& this.getDatalink().isMonitoring()) {
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
	 * Gets the maximum acceptable landing error of this OADRRT planner to
	 * consider the aircraft within the landing regime.
	 * 
	 * @return the maximum acceptable landing error of this OADRRT planner to
	 *         consider the aircraft within the landing regime
	 *
	 * @see OnlinePlanner#getMaxLandingError()
	 */
	@Override
	public AircraftTrackPointError getMaxLandingError() {
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
	 * @see OnlinePlanner#setMaxLandingError(AircraftTrackPointError)
	 */
	@Override
	public void setMaxLandingError(AircraftTrackPointError maxLandingError) {
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
	 * @see OnlinePlanner#isInLandingZone()
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
	 * Determines whether or not the aircraft of this OADRRT planner is within
	 * the landing window.
	 * 
	 * @return true if the aircraft of this OADRRT planner is within the
	 *         landing window, false otherwise
	 *
	 * @see OnlinePlanner#isInLandingWindow()
	 */
	@Override
	public boolean isInLandingWindow() {
		boolean isInLandingWindow = false;
		
		if (this.hasDatalink() && this.getDatalink().isConnected()
				&& this.getDatalink().isMonitoring()) {
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
	 * Gets the maximum acceptable track error of this OADRRT planner to
	 * consider the aircraft on track.
	 * 
	 * @return the maximum acceptable track error of this OADRRT planner to
	 *         consider the aircraft on track
	 * 
	 * @see OnlinePlanner#getMaxTrackError()
	 */
	@Override
	public AircraftTrackError getMaxTrackError() {
		return this.maxTrackError;
	}
	
	/**
	 * Sets the maximum acceptable track error of this OADRRT planner to
	 * consider the aircraft on track.
	 * 
	 * @param maxTrackError the maximum acceptable track error to be set
	 * 
	 * @see OnlinePlanner#setMaxTrackError(AircraftTrackError)
	 */
	@Override
	public void setMaxTrackError(AircraftTrackError maxTrackError) {
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
	 * @see OnlinePlanner#isOnTrack()
	 */
	@Override
	public boolean isOnTrack() {
		boolean isOnTrack = true;
		
		if (this.currentLeg.isPresent()
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
				DirectedEdge leg = (DirectedEdge) this.currentLeg.get();
				ADRRTreeWaypoint next = (ADRRTreeWaypoint) leg.getSecondPosition();
				
				// cross track check
				ADRRTreeWaypoint lastPosition = this.createWaypoint(last.getPosition());
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
					deto = this.maxTrackError.getTimingError().plusSeconds(1);
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
	protected void progress(int partIndex) {
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
				// progress planner tree
				while (!this.getStart().equals(this.createWaypoint(
						this.getDatalink().getNextMissionPosition()))
						&& this.hasWaypoints()) {
					ADRRTreeWaypoint previous = (ADRRTreeWaypoint) this.getWaypoints().removeFirst();
					if (this.hasWaypoints()) {
						this.currentLeg = this.getPlanningContinuum().findEdge(
								previous, this.getWaypoints().getFirst());
						this.root((ADRRTreeWaypoint) this.getWaypoints().getFirst());
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
	 * Re-plans an off-track aircraft of this OADRRT planner.
	 * 
	 * @param partIndex the of the plan to be re-planned
	 */
	protected void replan(int partIndex) {
		ADRRTreeWaypoint partStart = (ADRRTreeWaypoint) this.getStart().clone();
		
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
			ADRRTreeWaypoint offTrackWaypoint = this.createWaypoint(trackPoint.getPosition());
			
			if (!this.getStart().equals(offTrackWaypoint)) {
				offTrackWaypoint.setEto(trackPoint.getAto());
				this.computeEto(offTrackWaypoint, this.getStart());
				this.currentLeg = Optional.of(new DirectedEdge(
						this.getEnvironment(), offTrackWaypoint, this.getStart()));
				this.initialize(this.getStart(), this.getGoal(), this.getStart().getEto());
				this.planPart(partIndex);
				partStart.setEto(this.getStart().getEto());
			}
		}
		
		if (partStart.hasInfiniteCost()) {
			// TODO: invalid previous part?
		} else if (!this.getStart().getEto().equals(partStart.getEto())) {
			// plan current part from scratch if start ETO has changed
			this.initialize(this.getStart(), this.getGoal(), partStart.getEto());
			if (partStart.hasParent()) {
				this.getStart().setParent(partStart.getParent());
			}
			this.planPart(partIndex);
		}
	}
	
	/**
	 * Elaborates an OADRRT plan.
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
		
		// do not elaborate an exceeded risk policy solution beyond limit
		int riskyProbes = (this.getGoal().hasInfiniteCost()) ? 1 : 0;
		
		// proceed to next part only if fully improved and not in need of repair
		while (((!this.hasMaximumQuality() && (this.getMaxRiskyProbes() > riskyProbes))
				|| this.needsRepair()) && !this.hasTerminated()) {
			this.repair(partIndex);
			this.progress(partIndex);
			this.improve(partIndex);
			if (this.getGoal().hasInfiniteCost()) {
				riskyProbes++;
			} else {
				riskyProbes = 0;
			}
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
				ADRRTreeWaypoint end = this.createWaypoint(
						Position.interpolateGreatCircle(
								ratio, this.getStart(), this.getGoal()));
				this.computeEto(this.getStart(), end);
				DirectedEdge leg = new DirectedEdge(this.getEnvironment(), this.getStart(), end);
				double cost = leg.getCost(this.getStart().getEto(), end.getEto());
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
	 * Gets the datalink take-off communication of this OADRRT planner.
	 * 
	 * @return the datalink take-off communication of this OADRRT planner
	 * 
	 * @see OnlinePlanner#getTakeOff()
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
	 * @see OnlinePlanner#setTakeOff(Communication)
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
	 * @see OnlinePlanner#hasTakeOff()
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
	 * @see OnlinePlanner#getLanding()
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
	 * @see OnlinePlanner#setLanding(Communication)
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
	 * @see OnlinePlanner#hasLanding() 
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
	 * @see OnlinePlanner#getUnplannedLanding()
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
	 * @see OnlinePlanner#setUnplannedLanding(Communication)
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
	 * @see OnlinePlanner#hasUnplannedLanding()
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
	 * @see OnlinePlanner#getEstablishDataLink()
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
	 * @see OnlinePlanner#setEstablishDatalink(Communication)
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
	 * @see OnlinePlanner#hasEstablishDatalink()
	 */
	@Override
	public boolean hasEstablishDatalink() {
		return (null != this.establishDatalink);
	}
	
	/**
	 * Performs a take-off via the datalink communication of this OADRRT
	 * planner.
	 */
	private void performTakeOff() {
		if (this.hasTakeOff()) {
			this.takeOff.perform();
		}
	}
	
	/**
	 * Performs a landing via the datalink communication of this OADRRT
	 * planner.
	 */
	private void performLanding() {
		if (this.hasLanding()) {
			this.landing.perform();
		}
	}
	
	/**
	 * Performs an unplanned landing via the datalink communication of this
	 * OADRRT planner.
	 */
	private void performUnplannedLanding() {
		if (this.hasUnplannedLanding()) {
			this.unplannedLanding.perform();
		}
	}
	
	/**
	 * Establishes the datalink communication of this OADRRT planner.
	 */
	private void establishDatalink() {
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
	 * @see AbstractPlanner#matches(Specification)
	 */
	@Override
	public boolean matches(Specification<? extends FactoryProduct> specification) {
		boolean matches = false;
		
		if ((null != specification) && (specification.getProperties() instanceof OADRRTreeProperties)) {
			OADRRTreeProperties oadrrtp = (OADRRTreeProperties) specification.getProperties();
			matches = (this.getCostPolicy().equals(oadrrtp.getCostPolicy()))
					&& (this.getRiskPolicy().equals(oadrrtp.getRiskPolicy()))
					&& (this.getBias() == oadrrtp.getBias())
					&& (this.getEpsilon() == oadrrtp.getEpsilon())
					&& (this.getExtension() == oadrrtp.getExtension())
					&& (this.getGoalThreshold() == oadrrtp.getGoalThreshold())
					&& (this.getMaxIterations() == oadrrtp.getMaxIterations())
					&& (this.getSampling() == oadrrtp.getSampling())
					&& (this.getStrategy() == oadrrtp.getStrategy())
					&& (this.getMaximumQuality() == oadrrtp.getMaximumQuality())
					&& (this.getMinimumQuality() == oadrrtp.getMaximumQuality())
					&& (this.getNeighborLimit() == oadrrtp.getNeighborLimit())
					&& (this.getQualityImprovement() == oadrrtp.getQualityImprovement())
					&& (this.getSignificantChange() == oadrrtp.getSignificantChange())
					&& (this.getMaxTrackError().getCrossTrackError() == oadrrtp.getMaxCrossTrackError())
					&& (this.getMaxTrackError().getTimingError().equals(Duration.ofSeconds(oadrrtp.getMaxTimingError())))
					&& (this.getMaxTakeOffError().getHorizontalError() == oadrrtp.getMaxTakeOffHorizontalError())
					&& (this.getMaxTakeOffError().getTimingError().equals(Duration.ofSeconds(oadrrtp.getMaxTakeOffTimingError())))
					&& (this.getMaxLandingError().getHorizontalError() == oadrrtp.getMaxLandingHorizontalError())
					&& (this.getMaxLandingError().getTimingError().equals(Duration.ofSeconds(oadrrtp.getMaxLandingTimingError())))
					&& (specification.getId().equals(Specification.PLANNER_OADRRT_ID));
		}
		
		return matches;
	}
	
}
