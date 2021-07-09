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
import com.cfar.swim.worldwind.planning.TimeInterval;
import com.cfar.swim.worldwind.planning.Trajectory;
import com.cfar.swim.worldwind.registries.FactoryProduct;
import com.cfar.swim.worldwind.registries.Specification;
import com.cfar.swim.worldwind.registries.planners.rrt.OADRRTreeProperties;
import com.cfar.swim.worldwind.tracks.AircraftTrack;
import com.cfar.swim.worldwind.tracks.AircraftTrackError;
import com.cfar.swim.worldwind.tracks.AircraftTrackPoint;

import gov.nasa.worldwind.geom.Angle;
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
		this.addPlanRevisionListener(new PlanRevisionListener() {
			@Override
			public void revisePlan(Trajectory trajectory) {
				if (hasDatalink() && getDatalink().isConnected()) {
					Logging.logger().info("uploading mission...");
					// only update mission if still relevant
					if (!getStart().equals(createWaypoint(getDatalink().getNextMissionPosition()))) {
						Logging.logger().warning("start is not the next mission position...");
					}
					// do not upload an empty trajectory
					if (!trajectory.isEmpty()) {
						datalink.uploadMission(trajectory);
					} else {
						Logging.logger().warning("not uploading an empty trajectory...");
					}
				}
			}
		});
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
				double ced = leg.getCrossEdgeDistance(lastPosition);
				Angle ob = leg.getOpeningBearing(lastPosition);
				Angle cb = leg.getClosingBearing(lastPosition);
				Logging.logger().info("xtd = " + ced + ", ob = " + ob + ", cb = " + cb);
				
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
				
				isOnTrack = (this.getMaxTrackError().getCrossTrackError() >= ced)
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
			root.setParent(null);
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
				&& this.getDatalink().isMonitoring()) {
			
			if (this.hasWaypoints(partIndex - 1)) {
				this.backup(partIndex);
				this.restore(partIndex - 1);
				this.progress(partIndex - 1);
				this.backup(partIndex - 1);
				this.restore(partIndex);
			} else {
				// TODO: make sure mission has been uploaded
				// progress planner tree
				while (!this.getStart().equals(this.createWaypoint(this.getDatalink().getNextMissionPosition()))
						&& this.hasWaypoints()) {
					ADRRTreeWaypoint previous = (ADRRTreeWaypoint) this.removeFirstWaypoint();
					if (this.hasWaypoints()) {
						this.currentLeg = this.getEnvironment().findEdge(
								previous, this.getFirstWaypoint());
						this.root((ADRRTreeWaypoint) this.getFirstWaypoint());
					}
				}
				
				// check take-off and landing conditions
				AircraftTrackPoint last = this.getDatalink().getAircraftTrack().getLastTrackPoint();
				if (this.isLastIndex(partIndex)
						&& this.getDatalink().isAirborne()
						&& this.isInGoalRegion(last.getPosition())) {
					// calculate landing window with full error margin
					ZonedDateTime landingTime = this.getGoal().getEto();
					TimeInterval landingWindow = new TimeInterval(
							landingTime.minus(this.getMaxTrackError()
									.getTimingError().getSeconds(), ChronoUnit.SECONDS),
							landingTime.plus(this.getMaxTrackError()
									.getTimingError().getSeconds(), ChronoUnit.SECONDS));
					if (landingWindow.contains(last.getAto())) {
						this.performLanding();
					} else {
						this.performUnplannedLanding();
					}
				} else if ((0 == partIndex)
						&& !this.getDatalink().isAirborne()
						&& this.hasWaypoints()) {
					// calculate take-off window with reduced error margin
					ZonedDateTime takeOffTime = this.getStart().getEto();
					TimeInterval takeOffWindow = new TimeInterval(
							takeOffTime.minus(this.getMaxTrackError()
									.getTimingError().getSeconds() / 2, ChronoUnit.SECONDS),
							takeOffTime.plus(this.getMaxTrackError()
									.getTimingError().getSeconds() / 2, ChronoUnit.SECONDS));
					if (takeOffWindow.contains(last.getAto())) {
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
			// TODO: what if no plan could be found
			partStart.setEto(this.getGoal().getEto());
			partStart.setParent(this.getGoal().getParent());
			this.backup(partIndex - 1);
			this.restore(partIndex);
		} else {
			// naively re-plan based on aircraft capabilities
			AircraftTrackPoint trackPoint = this.getDatalink().getAircraftTrack().getLastTrackPoint();
			ADRRTreeWaypoint offTrackWaypoint = this.createWaypoint(trackPoint.getPosition());
			offTrackWaypoint.setEto(trackPoint.getAto());
			this.computeEto(offTrackWaypoint, this.getStart());
			this.currentLeg = Optional.of(new DirectedEdge(
					this.getEnvironment(), offTrackWaypoint, this.getStart()));
			this.initialize(this.getStart(), this.getGoal(), this.getStart().getEto());
			this.planPart(partIndex);
			partStart.setEto(this.getStart().getEto());
		}
		
		if (!this.getStart().getEto().equals(partStart.getEto())) {
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
		if (!this.isOnTrack()) {
			this.offTrackCount = 0;
			this.replan(partIndex);
		}
		
		// proceed to next part only if fully improved and not in need of repair
		while ((!this.hasMaximumQuality() || this.needsRepair()) && !this.hasTerminated()) {
			this.repair(partIndex);
			this.progress(partIndex);
			this.improve(partIndex);
		}
		// backup after elaboration
		this.backup(partIndex);
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
	 * Sets the establish datalink communication of this online planner.
	 * 
	 * @param establishDatalink the establish datalink communication of this
	 *                          online planner
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
	
	/*
	 * An online planner can plan for any departure time (in the past as well as
	 * in the future). If the current time and position of the aircraft matches
	 * the time and point of departure, the online planner should request a
	 * take-off clearance before initiating a take-off via its datalink.
	 * While the aircraft is following the planned trajectory, and reaching
	 * intermediate waypoints, the online planner has to remove irrelevant
	 * parent waypoints and their dependents. The online planner has to revise
	 * the plan if obstacles change the partially known environment, or if the
	 * aircraft is off track (exceeding maximum cross track or timing errors).
	 * Should the online planner influence the aircraft performance in order
	 * to maintain within the trajectory limits and avoid revisions?
	 * Probably not, the online planner is not a low-level controller but
	 * could communicate with one.
	 * Once the aircraft has arrived a the time and point of destination
	 * (within the track error limitations), the online planner should request
	 * a landing clearance before initiating a landing via its datalink.
	 * The online planner should issue warnings if expected next waypoints
	 * are not the ones the aircraft is navigating towards.
	 */
	
	// TODO: follow datalink target and compare course and track as well as ETOs and ATOs
	// TODO: revise plan if significantly off track (track or ATO)
	// TODO: repair plan with respect to current track point (start with ATO or next with ETO)
	// TODO: issue lost datalink connection warnings
	// TODO: possibly adjust speed or track to recover plan
	// TODO: trigger complete re-planning from present position off-track
	// TODO: add off-track notification callback
	// TODO: if not landed after termination: RTL versus LAND
	
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
					&& (this.getMaxTrackError().getTimingError() == Duration.ofSeconds(oadrrtp.getMaxTimingError()))
					&& (specification.getId().equals(Specification.PLANNER_OADRRT_ID));
		}
		
		return matches;
	}
	
}
