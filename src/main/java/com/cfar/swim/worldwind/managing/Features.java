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
package com.cfar.swim.worldwind.managing;

import java.time.Duration;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.ResourceBundle;
import java.util.Set;
import java.util.stream.Collectors;

import com.cfar.swim.worldwind.aircraft.Aircraft;
import com.cfar.swim.worldwind.environments.Environment;
import com.cfar.swim.worldwind.environments.PlanningContinuum;
import com.cfar.swim.worldwind.managers.AutonomicManager;
import com.cfar.swim.worldwind.planners.Planner;
import com.cfar.swim.worldwind.planning.TimeInterval;
import com.cfar.swim.worldwind.planning.Waypoint;
import com.cfar.swim.worldwind.render.Obstacle;
import com.cfar.swim.worldwind.render.ObstaclePath;
import com.cfar.swim.worldwind.session.Scenario;
import com.cfar.swim.worldwind.util.ResourceBundleLoader;

import gov.nasa.worldwind.geom.Box;
import gov.nasa.worldwind.geom.Extent;

/**
 * Realizes discriminating features of an autonomic context contained in a scenario.
 * 
 * @author Stephan Heinemann
 *
 * @see Scenario
 * @see AutonomicManager
 */
public class Features extends HashMap<String, Object> {

	/** the default serial identification of these features */
	private static final long serialVersionUID = 1L;
	
	/** the dictionary of these features */
	protected final ResourceBundle dictionary = ResourceBundleLoader.getDictionaryBundle();
	
	/** the default feature horizon */
	public static final Duration FEATURE_HORIZON = Duration.from(Duration.ofMinutes(5));
	
	/** the aircraft altitude feature key of these features */
	public static final String FEATURE_AIRCRAFT_ALTITUDE = "feature.aircraft.altitude";
	/** the aircraft cruise speed feature key of these features */
	public static final String FEATURE_AIRCRAFT_SPEED_HORIZONTAL = "feature.aircraft.speed.horizontal";
	/** the aircraft climb rate feature key of these features */
	public static final String FEATURE_AIRCRAFT_SPEED_VERTICAL_CLIMB = "feature.aircraft.speed.vertical.climb";
	/** the aircraft descent rate feature key of these features */
	public static final String FEATURE_AIRCRAFT_SPEED_VERTICAL_DESCENT = "feature.aircraft.speed.vertical.descent";
	/** the aircraft safety volume feature key of these features */
	public static final String FEATURE_AIRCRAFT_VOLUME_SAFETY = "feature.aircraft.volume.safety";
	/** the aircraft obstacles average distance feature key of these features */
	public static final String FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_AVG = "feature.aircraft.obstacles.distance.avg";
	/** the aircraft obstacles maximum distance feature key of these features */
	public static final String FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_MAX = "feature.aircraft.obstacles.distance.max";
	/** the aircraft obstacles minimum distance feature key of these features */
	public static final String FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_MIN = "feature.aircraft.obstacles.distance.min";
	/** the aircraft nearest obstacle cost feature key of these features */
	public static final String FEATURE_AIRCRAFT_OBSTACLES_NEAREST_COST = "feature.aircraft.obstacles.nearest.cost";
	/** the aircraft nearest obstacle policies cost feature key of these features */
	public static final String FEATURE_AIRCRAFT_OBSTACLES_NEAREST_COST_POLICIES = "feature.aircraft.obstacles.nearest.cost.policies";
	/** the aircraft nearest obstacle volume feature key of these features */
	public static final String FEATURE_AIRCRAFT_OBSTACLES_NEAREST_VOLUME = "feature.aircraft.obstacles.nearest.volume";
	/** the aircraft nearest obstacle volume ratio feature key of these features */
	public static final String FEATURE_AIRCRAFT_OBSTACLES_NEAREST_VOLUME_RATIO = "feature.aircraft.obstacles.nearest.volume.ratio";
	
	/** the environment diameter feature key of these features */
	public static final String FEATURE_ENVIRONMENT_DIAMETER = "feature.environment.diameter";
	/** the environment volume key of these features */
	public static final String FEATURE_ENVIRONMENT_VOLUME = "feature.environment.volume";
	/** the environment obstacles count feature key of these features */
	public static final String FEATURE_ENVIRONMENT_OBSTACLES_COUNT = "feature.environment.obstacles.count";
	/** the environment obstacles policies cost feature key of these features */
	public static final String FEATURE_ENVIRONMENT_OBSTACLES_COST_POLICIES = "feature.environment.obstacles.cost.policies";
	/** the environment obstacles average cost feature key of these features */
	public static final String FEATURE_ENVIRONMENT_OBSTACLES_COST_AVG = "feature.environment.obstacles.cost.avg";
	/** the environment obstacles maximum cost feature key of these features */
	public static final String FEATURE_ENVIRONMENT_OBSTACLES_COST_MAX = "feature.environment.obstacles.cost.max";
	/** the environment obstacles minimum cost feature key of these features */
	public static final String FEATURE_ENVIRONMENT_OBSTACLES_COST_MIN = "feature.environment.obstacles.cost.min";
	/** the environment obstacles accumulated cost feature key of these features */
	public static final String FEATURE_ENVIRONMENT_OBSTACLES_COST_SUM = "feature.environment.obstacles.cost.sum";
	/** the environment obstacles average volume feature key of these features */
	public static final String FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_AVG = "feature.envrionment.obstacles.volume.avg";
	/** the environment obstacles maximum volume feature key of these features */
	public static final String FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_MAX = "feature.environment.obstacles.volume.max";
	/** the environment obstacles minimum volume feature key of these features */
	public static final String FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_MIN = "feature.environment.obstacles.volume.min";
	/** the environment obstacles accumulated volume feature key of these features */
	public static final String FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_SUM = "feature.environment.obstacles.volume.sum";
	/** the environment obstacles volume ratio feature key of these features */
	public static final String FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_RATIO = "feature.environment.obstacles.volume.ratio";
	// TODO: volume ratios: avg, max, min
	
	/** the POIs average consecutive distance feature key of these features */
	public static final String FEATURE_POIS_DISTANCE_AVG = "feature.pois.distance.avg";
	/** the POIs maximum consecutive distance feature key of these features */
	public static final String FEATURE_POIS_DISTANCE_MAX = "feature.pois.distance.max";
	/** the POIs minimum consecutive distance feature key of these features */
	public static final String FEATURE_POIS_DISTANCE_MIN = "feature.pois.distance.min";
	/** the POIs diameter feature key of these features */
	public static final String FEATURE_POIS_DIAMETER = "feature.pois.diameter";
	/** the POIs volume feature key of these features */
	public static final String FEATURE_POIS_VOLUME = "feature.pois.volume";
	/** the POIs obstacles count feature key of these features */
	public static final String FEATURE_POIS_OBSTACLES_COUNT = "feature.pois.obstacles.count";
	/** the POIs obstacles policies cost feature key of these features */
	public static final String FEATURE_POIS_OBSTACLES_COST_POLICIES = "feature.pois.obstacles.cost.policies";
	/** the POIs obstacles average cost feature key of these features */
	public static final String FEATURE_POIS_OBSTACLES_COST_AVG = "feature.pois.obstacles.cost.avg";
	/** the POIs obstacles minimum cost feature key of these features */
	public static final String FEATURE_POIS_OBSTACLES_COST_MAX = "feature.pois.obstacles.cost.max";
	/** the POIs obstacles minimum cost feature key of these features */
	public static final String FEATURE_POIS_OBSTACLES_COST_MIN = "feature.pois.obstacles.cost.min";
	/** the POIs obstacles accumulated cost feature key of these features */
	public static final String FEATURE_POIS_OBSTACLES_COST_SUM = "feature.pois.obstacles.cost.sum";
	/** the POIs obstacles average volume feature key of these features */
	public static final String FEATURE_POIS_OBSTACLES_VOLUME_AVG = "feature.pois.obstacles.volume.avg";
	/** the POIs obstacles maximum volume feature key of these features */
	public static final String FEATURE_POIS_OBSTACLES_VOLUME_MAX = "feature.pois.obstacles.volume.max";
	/** the POIs obstacles minimum volume feature key of these features */
	public static final String FEATURE_POIS_OBSTACLES_VOLUME_MIN = "feature.pois.obstacles.volume.min";
	/** the POIs obstacles accumulated volume feature key of these features */
	public static final String FEATURE_POIS_OBSTACLES_VOLUME_SUM = "feature.pois.obstacles.volume.sum";
	/** the POIs obstacles volume ratio feature key of these features */
	public static final String FEATURE_POIS_OBSTACLES_VOLUME_RATIO = "feature.pois.obstacles.volume.ratio";
	// TODO: volume ratios: avg, max, min
	
	/** the POIs environment volume ratio feature key of these features */
	public static final String FEATURE_POIS_ENVIRONMENT_VOLUME_RATIO = "feature.pois.environment.volume.ratio";
	/** the POIs environment obstacles count ratio feature key of these features */
	public static final String FEATURE_POIS_ENVIRONMENT_OBSTACLES_COUNT_RATIO = "feature.pois.environment.obstacles.count.ratio";
	/** the POIs environment obstacle policies cost ratio feature key of these features */
	public static final String FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_POLICIES_RATIO = "feature.pois.environment.obstacles.cost.policies.ratio";
	/** the POIs environment obstacle average cost ratio feature key of these features */
	public static final String FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_AVG_RATIO = "feature.pois.environment.obstacles.cost.avg.ratio";
	/** the POIs environment obstacle maximum cost ratio feature key of these features */
	public static final String FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_MAX_RATIO = "feature.pois.environment.obstacles.cost.max.ratio";
	/** the POIs environment obstacle minimum cost ratio feature key of these features */
	public static final String FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_MIN_RATIO = "feature.pois.environment.obstacles.cost.min.ratio";
	/** the POIs environment obstacle accumulated cost ratio feature key of these features */
	public static final String FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_SUM_RATIO = "feature.pois.environment.obstacles.cost.sum.ratio";
	
	/** the POIs environment obstacles volume ratio feature key of these features */
	public static final String FEATURE_POIS_ENVIRONMENT_OBSTACLES_VOLUME_RATIO = "feature.pois.environment.obstacles.volume.ratio";
	// TODO: volume ratios: avg, max, min
	
	// TODO: aircraft position to next POI leg: distance, obstacles
	
	/** the cost policy feature key of these features */
	public static final String FEATURE_POLICY_COST = "feature.policy.cost";
	/** the risk policy feature key of these features */
	public static final String FEATURE_POLICY_RISK = "feature.policy.risk";
	
	/** the feature horizon of these features */
	private Duration horizon = Features.FEATURE_HORIZON;
	
	/**
	 * Constructs new empty features.
	 */
	public Features() {
	}
	
	/**
	 * Constructs new features from a scenario within the default feature
	 * horizon.
	 * 
	 * @param scenario the scenario containing the features
	 * 
	 * @throws IllegalArgumentException if the scenario is invalid
	 */
	public Features(Scenario scenario) {
		this.extractFeatures(scenario);
	}
	
	/**
	 * Constructs new features from a scenario within a feature horizon.
	 * 
	 * @param scenario the scenario containing the features
	 * @param horizon the feature horizon
	 * 
	 * @throws IllegalArgumentException if the scenario or horizon are invalid
	 */
	public Features(Scenario scenario, Duration horizon) {
		this.setHorizon(horizon);
		this.extractFeatures(scenario);
	}
	
	/**
	 * Gets the horizon of these features.
	 * 
	 * @return the horizon of these features
	 */
	public Duration getHorizon() {
		return this.horizon;
	}
	
	/**
	 * Sets the horizon of these features.
	 * 
	 * @param horizon the horizon to be set
	 * 
	 * @throws IllegalArgumentException if the horizon is null, negative or zero
	 */
	public void setHorizon(Duration horizon) {
		if ((null == horizon) || ((null != horizon)
				&& (horizon.isNegative() || horizon.isZero()))) {
			throw new IllegalArgumentException("feature horizon is invalid");
		}
		this.horizon = horizon;
	}
	
	/**
	 * Extracts these features from a scenario within the current feature
	 * horizon.
	 * 
	 * @param scenario the scenario containing the features
	 * 
	 * @throws IllegalArgumentException if the scenario is invalid
	 */
	public void extractFeatures(Scenario scenario) {
		this.extractAircraftFeatures(scenario);
		this.extractEnvironmentFeatures(scenario);
		this.extractPoisFeatures(scenario);
		this.extractPolicyFeatures(scenario);
	}
	
	/**
	 * Extracts aircraft features from a scenario within the current feature
	 * horizon.
	 * 
	 * @param scenario the scenario containing the aircraft features
	 * 
	 * @throws IllegalArgumentException if the scenario is invalid
	 */
	public void extractAircraftFeatures(Scenario scenario) {
		if ((null == scenario) || ((null != scenario))
				&& !scenario.hasAircraft()) {
			throw new IllegalArgumentException("scenario is invalid");
		}
		
		Aircraft aircraft = scenario.getAircraft();
		Environment env = scenario.getEnvironment();
		Planner planner = scenario.getPlanner();
		
		// aircraft capabilities features
		this.put(Features.FEATURE_AIRCRAFT_ALTITUDE,
				aircraft.getReferencePosition().getAltitude());
		this.put(Features.FEATURE_AIRCRAFT_SPEED_HORIZONTAL,
				aircraft.getCapabilities().getCruiseSpeed());
		this.put(Features.FEATURE_AIRCRAFT_SPEED_VERTICAL_CLIMB,
				aircraft.getCapabilities().getCruiseRateOfClimb());
		this.put(Features.FEATURE_AIRCRAFT_SPEED_VERTICAL_DESCENT,
				aircraft.getCapabilities().getCruiseRateOfDescent());
		this.put(Features.FEATURE_AIRCRAFT_VOLUME_SAFETY,
				aircraft.getVolume(env.getGlobe()));
	
		// environment obstacles within the feature horizon
		Set<Obstacle> featureObstacles = new HashSet<>();
		for (Obstacle obstacle : env.getObstacles()) {
			if (obstacle.getCostInterval().intersects(new TimeInterval(
					env.getTime(),
					env.getTime().plus(Features.FEATURE_HORIZON)))) {
				featureObstacles.add(obstacle);
			}
		}
		
		
		if (0 == featureObstacles.size()) {
			this.resetAircraftObstacleFeatures();
		} else {
			List<Double> distances = featureObstacles.stream()
					.map(o -> env.getGreatCircleDistance(
							o.getReferencePosition(),
							aircraft.getReferencePosition()))
					.collect(Collectors.toList());
			
			// aircraft obstacle distance features
			this.put(FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_AVG,
					distances.stream()
					.mapToDouble(Double::valueOf).average().getAsDouble());
			this.put(FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_MAX,
					distances.stream()
					.mapToDouble(Double::valueOf).max().getAsDouble());
			this.put(FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_MIN,
					distances.stream()
					.mapToDouble(Double::valueOf).min().getAsDouble());
			
			Obstacle nearest = featureObstacles.stream().min(
					new Comparator<Obstacle>() {
				@Override
				public int compare(Obstacle o1, Obstacle o2) {
					return Double.compare(
							env.getGreatCircleDistance(
									o1.getReferencePosition(),
									aircraft.getReferencePosition()),
							env.getGreatCircleDistance(
									o2.getReferencePosition(),
									aircraft.getReferencePosition()));
				}}).get();
			
			// aircraft nearest obstacle features
			this.put(Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_COST,
					nearest.getCostInterval().getCost());
			if (planner.getRiskPolicy().satisfies(nearest.getCostInterval().getCost())) {
				this.put(Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_COST_POLICIES,
						nearest.getCostInterval().getCost());
			} else {
				this.put(Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_COST_POLICIES,
						Double.POSITIVE_INFINITY);
			}			
			this.put(Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_VOLUME,
					nearest.getVolume(env.getGlobe()));
			this.put(Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_VOLUME_RATIO,
					((Double) this.get(Features.FEATURE_AIRCRAFT_VOLUME_SAFETY))
					/ ((Double) this.get(Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_VOLUME)));
		}
	}
	
	/**
	 * Extracts environment features from a scenario within the current feature
	 * horizon.
	 * 
	 * @param scenario the scenario containing the environment features
	 * 
	 * @throws IllegalArgumentException if the scenario is invalid
	 */
	public void extractEnvironmentFeatures(Scenario scenario) {
		if (null == scenario) {
			throw new IllegalArgumentException("scenario is invalid");
		}
		
		Environment env = scenario.getEnvironment();
		Planner planner = scenario.getPlanner();
		
		// environment space features
		this.put(Features.FEATURE_ENVIRONMENT_DIAMETER, env.getDiameter());
		this.put(Features.FEATURE_ENVIRONMENT_VOLUME, env.getVolume());
		
		// environment obstacles within the feature horizon
		Set<Obstacle> featureObstacles = new HashSet<>();
		for (Obstacle obstacle : env.getObstacles()) {
			if (obstacle.getCostInterval().intersects(new TimeInterval(
					env.getTime(),
					env.getTime().plus(Features.FEATURE_HORIZON)))) {
				featureObstacles.add(obstacle);
			}
		}
		
		// environment obstacle features
		if (0 == featureObstacles.size()) {
			this.resetEnvironmentObstacleFeatures();
		} else {
			// distinct environment obstacles (movement interpolation)
			HashMap<String, Set<Obstacle>> distinctObstacles = new HashMap<>();
			for (Obstacle featureObstacle : featureObstacles) {
				String key = featureObstacle.getCostInterval().getId();
				if (distinctObstacles.containsKey(key)) {
					distinctObstacles.get(key).add(featureObstacle);
				} else {
					HashSet<Obstacle> identicalObstacles = new HashSet<>();
					identicalObstacles.add(featureObstacle);
					distinctObstacles.put(key, identicalObstacles);
				}
			}
			
			this.put(Features.FEATURE_ENVIRONMENT_OBSTACLES_COUNT,
					distinctObstacles.keySet().size());
			this.put(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_POLICIES, env.getCost(
					env.getTime(),
					env.getTime().plus(Features.FEATURE_HORIZON),
					planner.getCostPolicy(), planner.getRiskPolicy()));
			
			// distinct obstacles cost
			ArrayList<Double> distinctCosts = new ArrayList<>();
			for (String key : distinctObstacles.keySet()) {
				distinctCosts.add(distinctObstacles.get(key).stream()
						.mapToDouble(o -> o.getCostInterval().getCost())
						.findFirst().getAsDouble());
			}
			
			this.put(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_AVG,
					distinctCosts.stream()
					.mapToDouble(Double::valueOf).average().getAsDouble());
			this.put(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_MAX,
					distinctCosts.stream()
					.mapToDouble(Double::valueOf).max().getAsDouble());
			this.put(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_MIN,
					distinctCosts.stream()
					.mapToDouble(Double::valueOf).min().getAsDouble());
			this.put(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_SUM,
					distinctCosts.stream()
					.mapToDouble(Double::valueOf).sum());
			
			// distinct average obstacles volume
			ArrayList<Double> distinctVolumes = new ArrayList<>();
			for (String key : distinctObstacles.keySet()) {
				distinctVolumes.add(distinctObstacles.get(key).stream()
						.mapToDouble(o -> o.getVolume(env.getGlobe()))
						.average().getAsDouble());
			}
			
			this.put(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_AVG,
					featureObstacles.stream()
					.mapToDouble(o -> o.getVolume(env.getGlobe()))
					.average().getAsDouble());
			this.put(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_MAX,
					featureObstacles.stream()
					.mapToDouble(o -> o.getVolume(env.getGlobe()))
					.max().getAsDouble());
			this.put(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_MIN,
					featureObstacles.stream()
					.mapToDouble(o -> o.getVolume(env.getGlobe()))
					.min().getAsDouble());
			// ignore mutual time-dependent obstacle intersections
			this.put(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_SUM,
					distinctVolumes.stream()
					.mapToDouble(Double::valueOf).sum());
			this.put(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_RATIO,
					((Double) this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_SUM))
					/ ((Double) this.get(Features.FEATURE_ENVIRONMENT_VOLUME)));
		}
	}
	
	/**
	 * Extracts POIs features from a scenario within the current feature
	 * horizon.
	 * 
	 * @param scenario the scenario containing the POIs features
	 * 
	 * @throws IllegalArgumentException if the scenario is invalid
	 */
	public void extractPoisFeatures(Scenario scenario) {
		if ((null == scenario) || ((null != scenario)
				&& (2 > scenario.getWaypoints().size()))) {
			throw new IllegalArgumentException("scenario is invalid");
		}
		
		Environment env = scenario.getEnvironment();
		Planner planner = scenario.getPlanner();
		List<Waypoint> pois = scenario.getWaypoints();
		
		// distances between consecutive POIs
		List<Double> poiDistances = new ArrayList<>();
		Iterator<Waypoint> poiIterator = pois.iterator();
		Waypoint current = poiIterator.next();
		while (poiIterator.hasNext()) {
			Waypoint next = poiIterator.next();
			poiDistances.add(env.getDistance(current, next));
			current = next;
		}
		
		// POI distance features
		this.put(Features.FEATURE_POIS_DISTANCE_AVG,
				poiDistances.stream()
				.mapToDouble(Double::valueOf).average().getAsDouble());
		this.put(Features.FEATURE_POIS_DISTANCE_MAX,
				poiDistances.stream()
				.mapToDouble(Double::valueOf).max().getAsDouble());
		this.put(Features.FEATURE_POIS_DISTANCE_MIN,
				poiDistances.stream()
				.mapToDouble(Double::valueOf).min().getAsDouble());
		
		ObstaclePath poiPath = new ObstaclePath(pois);
		Extent poiExtent = poiPath.getExtent(env.getGlobe());
		if ((null != poiExtent) && (poiExtent instanceof Box)) {
			PlanningContinuum poiContinuum = new PlanningContinuum(
					new com.cfar.swim.worldwind.geom.Box((Box) poiExtent));
			poiContinuum.setGlobe(env.getGlobe());
			
			// POI space features
			this.put(Features.FEATURE_POIS_DIAMETER, poiContinuum.getDiameter());
			this.put(Features.FEATURE_POIS_VOLUME, poiContinuum.getVolume());
			
			if (this.containsKey(Features.FEATURE_ENVIRONMENT_VOLUME) &&
					(0d != (Double) this.get(Features.FEATURE_ENVIRONMENT_VOLUME))) {
				this.put(Features.FEATURE_POIS_ENVIRONMENT_VOLUME_RATIO,
						(Double) this.get(Features.FEATURE_POIS_VOLUME)
						/ (Double) this.get(Features.FEATURE_ENVIRONMENT_VOLUME));
			}
			
			// environment obstacles within the feature horizon
			Set<Obstacle> featureObstacles = new HashSet<>();
			for (Obstacle obstacle : env.getObstacles()) {
				if (obstacle.getCostInterval().intersects(new TimeInterval(
						env.getTime(),
						env.getTime().plus(Features.FEATURE_HORIZON)))) {
					featureObstacles.add(obstacle);
				}
			}
			
			// check obstacle intersections using dedicated GJK collision test
			for (Obstacle featureObstacle : featureObstacles) {
				poiContinuum.embed(featureObstacle);
			}
			
			// distinct POI continuum obstacles (movement interpolation)
			Set<Obstacle> poiObstacles = new HashSet<>();
			HashMap<String, Set<Obstacle>> distinctObstacles = new HashMap<>();
			for (Obstacle poiObstacle : poiContinuum.getObstacles()) {
				poiObstacles.add(poiObstacle);
				String key = poiObstacle.getCostInterval().getId();
				if (distinctObstacles.containsKey(key)) {
					distinctObstacles.get(key).add(poiObstacle);
				} else {
					HashSet<Obstacle> identicalObstacles = new HashSet<>();
					identicalObstacles.add(poiObstacle);
					distinctObstacles.put(key, identicalObstacles);
				}
			}
			
			if (0 == poiObstacles.size()) {
				this.resetPoisObstacleFeatures();
			} else {
				this.put(Features.FEATURE_POIS_OBSTACLES_COUNT,
						distinctObstacles.keySet().size());
				this.put(Features.FEATURE_POIS_OBSTACLES_COST_POLICIES,
						poiContinuum.getCost(
								env.getTime(),
								env.getTime().plus(Features.FEATURE_HORIZON),
								planner.getCostPolicy(),
								planner.getRiskPolicy()));
				
				// distinct obstacles cost
				ArrayList<Double> distinctCosts = new ArrayList<>();
				for (String key : distinctObstacles.keySet()) {
					distinctCosts.add(distinctObstacles.get(key).stream()
							.mapToDouble(o -> o.getCostInterval().getCost())
							.findFirst().getAsDouble());
				}
				
				this.put(Features.FEATURE_POIS_OBSTACLES_COST_AVG,
						distinctCosts.stream()
						.mapToDouble(Double::valueOf).average().getAsDouble());
				this.put(Features.FEATURE_POIS_OBSTACLES_COST_MAX,
						distinctCosts.stream()
						.mapToDouble(Double::valueOf).max().getAsDouble());
				this.put(Features.FEATURE_POIS_OBSTACLES_COST_MIN,
						distinctCosts.stream()
						.mapToDouble(Double::valueOf).min().getAsDouble());
				this.put(Features.FEATURE_POIS_OBSTACLES_COST_SUM,
						distinctCosts.stream()
						.mapToDouble(Double::valueOf).sum());
				
				// distinct average obstacles volume
				ArrayList<Double> distinctVolumes = new ArrayList<>();
				for (String key : distinctObstacles.keySet()) {
					distinctVolumes.add(distinctObstacles.get(key).stream()
							.mapToDouble(o -> o.getVolume(env.getGlobe()))
							.average().getAsDouble());
				}
				
				this.put(Features.FEATURE_POIS_OBSTACLES_VOLUME_AVG,
						poiObstacles.stream()
						.mapToDouble(o -> o.getVolume(env.getGlobe()))
						.average().getAsDouble());
				this.put(Features.FEATURE_POIS_OBSTACLES_VOLUME_MAX,
						poiObstacles.stream()
						.mapToDouble(o -> o.getVolume(env.getGlobe()))
						.max().getAsDouble());
				this.put(Features.FEATURE_POIS_OBSTACLES_VOLUME_MIN,
						poiObstacles.stream()
						.mapToDouble(o -> o.getVolume(env.getGlobe()))
						.min().getAsDouble());
				// ignore mutual time-dependent obstacle intersections
				this.put(Features.FEATURE_POIS_OBSTACLES_VOLUME_SUM,
						distinctVolumes.stream()
						.mapToDouble(Double::valueOf).sum());
				this.put(Features.FEATURE_POIS_OBSTACLES_VOLUME_RATIO,
						((Double) this.get(Features.FEATURE_POIS_OBSTACLES_VOLUME_SUM))
						/ ((Double) this.get(Features.FEATURE_POIS_VOLUME)));
				
				if (this.containsKey(Features.FEATURE_ENVIRONMENT_OBSTACLES_COUNT) &&
						(0 != (Integer) this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_COUNT))) {
					this.put(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COUNT_RATIO,
							((Integer) this.get(Features.FEATURE_POIS_OBSTACLES_COUNT)).doubleValue()
							/ ((Integer) this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_COUNT)).doubleValue());
				}
				if (this.containsKey(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_POLICIES) &&
						(0d != (Double) this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_POLICIES))) {
					this.put(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_POLICIES_RATIO,
							(Double) this.get(Features.FEATURE_POIS_OBSTACLES_COST_POLICIES)
							/ (Double) this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_POLICIES));
				}
				if (this.containsKey(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_AVG) &&
						(0d != (Double) this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_AVG))) {
					this.put(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_AVG_RATIO,
							(Double) this.get(Features.FEATURE_POIS_OBSTACLES_COST_AVG)
							/ (Double) this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_AVG));
				}
				if (this.containsKey(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_MAX) &&
						(0d != (Double) this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_MAX))) {
					this.put(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_MAX_RATIO,
							(Double) this.get(Features.FEATURE_POIS_OBSTACLES_COST_MAX)
							/ (Double) this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_MAX));
				}
				if (this.containsKey(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_MIN) &&
						(0d != (Double) this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_MIN))) {
					this.put(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_MIN_RATIO,
							(Double) this.get(Features.FEATURE_POIS_OBSTACLES_COST_MIN)
							/ (Double) this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_MIN));
				}
				if (this.containsKey(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_SUM) &&
						(0d != (Double) this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_SUM))) {
					this.put(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_SUM_RATIO,
							(Double) this.get(Features.FEATURE_POIS_OBSTACLES_COST_SUM)
							/ (Double) this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_SUM));
				}
				if (this.containsKey(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_SUM) &&
						(0d != (Double) this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_SUM))) {
					this.put(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_VOLUME_RATIO,
							(Double) this.get(Features.FEATURE_POIS_OBSTACLES_VOLUME_SUM)
							/ (Double) this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_SUM));
				}
			}
		} else {
			this.resetPoisObstacleFeatures();
		}
	} 
	
	/**
	 * Extracts policy features from a scenario within the current feature
	 * horizon.
	 * 
	 * @param scenario the scenario containing the policy features
	 * 
	 * @throws IllegalArgumentException if the scenario is invalid
	 */
	public void extractPolicyFeatures(Scenario scenario) {
		if (null == scenario) {
			throw new IllegalArgumentException("scenario is invalid");
		}
		
		Planner planner = scenario.getPlanner();
		// operational policy features
		this.put(Features.FEATURE_POLICY_COST, planner.getCostPolicy());
		this.put(Features.FEATURE_POLICY_RISK, planner.getRiskPolicy());
	}
	
	/**
	 * Resets the aircraft obstacle features.
	 */
	protected void resetAircraftObstacleFeatures() {
		this.put(Features.FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_AVG, Double.POSITIVE_INFINITY);
		this.put(Features.FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_MAX, Double.POSITIVE_INFINITY);
		this.put(Features.FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_MIN, Double.POSITIVE_INFINITY);
		this.put(Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_COST, 0d);
		this.put(Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_COST_POLICIES, 0d);
		this.put(Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_VOLUME, 0d);
		this.put(Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_VOLUME_RATIO, 0d);
	}
	
	/**
	 * Resets the environment obstacle features.
	 */
	protected void resetEnvironmentObstacleFeatures() {
		this.put(Features.FEATURE_ENVIRONMENT_OBSTACLES_COUNT, 0);
		this.put(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_POLICIES, 0d);
		this.put(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_AVG, 0d);
		this.put(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_MAX, 0d);
		this.put(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_MIN, 0d);
		this.put(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_SUM, 0d);
		this.put(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_AVG, 0d);
		this.put(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_MAX, 0d);
		this.put(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_MIN, 0d);
		this.put(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_SUM, 0d);
		this.put(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_RATIO, 0d);
	}
	
	/**
	 * Resets the POIs obstacle features.
	 */
	protected void resetPoisObstacleFeatures() {
		this.put(Features.FEATURE_POIS_OBSTACLES_COUNT, 0);
		this.put(Features.FEATURE_POIS_OBSTACLES_COST_POLICIES, 0d);
		this.put(Features.FEATURE_POIS_OBSTACLES_COST_AVG, 0d);
		this.put(Features.FEATURE_POIS_OBSTACLES_COST_MAX, 0d);
		this.put(Features.FEATURE_POIS_OBSTACLES_COST_MIN, 0d);
		this.put(Features.FEATURE_POIS_OBSTACLES_COST_SUM, 0d);
		this.put(Features.FEATURE_POIS_OBSTACLES_VOLUME_AVG, 0d);
		this.put(Features.FEATURE_POIS_OBSTACLES_VOLUME_MAX, 0d);
		this.put(Features.FEATURE_POIS_OBSTACLES_VOLUME_MIN, 0d);
		this.put(Features.FEATURE_POIS_OBSTACLES_VOLUME_SUM, 0d);
		this.put(Features.FEATURE_POIS_OBSTACLES_VOLUME_RATIO, 0d);
		this.put(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COUNT_RATIO, 0d);
		this.put(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_POLICIES_RATIO, 0d);
		this.put(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_AVG_RATIO, 0d);
		this.put(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_MAX_RATIO, 0d);
		this.put(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_MIN_RATIO, 0d);
		this.put(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_SUM_RATIO, 0d);
		this.put(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_VOLUME_RATIO, 0d);
	}
	
	/**
	 * Removes all features.
	 */
	public void removeFeatures() {
		this.removeAircraftFeatures();
		this.removeEnvironmentFeatures();
		this.removePoisFeatures();
		this.removePolicyFeatures();
	}
	
	/**
	 * Removes all aircraft features.
	 */
	public void removeAircraftFeatures() {
		this.remove(Features.FEATURE_AIRCRAFT_ALTITUDE);
		this.remove(Features.FEATURE_AIRCRAFT_SPEED_HORIZONTAL);
		this.remove(Features.FEATURE_AIRCRAFT_SPEED_VERTICAL_CLIMB);
		this.remove(Features.FEATURE_AIRCRAFT_SPEED_VERTICAL_DESCENT);
		this.remove(Features.FEATURE_AIRCRAFT_VOLUME_SAFETY);
		this.remove(Features.FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_AVG);
		this.remove(Features.FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_MAX);
		this.remove(Features.FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_MIN);
		this.remove(Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_COST);
		this.remove(Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_COST_POLICIES);
		this.remove(Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_VOLUME);
		this.remove(Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_VOLUME_RATIO);
	}
	
	/**
	 * Removes all environment features.
	 */
	public void removeEnvironmentFeatures() {
		this.remove(Features.FEATURE_ENVIRONMENT_DIAMETER);
		this.remove(Features.FEATURE_ENVIRONMENT_VOLUME);
		this.remove(Features.FEATURE_ENVIRONMENT_OBSTACLES_COUNT);
		this.remove(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_POLICIES);
		this.remove(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_AVG);
		this.remove(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_MAX);
		this.remove(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_MIN);
		this.remove(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_SUM);
		this.remove(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_AVG);
		this.remove(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_MAX);
		this.remove(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_MIN);
		this.remove(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_RATIO);
		this.remove(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_SUM);
	}
	
	/**
	 * Removes all POIs features.
	 */
	public void removePoisFeatures() {
		this.remove(Features.FEATURE_POIS_DIAMETER);
		this.remove(Features.FEATURE_POIS_VOLUME);
		this.remove(Features.FEATURE_POIS_DISTANCE_AVG);
		this.remove(Features.FEATURE_POIS_DISTANCE_MAX);
		this.remove(Features.FEATURE_POIS_DISTANCE_MIN);
		this.remove(Features.FEATURE_POIS_OBSTACLES_COUNT);
		this.remove(Features.FEATURE_POIS_OBSTACLES_COST_POLICIES);
		this.remove(Features.FEATURE_POIS_OBSTACLES_COST_AVG);
		this.remove(Features.FEATURE_POIS_OBSTACLES_COST_MAX);
		this.remove(Features.FEATURE_POIS_OBSTACLES_COST_MIN);
		this.remove(Features.FEATURE_POIS_OBSTACLES_COST_SUM);
		this.remove(Features.FEATURE_POIS_OBSTACLES_VOLUME_AVG);
		this.remove(Features.FEATURE_POIS_OBSTACLES_VOLUME_MAX);
		this.remove(Features.FEATURE_POIS_OBSTACLES_VOLUME_MIN);
		this.remove(Features.FEATURE_POIS_OBSTACLES_VOLUME_RATIO);
		this.remove(Features.FEATURE_POIS_OBSTACLES_VOLUME_SUM);
		this.remove(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COUNT_RATIO);
		this.remove(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_POLICIES_RATIO);
		this.remove(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_AVG_RATIO);
		this.remove(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_MAX_RATIO);
		this.remove(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_MIN_RATIO);
		this.remove(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_SUM_RATIO);
		this.remove(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_VOLUME_RATIO);
		this.remove(Features.FEATURE_POIS_ENVIRONMENT_VOLUME_RATIO);
	}
	
	/**
	 * Removes all policy features.
	 */
	public void removePolicyFeatures() {
		this.remove(Features.FEATURE_POLICY_COST);
		this.remove(Features.FEATURE_POLICY_RISK);
	}
	
	/**
	 * Gets a string representation of these features.
	 * 
	 * @return the string representation of these features
	 */
	public String toString() {
		String features = "________________________________________________________________________________\n";
		
		if (this.containsKey(Features.FEATURE_AIRCRAFT_ALTITUDE)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_AIRCRAFT_ALTITUDE) + " = "
					+ this.get(Features.FEATURE_AIRCRAFT_ALTITUDE)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_AIRCRAFT_VOLUME_SAFETY)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_AIRCRAFT_VOLUME_SAFETY) + " = "
					+ this.get(Features.FEATURE_AIRCRAFT_VOLUME_SAFETY)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_AIRCRAFT_SPEED_HORIZONTAL)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_AIRCRAFT_SPEED_HORIZONTAL) + " = "
					+ this.get(Features.FEATURE_AIRCRAFT_SPEED_HORIZONTAL)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_AIRCRAFT_SPEED_VERTICAL_CLIMB)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_AIRCRAFT_SPEED_VERTICAL_CLIMB) + " = "
					+ this.get(Features.FEATURE_AIRCRAFT_SPEED_VERTICAL_CLIMB)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_AIRCRAFT_SPEED_VERTICAL_DESCENT)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_AIRCRAFT_SPEED_VERTICAL_DESCENT) + " = "
					+ this.get(Features.FEATURE_AIRCRAFT_SPEED_VERTICAL_DESCENT)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_AVG)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_AVG) + " = "
					+ this.get(Features.FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_AVG)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_MAX)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_MAX) + " = "
					+ this.get(Features.FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_MAX)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_MIN)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_MIN) + " = "
					+ this.get(Features.FEATURE_AIRCRAFT_OBSTACLES_DISTANCE_MIN)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_COST)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_COST) + " = "
					+ this.get(Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_COST)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_COST_POLICIES)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_COST_POLICIES) + " = "
					+ this.get(Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_COST_POLICIES)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_VOLUME)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_VOLUME) + " = "
					+ this.get(Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_VOLUME)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_VOLUME_RATIO)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_VOLUME_RATIO) + " = "
					+ this.get(Features.FEATURE_AIRCRAFT_OBSTACLES_NEAREST_VOLUME_RATIO)
					+ "\n");
		}
		
		features = features.concat("--------------------------------------------------------------------------------\n");
		
		if (this.containsKey(Features.FEATURE_ENVIRONMENT_DIAMETER)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_ENVIRONMENT_DIAMETER) + " = "
					+ this.get(Features.FEATURE_ENVIRONMENT_DIAMETER)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_ENVIRONMENT_VOLUME)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_ENVIRONMENT_VOLUME) + " = "
					+ this.get(Features.FEATURE_ENVIRONMENT_VOLUME)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_ENVIRONMENT_OBSTACLES_COUNT)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_ENVIRONMENT_OBSTACLES_COUNT) + " = "
					+ this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_COUNT)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_POLICIES)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_POLICIES) + " = "
					+ this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_POLICIES)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_AVG)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_AVG) + " = "
					+ this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_AVG)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_MAX)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_MAX) + " = "
					+ this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_MAX)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_MIN)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_MIN) + " = "
					+ this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_MIN)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_SUM)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_SUM) + " = "
					+ this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_COST_SUM)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_AVG)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_AVG) + " = "
					+ this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_AVG)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_MAX)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_MAX) + " = "
					+ this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_MAX)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_MIN)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_MIN) + " = "
					+ this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_MIN)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_SUM)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_SUM) + " = "
					+ this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_SUM)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_RATIO)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_RATIO) + " = "
					+ this.get(Features.FEATURE_ENVIRONMENT_OBSTACLES_VOLUME_RATIO)
					+ "\n");
		}
		
		features = features.concat("--------------------------------------------------------------------------------\n");
		
		if (this.containsKey(Features.FEATURE_POIS_DISTANCE_AVG)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_DISTANCE_AVG) + " = "
					+ this.get(Features.FEATURE_POIS_DISTANCE_AVG)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_DISTANCE_MAX)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_DISTANCE_MAX) + " = "
					+ this.get(Features.FEATURE_POIS_DISTANCE_MAX)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_DISTANCE_MIN)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_DISTANCE_MIN) + " = "
					+ this.get(Features.FEATURE_POIS_DISTANCE_MIN)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_DIAMETER)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_DIAMETER) + " = "
					+ this.get(Features.FEATURE_POIS_DIAMETER)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_VOLUME)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_VOLUME) + " = "
					+ this.get(Features.FEATURE_POIS_VOLUME)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_OBSTACLES_COUNT)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_OBSTACLES_COUNT) + " = "
					+ this.get(Features.FEATURE_POIS_OBSTACLES_COUNT)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_OBSTACLES_COST_POLICIES)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_OBSTACLES_COST_POLICIES) + " = "
					+ this.get(Features.FEATURE_POIS_OBSTACLES_COST_POLICIES)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_OBSTACLES_COST_AVG)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_OBSTACLES_COST_AVG) + " = "
					+ this.get(Features.FEATURE_POIS_OBSTACLES_COST_AVG)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_OBSTACLES_COST_MAX)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_OBSTACLES_COST_MAX) + " = "
					+ this.get(Features.FEATURE_POIS_OBSTACLES_COST_MAX)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_OBSTACLES_COST_MIN)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_OBSTACLES_COST_MIN) + " = "
					+ this.get(Features.FEATURE_POIS_OBSTACLES_COST_MIN)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_OBSTACLES_COST_SUM)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_OBSTACLES_COST_SUM) + " = "
					+ this.get(Features.FEATURE_POIS_OBSTACLES_COST_SUM)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_OBSTACLES_VOLUME_AVG)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_OBSTACLES_VOLUME_AVG) + " = "
					+ this.get(Features.FEATURE_POIS_OBSTACLES_VOLUME_AVG)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_OBSTACLES_VOLUME_MAX)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_OBSTACLES_VOLUME_MAX) + " = "
					+ this.get(Features.FEATURE_POIS_OBSTACLES_VOLUME_MAX)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_OBSTACLES_VOLUME_MIN)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_OBSTACLES_VOLUME_MIN) + " = "
					+ this.get(Features.FEATURE_POIS_OBSTACLES_VOLUME_MIN)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_OBSTACLES_VOLUME_SUM)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_OBSTACLES_VOLUME_SUM) + " = "
					+ this.get(Features.FEATURE_POIS_OBSTACLES_VOLUME_SUM)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_OBSTACLES_VOLUME_RATIO)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_OBSTACLES_VOLUME_RATIO) + " = "
					+ this.get(Features.FEATURE_POIS_OBSTACLES_VOLUME_RATIO)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COUNT_RATIO)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COUNT_RATIO) + " = "
					+ this.get(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COUNT_RATIO)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_POLICIES_RATIO)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_POLICIES_RATIO) + " = "
					+ this.get(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_POLICIES_RATIO)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_AVG_RATIO)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_AVG_RATIO) + " = "
					+ this.get(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_AVG_RATIO)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_MAX_RATIO)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_MAX_RATIO) + " = "
					+ this.get(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_MAX_RATIO)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_MIN_RATIO)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_MIN_RATIO) + " = "
					+ this.get(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_MIN_RATIO)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_SUM_RATIO)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_SUM_RATIO) + " = "
					+ this.get(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_COST_SUM_RATIO)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_VOLUME_RATIO)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_VOLUME_RATIO) + " = "
					+ this.get(Features.FEATURE_POIS_ENVIRONMENT_OBSTACLES_VOLUME_RATIO)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POIS_ENVIRONMENT_VOLUME_RATIO)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POIS_ENVIRONMENT_VOLUME_RATIO) + " = "
					+ this.get(Features.FEATURE_POIS_ENVIRONMENT_VOLUME_RATIO)
					+ "\n");
		}
		
		features = features.concat("--------------------------------------------------------------------------------\n");
		
		if (this.containsKey(Features.FEATURE_POLICY_COST)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POLICY_COST) + " = "
					+ this.get(Features.FEATURE_POLICY_COST)
					+ "\n");
		}
		if (this.containsKey(Features.FEATURE_POLICY_RISK)) {
			features = features.concat(this.dictionary.getString(
					Features.FEATURE_POLICY_RISK) + " = "
					+ this.get(Features.FEATURE_POLICY_RISK)
					+ "\n");
		}
		
		features = features.concat("________________________________________________________________________________\n");
			
		return features;
	}
	
}
