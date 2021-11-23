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
package com.cfar.swim.worldwind.tests;

import java.net.URI;
import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.Comparator;
import java.util.HashMap;
import java.util.TreeSet;

import org.junit.Test;

import com.cfar.swim.worldwind.flight.FlightPhase;
import com.cfar.swim.worldwind.managing.Difficulty;
import com.cfar.swim.worldwind.managing.FeatureTuning;
import com.cfar.swim.worldwind.managing.Features;
import com.cfar.swim.worldwind.managing.KnowledgeBase;
import com.cfar.swim.worldwind.managing.Performance;
import com.cfar.swim.worldwind.managing.Severity;
import com.cfar.swim.worldwind.managing.Tuning;
import com.cfar.swim.worldwind.registries.managers.AbstractManagerProperties;

/**
 * Performs criticality tests.
 * 
 * @author Stephan Heinemann
 *
 */
public class CriticalityTest {
	
	@Test
	public void testDifficulty() {
		int lowCount = 0, modCount = 0, subCount = 0, sevCount =0, crtCount = 0;
		
		KnowledgeBase knowledgeBase = new KnowledgeBase();
		knowledgeBase.load(URI.create(AbstractManagerProperties.KNOWLEDGE_BASE_RESOURCE));
		//System.out.println(knowledgeBase.toString());
		
		for (Tuning<?>  tuning : knowledgeBase.getReputation().keySet()) {
			if (tuning instanceof FeatureTuning) {
				Features features = ((FeatureTuning<?>) tuning).getFeatures();
				Difficulty low = Difficulty.createLow(features);
				Difficulty moderate = Difficulty.createModerate(features);
				Difficulty substantial = Difficulty.createSubstantial(features);
				Difficulty severe = Difficulty.createSevere(features);
				Difficulty critical = Difficulty.createCritical(features);
				if (low.covers(features)) lowCount++;
				if (moderate.covers(features)) modCount++;
				if (substantial.covers(features)) subCount++;
				if (severe.covers(features)) sevCount++;
				if (critical.covers(features)) crtCount++;
			}
		}
		System.out.println("difficulty: low = " + lowCount
				+ ", moderate = " + modCount
				+ ", substantial = " + subCount
				+ ", severe = " + sevCount
				+ ", critical = " + crtCount);
	}
	
	@Test
	public void testSeverity() {
		int lowCount = 0, modCount = 0, subCount = 0, sevCount =0, crtCount = 0;
		
		KnowledgeBase knowledgeBase = new KnowledgeBase();
		knowledgeBase.load(URI.create(AbstractManagerProperties.KNOWLEDGE_BASE_RESOURCE));
		//System.out.println(knowledgeBase.toString());
		
		for (Tuning<?>  tuning : knowledgeBase.getReputation().keySet()) {
			if (tuning instanceof FeatureTuning) {
				Features features = ((FeatureTuning<?>) tuning).getFeatures();
				Severity low = Severity.createLow(features);
				Severity moderate = Severity.createModerate(features);
				Severity substantial = Severity.createSubstantial(features);
				Severity severe = Severity.createSevere(features);
				Severity critical = Severity.createCritical(features);
				if (low.covers(features)) lowCount++;
				if (moderate.covers(features)) modCount++;
				if (substantial.covers(features)) subCount++;
				if (severe.covers(features)) sevCount++;
				if (critical.covers(features)) crtCount++;
			}
		}
		System.out.println("severity: low = " + lowCount
				+ ", moderate = " + modCount
				+ ", substantial = " + subCount
				+ ", severe = " + sevCount
				+ ", critical = " + crtCount);
	}
	
	@Test
	public void testFlightPhase() {
		int cruiseCount = 0, transCount = 0, termCount = 0, urgCount =0, emgCount = 0;
		
		KnowledgeBase knowledgeBase = new KnowledgeBase();
		knowledgeBase.load(URI.create(AbstractManagerProperties.KNOWLEDGE_BASE_RESOURCE));
		//System.out.println(knowledgeBase.toString());
		
		for (Tuning<?>  tuning : knowledgeBase.getReputation().keySet()) {
			if (tuning instanceof FeatureTuning) {
				Features features = ((FeatureTuning<?>) tuning).getFeatures();
				FlightPhase cruise = FlightPhase.createCruise(features);
				FlightPhase transition = FlightPhase.createTransition(features);
				FlightPhase terminal = FlightPhase.createTerminal(features);
				FlightPhase urgency = FlightPhase.createUrgency(features);
				FlightPhase emergency = FlightPhase.createEmergency(features);
				if (cruise.covers(features)) cruiseCount++;
				if (transition.covers(features)) transCount++;
				if (terminal.covers(features)) termCount++;
				if (urgency.covers(features)) urgCount++;
				if (emergency.covers(features)) emgCount++;
			}
		}
		System.out.println("flight phase: cruise = " + cruiseCount
				+ ", transition = " + transCount
				+ ", terminal = " + termCount
				+ ", urgency = " + urgCount
				+ ", emergency = " + emgCount);
	}
	
	@Test
	public void testPlot() {
		KnowledgeBase knowledgeBase = new KnowledgeBase();
		knowledgeBase.load(URI.create(AbstractManagerProperties.KNOWLEDGE_BASE_RESOURCE));
		
		ZonedDateTime epoch = ZonedDateTime.parse("2021-11-14T16:23:51.594821-08:00[America/Vancouver]");//ZonedDateTime.now();
		Duration tolerance = Duration.ofMinutes(5);
		
		HashMap<ZonedDateTime, TreeSet<Performance>> epochPerformances = new HashMap<>();
		HashMap<ZonedDateTime, Tuning<?>> epochTuning = new HashMap<>();
		
		// filter
		for (Tuning<?> tuning : knowledgeBase.getReputation().keySet()) {
			if (knowledgeBase.getReputation().hasPerformances(tuning)) {
				for (Performance performance : knowledgeBase.getReputation().get(tuning)) {
					if (performance.getEpoch().isAfter(epoch.minus(tolerance))
						&& performance.getEpoch().isBefore(epoch.plus(tolerance))) {
						
						if (epochPerformances.containsKey(performance.getEpoch())) {
							epochPerformances.get(performance.getEpoch()).add(performance);
						} else {
							TreeSet<Performance> orderedPerformnces = new TreeSet<>(
									new Comparator<Performance>() {
										@Override
										public int compare(Performance p1, Performance p2) {
											return p1.getQuantity().compareTo(p2.getQuantity());
										}});
							orderedPerformnces.add(performance);
							epochPerformances.put(performance.getEpoch(), orderedPerformnces);
							epochTuning.put(performance.getEpoch(), tuning);
						}
					}
				}
			}
		}
		
		// plot
		for (ZonedDateTime performanceEpoch : epochPerformances.keySet()) {
			System.out.println("________________________________________________________________________________");
			System.out.println("# gnuplot quality");
			System.out.println("set title \"Performance Epoch: "
					+ performanceEpoch + "\"\n"
					+ "set xlabel \"Duration Quantity [s]\"\n"
					+ "set ylabel \"Trajectory Quality\"\n"
					+ "set style line 1 lt 1 lc rgb \"#0000A0\" lw 1 pt 7 ps 2\n"
					+ "plot \"plot.dat\" using 1:2 with lp ls 1 title \""
					+ epochTuning.get(performanceEpoch).getSpecification().getId()
					+ "\" at end"
					);
			System.out.println("# gnuplot performance");
			System.out.println("set title \"Performance Epoch: "
					+ performanceEpoch + "\"\n"
					+ "set xlabel \"Duration Quantity [s]\"\n"
					+ "set ylabel \"Planner Performance\"\n"
					+ "set style line 1 lt 1 lc rgb \"#0000A0\" lw 1 pt 7 ps 2\n"
					+ "plot \"plot.dat\" using 1:3 with lp ls 1 title \""
					+ epochTuning.get(performanceEpoch).getSpecification().getId()
					+ "\" at end"
					);
			
			System.out.println("# gnuplot plot.dat");
			for (Performance epochPerformance : epochPerformances.get(performanceEpoch)) {
				System.out.println(epochPerformance.getQuantity().get()
						+ "\t" + epochPerformance.getQuality().get()
						+ "\t" + epochPerformance.get());
			}
		}
	}
	
}
