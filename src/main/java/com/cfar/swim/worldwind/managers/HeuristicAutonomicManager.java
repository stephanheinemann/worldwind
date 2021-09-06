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
package com.cfar.swim.worldwind.managers;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.cfar.swim.worldwind.planners.LifelongPlanner;
import com.cfar.swim.worldwind.session.Scenario;
import com.cfar.swim.worldwind.session.Session;

import gov.nasa.worldwind.geom.Position;

public class HeuristicAutonomicManager extends AbstractAutonomicManager {

	// TODO: real parallel service if possible (performance evaluation)
	// otherwise sequential
	ScheduledExecutorService executor = Executors.newScheduledThreadPool(4);
	
	@Override
	public void run(Session managedSession) {
		for (Scenario managedScenario : this.getManagedScenarios()) {
			System.out.println(managedScenario.getId());
			System.out.println(this.getFeatures(managedScenario));
		
			LifelongPlanner managedPlanner = (LifelongPlanner) managedScenario.getPlanner();
			List<Position> pois = new ArrayList<Position>();
			pois.addAll(managedScenario.getWaypoints());
			
			this.executor.schedule(new Runnable() {
				@Override
				public void run() {
					System.out.println("terminating planner " + managedScenario.getId());
					managedPlanner.terminate();
				}
				
			}, 10l, TimeUnit.SECONDS);
			
			// TODO: a managed planner could be a task itself?
			this.executor.execute(new Runnable() {
				@Override
				public void run() {
					
					System.out.println("running planner " + managedScenario.getId());
					Position origin = pois.remove(0);
					Position destination = pois.remove(pois.size() - 1);
					if (pois.isEmpty()) {
						managedPlanner.plan(origin, destination, managedSession.getActiveScenario().getTime());
					} else {
						managedPlanner.plan(origin, destination, pois, managedSession.getActiveScenario().getTime());
					}
				}
			});// TODO Auto-generated method stub
		}
	}
	// TODO: implement
}
