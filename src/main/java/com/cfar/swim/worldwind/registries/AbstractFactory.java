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
package com.cfar.swim.worldwind.registries;

import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;

import com.cfar.swim.worldwind.session.Scenario;

/**
 * Abstracts a factory of registered items. Factories can create items
 * such as environments and planners according to a customized specification.
 * 
 * @author Stephan Heinemann
 *
 * @param <Registree> the registered item
 * 
 * @see Registry
 * @see Specification
 */
public abstract class AbstractFactory<Registree> implements Factory<Registree> {

	/** the specification of this abstract factory */
	private Specification<Registree> specification;
	
	/** the scenario of this abstract factory */
	private Scenario scenario = null;
	
	/** the active scenario change listener of this abstract factory */
	private ActiveScenarioChangeListener ascl = new ActiveScenarioChangeListener();
	
	/**
	 * Constructs a new abstract factory without a customized specification.
	 */
	public AbstractFactory() {
		this.specification = null;
	}
	
	/**
	 * Constructs a new abstract factory to create registered items according
	 * to a customized specification.
	 * 
	 * @param specification the specification describing the registered item
	 */
	public AbstractFactory(Specification<Registree> specification) {
		this.specification = specification;
	}
	
	/**
	 * Gets the specification of this abstract factory.
	 * 
	 * @return the specification of this abstract factory
	 * 
	 * @see Factory#getSpecification()
	 */
	@Override
	public Specification<Registree> getSpecification() {
		return this.specification;
	}
	
	/**
	 * Sets the specification of this abstract factory.
	 * 
	 * @param specification the specification to be set
	 * 
	 * @see Factory#setSpecification(Specification)
	 */
	@Override
	public void setSpecification(Specification<Registree> specification) {
		this.specification = specification;
	}
	
	/**
	 * Determines whether or not this abstract factory has a specification.
	 * 
	 * @return true if this abstract factory has a specification,
	 *         false otherwise
	 * 
	 * @see Factory#hasSpecification()
	 */
	@Override
	public boolean hasSpecification() {
		return (null != this.specification);
	}
	
	/**
	 * Creates an item according to the specification of this abstract factory.
	 * 
	 * @return the created item
	 * 
	 * @see Factory#createInstance()
	 */
	@Override
	public abstract Registree createInstance();
	
	/**
	 * Gets the scenario of this abstract factory.
	 * 
	 * @return the scenario of this abstract factory
	 */
	protected Scenario getScenario() {
		return this.scenario;
	}
	
	/**
	 * Sets the scenario of this abstract factory.
	 * 
	 * @param scenario the scenario to be set
	 */
	protected void setScenario(Scenario scenario) {
		this.scenario = scenario;
	}
	
	/**
	 * Determines whether or not this abstract factory has a scenario.
	 * 
	 * @return true if this abstract factory has a scenario, false otherwise
	 */
	protected boolean hasScenario() {
		return (null != this.scenario);
	}
	
	/**
	 * Gets the active scenario change listener of this abstract factory.
	 * 
	 * @return the active scenario change listener of this abstract factory
	 */
	public PropertyChangeListener getActiveScenarioChangeListener() {
		return this.ascl;
	}
	
	/**
	 * Realizes an active scenario change listener for this abstract factory.
	 * 
	 * @author Stephan Heinemann
	 */
	private class ActiveScenarioChangeListener implements PropertyChangeListener {
		
		/**
		 * Notifies the abstract factory about an active scenario change.
		 * 
		 * @param evt the property change event
		 */
		@Override
		public void propertyChange(PropertyChangeEvent evt) {
			if (evt.getNewValue() instanceof Scenario) {
				setScenario((Scenario) evt.getNewValue());
			}
		}
	}

}
