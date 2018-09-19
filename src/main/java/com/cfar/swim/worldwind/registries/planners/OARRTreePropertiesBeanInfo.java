/**
 * 
 */
package com.cfar.swim.worldwind.registries.planners;

import java.beans.BeanDescriptor;
import java.beans.IntrospectionException;
import java.beans.PropertyDescriptor;

/**
 * Realizes the properties bean of an online anytime RRTree planner with the
 * property descriptors for each parameter.
 * 
 * @author Manuel
 *
 */
public class OARRTreePropertiesBeanInfo extends ARRTreePropertiesBeanInfo {

	/** the class which this bean info refers to */
	private final static Class<OARRTreeProperties> beanClass = OARRTreeProperties.class;

	/** the category of parameters that are online related */
	protected final static String CATEGORY_ONLINE = "Online Parameters";

	/**
	 * Customizes the property descriptors for each parameter of an online anytime
	 * RRTree planner.
	 * 
	 * @return the array of property descriptors
	 * 
	 * @see AbstractPlannerPropertiesBeanInfo#getPropertyDescriptors()
	 */
	@Override
	public PropertyDescriptor[] getPropertyDescriptors() {

		try {
			PropertyDescriptor online = this.createProperty(beanClass, "online",
					"Online Planning",
					"the starting position of the plan is updated as the aircraft moves until it reaches the goal",
					CATEGORY_ONLINE);
			PropertyDescriptor positionThreshold = this.createProperty(beanClass, "positionThreshold",
					"Position Threshold (m)",
					"the distance threshold to consider a position displacement as worthy of a new plan",
					CATEGORY_ONLINE);

			PropertyDescriptor rvNew[] = { online, positionThreshold };
			PropertyDescriptor rvOld[] = super.getPropertyDescriptors();
			PropertyDescriptor rv[] = this.addPropertyDescriptors(rvOld, rvNew);

			return rv;
		} catch (IntrospectionException e) {
			throw new Error(e.toString());
		}
	}

	/**
	 * Gets the bean descriptor of this online anytime RRTree properties bean info.
	 * 
	 * @return the bean descriptor of this class
	 */
	public BeanDescriptor getBeanDescriptor() {
		return new BeanDescriptor(beanClass, null);
	}

}
