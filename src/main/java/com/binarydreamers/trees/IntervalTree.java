/*
Copyright 2013 John Thomas McDole

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
package com.binarydreamers.trees;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Comparator;
import java.util.ConcurrentModificationException;
import java.util.Iterator;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.SortedSet;

/**
 * A binary tree that stores overlapping interval ranges. Implemented as an AVL tree (better searches).
 * Notes: Adapted from "Introduction to Algorithms", second edition,
 *        by Thomas H. Cormen, Charles E. leiserson,
 *           Ronald L. Rivest, Clifford Stein.
 *        chapter 13.2
 * @author John Thomas McDole
 * @param <V> the item type of the interval tree
 */
public class IntervalTree<V extends Comparable<V>> implements SortedSet<Interval<V>> {
	private IntervalNode<V> root;
	private int count;
	private Comparator<Interval<V>> comparator;
	private transient int modCount = 0; // Modification count to the tree, monotonically increasing

	public IntervalTree(Comparator<Interval<V>> comparator) {
		this.comparator = comparator;
	}
	
	public IntervalTree() {
		this.comparator = new Comparator<Interval<V>>() {
			public int compare(Interval<V> o1, Interval<V> o2) {
				int comp = o1.getLower().compareTo(o2.getLower());
				if(comp == 0) {
					comp = o1.getUpper().compareTo(o2.getUpper());
				}
				return comp;
			}
		};
	}	

	/**
	 * Add the element to the tree.
	 */
	public boolean add(Interval<V> element) {
		IntervalNode<V> x = root;

		IntervalNode<V> node = new IntervalNode<V>();
		V max = element.getUpper();
		node.max = max; // Initial value is always ourself.
		node.object = element;

		if (root == null) {
			root = node;
			++count;
			++modCount;
			return true;
		}

		while (true) {
			// We only update if the new element is larger than the existing ones
			if (node.max.compareTo(x.max) > 0) {
				x.max = max;
			}

			int compare = comparator.compare(element, x.object);
			if (0 == compare) {
				return false;
			} else if (compare < 0) {
				if (x.left == null) {
					node.parent = x;
					x.left = node;
					x.balanceFactor -= 1;
					break;
				}
				x = x.left;
			} else {
				if (x.right == null) {
					node.parent = x;
					x.right = node;
					x.balanceFactor += 1;
					break;
				}
				x = x.right;
			}
		}

		++modCount;

		/*
			AVL balancing act (for height balanced trees)
			Now that we've inserted, we've unbalanced some trees, we need
			to follow the tree back up to the root double checking that the tree
			is still balanced and _maybe_ perform a single or double rotation.
			Note: Left additions == -1, Right additions == +1
			Balanced Node = { -1, 0, 1 }, out of balance = { -2, 2 }
			Single rotation when Parent & Child share signed balance,
			Double rotation when sign differs!
		*/
		node = x;
		while (node.balanceFactor != 0 && node.parent != null) {
			// Find out which side of the parent we're on
			if (node.parent.left == node) {
				// Lefties are -1 since we hate lefties
				node.parent.balanceFactor -= 1;
			} else {
				node.parent.balanceFactor += 1;
			}

			node = node.parent;
			if (node.balanceFactor == 2) {
				// Heavy on the right side - Test for which rotation to perform
				if (node.right.balanceFactor == 1) {
					// Single (left) rotation; this will balance everything to zero
					rotateLeft(node);
					node.balanceFactor = node.parent.balanceFactor = 0;
					node = node.parent;

					// Update node's new max; but recalculate the children
					recalculateMax(node.left);
					recalculateMax(node.right);
					recalculateMax(node);
				} else {
					// Double (Right/Left) rotation
					// node will now be old node.right.left
					rotateRightLeft(node);
					node = node.parent; // Update to new parent (old grandchild)
					recalculateMax(node.left);
					recalculateMax(node.right);
					recalculateMax(node);
					if (node.balanceFactor == 1) {
						node.right.balanceFactor = 0;
						node.left.balanceFactor = -1;
					} else if (node.balanceFactor == 0) {
						node.right.balanceFactor = 0;
						node.left.balanceFactor = 0;
					} else {
						node.right.balanceFactor = 1;
						node.left.balanceFactor = 0;
					}
					node.balanceFactor = 0;
				}
				break; // out of loop, we're balanced
			} else if (node.balanceFactor == -2) {
				// Heavy on the left side - Test for which rotation to perform
				if (node.left.balanceFactor == -1) {
					rotateRight(node);
					node.balanceFactor = node.parent.balanceFactor = 0;
					node = node.parent;

					// Update node's new max; but recalculate the children
					recalculateMax(node.left);
					recalculateMax(node.right);
					recalculateMax(node);
				} else {
					// Double (Left/Right) rotation
					// node will now be old node.left.right
					rotateLeftRight(node);
					node = node.parent;
					recalculateMax(node.left);
					recalculateMax(node.right);
					recalculateMax(node);
					if (node.balanceFactor == -1) {
						node.right.balanceFactor = 1;
						node.left.balanceFactor = 0;
					} else if (node.balanceFactor == 0) {
						node.right.balanceFactor = 0;
						node.left.balanceFactor = 0;
					} else {
						node.right.balanceFactor = 0;
						node.left.balanceFactor = -1;
					}
					node.balanceFactor = 0;
				}
				break; // out of loop, we're balanced
			}
		} // end of while(balancing)
		count++;
		return true;
	}

	/**
	 * Test to see if an element is stored in the tree
	 */
	public boolean contains(Object object) {
		@SuppressWarnings("unchecked")
		Interval<V> element = (Interval<V>) object;
		IntervalNode<V> x = getIntervalNode(element);
		return x != null;
	}

	/**
	 * Test to see if an element is stored in the tree
	 */
	private IntervalNode<V> getIntervalNode(Interval<V> element) {
		if (element == null) return null;
		IntervalNode<V> x = root;
		while (x != null) {
			int compare = comparator.compare(element, x.object);
			if( 0 == compare) {
				// This only means our interval matches; we need to search for the exact element
				// We could have been glutons and used a hashmap to back.
				return x; // comparator.compare should check for further restrictions.
			} else if (compare < 0) {
				x = x.left;
			} else {
				x = x.right;
			}
		}
		return null;
	}

	/**
	 * Make sure the node has the right maximum of the subtree
	 * node.max = MAX( node.high, node.left.max, node.right.max );
	 */
	private void recalculateMax(IntervalNode<V> node) {
		V max;
		if(node == null) return;
		if (node.left == node.right && node.right == null) {
			node.max = node.object.getUpper();
			return;
		} else if (node.left == null) {
			max = node.right.max;
		} else if (node.right == null) {
			max = node.left.max;
		} else {
			/* Get the best of the children */
			max = node.left.max.compareTo(node.right.max) > 0 ? node.left.max : node.right.max;
		}

		/* And pit that against our interval */
		node.max = node.object.getUpper().compareTo(max) > 0 ? node.object.getUpper() : max;  
	}

	/**
	 * This function will right rotate/pivot N with its left child, placing
	 * it on the right of its left child.
	 *
	 *      N                      Y
	 *     / \                    / \
	 *    Y   A                  Z   N
	 *   / \          ==>       / \ / \
	 *  Z   B                  D  CB   A
	 * / \
	 *D   C
	 *
	 * Assertion: must have a left element!
	 */
	private void rotateRight(IntervalNode<V> node) {
		IntervalNode<V> y = node.left;
		assert y != null;

		/* turn Y's right subtree(B) into N's left subtree. */
		node.left = y.right;
		if (node.left != null) {
			node.left.parent = node;
		}
		y.parent = node.parent;
		if (y.parent == null) {
			root = y;
		} else {
			if (node.parent.left == node) {
				node.parent.left = y;
			} else {
				node.parent.right = y;
			}
		}
		y.right = node;
		node.parent = y;
	}

	/**
	 * This function will left rotate/pivot N with its right child, placing
	 * it on the left of its right child.
	 *
	 *      N                      Y
	 *     / \                    / \
	 *    A   Y                  N   Z
	 *       / \      ==>       / \ / \
	 *      B   Z              A  BC   D
	 *         / \
	 *        C   D
	 *
	 * Assertion: must have a right element!
	 */
	private void rotateLeft(IntervalNode<V> node) {
		IntervalNode<V> y = node.right;
		assert y != null;

		/* turn Y's left subtree(B) into N's right subtree. */
		node.right = y.left;
		if (node.right != null) {
			node.right.parent = node;
		}
		y.parent = node.parent;
		if (y.parent == null) {
			root = y;
		} else {
			if (node.parent.left == node) {
				y.parent.left = y;
			} else {
				y.parent.right = y;
			}
		}
		y.left = node;
		node.parent = y;
	}

	/**
	 *  This function will double rotate node with right/left operations.
	 *  node is S.
	 *
	 *    S                      G
	 *   / \                    / \
	 *  A   C                  S   C
	 *     / \      ==>       / \ / \
	 *    G   B              A  DC   B
	 *   / \
	 *  D   C
	 */
	private void rotateRightLeft(IntervalNode<V> node) {
		rotateRight(node.right);
		rotateLeft(node);
	}

	/**
	 * This function will double rotate node with left/right operations.
	 * node is S.
	 *
	 *    S                      G
	 *   / \                    / \
	 *  C   A                  C   S
	 * / \          ==>       / \ / \
	 *B   G                  B  CD   A
	 *   / \
	 *  C   D
	 */
	private void rotateLeftRight(IntervalNode<V> node) {
		rotateLeft(node.left);
		rotateRight(node);
	}

	/**
	 *  Return the minimum node for the subtree
	 */
	IntervalNode<V> minimumNode(IntervalNode<V> node) {
		while (node.left != null) {
			node = node.left;
		}
		return node;
	}

	/**
	 *  Return the maximum node for the subtree
	 */
	IntervalNode<V> maxiumNode(IntervalNode<V> node) {
		while (node.right != null) {
			node = node.right;
		}
		return node;
	}

	/**
	 *  Return the next greatest element (or null)
	 */
	IntervalNode<V> successor(IntervalNode<V> node) {
		if (node.right != null) {
			return minimumNode(node.right);
		}
		while (node.parent != null && node == node.parent.right) {
			node = node.parent;
		}
		return node.parent;
	}

	/**
	 *  Return the next smaller element (or null)
	 */
	IntervalNode<V> predecessor(IntervalNode<V> node) {
		if (node.left != null) {
			return maxiumNode(node.left);
		}
		while (node.parent != null && node.parent.left == node) {
			node = node.parent;
		}
		return node.parent;
	}

	/**
	 * A node in the interval tree
	 */
	static class IntervalNode<V extends Comparable<V>> {
		IntervalNode<V> parent;
		IntervalNode<V> left;
		IntervalNode<V> right;
		int balanceFactor;
		Interval<V> object;
		V max;

		@Override
		public String toString() {
			boolean leftSet = left != null;
			boolean rightSet = right != null;
			return "(b:" + balanceFactor + " o:" + object + " l:" + leftSet + " r:" + rightSet
					+ " max:" + max + ")";
		}
	}

	@Override
	public boolean addAll(Collection<? extends Interval<V>> arg0) {
		boolean modified = false;
		for(Interval<V> ele : arg0) {
			modified = add(ele) ? true : modified;
		}
		return modified;
	}

	@Override
	public void clear() {
		count = 0;
		root = null;
	}

	@Override
	public boolean containsAll(Collection<?> arg0) {
		for(Object ele : arg0) {
			if(!contains(ele)) return false;;
		}
		return true;
	}

	@Override
	public boolean isEmpty() {
		return count == 0;
	}

	@Override
	public Iterator<Interval<V>> iterator() {
		return new Iterator<Interval<V>>() {
			int modCountGaurd = modCount;
			IntervalNode<V> current = root != null ? minimumNode(root) : null;
			IntervalNode<V> last = null;

			@Override
			public boolean hasNext() {
				if (modCountGaurd != modCount)
					throw new ConcurrentModificationException();
				if (current == null) {
					return false;
				}
				return current != null;
			}

			@Override
			public Interval<V> next() {
				if (modCountGaurd != modCount)
					throw new ConcurrentModificationException();
				if (current == null) {
					throw new NoSuchElementException();
				}
				last = current;
				current = successor(current);
				if (null == last) {
					throw new NoSuchElementException();
				}
				return last.object;
			}

			@Override
			public void remove() {
				if (modCountGaurd != modCount)
					throw new ConcurrentModificationException();
				if (last == null) {
					throw new IllegalStateException();
				}
				IntervalTree.this.remove(last);
				modCountGaurd++; // we're allowed to delete nodes and keep going
				last = null;
			}
		};
	}

	@Override
	public boolean remove(Object arg0) {
		@SuppressWarnings("unchecked")
		Interval<V> element = (Interval<V>) arg0;
		IntervalNode<V> x = getIntervalNode(element);
		if(x != null) {
			remove(x);
			return true;
		}
		return false;
	}

	public void remove(IntervalNode<V> node) {
		IntervalNode<V> y, w;

		++modCount;
		--count;

		/*
		 * JTM - if you read wikipedia, it states remove the node if its a leaf,
		 * otherwise, replace it with its predecessor or successor. we've not
		 * done that here; though we probably should!
		 */
		if (node.right == null || node.right.left == null) {
			// simple solutions
			if (node.right != null) {
				y = node.right;
				y.parent = node.parent;
				y.balanceFactor = node.balanceFactor - 1;
				y.left = node.left;
				if (y.left != null) {
					y.left.parent = y;
				}
			} else if (node.left != null) {
				y = node.left;
				y.parent = node.parent;
				y.balanceFactor = node.balanceFactor + 1;
			} else {
				y = null;
			}
			if (root == node) {
				root = y;
			} else if (node.parent.left == node) {
				node.parent.left = y;
				if (y == null) {
					// account for leaf deletions changing the balance
					node.parent.balanceFactor += 1;
					y = node.parent; // start searching from here;
				}
			} else {
				node.parent.right = y;
				if (y == null) {
					node.parent.balanceFactor -= 1;
					y = node.parent;
				}
			}
			w = y;
		} else {
			/*
			 * This node is not a leaf; we should find the successor node, swap
			 * it with this* and then update the balance factors.
			 */
			y = successor(node);
			y.left = node.left;
			if (y.left != null) {
				y.left.parent = y;
			}

			w = y.parent;
			w.left = y.right;
			if (w.left != null) {
				w.left.parent = w;
			}
			// known: we're removing from the left
			w.balanceFactor += 1;

			// known due to test for n->r->l above
			y.right = node.right;
			y.right.parent = y;
			y.balanceFactor = node.balanceFactor;

			y.parent = node.parent;
			if (root == node) {
				root = y;
			} else if (node.parent.left == node) {
				node.parent.left = y;
			} else {
				node.parent.right = y;
			}
		}

		// Safe to kill node now; its free to go.
		node.balanceFactor = 0;
		node.left = node.right = node.parent = null;
		node.object = null;

		// Recalculate max values all the way to the top.
		node = w;
		while (node != null) {
			recalculateMax(node);
			node = node.parent;
		}

		// Re-balance to the top, ending early if OK
		node = w;
		while (node != null) {
			if (node.balanceFactor == -1 || node.balanceFactor == 1) {
				// The height of node hasn't changed; done!
				break;
			}
			if (node.balanceFactor == 2) {
				// Heavy on the right side; figure out which rotation to perform
				if (node.right.balanceFactor == -1) {
					rotateRightLeft(node);
					node = node.parent; // old grand-child!
					recalculateMax(node.left);
					recalculateMax(node.right);
					recalculateMax(node);
					if (node.balanceFactor == 1) {
						node.right.balanceFactor = 0;
						node.left.balanceFactor = -1;
					} else if (node.balanceFactor == 0) {
						node.right.balanceFactor = 0;
						node.left.balanceFactor = 0;
					} else {
						node.right.balanceFactor = 1;
						node.left.balanceFactor = 0;
					}
					node.balanceFactor = 0;
				} else {
					// single left-rotation
					rotateLeft(node);
					recalculateMax(node.left);
					recalculateMax(node.right);
					recalculateMax(node);
					if (node.parent.balanceFactor == 0) {
						node.parent.balanceFactor = -1;
						node.balanceFactor = 1;
						break;
					} else {
						node.parent.balanceFactor = 0;
						node.balanceFactor = 0;
						node = node.parent;
						continue;
					}
				}
			} else if (node.balanceFactor == -2) {
				// Heavy on the left
				if (node.left.balanceFactor == 1) {
					rotateLeftRight(node);
					node = node.parent; // old grand-child!
					recalculateMax(node.left);
					recalculateMax(node.right);
					recalculateMax(node);
					if (node.balanceFactor == -1) {
						node.right.balanceFactor = 1;
						node.left.balanceFactor = 0;
					} else if (node.balanceFactor == 0) {
						node.right.balanceFactor = 0;
						node.left.balanceFactor = 0;
					} else {
						node.right.balanceFactor = 0;
						node.left.balanceFactor = -1;
					}
					node.balanceFactor = 0;
				} else {
					rotateRight(node);
					recalculateMax(node.left);
					recalculateMax(node.right);
					recalculateMax(node);
					if (node.parent.balanceFactor == 0) {
						node.parent.balanceFactor = 1;
						node.balanceFactor = -1;
						break;
					} else {
						node.parent.balanceFactor = 0;
						node.balanceFactor = 0;
						node = node.parent;
						continue;
					}
				}
			}

			// continue up the tree for testing
			if (node.parent != null) {
				/*
				 * The concept of balance here is reverse from addition; since
				 * we are taking away weight from one side or the other (thus
				 * the balance changes in favor of the other side)
				 */
				if (node.parent.left == node) {
					node.parent.balanceFactor += 1;
				} else {
					node.parent.balanceFactor -= 1;
				}
			}
			node = node.parent;
		}
	}

	@Override
	public boolean removeAll(Collection<?> arg0) {
		boolean changed = false;
		for(Object ele : arg0) {
			changed = remove(ele) ? true : changed;
		}
		return changed;
	}

	@Override
	public boolean retainAll(Collection<?> arg0) {
		int beforeCount = count;
		Iterator<Interval<V>> it = this.iterator();
		while(it.hasNext()) {
			Interval<V> ele = it.next();
			if(!arg0.contains(ele)) {
				it.remove();
			}
		}
		return count != beforeCount;
	}

	@Override
	public int size() {
		return count;
	}

	@Override
	public Object[] toArray() {
		Object[] array = new Object[count];
		IntervalNode<V> x = minimumNode(root);
		int i = 0;
		while(x!=null) {
			array[i++] = x.object;
			x = successor(x);
		}
		return array;
	}

	@SuppressWarnings("unchecked")
	@Override
	public <Y> Y[] toArray(Y[] a) {
		int size = size();
		Y[] r = a.length >= size ? a : (Y[]) Array.newInstance(a.getClass().getComponentType(),
				size);
		Iterator<Interval<V>> it = iterator();
		for (int i = 0; i < r.length; i++) {
			if (!it.hasNext()) {
				if (a != r)
					return Arrays.copyOf(r, i);
				r[i] = null; // null-terminate
				return r;
			}
			r[i] = (Y) it.next();
		}
		// it.hasNext() ? finishToArray(r, it) : - meh.
		return r;
	}

	@Override
	public Comparator<Interval<V>> comparator() {
		return comparator;
	}

	@Override
	public Interval<V> first() {
		IntervalNode<V> min = minimumNode(root);
		return min != null ? min.object : null;
	}

	@Override
	public Interval<V> last() {
		IntervalNode<V> max = maxiumNode(root);
		return max != null ? max.object : null;
	}

	@Override
	public SortedSet<Interval<V>> headSet(Interval<V> toElement) {
		return new SubIntervalTree<V>(this, null, toElement);
	}

	@Override
	public SortedSet<Interval<V>> subSet(Interval<V> fromElement, Interval<V> toElement) {
		return new SubIntervalTree<V>(this, fromElement, toElement);
	}

	@Override
	public SortedSet<Interval<V>> tailSet(Interval<V> fromElement) {
		return new SubIntervalTree<V>(this, fromElement, null);
	}

	static class SubIntervalTree<T extends Comparable<T>> implements SortedSet<Interval<T>> {
		// For supporting subSet, headSet, and tailSet backed views.
		// These elements do not have to be in the tree, they are just boundary conditions for the view.
		private IntervalTree<T> backingSet;
		private Interval<T> fromElement;
		private Interval<T> toElement;
		boolean fromInclusive;
		boolean toInclusive;

		private SubIntervalTree(IntervalTree<T> tree, Interval<T> fromElement, Interval<T> toElement) {
			this(tree, fromElement, true, toElement, false);
		}

		private SubIntervalTree(IntervalTree<T> tree, Interval<T> fromElement,
				boolean fromInclusive, Interval<T> toElement, boolean toInclusive) {
			this.backingSet = tree;
			this.fromElement = fromElement;
			this.fromInclusive = fromInclusive;
			this.toElement = toElement;
			this.toInclusive = toInclusive;
		}

		@Override
		public boolean add(Interval<T> element) {
			//IllegalArgumentException for elements outside the range.
			if (outsideRange(element))
				throw new IllegalArgumentException("Element " + element + " is outside the view");
			return backingSet.add(element);
		}
		
		@Override
		public boolean remove(Object arg0) {
			@SuppressWarnings("unchecked")
			Interval<T> element = (Interval<T>) arg0;
			if (outsideRange(element))
				return false;
			return backingSet.remove(arg0);
		}

		private boolean outsideRange(IntervalNode<T> node) {
			return outsideRange(node.object);
		}

		private boolean outsideRange(Interval<T> element) {
			if(fromElement != null) {
				if (backingSet.comparator.compare(element, fromElement) < (fromInclusive ? 1 : 0)) {
					return true;
				}
			}
			if (toElement != null) {
				if (backingSet.comparator.compare(toElement, element) < (toInclusive ? 1 : 0)) {
					return true;
				}
			}
			return false;
		}

		@Override
		public boolean addAll(Collection<? extends Interval<T>> arg0) {
			// Question; do we throw an exception before modifying the set? or while we're
			// inserting? TreeSet adds each one and throws when it encounters.
			boolean modified = false;
			for(Interval<T> ele : arg0) {
				modified = add(ele) ? true : modified;
			}
			return modified;
		}

		@Override
		public void clear() {
			Iterator<Interval<T>> it = iterator();
			while(it.hasNext()) {
				it.next();
				it.remove();
			}
		}

		@Override
		public boolean contains(Object o) {
			@SuppressWarnings("unchecked")
			Interval<T> element = (Interval<T>) o;
			if (outsideRange(element))
				return false;
			return backingSet.contains(o);
		}

		@Override
		public boolean containsAll(Collection<?> arg0) {
			for(Object ele : arg0) {
				if(!contains(ele)) return false;;
			}
			return true;
		}

		@Override
		public boolean isEmpty() {
			return firstNode() != null;
		}

		@Override
		public Iterator<Interval<T>> iterator() {
			return new Iterator<Interval<T>>() {
				// NOTE(jtmcdole): The behavior of sub-set iterator of a TreeSet that has an
				// element added that fills the subset is to return an error instead of returning
				// the updated head.
				int modCountGaurd = backingSet.modCount;
				IntervalNode<T> current = firstNode();
				IntervalNode<T> last = null;

				@Override
				public boolean hasNext() {
					if (modCountGaurd != backingSet.modCount)
						throw new ConcurrentModificationException();
					if (current == null) {
						return false;
					}
					return current != null && !outsideRange(current);
				}

				@Override
				public Interval<T> next() {
					if (modCountGaurd != backingSet.modCount)
						throw new ConcurrentModificationException();
					if (current == null) {
						throw new NoSuchElementException();
					}
					last = current;
					current = backingSet.successor(current);
					if(current != null && outsideRange(current)) {
						current = null;
					}
					if (null == last) {
						throw new NoSuchElementException();
					}
					return last.object;
				}

				@Override
				public void remove() {
					if (modCountGaurd != backingSet.modCount)
						throw new ConcurrentModificationException();
					if (last == null) {
						throw new IllegalStateException();
					}
					backingSet.remove(last);
					modCountGaurd++; // we're allowed to delete nodes and keep going
					last = null;
				}
			};
		}

		@Override
		public boolean removeAll(Collection<?> arg0) {
			boolean changed = false;
			for(Object ele : arg0) {
				changed = remove(ele) ? true : changed;
			}
			return changed;
		}

		@Override
		public boolean retainAll(Collection<?> c) {
			Iterator<Interval<T>> it = iterator();
			boolean changed = false;
			while(it.hasNext()) {
				Interval<T> ele = it.next();
				if(c.contains(ele)) {
					it.remove();
					changed = true;
				}
			}
			return changed;
		}

		// Used for tracking size changes
		int modCount = -1;
		int size = 0;

		@Override
		public int size() {
			if(modCount != backingSet.modCount) {
				size = 0;
				for (@SuppressWarnings("unused") Interval<T> element : this) {
					size++;
				}
			}
			return size;
		}

		@Override
		public Object[] toArray() {
			ArrayList<Interval<T>> list = new ArrayList<Interval<T>>();
			Iterator<Interval<T>> it = iterator();
			while(it.hasNext()) {
				list.add(it.next());
			}
			return list.toArray();
		}

		@SuppressWarnings("unchecked")
		@Override
		public <B> B[] toArray(B[] a) {
			int size = size();
			B[] r = a.length >= size ? a : (B[]) Array.newInstance(a.getClass().getComponentType(),
					size);
			Iterator<Interval<T>> it = iterator();
			for (int i = 0; i < r.length; i++) {
				if (!it.hasNext()) {
					if (a != r)
						return Arrays.copyOf(r, i);
					r[i] = null; // null-terminate
					return r;
				}
				r[i] = (B) it.next();
			}
			// it.hasNext() ? finishToArray(r, it) : - meh.
			return r;
		}

		@Override
		public Comparator<? super Interval<T>> comparator() {
			return backingSet.comparator;
		}

		@Override
		public Interval<T> first() {
			IntervalNode<T> potential = firstNode();
			if(potential != null) return potential.object;
			return null;
		}

		public IntervalNode<T> firstNode() {
			if (fromElement == null)
				return backingSet.minimumNode(backingSet.root);
			IntervalNode<T> potential = backingSet.searchNearest(fromElement,
					SearchNearest.SEARCH_NEAREST_ROUNDED_UP);
			if (!fromInclusive && comparator().compare(potential.object, fromElement) == 0)
				potential = backingSet.successor(potential);
			if (outsideRange(potential))
				return null;
			return potential;
		}

		@Override
		public Interval<T> last() {
			if (toElement == null)
				return backingSet.first();
			IntervalNode<T> potential = backingSet.searchNearest(toElement,
					SearchNearest.SEARCH_NEAREST_ROUNDED_DOWN);
			if (!toInclusive && comparator().compare(potential.object, toElement) == 0)
				potential = backingSet.predecessor(potential);
			if (outsideRange(potential))
				return null;
			return potential.object;
		}

		@Override
		public SortedSet<Interval<T>> headSet(Interval<T> toElement) {
			return new SubIntervalTree<T>(backingSet, null, toElement);
		}

		@Override
		public SortedSet<Interval<T>> subSet(Interval<T> fromElement, Interval<T> toElement) {
			return new SubIntervalTree<T>(backingSet, fromElement, toElement);
		}

		@Override
		public SortedSet<Interval<T>> tailSet(Interval<T> afromElementrg0) {
			return new SubIntervalTree<T>(backingSet, fromElement, null);
		}
	}

	/**
	 * Verify every node of the tree has the correct balance factor between right and left nodes
	 * 
	 * @param node to test
	 * @return height of this node
	 */
	private int verifyHeight(IntervalNode<V> node) {
		if (node == null)
			return 0;
		int left = verifyHeight(node.left);
		int right = verifyHeight(node.right);
		int calcBalanc = right - left;
		if (node.balanceFactor != calcBalanc) {
			throw new IllegalStateException("Balance off; is:" + node.balanceFactor + " should:"
					+ calcBalanc);
		}
		return Math.max(left, right) + 1;
	}

	/**
	 * Verify every node of the tree has the correct height
	 */
	protected void verifyHeight() {
		verifyHeight(root);
	}

	/**
	 * Verify every node of the tree has the correct balance factor between right and left nodes
	 */
	protected void verifyOrder() {
		if(root == null) return;
		IntervalNode<V> first = minimumNode(root);
		V last = first.object.getLower();
		for(Interval<V> node : this) {
			if(node.getLower().compareTo(last) < 0) {
				throw new IllegalStateException("Order is off; last:" + last + " now:"
						+ node.getLower());
			}
		}
	}

	/**
	 * Search the set for any elements in the interval
	 * 
	 * TODO(jtmcdole): should be a set backed by the tree?
	 *
	 * @param interval the interval to be searched
	 * 
	 * @return the list of found intervals
	 */
	public List<Interval<V>> searchInterval(Interval<V> interval) {
		List<Interval<V>> found = new ArrayList<Interval<V>>();
		searchIntervalRecursive(interval, root, found);
		return found;
	}

	/**
	 * Search each node, recursively matching against the search interval
	 * 
	 * @param interval the search interval
	 * @param node the interval node
	 * @param storage the interval storage
	 */
	private void searchIntervalRecursive(Interval<V> interval, IntervalNode<V> node,
			List<Interval<V>> storage) {
		if (node == null)
			return;

		// If the node's max interval is less than the low interval, no children will match
		if (node.max.compareTo(interval.getLower()) < 0)
			return;

		// left children
		searchIntervalRecursive(interval, node.left, storage);

		// Do we overlap? s.low <= n.high && n.low <= s.high; where s = interval, n = node
		if (interval.getLower().compareTo(node.object.getUpper()) < 0
				&& node.object.getLower().compareTo(interval.getUpper()) < 0) {
			storage.add(node.object);
		}

		// if interval.high is to the left of the start of Node's interval, then no children to the
		// right will match (short cut)
		if (interval.getUpper().compareTo(node.object.getLower()) < 0) {
			return;
		}

		// else, search the right nodes as well
		searchIntervalRecursive(interval, node.right, storage);
	}

	/**
	 * Search the tree for the matching element, or the 'nearest' one.
	 * 
	 * @param element the element to be searched
	 * 
	 * @return the found interval
	 */
	public Interval<V> searchNearestElement(Interval<V> element) {
		return searchNearestElement(element, SearchNearest.SEARCH_NEAREST_ABSOLUTE);
	}

	/**
	 * Search the tree for the matching element, or the 'nearest' one.
	 * 
	 * @param element the element to be searched
	 * @param nearestOption the nearest option to the element
	 * 
	 * @return the found interval
	 */
	public Interval<V> searchNearestElement(Interval<V> element, SearchNearest nearestOption) {
		IntervalNode<V> found = searchNearest(element, nearestOption);
		if (found != null)
			return found.object;
		return null;
	}

	/**
	 * Controls the results for searchNearest()
	 */
	public static enum SearchNearest {
		SEARCH_NEAREST_ROUNDED_DOWN, // If result not found, always chose the lower element
		SEARCH_NEAREST_ABSOLUTE,     // If result not found, chose the nearest based on comparison
		SEARCH_NEAREST_ROUNDED_UP    // If result not found, always chose the higher element
	}

	/**
	 * Search the tree for the matching element, or the 'nearest' node.
	 * 
	 * @param element the element to be searched
	 * @param option the nearest option to the element
	 * 
	 * @return the found interval node
	 */
	protected IntervalNode<V> searchNearest(Interval<V> element, SearchNearest option) {
		if (element == null)
			return null;
		IntervalNode<V> x = root;
		if (x == null)
			return null;
		IntervalNode<V> previous = x;
		int compare = 0;
		while (x != null) {
			previous = x;
			compare = comparator.compare(element, x.object);
			if (0 == compare) {
				return x; // comparator.compare should check for further
							// restrictions.
			} else if (compare < 0) {
				x = x.left;
			} else {
				x = x.right;
			}
		}

		if (option == SearchNearest.SEARCH_NEAREST_ROUNDED_UP) {
			return (compare < 0) ? previous : successor(previous);
		} else if (option == SearchNearest.SEARCH_NEAREST_ROUNDED_DOWN) {
			return (compare < 0) ? predecessor(previous) : previous;
		}
		// Default: nearest absolute value
		// Fell off the tree looking for the exact match; now we need
		// to find the nearest element.
		x = (compare < 0) ? predecessor(previous) : successor(previous);
		if (x == null)
			return previous;
		int otherCompare = comparator.compare(element, x.object);
		if (compare < 0) {
			return Math.abs(compare) < otherCompare ? previous : x;
		}
		return Math.abs(otherCompare) < compare ? x : previous;

	}
}
