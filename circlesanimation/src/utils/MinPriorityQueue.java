package utils;

import java.util.ArrayList;

public class MinPriorityQueue<T extends Comparable<T>> {
    
    private ArrayList<T>     queue;

    // Constructor: creates an ArrayList starting with null
    public MinPriorityQueue() {
        
        queue = new ArrayList<T>();
        queue.add(null); // the first element is always null
    }

    // Returns the size of the queue (0-indexed)
    public int size() {
        
        return queue.size()-1;
    }

    // Adds an element to the queue and sorts it
    // @param elem the element to be added
    public void add(T elem) {

        queue.add(elem);

        // If only element, no need to sort
        if (size() == 1) {

            return;
        }

        // Get parent
        int elem_pos = size();
        int parent_pos = elem_pos / 2;
        T parent = queue.get(parent_pos);

        // Repeat loop until parent is no longer greater than elem
        // or elem_pos becomes root
        while (elem_pos > 1 && parent.compareTo(elem) > 0) {

            // Swap parent and elem
            T temp = parent;
            this.queue.set(parent_pos, elem);
            this.queue.set(elem_pos, temp);
            
            // Proceed to next parent
            elem_pos = parent_pos;
            parent_pos = elem_pos/2;
            parent = queue.get(parent_pos);

        }
    }

    // Removes the root and rebalances the binary heap
    public T remove() {
        
        // Get last element
        int elem_pos = size();
        T elem = queue.get(elem_pos);

        // If queue is empty, last element is root
        if (isEmpty()) {

            queue.remove(elem_pos);
            return elem;
        }

        // Get root
        int root_pos = 1;
        T root = queue.get(root_pos);

        // If queue has only root
        if (size() == 1) {

            queue.remove(root);
            return root;
        }

        // Move elem to root
        queue.set(root_pos, elem);
        queue.remove(elem_pos);

        // Find smallest child, if none -1
        elem_pos = root_pos;
        int min_child_pos = find_min_child(elem_pos);

        if (min_child_pos > 0) {
            
            // Get smallest child
            T min_child = queue.get(min_child_pos);

            // Repeat loop until elem is no longer greater than child
            while (elem.compareTo(min_child) > 0) {
                
                // Swap elem with smallest child
                T temp = elem;
                queue.set(min_child_pos, temp);
                queue.set(elem_pos, min_child);

                // Proceed to next children
                elem_pos = min_child_pos;
                min_child_pos = find_min_child(elem_pos);

                if (min_child_pos > 0) {

                    min_child = queue.get(min_child_pos);
                }
                // If no children, end loop
                else {

                    break;
                }
            }
        }
        
        return root;
    }

    // Returns index of smallest child of an element or -1 if no children
    // @param parent_pos : the index of the parent element
    private int find_min_child(int parent_pos) {

        int left_child_pos = parent_pos * 2;
        int right_child_pos = left_child_pos + 1;

        // If there are children, find the smallest one
        if (left_child_pos <= size()) {

            T left_child = queue.get(left_child_pos);
            if (right_child_pos <= size()) {

                if (left_child.compareTo(queue.get(right_child_pos)) >= 0) {
                    
                    return right_child_pos;
                }
            }

            return left_child_pos;
        }
        // If no children, return -1
        else {

            return -1;
        }
    }

    // Returns true if queue is empty
    // false otherwise
    public boolean isEmpty() {
        
        return size() == 0;

    }
}
