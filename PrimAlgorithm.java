//Writen by Ankita Patra
//B-number : B01101280 
//Implementation of Prims (MST) algo using Dijkstra algo as the basis.

import java.util.*;

class MinHeap {
    private List<Node> heap;  // Min-Heap to store nodes based on key value
    private Map<Integer, Integer> indexMap;  // Maps vertex to its index in the heap

    public MinHeap() {
        heap = new ArrayList<>();
        indexMap = new HashMap<>();
    }

    // Inserts and updates the vertex with the given key (priority)
    public void insert(int vertex, int key) {
        if (indexMap.containsKey(vertex)) {
            decreaseKey(vertex, key);  // If vertex exists, decreases its key
        } else {
            indexMap.put(vertex, heap.size());  // here Inserts new vertex
            heap.add(new Node(key, vertex));
            siftUp(heap.size() - 1);  // Ensures heap property
        }
    }

    // Decreases the key of an existing vertex
    public void decreaseKey(int vertex, int newKey) {
        int idx = indexMap.get(vertex);
        if (idx >= 0) {
            heap.get(idx).key = newKey;
            siftUp(idx);  // Fixes heap upwards
            siftDown(idx);  // Fixes heap downwards
        }
    }

    // Extracts the vertex with the minimum key
    public int[] extractMin() {
        Node minNode = heap.get(0);  // Gets the minimum node (root)
        int vertex = minNode.vertex;
        int key = minNode.key;
        int lastIdx = heap.size() - 1;
        heap.set(0, heap.get(lastIdx));  // Moves the last node to the root
        heap.remove(lastIdx);  // Removes the last node
        indexMap.remove(vertex);  // Removes from index map

        if (!heap.isEmpty()) {
            indexMap.put(heap.get(0).vertex, 0);  // Updates root's index
            siftDown(0);  // Restores heap property
        }

        return new int[]{vertex, key};  // Returns the vertex and its key
    }

    //checks if the heap is empty or not 
    public boolean isEmpty() { 
        return heap.isEmpty(); // Returns true if the heap has no elements , And print false otherwise 
    }

    private void siftUp(int idx) { //Moves a node upward the heap to maintain the main heap property
        while (idx > 0) { // countinues untill the current node is the root 
            int parentIdx = (idx - 1) / 2; // here its calculate the index of the parent node 
            if (heap.get(idx).key < heap.get(parentIdx).key) { // find minimum key value between current and parent and swap them 
                swap(idx, parentIdx);  // Swaps with parent if needed
                idx = parentIdx; // update index to continue checking upwards 
            } else {
                break;   // if property is satisfy then stop 
            }
        }
    }

    private void siftDown(int idx) { // moves a node down the heap to maintain the min-heap property 
        int size = heap.size(); // gets the current size  of the heap
        while (true) {
            int leftChildIdx = 2 * idx + 1; // index : left child
            int rightChildIdx = 2 * idx + 2; // index : right child 
            int smallestIdx = idx; // imagin orassume the current node is the smallest node 

            if (leftChildIdx < size && heap.get(leftChildIdx).key < heap.get(smallestIdx).key) { // checks left child exist and smaller then the current smallest one 
                smallestIdx = leftChildIdx;   // updates idx if the right child is smaller 
            }
            if (rightChildIdx < size && heap.get(rightChildIdx).key < heap.get(smallestIdx).key) { // same as above but insteade of left we took right child 
                smallestIdx = rightChildIdx; // updates smallestIdx if right child is smaller
            }

            if (smallestIdx != idx) { // if smallest node is the current node then swap them 
                swap(idx, smallestIdx);  // Swaps if necessary
                idx = smallestIdx; // here it updates index and difted to downwards 
            } else {
                break; //exit loop 
            }
        }
    }

    private void swap(int i, int j) { // Swaps two nodes that is I took here "i" and "j"  in the heap and updates their indices in the index map
        Node temp = heap.get(i); //temporary stor the  node at index i 
        heap.set(i, heap.get(j)); // replace the node i with index j 
        heap.set(j, temp); // set the node at j to the temporary stored node 
        indexMap.put(heap.get(i).vertex, i); //here update the indext map to reflect the new positions of the swapped nodes 
        indexMap.put(heap.get(j).vertex, j);
    }

    /**
    //here I wrote the code for node class to stores key and vertex 
    // i took key : comparison in the heap
    // vertex: identifier for the vertex  associated with this node and assign it. 
    */
    private static class Node {
        int key; 
        int vertex;

        Node(int key, int vertex) {
            this.key = key;
            this.vertex = vertex;
        }
    }
}

// Edge class to represents graph edges
class Edge {
    int vertex;
    int weight;

    Edge(int vertex, int weight) {
        this.vertex = vertex;
        this.weight = weight;
    }
}

// Prim's algorithm to finds the Minimum Spanning Tree (MST)
public class PrimAlgorithm {
    public static int primMST(Map<Integer, List<Edge>> graph, int numVertices) {
        int totalCost = 0;
        boolean[] inMST = new boolean[numVertices];  // Tracks if a vertex is included in MST
        int[] key = new int[numVertices];  // Min weight to reach each vertex
        Arrays.fill(key, Integer.MAX_VALUE);  // Set all keys to "infinity"

        MinHeap minHeap = new MinHeap();  // Min-Heap to extract min weight edge
        key[0] = 0;  // Start from vertex 0
        minHeap.insert(0, 0);

        while (!minHeap.isEmpty()) {
            int[] minNode = minHeap.extractMin();
            int u = minNode[0];

            if (inMST[u]) continue;  // If vertex is already included, skip it
            inMST[u] = true;  // Mark the vertex as included in the MST
            totalCost += minNode[1];  // Add the edge weight to total cost

            // Explore all adjacent vertices
            for (Edge edge : graph.getOrDefault(u, new ArrayList<>())) {
                int v = edge.vertex;
                int weight = edge.weight;

                // If the adjacent vertex is not in MST and the edge weight is less than current key
                if (!inMST[v] && weight < key[v]) {
                    key[v] = weight;
                    minHeap.insert(v, key[v]);  // Insert/update with the new key
                }
            }
        }

        // Check if all vertices were included in the MST
        for (boolean included : inMST) {
            if (!included) {
                return -1;  // Return -1 if the graph is disconnected
            }
        }

        return totalCost;  // Return the total cost of the MST
    }

    public static void main(String[] args) {
        Map<Integer, List<Edge>> graph = new HashMap<>();
        Scanner scanner = new Scanner(System.in);

        // Read input and build the graph
        while (scanner.hasNextLine()) {
            String line = scanner.nextLine().trim();
            if (line.isEmpty()) continue;  // Skip empty lines

            String[] parts = line.split("\\s+");
            if (parts.length != 3) {
                continue;  // Skip invalid input
            }

            try {
                int u = Integer.parseInt(parts[0]);
                int v = Integer.parseInt(parts[1]);
                int length = Integer.parseInt(parts[2]);

                graph.putIfAbsent(u, new ArrayList<>());
                graph.putIfAbsent(v, new ArrayList<>());

                graph.get(u).add(new Edge(v, length));  // Add edge u -> v
                graph.get(v).add(new Edge(u, length));  // Add edge v -> u (since undirected)
            } catch (NumberFormatException e) {
                // Skip lines with invalid integers
            }
        }

        if (graph.isEmpty()) {
            System.out.println(0);  // Print 0 if no edges were provided
            return;
        }

        int numVertices = graph.size();  // Determine the number of vertices
        int mstCost = primMST(graph, numVertices);  // Run Prim's algorithm to find MST cost

        // Print only the total cost
        if (mstCost != -1) {
            System.out.println(mstCost);  // Print total MST cost
        } else {
            System.out.println(0);  // Print 0 if the graph is disconnected
        }

        scanner.close();
    }
}