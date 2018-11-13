/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.*;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {

    private Map<Node, ArrayList<Node>> adjList;
	/**
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
        adjList = new HashMap<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return getVertices().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
        Set<Node> nodes = adjList.keySet();
        Set<GeographicPoint> geographicPoints = new HashSet<>();
        for (Node node : nodes){
            geographicPoints.add(node.getGeoPoint());
        }
        return geographicPoints;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
        int numEdges = 0;
        for (List<Node> nodes : adjList.values()){
            numEdges += nodes.size();
        }
		return numEdges;
	}

	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
    {
        if (location != null) {
            for (Node node:adjList.keySet()){
                if (node.getGeoPoint().equals(location)) {
                    return false;
                }
            }
            adjList.put(new Node(location), new ArrayList<>());
            return true;
        }
        return false;
    }
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from,
                        GeographicPoint to,
                        String roadName,
			            String roadType,
                        double length) throws IllegalArgumentException
    {
        preconditionChecks(from, to, roadName, roadType, length);
        Edge edge = new Edge(from, to, roadName, roadType, length);
        Node nodeFrom = new Node(from);
        nodeFrom.getEdges().add(edge);
        Node nodeTo = new Node(to);
        nodeTo.getEdges().add(edge);
        adjList.get(nodeFrom).add(nodeTo);
	}

    /**
     * Helper method for addEdge(...)
     * @param from The starting point of the edge
     * @param to The ending point of the edge
     * @param roadName The name of the road
     * @param roadType The type of the road
     * @param length The length of the road, in km
     * @throws IllegalArgumentException If the points have not already been
     *   added as nodes to the graph, if any of the arguments is null,
     *   or if the length is less than 0.     *
     */
    private void preconditionChecks(GeographicPoint from,
                                    GeographicPoint to,
                                    String roadName,
                                    String roadType,
                                    double length)
    {
        boolean fromOk = false;
        boolean toOk = false;
        for (Node node : adjList.keySet()) {
            if (node.getGeoPoint().equals(from)) {
                fromOk = true;
            }
            if (node.getGeoPoint().equals(to)) {
                toOk = true;
            }
        }
        if (!fromOk) {
            throw new IllegalArgumentException("from is not added to the graph");
        }
        if (!toOk) {
            throw new IllegalArgumentException("to is not added to the graph");
        }

        if (roadName == null || roadType == null) {
            throw new IllegalArgumentException("roadName and roadType must be added");
        }  else if (length < 0) {
            throw new IllegalArgumentException("road length has to be positive");
        }
    }

    /** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal,
                                     Consumer<GeographicPoint> nodeSearched)
	{
	    // Initializing data structures
        HashSet<GeographicPoint> visited = new HashSet<>();
        Queue<GeographicPoint> toExplore = new LinkedList<>();
        HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
        toExplore.add(start);

        // if bfsSearch(...) returns true, then it found the path, and we use constructPath(...) helper method
        // to record it, otherwise we return empty list
        if (bfsSearch(goal, visited, toExplore, parentMap, nodeSearched)) {
            return constructPath(start, goal, parentMap);
        } else {
            System.out.println("No path exists");
            return new ArrayList<>();
        }
	}

    /**
     * Helper method for bfs(...) that constructs and returns path from start to goal
     * @param start The starting location
     * @param goal The goal location
     * @param parentMap Map to keep track of connections between locations
     * @return list of GeographicPoints ordered from start to goal locations
     */
    private List<GeographicPoint> constructPath(GeographicPoint start,
                                                GeographicPoint goal,
                                                HashMap<GeographicPoint, GeographicPoint> parentMap)
    {
        LinkedList<GeographicPoint> path = new LinkedList<>();
        GeographicPoint curr = goal;
        while (!curr.equals(start)) {
            path.addFirst(curr);
            curr = parentMap.get(curr);
        }
        path.addFirst(start);
        return path;
    }

    /**
     * Helper method for bfs(...) that contains searching logic
     * @param goal The goal location
     * @param visited Set to keep track of visited locations
     * @param toExplore Queue to process locations one by one
     * @param parentMap Map to keep track of connections between locations
     * @param nodeSearched Consumer used for adding visualization feature to MapApp tool
     * @return true if path is found, false otherwise
     */
    private boolean bfsSearch(GeographicPoint goal,
                              HashSet<GeographicPoint> visited,
                              Queue<GeographicPoint> toExplore,
                              HashMap<GeographicPoint, GeographicPoint> parentMap,
                              Consumer<GeographicPoint> nodeSearched)
    {
		while (!toExplore.isEmpty()) {
            GeographicPoint curr = toExplore.remove();
            if (curr.equals(goal)) {
				return true;
            }
            List<Node> neighborNodes = adjList.get(new Node(curr));
            List<GeographicPoint> geographicPoints = new ArrayList<>();
            for (Node node : neighborNodes) {
                geographicPoints.add(node.getGeoPoint());
            }
            ListIterator<GeographicPoint> it = geographicPoints.listIterator(geographicPoints.size());
            while (it.hasPrevious()) {
                GeographicPoint next = it.previous();
                nodeSearched.accept(next);
                if (!visited.contains(next)) {
                    visited.add(next);
                    parentMap.put(next, curr);
                    toExplore.add(next);
                }
            }
        }
		return false;
	}


	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

        PriorityQueue<Node> toExplore = new PriorityQueue<>();
        HashSet<Node> visited = new HashSet<>();
        HashMap<Node, Node> parentMap = new HashMap<>();

        toExplore.add(new Node(start));



        return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
//		System.out.print("Making a new map...");
//		MapGraph firstMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
//        System.out.println("DONE.");
//
//        GeographicPoint testStart1 = new GeographicPoint(1.0, 1.0);
//        GeographicPoint testEnd1 = new GeographicPoint(8.0, -1.0);
//        System.out.println("BFS path : " + firstMap.bfs(testStart1, testEnd1));


		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */

		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		System.out.println(testroute);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		System.out.println(testroute2);
		
		/*
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
