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
import java.util.stream.Collectors;

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

//    private Map<Node, ArrayList<Node>> nodeSet;
    private Set<Node> nodeSet;
    /**
     * Create a new empty MapGraph
     */
    public MapGraph()
    {
        nodeSet = new HashSet<>();
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
//        Set<Node> nodes = nodeSet.keySet();
        return nodeSet.stream().map(Node::getGeoPoint).collect(Collectors.toSet());
    }

    /**
     * Get the number of road segments in the graph
     * @return The number of edges in the graph.
     */
    public int getNumEdges()
    {
        int numEdges = 0;
        for (Node node : nodeSet){
            numEdges += node.getEdges().size();
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
            for (Node node: nodeSet){
                if (node.getGeoPoint().equals(location)) {
                    return false;
                }
            }
            nodeSet.add(new Node(location));
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

        for (Node node : nodeSet) {
            if (node.getGeoPoint().equals(from)) {
                node.getEdges().add(edge);
                break;
            }
        }
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
        for (Node node : nodeSet) {
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
            List<GeographicPoint> geographicPoints = new ArrayList<>();
            for (Node node : nodeSet){
                if (node.equals(new Node((curr)))) {
                    List<Edge> edges = node.getEdges();
                    for (Edge edge : edges){
                        GeographicPoint geoPoint2 = edge.getGeoPoint2();
                        geographicPoints.add(geoPoint2);
                    }
                }
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
                                          GeographicPoint goal,
                                          Consumer<GeographicPoint> nodeSearched)
    {
        PriorityQueue<Node> nodePriorityQueue = new PriorityQueue<>();
        HashSet<Node> visitedNodes = new HashSet<>();
        HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();

        Node startNode = getNodeByGeoPoint(start);
        Node goalNode = getNodeByGeoPoint(goal);

        startNode.setDistance(0);
        nodePriorityQueue.add(startNode);
        int count = 0;

        while (!nodePriorityQueue.isEmpty()) {
            Node currNode = nodePriorityQueue.remove();
            nodeSearched.accept(currNode.getGeoPoint());
            System.out.print("* ");
            count++;
            if (!visitedNodes.contains(currNode)) {
                visitedNodes.add(currNode);
                if (currNode.equals(goalNode)) {
                    System.out.println(count);
                    return constructPath(start, goal, parentMap);
                }
                for (Edge edge : currNode.getEdges()){
                    GeographicPoint geoPoint2 = edge.getGeoPoint2();
                    Node neighborNode = getNodeByGeoPoint(geoPoint2);
                    if (!visitedNodes.contains(neighborNode)) {
                        double edgeLength = edge.getLength();
                        double distFromStart = currNode.getDistance() + edgeLength;
                        if (!nodePriorityQueue.contains(neighborNode)) {
                            updatePQueue(nodePriorityQueue, parentMap, currNode, neighborNode, distFromStart);
                        } else {
                            if (distFromStart < neighborNode.getDistance()) {
                                updatePQueue(nodePriorityQueue, parentMap, currNode, neighborNode, distFromStart);
                            }
                        }
                    }
                }
            }
        }
        System.out.println("No path exists");
        return new ArrayList<>();
    }

    private void updatePQueue(PriorityQueue<Node> nodePriorityQueue,
                              HashMap<GeographicPoint, GeographicPoint> parentMap,
                              Node currNode,
                              Node neighbor,
                              double distFromStart)
    {
        neighbor.setDistance(distFromStart);
        parentMap.put(neighbor.getGeoPoint(), currNode.getGeoPoint());
        nodePriorityQueue.add(neighbor);
    }

    private Node getNodeByGeoPoint(GeographicPoint geoPoint) {
        for (Node node : nodeSet) {
            if (node.getGeoPoint().equals(geoPoint)) {
                return node;
            }
        }
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
                                             GeographicPoint goal,
                                             Consumer<GeographicPoint> nodeSearched)
    {
        PriorityQueue<Node> nodePriorityQueue = new PriorityQueue<>((o1, o2) -> {
            if (o1.getPredictedDist() - o2.getPredictedDist() < 0) {
                return -1;
            } else if (o1.getPredictedDist() - o2.getPredictedDist() > 0) {
                return 1;
            } else {
                return 0;
            }
        });
        HashSet<Node> visitedNodes = new HashSet<>();
        HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();

        Node startNode = getNodeByGeoPoint(start);
        Node goalNode = getNodeByGeoPoint(goal);

        startNode.setDistance(0);
        startNode.setPredictedDist(goal.distance(start));
        nodePriorityQueue.add(startNode);
        int count = 0;

        while (!nodePriorityQueue.isEmpty()) {
            Node currNode = nodePriorityQueue.remove();
//            System.out.println(currNode.toString());
            nodeSearched.accept(currNode.getGeoPoint());
            System.out.print("* ");
            count++;
            if (!visitedNodes.contains(currNode)) {
                visitedNodes.add(currNode);
                if (currNode.equals(goalNode)) {
                    System.out.println(count);
                    return constructPath(start, goal, parentMap);
                }
                for (Edge edge : currNode.getEdges()){
                    GeographicPoint geoPoint2 = edge.getGeoPoint2();
                    Node neighborNode = getNodeByGeoPoint(geoPoint2);
                    if (!visitedNodes.contains(neighborNode)) {
                        double edgeLength = edge.getLength();
                        double distFromStartToNeighbor = currNode.getDistance() + edgeLength;
                        double neighborPredictedDist = goal.distance(neighborNode.getGeoPoint());
                        if (!nodePriorityQueue.contains(neighborNode)) {
                            updatePQueueAStar(nodePriorityQueue, parentMap, currNode, neighborNode, distFromStartToNeighbor, neighborPredictedDist);
                        } else {
                            if (distFromStartToNeighbor < neighborNode.getDistance()) {
                                updatePQueueAStar(nodePriorityQueue, parentMap, currNode, neighborNode, distFromStartToNeighbor, neighborPredictedDist);
                            }
                        }
                    }
                }
            }
        }
        System.out.println("No path exists");
        return new ArrayList<>();
    }

    private void updatePQueueAStar(PriorityQueue<Node> nodePriorityQueue,
                                   HashMap<GeographicPoint, GeographicPoint> parentMap,
                                   Node currNode,
                                   Node neighborNode,
                                   double distFromStartToNeighbor,
                                   double neighborPredictedDist)
    {
        neighborNode.setDistance(distFromStartToNeighbor);
        neighborNode.setPredictedDist(distFromStartToNeighbor + neighborPredictedDist);
        parentMap.put(neighborNode.getGeoPoint(), currNode.getGeoPoint());
        nodePriorityQueue.add(neighborNode);
    }

    public List<GeographicPoint> greedyAlgorithm(GeographicPoint start)
    {
        List<Node> nodesToVisit = new ArrayList<>(nodeSet);
        Set<Node> visitedNodes = new HashSet<>();
        List<GeographicPoint> steps = new ArrayList<>();

        steps.add(start);
        Node currNode = getNodeByGeoPoint(start);

        nodesToVisit.remove(currNode);
        visitedNodes.add(currNode);

//      while we have nodes to visit
        while (!nodesToVisit.isEmpty()) {
//          get all available egdes
            List<Edge> currNodeEdges = currNode.getEdges();
//          find smallest edge
            Edge minEdge = findMinEdge(currNodeEdges, visitedNodes);
//          if we get null - it means we are in deadend or in node, which neighbors we visited
            if (minEdge == null) {
//              find the closest node from nodesToVisit
                Node closestNode = findClosestFromNotVisited(nodesToVisit, currNode);
//              use A* search algorithm to find path from where we are to closest node
                List<GeographicPoint> pathFromAStar = aStarSearch(currNode.getGeoPoint(), closestNode.getGeoPoint());
                addPathToSteps(nodesToVisit, visitedNodes, steps, pathFromAStar);
                currNode = getNodeByGeoPoint(closestNode.getGeoPoint());
            } else {
                steps.add(minEdge.getGeoPoint2());
    //          get the other end node of the smallest edge
                currNode = getNodeByGeoPoint(minEdge.getGeoPoint2());
                nodesToVisit.remove(currNode);
                visitedNodes.add(currNode);
            }
        }
//      after we visited all nodes, again use A* search to find the best path to start node
        List<GeographicPoint> aStarPath = aStarSearch(currNode.getGeoPoint(), start);
        for (int i = 1; i < aStarPath.size(); i++) {
            steps.add(aStarPath.get(i));
        }
        System.out.println("Number of nodes: " + nodeSet.size());
        System.out.println("Total number of visited nodes: " + steps.size());
        return steps;
    }

    /**
     * Helper method to add path found by A* algorithm to steps we take from start
     * @param nodesToVisit
     * @param visitedNodes
     * @param steps
     * @param pathFromAStar
     */
    private void addPathToSteps(List<Node> nodesToVisit, Set<Node> visitedNodes, List<GeographicPoint> steps, List<GeographicPoint> pathFromAStar) {
        for (int i = 1; i < pathFromAStar.size(); i++) {
            GeographicPoint geoPoint = pathFromAStar.get(i);
            steps.add(geoPoint);
            Node nodeByGeoPoint = getNodeByGeoPoint(geoPoint);
            if (nodesToVisit.remove(nodeByGeoPoint)) {
                visitedNodes.add(nodeByGeoPoint);
            }
        }
    }

    /**
     * Helper method to find the closest node among those that we have not visited yet
     * @param nodesToVisit
     * @param currNode
     * @return
     */
    private Node findClosestFromNotVisited(List<Node> nodesToVisit, Node currNode) {
        double minEstimatedDist = Double.MAX_VALUE;
        Node closestNode = null;
        for (Node node : nodesToVisit) {
            double pointsDistance = node.getGeoPoint().distance(currNode.getGeoPoint());
            if (minEstimatedDist > pointsDistance) {
                minEstimatedDist = pointsDistance;
                closestNode = node;
            }
        }
        return closestNode;
    }

    /**
     * Helper method to find smallest egde
     * @param currNodeEdges
     * @param visitedNodes
     * @return
     */
    private Edge findMinEdge(List<Edge> currNodeEdges, Set<Node> visitedNodes) {
        double minEdgeLength = Double.MAX_VALUE;
        Edge edgeWithMinLength = null;
        for (Edge edge : currNodeEdges) {
            double edgeLength = edge.getLength();
            Node node = getNodeByGeoPoint(edge.getGeoPoint2());
            if (minEdgeLength > edgeLength
                    && !visitedNodes.contains(node)) {
                minEdgeLength = edgeLength;
                edgeWithMinLength = edge;
            }
        }
        return edgeWithMinLength;
    }

    public static void main(String[] args)
    {
        MapGraph simpleTestMap = new MapGraph();
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
        GeographicPoint testStart1 = new GeographicPoint(1.0, 1.0);
        System.out.println(simpleTestMap.greedyAlgorithm(testStart1));

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

//		Dijkstra TEST 1

//        MapGraph simpleTestMap = new MapGraph();
//        GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
//
//        GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
//        GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
//
//        System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
//        List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
//        System.out.println(testroute);
//        List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
//        System.out.println(testroute2);

//		Dijkstra TEST 2

//		MapGraph testMap = new MapGraph();
//		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
//
//		// A very simple test using real data
//        GeographicPoint testStart = new GeographicPoint(32.869423, -117.220917);
//        GeographicPoint testEnd = new GeographicPoint(32.869255, -117.216927);
//		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
//        List<GeographicPoint> testroute = testMap.dijkstra(testStart,testEnd);
//        System.out.println(testroute);
//        List<GeographicPoint> testroute2 = testMap.aStarSearch(testStart,testEnd);
//        System.out.println(testroute2);

        //		Dijkstra TEST 3

		// A slightly more complex test using real data
//   		MapGraph testMap = new MapGraph();
//		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
//        GeographicPoint testStart = new GeographicPoint(32.8674388, -117.2190213);
//        GeographicPoint testEnd = new GeographicPoint(32.8697828, -117.2244506);
//		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
//        List<GeographicPoint> testroute = testMap.dijkstra(testStart,testEnd);
//        System.out.println(testroute);
//        List<GeographicPoint> testroute2 = testMap.aStarSearch(testStart,testEnd);
//        System.out.println(testroute2);

		
		
		/* Use this code in Week 3 End of Week Quiz */
//		MapGraph theMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
//		System.out.println("DONE.");
//
//		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
//		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
//
//
//		List<GeographicPoint> route = theMap.dijkstra(start,end);
//        System.out.println(route);
//        List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
//        System.out.println(route2);

//        MapGraph theMap = new MapGraph();
//        System.out.print("DONE. \nLoading the map...");
//        GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
//        System.out.println("DONE.");
//
//        GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
//        GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
//
//        List<GeographicPoint> route = theMap.dijkstra(start,end);
//        System.out.println(route);
//        List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
//        System.out.println(route2);
    }
}
