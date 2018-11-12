package roadgraph;

import geography.GeographicPoint;

import java.util.ArrayList;
import java.util.List;

public class Node {
    private GeographicPoint geoPoint;
    private List<Edge> egdes;

    public Node(GeographicPoint geoPoint) {
        this.geoPoint = geoPoint;
        egdes = new ArrayList<>();
    }

    public GeographicPoint getGeoPoint() {
        return geoPoint;
    }

    public List<Edge> getArcs() {
        return egdes;
    }
}
