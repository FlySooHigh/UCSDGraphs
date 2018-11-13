package roadgraph;

import geography.GeographicPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class Node {
    private GeographicPoint geoPoint;
    private List<Edge> egdes;
    private double distance;

    public Node(GeographicPoint geoPoint) {
        this.geoPoint = geoPoint;
        egdes = new ArrayList<>();
        distance = Double.MAX_VALUE;
    }

    public GeographicPoint getGeoPoint() {
        return geoPoint;
    }

    public List<Edge> getEdges() {
        return egdes;
    }

    public double getDistance() {
        return distance;
    }

    public void setGeoPoint(GeographicPoint geoPoint) {
        this.geoPoint = geoPoint;
    }

    public void setEgdes(List<Edge> egdes) {
        this.egdes = egdes;
    }

    public void setDistance(int distance) {
        this.distance = distance;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Node node = (Node) o;
        return Objects.equals(geoPoint, node.geoPoint);
    }

    @Override
    public int hashCode() {
        return Objects.hash(geoPoint);
    }
}
