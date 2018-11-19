package roadgraph;

import geography.GeographicPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class Node implements Comparable{
    private GeographicPoint geoPoint;
    private List<Edge> egdes;
    private double distance;
    private double predictedDist;

    public Node(GeographicPoint geoPoint) {
        this.geoPoint = geoPoint;
        egdes = new ArrayList<>();
        distance = Double.MAX_VALUE;
        predictedDist = Double.MAX_VALUE;
    }

    public double getPredictedDist() {
        return predictedDist;
    }

    public void setPredictedDist(double predictedDist) {
        this.predictedDist = predictedDist;
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

    public void setDistance(double distance) {
        this.distance = distance;
    }


    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Node node = (Node) o;
        return Double.compare(node.distance, distance) == 0 &&
                Objects.equals(geoPoint, node.geoPoint);
    }

    @Override
    public int hashCode() {
        return Objects.hash(geoPoint, distance);
    }

    @Override
    public int compareTo(Object o) {
        Node other = (Node) o;
        if (this.distance - other.distance < 0) {
            return -1;
        } else if (this.distance - other.distance > 0) {
            return 1;
        } else {
            return 0;
        }
    }

    @Override
    public String toString() {
        return "Node{" +
                "geoPoint=" + geoPoint +
                ", egdes=" + egdes +
                ", distance=" + distance +
                ", predictedDist=" + predictedDist +
                '}';
    }
}
