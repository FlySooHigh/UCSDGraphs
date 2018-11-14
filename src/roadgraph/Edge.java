package roadgraph;

import geography.GeographicPoint;

public class Edge {
    private GeographicPoint geoPoint1;
    private GeographicPoint geoPoint2;
    private String roadName;
    private String roadType;
    private double length;

    public Edge(GeographicPoint geoPoint1, GeographicPoint geoPoint2, String roadName, String roadType, double length) {
        this.geoPoint1 = geoPoint1;
        this.geoPoint2 = geoPoint2;
        this.roadName = roadName;
        this.roadType = roadType;
        this.length = length;
    }

    public GeographicPoint getGeoPoint1() {
        return geoPoint1;
    }

    public GeographicPoint getGeoPoint2() {
        return geoPoint2;
    }

    public String getRoadName() {
        return roadName;
    }

    public String getRoadType() {
        return roadType;
    }

    public double getLength() {
        return length;
    }

    @Override
    public String toString() {
        return "Edge{" +
                "geoPoint1=" + geoPoint1 +
                ", geoPoint2=" + geoPoint2 +
                ", roadName='" + roadName + '\'' +
                ", roadType='" + roadType + '\'' +
                ", length=" + length +
                '}';
    }
}
