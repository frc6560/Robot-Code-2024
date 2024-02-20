package com.team6560.frc2024.utility;

import java.util.ArrayList;
import java.util.List;

public class ShooterConfigMap {
    
    public static class Point {

        private double distance;
        private double rpm;
        private double angle;

        public Point(double distance, double rpm, double angle) {
            this.distance = distance;
            this.rpm = rpm;
            this.angle = angle;
        }

        public double getDistance() {
            return distance;
        }
        public double getRPM() {
            return rpm;
        }
        public double getAngle() {
            return angle;
        }

        @Override
        public String toString() {
            return "Distance: " + distance + ". RPM: " + rpm + ". Angle: " + angle + ".";        
        }
    }

    private List<Point> shooterConfigs = new ArrayList<Point>();

    public double[] getRPMandAngle(double distance) {
        int highDist = 0, lowDist = 0;

        for (int i = 0; i < shooterConfigs.size(); i++) 
            if (shooterConfigs.get(i).getDistance() <= distance) lowDist = i;
            else break;
        highDist = lowDist + 1;

        if (highDist == 0 || lowDist == shooterConfigs.size()) return null;

        double highLowDistDiff = shooterConfigs.get(highDist).getDistance() - shooterConfigs.get(lowDist).getDistance();
        double lowActualDistDiff = distance - shooterConfigs.get(lowDist).getDistance();

        double rpmDiff = shooterConfigs.get(highDist).getRPM() - shooterConfigs.get(lowDist).getRPM();
        
        double angleDiff = shooterConfigs.get(highDist).getAngle() - shooterConfigs.get(lowDist).getAngle();


        double resRPM = (rpmDiff * lowActualDistDiff)/highLowDistDiff + shooterConfigs.get(lowDist).getRPM();
        double resAngle = (angleDiff * lowActualDistDiff)/highLowDistDiff + shooterConfigs.get(lowDist).getRPM();

        return new double[] {resRPM, resAngle};
    }

    public void add(Point... points) {
        for (Point i : points) {
            shooterConfigs.add(i);
        }
    }

    @Override
    public String toString() {
        return String.join(";", shooterConfigs.stream().map(Point::toString).toArray(String[]::new));
    }
}
