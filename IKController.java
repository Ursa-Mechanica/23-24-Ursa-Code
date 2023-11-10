package org.firstinspires.teamcode;

import java.lang.Math;

public class IKController {
    
    double[] segLens = new double[3];
    
    double phi = 0;
    
    double xOffset = 6.266;
    double yOffset = 3.28;
    
    public IKController(double len1, double len2, double len3) 
    {
        segLens[0] = len1;
        segLens[1] = len2;
        segLens[2] = len3;
    }
    
    double[] calculate(double targetX, double targetY)
    {
        double[] angles = new double[3];
        
        targetX += xOffset;
        targetY += yOffset;

        double wx = targetX - segLens[2] * Math.cos(phi);
        double wy = targetY - segLens[2] * Math.sin(phi);  
    
        double fish = wx * wx + wy * wy;
        double phi1 = Math.acos((segLens[0] * segLens[0] + fish - segLens[1] * segLens[1]) / (2 * segLens[0] * Math.sqrt(fish)));
        double phi2 = Math.atan2(wy, wx);
        angles[0] = phi1 + phi2;
        
        double phi3 = Math.acos((segLens[0] * segLens[0] + segLens[1] * segLens[1] - fish) / (2 * segLens[0] * segLens[1]));
        angles[1] = Math.PI + phi3;
        
        angles[2] = phi - angles[0] - angles[1];

        return remapValues(angles);
    }
    
    double[] remapValues(double[] angles)
    {
        // ostart + (ostop - ostart) * ((value - istart) / (istop - istart))
        angles[0] = 1 - (angles[0] / (Math.PI / 2));
        angles[0] = Math.round(angles[0] * 100.0) / 100.0;
        angles[1] = (angles[1] - Math.toRadians(225)) / (Math.toRadians(325) - Math.toRadians(225));
        angles[1] = Math.round(angles[1] * 100.0) / 100.0;

        return angles;
    }
}
