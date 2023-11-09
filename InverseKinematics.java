import java.lang.Math;

public class IK
{
    double len1 = 10.6;
    double len2 = 5.16;
    double len3 = 1;

    double phi = 0;

    double[] run(double targetX, double targetY)
    {
        double[] angles = new double[3];

        double wx = targetX - len3 * Math.cos(phi);
        double wy = targetY - len3 * Math.sin(phi);  
    
        double delta = wx * wx + wy * wy;
        double phi1 = Math.acos((len1 * len1 + delta - len2 * len2) / (2 * len1 * Math.sqrt(delta)));
        double phi2 = Math.atan2(wy, wx);
        angles[0] = phi1 + phi2;
        
        double phi3 = Math.acos((len1 * len1 + len2 * len2 - delta) / (2 * len1 * len2));
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

    public static void main(String[] args) {
        IK ik = new IK();
        double[] angles = ik.run(8.2024, 1.628225);
		System.out.println(angles[0] + ", " + angles[1] + ", " + angles[2]);
	}
}
