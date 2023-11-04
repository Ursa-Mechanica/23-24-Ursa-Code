// Target point for end effector
float targetX = 3.3;
float targetY = 4;

// Lengths of each arm segment
float len1 = 5.3 * 2;
float len2 = 2.58 * 2;
float len3 = 1;

// Angle relative to +X for end effector
float phi = radians(0);

// Angle for each segment
float theta1, theta2, theta3 = 0;

// Point at joint between segments 2 and 3
float wx = targetX - len3 * cos(phi);
float wy = targetY - len3 * sin(phi);  

// Law of Cosines
float delta = sq(wx) + sq(wy);
float phi1 = acos((sq(len1) + delta - sq(len2)) / (2 * len1 * sqrt(delta)));
float phi2 = atan2(wy, wx);
theta1 = phi1 + phi2;

float phi3 = acos((sq(len1) + sq(len2) - delta) / (2 * len1 * len2));
theta2 = radians(180) + phi3;

theta3 = phi - theta1 - theta2;
