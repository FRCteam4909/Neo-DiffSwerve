// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import edu.wpi.first.wpilibj.drive.Vector2d;

// /** This is a 2D vector struct that supports basic vector operations. */
// public class BBVector2d{

//   @SuppressWarnings("MemberName")
//   public double x;

//   @SuppressWarnings("MemberName")
//   public double y;

//   public BBVector2d() {}

//   public BBVector2d(double x, double y) {
//     this.x = x;
//     this.y = y;
//   }

  
//   public static BBVector2d FromMagDir(double mag, double angleRad) {

//     // normalize
//     if (angleRad > 2 * Math.PI) {
//       angleRad = angleRad % (2 * Math.PI);
//     }

//     double magY = mag;
//     double angleY = angleRad;
//     if (angleRad < -Math.PI/2 || angleRad > Math.PI/2) {
//       magY = magY * -1;
//       // add PI/2 then normalize?

//     }

//     double magx = mag;
//     double y = mag * Math.asin(angleRad); // -pi/2 and pi/2
//     double x = mag * Math.acos(angleRad); // 0 -> pi
//     return new BBVector2d(x,y);
//   }

//   public BBVector2d add(BBVector2d vec) {
//     return new BBVector2d(x + vec.x, y + vec.y);
//   }

//   /**
//    * Rotate a vector in Cartesian space.
//    *
//    * @param angle angle in degrees by which to rotate vector counter-clockwise.
//    */
//   public void rotate(double angle) {
//     double cosA = Math.cos(angle * (Math.PI / 180.0));
//     double sinA = Math.sin(angle * (Math.PI / 180.0));
//     double[] out = new double[2];
//     out[0] = x * cosA - y * sinA;
//     out[1] = x * sinA + y * cosA;
//     x = out[0];
//     y = out[1];
//   }

//   /**
//    * Returns dot product of this vector with argument.
//    *
//    * @param vec Vector with which to perform dot product.
//    */
//   public double dot(Vector2d vec) {
//     return x * vec.x + y * vec.y;
//   }

//   /** Returns magnitude of vector. */
//   public double magnitude() {
//     return Math.sqrt(x * x + y * y);
//   }

//   /**
//    * Returns scalar projection of this vector onto argument.
//    *
//    * @param vec Vector onto which to project this vector.
//    */
//   public double scalarProject(Vector2d vec) {
//     return dot(vec) / vec.magnitude();
//   }
// }
