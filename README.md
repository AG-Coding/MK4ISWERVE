# This is the full MK4I Swerve Code with L2 Gear Ratios

Parts Needed:
- 4 Modules MK4I
- Pigeon 2.0
- 4 Can Coders (Absolute magnet Encoders for steering)
- 8 neo Motors
- Spark Maxes

Step by Step Swerve Tuning Instructions:
-find azimuth angle offsets based of the abosulute angle
-tune azimuth position control(ff + PD) for quick, accurate alignment without oscillation (ff is helpful when using profiled control)
-check wheel odometry is accurate
-tune drive wheel velocity control(ff + P)
-tune path controller P(D?) values

Credit to Sushi Squad's (7461) Swerve Library and Jack in the Bot (2910) Swerve Library.
