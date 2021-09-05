###Calibration procedure
1. Measure average difference between electrical angle and encoder angle using calibration.ino.
2. Subtract the difference from angle returned by encoder.
3. Measure and record 720 degrees rotation errors. Preserve that for future processing.
4. Measure and record electric zero offset and direction.
5. Use offset and direction in initFOC function.
