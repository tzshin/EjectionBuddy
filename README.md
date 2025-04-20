IMPORTANT: There is a FATAL wiring flaw in the IMU CPOUT capacitor in HW1.0 board!
The other side of that capacitor should connect to ground, NOT 3.3V. Fix it.

You can use some continuity tests
1. Find the falsely connected pad. The pad is shorted to 3.3V.
2. Verify your cutting work. After the cut, the pad should be floating (not shorted to 3.3V or ground).
3. Verify your connecting work. With a decent solder bridge (apply this after hot plate assembly), the pad should be shorted only to ground. 
