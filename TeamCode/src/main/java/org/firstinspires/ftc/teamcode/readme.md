> # Welcome to the 10130 Code Base!
Feel free to ask `dennis.colak@pinecrest.edu` or `jake.taubman@pinecrest.edu` for any questions or concerns. This is the code base for the 2024-2025 season. We are using the `FTC SDK` and `Java` for this season.

---

# Code Structure
<pre>
📦 <b>TeamCode</b>
 ├─ 📂 <b>autonomous</b> <i>(contains all match autonomous programs. Use x+y+z for scoring, where x is specimens, y for samples, z for parking)</i> 
 │   ├─ 📜 <b>ObservationZoneSalvage.java</b> <i>(if all else fails, drive to the corner. 0+0+3)</i>
 │   └─ 📜 <b>TwoSpecimenRisk.java</b> <i>(0+2+3. Parking is maybe. This is semi-risky)</i>
 ├─ 📂 <b>subsystems</b> <i>(contains all seperate subsystems in robot. EXP: Intake, Bucket, Claw)</i>
 ├─ 📂 <b>support</b> <i>(contains all supporting files that aren't being used directly. Most utility classes and non-specific classes go here)</i>
 ├─ 📂 <b>testing</b> <i>(contains all testing programs and tuning files)</i>
 ├─ 📜 <b>Robot.java</b> <i>(main robot file that respresents the entire robot. Every teleop or autonomous files instantiates this class)</i>
 └─ 📜 <b>Settings.java</b> <i>(all settings and constant values so that it can be changed easily)</i>
</pre>

# Autonomous Routines
- [x] **ObservationZoneSalvage.java**: 0+0+3. If all else fails, drive to the corner.
- [x] **TwoSpecimenRisk.java**: 2+0+3. Parking is maybe. This is semi-risky
- [ ] **ThreeSpecimenRisk.java**: 3+0+3. This is the most risky program. It goes for 3 specimens and parks.
- [ ] **FourSpecimenUltraRisk.java**: 4+0+3. This hasn't been made yet. Our robot is too slow lol
- [ ] **ThreeSample.java**: 0+3+3. Sample Program. Gets two samples from the field and parks.
- [ ] **FourSampleRisk.java**: 0+4+3. Sample Program. Gets all samples from the field and parks.