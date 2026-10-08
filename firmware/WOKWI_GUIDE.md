# Wokwi guide, from zero (browser only, nothing to install)

You need: a web browser. A free wokwi.com account is optional for this.

## A. Create the project
1. Go to https://wokwi.com and click **Start from scratch** (or "New Project").
2. Choose **ESP32**, then **Arduino-ESP32** (a plain blank sketch).
3. You now see tabs: `sketch.ino`, `diagram.json`, and a "Library Manager". Leave this window open.

## B. Add the code files
In the repo these live in `firmware/` (open them on GitHub, branch `claude/funny-noether-0b1r6a`, click **Raw**, select-all, copy).
4. **sketch.ino**: click the tab, select everything, delete, paste the contents of `firmware/src/main.cpp`.
5. Click the little **down-arrow next to the `sketch.ino` tab (New file)**, name it `echo_params.h`, paste `firmware/include/echo_params.h`.
6. Same again: new file `echo_core.hpp`, paste `firmware/include/echo_core.hpp`.
7. **diagram.json**: click the tab, select all, delete, paste `firmware/diagram.json`.

## C. Run
8. Click the green **Start the simulation** (play) button. The first build takes ~1 minute.
9. **What you should see:**
   - The serial monitor prints `# Echo Balancer NIS gate | MPU6050 ok ...`, then lines like `0.00,3.1,...,NORMAL,NORMAL,...`.
   - The **green LED is on**. Leave it ~10 s: nothing should change (this is the "nominal" check).
   - If it says `MPU6050 NOT FOUND`, the wiring in the diagram is wrong: stop and send me a screenshot.
   - If the build fails, copy the red error text and send it to me.

## D. Try the faults (this is the demo)
Click inside the serial monitor box, then type one letter (no Enter needed on most browsers; if nothing happens, type it and press Enter):
10. `g` -> gyro bias fault. Within ~0.5 s the LED goes yellow, then **red** and the buzzer beeps. Type `g` again to switch off, `r` to clear all.
11. `a` -> accel noise x5: goes red within ~0.2 s. `r` to clear. (Note `nis_mode` vs `tilt_mode` columns: the tilt gate stays NORMAL, ours reacts.)
12. `p` -> a push. Honest expectation: probably **no** mode change (known limit).
13. `m` -> payload shift: goes red, `fallen` becomes 1. `r` to clear.
14. Click the **MPU6050** part and drag its **accelerometer X** slider away from its start value: it counts as a sensor fault, LED goes red. Move it back and wait ~10 s to recover.
15. Click the **KY-040** knob and turn it: encoder (wheel slip) offset.

## E. Save evidence for the paper
16. Take a screenshot of the circuit with the red LED on, and copy the serial log (select text, copy) into a file.
17. Click **Save** (needs the free login) and copy the project URL to cite in the report.

## F. Optional: run it locally (VS Code)
Install VS Code, the PlatformIO and Wokwi extensions, open the `firmware/` folder, run `pio run`, then press F1 -> "Wokwi: Start Simulator" (free license via the prompt).
