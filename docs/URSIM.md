# Validating the daemon with URSim (offline UR controller)

URSim is Universal Robots' official offline simulator — it runs the **real
controller software and executes URScript**, so it validates the things MuJoCo
cannot: that `motion_daemon.script` uploads + runs, connects back, dispatches
commands (moveL/moveJ/speedl/home), the `force_mode` API + cooperative stop are
valid, and the streamed state-frame format matches the real controller.

URSim has **no environment/contact and no real FT**, so force *behavior*
(guarded_move stopping on contact, force_mode pushing at X N) is not meaningfully
testable here — use MuJoCo (`ur5e_control.sim`) for that. The two are
complementary.

## Networking design

The daemon connects **back** to the PC at `pc_host:state_port`. With the
container on `--network host`, the container's `127.0.0.1` is the host's, so:

| setting | value | why |
|---|---|---|
| `controller_ip` / `script_port` | `127.0.0.1` / `30001` | upload the daemon to URSim's primary interface |
| `pc_host` / `state_port` | `127.0.0.1` / **`40002`** | our listener. **Not 30002** — URSim's own secondary interface uses 30002; 40002 avoids the clash |

(`examples_ursim/ursim_validate.py` already uses these.)

## Steps

1. **Install Docker** (one time; needs sudo — run with `!` so you can enter the password):
   ```
   sudo apt-get update && sudo apt-get install -y docker.io
   sudo usermod -aG docker $USER      # optional: docker without sudo (after re-login)
   ```

2. **Run URSim e-series** (UR5e), host networking so all ports + the callback work:
   ```
   sudo docker run -d --name ursim -e ROBOT_MODEL=UR5 --network host universalrobots/ursim_e-series
   ```
   First run pulls the image (~GB, a few minutes). Check it's up: `sudo docker ps`.

3. **Configure PolyScope** in a browser at **http://127.0.0.1:6080/vnc.html**:
   - Initialise the robot: power **ON**, then **release brakes** (START).
   - Enable **Remote Control**: hamburger menu → *Settings → System → Remote
     Control → Enable*, then switch the top-right toggle to **Remote Control**.
     (External programs sent to the interface only run in Remote Control mode.)

4. **Run the validation** (no sudo — the assistant can run this):
   ```
   python examples_ursim/ursim_validate.py
   ```
   Expected: daemon connects back, state parses, moveJ / moveL / home all execute.

## Stop / cleanup
```
sudo docker stop ursim && sudo docker rm ursim
```

## Troubleshooting
- *Daemon never connects back* → Remote Control not enabled, robot not
  initialised, or port 40002 busy. Confirm `sudo docker ps` shows ursim Up.
- *Program rejected* → not in Remote Control mode.
- *Can't reach :6080* → container not running, or host networking blocked.
