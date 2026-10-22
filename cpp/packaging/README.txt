AP Tuner (C++ edition)
======================

ArduPilot rate-PID and notch-filter tuning from flight logs and live over MAVLink. One program, no runtime to
install: unzip, then double-click "Install AP Tuner.cmd" (or run aptuner.exe directly).

Programs
  aptuner.exe           the app: opens its own window (Microsoft Edge app mode), closes with it
                          aptuner.exe --port 8092   another port
                          aptuner.exe --open        use the default browser instead of an app window
                          aptuner.exe --server      server only (http://127.0.0.1:8090)
  aptuner-analyze.exe   command-line log analysis:  aptuner-analyze <log.bin> [--json out.json] [--param out.param]
  aptuner-link.exe      link check:                 aptuner-link auto   |   aptuner-link serial COM5 57600
  aptuner-heavy.exe     model tuning of heavy / slow drones from logs of one vehicle:
                          aptuner-heavy <log.bin> [more.bin ...] [--plan plan.json] [--out folder] [--spacing m]

Model tune (heavy and slow drones)
  The "Model tune" tab: drop logs of one drone, or connect (Mission Planner is found by itself) and press Start
  listening. The app finds what's wrong, builds a model of the drone, replays your real flights on it, designs the
  settings and gives you the exact values. Type your own and the model re-checks them; download the .param file of
  the values shown and load it in Mission Planner (Full Parameter List > Load from file > Write Params); or press
  "Write to drone" (connected, disarmed, the same drone: UNDO file first, every value read back, rolled back if one
  does not hold). Fly the step, bring the log (or fly while the app listens) and it checks its own prediction.

Folders
  web\        the user interface (HTML / JavaScript)
  docs\       how it works, validation results, the C++ port plan
  sessions\   session journals, uploaded logs, Telemetry + USB downloads (created on first use)

Safety
  The app never writes parameters without asking. Live changes are applied in small steps, each watched for
  oscillation, and reverted automatically if the vehicle starts to oscillate. Every write is journaled and can be
  undone. Proposals from flights with little stick input are marked "lower confidence".

Licence: MIT (see LICENSE). Source: the cpp\ folder of the AP Tuner project.
