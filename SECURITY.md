# Safety and security

AP Tuner changes autopilot settings, so a defect can be a flight-safety problem, not only a software bug.

## Report

* **Flight-safety problems** (a proposal that made an aircraft unstable or uncontrollable, a value written that was
  not the one shown, a write to the wrong vehicle, a write while armed without the in-flight switch on): open an issue
  with the *Bug report* template and put **[SAFETY]** at the start of the title. Attach the log and the AP Tuner
  parameter / UNDO files if you can. These are looked at first.
* **Security problems** (for example a way for another program or a web page to make the app write to a vehicle):
  please do not open a public issue. Use GitHub's private vulnerability reporting (the repository's *Security* tab,
  *Report a vulnerability*).

## What protects the vehicle today

See "Safety" and "Status and limitations" in the [README](README.md): nothing is written while the vehicle is armed
unless you turn that on, every write is read back and rolled back on failure, an UNDO file is saved first, the app
writes only to the autopilot whose system id matches the analysed flights, and the local web interface accepts
changes only from its own page.

## Supported versions

Only the latest release gets fixes.
