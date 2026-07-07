# ACNeXtGen Complete Fixed Package

## Overview

ACNeXtGen is an attempt to enhance and reinterpret part of Assetto Corsa’s physics and calculation behavior through Lua scripting.

This package is placed in:

```text
apps/lua/ACNextGen/
  ACNextGen.lua
  manifest.ini
  modules/*.lua
```

Its purpose is not to decorate the surface of the simulation, nor to simply add visual or force-feedback effects.
ACNeXtGen was created as a “root and trunk” approach: a project that studies the behavior of the car itself, then reshapes the way grip, load, compliance, drivetrain response, suspension input, tire memory, yaw behavior, and road information are interpreted.

By analyzing and editing the Lua scripts, anyone can touch the behavior of Assetto Corsa not only from the vehicle data folder, but from a more physical and systemic layer.

This project also works without placing `script.lua` inside each individual car.
That point is important. ACNeXtGen is intended to remain universal, modular, and independent from per-car edits as much as possible.

## Philosophy

ACNeXtGen was born from one belief:

A simulator should not only move a car.
It should give the driver a reason to understand the car.

The goal of this project is to help Assetto Corsa express more of the hidden work happening beneath the surface: the contact patch, the tire carcass, the load path, the driveline windup, the suspension delay, the recovery from slip, the yaw moment budget, and the dialogue between road, tire, chassis, and driver.

This is not a claim that ACNeXtGen is perfect.
Rather, it is a completed step — one possible contribution toward better physics, deeper vehicle behavior, and a more honest driving experience.

## Message

The One Point One Plan is now complete.

To everyone who opens these Lua files, reads them, edits them, breaks them, repairs them, and improves them:

Please do not treat physics as a closed box.

Please question it.
Please analyze it.
Please improve it.
Please search for what the car is really trying to say.

I sincerely hope this project becomes a small aid for those who pursue better physics, better simulation, and the truth hidden inside vehicle behavior.

May every user keep the power to seek truth.

## Credits

I took part in the creation and direction of this project, while the overall construction and refinement were supported with GPT-5.5.

Thank you for walking with ACNeXtGen.
And from here on as well — I look forward to what we will build next.

