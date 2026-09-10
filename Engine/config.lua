-- Legacy remains the default until the production gates pass.
-- Set mode to "pilot" to run the definition-backed brake_fade at its original slot.
-- Restart the app after editing definitions or mode: no runtime file reloads.
return { mode = "legacy", observerHz = 20 }
