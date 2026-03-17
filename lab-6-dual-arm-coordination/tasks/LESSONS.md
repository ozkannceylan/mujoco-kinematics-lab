# Lab 6: Lessons Learned

## Bugs & Fixes

*(none yet — log as they occur)*

## Debug Strategies

### Two separate Pinocchio models
Each arm gets its own `(model, data)` pair. This avoids frame ID conflicts and is simpler than a combined model. Both share the same `ur5e.urdf` from Lab 3.

## Key Insights

*(none yet)*
