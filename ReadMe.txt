Script -> Plotter -> Either -> modelSim -> linModel/ nonlinModel
                       Or   -> controllerSim -> plant & (linController & linUtoThrust)/(nonlinController & nonlinUtoThrust)/
                       Or   -> adaptSim -> plant & adaptGains & (linController & linUtoThrust)/(nonlinController & nonlinUtoThrust)/

Important Note: When we do variation, all the constants should be global, except in linModel & nonLinModel (Because they are model).

We need to check if anything is wrong with plant in adaptSim.
We need to check our adaptation laws.


FOUND THE MISTAKE:
We successfully made a linear controller that controls a non-linear system.
We were trying to make our plant adapt to a model that has a linear contoller on a LINEAR system,
Instead, we are supposed to adapt to a model that has a linear controller on a NON-LINEAR system.

Analogy:
In life, we adapt to the things that happen to us. But we are supposed to try to follow a model realistic life, not a model idealistic life.
Our plant was trying to adapt to an imaginary system, not a real one.