# Project Motivation

The main motivation for creating this model is to provide an interesting
non-linear system for expanding my knowledge of control algorithms.
The well-studied nature of physics should allow for validation of its
realism.

While I retain some concerns about simulations not reflecting reality,
I have largely shifted to the view that a model is absolutely necessary
for building any form of advanced, commerical control system. Of course,
models are important for the nature of control algorithms which it enables,
and perhaps even moreso in the interest of offline testing and CI. If this
conjecture proves to be correct, a further study of system identification
technique may be prudent.

There are many subproblems to potentially explore. A quick brainstorm will
perhaps increase the future fruits of work on this project. First and
foremost, what problem are we trying to solve here?

## Context

Unfornately, it's unlikely to be a problem that's of particular practical
relevance since there isn't a particular demand need for robot controlled
bicycles. (But should there be?) That said, I think there is still plenty
of relevance to the robotics space as it currently exists.

1. Electric vehicles are quite in demand: I'm thinking hoverboards, electric
   bikes, etc. This is of course a slightly different subarea than full
   autonomy since there human interaction involved. Impedance control is
   likely relavant in this space.
2. Fully autonomous driving is very uncertain and probabalistic in nature,
   If you are unsure whether a pedestrian will emerge around an occluded
   corner, it's probably a good idea to slow down. There's of course also
   potential a human interactivity challenge here as well. We signal to each
   other through our controls: for example, slowing down to let someone in
   your lane, or giving the pedestrain confidence to cross the street.
3. And then there's the more independent mode of autotonomy where the robot
   is more isolated from other players, along the lines of manufacturing,
   construction, or food preparation applications. Of course, some
   interaction is always necessary, but it's maybe simpler: play, pause,
   stop and do a dance. There is still some degree of environental
   uncertainty and interactivity, otherwise, it wouldn't be a very
   interesting problem!
4. Lastly, there's my more abstract and far-fetch notion of bio-embedded
   robotics. How can we use control theory to improve human health? The
   exact medium for interaction is where I get stuck here. Are the sensor
   readings that of an Apple watch and the control signal notifications
   of inspiration? Or are the deeper, more fundamental ways which we can
   leverage technology to help people recover more quickly from injury,
   regain control over mental health, increase longevity, boost atheletic
   performance? Biosensors, physical therapy robots? These are all pleasant
   ideas, but let's keep an eye on feasibility. What are research institutes
   and industry actually working on?

Okay, so let's leave 4. on the backburner for now. Within the first three
problem areas the major theme seems to be interaction and uncertainty,
each case with its own specifics and approach.

## Nonlinearity

The so-far-unmentioned topic that comes to mind is non-linearity. How
does it relate to the use cases above? For impedance controlled application,
there's a pretty good chance the system is nonlinear. A car is similarly
non-holonomic to the bike, but the dynamics are probably somewhat more
straightforward (linearizable). Environmental interaction in terms of
force-controlled manipulators is pretty non-linear with it's contact
discontinuities (and of course the inherent non-linearity of
joint-controlled mechanisms).

## Subproblems
With all that context in mind, let's throw some ideas at the wall and see
what sticks.
1. At slow speeds, the bike is passively unstable. This certainly makes it
   much more interesting. Can we turn and then rapidly accelerate into a
   straight motion.
2. Recovery motions to return to a nominal position from a compromised one.
   Is there a good way to do this from arbitrary positions? Or do we need
   certain constraints
3. Along the lines of the first problem, another interesting problem is the
   navigation of narrow environments, esp. due to it's highly dynamic,
   non-holonomic nature. As soon as we talk about navigation, there's a
   certain planning element that is implied. A first step would be operating
   with a known map. An interesting extension would be to add a degree of
   uncertainty, such that the planning would need to be recomputed online.
   There's a pretty wild array of possible assumptions here, but it feels
   like this has some nice overlap with self-driving problems
4. Impedance control is potentially interesting. Certainly has been
   implemented before on existing electric bikes, in the specific
   form of impedance controlled pedalling. This of course somewhat dodges
   the more complex dynamics of the bicycle, but would be quite interesting
   (and surely useful!) nonetheless.
5. A more classical controls approach of stabilizing the system could also
   be implemented, and might be a useful benchmark against which to compare
   the more advanced techniques. If we go this direction, we might want to
   start with the simpler problem of control within the nicely linearized
   regime. The input would be a path the we would like to follow. Of course,
   once we are outside of the approximately linear regime, we start running
   into many unfeasible trajectories.
6. Of course, more advanced control techniques could also be used
   solve the simpler problem, and maybe to a certain extent we want to avoid
   conflating the problem that is being solved, with the control solution.
   But on the other hand, each control strategy has it's own form of input
   so perhaps the effort of decoupling the two is more effort than it's
   worth.
7. Different actuation methods: my initial interest was in steering the bike
   with 2d actions to the seat. I'm sure holding the steering wheel helps!

## Next Steps
Okay, good brainstorming sesh! It seems like I'm gaining some clarify on
next steps. It seems like:
1. Start with classical control approach. Figure out the actuation mechanism
   input and output formats.
2. Achieve the same thing with a model-based approach.
3. From here we have a lot of options to choose from:
      a. Slow speed, low profile rapid turn
      b. Recovery motions
      c. non-holonomic motions; fastest way to reach some final state; can
          we compute the most efficient way parallel park?
4. After completing the above, move onto a more exhaustive navigation
   problem. Especially with some online uncertainty, we can really put the
   performance to the test!

## Time estimates
So regarding timeline for all of this... How much time do I want to devote?
Hanging out in Buenos Aires is nice and all, but I've got places to see,
the world to explore. Assuming that the robotics world of work is something
I'd like to continue with, it certainly will be fruitful for future career
prospects. And regardless, they are fun problems to think about!

But to set expectations to myself and for planning purposes, let's set some
time estimates. They will of course be horrendously inaccurate, because
solving novel problems (at least novel to me) is inherently uncertain. But
it is still a worthwhile exercise.

1. Classic controls -> 1 days (8 hours)
2. Model-based -> 2 day
3. Maybe another 2-3 days for each subproject
4. Full navigation (depending on how easily the previous projects are
   extended) maybe 3 days.

So probably try to knock out the first two while still in BA and work from
there!
