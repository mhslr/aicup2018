# Russian A.I. Cup - codeball 2018
website http://russianaicup.ru/

This strategy got me to rank ~175 in sandbox at round 1.
It is very light: I could use it with the graphical local runner at 60fps.
Allowed time in competition was 20s for "decision" per every tick, sampling rate was 60 ticks per second.

## Content
- MyStrategy.cpp: simple strategy with 1 attacker trying to catch ball asap and push it and
  1 defender staying at base and waiting for "danger" to act
- geo.cpp: extensively used for predictions since most trajectories are parabolas these mwthods allow precise interpolation in time. (it is still missing bounces, but it is easy to implement and they stop being relevant pretty quick (geometric sequence for the height))
- parameters.cpp: some constants used for computations
