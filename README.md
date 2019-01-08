# Russian A.I. Cup - codeball 2018

## todo
We will first focus on single robot - ball interactions.

We will consider two different roles:

### ATK
- don't kick in wrong direction
- deviate opponent kick
- what to value in a strategy:
  - kicking early
  - in right direction
  - with enough impact force

### DEF
- will try to get to optimal pending position
- danger flags meaning need to kick before
  - ball going to enter
  - close opponent going to kick

## remarks:
- since there is no friction, trajectories for in-air objects are just parabolas (without nitro)
- it is often interesting to work in the 2D plane for finding interaction time
- should then do some regression in order to find good strategies
