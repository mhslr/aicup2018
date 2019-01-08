#include "MyStrategy.h"

#include <bits/stdc++.h>
using namespace std;

#include "parameters.cpp"
#include "geo.cpp"

void apply(
        model::Action& action,
        const p3 dir         ,
        const double jump    ) {
    action.target_velocity_x = dir.x;
    action.target_velocity_y = dir.y;
    action.target_velocity_z = dir.z;
    action.jump_speed = jump;
}

MyStrategy::MyStrategy() {}

int ATK_ID;
// parameters
const double
ZOFF = 0.64*(RRAD+BRAD);
const p3
GOAL (0.0,0.0,44.0);

void MyStrategy::act
       (const model::Robot&  thisROB,
        const model::Rules&  rules  ,
        const model::Game&   game   ,
              model::Action& action ) {

    if (!ATK_ID) ATK_ID = thisROB.id; // init roles

    p3 meP (   thisROB.x      ,    thisROB.y      ,     thisROB.z     );
    p3 meV (thisROB.velocity_x, thisROB.velocity_y, thisROB.velocity_z);

    p3 curBP (    game.ball.x     ,    game.ball.y      ,     game.ball.z     );
    p3 curBV (game.ball.velocity_x, game.ball.velocity_y, game.ball.velocity_z);

    traj BallT (curBP, curBV);

    if (!thisROB.touch) { // in air
        return;
        // could jump off wall
        // or (???) increase radius to kick
    }

    if (1 || ATK_ID == thisROB.id) { // ATKer
        for (int tick=1;tick<=300;tick++) { // up to 5 second
            double t = tick * TD;
            auto [Bpos,Bvel] = BallT.predict(t);
            p3 dir = Bpos - meP;
            p3 obj = GOAL - Bpos;
            dir -= obj*(ZOFF/obj.norm());
            double n = dir.norm();
            if (n < t*MGS || tick==200) { // heuristic for when to touch
                dir *= (MGS/n);
                double j =
                    tick < 100
                    && n < 2.0*BRAD
                    && (Bvel-meV).flatnorm2() < 3.0
                    && Bpos.y > 1.5*BRAD
                    ? 15.0 : 0.0;
                apply(action,dir,j);
                return;
            }
        }
    } else { // DEFender
        return;
    }
}
