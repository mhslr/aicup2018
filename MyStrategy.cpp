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
GOAL (0.0,0.0,44.0),
HOME (0.0,0.0,-43.0);

// current
bool DDEF = 0; // will kick
int DTIK;      // till jump
p3 DOBJ;       // general direction
double DJUM;   // jump

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

    if (ATK_ID == thisROB.id) { // ATKer
        for (int tick=1;tick<=300;tick++) { // up to 5 second
            double t = tick * TDUR;
            auto [Bpos,Bvel] = BallT.predict(t);
            p3 dir = Bpos - meP;
            p3 obj = GOAL - Bpos;
            dir -= obj*(ZOFF/obj.norm());
            double n = dir.norm();
            if (n < t*MAGS || tick==200) { // heuristic for when to touch
                dir *= (MAGS/n);
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
        if (DDEF) {
            if (--DTIK<=0) {
                apply(action,DOBJ,DJUM);
                DDEF = 0;
                return;
            }
            apply(action,DOBJ,0.0);
            return;
        }
        {
            p3 dir = HOME-meP;
            dir.x += curBP.x/4;
            double n = dir.norm();
            if (n > 2.0*RRAD) {
                dir *= (MAGS/n);
                apply(action,dir,0.0);
                return;
            }
        }
        double tgrd = MAGS/MACC;
        double tair = 0.3;
                        // try to reduce it to 0.6 and following coefficients also have to be updated
        auto [Bpos,Bvel] = BallT.predict(tgrd+tair);
        p3 obj = Bpos - meP;
        obj.z -= RRAD + BRAD;
        double fn = obj.flatnorm();
        if (fn < MACC*tgrd*tgrd/2 + MAGS*tair) { // reachable in 1 sec
            // j*tair - GRAV*tair*tair/2 = H
            DJUM = obj.y/tair + GRAV*tair/2;
            DOBJ = obj*1000;
            DTIK = tgrd/TDUR;
            DDEF = 1;
            apply(action,DOBJ,0.0);
        }
        return;
    }
}
