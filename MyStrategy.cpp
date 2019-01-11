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
ZOFF = 0.66*(RRAD+BRAD),
ZDEF = -41.0,
XDEF = 8.0;
const p3
GOAL (0.0,0.0,44.0);
int TK = -1;

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


    if (ATK_ID == thisROB.id) { // ATKer
    if (!thisROB.touch) { // in air
        return;
        // could jump off wall
        // or (???) increase radius to kick
    }
        for (int tick=1;tick<=200;tick++) {
            double t = tick * TDUR;
            auto [Bpos,Bvel] = BallT.predict(t);
            p3 dir = Bpos - meP;
            p3 obj = GOAL - Bpos;
            dir -= obj*(ZOFF/obj.norm());
            double n = dir.norm();
            if (n < t*MAGS || tick==200) { // heuristic for when to touch
                dir *= (MAGS/n);
                double j =
                    t < 0.5
                    && n < 2.0*BRAD
                    && (Bvel-meV).flatnorm2() < 3.0
                    && Bpos.y > 1.5*BRAD
                    ? 15.0 : 0.0;
                apply(action,dir,j);
                return;
            }
        }
    } else { // DEFender
        if (!thisROB.touch) { // in air
            return;
            // could jump off wall
            // or (???) increase radius to kick
        }
        if (curBP.z < -25 || (curBP.z < -5 && curBV.z < -20)) // we try to predict a kick
        for (int tick=1;tick<120;tick++) {
            double t = tick * TDUR;
            auto [Bpos,Bvel] = BallT.predict(t);
            p3 dir = Bpos - meP;
            if (fabs(Bpos.x) > XDEF+2*fabs(Bpos.z-ZDEF)) break; // dont do weird shit
            // p3 obj = GOAL - Bpos;
            // dir -= obj*(ZOFF/obj.norm());
            dir.y -= 0.4;
            dir.z -= 1.6;
            p3 delta = dir - meV*t;
            if ( t<0.2
                && dir.y < 3*BRAD
                && dir.flatnorm() < 1
                && min(meV.flatnorm(),(dir-meV*t).flatnorm())<0.4)
            { // immediate need to jump
                //cout << "triggered"<<endl;
                apply(action,dir,15.0);
                return;
            }
            double n = dir.norm();
            if (n < t*MAGS) { // heuristic for when to touch
                dir *= (MAGS/n);
                double j =
                        t < 0.4
                        && fabs(15.0*t - GRAV*t*t/2 - dir.y) < 0.4
                        && fabs(delta.x) < 1.2 && delta.z < -0.3 && delta.z > 1.4 ?
                    15 : 0 ;
                apply(action,dir,j);
                return;
            }
        }
        // rest position - no immediate danger
        p3 obj (min(XDEF,max(-XDEF,XDEF*curBP.x/(55+curBP.z))),0.0,ZDEF);
        p3 dir = obj - meP;
        apply(action,dir*30,0.0);
        return;
    }
}
