// general purpose headers
#include <iostream>

// cfml graphics headers
#include <SFML/Graphics.hpp>

// chipmunk physics headers
#include <chipmunk/chipmunk.h>
#include <cmath>

// degree <-> radian helper macros
#define D2R(a) (3.141592 * (a) / 180.0)
#define R2D(a) (180.0 * (a) / 3.141592)
#define dsin(a) (std::sin(D2R(a)))
#define dcos(a) (std::cos(D2R(a)))

int main(int argc, char **argv) {
    // superficial settings
    sf::VideoMode video_mode(800, 600);
    char win_title[] = "PendBalloon Sim";
    sf::ContextSettings settings; settings.antialiasingLevel = 8;
    sf::Color foreground(0xff, 0xff, 0xff);
    sf::Color background(0xc0, 0xc0, 0xc0);
    sf::Color outline(0x00, 0x00, 0x00);
    double jointRadius = 30.f;
    double lineWidth = 10.f;

    // physical settings
    cpVect fixedPos = cpv(300, 300);
    double rodLength = 150.f;
    double initPendAngle = 90.f;
    double initBalloonAngle = 90.f;
    double buoyancy = 5000;
    double gravity = 500;
    double drag = 1;

    // chipmunk setup
    cpSpace *space = cpSpaceNew();
    cpSpaceSetGravity(space, cpv(0, gravity));

    // create window
    sf::RenderWindow window(video_mode, win_title, sf::Style::Default, settings);

    // fixed point
    sf::RectangleShape fixed(sf::Vector2f(2*jointRadius, jointRadius));
    fixed.setOrigin(jointRadius, jointRadius/2);
    fixed.setFillColor(foreground);
    fixed.setOutlineThickness(-lineWidth);
    fixed.setOutlineColor(outline);
    fixed.setPosition(fixedPos.x, fixedPos.y);

    // rod from fixed point to pendulum
    sf::RectangleShape rodFixedPend(sf::Vector2f(lineWidth, rodLength));
    rodFixedPend.setOrigin(lineWidth/2, 0);
    rodFixedPend.setFillColor(outline);
    rodFixedPend.setPosition(fixedPos.x, fixedPos.y);

    // pendulum
    // physical
    cpFloat pendMass = 1;
    cpFloat pendMoment = cpMomentForCircle(pendMass, jointRadius, 0, cpvzero);
    cpBody *pendBody = cpBodyNew(pendMass, pendMoment);
    cpVect pendDir = cpv(std::sin(D2R(initPendAngle)), std::cos(D2R(initPendAngle)));
    cpBodySetPos(pendBody, fixedPos + cpvmult(pendDir, rodLength));
    cpSpaceAddBody(space, pendBody);
    // visual
    sf::CircleShape pend(jointRadius);
    pend.setOrigin(jointRadius, jointRadius);
    pend.setFillColor(foreground);
    pend.setOutlineThickness(-lineWidth);
    pend.setOutlineColor(outline);
    // attachment
    cpConstraint *pinPendFixed = cpPinJointNew(space->staticBody, pendBody, fixedPos, cpvzero);
    cpSpaceAddConstraint(space, pinPendFixed);

    // rod from pendulum to balloon
    sf::RectangleShape rodPendBalloon(sf::Vector2f(lineWidth, rodLength));
    rodPendBalloon.setOrigin(lineWidth/2, 0);
    rodPendBalloon.setFillColor(outline);
    rodPendBalloon.setPosition(fixedPos.x, fixedPos.y + rodLength);

    // balloon
    // visual
    sf::CircleShape balloon(jointRadius);
    balloon.setOrigin(jointRadius, jointRadius);
    balloon.setFillColor(foreground);
    balloon.setOutlineThickness(-lineWidth);
    balloon.setOutlineColor(outline);
    balloon.setPosition(fixedPos.x, fixedPos.y + 2*rodLength);
    // physical
    cpFloat balloonMass = 1;
    cpFloat balloonMoment = cpMomentForCircle(balloonMass, jointRadius, 0, cpvzero);
    cpBody *balloonBody = cpBodyNew(balloonMass, balloonMoment);
    cpVect balloonBuoy = cpv(0, -gravity - buoyancy);
    cpBodySetForce(balloonBody, balloonBuoy);
    cpVect balloonDir = cpv(dsin(initBalloonAngle), -dcos(initBalloonAngle));
    cpBodySetPos(balloonBody, cpBodyGetPos(pendBody) + cpvmult(balloonDir, rodLength));
    cpSpaceAddBody(space, balloonBody);
    // attachment
    cpConstraint *pinBalloonPend = cpPinJointNew(pendBody, balloonBody, cpvzero, cpvzero);
    cpSpaceAddConstraint(space, pinBalloonPend);

    //cpBodyApplyImpulse(pendBody, cpv(-1000, 0), cpvzero);
    cpBodyApplyImpulse(balloonBody, cpv(0, 1000), cpvzero);

    // event and physics loop
    cpFloat timeStep = 1.0/60.0;
    while(window.isOpen()) {
        // handle sfml events
        sf::Event event;
        while(window.pollEvent(event)) {
            if(event.type == sf::Event::Closed) {
                window.close();
            }
        }

        // query engine for physics data and update
        cpVect pendPos = cpBodyGetPos(pendBody);
        pend.setPosition(pendPos.x, pendPos.y);
        rodPendBalloon.setPosition(pendPos.x, pendPos.y);

        cpVect balloonPos = cpBodyGetPos(balloonBody);
        balloon.setPosition(balloonPos.x, balloonPos.y);

        cpVect dFixedPend = pendPos - fixedPos;
        double angleFixedPend = std::atan2(dFixedPend.y, dFixedPend.x);
        rodFixedPend.setRotation(R2D(angleFixedPend) - 90);

        cpVect dPendBalloon = balloonPos - pendPos;
        double anglePendBalloon = std::atan2(dPendBalloon.y, dPendBalloon.x);
        rodPendBalloon.setRotation(R2D(anglePendBalloon) - 90);

        // display
        window.clear(background);
        window.draw(rodFixedPend);
        window.draw(rodPendBalloon);
        window.draw(fixed);
        window.draw(pend);
        window.draw(balloon);
        window.display();

        // step physics engine
        cpSpaceStep(space, timeStep);
        
        // apply drag
        cpVect balloonVel = cpBodyGetVel(balloonBody);
        cpBodySetForce(balloonBody, balloonBuoy + cpvmult(balloonVel, -drag));
    }

    return 0;
}
