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

int main(int argc, char **argv) {
    // superficial settings
    sf::VideoMode video_mode(800, 600);
    char win_title[] = "PendBalloon Sim";
    sf::ContextSettings settings; settings.antialiasingLevel = 8;
    sf::Color foreground(0xff, 0xff, 0xff);
    sf::Color background(0xc0, 0xc0, 0xc0);
    sf::Color outline(0x00, 0x00, 0x00);
    double fixed_x = 300, fixed_y = 300;
    double joint_radius = 30.f;
    double line_width = 10.f;

    // physical settings
    double rod_length = 150.f;
    double initPendAngle = 0.f;
    double initBalloonAngle = 0.f;

    // chipmunk setup
    cpVect gravity = cpv(0, 750);
    cpSpace *space = cpSpaceNew();
    cpSpaceSetGravity(space, gravity);

    // create window
    sf::RenderWindow window(video_mode, win_title, sf::Style::Default, settings);

    // fixed point
    sf::RectangleShape fixed(sf::Vector2f(2*joint_radius, joint_radius));
    fixed.setOrigin(joint_radius, joint_radius/2);
    fixed.setFillColor(foreground);
    fixed.setOutlineThickness(-line_width);
    fixed.setOutlineColor(outline);
    fixed.setPosition(fixed_x, fixed_y);

    // rod from fixed point to pendulum
    sf::RectangleShape rodFixedPend(sf::Vector2f(line_width, rod_length));
    rodFixedPend.setOrigin(line_width/2, 0);
    rodFixedPend.setFillColor(outline);
    rodFixedPend.setPosition(fixed_x, fixed_y);

    // pendulum
    // physical
    cpFloat pendMass = 5;
    cpFloat pendMoment = cpMomentForCircle(pendMass, joint_radius, 0, cpvzero);
    cpBody *pendBody = cpBodyNew(pendMass, pendMoment);
    cpVect pendDir = cpv(std::sin(D2R(initPendAngle)), std::cos(D2R(initPendAngle)));
    cpBodySetPos(pendBody, cpv(fixed_x, fixed_y) + cpvmult(pendDir, rod_length));
    cpSpaceAddBody(space, pendBody);
    // visual
    sf::CircleShape pend(joint_radius);
    pend.setOrigin(joint_radius, joint_radius);
    pend.setFillColor(foreground);
    pend.setOutlineThickness(-line_width);
    pend.setOutlineColor(outline);
    // attachment
    cpConstraint *pinPendFixed = cpPinJointNew(space->staticBody, pendBody, cpv(fixed_x, fixed_y), cpvzero);
    cpSpaceAddConstraint(space, pinPendFixed);

    // rod from pendulum to balloon
    sf::RectangleShape rodPendBalloon(sf::Vector2f(line_width, rod_length));
    rodPendBalloon.setOrigin(line_width/2, 0);
    rodPendBalloon.setFillColor(outline);
    rodPendBalloon.setPosition(fixed_x, fixed_y + rod_length);

    // balloon
    // visual
    sf::CircleShape balloon(joint_radius);
    balloon.setOrigin(joint_radius, joint_radius);
    balloon.setFillColor(foreground);
    balloon.setOutlineThickness(-line_width);
    balloon.setOutlineColor(outline);
    balloon.setPosition(fixed_x, fixed_y + 2*rod_length);
    // physical
    cpFloat balloonMass = 5;
    cpFloat balloonMoment = cpMomentForCircle(balloonMass, joint_radius, 0, cpvzero);
    cpBody *balloonBody = cpBodyNew(balloonMass, balloonMoment);
    cpVect balloonDir = cpv(std::sin(D2R(initBalloonAngle)), -std::cos(D2R(initBalloonAngle)));
    cpBodySetPos(balloonBody, cpBodyGetPos(pendBody) + cpvmult(balloonDir, rod_length));
    cpSpaceAddBody(space, balloonBody);
    // attachment
    cpConstraint *pinBalloonPend = cpPinJointNew(pendBody, balloonBody, cpvzero, cpvzero);
    cpSpaceAddConstraint(space, pinBalloonPend);

    cpBodyApplyImpulse(pendBody, cpv(-5000, 0), cpvzero);
    cpBodyApplyImpulse(balloonBody, cpv(5000, 0), cpvzero);

    // event and physics loop
    cpVect fixedPos = cpv(fixed_x, fixed_y);
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
        window.draw(pend);
        window.draw(balloon);
        window.draw(fixed);
        window.display();

        // step physics engine
        cpSpaceStep(space, timeStep);
    }

    return 0;
}
