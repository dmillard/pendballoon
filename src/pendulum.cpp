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
    char win_title[] = "Pendulum Sim";
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
    double gravity = 750.f;

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
    cpVect pendDir = cpv(dsin(initPendAngle), dcos(initPendAngle));
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

    cpBodyApplyImpulse(pendBody, cpv(-5000, 0), cpvzero);

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

        cpVect dFixedPend = pendPos - fixedPos;
        double angleFixedPend = std::atan2(dFixedPend.y, dFixedPend.x);
        rodFixedPend.setRotation(R2D(angleFixedPend) - 90);

        // display
        window.clear(background);
        window.draw(rodFixedPend);
        window.draw(pend);
        window.draw(fixed);
        window.display();

        // step physics engine
        cpSpaceStep(space, timeStep);
    }

    return 0;
}
