#include <SFML/Graphics.hpp>

int main(int argc, char **argv) {
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(800, 600), "SFML works!", sf::Style::Default, settings);

    sf::RectangleShape fixed(sf::Vector2f(60, 30));
    fixed.setFillColor(sf::Color::White);
    fixed.setOutlineThickness(-10);
    fixed.setOutlineColor(sf::Color::Black);
    fixed.setPosition(300, 100);

    sf::RectangleShape stringFixedPendulum(sf::Vector2f(10, 200));
    stringFixedPendulum.setFillColor(sf::Color::Black);
    stringFixedPendulum.setPosition(325, 100);

    sf::CircleShape pendulum(30.f);
    pendulum.setFillColor(sf::Color::White);
    pendulum.setOutlineThickness(-10);
    pendulum.setOutlineColor(sf::Color::Black);
    pendulum.setPosition(300, 270);

    sf::RectangleShape stringPendulumBalloon(sf::Vector2f(10, 200));
    stringPendulumBalloon.setFillColor(sf::Color::Black);
    stringPendulumBalloon.setPosition(325, 300);

    sf::CircleShape balloon(30.f);
    balloon.setFillColor(sf::Color::White);
    balloon.setOutlineThickness(-10);
    balloon.setOutlineColor(sf::Color::Black);
    balloon.setPosition(300, 470);

    while(window.isOpen()) {
        sf::Event event;
        while(window.pollEvent(event)) {
            if(event.type == sf::Event::Closed) {
                window.close();
            }
        }

        window.clear(sf::Color(192, 192, 192));
        window.draw(stringFixedPendulum);
        window.draw(stringPendulumBalloon);
        window.draw(pendulum);
        window.draw(balloon);
        window.draw(fixed);
        window.display();
    }

    return 0;
}
