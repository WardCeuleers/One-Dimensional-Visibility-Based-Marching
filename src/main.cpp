#include <iostream>
#include <ncurses.h>
#include <thread>
#include <chrono>

#include "environment/environment.hpp"
#include "solver/solver.hpp"

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
bool askForInputNumber(const char* message, int &value, int line = 0) {
  mvprintw(line, 0, "%s", message);
  refresh();
  int ch;
  char buffer[10]; // buffer to store user input value
  int bufferIndex = 0;
  while (true) {
    ch = getch();
    // add number to buffer
    if (ch != ERR && ch != '\n' && bufferIndex < 9 && std::isdigit(ch)) {
      buffer[bufferIndex] = ch;
      bufferIndex++;
      buffer[bufferIndex] = '\0';
      mvprintw(line, 0, "%s: %s", message, buffer);
      refresh();
    }
    else if (ch == KEY_BACKSPACE && bufferIndex > 0) {
      bufferIndex--;
      buffer[bufferIndex] = '\0';
      move (line, 0);
      clrtoeol();
      mvprintw(line, 0, "%s: %s", message, buffer);
      refresh();
    }
    else if (ch == '\n') {
      break;
    }
    else if (ch == 'q') {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  // convert buffer to integer
  value = std::stoi(buffer);
  return true;
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void vbd::Solver::changeMarchStepSize() {
  int stepSize;
  clear();
  askForInputNumber("Enter the new marchStepSize: ", stepSize);
  marchStepSize_ = stepSize;
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void vbd::Solver::setMaxIter() {
  int maxIter;
  clear();
  askForInputNumber("Enter the desired target iteration: ", maxIter);
  max_nb_of_iter_ = maxIter;
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void vbd::Solver::setTarget() {
  int maxSteps;
  int nbOfVisits;
  int x, y;
  int line = 0;
  clear();
  while (line < 3) {
    switch (line) {
      case 0:
        if (askForInputNumber("Enter the x coordinate of the target: ", x, line)) {
          target_x_ = x;
          line++;
        }
        else {
          max_nb_of_iter_ = nb_of_iterations_;
          return;
        }
        break;
      case 1:
        if (askForInputNumber("Enter the y coordinate of the target: ", y, line)) {
          target_y_ = ny_-1-y;
          line++;
        }
        else {
          clear();
          line--;
        }
        break;
      case 2:
        if (askForInputNumber("Enter how many times the node can be visited: ", nbOfVisits, line)) {
          nbOfVisits_ = nbOfVisits;
          line++;
        }
        else {
          ::move (line, 0);
          clrtoeol();
          ::move (line-1, 0);
          clrtoeol();
          line--;
        }
    }
  }
  max_nb_of_iter_ = sharedConfig_->max_nb_of_iterations;
  findNode_ = true;
  return;
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
int main() {
  vbd::ConfigParser &parser = vbd::ConfigParser::getInstance();
  if (!parser.parse("config/settings.config")) {
    std::cout << "########################### Parsing results: ####"
                 "########################## \n";
    std::cout << "Error parsing config file" << std::endl;
    return 1;
  } else {
    std::cout << "########################### Parsing results: ####"
                 "########################## \n";
    std::cout << "Config file parsed successfully \n" << std::endl;
  }
  auto config = parser.getConfig();

  vbd::Environment env = vbd::Environment(config);

  vbd::Solver sol = vbd::Solver(env);
  if (!config.marchControl) {
    sol.visibilityBasedSolver();
  }
  else {
    int ch;
    bool running = true;
    while (running) {
      // calculate current solution
      sol.visibilityBasedSolver();
      // initialize ncurses
      initscr();
      cbreak();
      noecho();
      keypad(stdscr, TRUE);
      nodelay(stdscr, TRUE);
      //read key inputs
      bool reading = true;
      while (reading) {
        ch = getch();  
        switch (ch) {
          case KEY_RIGHT:
            sol.increaseMaxIter();
            reading = false;
            break;
          case KEY_LEFT:
            sol.decreaseMaxIter();
            reading = false;
            break;
          case 'p':
            sol.togglePivots();
            reading = false;
            break;
          case 'b':
            sol.toggleBoundaries();
            reading = false;
            break;
          case 'c':
            sol.toggleContourLines();
            reading = false;
            break;
          case 'o':
            sol.toggleCameFrom();
            reading = false;
            break;
          case 's':
            sol.changeMarchStepSize();
            reading = false;
            break;
          case 't':
            sol.setTarget();
            reading = false;
            break;
          case 'i':
            sol.setMaxIter();
            reading = false;
            break;
          case 'q':  // Press 'q' to exit the loop
            endwin();
            return 1;
          case ERR:
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      endwin();
      std::cout << "\033[2J\033[H";
      std::cout.flush();
    }
  }
  return 1;
}