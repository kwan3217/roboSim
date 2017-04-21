#include <signal.h>

int Argc;
char** Argv;

static volatile bool done=false;

void intHandler(int dummy) {
  done=true;
}

void setup() {

}

void loop() {

}

int main(int argc, char** argv) {
  signal(SIGINT, intHandler); //trap SIGINT (Ctrl-C) so that we exit instead of crashing, thus running the destructors and flushing our logs
  Argc=argc;
  Argv=argv;
  setup();
  while(!done) {
    loop();
  }
}
