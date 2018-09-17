#include <cmath>
#include <string>
#include <iostream>
#include "svo.h"

int main(int argc, char **argv) {
	std::cout << "Hello World" << std::endl;
	svo obj(argc, argv);
    obj.continousOperation();
	return 0;
}
