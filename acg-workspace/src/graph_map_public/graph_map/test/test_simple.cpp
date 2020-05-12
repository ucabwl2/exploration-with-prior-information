#include "ndt_map/ndt_map.h"

int main(int argc, char **argv){

	std::cout << "Hello13" << std::endl;
	perception_oru::NDTMap* map = new perception_oru::NDTMap(new perception_oru::LazyGrid(0.5));
	map->initialize(0.0,0.0,0.0,10,10,10);

	std::cout << "Bye3" << std::endl;

}



