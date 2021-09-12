#ifndef MATRIXMANIP_INCLUDE
#define MATRIXMANIP_INCLUDE
#include "main.h"


int transClear(object3d& tmatrix);

int transMove(std::string para1, std::string para2, std::string para3, object3d& TMatrix);
int transRota(std::string para1, std::string para2, std::string para3, std::string para4, object3d& TMatrix);
int transScale(std::string para1, std::string para2, std::string para3, object3d& TMatrix);
int loadObj(std::string filename, triangle& tri);
int saveObj(std::string filename, object3d& TMatrix);

#endif