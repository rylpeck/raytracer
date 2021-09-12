#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <algorithm>

#include "matrixManip.h"
#include "fileReader.h"
#include "main.h"



void fullReadin(std::string filename, std::string saveMe, camera& maincamera, std::vector <ambient>& ambientList, std::vector <light>& lightList, model& models){
    //this is long lets list these
    //string filename, string saveme, camera maincamera
    //ambientlist alist, listlist lList, 
    //sphere list

    std::fstream inputFile;
    inputFile.open(filename);
    std::string lineIn;
    std::string word;

    //std::cout << "hey" << "\n";

    object3d TMatrix;

    transClear(TMatrix);

    triangle tri;
    
    //std::cout << "initialization complete"  << "\n";
       
    if (inputFile.fail()){
        std::cout << "Failure in opening driver\n";
        return;
    }

     while (getline(inputFile, lineIn)){
        
        //std::cout << lineIn << "\n";
        std::stringstream breaker(lineIn);
        //this while loop breaks it apart line by line and does what it needs to do
        
        
        while (breaker >> word){
            //std::cout << word << "\n";
            //code to ignore any comments
            if (word.compare("#") == 0){
                //std::cout << "WEEEEEEEEE" << "\n";
                break;
            }

            if (word.compare("camera") == 0){

                (breaker >> word);
                maincamera.eye[0] = std::stod(word);
                (breaker >> word);
                maincamera.eye[1] = std::stod(word);
                (breaker >> word);
                maincamera.eye[2] = std::stod(word);
                maincamera.vEye << maincamera.eye[0], maincamera.eye[1], maincamera.eye[2];
    
                (breaker >> word);    
                maincamera.look[0] = std::stod(word);
                (breaker >> word);  
                maincamera.look[1] = std::stod(word);
                (breaker >> word);  
                maincamera.look[2] = std::stod(word);
                maincamera.vLook << maincamera.look[0], maincamera.look[1], maincamera.look[2];


                (breaker >> word);  
                maincamera.up[0] = std::stod(word);
                (breaker >> word);  
                maincamera.up[1] = std::stod(word);
                (breaker >> word);  
                maincamera.up[2] = std::stod(word);
                maincamera.vUp << maincamera.up[0], maincamera.up[1], maincamera.up[2];



                (breaker >> word);  
                maincamera.near += std::stod(word);

                maincamera.W = ((maincamera.vEye - maincamera.vLook));
                maincamera.W = (1/maincamera.W.norm()) * maincamera.W;

                maincamera.U = maincamera.vUp.cross(maincamera.W);
                maincamera.U = (1/maincamera.U.norm()) * maincamera.U;

                maincamera.V = maincamera.W.cross(maincamera.U);
                
              
                break;
            }

            if (word.compare("bounds") == 0){
                
                (breaker >> word); 
                maincamera.left = std::stod(word);
                (breaker >> word); 
                maincamera.right = std::stod(word);
                (breaker >> word); 
                maincamera.bottom = std::stod(word);
                (breaker >> word); 
                maincamera.top = std::stod(word);
                    
                
                break;
            }

            if (word.compare("res") == 0){

                (breaker >> word);
                maincamera.width = stod(word);
                (breaker >> word);
                maincamera.height = stod(word);
                
                break;
            }

            if (word.compare("ambient") == 0){
                
                ambient newLight;

                (breaker >> word);
                newLight.red = stod(word);
                //std::cout << "ambient red: " << word << " " << newLight.red << "\n";
                (breaker >> word);
                newLight.green = stod(word);
                (breaker >> word);
                newLight.blue = stod(word);

                newLight.rgbv<< newLight.red, newLight.green, newLight.blue;

                ambientList.push_back(newLight);
                                    
                break;
            }

            if (word.compare("light") == 0){

                light newLight;

                
                (breaker >> word);
                newLight.xcord = stod(word);
                (breaker >> word);
                newLight.ycord = stod(word);
                (breaker >> word);
                newLight.zcord = stod(word);
                newLight.cordv << newLight.xcord, newLight.ycord, newLight.zcord;
                
                (breaker >> word);    
                newLight.w = stod(word);

                (breaker >> word);  
                newLight.er = stod(word);
                (breaker >> word);  
                newLight.eg = stod(word);
                (breaker >> word);  
                newLight.eb = stod(word);
                newLight.ev << newLight.er, newLight.eg, newLight.eb;
              



                lightList.push_back(newLight);

                break;
            }
                
            if (word.compare("sphere") == 0){

                sphere newSphere;

                (breaker >> word);
                newSphere.xcord = stod(word);
                (breaker >> word);
                newSphere.ycord = stod(word);
                (breaker >> word);
                newSphere.zcord = stod(word);
                newSphere.cordsv << newSphere.xcord, newSphere.ycord, newSphere.zcord;

                (breaker >> word);    
                newSphere.radius = stod(word);

                (breaker >> word);  
                newSphere.kar = stod(word);
                (breaker >> word);  
                newSphere.kag = stod(word);
                (breaker >> word);  
                newSphere.kab = stod(word);
                newSphere.kav << newSphere.kar, newSphere.kag, newSphere.kab;

                (breaker >> word);  
                newSphere.kdr = stod(word);
                (breaker >> word);  
                newSphere.kdg = stod(word);
                (breaker >> word);  
                newSphere.kdb = stod(word);
                newSphere.kdv << newSphere.kdr, newSphere.kdg, newSphere.kdb;
                
                (breaker >> word);  
                newSphere.ksr = stod(word);
                (breaker >> word);  
                newSphere.ksg = stod(word);
                (breaker >> word);  
                newSphere.ksb = stod(word);
                newSphere.ksv << newSphere.ksr, newSphere.ksg, newSphere.ksb;

                (breaker >> word);  
                newSphere.krr = stod(word);
                (breaker >> word);  
                newSphere.krg = stod(word);
                (breaker >> word);  
                newSphere.krb = stod(word);
                newSphere.krv << newSphere.krr, newSphere.krg, newSphere.krb;

                (breaker >> word);
                newSphere.ni = stod(word);

                newSphere.isSphere = true;


                models.sList.push_back(newSphere);


            }

            if (word.compare("move") == 0){
                (breaker >> word);
                std::string para1 = word;
                (breaker >> word);
                std::string para2 = word;
                (breaker >> word);
                std::string para3 = word;
                    
                transMove(para1, para2, para3, TMatrix);
                break;
            }

            if (word.compare("rota") == 0){
                //std::cout << "time to rotate" << "\n";
                (breaker >> word);
                std::string para1 = word;
                (breaker >> word);
                std::string para2 = word;
                (breaker >> word);
                std::string para3 = word;
                breaker >> word;
                std::string para4 = word;
                
                transRota(para1, para2, para3, para4, TMatrix);
                //std::cout << "Just roated" << "\n";
                //std::cout << TMatrix.m << "\n";

                break;
            }

            if (word.compare("scale") == 0){
                (breaker >> word);
                std::string para1 = word;
                (breaker >> word);
                std::string para2 = word;
                (breaker >> word);
                std::string para3 = word;
                 
                transScale(para1, para2, para3, TMatrix);
                break;
            }

            if (word.compare("clear") == 0){
                transClear(TMatrix);
                break;
                
            }

            if (word.compare("cutoffAngle") == 0){
                std::cout << "cutting " << "\n";
                (breaker >> word);
                std::cout << "Cut of was: " << stod(word) << "\n";

                tri.cutter = stod(word);

                break;

            }

            if (word.compare("load") == 0){
                //std::cout << "Time to actually load it in" << "\n";
                (breaker >> word);
                
                tri.replica.m = TMatrix.m;

                loadObj(word, tri);

                std::cout << "Post load 3" << "\n";
                std::cout << tri.materialList.size() << "\n";

                //std::cin.get();

                models.tList.push_back(tri);

                tri.materialList.clear();
                tri.faceList.clear();

                break;
            }

            
            if (word.compare("save") == 0){
                //std::cout << "Time to save" << "\n";
                breaker >> word;
                //saveObj(word, tri.replica);
                break;
            }

            if (word.compare("recursionlevel") == 0){
                //std::cout << "Recurved" << "\n";
                (breaker >> word);
                models.recur = stod(word);
                //std::cout << "recurved pt2" << "\n";
                break;
            }


            


            //std::cout << word << "\n";
        }
    }

    std::cout << "loading complete" << "\n";
    //std::cout << models.tList[0].faceList.size() << "\n";



    return;
}