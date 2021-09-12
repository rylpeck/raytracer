#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

#include "matrixManip.h"
#include "main.h"





int transClear(object3d& TMatrix){
    
    //Eigen::MatrixXd j(4,4);
    //TMatrix.m = j;
    TMatrix.m << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,1;
    
    //std::cout << TMatrix.m << "\n";
    //std::cout << "Leaving Clear" << "\n";

    return 0;
}

int transMove(std::string para1, std::string para2, std::string para3, object3d& TMatrix){

    
    //std::string word;

    Eigen::Matrix4d tempMatrix = Eigen::Matrix4d::Zero(4,4);
    tempMatrix << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,1;

    //std::cout << "In move" << "\n";
    //std::cout << para1 << para2 << para3 << "\n";
    //code to add change to the x, cumulative
    tempMatrix(0,3) = std::stod(para1);
    //code to add change to the y, cumulative
    tempMatrix(1,3) = std::stod(para2);
    //code to add change to teh z, cumulative
    tempMatrix(2,3) =  std::stod(para3);

    TMatrix.m = tempMatrix * TMatrix.m;
    
    //std::cout << TMatrix.m << "\n";

    return 0;
}

int transRota(std::string para1, std::string para2, std::string para3, std::string para4, object3d& TMatrix){

    //std::cout << "Rotation! \n";

    Eigen::Matrix3d tempMatrix = Eigen::Matrix3d::Zero(3,3);

    Eigen::Vector3d vectorW(0,0,0);

    vectorW << stod(para1), stod(para2), stod(para3);
   

    //std::cout << para1 << para2 << para3 << para4 << "\n";

    Eigen::Vector3d vectorM(0,0,0);

    

    vectorM(2) = 1;

    Eigen::Vector3d vectorU(0,0,0);
    Eigen::Vector3d vectorV(0,0,0);

    //std::cout << "Vector W: " << "\n";
    //std::cout << vectorW << "\n";

    vectorW = (1/vectorW.norm()) * vectorW;

    //std::cout << "Vector W again: " << "\n";
    //std::cout << vectorW << "\n";
    //std::cout << "Vectorw min coeff" << "\n";
    //std::cout << vectorW.minCoeff() << "\n";

    double minCoeff = vectorW.minCoeff();

    int j = 0;

    vectorM = vectorW;

    for (int i = 0; i < 3; i++){
        if (vectorM(i) == minCoeff){
            j = i;
        }

    }

    vectorM(j) = 1;

    //std::cout << "Vector M: " << "\n";
    //std::cout << vectorM << "\n";

    vectorU = vectorW.cross(vectorM);
    vectorU = (1/vectorU.norm()) * vectorU;
    vectorV = vectorW.cross(vectorU);

    //std::cout << "Cross product into U: " << "\n";
    //std::cout << vectorU << "\n";
    //std::cout << "Cross product into V: " << "\n";
    //std::cout << vectorV << "\n";
    
    Eigen::MatrixXd vectorsGrouped(3,3);
    vectorsGrouped.row(0) = vectorU;
    vectorsGrouped.row(1) = vectorV;
    vectorsGrouped.row(2) = vectorW;

    //std::cout << "Vectors grouped is: " << "\n";
    //std::cout << vectorsGrouped << "\n";

    Eigen::MatrixXd matrixT(3,3);
    matrixT = vectorsGrouped.transpose();

    //std::cout << "Vector transposed: " << "\n";
    //std::cout << matrixT << "\n";

    //std::cout << "Sanity check" << "\n";
    //std::cout << vectorsGrouped * matrixT << "\n";

    double pi = 2*acos(0.0);

    tempMatrix << 1, 0, 0, 0,1,0,0,0,1;
    tempMatrix(0,0) = cos(std::stod(para4) * (pi/180));
    tempMatrix(1,0) = sin(std::stod(para4) * (pi/180));
    tempMatrix(0,1) = -sin(std::stod(para4) * (pi/180));
    tempMatrix(1,1) = cos(std::stod(para4) * (pi/180));
    //at this point temp matrix now holds our z boi

    //std::cout << "Temp matrix here: " << "\n";
    //std::cout << tempMatrix << "\n";

    Eigen::Matrix3d finalMatrix = Eigen::Matrix3d::Zero(3,3);

    finalMatrix = matrixT * tempMatrix * vectorsGrouped;
    

    //std::cout << "Final matrix is: " << "\n";
    //std::cout << finalMatrix << "\n";

    //bad naming, probably fix. This is the last one to become the Identity matrix.
    Eigen::Matrix4d realMatrix = Eigen::Matrix4d::Zero(4,4);

    realMatrix(0,0)=finalMatrix(0,0);
    realMatrix(0,1)=finalMatrix(0,1);
    realMatrix(0,2)=finalMatrix(0,2);

    realMatrix(1,0)=finalMatrix(1,0);
    realMatrix(1,1)=finalMatrix(1,1);
    realMatrix(1,2)=finalMatrix(1,2);

    realMatrix(2,0)=finalMatrix(2,0);
    realMatrix(2,1)=finalMatrix(2,1);
    realMatrix(2,2)=finalMatrix(2,2);
    realMatrix(3,3)=1;

    //std::cout<< "One more time: " << "\n";
    //std::cout << realMatrix << "\n";

    TMatrix.m = realMatrix * TMatrix.m;

    //std::cout << "This is the end: " << "\n";
    //std::cout << TMatrix.m << "\n";
    

    return 0;
}

int transScale(std::string para1, std::string para2, std::string para3, object3d& TMatrix){


    //std::cout << "Scale! \n";

    Eigen::Matrix4d tempMatrix = Eigen::Matrix4d::Zero(4,4);
    tempMatrix << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,1;

    //std::cout << para1 << para2 << para3 << "\n";

    //code to add change to the x, cumulative
    tempMatrix(0,0) = std::stod(para1);
    //code to add change to the y, cumulative
    tempMatrix(1,1) = std::stod(para2);
    //code to add change to teh z, cumulative
    tempMatrix(2,2) =  std::stod(para3);

    TMatrix.m = tempMatrix * TMatrix.m;

    //std::cout << "This is our scaled up bro" << "\n";
    //std::cout << TMatrix.m << "\n";

    return 0;
}

void loadMaterial(std::string filename, triangle& tri){
    std::fstream readObj;
    readObj.open(filename);

    std::string lineIn;
    std::string word;

    double holdy1;
    double holdy2;
    double holdy3;
    material newMat;
   
    

    while (getline(readObj, lineIn)){

        std::stringstream breaker(lineIn);

        while (breaker >> word){

            if (word.compare("#") == 0){
                    //std::cout << "WEEEEEEEEE" << "\n";
                    break;
            }

            if (word.compare("newmtl") == 0){
                //std::cout << "New material found" << "\n";
                (breaker >> word);
                
                newMat.materialName = word;
                                
                
                break;
            }    
               
            
                if (word.compare("Ns") == 0){
                    //std::cout << "NS___________________________" << "\n";
                   (breaker >> word);
                    newMat.Ns = std::stod(word);
               
                }
               if (word.compare("Ka") == 0){
                    //std::cout << "KAAAAAAA" << "\n";
                    (breaker >> word);
                    holdy1 = std::stod(word);
                    (breaker >> word);
                    holdy2 = std::stod(word);
                    (breaker >> word);
                    holdy3 = std::stod(word);
                    //std::cout << holdy1 << holdy2 << holdy3 << "\n";
                    newMat.Ka << holdy1, holdy2, holdy3;

                }
                if (word.compare("Kd") == 0){
                    (breaker >> word);
                    holdy1 = std::stod(word);
                    (breaker >> word);
                    holdy2 = std::stod(word);
                    (breaker >> word);
                    holdy3 = std::stod(word);


                   // std::cout << "IN KD" << "\n";
                    //std::cout << holdy1 << " " << holdy2 << "\n";

                    newMat.Kd << holdy1, holdy2, holdy3;
                    //std::cout << newMat.Kd << "\n";


                }
                if (word.compare("Ks") == 0){
                   (breaker >> word);
                    holdy1 = std::stod(word);
                    (breaker >> word);
                    holdy2 = std::stod(word);
                    (breaker >> word);
                    holdy3 = std::stod(word);
                    newMat.Ks << holdy1, holdy2, holdy3;
                
                }
                if (word.compare("Ke") == 0){
                    (breaker >> word);
                    holdy1 = std::stod(word);
                    (breaker >> word);
                    holdy2 = std::stod(word);
                    (breaker >> word);
                    holdy3 = std::stod(word);
                    newMat.Ke << holdy1, holdy2, holdy3;
                
                }
                if (word.compare("Ni") == 0){
                    (breaker >> word);
                    newMat.Ni = std::stod(word);

                }
                if (word.compare("d") == 0){
                    (breaker >> word);
                    newMat.d = std::stod(word);

                }
                if (word.compare("illum") == 0){
                    (breaker >> word);
                    newMat.illum = std::stod(word);
                    tri.materialList.push_back(newMat);

                }
                
            
            

        }

        //std::cout << "KA in processor" << "\n";
        //std::cout << newMat.Ka;

             



    }

    //tri.materialList.push_back(newMat);
    //std::cout << "Added material list" << "\n";


}

int loadObj(std::string filename, triangle& tri){
    //std::cout << "Welcome to reader" << "\n";

    //TMatrix.m(0,0) = 6;

    std::fstream readObj;
    readObj.open(filename);
    std::string lineIn;
    std::string word;

    std::vector <double> Xpoints;
    std::vector <double> Ypoints;
    std::vector <double> Zpoints;

    int vCount = 0;
    int currentF = 0;


    
    if (readObj.fail()){
        std::cout << "Failure in opening driver\n";
        return 1;
    }

    std::string currentMat;

    

    while (getline(readObj, lineIn)){

        std::stringstream breaker(lineIn);

        while (breaker >> word){

            if (word.compare("#") == 0){
                //std::cout << "WEEEEEEEEE" << "\n";
                break;
            }

            if (word.compare("v") == 0){
                //std::cout << "vector here!" << "\n";
                vCount++;
                breaker >> word;
                //std::cout << "X is: " << word << "\n";
                //std::cout << "Floated to: " << std::stod(word) << "\n";
                Xpoints.push_back(std::stod(word));
                breaker >> word;
                //std::cout << "Y is: " << word << "\n";
                Ypoints.push_back(std::stod(word));
                breaker >> word;
                //std::cout << "Z is: " << word << "\n";
                Zpoints.push_back(std::stod(word));
                break;

            }
            if (word.compare("mtllib") == 0){
                breaker >> word;
                std::cout << "Loading material" << "\n";
                loadMaterial(word, tri);
                std::cout << "post load" << "\n";
                break;
            }

            if (word.compare("usemtl") == 0){
                breaker >> word;
                currentMat = word;
                break;
            }

            if (word.compare("f") == 0){

                if (currentF == 0){
                    tri.vmap.resize(vCount+1);
                }

                face newFace;
                breaker >> word;
                tri.vmap[stod(word)].push_back(currentF);
                newFace.face1 = std::stod(word);

                breaker >> word;
                tri.vmap[stod(word)].push_back(currentF);
                newFace.face2 = std::stod(word);

                breaker >> word;
                tri.vmap[stod(word)].push_back(currentF);
                newFace.face3 = std::stod(word);
                newFace.materialName = currentMat;
                
                tri.faceList.push_back(newFace);
                
                currentF++;

                break;

            }
            
            if (word.compare("vn") == 0){
                break;
                //this code ignores any of the vector normals
            }
            else{
                word = breaker.str();
                tri.replica.notNeeded.push_back(word);
                break;
            }

            
            
            

        }


    }

    //time to make the actual temp matrix

    //std::cout << Xpoints.size() << "\n";

    tri.replica.p.resize(4, Xpoints.size());

    //std::cout << TMatrix.p << "\n";

    tri.replica.maxSize = Xpoints.size();
    std::reverse(Xpoints.begin(), Xpoints.end());
    std::reverse(Ypoints.begin(), Ypoints.end());
    std::reverse(Zpoints.begin(), Zpoints.end());

    std::reverse(tri.replica.notNeeded.begin(), tri.replica.notNeeded.end());



    for (int i = 0; i < tri.replica.maxSize; i++){
        tri.replica.p(0, i) = Xpoints.back();
        Xpoints.pop_back();
        tri.replica.p(1,i) = Ypoints.back();
        Ypoints.pop_back();
        tri.replica.p(2,i) = Zpoints.back();
        Zpoints.pop_back();
        tri.replica.p(3,i) = 1;

        //std::cout << TMatrix.p << "\n";
    }
    //std::cout << "TMATRIX " << "\n";
    //std::cout << tri.replica.p << "\n";


    std::cout << "Post readin" << "\n";

    tri.replica.endMatrix.resize(4, Xpoints.size());

    //std::cout << "Translation matrix first then math." << "\n";

    tri.replica.endMatrix = tri.replica.m * tri.replica.p;


    //std::cout << tri.replica.m << "\n";
    //std::cout << "--------------" << "\n";

    //std::cout << tri.replica.endMatrix << "\n";

    return 0;
}

int saveObj(std::string filename, object3d& TMatrix){
    TMatrix.m(0,0) = TMatrix.m(0,0);
    //std::cout << filename << "\n";

    //will need to parse through matrix and pull out numbers and record them back into whatever. 
    std::ofstream outputFile (filename);

    outputFile << TMatrix.notNeeded.back() << "\n";
    TMatrix.notNeeded.pop_back();

    outputFile << TMatrix.notNeeded.back() << "\n";
    TMatrix.notNeeded.pop_back();

    //these two will always default print the header, following format of the given ones.
    //now we strip the points out of the transformed matrix
    std::string pCoordinator;

    for (int i = 0; i < TMatrix.maxSize; i++){
        pCoordinator = "v " + std::to_string(TMatrix.endMatrix(0,i)) + " " + std::to_string(TMatrix.endMatrix(1,i)) + " " + std::to_string(TMatrix.endMatrix(2,i)) + "\n";
        outputFile << pCoordinator;

    }

    int remainderSize = TMatrix.notNeeded.size();

    for (int i = 0; i < remainderSize; i++){

        if (i + 1 == remainderSize){
            outputFile << TMatrix.notNeeded.back();
            TMatrix.notNeeded.pop_back();
        }

        else{
            outputFile << TMatrix.notNeeded.back() << "\n";
            TMatrix.notNeeded.pop_back();
        }
    }

    //debug output
    std::ofstream debugOut ("transformationmatrix.txt");

    debugOut << TMatrix.m;




    outputFile.close();

    return 0;
}


/*int main(int argc, char **argv) {

    //std::cout << "Argument parameters: " << argc << argv[0] << std::endl;
    //std::cout << "Program starting\n";

    if (argc != 2){
        std::cout << "Error: No Driver given\n";
    }
    else{
        //td::cout << "Given driver is: " << argv[1] << "\n";
        readIn(argv[1]);

    }


    return 0;
}
*/
