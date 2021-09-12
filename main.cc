#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include "main.h"
#include "matrixManip.h"
#include "fileReader.h"
#include <pthread.h>
#include <thread>

#define NUM_THREADS 8


int debug = 0;

Ray raySphereTest(Ray& ray, sphere sph){
    //std::vector <double> results;

    double radius = sph.radius;
    Eigen::Vector3d CV;
    CV = sph.cordsv;

    //ray initialization
    
    Eigen::Vector3d UV = ray.UV;
    Eigen::Vector3d TV; 
    TV = CV - ray.LV;

    //std::cout << "TV is: " << TV << "\n";
    double V = TV.dot(UV);
    double csq = TV.dot(TV);
    //std::cout << "v is: " << V << "\n";
    //std::cout << "csq is: " << csq << "\n";
    //std::cout << "radius is: " << radius << "\n";
    
    //std::cout << "radius 2d: " << pow(radius, 2) << "\n";
    double disc = pow(radius, 2) - (csq - pow(V,2));
    //std::cout << disc << "\n";
    
    if (disc < 0){
        //ray.flag =0;
        //plz
        return ray;
    }
    else {
        //std::cout << "hit" << "\n";
        double d = sqrt(disc);
        double t = V - d;
        //ray.flag = 1;
        //std::cout << "t: " << t << "\n";

        if ((ray.bestt == -1) && (t > 0.00001)){
            ray.modelType = 0;
            ray.flag = 1;
            ray.pt = ray.LV + t * UV;
            //std::cout << "ray pt: " << ray.pt << "\n";
            ray.frontSphere = sph;
            ray.bestt = t;
        }

        if ((t < ray.bestt) && (t > 0.00001)){

            //std::cout << "hey" << "\n";
            
            ray.modelType = 0;
            ray.flag = 1;
            ray.pt = ray.LV + t * UV;
            //std::cout << "ray pt: " << ray.pt << "\n";
            ray.frontSphere = sph;
            ray.bestt = t;
            
        }

       
        //std::cout << "cords: " << xcord << ycord << zcord << "\n";
        //std::cout << "t: " << t << "\n";
               

        return ray;
    }

    std::cout << "If you got here, things are very bad" << "\n";
    
    return ray;
}

Eigen::Vector3d PairwiseProd(Eigen::Vector3d a, Eigen::Vector3d b){

    Eigen::Vector3d result = {0,0,0};

    //std::cout << "pairwise " << "\n";
    
    result[0] = a[0] * b[0];
    result[1] = a[1] * b[1];
    result[2] = a[2] * b[2];

    //std::cout << a[0] << " * " << b[0] << " = " << result[0] << "\n";


    return result;

}




Ray rayTriangleTest(Ray& ray, triangle tri){



    Eigen::Matrix3d MM; 
    MM << 0,0,0,0,0,0,0,0,0;
    Eigen::Vector3d YV (0,0,0);
    Eigen::Vector3d XV (0,0,0);
    Eigen::Matrix3d MMI;
    MMI << 0,0,0,0,0,0,0,0,0;
    face facer;


    for (size_t i = 0; i < tri.faceList.size(); i++){

        facer = tri.faceList[i];
        
        YV = facer.A - ray.LV;
        
        MM.row(0) = facer.A - facer.B;
        MM.row(1) = facer.A - facer.C;
        MM.row(2) = ray.UV;
        MM.transposeInPlace();
       

        MMI = MM.inverse();

        //std::cout << "HEEEEEEEEE" << "\n";

        //std::cout << "MMI: " << "\n";
        //std::cout << MMI << "\n";

        XV = MMI * YV;

        double sbeta;
        double sgamma;
        double stval;

        sbeta = XV(0);
        sgamma = XV(1);
        stval = XV(2);

        if (sbeta == -0){
            sbeta = 0;
        }
        if (sgamma == -0){
            sgamma = 0;
        }
        if (stval == -0){
            stval = 0;
        }
        //std::cout << XV << "\n";
        //std::cout << "Yerrr " << sbeta << " " << sgamma << " " << stval << "\n";
        //std::cin.get();

        if ((sbeta > 0.0) && (sgamma > 0.0) && ((sbeta + sgamma) < 1.0) && (stval > 0.00001)){
            //hooray its a valid point
            //std::cout << "its a triangle" << "\n";
            material finder;
        
            if (ray.UV.dot(facer.Normal) > 0){
                facer.Normal = facer.Normal * -1;
            }
        
            for (size_t y = 0; y < tri.materialList.size(); y++){
                if (tri.materialList[y].materialName.compare(facer.materialName) == 0){
                    finder = tri.materialList[y];
                    //std::cout << facer.materialName << "\n";
                    //std::cout << "Material Loaded" << "\n";
                    break;
                }
            }

            if ((stval <= ray.bestt) && (stval > 0.000001)){

                ray.pt = ray.LV + stval * ray.UV;
                ray.modelType = 1;
                ray.Mat = finder;
                ray.tri = tri; 
                ray.bestt = stval;
                ray.flag++;
                ray.triNormal = facer.Normal;
                facer.gamma = sgamma;
                facer.beta = sbeta;
                ray.cutoff = tri.cutter;

                ray.bFace = facer;

            }

        }
    }

    //std::cout << "congrats" << "\n";

    return ray;
}



void multiSphere(Ray& ray, model models){

     
    ray.bestt = INFINITY;
    ray.modelType = 0;
    double shadow = 0;
    

    if (ray.flag == 2){
        //shadow looking, stop on first one then.
        shadow = 1;
    }

    ray.flag = 0;

    //std::cout << "Model size: " << models.tList.size();
    //std::cout << "Models t size is: " << models.tList.size() << "\n";
    //std::cout << "Models facelist size: " << models.tList[0].faceList.size() << "\n";
    
    for(size_t j = 0; j < models.tList.size(); j++){
        //for(size_t k = 0; k < models.tList[j].faceList.size(); k++){
            //std::cout << "Bongos" << "\n";
            ray = rayTriangleTest(ray,models.tList[j]);

            
            
        //}


    }

    for (size_t i = 0; i < models.sList.size(); i++){

        raySphereTest(ray, models.sList[i]);

            
        
    }

    if (shadow == 1){
        if (ray.flag != 0){
            ray.flag = 5;
            return;
        }

    }


    return;
}

Eigen::Vector3d VertexAverager(face facer, std::vector<int> vtm, triangle tri){
    Eigen::Vector3d avgn {0,0,0};

    Eigen::Vector3d sumN {0,0,0};
    double cnt = 0;
    double costheta = 0;
    double theta = 0;
    double pi = 2*acos(0.0);
    //std::cout << "Tri facelist size: " << tri.faceList.size() << "\n";
    //std::cout << tri.cutter << "\n";

    for (size_t i = 0; i < vtm.size(); i++){
        //std::cout <<"Break?" << "\n";
        //std::cout << vtm[i] << "\n";
        costheta = facer.Normal.dot(tri.faceList[vtm[i]].Normal);
        /*std::cout << "My n is: " << facer.Normal << "\n";
        std::cout << "cosTheta is: " << costheta << "\n";
        std::cout << "Nope" << "\n";*/
        theta = acos(costheta) * (180/pi);

        if (std::isnan(theta)){
            theta = 0;
        }
        
        if (theta <= tri.cutter){
            sumN += tri.faceList[vtm[i]].Normal;
            cnt++;
        }
        /*
        std::cout << "triangle is: " << vtm[i] << "\n";
        std::cout << "Theta is: " << theta << "Normal is" << tri.faceList[vtm[i]].Normal << "\n";
        std::cout << "SumN is: " << sumN << "\n";
        std::cout << tri.faceList[vtm[i]].face1 << " " << tri.faceList[vtm[i]].face2 << "\n";
        std::cout << "Points list: " << "\n";
        std::cout << tri.faceList[vtm[i]].A << "\n";
        std::cout << tri.faceList[vtm[i]].B << "\n";
        
        std::cout << "End of stuff" << "\n";
        
        std::cin.get();
        */
    
    }
    
    avgn = sumN / cnt;

    avgn = (1/avgn.norm()) * avgn;

    //std::cout << "Avg is: " << avgn << "\n";

    //std::cin.get();

    return avgn;

}

void makeVrtNorm(face& facer, triangle& tri){
    //possible speed down here, recommending speedup method?

    facer.AvgN = VertexAverager(facer, tri.vmap[facer.face1], tri);
    facer.BvgN = VertexAverager(facer, tri.vmap[facer.face2], tri);
    facer.CvgN = VertexAverager(facer, tri.vmap[facer.face3], tri);

    if (facer.face1 == 1 && facer.face2 == 43 && facer.face3 == 45){
        //std::cout << facer.AvgN << " " << facer.BvgN << " " << facer.CvgN << "\n";
    }

}


void triangleSetter(face& facer, triangle& tri){
        
    Eigen::Vector3d firstPoint;
    Eigen::Vector3d secondPoint;
    Eigen::Vector3d thirdPoint;
    Eigen::Vector4d cheat1;
    //std::cout << tri.replica.endMatrix << "\n";
    //std::cout << "REPLICA M" << "\n";
    //std::cout << tri.replica.m << "\n";
    cheat1 = tri.replica.endMatrix.col(facer.face1-1);
    //std::cout << facer.face1 << " " << facer.face2 << " " << facer.face3 << "\n";
   
    //std::cout << tri.replica.p << "\n";
    //std::cout << cheat1(0) << "\n";
    firstPoint(0) = cheat1(0);
    firstPoint(1) = cheat1(1);
    firstPoint(2) = cheat1(2);
    //std::cout << "Begin first point" << "\n";
    //std::cout << firstPoint << "\n";
    cheat1 = tri.replica.endMatrix.col(facer.face2-1);
    secondPoint(0) = cheat1(0);
    secondPoint(1) = cheat1(1);
    secondPoint(2) = cheat1(2);
    //std::cout << "Second point" << "\n";

    //std::cout << secondPoint << "\n";
    cheat1 = tri.replica.endMatrix.col(facer.face3-1);
    thirdPoint(0) = cheat1(0);
    thirdPoint(1) = cheat1(1);
    thirdPoint(2) = cheat1(2);
    //std::cout << "Third point" << "\n";


    facer.A = firstPoint;
    facer.B = secondPoint;
    facer.C = thirdPoint;

    facer.BA = secondPoint - firstPoint;
    facer.CB = thirdPoint - secondPoint;

    facer.Normal = facer.BA.cross(facer.CB);

    facer.Normal = (1/facer.Normal.norm()) * facer.Normal;

    if (facer.face1 == 1 && facer.face2 == 43 && facer.face3 == 999){
        std::cout << "FACED" << "\n";
        std::cout << "Normal is: " << facer.Normal << "\n";
        std::cout << "point 1: " << facer.A << "\n";
        std::cout << "point 2: " << facer.B << "\n";
        std::cout << "point 3: " << facer.C << "\n";
        std::cout << "BA: " << facer.B - facer.A << "\n";
        std::cout << "BA: " << facer.BA << "\n";
        std::cout << "CB: " << facer.CB << "\n";
        std::cout << "Trying normal: " << facer.BA.cross(facer.CB);
        //std::cin.get();

    }



    

    return;

}

void triangleSetterHandler(model& models){

    std::cout << "In handler " << "\n";
    //std::cout << "models list size" << models.tList.size() << "\n";

    for(size_t j = 0; j < models.tList.size(); j++){

        //std::cout << "pass" << "\n";
        //std::cout << "Face size of here" << models.tList[j].faceList.size() << "\n";
        //std::cin.get();

        for(size_t k = 0; k < models.tList[j].faceList.size(); k++){
            /*std::cout << "Bongos" << "\n";
            std::cout << k << "\n";
            std::cout << "goin in" << "\n";
            std::cout << models.tList[j].faceList[k].face1 << "\n";
            std::cout << models.tList[j].faceList[k].face2 << "\n";
            std::cout << models.tList[j].faceList[k].face3 << "\n";
            */

            triangleSetter(models.tList[j].faceList[k], models.tList[j]);  
                     
        }
        std::cout << "post pass" << "\n";

    }

    std::cout << "Second part" << "\n";


    for(size_t j = 0; j < models.tList.size(); j++){

        for(size_t k = 0; k < models.tList[j].faceList.size(); k++){
            //std::cout << "Bongos" << "\n";
             makeVrtNorm(models.tList[j].faceList[k], models.tList[j]); 
                     
        }

    }
   


   return;
}

Eigen::Vector3d smoothNorm(face facer){
    Eigen::Vector3d sumN;

    sumN = (1.0 - facer.beta - facer.gamma) * facer.AvgN + facer.beta * facer.BvgN + facer.gamma * facer.CvgN;
    
    sumN = 1/(sumN.norm()) * sumN;

    return sumN;

}



Eigen::Vector3d ptIllum(Ray ray, camera cam, model models, std::vector <ambient> ambientList, std::vector<light> lightList, Eigen::Vector3d refatt, Eigen::Vector3d accum){
    


    

    ColorSpot color1;
    //color1.colors << 0,0,0;
    
   

        
        //ptIllum(ray, cam, models, ambientList, lightList, refatt, accum);

        Eigen::Vector3d matKr;       
        Eigen::Vector3d C;
        Eigen::Vector3d N;
                
        if (ray.modelType == 0){
            color1.colors = PairwiseProd(ambientList[0].rgbv, ray.frontSphere.kav);
            C << ray.frontSphere.xcord, ray.frontSphere.ycord, ray.frontSphere.zcord;
            N = ray.pt - C;
            N = (1/N.norm()) * N;
            matKr = ray.frontSphere.krv;
        }

        else{
            //std::cout << "paint the town blue" << "\n";
            color1.colors = PairwiseProd(ambientList[0].rgbv, ray.Mat.Ka);
            //std::cout << ray.Mat.Ka << "\n";
            //std::cout << ray.Mat.Ka << "\n";
            //std::cout << color1.colors << "\n";
            //we need the normal instead for C
            //C = ray.triNormal;
            
            //std::cout << "Hit something" << "\n";
            //std::cout << N << "\n";
            //std::cin.get();
            if (ray.cutoff!=0){
                N = smoothNorm(ray.bFace);
            }
            else{
                N =ray.triNormal;
            }
            //N = smoothNorm(ray.bFace);
            //N = (1/N.norm()) * N;
            matKr = ray.Mat.Ks;
            
            //std::cin.get();
        }

        
        

        
        for (size_t i = 0; i < lightList.size(); i++){

            Eigen::Vector3d ptL = lightList[i].cordv;
            Eigen::Vector3d emL = lightList[i].ev;
            //std::cout << "ptl " << ptL << "\n";
            //std::cout << "eml " << emL <<"\n";
            Eigen::Vector3d toL = ptL - ray.pt;

            toL = (1/toL.norm()) * toL;
            //std::cout << "SNRM" << snrm << "\n";
            //std::cout << "TOLLLLLLLLLLLL " << toL << "\n";
            //std::cout << "dotted" << "\n";
            //std::cout << snrm.dot(toL) << "\n";
            
            double snrmtol = N.dot(toL);

            

            //shadow tester here
            Ray Shadow;
            //ray L is our point, therefore the point we chose here
            //ray Uv 
            Shadow.LV = ray.pt;
            Shadow.UV = toL;
            Shadow.flag = 2;
            Shadow.bestt = -1;
            Shadow.shadowBestt = toL;

            if (Shadow.UV.dot(ray.triNormal) < 0){
                ray.triNormal = ray.triNormal * -1;
            }
                   
            //std::cout << "shadowing" << "\n";
            Ray Legend = Shadow;
            sphere lightSource;

            lightSource.cordsv = lightList[i].cordv;
            lightSource.radius = 1;

            raySphereTest(Legend, lightSource);

            /*std::cout << "Help" << "\n";
            std::cout << Shadow.bestt << "\n";
            std::cout << Shadow.flag << "\n";
            */

            multiSphere(Shadow,models);
            //std::cout << snrmtol << "\n";
            
                   
            //if (false){
            if (Shadow.flag == 5 && (Shadow.bestt < Legend.bestt)){
                //shadow so we do nothing
                //std::cout << "shadowed " << "\n";
            }

            else{
                
            //std::cout << snrmtol << "\n";
                if (snrmtol > 0.0){
                    //std::cout << "BALOOON " << i << "\n";
                    if (ray.modelType == 0){

                        color1.colors += PairwiseProd(ray.frontSphere.kdv, emL) * N.dot(toL);
                        //std::cout << ray.frontSphere.kdv << "\n";
                    }
                    else{
                       
                        color1.colors += (snrmtol * (PairwiseProd(ray.Mat.Kd, emL)));
                        //color1.colors = {0.4, 0.1, 0.7};

                    }
                    
                    Eigen::Vector3d toc = (ray.LV - ray.pt);
                    toc = (1/toc.norm()) * toc;

                    Eigen::Vector3d spR = ((2 * snrmtol * N) - toL);
                    spR = (1/spR.norm()) * spR;

                    double cdr = toc.dot(spR);

                    cdr = cdr;
                    //std::cout << "cdr: " << cdr << "\n";

                    if (cdr > 0.0){
                        double holdy;
                        if (ray.modelType == 0){
                            holdy = pow(cdr,16);
                        }
                        else{
                            holdy = pow(cdr,ray.Mat.Ns);
                        }

                        if (ray.modelType == 0){
                            double holdyx = ray.frontSphere.ksv[0] * emL[0];
                            double holdyy = ray.frontSphere.ksv[1] * emL[1];
                            double holdyz = ray.frontSphere.ksv[2] * emL[2];

                            Eigen::Vector3d holdyv; 
                            holdyv << holdyx, holdyy, holdyz;

                            holdyv = PairwiseProd(ray.frontSphere.ksv, emL);

                            color1.colors += holdyv * holdy;
                            }
                        else{
                            Eigen::Vector3d holdyv; 

                            holdyv = PairwiseProd(ray.Mat.Ks, emL) * holdy;

                            color1.colors += holdyv * holdy;
                        }

                    }

                }


                //std::cout << "color1: " << color1.colors << "\n";

            }
        }

        //recursion
        
    accum[0] += refatt[0] * color1.colors[0];
    accum[1] += refatt[1] * color1.colors[1];
    accum[2] += refatt[2] * color1.colors[2];

        

    //color1.colors = accum;
    //return color1
    

    //color1.colors = accum;

    //color1.colors << -1,-1,-1;
    return accum;

    



}

Eigen::Vector3d refractTray(Eigen::Vector3d W, Eigen::Vector3d pt, Eigen::Vector3d N, double eta1, double eta2){
    
    Eigen::Vector3d returner = {0,0,0};

    double etar = eta1/eta2;
    double a = -etar;
    double wn = W.dot(N);
    double radsq = pow(etar,2) * (pow(wn,2) - 1) + 1;


    if (radsq < 0.0){
        //nothing its set at the start eyyy;
    }
    else {
        //std::cout << "REFRACT" << "\n";
        double b = (etar * wn) - sqrt(radsq);
        returner = (a * W) + (b * N);
        
    }

    return returner;   

}

Ray refractExit(Eigen::Vector3d W, Eigen::Vector3d pt, double etaIn, sphere Front){
    //eta out is always 1, thats the air
    Eigen::Vector3d T1;
    //std::cout << "ref test" << "\n";
    Eigen::Vector3d temp;
    Ray returner;
    returner.flag = 0;
    temp = pt - Front.cordsv;
    temp = (1/temp.norm()) * temp;

    T1 = refractTray(W, pt, temp,1.0, etaIn);

    if (T1.sum() == 0){
        returner.flag = -1;
        return returner;
    }
    else {
        //std::cout << "refrract me" << "\n";
        Eigen::Vector3d exit {0,0,0};
        exit = pt + 2 * (Front.cordsv - pt).dot(T1) * T1;
        temp = Front.cordsv - exit;
        temp = (1/temp.norm()) * temp;

        T1 = refractTray(-T1, exit, temp, etaIn, 1.0);
        
        if (T1.sum() != 0){
            //std::cout << "SCREEEEEEEEEEEEEEEEEEEEEEEEE" << "\n";
        }

        returner.LV = exit;
        returner.UV = T1;

    }

    return returner;
}

Eigen::Vector3d rayTrace(Ray ray, camera cam, model models, std::vector <ambient> ambientList, std::vector<light> lightList, Eigen::Vector3d refatt, Eigen::Vector3d accum){

    Eigen::Vector3d matKr;       
    Eigen::Vector3d C;
    Eigen::Vector3d N;
    ColorSpot color1;

    multiSphere(ray,models);
    //std::cout << "we return" << "\n";
    //std::cout << ray.flag << "\n";

    if (ray.flag >= 1){

                
        if (ray.modelType == 0){
            //color1.colors = PairwiseProd(ambientList[0].rgbv, ray.frontSphere.kav);
            C << ray.frontSphere.xcord, ray.frontSphere.ycord, ray.frontSphere.zcord;
            N = ray.pt - C;
            N = (1/N.norm()) * N;
            matKr = ray.frontSphere.krv;

        }
        else{
            //color1.colors = PairwiseProd(ambientList[0].rgbv, ray.Mat.Ka);
            //C = ray.triNormal;
            //N = (1/N.norm()) * N;

            if (ray.cutoff!=0){
                N = smoothNorm(ray.bFace);
                
            }
            else{
                N = ray.bFace.Normal;
                //N = (1/N.norm()) * N;
            }
            matKr = ray.Mat.Ks;
        }


        accum = ptIllum(ray, cam, models, ambientList, lightList, refatt, accum);

        int skippo = 0;

        if (ray.modelType != 0){
            if (ray.Mat.illum != 3){
                skippo = 1;
            }

        }
            
        if (skippo == 0){
        Eigen::Vector3d Uinv (0,0,0);
        Eigen::Vector3d refR (0,0,0);
        //reflection
        if (models.recur > 0){

            Uinv = -1 * ray.UV;

            refR = ((2 * N.dot(Uinv) * N) - Uinv);

            refR = (1/refR.norm()) * refR;

            Eigen::Vector3d flec (0,0,0);
            models.recur--;
            Ray recurve; 
            recurve.LV = ray.pt;
            recurve.UV = refR;

            flec = rayTrace(recurve, cam, models, ambientList, lightList, PairwiseProd(matKr, refatt), flec);
            
            accum[0] += refatt[0] * flec[0];
            accum[1] += refatt[1] * flec[1];
            accum[2] += refatt[2] * flec[2];

            


            //color1.colors = accum;
        }
        //refraction
        if (ray.modelType == 0){
            if ((models.recur > 0) && ray.frontSphere.ni != 0){
                //std::cout << "Refraction" << "\n";

                Eigen::Vector3d Thru (0,0,0);
                Ray fraR;

                fraR = refractExit(-ray.UV, ray.pt, ray.frontSphere.ni, ray.frontSphere);

                if (fraR.flag != -1){

                    //std::cout << "this should change color plz" << "\n";
                    models.recur--;
                    Eigen::Vector3d tempy (0,0,0);
                    tempy[0] = 1 - matKr[0];
                    tempy[1] = 1 - matKr[1];
                    tempy[2] = 1 - matKr[2];
  
                    Thru = rayTrace(fraR,cam,models,ambientList,lightList, PairwiseProd(tempy, refatt), Thru);
                    
                    //std::cout << Thru << "\n";

                    

                    accum[0] += (refatt[0] * (1 - ray.frontSphere.krv[0]) * Thru[0]);
                    accum[1] += (refatt[1] * (1 - ray.frontSphere.krv[1]) * Thru[1]);
                    accum[2] += (refatt[2] * (1 - ray.frontSphere.krv[2]) * Thru[2]);

                   
                    //std::cout << accum << "\n";


                }



            }
        }
        }
            
    }

    return accum;



}

Ray pixelRay(int i, int j, camera cam){
    Ray ray;
    ray.LV << 0,0,0;
    ray.UV << 0,0,0;

    double holdy = cam.width-1;
    double holdy2 = cam.right - cam.left;
    double holdy3 = i/holdy;
   
      
    double px = holdy3 * holdy2 + cam.left;
    
    //double py = (j/(cam.height-1)*(cam.vmax - cam.vmin) + cam.vmin);

    holdy = cam.height-1;
    holdy2 = cam.bottom - cam.top;
    holdy3 = j/holdy;

    double py = holdy3 * holdy2 + cam.top;

    //std::cout << i << " " << cam.width << " " << cam.right << " " << cam.left << "\n";
    //std::cout << j << " " << cam.height << " " << cam.bottom << " " << cam.top << "\n";

    //std::cout << "px: " << px << "\n";
    //std::cout << "py: " << py << "\n";

    ray.LV =cam.vEye+(cam.near*cam.W) + (px * cam.U) + (py * cam.V);
    

    ray.UV = ray.LV - cam.vEye;
    
    ray.UV = (1/ray.UV.norm())*ray.UV;

    //std::cout << "UV is: " << ray.UV << "\n";
    //std::cout << "LV is: " << ray.LV << "\n";

    return ray;
}



int postInitialize(camera& mainCam){

    //first we vectorize!
    mainCam.vEye << mainCam.eye[0], mainCam.eye[1], mainCam.eye[2];
    mainCam.vLook << mainCam.look[0], mainCam.look[1], mainCam.look[2];
    mainCam.vUp << mainCam.up[0], mainCam.up[1], mainCam.up[2];

    mainCam.W = ((mainCam.vEye - mainCam.vLook));
    mainCam.W = (1/mainCam.W.norm()) * mainCam.W;

    mainCam.U = mainCam.vUp.cross(mainCam.U);
    mainCam.U = (1/mainCam.V.norm()) * mainCam.U;

    mainCam.V = mainCam.W.cross(mainCam.U);


    return 0;
}

void fileSaver(std::vector<std::vector<Eigen::Vector3d>>pixels, std::string saveLocation){
    std::ofstream outputFile (saveLocation);
    //std::cout << "Begin print" << "\n";
    
    outputFile << "P3\n";
    //std::cout << pixels.size();
    
    outputFile << pixels[0].size() << " " << pixels.size() << " " << "255\n";

    //std::cout << "HEader made" << "\n";

    int red;
    int green;
    int blue;
    

    for (size_t i = 0; i < pixels[0].size(); i++){

        for (size_t j = 0; j < pixels.size(); j++){
            //outputFile << 0; 
            red = 255 * pixels[j][i][0];
            green = 255 * pixels[j][i][1];
            blue = 255 * pixels[j][i][2];
            outputFile << red << " " << green << " " << blue << " ";
                      
        }

        outputFile << "\n";
    }

   
    outputFile.close();


    
}

std::vector< std::vector <Eigen::Vector3d>> pixelMake(camera mainCam){
    
      

    std::vector< std::vector <Eigen::Vector3d>> pixels;

    Eigen::Vector3d Clearer = {0,0,0};

    pixels.resize(mainCam.width);

    for (int i = 0; i < mainCam.width; i++){
        pixels[i].resize(mainCam.width);

        for (int j = 0; j < mainCam.height; j++){
            pixels[i][j] = Clearer;
        }
    }

    if (debug == 1){
        std::cout << "Vector of pixels made: " << "\n";
        std::cout << "Width: " << pixels.size() << "\n";
        std::cout << "Height: " << pixels[0].size() << "\n";
        std::cout << "Cell size: " << pixels[0][0].size() << "\n";
    }

   
    return pixels;
}

void rayTracer (camera cam, std::vector<ambient> aList, std::vector<light> lList, model models, threadedParam& params){
    //we have aList - ambientList
    //lList - Light list
    //slist - Spherelist
    //the camera
           
    
    ColorSpot returner;

    //std::cout << "hi" << "\n";
    
    if (debug == 1){
        std::cout << "Camera parts:" << "\n";
        std::cout << "Eye " << cam.eye[0] << " " << cam.eye[1] << " " << cam.eye[2] << "\n";
        std::cout << "Look " << cam.look[0] << " " << cam.look[1]<< " " << cam.look[2] << "\n";
        std::cout << "Up " << cam.up[0] << " " << cam.up[1] << " " << cam.up[2] << "\n";
        std::cout << "--------------" << "\n";
        std::cout << "umin: " << cam.left << "\n";
        std::cout << "umax: " << cam.right << "\n";
        std::cout << "vmin: " << cam.bottom << "\n";
        std::cout << "vmax: " << cam.top << "\n";
        std::cout << "--------------" << "\n";
        std::cout << "near: " << cam.near << "\n";

        std::cout << "Number of ambient lights set" << "\n";
        std::cout << aList.size() << "\n";
        std::cout << aList[0].rgbv << "\n";
        std::cout << aList[0].red << "\n";
        std::cout << "Number of Diffuse lights set" << "\n";
        std::cout << lList.size() << "\n";
        std::cout << "Number of Spheres created" << "\n";
        std::cout << models.sList.size() << "\n";
       
    }

    //quick sort to align all the stuffs.

    std::vector <light> sorted; // this will have max first so it does back forward on the rendering.

    //for (size_t k = 0; k < lList.size(); k++){
    Eigen::Vector3d refatt;
    Eigen::Vector3d accum;
    
    refatt << 1,1,1;
    accum << 0,0,0;
    int recursionHolder = models.recur;
    Eigen::Vector3d returned;


        for (int i = params.startWidth; i < params.endWidth; i++){
        
            for (int j = params.startHeight; j < params.endHeight; j++){
                
                if (i == params.endWidth/2){
                    if (j == params.endHeight/2){
                        std::cout << "This thread is Halfway" << "\n";
                    }
                }
                //std::cout << "crusade" << "\n";
                models.recur = recursionHolder;
                returned = rayTrace(pixelRay(i,j,cam),cam ,models, aList, lList, refatt, accum);
                //std::cout << returner.colors;
                //std::cout << "Good morning" << i << " " << j << "\n";
                //std::cout << "Colorizer" << "\n";
                //std::cout << returner.colors;
                params.pixels[i][j] += returned;

                if (returned[0] == -1){
                    params.pixels[i][j] = {0,0,0};
                }
                
                //std::cout << "Make bacon" << "\n";
            }
        }
    //}
    std::cout << "post bulk tracing" << "\n";
    
   
    //time to concotenate them

    return;
}


std::vector <std::vector <Eigen::Vector3d>> multiThreadTracer(camera cam, std::vector<ambient> aList, std::vector<light> lList, model models){
    
        //std is working with 4 threads
    
    triangleSetterHandler(models);
    if (NUM_THREADS == 1){
        std::vector< std::vector <Eigen::Vector3d>> pixelFull = pixelMake(cam);
        threadedParam params1;

        params1.startWidth = 0;
        params1.startHeight = 0;
        params1.endWidth = cam.width;
        params1.endHeight = cam.height;
        params1.pixels = pixelFull;

        std::thread t1(rayTracer, cam, aList, lList, models, std::ref(params1));
        t1.join();

        return params1.pixels;


    }

    if (NUM_THREADS == 4){

        std::vector< std::vector <Eigen::Vector3d>> pixelFull = pixelMake(cam);

        std::cout << "Multi threading 4" << "\n";
        //will divide the resolution by 4;
        int widthMid = cam.width/2;
        int heightMid = cam.height/2;

        threadedParam params1;
        threadedParam params2;
        threadedParam params3;
        threadedParam params4;

        params1.startWidth = 0;
        params1.startHeight = 0;
        params1.endWidth = widthMid;
        params1.endHeight = heightMid;
        params1.pixels = pixelFull;

        params2.startWidth = widthMid;
        params2.startHeight = 0;
        params2.endWidth = cam.width;
        params2.endHeight = heightMid;
        params2.pixels = pixelFull;

        params3.startWidth = 0;
        params3.startHeight = heightMid;
        params3.endWidth = widthMid;
        params3.endHeight = cam.height;
        params3.pixels = pixelFull;

        params4.startWidth = widthMid;
        params4.startHeight = heightMid;
        params4.endWidth = cam.width;
        params4.endHeight = cam.height;
        params4.pixels = pixelFull;

        std::thread t1(rayTracer, cam, aList, lList, models, std::ref(params1));
        std::thread t2(rayTracer, cam, aList, lList, models, std::ref(params2));
        std::thread t3(rayTracer, cam, aList, lList, models, std::ref(params3));
        std::thread t4(rayTracer, cam, aList, lList, models, std::ref(params4));
        //std::thread t1(rayTracer, cam, aList, lList, models, params);
        t1.join();
        t2.join();
        t3.join();
        t4.join();

        //the master joiner

        for (int i = 0; i < cam.width; i++){
           
            for (int j = 0; j < cam.height; j++){
                if (i <= widthMid){
                    if (j < heightMid){
                        pixelFull[i][j] = params1.pixels[i][j];
                    }
                    if (j >= heightMid){
                        pixelFull[i][j] = params3.pixels[i][j];
                    }
                }
                if (i >= widthMid){
                    if (j < heightMid){
                        pixelFull[i][j] = params2.pixels[i][j];
                    }
                    if (j >= heightMid){
                        pixelFull[i][j] = params4.pixels[i][j];
                    }
                }

                
            }
        }
        return pixelFull;
        //we will have to accumulate them all;
    }

    else if (NUM_THREADS == 8){

        std::vector< std::vector <Eigen::Vector3d>> pixelFull = pixelMake(cam);

        std::cout << "Multi threading 8 woot" << "\n";

               
        int widthMid = cam.width/2;
        int heightMid = cam.height/2;

        int q1width = widthMid/2;
        int q3width = widthMid + q1width;
       

        threadedParam params1;
        threadedParam params2;
        threadedParam params3;
        threadedParam params4;

        threadedParam params5;
        threadedParam params6;
        threadedParam params7;
        threadedParam params8;

        std::cout << "Full size is: " << cam.width << " " << cam.height << "\n";

        params1.startWidth = 0;
        params1.startHeight = 0;

        params1.endWidth = q1width;
        params1.endHeight = heightMid;

        params1.pixels = pixelFull;

        std::cout << "Width min and max: " << params1.startWidth << " "<< params1.endWidth << "\n";
        std::cout << "Height min and max: " << params1.startHeight << " " << params1.endHeight << "\n";
    

        params2.startWidth = q1width;
        params2.startHeight = 0;

        params2.endWidth = widthMid;
        params2.endHeight = heightMid;

        params2.pixels = pixelFull;

        std::cout << "Width min and max: " << params2.startWidth << " "<< params2.endWidth << "\n";
        std::cout << "Height min and max: " << params2.startHeight << " " << params2.endHeight << "\n";
    

        params3.startWidth = widthMid;
        params3.startHeight = 0;

        params3.endWidth = q3width;
        params3.endHeight = heightMid;

        params3.pixels = pixelFull;

        std::cout << "Width min and max: " << params3.startWidth << " "<< params3.endWidth << "\n";
        std::cout << "Height min and max: " << params3.startHeight << " " << params3.endHeight << "\n";
    

        params4.startWidth = q3width;
        params4.startHeight = 0;
        params4.endWidth = cam.width;
        params4.endHeight = heightMid;
        params4.pixels = pixelFull;

        std::cout << "Width min and max: " << params4.startWidth << " "<< params4.endWidth << "\n";
        std::cout << "Height min and max: " << params4.startHeight << " " << params4.endHeight << "\n";
    

        //bottom half of image

        params5.startWidth = 0;
        params5.startHeight = heightMid;
        params5.endWidth = q1width;
        params5.endHeight = cam.height;
        params5.pixels = pixelFull;

        params6.startWidth = q1width;
        params6.startHeight = heightMid;
        params6.endWidth = widthMid;
        params6.endHeight = cam.height;
        params6.pixels = pixelFull;

        params7.startWidth = widthMid;
        params7.startHeight = heightMid;
        params7.endWidth = q3width;
        params7.endHeight = cam.height;
        params7.pixels = pixelFull;

        params8.startWidth = q3width;
        params8.startHeight = heightMid;
        params8.endWidth = cam.width;
        params8.endHeight = cam.height;
        params8.pixels = pixelFull;


        std::thread t1(rayTracer, cam, aList, lList, models, std::ref(params1));
        std::thread t2(rayTracer, cam, aList, lList, models, std::ref(params2));
        std::thread t3(rayTracer, cam, aList, lList, models, std::ref(params3));
        std::thread t4(rayTracer, cam, aList, lList, models, std::ref(params4));

        std::thread t5(rayTracer, cam, aList, lList, models, std::ref(params5));
        std::thread t6(rayTracer, cam, aList, lList, models, std::ref(params6));
        std::thread t7(rayTracer, cam, aList, lList, models, std::ref(params7));
        std::thread t8(rayTracer, cam, aList, lList, models, std::ref(params8));

        //std::thread t1(rayTracer, cam, aList, lList, models, params);
        t1.join();
        t2.join();
        t3.join();
        t4.join();
        t5.join();
        t6.join();
        t7.join();
        t8.join();

        int sec1 = 0;
        int sec2 = 0;
        int sec3 = 0;
        int sec4 = 0;
        int sec5 = 0;
        int sec6 = 0;
        int sec7 = 0;
        int sec8 = 0;


        //the master joiner
        std::cout << cam.width << " " << cam.height << "\n";

        for (int i = 0; i < cam.width; i++){
           
            for (int j = 0; j < cam.height; j++){

           
                  
                
                             
               
                if (j < heightMid){

                    if (i < q1width){
                        pixelFull[i][j] = params1.pixels[i][j];
                        
                        sec1++;

                    }
                    else if (i >= q1width && i < widthMid){
                        
                        pixelFull[i][j] = params2.pixels[i][j];
                        
                        sec2++;

                    }
                    else if (i < q3width && i >= widthMid) {
                        pixelFull[i][j] = params3.pixels[i][j];
                        sec3++;
                    }
                    else if (i >= q3width){
                        pixelFull[i][j] = params4.pixels[i][j];
                        sec4++;
                    }
                }

                else if (j >= heightMid){

                    if (i < q1width){
                        pixelFull[i][j] = params5.pixels[i][j];
                        sec5++;
                    }
                    else if (i >= q1width && i < widthMid){
                        pixelFull[i][j] = params6.pixels[i][j];
                        sec6++;
                    }

                    else if (i < q3width && i >= widthMid){
                        pixelFull[i][j] = params7.pixels[i][j];
                        sec7+=2;
                    }
                    else if (i >= q3width){
                        pixelFull[i][j] = params8.pixels[i][j];
                        sec8++;
                    }
                }
                
                
            }
        }
  

        std::cout << sec1 << " " << sec2 << " " << sec3 << " " << sec4 << "\n";
        std::cout << sec5 << " " << sec6 << " " << sec7 << " " << sec8 << "\n";
        
        return pixelFull;

    }

    
    std::cout << "No threading given returning dead" << "\n";

    std::vector< std::vector <Eigen::Vector3d>> pixelFull = pixelMake(cam);

    return pixelFull;
}




int readIn(std::string filename, std::string saveMe){
    
    camera maincamera;
    model models;
    std::vector <ambient> ambientList;
    std::vector<light> lightList;

    
    
    
    std::fstream inputFile;
    inputFile.open(filename);
    std::string lineIn;
    std::string word;

    

    if (inputFile.fail()){
        std::cout << "Failure in opening driver\n";
        return 1;
    }
    std::cout << "hi" << "\n";
    fullReadin(filename, saveMe, maincamera, ambientList, lightList, models);
    std::cout << "post readin" << "\n";
    /*
    std::cout << "post readin" << "\n";
    std::cout << "triangle list" << models.tList.size() << "\n";
    std::cout << "size of mat list: " << models.tList[0].materialList.size() << "\n";
    std::cout << models.tList[0].faceList[0].face1 << "\n";
    std::cout << models.tList[0].faceList[0].face2 << "\n";
    std::cout << models.tList[0].faceList[0].face3 << "\n";
    std::cout << models.tList[0].faceList[0].materialName << "\n";
    std::cout << "MATERIAL NAME: " << "\n";

    std::cout << models.tList[0].materialList[0].materialName << "\n";
    //std::cout << models.tList[0].materialList[1].materialName << "\n";

    std::cout << models.tList[0].materialList[0].Kd << "\n";
    std::cout << models.tList[0].materialList[0].Ka << "\n";
    std::cout << models.tList[0].materialList[0].Ks << "\n";


    std::cin.get();
    */
    
    
    
    
    //while loop gathers by line then we process whats inside of it
   

    if (debug == 1){
        std::cout << "Camera parts:" << "\n";
        std::cout << "Eye " << maincamera.eye[0] << " " << maincamera.eye[1] << " " << maincamera.eye[2] << "\n";
        std::cout << "Look " << maincamera.look[0] << " " << maincamera.look[1]<< " " << maincamera.look[2] << "\n";
        std::cout << "Up " << maincamera.up[0] << " " << maincamera.up[1] << " " << maincamera.up[2] << "\n";
        std::cout << "--------------" << "\n";
        std::cout << "left: " << maincamera.left << "\n";
        std::cout << "right: " << maincamera.right << "\n";
        std::cout << "bottom: " << maincamera.bottom << "\n";
        std::cout << "top: " << maincamera.top << "\n";
        std::cout << "--------------" << "\n";
        std::cout << "near: " << maincamera.near << "\n";
        std::cout << "height: " << maincamera.height << "\n";
        std::cout << "width: " << maincamera.width << "\n";

        std::cout << "Number of ambient lights set" << "\n";
        std::cout << ambientList.size() << "\n";
        std::cout << "Number of Diffuse lights set" << "\n";
        std::cout << lightList.size() << "\n";
        std::cout << "Number of Spheres created" << "\n";
        std::cout << models.sList.size() << "\n";
       
    }
    std::vector <std::vector<Eigen::Vector3d>> pixels;

    std::cout << "HI pre thread" << "\n";

    pixels = multiThreadTracer(maincamera, ambientList, lightList, models);

    std::cout << "HI" << "\n";

    fileSaver(pixels, saveMe);

    //std::cout << "HI" << "\n";


    inputFile.close();
    return 0;
}

int main(int argc, char **argv) {

    //std::cout << "Argument parameters: " << argc << argv[0] << std::endl;
    //std::cout << "Program starting\n";
    //first one is read in
    //second one is our save to!
    if (argc != 3){
        std::cout << "Error: not enough Drivers given\n";
    }
    else{
        //td::cout << "Given driver is: " << argv[1] << "\n";
        readIn(argv[1], argv[2]);

    }
    //std::cout << "hi" << "\n";

    return 0;
}
