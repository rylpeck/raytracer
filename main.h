#ifndef MAIN_DEFINE
#define MAIN_DEFINE


class camera {

    public:
        camera(){}
        double eye [3];
        double look [3];
        double up [3];

        Eigen::Vector3d vEye;
        Eigen::Vector3d vLook;
        Eigen::Vector3d vUp;

        
        double left = 0; 
        double right = 0; 
        double bottom = 0;
        double top = 0; 
        
        double near = 0;

        double height = 0;
        double width = 0;

        Eigen::Vector3d V;
        Eigen::Vector3d U;
        Eigen::Vector3d W;

};

class threadedParam{
    public:
        int startWidth = 0;
        int endWidth = 0;
        int startHeight = 0;
        int endHeight = 0;
        std::vector< std::vector <Eigen::Vector3d>> pixels;

};

class ambient {
    public:
        double red;
        double green;
        double blue;
        Eigen::Vector3d rgbv;

};

class light {
    public:
        double xcord;
        double ycord;
        double zcord;
        Eigen::Vector3d cordv;

        double w;

        double er;
        double eg;
        double eb;
        Eigen::Vector3d ev;
};

class sphere{
    public:
        double xcord = 0;
        double ycord = 0;
        double zcord = 0;
        Eigen::Vector3d cordsv;
        double radius = 0;

        double kar = 0;
        double kag = 0;
        double kab = 0;
        Eigen::Vector3d kav = {0,0,0};

        double kdr = 0;
        double kdg = 0;
        double kdb = 0;
        Eigen::Vector3d kdv = {0,0,0};

        double ksr = 0;
        double ksg = 0;
        double ksb = 0;
        Eigen::Vector3d ksv = {0,0,0};

        double krr = 0;
        double krg = 0;
        double krb = 0;
        Eigen::Vector3d krv = {0,0,0};

        double ni = 0;

        bool isSphere = true;
};

class object3d{
    public:
        Eigen::Matrix4d m;
        Eigen::MatrixXd p;
        Eigen::MatrixXd endMatrix;
        std::vector <std::string> notNeeded;
        int maxSize;
};

class material{
    public:
        std::string materialName;
        double Ns;
        Eigen::Vector3d Ka;
        Eigen::Vector3d Kd;
        Eigen::Vector3d Ks;
        Eigen::Vector3d Ke;
        double Ni;
        double d;
        double illum;
};

class face{
    public:

        //rename these, these are verticies
        double face1;
        double face2;
        double face3;
        Eigen::Vector3d A;
        Eigen::Vector3d B;
        Eigen::Vector3d C;
        Eigen::Vector3d BA;
        Eigen::Vector3d CB;
        Eigen::Vector3d Normal;
        
        double gamma;
        double beta;

        Eigen::Vector3d AvgN;
        Eigen::Vector3d BvgN;
        Eigen::Vector3d CvgN;


        std::string materialName;
        
};

class triangle{
    public:

        object3d replica;
        std::vector <material> materialList;
        std::vector <face> faceList;
        double cutter = 0;
        std::vector <std::vector<int>> vmap;
        //a note on vmap, 0 is not a given vector, ignore it
        //vmap[x] is the vector number, the entries are the faces that have it
        
};

class model {
    public: 
        std::vector <sphere> sList;
        std::vector <triangle> tList;
        double recur;
   
};

class ColorSpot{
    public:
        Eigen::Vector3d colors;
        int hitrmiss;
};

class Ray{
    public:
        Eigen::Vector3d LV;
        Eigen::Vector3d UV; //also is DV (probably)
        double bestt ;
        sphere frontSphere;
        Eigen::Vector3d pt;
        int flag;
        int modelType = 0; //0 for sphere, 1 for not
        triangle tri;
        face bFace;
        material Mat;
        Eigen::Vector3d triNormal;
        Eigen::Vector3d shadowBestt;
        double cutoff;
        
 
        
        
};

Ray raySphereTest(Ray& ray, sphere sph);
Eigen::Vector3d PairwiseProd(Eigen::Vector3d a, Eigen::Vector3d b);
Ray multiSphere(Ray& ray, std::vector <model> slist);

ColorSpot rayTrace(Ray ray, model models, std::vector <ambient> ambientList, std::vector<light> lightList);

Ray pixelRay(int i, int j, camera cam);
int postInitialize(camera& mainCam);
void fileSaver(std::vector<std::vector<Eigen::Vector3d>>pixels, std::string saveLocation);
std::vector< std::vector <Eigen::Vector3d>> pixelMake(camera mainCam);

int readIn(std::string filename, std::string saveMe);


#endif