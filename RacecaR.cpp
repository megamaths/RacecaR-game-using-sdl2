#include <iostream>
#include <SDL2/SDL.h>
#include <cmath>
#include <vector>
#include <array>
#include <sys/time.h>
#include <fstream>
#define _USE_MATH_DEFINES

const int dispwidth = 1080;
const int dispheight = 720;
const int dispdist = 1024;

const double camspeed = 16;
const double camrotspeed = 0.025;

const bool fullzbuffer = false; // decide if use z buffer (slow but reliable ) or just draw further polygons first (fast may cause cliping)

double lightpos[3] = {128, -1024 , 0};
double lightshade[3] = {0.8,0.6,0.6};

long long lastframetime;



//The window we'll be rendering toSS
SDL_Window* window = NULL;

//The surface contained by the window
SDL_Surface* screenSurface = NULL;

//The window renderer
SDL_Renderer* renderer = NULL;

struct point{
    double pos[3];
};

struct hyperpoint{
    std::array<double,4> pos;
};


double * multiquaternion(double quat1[4],double quat2[4]){
    double a = quat1[0];
    double b = quat1[1];
    double c = quat1[2];
    double d = quat1[3];
    double e = quat2[0];
    double f = quat2[1];
    double g = quat2[2];
    double h = quat2[3];

    static double q[4];


    q[0] = a*e - b*f - c*g - d*h;
    q[1] = b*e + a*f + c*h - d*g;
    q[2] = a*g - b*h + c*e + d*f;
    q[3] = a*h + b*g - c*f + d*e;

    return q;
}


void fillpoly(std::vector<point> polygon){
    int miny = dispheight;
    int maxy = 0;
    for (int i = 0;i < polygon.size();i++){
        if (polygon[i].pos[1] < miny){
            miny = polygon[i].pos[1];
        }
        if (polygon[i].pos[1] > maxy){
            maxy = polygon[i].pos[1];
        }
    }

    for (int i = miny;i < maxy;i++){
        std::vector<int> intersections;

        
        for (int j = 0; j < polygon.size();j++){
            int otherpoint = (j+1)%polygon.size();
            if ((polygon[j].pos[1] < i && polygon[otherpoint].pos[1] > i) 
                || (polygon[j].pos[1] > i && polygon[otherpoint].pos[1] < i)){// if there is an intersection

                if (abs(polygon[j].pos[1]-polygon[otherpoint].pos[1]) < 1){// almost straight up/ down
                    intersections.push_back(polygon[j].pos[0]);
                }
                else{
                    double m = (polygon[j].pos[0]-polygon[otherpoint].pos[0])/(polygon[j].pos[1]-polygon[otherpoint].pos[1]);
                    double intersectpoint = polygon[j].pos[0]+m*(i-polygon[j].pos[1]);
                    intersections.push_back(intersectpoint);
                }
                
            }
        }

        
        bool parity = false;
        int intersectionssize = intersections.size();
        int currentpoint = -dispwidth/2-25;
        if (intersectionssize > 1){
            for (int j = 0; j < intersections.size();j++){
                int nextpoint = dispwidth/2+25;
                for (int k = 0; k < intersections.size();k++){
                    if (intersections[k] < nextpoint && intersections[k] > currentpoint){
                        nextpoint = intersections[k];
                    }
                    
                }
                if (parity && nextpoint != dispwidth/2+25){// && j+1 != intersections.size()){
                    int rightpoint = nextpoint;
                    int leftpoint = currentpoint;
                    
                    SDL_RenderDrawLine(renderer,leftpoint+dispwidth/2,i+dispheight/2,rightpoint+dispwidth/2,i+dispheight/2);
                }

                currentpoint = nextpoint;
                parity = !parity;

            }
        }
    }

}




bool isinpoly(std::vector<point> polygon, point querypoint){

    bool firstside;
    bool onedge = true;
    for (int i = 0;i < polygon.size();i++){
        int j = (i+1) %polygon.size();

        double side = ((polygon[j].pos[0] - polygon[i].pos[0]) * (querypoint.pos[2] - polygon[i].pos[2]) - 
                        (polygon[j].pos[2] - polygon[i].pos[2]) * (querypoint.pos[0] - polygon[i].pos[0]));


        //std::cout << polygon[i].pos[0] << " " << polygon[i].pos[2] << " point1\n";
        //std::cout << polygon[j].pos[0] << " " << polygon[j].pos[2] << " point2\n";
        //std::cout << querypoint.pos[0] << " " << querypoint.pos[2] << " pointq\n";
        //std::cout << side << "\n";
        if (i == 0){
            if (side == 0){
                firstside = (side > 0);
            }
            else{
                onedge = false;
                firstside = (side > 0);
            }
        }
        else if (onedge){
                if (side == 0){
                    firstside = (side > 0);
                }
                else{
                    onedge = false;
                    firstside = (side > 0);
                }
            }
        else if ((side > 0) != firstside){
            if (side != 0){
                return false;
            }
        }
    }
    
    return true;
}

struct Bufferpolygon{// what is stored in the buffer
    std::vector<point> points;
    double distance;
    int colour[3];
};


class buffer{// a z buffer for giving polygons
    public:
        //double* zvals;
        std::vector<double> zvals;
        double a;// used for geting depth of current polygon
        double b;
        double c;
        double d;

        std::vector<Bufferpolygon> polygonbuffer;


        void newframe(){
            zvals.clear();
            zvals.resize(dispheight * dispwidth);
        }

        void getequation(std::vector<point> polygon){
            int polylen = polygon.size();

            //use real space points as this must not get warped
            //std::cout << "CAMERA equation \n";
            point realpoints[polygon.size()];
            for (int i = 0; i < polygon.size(); i++){
                realpoints[i].pos[0] = polygon[i].pos[0]*polygon[i].pos[2]/dispdist;
                realpoints[i].pos[1] = polygon[i].pos[1]*polygon[i].pos[2]/dispdist;
                realpoints[i].pos[2] = polygon[i].pos[2];
                //std::cout << "CAMERA equation " << realpoints[i].pos[0] << " " << realpoints[i].pos[1] << " " << realpoints[i].pos[2] << "\n";
            }

            double normal[3];
            double vect1[3];
            double vect2[3];
            for (int i = 0;i < 3;i++){
                vect1[i] = (realpoints[1].pos[i]
                        -realpoints[0].pos[i]); // get the diff between 0 and 1 points as vect
                vect2[i] = (realpoints[2].pos[i]
                        -realpoints[0].pos[i]); // get the diff between 0 and 2 points as vect
            }
            normal[0] = vect1[1]*vect2[2]-vect1[2]*vect2[1];
            normal[1] = vect1[2]*vect2[0]-vect1[0]*vect2[2];
            normal[2] = vect1[0]*vect2[1]-vect1[1]*vect2[0];

            double normlen = sqrt(normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2]);

            a = normal[0] / normlen;
            b = normal[1] / normlen;
            c = normal[2] / normlen;
            d = -(a*realpoints[0].pos[0]+b*realpoints[0].pos[1]+c*realpoints[0].pos[2]);

            //std::cout << "CAMERA equation " << a << " " << b << " " << c << " " << d << "\n";

            /*for (int i = 0; i < polygon.size(); i++){
                std::cout << "CAMERA equation " << getzat(polygon[i].pos[0],polygon[i].pos[1]) << " " << polygon[i].pos[2] << "\n";
            }*/
            //ax+by+cz+d = 0????

        }

        double getzat(double x,double y){
            /*//ax+by+cz+d = 0????
            //cz = -d-a*x-b*y;
            double z = (-d-(a*x)-(b*y))/c;*/ // screen space version

            //std::cout << "CAMERA equation" << a << " " << b << " " << c << " " << d << "\n";

            //ax+by+cz+d = 0 plane
            //x = r + qu  y = s + qv  z = t + qw line         r s and t are all 0 as start at origin
            //x = qu  y = qv  z = qw                    we are working out what q is so we can find qw which is z value

            // as z is const and x and y is given u is given x   v is given y    and z is dispdist    this gives vector from origin to point on screen

            // so substitute things in q*a*(given x) + q*b*(given y) + q*c*(dispdist) + d = 0
            //factor out q             q*(a*(given x) + b*(given y) + c*(dispdist)) + d = 0
            //move d and divide by bracket     q = -d/(a*(given x) + b*(given y) + c*(dispdist))
            //substitute q into z = qw to give     z = -d/(a*(given x) + b*(given y) + c*(dispdist))*dispdist


            double z = -d/(a*(x) + b*(y) + c*(dispdist))*dispdist;
            return z;
        }


        void fillpolybuffered(std::vector<point> polygon){
            getequation(polygon);
            int miny = dispheight;
            int maxy = 0;
            int minx = dispwidth;
            int maxx = 0;
            for (int i = 0;i < polygon.size();i++){
                if (polygon[i].pos[1] < miny){
                    miny = polygon[i].pos[1];
                }
                if (polygon[i].pos[1] > maxy){
                    maxy = polygon[i].pos[1];
                }
                if (polygon[i].pos[0] < minx){
                    minx = polygon[i].pos[0];
                }
                if (polygon[i].pos[0] > maxx){
                    maxx = polygon[i].pos[0];
                }
            }

            for (int i = miny;i < maxy;i++){
                std::vector<int> intersections;

                
                for (int j = 0; j < polygon.size();j++){
                    int otherpoint = (j+1)%polygon.size();
                    if ((polygon[j].pos[1] < i && polygon[otherpoint].pos[1] > i) 
                        || (polygon[j].pos[1] > i && polygon[otherpoint].pos[1] < i)){// if there is an intersection

                        if (abs(polygon[j].pos[1]-polygon[otherpoint].pos[1]) < 1){// almost straight up/ down
                            intersections.push_back(polygon[j].pos[0]);
                        }
                        else{
                            double m = (polygon[j].pos[0]-polygon[otherpoint].pos[0])/(polygon[j].pos[1]-polygon[otherpoint].pos[1]);
                            double intersectpoint = polygon[j].pos[0]+m*(i-polygon[j].pos[1]);
                            intersections.push_back(intersectpoint);
                        }
                        
                    }
                }

                
                bool parity = false;
                int intersectionssize = intersections.size();
                int currentpoint = -dispwidth/2-25;
                if (intersectionssize > 1){
                    for (int j = 0; j < intersections.size();j++){
                        int nextpoint = dispwidth/2+25;
                        for (int k = 0; k < intersections.size();k++){
                            if (intersections[k] < nextpoint && intersections[k] > currentpoint){
                                nextpoint = intersections[k];
                            }
                            
                        }
                        if (parity && nextpoint != dispwidth/2+25){// && j+1 != intersections.size()){
                            int rightpoint = nextpoint;
                            int leftpoint = currentpoint;
                            for (int k = leftpoint;k <= rightpoint; k++){
                                double pixeldepth = getzat(k,i);
                                double currentval = zvals[(i+dispheight/2)*dispwidth + k+dispwidth/2];
                                if (pixeldepth < currentval || currentval == 0){
                                    //SDL_SetRenderDrawColor(renderer , (int)pixeldepth%256 , pixeldepth , pixeldepth ,255);
                                    zvals[(i+dispheight/2)*dispwidth + k+dispwidth/2] = pixeldepth;
                                    SDL_RenderDrawPoint(renderer, k+dispwidth/2,i+dispheight/2);
                                }
                            }
                            //SDL_RenderDrawLine(renderer,leftpoint+dispwidth/2,i+dispheight/2,rightpoint+dispwidth/2,i+dispheight/2);
                        }

                        currentpoint = nextpoint;
                        parity = !parity;

                    }
                }
            }
        }

        bool ispointhidden(double x ,double y, double z){ //returns if a point is hidden for less important triangles works with the fill poly buffer function
            if (zvals[(x+dispheight/2)*dispwidth + y+dispwidth/2] < z){
                return false;
            }
            return true;

        }

        void simplebuffer(std::vector<point> polygon , int r , int g , int b){ //adds polygon to a list incomplatible to fill poly buffer
            Bufferpolygon newpolygon;
            newpolygon.points = polygon;
            double distz = 0;
            for (int i = 0; i < polygon.size(); i++){
                if (sqrt(polygon[i].pos[0]*polygon[i].pos[0]+polygon[i].pos[1]*polygon[i].pos[1]+polygon[i].pos[2]*polygon[i].pos[2]) > distz){
                    distz = sqrt(polygon[i].pos[0]*polygon[i].pos[0]+polygon[i].pos[1]*polygon[i].pos[1]+polygon[i].pos[2]*polygon[i].pos[2]);
                }
                //distz = distz + polygon[i].pos[2];// ave point
            }
            //distz = distz/polygon.size();// avepoint
            newpolygon.distance = distz;

            newpolygon.colour[0] = r;
            newpolygon.colour[1] = g;
            newpolygon.colour[2] = b;

            polygonbuffer.push_back(newpolygon);
        }

        void showlisted(){ // goes through above list and shows them in order of reverse dist to center incomplatible to fill poly buffer

            for (int i = 0; i < polygonbuffer.size(); i++){
                int nextpoly = 0;
                for (int j = 0; j < polygonbuffer.size(); j++){
                    if (polygonbuffer[j].distance > polygonbuffer[nextpoly].distance){
                        nextpoly = j;
                    }
                }
                if (polygonbuffer[nextpoly].distance != -1){// not already done it
                    polygonbuffer[nextpoly].distance = -1;
                    SDL_SetRenderDrawColor(renderer , polygonbuffer[nextpoly].colour[0]*lightshade[0] , polygonbuffer[nextpoly].colour[1]*lightshade[1]
                                                    , polygonbuffer[nextpoly].colour[2]*lightshade[2] , 255);
                    //std::cout << "polybuffer " << nextpoly << "\n";
                    fillpoly(polygonbuffer[nextpoly].points);
                }
            }

            polygonbuffer.clear();

        }


};

buffer mainbuffer;





class camera{
    public:
        double selfpos[3];
        double xangle;
        double yangle;
        double roll;
        double pointing[3];



        camera(){
            selfpos[0] = 0;
            selfpos[1] = -64;
            selfpos[2] = 0;
            xangle = 0;
            yangle = 0;
            roll = 0;

            pointing[0] = 0;
            pointing[1] = 0;
            pointing[2] = 1;
        }

        void move(double x,double y,double z){
            selfpos[0] += x;
            selfpos[1] += y;
            selfpos[2] += z;
            //printf("CAMERA anglex %.3f angley %.3f (pos %.2f,%.2f,%.2f)\n", xangle, yangle, selfpos[0], selfpos[1], selfpos[2]);
        }

        void rotate(double x,double y){
            xangle += x;
            yangle += y;
            if (yangle > M_PI/2-0.1){
                yangle = M_PI/2-0.1;
            }
            if (yangle < -M_PI/2+0.1){
                yangle = -M_PI/2+0.1;
            }
            if (xangle > 2*M_PI){
                xangle += -2*M_PI;
            }
            if (xangle < 0){
                xangle += 2*M_PI;
            }

            pointing[0] = 0;
            pointing[1] = 0;
            pointing[2] = 1;

            double * newpointing = getrevrotpos(pointing);
            for (int i = 0;i < 3;i++){
                pointing[i] = *(newpointing+i);
            }


            //std::cout << "CAMERA " << anglex << " " << angley << "\n";
            //printf("CAMERA anglex %.3f angley %.3f (pos %.2f,%.2f,%.2f)\n", xangle, yangle, selfpos[0], selfpos[1], selfpos[2]);

        }

        double * getrotpos(double pos[3]){

            double afterxpos[3];
            afterxpos[1] = pos[1];//y stay same rot around y axis

            afterxpos[0] = sin(xangle)*pos[2]+cos(xangle)*pos[0];
            afterxpos[2] = -sin(xangle)*pos[0]+cos(xangle)*pos[2];

            double afterxypos[3];

             
            afterxypos[0] = afterxpos[0];//rot around x axis
            
            afterxypos[1] = sin(yangle)*afterxpos[2]+cos(yangle)*afterxpos[1];
            afterxypos[2] = -sin(yangle)*afterxpos[1]+cos(yangle)*afterxpos[2];
            


            static double endpos[3];

            endpos[2] = afterxypos[2];//roll around z axis
            
            endpos[0] = sin(roll)*afterxypos[1]+cos(roll)*afterxypos[0];
            endpos[1] = -sin(roll)*afterxypos[0]+cos(roll)*afterxypos[1];

            // rot around y axis is rolling the camera


            return endpos;
        }

        double * getrevrotpos(double pos[3]){

            double afterrollpos[3];
            afterrollpos[2] = pos[2];//roll around z axis

            afterrollpos[0] = sin(-roll)*pos[1]+cos(-roll)*pos[0];
            afterrollpos[1] = -sin(-roll)*pos[0]+cos(-roll)*pos[1];

            double afterrollypos[3];

             
            afterrollypos[0] = afterrollpos[0];//rot around x axis
            
            afterrollypos[1] = sin(-yangle)*afterrollpos[2]+cos(-yangle)*afterrollpos[1];
            afterrollypos[2] = -sin(-yangle)*afterrollpos[1]+cos(-yangle)*afterrollpos[2];
            


            static double endpos[3];

            endpos[1] = afterrollypos[1];//y stay same rot around y axis
            
            endpos[0] = sin(-xangle)*afterrollypos[2]+cos(-xangle)*afterrollypos[0];
            endpos[2] = -sin(-xangle)*afterrollypos[0]+cos(-xangle)*afterrollypos[2];


            return endpos;
        }


        double * getrelpos(double pos[3]){
            
            for (int i = 0;i < 3; i++){// get rel pos
                pos[i] = pos[i] - selfpos[i];
            }
            static double relpos[3];
            
            double * nextpos1 = getrotpos(pos);//rotate points to cam perspective
            for (int i = 0;i < 3;i++){
                relpos[i] = *(nextpos1 + i);
            }
            
            return relpos;

        }


        void clippolygon(std::vector<point> &thepolygon,int numlines,double a,double b,double c,double d,bool keepbig){ // takes a polygon and clips against a plane
            //plane formulae ax+by+cz+d = 0 keepbig deturmines if keep ax+by+cz+d > 0 or ax+by+cz+d < 0
            std::vector<point> newpolygon;
            int numnewpoints = 0;
            for (int linenum = 0;linenum < numlines;linenum++){
                int secondpoint = linenum + 1;
                if (secondpoint == numlines){
                    secondpoint = 0;
                }
                double linevect[3];
                for (int dimesion = 0;dimesion < 3;dimesion++){
                    linevect[dimesion] = thepolygon[secondpoint].pos[dimesion]-thepolygon[linenum].pos[dimesion];
                }
                //a*thepolygon[linenum].pos[0]+a*t*linevect[0]+b*thepolygon[linenum].pos[1]+b*t*linevect[1]+c*thepolygon[linenum].pos[2]+c*t*linevect[2]+d = 0
                // for t is the intersection
                //a*t*linevect[0]+b*t*linevect[1]+c*t*linevect[2] = -(a*thepolygon[linenum].pos[0]+b*thepolygon[linenum].pos[1]+c*thepolygon[linenum].pos[2]+d)
                //t(a*linevect[0]+b*linevect[1]+c*linevect[2]) = -(a*thepolygon[linenum].pos[0]+b*thepolygon[linenum].pos[1]+c*thepolygon[linenum].pos[2]+d)
                
                double t;
                if (a*linevect[0]+b*linevect[1]+c*linevect[2] == 0){//line and plane paralel
                    t = -1;
                }
                else{
                    t = -(a*thepolygon[linenum].pos[0]+b*thepolygon[linenum].pos[1]+c*thepolygon[linenum].pos[2]+d)/(a*linevect[0]+b*linevect[1]+c*linevect[2]);
                }
                if (t < 1 && t > 0){ // if intersection is between relavant points
                    point intersectpoint;
                    for (int dimension = 0;dimension < 3;dimension++){
                        intersectpoint.pos[dimension] = t*linevect[dimension]+thepolygon[linenum].pos[dimension];
                    }
                    if ((a*thepolygon[linenum].pos[0]+b*thepolygon[linenum].pos[1]+c*thepolygon[linenum].pos[2]+d > 0) == keepbig){// whether we keep start or end point
                        point startpoint;
                        for (int dimension = 0;dimension < 3;dimension++){
                            startpoint.pos[dimension] = thepolygon[linenum].pos[dimension];
                        }
                        newpolygon.push_back(startpoint);
                        newpolygon.push_back(intersectpoint);//here this end point is different to next one
                        numnewpoints = numnewpoints + 2;

                    }
                    else{
                        point endpoint;
                        for (int dimension = 0;dimension < 3;dimension++){
                            endpoint.pos[dimension] = thepolygon[secondpoint].pos[dimension];
                        }
                        newpolygon.push_back(intersectpoint);
                        //newpolygon.push_back(endpoint); // end points should not be added as will be added later
                        numnewpoints = numnewpoints + 1;

                    }

                }
                else{
                    point intersectpoint;
                    for (int dimension = 0;dimension < 3;dimension++){
                        intersectpoint.pos[dimension] = t*linevect[dimension]+thepolygon[linenum].pos[dimension];
                    }
                    
                    if (abs(a*intersectpoint.pos[0] + b*intersectpoint.pos[1] + c*intersectpoint.pos[2] + d) > 1){ /*means that the problem is that it is paralel*/}
                    else{
                        //std::cout << "CAMERA intersect" << intersectpoint.pos[0] << " " << intersectpoint.pos[1] << " " << intersectpoint.pos[2] << "\n";
                    }
                        
                    if ((a*thepolygon[linenum].pos[0]+b*thepolygon[linenum].pos[1]+c*thepolygon[linenum].pos[2]+d > 0) == keepbig){// if whole line is on right side
                        //std::cout << "CAMERA" << a*thepolygon[linenum].pos[0]+b*thepolygon[linenum].pos[1]+c*thepolygon[linenum].pos[2]+d << "\n";
                        point endpoint;
                        point startpoint;
                        for (int dimension = 0;dimension < 3;dimension++){
                            startpoint.pos[dimension] = thepolygon[linenum].pos[dimension];
                            endpoint.pos[dimension] = thepolygon[secondpoint].pos[dimension];
                        }
                        newpolygon.push_back(startpoint);
                        //newpolygon.push_back(endpoint); // end points should not be added as will be added later
                        numnewpoints = numnewpoints + 1;
                    }
                    //else dont add a point as line out of clip
                    
                }

            }

            thepolygon.clear();
            for (int pointnum = 0;pointnum < numnewpoints;pointnum++){
                thepolygon.push_back(newpolygon[pointnum]);
            }

        }

        void preppolygon(std::vector<point> &thepolygon,int numlines){
            
            for (int lines = 0;lines < numlines;lines++){ // get relpos

                //std::cout << "CAMERA point start space at" << thepolygon[lines].pos[0] << " "  << 
                                //thepolygon[lines].pos[1] << " " << thepolygon[lines].pos[2]<< "\n";


                double pos1[3];
                for (int dimension = 0;dimension < 3;dimension++){
                    pos1[dimension] = thepolygon[lines].pos[dimension];
                }
                double * nextpos1 = getrelpos(pos1);//rotate points to cam perspective
                for (int i = 0;i < 3;i++){
                    thepolygon[lines].pos[i] = *(nextpos1 + i);
                }

                //std::cout << "CAMERA point rel space at" << thepolygon[lines].pos[0] << " "  << 
                                //thepolygon[lines].pos[1] << " " << thepolygon[lines].pos[2]<< "\n";
            }
            

            //clippolygon(thepolygon,numlines,0,0,1,-1,true); //cut this along z = 1 keep above
            numlines = thepolygon.size();

            
            // need to not go to screen space till after the clip finish as it is interpolating the z of point which crossed the edge
            // this means more complicated plane formulae





            // to get the plane equation 

            // vect AB and vect AC but as the center 0,0,0 is part of all planes I can just use points not vects

            //a=(By−Ay)(Cz−Az)−(Cy−Ay)(Bz−Az)
            //b=(Bz−Az)(Cx−Ax)−(Cz−Az)(Bx−Ax)
            //c=(Bx−Ax)(Cy−Ay)−(Cx−Ax)(By−Ay)
            //d=−(aAx+bAy+cAz)

            // or as A is all 0

            //a = By*Cz - Cy*Bz
            //b = Bz*Cx - Cz*Bx
            //c = Bx*Cy - Cx*By
            
            //d = 0 as all A coefficients are 0



            //one of a or b will be 0 as z*(x or y) is const and took from self

            clippolygon(thepolygon,numlines,dispdist,0,dispwidth/2,0,true);//left
            numlines = thepolygon.size();
            
            clippolygon(thepolygon,numlines,dispdist,0,-dispwidth/2,0,false);//right
            numlines = thepolygon.size();
            
            clippolygon(thepolygon,numlines,0,dispdist,dispheight/2,0,true);//top
            numlines = thepolygon.size();
            
            clippolygon(thepolygon,numlines,0,dispdist,-dispheight/2,0,false);//bottom
            numlines = thepolygon.size();
            

            for (int pointnum = 0;pointnum < numlines;pointnum++){// to screen space
                thepolygon[pointnum].pos[0] = thepolygon[pointnum].pos[0]/thepolygon[pointnum].pos[2]*dispdist;//+dispwidth/2;
                thepolygon[pointnum].pos[1] = thepolygon[pointnum].pos[1]/thepolygon[pointnum].pos[2]*dispdist;//+dispheight/2;
                //std::cout << "CAMERA point screen space at" << thepolygon[pointnum].pos[0] << " "  << 
                                //thepolygon[pointnum].pos[1] << " " << thepolygon[pointnum].pos[2]<< "\n";
            }

            // dist scale so cube not frustum


        }

};


camera maincamera;

class letter{
    public:
        std::vector<point> lines;

        void addline(double x,double y ,double otherx ,double othery){
            point newpoint;
            newpoint.pos[0] = (x + 0.5)*0.7; // kerning
            newpoint.pos[1] = 0;
            newpoint.pos[2] = y;

            point otherpoint;
            otherpoint.pos[0] = (otherx + 0.5)*0.7;
            otherpoint.pos[1] = 0;
            otherpoint.pos[2] = othery;

            lines.push_back(newpoint);
            lines.push_back(otherpoint);
        }


};


class wordwriter{
    public:
        std::vector<letter> letters;

        void makeletters(){
            // all letters are done by drawing lines  with a width
            letter a;//A
            a.addline(-0.5,-1 , 0,1);
            a.addline(0,1 , 0.5,-1);
            a.addline(-0.25,0 , 0.25,0);

            letter b;//B
            b.addline(-0.5,1 , -0.5,-1);
            b.addline(-0.5,1 , 0.3,1);
            b.addline(-0.5,0 , 0.3,0);
            b.addline(-0.5,-1 , 0.3,-1);
            b.addline(0.3,1 , 0.5,0.5);
            b.addline(0.5,0.5 , 0.3,0);
            b.addline(0.3,-1 , 0.5,-0.5);
            b.addline(0.5,-0.5 , 0.3,0);

            letter c;//C
            c.addline(0.5,0.8 , 0,1);
            c.addline(0,1 , -0.5,0.8);
            c.addline(-0.5,0.8 , -0.5,-0.8);
            c.addline(-0.5,-0.8 , 0,-1);
            c.addline(0,-1 , 0.5,-0.8);

            letter d;//D
            d.addline(-0.5,1 , -0.5,-1);
            d.addline(-0.5,1 , 0.2,1);
            d.addline(0.2,1 , 0.5,0.6);
            d.addline(0.5,0.6 , 0.5,-0.6);
            d.addline(0.5,-0.6 , 0.2,-1);
            d.addline(0.2,-1 , -0.5,-1);

            letter e;//E
            e.addline(-0.5,1 , -0.5,-1);
            e.addline(-0.5,1 , 0.5,1);
            e.addline(-0.5,0 , 0.4,0);
            e.addline(-0.5,-1 , 0.5,-1);

            letter f;//F
            f.addline(-0.5,1 , -0.5,-1);
            f.addline(-0.5,1 , 0.5,1);
            f.addline(-0.5,0 , 0.4,0);
            
            letter g;//G
            g.addline(0.5,0.8 , 0,1);
            g.addline(0,1 , -0.5,0.8);
            g.addline(-0.5,0.8 , -0.5,-0.8);
            g.addline(-0.5,-0.8 , 0,-1);
            g.addline(0,-1 , 0.5,-0.8);
            g.addline(0.5,-0.8 , 0.5,0);
            g.addline(0.5,0 , 0,0);

            letter h;//H
            h.addline(-0.5,1 , -0.5,-1);
            h.addline(0.5,1 , 0.5,-1);
            h.addline(-0.5,0 , 0.5,0);

            letter i;//I
            i.addline(-0.5,1 , 0.5,1);
            i.addline(-0.5,-1 , 0.5,-1);
            i.addline(0,1 , 0,-1);

            letter j;//J
            j.addline(0,1 , 0.5,1);
            j.addline(0.5,1 , 0.5,-0.8);
            j.addline(0.5,-0.8 , 0,-1);
            j.addline(0,-1 , -0.5,-0.8);

            letter k;//K
            k.addline(-0.5,1 , -0.5,-1);
            k.addline(-0.5,0 , 0.5,1);
            k.addline(-0.5,0 , 0.5,-1);

            letter l;//L
            l.addline(-0.5,1 , -0.5,-1);
            l.addline(-0.5,-1 , 0.5,-1);

            letter m;//M
            m.addline(-0.5,1 , -0.5,-1);
            m.addline(-0.5,1 , 0,0);
            m.addline(0,0 , 0.5,1);
            m.addline(0.5,1 , 0.5,-1);

            letter n;//N
            n.addline(-0.5,1 , -0.5,-1);
            n.addline(-0.5,1 , 0.5,-1);
            n.addline(0.5,1 , 0.5,-1);

            letter o;//O
            o.addline(0.5,0.8 , 0,1);
            o.addline(0,1 , -0.5,0.8);
            o.addline(-0.5,0.8 , -0.5,-0.8);
            o.addline(-0.5,-0.8 , 0,-1);
            o.addline(0,-1 , 0.5,-0.8);
            o.addline(0.5,0.8 , 0.5,-0.8);

            letter p;//P
            p.addline(-0.5,1 , -0.5,-1);
            p.addline(-0.5,1 , 0.3,1);
            p.addline(-0.5,0 , 0.3,0);
            p.addline(0.3,1 , 0.5,0.5);
            p.addline(0.5,0.5 , 0.3,0);

            letter q;//Q
            q.addline(0.5,0.8 , 0,1);
            q.addline(0,1 , -0.5,0.8);
            q.addline(-0.5,0.8 , -0.5,-0.8);
            q.addline(-0.5,-0.8 , 0,-1);
            q.addline(0,-1 , 0.5,-0.8);
            q.addline(0.5,0.8 , 0.5,-0.8);
            q.addline(0.25,-0.5 , 0.5,-1);

            letter r;//R
            r.addline(-0.5,1 , -0.5,-1);
            r.addline(-0.5,1 , 0.3,1);
            r.addline(-0.5,0 , 0.3,0);
            r.addline(0.3,1 , 0.5,0.5);
            r.addline(0.5,0.5 , 0.3,0);
            r.addline(0.3,0 , 0.5,-1);

            letter s;//S
            s.addline(0.5,0.8 , 0,1);
            s.addline(0,1 , -0.5,0.8);
            s.addline(-0.5,0.8 , 0.5,-0.8);
            s.addline(-0.5,-0.8 , 0,-1);
            s.addline(0,-1 , 0.5,-0.8);

            letter t;//T
            t.addline(-0.5,1 , 0.5,1);
            t.addline(0,1 , 0,-1);

            letter u;//U
            u.addline(-0.5,1 , -0.5,-0.8);
            u.addline(-0.5,-0.8 , 0,-1);
            u.addline(0,-1 , 0.5,-0.8);
            u.addline(0.5,1 , 0.5,-0.8);

            letter v;//V
            v.addline(-0.5,1 , 0,-1);
            v.addline(0,-1 , 0.5,1);

            letter w;//W
            w.addline(-0.5,1 , -0.5,-1);
            w.addline(-0.5,-1 , 0,0);
            w.addline(0,0 , 0.5,-1);
            w.addline(0.5,1 , 0.5,-1);

            letter x;//X
            x.addline(-0.5,1 , 0.5,-1);
            x.addline(-0.5,-1 , 0.5,1);

            letter y;//Y
            y.addline(-0.5,1 , 0,0);
            y.addline(0.5,1 , 0,0);
            y.addline(0,0 , 0,-1);

            letter z;//Z
            z.addline(-0.5,1 , 0.5,1);
            z.addline(0.5,1 , -0.5,-1);
            z.addline(-0.5,-1 , 0.5,-1);

            letter zero;//0
            zero.addline(0.5,0.8 , 0,1);
            zero.addline(0,1 , -0.5,0.8);
            zero.addline(-0.5,0.8 , -0.5,-0.8);
            zero.addline(-0.5,-0.8 , 0,-1);
            zero.addline(0,-1 , 0.5,-0.8);
            zero.addline(0.5,0.8 , 0.5,-0.8);
            zero.addline(-0.5,-0.8 , 0.5,0.8);



            letter one;//1
            one.addline(0,1 , 0,-1);
            one.addline(-0.5,-1 , 0.5,-1);
            one.addline(0,1 , -0.2,0.9);

            letter two;//2
            two.addline(-0.5,0.8 , 0,1);
            two.addline(0,1 , 0.5,0.8);
            two.addline(0.5,0.8 , -0.5,-1);
            two.addline(-0.5,-1 , 0.5,-1);

            letter three;//3
            three.addline(-0.5,0.9 , 0.3,1);
            three.addline(-0.2,0 , 0.3,0);
            three.addline(-0.5,-0.9 , 0.3,-1);
            three.addline(0.3,1 , 0.5,0.5);
            three.addline(0.5,0.5 , 0.3,0);
            three.addline(0.3,-1 , 0.5,-0.5);
            three.addline(0.5,-0.5 , 0.3,0);

            letter four;//4
            four.addline(0.5,-0.2 , -0.5,-0.2);
            four.addline(-0.5,-0.2 , 0.2,1);
            four.addline(0.2,1 , 0.2,-1);

            letter five;//5
            five.addline(0.5,1 , -0.5,1);
            five.addline(-0.5,1 , -0.5,0);
            five.addline(-0.5,0 , 0.3,0.1);
            five.addline(0.3,0.1 , 0.5,-0.2);
            five.addline(0.5,-0.2 , 0.3,-1);
            five.addline(0.3,-1 , -0.5,-0.9);

            letter six;//6
            six.addline(0.5,0.8 , 0,1);
            six.addline(0,1 , -0.5,0.8);
            six.addline(-0.5,0.8 , -0.5,-0.8);
            six.addline(-0.5,-0.8 , 0,-1);
            six.addline(0,-1 , 0.5,-0.8);
            six.addline(0.5,0.2 , 0.5,-0.8);
            six.addline(0.5,0.2 , 0,0.4);
            six.addline(0,0.4 , -0.5,0.2);

            letter seven;//7
            seven.addline(-0.5,1 , 0.5,1);
            seven.addline(0.5,1 , 0,-1);

            letter eight;//8
            eight.addline(0,1 , -0.5,0.8);
            eight.addline(-0.5,0.8 , 0.5,-0.8);
            eight.addline(0.5,-0.8 , 0,-1);
            eight.addline(0,-1 , -0.5,-0.8);
            eight.addline(-0.5,-0.8 , 0.5,0.8);
            eight.addline(0.5,0.8 , 0,1);

            letter nine;//9
            nine.addline(-0.5,-0.8 , 0,-1);
            nine.addline(0,-1 , 0.5,-0.8);
            nine.addline(0.5,-0.8 , 0.5,0.8);
            nine.addline(0.5,0.8 , 0,1);
            nine.addline(0,1 , -0.5,0.8);
            nine.addline(-0.5,-0.2 , -0.5,0.8);
            nine.addline(-0.5,-0.2 , 0,-0.4);
            nine.addline(0,-0.4 , 0.5,-0.2);



            letter dot;//.
            dot.addline(0,-0.8 , 0,-0.8);

            letter colon;//:
            colon.addline(0,-0.8 , 0,-0.8);
            colon.addline(0,0.8 , 0,0.8);







            letters.push_back(a);
            letters.push_back(b);
            letters.push_back(c);
            letters.push_back(d);
            letters.push_back(e);
            letters.push_back(f);
            letters.push_back(g);
            letters.push_back(h);
            letters.push_back(i);
            letters.push_back(j);
            letters.push_back(k);
            letters.push_back(l);
            letters.push_back(m);
            letters.push_back(n);
            letters.push_back(o);
            letters.push_back(p);
            letters.push_back(q);
            letters.push_back(r);
            letters.push_back(s);
            letters.push_back(t);
            letters.push_back(u);
            letters.push_back(v);
            letters.push_back(w);
            letters.push_back(x);
            letters.push_back(y);
            letters.push_back(z);

            letters.push_back(zero);
            letters.push_back(one);
            letters.push_back(two);
            letters.push_back(three);
            letters.push_back(four);
            letters.push_back(five);
            letters.push_back(six);
            letters.push_back(seven);
            letters.push_back(eight);
            letters.push_back(nine);

            letters.push_back(dot);
            letters.push_back(colon);


        }

        void writeletter(int x,int y,double size,char theletter ,int width){// for width could get manhatan dists then draw lines across
            int num;
            //std::cout << "words " << int(theletter) << "\n";
            if (int(theletter) == 32){num = -1;}
            else if (int(theletter) >= 97 && int(theletter) <= 26+97){
                num = int(theletter)-97;// will get location in list
            }
            else if (int(theletter) >= 48 && int(theletter) <= 57){
                num = int(theletter)-48+26;

            }
            else if (int(theletter) == 46){
                num = 36;
            }
            else if (int(theletter) == 58){
                num = 37;
            }
            if (num == -1){}
            else{
                //std::cout << "words " << int(theletter) << "\n";
                //std::cout << "words" << letters[num].lines.size() << "\n";
                for (int i = 0;i < letters[num].lines.size();i = i+2){
                    
                    int startx = letters[num].lines[i].pos[0]*size + x;
                    int starty = letters[num].lines[i].pos[2]*size + y;
                    int endx = letters[num].lines[i+1].pos[0]*size + x;
                    int endy = letters[num].lines[i+1].pos[2]*size + y;
                    
                    int miny = starty - width;
                    if (endy - width < miny){
                        miny = endy - width;
                    }
                    int maxy = starty + width;
                    if (endy + width > maxy){
                        maxy = endy + width;
                    }
                    
                    for (int j = miny; j < maxy+1; j++){
                        int minx;
                        int maxx;
                        int xintercept;
                        double rowwidth;
                        if (starty == endy){
                            xintercept = (startx+endx)/2; // get the middle
                            rowwidth = abs(endx-startx)/2+width;
                        }
                        else{
                            double m = (endx-startx)/(double)(endy-starty);
                            xintercept = startx + (j-starty)*m;
                            if (m != 0){
                                rowwidth = abs(width/(sin(atan(1/m))));
                            }
                            else{
                                rowwidth = width;
                            }
                        }
                        minx = startx - width;
                        if (endx - width < minx){
                            minx = endx - width;
                        }
                        maxx = startx + width;
                        if (endx + width > maxx){
                            maxx = endx + width;
                        }

                        if (xintercept+rowwidth >= minx && xintercept+rowwidth <= maxx){
                            maxx = xintercept+rowwidth;
                        }
                        if (xintercept-rowwidth >= minx && xintercept-rowwidth <= maxx){
                            minx = xintercept-rowwidth;
                        }
                        //std::cout << "words" << minx << " " << maxx << " " << j << "\n";
                        //std::cout << "words" << rowwidth << " " << (endx-startx)/(double)(endy-starty) << "\n";
                        
                        SDL_SetRenderDrawColor(renderer , 0,0,0,255);
                        SDL_RenderDrawLine(renderer , minx+dispwidth/2 , -j+dispheight/2 , maxx+dispwidth/2 , -j+dispheight/2);

                    }

                    //std::cout << "words" << startx << " " << starty << " " << endx << " "  << endy << "\n";
                    SDL_SetRenderDrawColor(renderer , 255*lightshade[0],0*lightshade[1],0*lightshade[2],255);
                    SDL_RenderDrawLine(renderer , startx+dispwidth/2, -starty+dispheight/2 , endx+dispwidth/2, -endy+dispheight/2);
                }
            }
        }

        void writechars(int x , int y , double size , std::string shownstring , int width){

            for (int i = 0; i < shownstring.length();i++){
                //std::cout << "words " << shownstring[i] <<"\n";
                writeletter(x + i*size , y , size , shownstring[i] , width);
            }

        }
};

wordwriter mainwordwriter;


class road{
    public:

        std::vector<hyperpoint> points; // points will start by being linear interpolation but will swap for beizer curves
                                        // the forth dimension is for banking 4d bezier 

        hyperpoint startpoint;// defines hermite curve
        hyperpoint endpoint;
        hyperpoint starttangent;
        hyperpoint endtangent;

        double length = 1;
        double spacing = 1;
        double width = 1;
        std::vector<point> leftpoints;
        std::vector<point> rightpoints;


        int roadid = -1; // this is a number that identifies this roadsegment


        road(double newlength , double newspacing , double newwidth , std::vector<hyperpoint> newpoints , int id){
            length = newlength;
            spacing = newspacing;
            width = newwidth;
            points.clear();

            /*for (int i = 0; i < newpoints.size(); i++){
                points.push_back(newpoints[i]);
            }*/
            startpoint = newpoints[0];
            endpoint = newpoints[1];
            starttangent = newpoints[2];
            endtangent = newpoints[3];

            roadid = id;

            getoutsidepoints();

        }


        hyperpoint getposofroad(double ratio){ // this will give position of the road at a given fraction through it 0 giving start and 1 end
            double t0 = 1;
            double t1 = ratio;
            double t2 = ratio*ratio;
            double t3 = ratio*ratio*ratio;


            double v0 = 2*t3 - 3*t2 + 1;
            double v1 = -2*t3 + 3*t2;
            double v2 = t3 - 2*t2 + t1;
            double v3 = t3 - t2;

            hyperpoint returnpoint;
            for (int i = 0; i < 4; i++){
                returnpoint.pos[i] = v0*startpoint.pos[i] + v1*endpoint.pos[i] + v2*starttangent.pos[i] + v3*endtangent.pos[i];
            }

            return returnpoint;
            
            /*std::vector<hyperpoint> laststagepoints; // for beziers
            for (int i = 0;i < points.size()-1;i++){
                std::vector<hyperpoint> newpoints;
                if (i == 0){
                    for (int j = 0; j < points.size()-1;j++){
                        hyperpoint newpoint;
                        newpoint.pos[0] = (1-ratio)*points[j].pos[0]+ratio*points[j+1].pos[0];
                        newpoint.pos[1] = (1-ratio)*points[j].pos[1]+ratio*points[j+1].pos[1];
                        newpoint.pos[2] = (1-ratio)*points[j].pos[2]+ratio*points[j+1].pos[2];
                        newpoint.pos[3] = (1-ratio)*points[j].pos[3]+ratio*points[j+1].pos[3];
                        newpoints.push_back(newpoint);
                    }
                }
                else{
                    for (int j = 0; j < points.size()-1;j++){
                        hyperpoint newpoint;
                        newpoint.pos[0] = (1-ratio)*laststagepoints[j].pos[0]+ratio*laststagepoints[j+1].pos[0];
                        newpoint.pos[1] = (1-ratio)*laststagepoints[j].pos[1]+ratio*laststagepoints[j+1].pos[1];
                        newpoint.pos[2] = (1-ratio)*laststagepoints[j].pos[2]+ratio*laststagepoints[j+1].pos[2];
                        newpoint.pos[3] = (1-ratio)*laststagepoints[j].pos[3]+ratio*laststagepoints[j+1].pos[3];
                        newpoints.push_back(newpoint);
                    }
                }

                laststagepoints.clear();
                for (int j = 0;j < newpoints.size();j++){
                    laststagepoints.push_back(newpoints[j]);
                }
            }

            return laststagepoints[0];*/
        
        }

        void getoutsidepoints(){
            leftpoints.clear();
            rightpoints.clear();


            int segnum = 0;
            for (double dist = 0;dist < length+spacing;dist += spacing){
                segnum = segnum + 1;



                double ratio = dist/length;
                hyperpoint nextpoint = getposofroad(ratio);


                hyperpoint slightlyfurtherpoint = getposofroad(ratio + 0.001); // does not need hyper capabilities
                point normal;
                normal.pos[0] = -(slightlyfurtherpoint.pos[2]-nextpoint.pos[2]);
                normal.pos[2] = (slightlyfurtherpoint.pos[0]-nextpoint.pos[0]);

                double normlen = sqrt(normal.pos[0]*normal.pos[0]+normal.pos[2]*normal.pos[2]);
                double banklen = sqrt(1-nextpoint.pos[3]*nextpoint.pos[3]);

                //std::cout << "normal " << normal.pos[0] << " " << normal.pos[1] << " " << normal.pos[2] << "\n";
                

                normal.pos[0] = width*normal.pos[0]/normlen*banklen;
                normal.pos[1] = width*normlen*nextpoint.pos[3];
                normal.pos[2] = width*normal.pos[2]/normlen*banklen;

                point nextleftpoint;
                nextleftpoint.pos[0] = nextpoint.pos[0]+normal.pos[0];
                nextleftpoint.pos[1] = nextpoint.pos[1]+normal.pos[1];
                nextleftpoint.pos[2] = nextpoint.pos[2]+normal.pos[2];
                point nextrightpoint;
                nextrightpoint.pos[0] = nextpoint.pos[0]-normal.pos[0];
                nextrightpoint.pos[1] = nextpoint.pos[1]-normal.pos[1];
                nextrightpoint.pos[2] = nextpoint.pos[2]-normal.pos[2];


                leftpoints.push_back(nextleftpoint);
                rightpoints.push_back(nextrightpoint);
                //std::cout << "start track pos" << nextleftpoint.pos[0] << " " << nextleftpoint.pos[1] << " " << nextleftpoint.pos[2] << "\n";
                //std::cout << "start track pos" << nextrightpoint.pos[0] << " " << nextrightpoint.pos[1] << " " << nextrightpoint.pos[2] << "\n";
                //std::cout << "normal " << normal.pos[0] << " " << normal.pos[1] << " " << normal.pos[2] << "\n";
                //std::cout << "normal length " << normlen << "\n";
            }
        }

        void minimaprender(){
            point lastpoint;
            
            lastpoint.pos[0] = startpoint.pos[0];
            lastpoint.pos[1] = 0;
            lastpoint.pos[2] = startpoint.pos[2];

            point startleftpoint = leftpoints[0];
            point startrightpoint = rightpoints[0];

            double scaler = 32/2048.0;

            SDL_RenderDrawLine(renderer,(lastpoint.pos[0])*scaler+dispwidth*3/4,(lastpoint.pos[2])*scaler+dispheight*3/4,
                                (startleftpoint.pos[0])*scaler+dispwidth*3/4,(startleftpoint.pos[2])*scaler+dispheight*3/4);
            SDL_RenderDrawLine(renderer,(lastpoint.pos[0])*scaler+dispwidth*3/4,(lastpoint.pos[2])*scaler+dispheight*3/4,
                                (startrightpoint.pos[0])*scaler+dispwidth*3/4,(startrightpoint.pos[2])*scaler+dispheight*3/4);

            for (int segnum = 1; segnum <= length/spacing; segnum++){
                
                double ratio = segnum*spacing/length;
                hyperpoint nextpoint = getposofroad(ratio);

                SDL_RenderDrawLine(renderer,(lastpoint.pos[0])*scaler+dispwidth*3/4,(lastpoint.pos[2])*scaler+dispheight*3/4,
                                        (nextpoint.pos[0])*scaler+dispwidth*3/4,(nextpoint.pos[2])*scaler+dispheight*3/4);


                point newleftpoint = leftpoints[segnum];
                
                point newrightpoint = rightpoints[segnum];

                point lastleftpoint = leftpoints[segnum-1];

                point lastrightpoint = rightpoints[segnum-1];

                SDL_RenderDrawLine(renderer,(nextpoint.pos[0])*scaler+dispwidth*3/4,(nextpoint.pos[2])*scaler+dispheight*3/4,
                                (newleftpoint.pos[0])*scaler+dispwidth*3/4,(newleftpoint.pos[2])*scaler+dispheight*3/4);
                SDL_RenderDrawLine(renderer,(nextpoint.pos[0])*scaler+dispwidth*3/4,(nextpoint.pos[2])*scaler+dispheight*3/4,
                                (newrightpoint.pos[0])*scaler+dispwidth*3/4,(newrightpoint.pos[2])*scaler+dispheight*3/4);
                
                lastpoint.pos[0] = nextpoint.pos[0];
                lastpoint.pos[2] = nextpoint.pos[2];
            }
        }

        void renderroad(){

            //getoutsidepoints();




            

            point startleftpoint = leftpoints[0];
            
            point startrightpoint = rightpoints[0];
            


            int segnum = 0;
            for (double dist = spacing;dist < length+spacing;dist += spacing){
                if (segnum%2 == 0){
                    SDL_SetRenderDrawColor( renderer, 0x70*lightshade[0], 0x70*lightshade[1], 0x70*lightshade[2], 0xFF);
                }
                else{
                    SDL_SetRenderDrawColor( renderer, 0x90*lightshade[0], 0x90*lightshade[1], 0x90*lightshade[2], 0xFF);
                }
                segnum = segnum + 1;
                

                point newleftpoint = leftpoints[segnum];
                
                point newrightpoint = rightpoints[segnum];

                point lastleftpoint = leftpoints[segnum-1];

                point lastrightpoint = rightpoints[segnum-1];


                std::vector<point> newpolygon;
                newpolygon.push_back(lastleftpoint);
                newpolygon.push_back(lastrightpoint);
                newpolygon.push_back(newrightpoint);
                newpolygon.push_back(newleftpoint);

                //std::cout << leftpoints[segnum].pos[0] << " " << leftpoints[segnum].pos[2] << " left\n";
                //std::cout << rightpoints[segnum].pos[0] << " " << rightpoints[segnum].pos[2] << " right\n";
                //std::cout << leftpoints[segnum-1].pos[0] << " " << leftpoints[segnum-1].pos[2] << " left\n";
                //std::cout << rightpoints[segnum-1].pos[0] << " " << rightpoints[segnum-1].pos[2] << " right\n";


                maincamera.preppolygon(newpolygon,4);

                if (newpolygon.size() != 0){
                    if (fullzbuffer){
                        mainbuffer.fillpolybuffered(newpolygon);
                    }
                    else{
                        if (segnum%2 == 0){
                            mainbuffer.simplebuffer(newpolygon , 0x90 ,0x90 ,0x90);
                        }
                        else{
                            mainbuffer.simplebuffer(newpolygon , 0x70 ,0x70 ,0x70);
                        }
                        
                    }
                }
                

            }
            //std::cout << "\n";
        }

        int isincheckpoint(double givenpos[3]){// similar to isontrack but only for first segment returns identification if true else -1

            
            point givenpospoint;
            givenpospoint.pos[0] = givenpos[0];
            givenpospoint.pos[1] = givenpos[1];
            givenpospoint.pos[2] = givenpos[2];


            point newleftpoint = leftpoints[1];
                
            point newrightpoint = rightpoints[1];

            point lastleftpoint = leftpoints[0];

            point lastrightpoint = rightpoints[0];


            std::vector<point> newpolygon;
            newpolygon.push_back(lastleftpoint);
            newpolygon.push_back(lastrightpoint);
            newpolygon.push_back(newrightpoint);
            newpolygon.push_back(newleftpoint);
            if (isinpoly(newpolygon,givenpospoint)){
                return roadid;
            }

            return -1;

        }

        bool isontrack(double givenpos[3]){

            point givenpospoint;
            givenpospoint.pos[0] = givenpos[0];
            givenpospoint.pos[1] = givenpos[1];
            givenpospoint.pos[2] = givenpos[2];


            for (double segnum = 1;segnum <= length/spacing;segnum++){

                point newleftpoint = leftpoints[segnum];
                
                point newrightpoint = rightpoints[segnum];

                point lastleftpoint = leftpoints[segnum-1];

                point lastrightpoint = rightpoints[segnum-1];

                std::vector<point> newpolygon;
                newpolygon.push_back(lastleftpoint);
                newpolygon.push_back(lastrightpoint);
                newpolygon.push_back(newrightpoint);
                newpolygon.push_back(newleftpoint);
                if (isinpoly(newpolygon,givenpospoint)){
                    //std::cout << "trackid" << segnum << "\n";
                    
                    return true;
                }
                
                //std::cout << "trackid" <<newleftpoint.pos[0] << " " << newleftpoint.pos[2] << "\n";
                //std::cout << "trackid" <<newrightpoint.pos[0] << " " << newrightpoint.pos[2] << "\n";
                //std::cout << "trackid" <<lastleftpoint.pos[0] << " " << lastleftpoint.pos[2] << "\n";
                //std::cout << "trackid" <<lastrightpoint.pos[0] << " " << lastrightpoint.pos[2] << "\n";
                

            }

            return false;


        }

        int getsegnumfrompos(double givenpos[3]){

            //getoutsidepoints();

            point givenpospoint;
            givenpospoint.pos[0] = givenpos[0];
            givenpospoint.pos[1] = givenpos[1];
            givenpospoint.pos[2] = givenpos[2];


            int segnum = 0;
            for (double dist = 0;dist < length+spacing;dist += spacing){

                segnum = segnum + 1;

                point newleftpoint = leftpoints[segnum];
                
                point newrightpoint = rightpoints[segnum];

                point lastleftpoint = leftpoints[segnum-1];

                point lastrightpoint = rightpoints[segnum-1];

                std::vector<point> newpolygon;
                newpolygon.push_back(lastleftpoint);
                newpolygon.push_back(lastrightpoint);
                newpolygon.push_back(newrightpoint);
                newpolygon.push_back(newleftpoint);
                if (isinpoly(newpolygon,givenpospoint)){
                    return segnum;
                }
                //std::cout << segnum << "\n";

                //std::cout << newleftpoint.pos[0] << " " << newleftpoint.pos[2] << "\n";
                //std::cout << newrightpoint.pos[0] << " " << newrightpoint.pos[2] << "\n";
                //std::cout << lastleftpoint.pos[0] << " " << lastleftpoint.pos[2] << "\n";
                //std::cout << lastrightpoint.pos[0] << " " << lastrightpoint.pos[2] << "\n";

            }

            return -1;


        }


        double trackheight(double givenpos[3]){
            int segnum = getsegnumfrompos(givenpos);
            if (segnum < 1 || segnum > length/spacing){ // the point is not over a section of this track
                return -1024;
            }
            
            point newleftpoint = leftpoints[segnum];
                
            point newrightpoint = rightpoints[segnum];

            point lastleftpoint = leftpoints[segnum-1];

            point lastrightpoint = rightpoints[segnum-1];

            std::vector<point> newpolygon;
            newpolygon.push_back(lastleftpoint);
            newpolygon.push_back(lastrightpoint);
            newpolygon.push_back(newrightpoint);
            newpolygon.push_back(newleftpoint);


            double height;

            double normal[3];
            double vect1[3];
            double vect2[3];
            for (int i = 0;i < 3;i++){
                vect1[i] = (newpolygon[1].pos[i]
                        -newpolygon[0].pos[i]); // get the diff between 0 and 1 points as vect
                vect2[i] = (newpolygon[2].pos[i]
                        -newpolygon[0].pos[i]); // get the diff between 0 and 2 points as vect
            }
            normal[0] = vect1[1]*vect2[2]-vect1[2]*vect2[1];
            normal[1] = vect1[2]*vect2[0]-vect1[0]*vect2[2];
            normal[2] = vect1[0]*vect2[1]-vect1[1]*vect2[0];

            double normlen = sqrt(normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2]);

            double a = normal[0] / normlen;
            double b = normal[1] / normlen;
            double c = normal[2] / normlen;
            double d = -(a*newpolygon[0].pos[0]+b*newpolygon[0].pos[1]+c*newpolygon[0].pos[2]);

            // ax+by+cz+d = 0
            //by = -(ax+cz+d)
            //y = -(ax+cz+d)/b
            height = -(a*givenpos[0]+c*givenpos[2]+d)/b;
            if (isnan(height)){
                return -1024;
            }
            return height;
        }

};

std::vector<road*> maintrack;



class face{ // a list of points in a order
    public:
        std::vector<point>& allpoints;
        std::vector<int> pointnums;
        int colour[3];

        face(
            std::vector<point> &objectpoints,
            std::vector<int> pointindices,
            int facecolour[3]
            ) :
          allpoints(objectpoints),
          pointnums(pointindices),
          isinsideout(false),
          valid(false)
        {
            for (int i=0; i<3; i++) {
                colour[i] = facecolour[i];
            }

            getfacenormal();
        }

        double pointdotnorm(double pointpos[3],double center[3]){
            if (!valid) {
                getfacenormal();
            }
            double pointdist = sqrt(((allpoints[pointnums[0]].pos[0])+center[0]-pointpos[0])*((allpoints[pointnums[0]].pos[0])+center[0]-pointpos[0])
                                    +((allpoints[pointnums[0]].pos[1])+center[1]-pointpos[1])*((allpoints[pointnums[0]].pos[1])+center[1]-pointpos[1])
                                    +((allpoints[pointnums[0]].pos[2])+center[2]-pointpos[2])*((allpoints[pointnums[0]].pos[2])+center[2]-pointpos[2]));
            double thepointdotnorm = 0;
            for (int i = 0;i < 3;i++){
                thepointdotnorm += ((allpoints[pointnums[0]].pos[i])+center[i]-pointpos[i])*normal[i]/pointdist;
            }

            //std::cout << thepointdotnorm << " point dot norm \n";

            return thepointdotnorm;
        }

        void selfrender(double *center) {
            if (pointdotnorm(maincamera.selfpos,center) <= 0){
                std::vector<point> currentpolygon;
                for (int j = 0; j < pointnums.size();j++){// construct list of points of the polygon
                    point nextpoint;
                    nextpoint.pos[0] = allpoints[pointnums[j]].pos[0] + center[0];
                    nextpoint.pos[1] = allpoints[pointnums[j]].pos[1] + center[1];
                    nextpoint.pos[2] = allpoints[pointnums[j]].pos[2] + center[2];
                    currentpolygon.push_back(nextpoint);
                }

                maincamera.preppolygon(currentpolygon,currentpolygon.size());

                

                double brightness = 0.5-pointdotnorm(lightpos,center)/3;
                //std::cout << brightness << " brightness\n";

                SDL_SetRenderDrawColor(renderer, colour[0]*brightness*lightshade[0], colour[1]*brightness*lightshade[1]
                                                , colour[2]*brightness*lightshade[2], 255);
                if (fullzbuffer){
                    mainbuffer.fillpolybuffered(currentpolygon);
                }
                else{
                    mainbuffer.simplebuffer(currentpolygon, colour[0]*brightness, colour[1]*brightness, colour[2]*brightness);
                }
            }
        }

        void invalidate() {
            valid = false;
        }

        void setinsideout(bool insideout) {
            isinsideout = insideout;
            invalidate();
        }

    private:
        double normal[3];
        bool isinsideout;
        bool valid;

        void getfacenormal(){
            //std::cout << allpoints[pointnums[0]].pos[0] << " " << allpoints[pointnums[0]].pos[1] << " " << allpoints[pointnums[0]].pos[2] << "\n";
            //std::cout << allpoints[pointnums[1]].pos[0] << " " << allpoints[pointnums[1]].pos[1] << " " << allpoints[pointnums[1]].pos[2] << "\n";
            //std::cout << allpoints[pointnums[2]].pos[0] << " " << allpoints[pointnums[2]].pos[1] << " " << allpoints[pointnums[2]].pos[2] << "\n";

            double vect1[3];
            double vect2[3];
            for (int i = 0;i < 3;i++){
                vect1[i] = (allpoints[pointnums[1]].pos[i]
                        -allpoints[pointnums[0]].pos[i]); // get the diff between 0 and 1 points as vect
                vect2[i] = (allpoints[pointnums[2]].pos[i]
                        -allpoints[pointnums[0]].pos[i]); // get the diff between 0 and 2 points as vect
            }
            normal[0] = vect1[1]*vect2[2]-vect1[2]*vect2[1];
            normal[1] = vect1[2]*vect2[0]-vect1[0]*vect2[2];
            normal[2] = vect1[0]*vect2[1]-vect1[1]*vect2[0];

            double normdotpos = 0;//make sure normal is same direction as the plane is to origin
            for (int i = 0;i < 3;i++){
                normdotpos += normal[i]*allpoints[pointnums[0]].pos[i];// get pointdotnorm to center which says which side it is on
            }
            if (normdotpos < 0){
                for (int i = 0;i < 3;i++){
                    normal[i] = -normal[i];
                }
            }
            
            if (isinsideout){
                for (int i = 0;i < 3;i++){
                    normal[i] = -normal[i];
                }
                
            }
            
            double normlen = sqrt(normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2]);
            for (int i = 0; i < 3;i++){
                normal[i] = normal[i]/normlen;
            }

            valid = true;

            //std::cout << "normal " << normal[0] << " " << normal[1] << " " << normal[2] << "\n";

        }



};



class object{ // a thing which will be show contains points and connections and how to show them
    public:
        std::vector<point> points;
        std::vector<face> faces; // done through reference (not actual reference) to points
        double pointing[3] = {0,0,1}; // a normalised vector directing starting forwards of {0,0,1}
        double objectup[3] = {0,1,0}; // like pointing but points up at start for roll
        double center[3] = {0,0,0};


        


        void objectrender(){
            //std::cout << "start render\n";
            //std::cout << points[0].pos[0] << " " << points[0].pos[1] << " " << points[0].pos[2] << "\n";
            //std::cout << points[1].pos[0] << " " << points[1].pos[1] << " " << points[1].pos[2] << "\n";
            //std::cout << points[2].pos[0] << " " << points[2].pos[1] << " " << points[2].pos[2] << "\n";
            //std::cout << points[3].pos[0] << " " << points[3].pos[1] << " " << points[3].pos[2] << "\n";

            for (int i = 0; i < faces.size();i++){
                faces[i].selfrender(center);
            }

        }


        void selfrotate(double rotvect[3],double rotation){ // slightly more efficiant for rotating whole object
            double rotquaternion[4];
            rotquaternion[0] = cos((double) rotation/2);
            rotquaternion[1] = rotvect[0]*sin((double) rotation/2);
            rotquaternion[2] = rotvect[1]*sin((double) rotation/2);
            rotquaternion[3] = rotvect[2]*sin((double) rotation/2);
            double revrotquaternion[4];
            revrotquaternion[0] = rotquaternion[0];
            revrotquaternion[1] = -rotquaternion[1];
            revrotquaternion[2] = -rotquaternion[2];
            revrotquaternion[3] = -rotquaternion[3];

            for (int i = 0;i < points.size()+2;i++){// the +2 is for the forwards vector and upwards vector
                double* locq;

                double startquat[4];
                if (i == points.size()){
                    startquat[0] = 0;
                    startquat[1] = (double) pointing[0];
                    startquat[2] = (double) pointing[1];
                    startquat[3] = (double) pointing[2];
                }
                else if (i == points.size() + 1){
                    startquat[0] = 0;
                    startquat[1] = (double) objectup[0];
                    startquat[2] = (double) objectup[1];
                    startquat[3] = (double) objectup[2];
                }
                else{
                    startquat[0] = 0;
                    startquat[1] = (double) points[i].pos[0];
                    startquat[2] = (double) points[i].pos[1];
                    startquat[3] = (double) points[i].pos[2];
                }
                
                


                locq = multiquaternion(rotquaternion,startquat);
                double newquat[4];
                newquat[0] = *(locq+0);
                newquat[1] = *(locq+1);
                newquat[2] = *(locq+2);
                newquat[3] = *(locq+3);

                locq = multiquaternion(newquat,revrotquaternion);
                double endx = *(locq+1);
                double endy = *(locq+2);
                double endz = *(locq+3);
                //std::cout << endx << " " << endy << " " << endz << "\n";

                if (i == points.size()){
                    pointing[0] = endx;
                    pointing[1] = endy;
                    pointing[2] = endz;
                }
                else if (i == points.size() + 1){
                    objectup[0] = endx;
                    objectup[1] = endy;
                    objectup[2] = endz;
                }
                else{
                    points[i].pos[0] = endx;
                    points[i].pos[1] = endy;
                    points[i].pos[2] = endz;
                }

                
                //std::cout << (endx*endx+endy*endy+endz*endz) << "\n";
            }

            for (auto &f: faces) {
                f.invalidate();
            }
        }
};

std::vector<object*> displayobjects;
std::vector<object*> treetemplate;
std::vector<point> treeobjlocations;


class player{
    public:
        double selfpos[3];
        double needtomove[3];
        double playerroll;

        double momentum[3];
        double revspeed;
        double maxspeed;
        double rotspeed;

        double pointing[3];
        double carup[3];

        object playercar;

        std::vector<int> checkpointspassed;
        int totalnumcheckpoints;

        long long timelapstart;
        long long timestartrace;
        std::vector<long long> laptimes;
        int numlaps;

        int courseid;

        int currenttrackid;
        

        player(){
            selfpos[0] = 0;
            selfpos[1] = 0;
            selfpos[2] = 0;

            needtomove[0] = 0;
            needtomove[1] = 0;
            needtomove[2] = 0;

            pointing[0] = 0;
            pointing[1] = 0;
            pointing[2] = 1;

            carup[0] = 0;
            carup[1] = -1;
            carup[2] = 0;

            playerroll = 0;

            momentum[0] = 0;
            momentum[1] = 0;
            momentum[2] = 0;

            rotspeed = 0;

            revspeed = -camspeed/2;
            maxspeed = camspeed;


            totalnumcheckpoints = 0;

            timeval currenttime;
            gettimeofday(&currenttime,NULL);
            timelapstart = currenttime.tv_sec*1000000 + currenttime.tv_usec;
            timestartrace = timelapstart;


            courseid = -1;
            currenttrackid = -1;
            
        }

        void makecar(){ // get the points needed to make the car and face construction
            point carpoint;
            carpoint.pos[0] = 0;
            carpoint.pos[1] = 0;
            carpoint.pos[2] = 64;

            playercar.points.push_back(carpoint); // front

            carpoint.pos[0] = -16;
            carpoint.pos[1] = 0;
            carpoint.pos[2] = -64;

            playercar.points.push_back(carpoint);// back left

            carpoint.pos[0] = 16;
            carpoint.pos[1] = 0;
            carpoint.pos[2] = -64;

            playercar.points.push_back(carpoint);// back right

            carpoint.pos[0] = 0;
            carpoint.pos[1] = -16;
            carpoint.pos[2] = -48;

            playercar.points.push_back(carpoint);// top

            int red[3] = {255, 0, 0};

            face carface1(playercar.points, {0, 1, 3}, red);
            playercar.faces.push_back(carface1);

            face carface2(playercar.points, {0, 2, 3}, red);
            playercar.faces.push_back(carface2);

            face carface3(playercar.points, {1, 2, 3}, red);
            playercar.faces.push_back(carface3);

        }

        void relmove(double x,double y,double z){// relative to player
            needtomove[1] = needtomove[1] + y;// height change not depend on the direction pointing

            needtomove[0] = needtomove[0] + x*pointing[2]+z*pointing[0];
            needtomove[2] = needtomove[2] + x*pointing[0]+z*pointing[2];
        }

        void premove(double x,double y,double z){
            needtomove[0] = needtomove[0] + x;
            needtomove[1] = needtomove[1] + y;
            needtomove[2] = needtomove[2] + z;
            
        }

        void move(){
            

            //std::cout << momentum[0] << " " << momentum[1] << " " << momentum[2] << "\n";

            double momentumdotpoint = momentum[0]*pointing[0]+momentum[1]*pointing[1]+momentum[2]*pointing[2];
            
            momentum[0] = momentumdotpoint*pointing[0];
            momentum[1] = momentumdotpoint*pointing[1];
            momentum[2] = momentumdotpoint*pointing[2];

            premove(momentum[0],momentum[1],momentum[2]); // keep momentum
            //std::cout << momentum[0] << " " << momentum[1] << " " << momentum[2] << "\n";

            selfpos[0] = selfpos[0] + needtomove[0];
            selfpos[1] = selfpos[1] + needtomove[1];
            selfpos[2] = selfpos[2] + needtomove[2];
        }

        void turnrotate(double x, double y){

            if (x == 0){
                rotspeed = rotspeed *0.9;
            }
            else if (x > 0){
                rotspeed = rotspeed + 0.1;
            }
            else if (x < 0){
                rotspeed = rotspeed + 0.1;
            }

            //std::cout << "rotspeed " << rotspeed << "\n";

            double newx = -pointing[2]*sin(x*abs(rotspeed))+pointing[0]*cos(x*abs(rotspeed));
            double newz = pointing[0]*sin(x*abs(rotspeed))+pointing[2]*cos(x*abs(rotspeed));

            //maincamera.rotate(x*abs(rotspeed) , y);


            pointing[0] = newx;
            pointing[2] = newz;

            double pointinglen = sqrt(pointing[0]*pointing[0]+pointing[2]*pointing[2]);

            pointing[0] = pointing[0]/pointinglen;
            pointing[2] = pointing[2]/pointinglen;
            


            //std::cout << "pointing" << pointing[0] << " " << pointing[1] << " " << pointing[2] << "\n";



        }

        void rotate(double x, double y){

            
            

            //maincamera.rotate(x , y);


            //pointing[0] = maincamera.pointing[0];
            //pointing[2] = maincamera.pointing[2];

            double newx = -pointing[2]*sin(x)+pointing[0]*cos(x);
            double newz = pointing[0]*sin(x)+pointing[2]*cos(x);


            double pointinglen = sqrt(newx*newx+newz*newz);

            pointing[0] = newx/pointinglen;
            pointing[2] = newz/pointinglen;
            


            //std::cout << "pointing" << pointing[0] << " " << pointing[1] << " " << pointing[2] << "\n";



        }

        void accelerate(double acceleration){

            double roadtypemodifier = 1;
            for (int i = 0 ; i < maintrack.size(); i++){
                if (maintrack[i]->isontrack(selfpos)){
                    roadtypemodifier = 2;
                }
            }

            acceleration = acceleration * roadtypemodifier;

            momentum[0] = momentum[0] + acceleration*pointing[0];//sin(maincamera.xangle); // x is drift
            momentum[2] = momentum[2] + acceleration*pointing[2];//cos(maincamera.xangle);
            //std::cout << momentum[0] << " " << momentum[1] << " " << momentum[2] << " momentum\n";

        }

        void sideaccelerate(double sideacceleration){

            double roadtypemodifier = 1;
            for (int i = 0 ; i < maintrack.size(); i++){
                if (maintrack[i]->isontrack(selfpos)){
                    roadtypemodifier = 2;
                }
            }

            sideacceleration = sideacceleration * roadtypemodifier;

            momentum[0] = momentum[0] + sideacceleration*pointing[2];//cos(maincamera.xangle);
            momentum[2] = momentum[2] - sideacceleration*pointing[0];//sin(maincamera.xangle);
            //std::cout << momentum[0] << " " << momentum[1] << " " << momentum[2] << " momentum\n";

        }

        void speedcheck(){ // this works out friction and stuff on the car and slows it
            double frictionco = 0.025;
            maxspeed = 16;
            revspeed = 16;
            for (int i = 0 ; i < maintrack.size(); i++){
                if (maintrack[i]->isontrack(selfpos)){
                    frictionco = 0.05;
                    maxspeed = 48;
                    revspeed = 24;
                }
            }

            double veldotforward = 0;
            for (int i = 0; i < 3; i++){
                veldotforward = veldotforward + pointing[i]*momentum[i];
            }

            double speed = sqrt(momentum[0]*momentum[0]/*+momentum[1]*momentum[1]*/+momentum[2]*momentum[2]);
            if (veldotforward < 0){
                speed = -speed;
            }

            frictionco = (frictionco+abs(rotspeed)/20);

            //std::cout << speed << " speed\n";
            if (speed == 0){}
            else if (speed > 0){ // speed is always pos or 0 ignoring the direction
                double newspeed = speed - abs(speed*speed/(maxspeed*maxspeed)+1)*frictionco;

                momentum[0] = momentum[0]*newspeed/speed;
                momentum[1] = momentum[1]*newspeed/speed;
                momentum[2] = momentum[2]*newspeed/speed;
            }
            else{
                double newspeed = speed + abs(speed*speed/(revspeed*revspeed)+1)*frictionco;

                momentum[0] = momentum[0]*newspeed/speed;
                momentum[1] = momentum[1]*newspeed/speed;
                momentum[2] = momentum[2]*newspeed/speed;
            }
            //std::cout << sqrt(momentum[0]*momentum[0]+momentum[2]*momentum[2]) << " new speed\n";
            
            //double momentumdotforward = momentum[0]*pointing[0]+momentum[1]*pointing[1]+momentum[2]*pointing[2];
            //momentum[0] = momentum[0]*momentumdotforward;
            //momentum[1] = momentum[1]*momentumdotforward;
            //momentum[2] = momentum[2]*momentumdotforward;
        }


        void correctcarobject(){

            double flatpointlength = sqrt(playercar.pointing[0]*playercar.pointing[0]+
                                        playercar.pointing[2]*playercar.pointing[2]);// this is the length of the flatened to xz plane pointing so can scale up
            double currentpointrot[3] = {-playercar.pointing[2]/flatpointlength,0/*playercar.pointing[1] for reasons of flatness*/
                                        ,playercar.pointing[0]/flatpointlength}; // this is the playercar.pointing rotated pi/2
            double rotpointdotwanted = 0;// is this rotated pointing forward or backward equivilent to is currentpointing left or right
            rotpointdotwanted = rotpointdotwanted + currentpointrot[0]*pointing[0];// vert not matter
            rotpointdotwanted = rotpointdotwanted + currentpointrot[2]*pointing[2];
            
            double currentpointdotwanted = 0;// is this so i can see if it is totaly backwards for my checks
            currentpointdotwanted = currentpointdotwanted + playercar.pointing[0]*pointing[0];
            currentpointdotwanted = currentpointdotwanted + playercar.pointing[2]*pointing[2];

            double abslensdotandcurrent = (sqrt(playercar.pointing[0]*playercar.pointing[0]+playercar.pointing[2]*playercar.pointing[2])
                                            *sqrt(pointing[0]*pointing[0]+pointing[2]*pointing[2]));
            
            //std::cout << "carangle start pointing " << pointing[0] << " " << pointing[1] << " " << pointing[2] << "\n";
            //std::cout << "carangle start pointing " << playercar.pointing[0] << " " << playercar.pointing[1] << " " << playercar.pointing[2] << "\n";
            //std::cout << "carangle deciders " << currentpointdotwanted << " " << abslensdotandcurrent << "\n";


            double changex=0;
            if (abs(rotpointdotwanted) < 0.01 && currentpointdotwanted > 0){/*if pointing exactly on then it can cause error as it cant do acos(1) which is 0*/}
            else if (rotpointdotwanted > 0) {
                changex = abs(acos(currentpointdotwanted/abslensdotandcurrent));
            } else if (rotpointdotwanted < 0) {
                changex = -abs(acos(currentpointdotwanted/abslensdotandcurrent));
            }

            //std::cout << "carangle change " << changex << "\n";
            //std::cout << "carangle current pointing " << pointing[0] << " " << pointing[1] << " " << pointing[2] << "\n";


            //std::cout << playercar.pointing[0] << " " << playercar.pointing[1] << " " << playercar.pointing[2] << " start pointing\n";

            double startdot = 0;
            for (int i = 0; i < 3;i++){
                startdot = startdot + pointing[i]*playercar.pointing[i];
            }

            if (changex != 0){// yaw
                double rotvect[3];
                rotvect[0] = 0;
                rotvect[1] = -1;
                rotvect[2] = 0;
                playercar.selfrotate(rotvect,changex);

                //std::cout << changex << " yaw\n";
            }

            double changey = 0;
            if (abs(playercar.pointing[1] - pointing[1]) < 0.05){}
            else if (playercar.pointing[1] < pointing[1]){
                changey = 0.025;
            }
            else if (playercar.pointing[1] > pointing[1]){
                changey = -0.025;
            }

            if (changey != 0){// pitch
                double rotvect[3];
                double xzpointlen = sqrt(playercar.pointing[0]*playercar.pointing[0]+playercar.pointing[2]*playercar.pointing[2]);
                // useing angles is inacurate as angle is perfect but the actual direction is a bit off
                rotvect[0] = -playercar.pointing[2]/xzpointlen;
                rotvect[1] = 0;
                rotvect[2] = playercar.pointing[0]/xzpointlen;

                
                playercar.selfrotate(rotvect,changey);
                
                //std::cout << -changey << " pitch\n";

            }


            // get forward crossed with "up" to give a plane which is good for the car to have as up
            // then dot the cars up with this cross to see if it is left or right of the plane then roll
            
            double upcrosspointing[3];// used as simple plane containing up and pointing
            for (int i = 0; i < 3; i++){
                upcrosspointing[i] = (pointing[(i+1)%3]*carup[(i+2)%3])-(pointing[(i+2)%3]*carup[(i+1)%3]);
            }
            double nowdotcross = 0;
            for (int i = 0; i < 3; i++){
                nowdotcross = nowdotcross + playercar.objectup[i]*upcrosspointing[i];
            }
            
            
            double changez = 0;
            if (abs(nowdotcross) < 0.03){/*close enough not to bother*/}
            else if (nowdotcross > 0){
                changez = 0.01;
            }
            else if (nowdotcross < 0){
                changez = -0.01;
            }

            //std::cout << "roll cross " << upcrosspointing[0] << " " << upcrosspointing[1] << " " << upcrosspointing[2] << "\n";
            //std::cout << "roll " << nowdotcross << "\n";

            if (changez != 0){// roll
                double rotvect[3];
                double xzpointlen = sqrt(playercar.pointing[0]*playercar.pointing[0]+playercar.pointing[2]*playercar.pointing[2]);
                // useing angles is inacurate as angle is perfect but the actual direction is a bit off
                rotvect[0] = playercar.pointing[0]/xzpointlen;
                rotvect[1] = 0;
                rotvect[2] = playercar.pointing[2]/xzpointlen;
                playercar.selfrotate(rotvect,changez);

                //std::cout << changez << " roll\n";
            }
            
            
            //std::cout << playercar.pointing[0] << " " << playercar.pointing[1] << " " << playercar.pointing[2] << " end pointing\n";


            double enddot = 0;
            for (int i = 0; i < 3;i++){
                enddot = enddot + pointing[i]*playercar.pointing[i];
            }

            //std::cout << startdot << " " << enddot << " dots\n";
            //std::cout << enddot-startdot << " dots diff\n";

            double newx = atan2(playercar.pointing[0],playercar.pointing[2]);
            double newy = atan2(playercar.pointing[1],playercar.pointing[2]);

            //std::cout << newx << " " << newy << " final angles\n";

        }


        void slopeaccelerate(){

            double slope = 0;
            double sideslope = 0;

            if (currenttrackid != -1){
                if (maintrack[currenttrackid]->isontrack(selfpos)){
                    double height = maintrack[currenttrackid]->trackheight(selfpos);
                    selfpos[1] = height;

                    double differentpos[3] = {selfpos[0]+pointing[0],selfpos[1],selfpos[2]+pointing[2]};
                    double differentheight = maintrack[currenttrackid]->trackheight(differentpos);

                    slope = (differentheight-height)/sqrt(pointing[0]*pointing[0]+pointing[2]*pointing[2]);
                    if (differentheight == -1024){
                        differentheight = height;
                        slope = 0;
                    }
                    

                    pointing[1] = slope;
                    double newlength = sqrt(pointing[0]*pointing[0]+pointing[1]*pointing[1]+pointing[2]*pointing[2]);
                    pointing[0] = pointing[0]/newlength;
                    pointing[1] = pointing[1]/newlength;
                    pointing[2] = pointing[2]/newlength;

                    double parallelpos[3] = {selfpos[0]+pointing[2],selfpos[1],selfpos[2]-pointing[0]};
                    double parallelheight = maintrack[currenttrackid]->trackheight(parallelpos);

                    sideslope = (parallelheight-height)/sqrt(pointing[0]*pointing[0]+pointing[2]*pointing[2]);
                    if (parallelheight == -1024){
                        parallelheight = height;
                        sideslope = 0;
                    }

                    parallelpos[1] = sideslope;
                    parallelpos[0] = pointing[2]*newlength;
                    parallelpos[2] = -pointing[0]*newlength;
                    double parallellength = sqrt(parallelpos[0]*parallelpos[0]+parallelpos[1]*parallelpos[1]+parallelpos[2]*parallelpos[2]);
                    parallelpos[0] = pointing[2]/parallellength;
                    parallelpos[1] = parallelpos[1]/parallellength;
                    parallelpos[2] = -pointing[0]/parallellength;

                    // the cross product of parallelpos and pointing will be the new up

                    
                    for (int i = 0; i < 3; i++){
                        carup[i] = (parallelpos[(i+1)%3]*pointing[(i+2)%3])-(parallelpos[(i+2)%3]*pointing[(i+1)%3]);
                    }
                }
            }

            accelerate(slope);
            sideaccelerate(sideslope);

        }

        void getcurrenttrackid(){
            double minchange = 100000;
            currenttrackid = -1;

            for (int i = 0; i < maintrack.size(); i++){
                if (maintrack[i]->isontrack(selfpos)){
                    //std::cout << "trackid posible" << i << "\n";
                    if (abs(maintrack[i]->trackheight(selfpos)-selfpos[1]) < minchange){
                        minchange = abs(maintrack[i]->trackheight(selfpos)-selfpos[1]);
                        currenttrackid = maintrack[i]->roadid;
                    }
                }
            }

            //std::cout << "trackid " << currenttrackid << "\n";
        }

        void checkpointcheck(){
            if (currenttrackid != -1){
                int checkpointid = maintrack[currenttrackid]->isincheckpoint(selfpos);
                if (checkpointid != -1){
                    bool haspassed = false;
                    for (int i = 0; i < checkpointspassed.size(); i++){
                        if (checkpointspassed[i] == checkpointid){
                            haspassed = true;
                        }
                    }

                    if (!haspassed){
                        checkpointspassed.push_back(checkpointid);
                        std::cout << "checkpoint " << checkpointid << "\n";
                    }

                    if (checkpointid == 0 && checkpointspassed.size() >= totalnumcheckpoints/3*2){
                        checkpointspassed.clear();

                        timeval currenttime;
                        gettimeofday(&currenttime,NULL);

                        long long newtime = currenttime.tv_sec*1000000 + currenttime.tv_usec;

                        //std::cout << "checkpoint difference " << newtime-timelapstart << "\n";

                        laptimes.push_back(newtime-timelapstart);

                        timelapstart = newtime;

                        long long currentmintime = laptimes[0];
                        long long totaltime = laptimes[0];
                        for (int i = 1; i < laptimes.size(); i++){
                            if (laptimes[i] < currentmintime){
                                currentmintime = laptimes[i];
                            }
                            totaltime = totaltime + laptimes[i];

                        }

                        //std::cout << "checkpoint besttime " << currentmintime << "\n";
                        //std::cout << "checkpoint totaltime " << totaltime << "\n";

                    }
                }
            }
        }

        void startrace(){// tells the thing it has started the race
            timeval currenttime;
            gettimeofday(&currenttime,NULL);

            long long newtime = currenttime.tv_sec*1000000 + currenttime.tv_usec;
            

            timelapstart = newtime;
            timestartrace = newtime;
        }


        void correctcamerapos(){

            double changevect[3];
            for (int dimension = 0; dimension < 3;dimension++){
                changevect[dimension] = selfpos[dimension] - maincamera.selfpos[dimension] - maincamera.pointing[dimension]*256;
                
                momentum[dimension] = needtomove[dimension];//keep momentum
                
                needtomove[dimension] = 0;
                
                playercar.center[dimension] = selfpos[dimension];
                //std::cout << "car pos" << playercar.center[dimension] << "\n";
                
            }
            maincamera.move(changevect[0],changevect[1]-32,changevect[2]);

            //std::cout << "camera pos" << maincamera.selfpos[0] << " " << maincamera.selfpos[1] << " " << maincamera.selfpos[2] << "\n";


        }

        void correctcamerapointing(){

            double flatpointlength = sqrt(maincamera.pointing[0]*maincamera.pointing[0]+
                                        maincamera.pointing[2]*maincamera.pointing[2]);// this is the length of the flatened to xz plane pointing so can scale up
            double currentpointrot[3] = {-maincamera.pointing[2]/flatpointlength,0/*playercar.pointing[1] for reasons of flatness*/
                                        ,maincamera.pointing[0]/flatpointlength}; // this is the playercar.pointing rotated pi/2
            double rotpointdotwanted = 0;// is this rotated pointing forward or backward equivilent to is currentpointing left or right
            rotpointdotwanted = rotpointdotwanted + currentpointrot[0]*pointing[0];// vert not matter
            rotpointdotwanted = rotpointdotwanted + currentpointrot[2]*pointing[2];
            
            double currentpointdotwanted = 0;// is this so i can see if it is totaly backwards for my checks
            currentpointdotwanted = currentpointdotwanted + maincamera.pointing[0]*pointing[0];
            currentpointdotwanted = currentpointdotwanted + maincamera.pointing[2]*pointing[2];

            double abslensdotandcurrent = (sqrt(maincamera.pointing[0]*maincamera.pointing[0]+maincamera.pointing[2]*maincamera.pointing[2])
                                            *sqrt(pointing[0]*pointing[0]+pointing[2]*pointing[2]));
            
            
            double changex=0;
            if (abs(rotpointdotwanted) < 0.01){/*if pointing exactly on then it can cause error as it cant do acos(1) which is 0*/
                if (currentpointdotwanted < 0){
                    changex = M_PI;
                }
            }
            else if (rotpointdotwanted > 0) {
                changex = abs(acos(currentpointdotwanted/abslensdotandcurrent));
            } else if (rotpointdotwanted < 0) {
                changex = -abs(acos(currentpointdotwanted/abslensdotandcurrent));
            }

            
            double startdot = 0;
            for (int i = 0; i < 3;i++){
                startdot = startdot + pointing[i]*playercar.pointing[i];
            }

            //std::cout << "camera decisions" << abslensdotandcurrent << " " << currentpointdotwanted << " " << rotpointdotwanted << "\n";

            //std::cout << "camera start pointing" << maincamera.pointing[0] << " " << maincamera.pointing[1] << " " << maincamera.pointing[2] << "\n";

            if (changex != 0){// yaw
                maincamera.rotate(changex/3,0);

                //std::cout << changex << " camera yaw\n";
            }


            //std::cout << "camera pointing" << maincamera.pointing[0] << " " << maincamera.pointing[1] << " " << maincamera.pointing[2] << "\n";


        }



        void update(){

            getcurrenttrackid();

            slopeaccelerate();

            checkpointcheck();


            speedcheck();

            


            move();
            

            turnrotate(0,0);// used to reduce turn speed and update vars

            correctcamerapos();
            correctcamerapointing();
            correctcarobject();

            
            
        }

        void selfrender(){
            playercar.objectrender();

        }



};

player mainplayer;


void renderground(){

    int horizon = dispdist*tan(maincamera.yangle)+dispheight/2; // height not matter as far enough away not matter
    /*bool iscarontrack = false;
    bool carincheckpoint = false;
    for (int i = 0; i < maintrack.size(); i++){
        if (maintrack[i]->isontrack(mainplayer.selfpos)){
            iscarontrack = true;
        }
        if (maintrack[i]->isincheckpoint(mainplayer.selfpos) != -1){
            carincheckpoint = true;
        }
    }
    if (carincheckpoint){
        SDL_SetRenderDrawColor( renderer, 0x00*lightshade[0], 0x00*lightshade[1], 0xff*lightshade[2], 0xFF );
    }
    else if (iscarontrack){*/
    SDL_SetRenderDrawColor( renderer, 0x00*lightshade[0], 0xff*lightshade[1], 0x00*lightshade[2], 0xFF );
    /*}
    else{
        SDL_SetRenderDrawColor( renderer, 0xff*lightshade[0], 0x00*lightshade[1], 0xff*lightshade[2], 0xFF );
    }*/
    for (int i = horizon;i < dispheight;i++){
        SDL_RenderDrawLine(renderer , 0,i,dispwidth,i);
    }

    for (int i = 0; i < maintrack.size(); i++){
        //if (visable){
        SDL_SetRenderDrawColor( renderer, 0x00, 0x00, 0x00, 0xFF );
        maintrack[i]->renderroad();
    }
}


void gettrees(){
    treeobjlocations.clear();
    srand(123);
    point treeloc;
        
    for (int i = 0; i < 200; i++){
        double x = rand()/100000-RAND_MAX/200000;
        double z = rand()/100000-RAND_MAX/200000;

        //std::cout << "trees " << x << " " << z << "\n";

        treeloc.pos[0] = x;
        treeloc.pos[1] = 0;
        treeloc.pos[2] = z;

        bool ontrack = false;
        double treepos[3] = {x,0,z};
        for (int j = 0; j < maintrack.size(); j++){
            if (maintrack[j]->isontrack(treepos)){
                ontrack = true;
            }
        }
        if (!ontrack){
            treeobjlocations.push_back(treeloc);
        }
    }
}


void gettrack(int num){

    mainplayer.courseid = num;
    mainplayer.checkpointspassed.clear();
    mainplayer.laptimes.clear();

    maintrack.clear();
    hyperpoint startpoint;
    hyperpoint midpoint;
    hyperpoint midpoint2;
    hyperpoint endpoint;
    std::vector<hyperpoint> roadpoints;
    
    hyperpoint endtangent;
    hyperpoint starttangent;


    std::fstream trackfile;
    trackfile.open("RacecaR_tracks.txt");
    std::vector<std::string> trackfilelines;
    
    std::string line;
    std::vector<int> trackstartpoints;

    while (getline(trackfile,line)){
        //std::cout << line << " file\n";
        if (line.length() != 0){
            trackfilelines.push_back(line);
        }
        if (line.length() >= 5){
            if (line[0] == '#'){
                std::cout << trackfilelines.size()-1 << "\n";
                trackstartpoints.push_back(trackfilelines.size()-1);
            }
        }
    }
    std::cout << trackfilelines.size() << "track file size\n";
    trackstartpoints.push_back(trackfilelines.size());// endpoint use

    int startline = 0;
    int endline = 0;

    for (int i = 0; i < trackstartpoints.size()-1; i++){
        std::string tracknumstr = trackfilelines[trackstartpoints[i]].substr(6,trackfilelines[trackstartpoints[i]].size()-6);
        if (stoi(tracknumstr) == num){
            startline = trackstartpoints[i];
            endline = trackstartpoints[i+1];
        }

    }

    std::vector<int> seglines;

    for (int i = startline; i < endline; i++){
        if (trackfilelines[i][0] == '/'){
            seglines.push_back(i);
        }
    }
    seglines.push_back(endline);

    for (int i = startline; i < endline; i++){
        if (trackfilelines[i][0] == '.'){// an action to set somthing
            int posequals = 0;
            std::vector<std::string> arguements;
            int startletter;
            int endletter;
            for (int j = 0; j < trackfilelines[i].size(); j++){// look for = to get arguments
                if (posequals != 0){
                    if (trackfilelines[i][j] == ',' || trackfilelines[i][j] == ';'){
                        endletter = j;
                        arguements.push_back(trackfilelines[i].substr(startletter,endletter-startletter));
                        //std::cout << arguements[arguements.size()-1] << " is a arguement\n";
                        startletter = j + 1;
                    }

                }
                if (trackfilelines[i][j] == '='){
                    //std::cout << j << " is where = found\n";
                    posequals = j;
                    startletter = j+1;
                }
            }

            if (trackfilelines[i].substr(0,posequals) == ".selfpos "){
                for (int j = 0; j < 3; j++){
                    mainplayer.selfpos[j] = stod(arguements[j]);
                    //std::cout << stod(arguements[j]) << " selfpos" << j << "\n";
                }
            }

            if (trackfilelines[i].substr(0,posequals) == ".pointing "){
                for (int j = 0; j < 3; j++){
                    mainplayer.pointing[j] = stod(arguements[j]);
                    //std::cout << stod(arguements[j]) << " pointing" << j << "\n";
                }
            }

            if (trackfilelines[i].substr(0,posequals) == ".totalnumcheckpoints "){
                mainplayer.totalnumcheckpoints = stod(arguements[0]);
                //std::cout << stod(arguements[0]) << " totalnumcheckpoints" << "\n";
            }

            if (trackfilelines[i].substr(0,posequals) == ".numlaps "){
                mainplayer.numlaps = stod(arguements[0]);
                //std::cout << stod(arguements[0]) << " numlaps" << "\n";
            }

            if (trackfilelines[i].substr(0,posequals) == ".displayobjects center "){
                for (int j = 0; j < 3; j++){
                    displayobjects[0]->center[j] = stod(arguements[j]);
                    //std::cout << stod(arguements[j]) << " displayobjects center" << j << "\n";
                }
            }

            if (trackfilelines[i].substr(0,posequals) == ".displayobjects selfrotate "){
                double rotvect[3] = {0,-1,0};
                displayobjects[0]->selfrotate(rotvect,stod(arguements[0])*M_PI);
                //std::cout << stod(arguements[0])*M_PI << " displayobjects selfrotate\n";
            }
        }
    }


    for (int i = 0; i < seglines.size()-1; i++){

        if (i != 0){
            startpoint.pos = endpoint.pos;
            starttangent.pos = endtangent.pos;
        }

        double starttangentmultiplier = 1;// change how curve a curve is
        double endtangentmultiplier = 1;

        double roadid = 0;

        for(int j = seglines[i];j < seglines[i+1]; j++){
            int posequals = 0;
            std::vector<std::string> arguements;
            int startletter;
            int endletter;

            for (int k = 0; k < trackfilelines[j].size(); k++){// look for = to get arguments
                if (posequals != 0){
                    if (trackfilelines[j][k] == ',' || trackfilelines[j][k] == ';'){
                        endletter = k;
                        arguements.push_back(trackfilelines[j].substr(startletter,endletter-startletter));
                        //std::cout << arguements[arguements.size()-1] << " is a arguement\n";
                        startletter = k + 1;
                    }

                }
                if (trackfilelines[j][k] == '='){
                    //std::cout << k << " is where = found\n";
                    posequals = k;
                    startletter = k+1;
                }
            }

            if (trackfilelines[j].substr(0,posequals) == "startpoint.pos "){
                for (int k = 0; k < 4; k++){
                    startpoint.pos[k] = stod(arguements[k]);
                    //std::cout << stod(arguements[k]) << " startpoint" << k << "\n";
                }
            }

            if (trackfilelines[j].substr(0,posequals) == "starttangent.pos "){
                for (int k = 0; k < 4; k++){
                    starttangent.pos[k] = stod(arguements[k]);
                    //std::cout << stod(arguements[k]) << " starttangent" << k << "\n";
                }
            }

            if (trackfilelines[j].substr(0,posequals) == "endpoint.pos "){
                for (int k = 0; k < 4; k++){
                    endpoint.pos[k] = stod(arguements[k]);
                    //std::cout << stod(arguements[k]) << " endpoint" << k << "\n";
                }
            }

            if (trackfilelines[j].substr(0,posequals) == "endtangent.pos "){
                for (int k = 0; k < 4; k++){
                    endtangent.pos[k] = stod(arguements[k]);
                    //std::cout << stod(arguements[k]) << " endtangent" << k << "\n";
                }
            }

            if (trackfilelines[j].substr(0,posequals) == "starttangent.mult "){
                starttangentmultiplier = stod(arguements[0]);
                endtangentmultiplier = stod(arguements[0]);
                //std::cout << stod(arguements[0]) << " starttangent.mult" << "\n";
            }

            if (trackfilelines[j].substr(0,posequals) == "roadid "){
                roadid = stod(arguements[0]);
                //std::cout << stod(arguements[0]) << " roadid" << "\n";
            }

        }

        double roadlength = sqrt((endpoint.pos[0]-startpoint.pos[0])*(endpoint.pos[0]-startpoint.pos[0])
                                +(endpoint.pos[1]-startpoint.pos[1])*(endpoint.pos[1]-startpoint.pos[1])+
                                (endpoint.pos[2]-startpoint.pos[2])*(endpoint.pos[2]-startpoint.pos[2]));
        double roadspacing = 64;
        double roadwidth = 256;
        
        roadlength = (int)(roadlength/roadspacing)*roadspacing;// so exact divide
        //std::cout << roadlength << " roadlen \n";

        starttangent.pos[0] = starttangent.pos[0]*starttangentmultiplier;
        starttangent.pos[1] = starttangent.pos[1]*starttangentmultiplier;
        starttangent.pos[2] = starttangent.pos[2]*starttangentmultiplier;
        starttangent.pos[3] = starttangent.pos[3]*starttangentmultiplier;

        endtangent.pos[0] = endtangent.pos[0]*starttangentmultiplier;
        endtangent.pos[1] = endtangent.pos[1]*starttangentmultiplier;
        endtangent.pos[2] = endtangent.pos[2]*starttangentmultiplier;
        endtangent.pos[3] = endtangent.pos[3]*starttangentmultiplier;
        

        roadpoints.clear();
        roadpoints.push_back(startpoint);
        roadpoints.push_back(endpoint);
        roadpoints.push_back(starttangent);
        roadpoints.push_back(endtangent);

        road* roadsegment = new road(roadlength, roadspacing, roadwidth, roadpoints, roadid);
        maintrack.push_back(roadsegment);

        endtangent.pos[0] = endtangent.pos[0]/starttangentmultiplier;
        endtangent.pos[1] = endtangent.pos[1]/starttangentmultiplier;
        endtangent.pos[2] = endtangent.pos[2]/starttangentmultiplier;
        endtangent.pos[3] = endtangent.pos[3]/starttangentmultiplier;
    }

    gettrees();




}


bool startscreen(){
    SDL_Event e;


    int blue = 0xff;
    int green = 0xff;
    int red = 0x00;

    bool started = false;
    gettrack(-1);

    double mainmenucarspeed = 8;
    maincamera.rotate(-0.7 , -0.2);

    while (!started){// start menu stuff
        while ( SDL_PollEvent( &e ) != 0 ){ //basic event thing so can quit
            if( e.type == SDL_QUIT ){
                started = true;
                return true;
            }
            else if( e.type == SDL_KEYDOWN ){
                if (e.key.keysym.sym == SDLK_ESCAPE){
                started = true;
                return true;
                }
            }
        }

        // get key input area
        const Uint8* currentKeyStates = SDL_GetKeyboardState( NULL );

        if( currentKeyStates[ SDL_SCANCODE_SPACE ] ){
            //std::cout << "START\n";
            started = true;
        }

        

        // all the main physics
        if (mainplayer.selfpos[2] > 6144){
            mainplayer.selfpos[2] = -7680;
        }
        mainplayer.momentum[0] = 0;
        mainplayer.momentum[1] = 0;
        mainplayer.momentum[2] = 0;

        mainplayer.relmove(0 , 0 , mainmenucarspeed);
        mainplayer.move();
        mainplayer.correctcamerapos();

        SDL_SetRenderDrawColor( renderer, red*lightshade[0], green*lightshade[1], blue*lightshade[2], 0xFF );
        SDL_FillRect( screenSurface, NULL, SDL_MapRGB (screenSurface->format,red,green,blue )); // set background


        SDL_RenderClear( renderer ); // start rendering objects
        mainbuffer.newframe();


        renderground();


        mainplayer.selfrender();
        for (int i = 0; i < displayobjects.size(); i++){// all one time objects
            displayobjects[i]->objectrender();
        }

        for (int i = 0; i < treeobjlocations.size(); i++){
            treetemplate[i%5]->center[0] = treeobjlocations[i].pos[0];
            treetemplate[i%5]->center[1] = treeobjlocations[i].pos[1];
            treetemplate[i%5]->center[2] = treeobjlocations[i].pos[2];
            treetemplate[i%5]->objectrender();
        }

        mainbuffer.showlisted();
        

        std::string hellostring = "racecar";
        SDL_SetRenderDrawColor(renderer ,0x00 ,0x00 ,0x00 ,0xff);
        mainwordwriter.writechars(-dispwidth/2+32,128,64,hellostring,4);
        std::string numstring = "space to continue";
        mainwordwriter.writechars(-dispwidth/2+256 , -128 , 32, numstring,2);

        SDL_RenderPresent( renderer ); //update renderer ??


        timeval currenttime;
        gettimeofday(&currenttime,NULL);
        long long thisframetime = currenttime.tv_sec*1000000 + currenttime.tv_usec;
        if (( 16666 - (thisframetime-lastframetime)) /1000.0 > 0){
            SDL_Delay(( 16666 - (thisframetime-lastframetime)) /1000.0);
        }
        gettimeofday(&currenttime,NULL);
        lastframetime = currenttime.tv_sec*1000000 + currenttime.tv_usec;

    }

    mainplayer.rotate(0.7,-0.05);// also undo previous camangle
    mainplayer.update();

    return false;

}

bool finishedrace(){
    SDL_Event e;


    int blue = 0xff;
    int green = 0xff;
    int red = 0x00;

    lightshade[0] = 0.6;
    lightshade[1] = 0.6;
    lightshade[2] = 0.6;


    bool quit = false;

    std::vector<std::string> stringlaptimes;
    int bestlap = 0;

    std::string mins;
    std::string secs;
    std::string milsecs;

    long long totalracetime = 0;

    for (int i = 0; i < mainplayer.laptimes.size(); i++){
        totalracetime = totalracetime + mainplayer.laptimes[i];

        mins = std::to_string((mainplayer.laptimes[i]/1000000)/60);
        secs = std::to_string((mainplayer.laptimes[i]/1000000)%60);
        milsecs = std::to_string((mainplayer.laptimes[i]/1000)%1000);
        std::string istr = std::to_string(i+1) + " ";

        secs = std::string((2-secs.length()),'0') +secs;
        milsecs = std::string((3-milsecs.length()),'0') +milsecs;

        std::string laptimestring = "lap " + istr + mins + ":" + secs + "." + milsecs;

        stringlaptimes.push_back(laptimestring);

        if (mainplayer.laptimes[i] < mainplayer.laptimes[bestlap]){
            bestlap = i;
        }


    }

    mins = std::to_string((totalracetime/1000000)/60);
    secs = std::to_string((totalracetime/1000000)%60);
    milsecs = std::to_string((totalracetime/1000)%1000);

    secs = std::string((2-secs.length()),'0') +secs;
    milsecs = std::string((3-milsecs.length()),'0') +milsecs;

    std::string totalracetimestring = "total " + mins + ":" + secs + "." + milsecs;




    std::vector<std::string> contents;

    std::fstream pbfiles;
    pbfiles.open("RacecaR_pbs_file.txt",std::fstream::in);
    //std::cout << pbfiles << " file\n";

    std::string line;

    while (getline(pbfiles,line)){
        //std::cout << line << " file\n";
        contents.push_back(line);
    }

    pbfiles.close();

    long pbtime = std::stoi(contents[mainplayer.courseid]);

    mins = std::to_string((pbtime/1000000)/60);
    secs = std::to_string((pbtime/1000000)%60);
    milsecs = std::to_string((pbtime/1000)%1000);

    secs = std::string((2-secs.length()),'0') +secs;
    milsecs = std::string((3-milsecs.length()),'0') +milsecs;

    std::string pbstring = "best " + mins + ":" + secs + "." + milsecs;

    bool isnewpb;
    pbfiles.open("RacecaR_pbs_file.txt",std::fstream::out | std::fstream::trunc);
    for (int i = 0; i < contents.size(); i++){
        //std::cout << contents[i] << " file\n";
        if (i == mainplayer.courseid && totalracetime < std::stoi(contents[i])){
            pbfiles << totalracetime << "\n";
            isnewpb = true;
        }
        else{
            pbfiles << contents[i] << "\n";
        }
        
    }

    pbfiles.close();




    while (!quit){
        while ( SDL_PollEvent( &e ) != 0 ){ //basic event thing so can quit
            if( e.type == SDL_QUIT ){
                quit = true;
                return true;
            }
            else if( e.type == SDL_KEYDOWN ){
                if (e.key.keysym.sym == SDLK_ESCAPE){
                quit = true;
                return true;
                }
            }
        }

        // get key input area
        const Uint8* currentKeyStates = SDL_GetKeyboardState( NULL );

        if( currentKeyStates[ SDL_SCANCODE_SPACE ] ){
            std::cout << "continue\n";
            quit = true;
        }

        // all the main physics
        mainplayer.update();
        

        SDL_SetRenderDrawColor( renderer, red*lightshade[0], green*lightshade[1], blue*lightshade[2], 0xFF );
        SDL_FillRect( screenSurface, NULL, SDL_MapRGB (screenSurface->format,red,green,blue )); // set background


        SDL_RenderClear( renderer ); // start rendering objects
        mainbuffer.newframe();


        renderground();

        for (int i = 0; i < displayobjects.size(); i++){// all one time objects
            displayobjects[i]->objectrender();
        }

        for (int i = 0; i < treeobjlocations.size(); i++){
            treetemplate[i%5]->center[0] = treeobjlocations[i].pos[0];
            treetemplate[i%5]->center[1] = treeobjlocations[i].pos[1];
            treetemplate[i%5]->center[2] = treeobjlocations[i].pos[2];
            treetemplate[i%5]->objectrender();
        }


        mainplayer.selfrender();

        mainbuffer.showlisted();

        SDL_SetRenderDrawColor(renderer ,0x00 ,0x00 ,0x00 ,0xff);
        
        for (int i = 0; i < stringlaptimes.size(); i++){
            if (i == bestlap){
                mainwordwriter.writechars(-256 , 256-80*i , 32, stringlaptimes[i],4);
                std::string bestlapstring = " best";
                mainwordwriter.writechars(-256 + 32*stringlaptimes[i].length() , 256-80*i , 32, bestlapstring,4);
                
            }
            else{
                mainwordwriter.writechars(-256 , 256-80*i , 32, stringlaptimes[i],4);
            }
            
        }

        mainwordwriter.writechars(-256 , -160 , 48, totalracetimestring,4);
        mainwordwriter.writechars(-256+48 , -288 , 48, pbstring, 4);


        SDL_RenderPresent( renderer ); //update renderer ??

        timeval currenttime;
        gettimeofday(&currenttime,NULL);
        long long thisframetime = currenttime.tv_sec*1000000 + currenttime.tv_usec;
        if (( 16666 - (thisframetime-lastframetime)) /1000.0 > 0){
                SDL_Delay(( 16666 - (thisframetime-lastframetime)) /1000.0);
        }
        gettimeofday(&currenttime,NULL);
        lastframetime = currenttime.tv_sec*1000000 + currenttime.tv_usec;

    }

    lightshade[0] = 1;
    lightshade[1] = 1;
    lightshade[2] = 1;

    gettrack(mainplayer.courseid+1);

    return false;

}



bool startup(){ // basic set up


    window = SDL_CreateWindow( "RacecaR", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, dispwidth, dispheight, SDL_WINDOW_SHOWN );
    std::cout << window;
    if( window == NULL ){
        std::cout << "Window could not be created! SDL_Error: " << SDL_GetError() << "\n";
        return false;
    }
    renderer = SDL_CreateRenderer( window, -1, SDL_RENDERER_ACCELERATED );
    if( renderer == NULL ){
        std::cout << "renderer could not be created! SDL_Error: " << SDL_GetError() << "\n";
        return false;
    }


    return true;

}

void shutdown(){ //basic closing thing
    //destroy renderer
    SDL_DestroyRenderer( renderer );

    //Destroy window
    SDL_DestroyWindow( window );

    //Quit SDL subsystems
    SDL_Quit();

}



int main(int argc, char **argv)
{
    if (startup())
    {
        screenSurface = SDL_GetWindowSurface( window );
        SDL_SetRenderDrawColor( renderer, 0xFF*lightshade[0], 0xFF*lightshade[1], 0xFF*lightshade[2], 0xFF );

        bool quit = false;
        SDL_Event e;

        int blue = 0xff;
        int green = 0xff;
        int red = 0x00;
        SDL_UpdateWindowSurface( window );

        mainwordwriter.makeletters();
        mainplayer.makecar();

        point objectpoint;
        object racestartobject;

        objectpoint.pos[0] = -256;
        objectpoint.pos[1] = 0;
        objectpoint.pos[2] = -16;
        racestartobject.points.push_back(objectpoint);

        objectpoint.pos[0] = -256;
        objectpoint.pos[1] = 0;
        objectpoint.pos[2] = 16;
        racestartobject.points.push_back(objectpoint);

        objectpoint.pos[0] = -256-16;
        objectpoint.pos[1] = 0;
        objectpoint.pos[2] = -16;
        racestartobject.points.push_back(objectpoint);

        objectpoint.pos[0] = -256-16;
        objectpoint.pos[1] = 0;
        objectpoint.pos[2] = 16;
        racestartobject.points.push_back(objectpoint);



        objectpoint.pos[0] = 256;
        objectpoint.pos[1] = 0;
        objectpoint.pos[2] = -16;
        racestartobject.points.push_back(objectpoint);

        objectpoint.pos[0] = 256;
        objectpoint.pos[1] = 0;
        objectpoint.pos[2] = 16;
        racestartobject.points.push_back(objectpoint);

        objectpoint.pos[0] = 256+16;
        objectpoint.pos[1] = 0;
        objectpoint.pos[2] = -16;
        racestartobject.points.push_back(objectpoint);

        objectpoint.pos[0] = 256+16;
        objectpoint.pos[1] = 0;
        objectpoint.pos[2] = 16;
        racestartobject.points.push_back(objectpoint);




        objectpoint.pos[0] = -256;
        objectpoint.pos[1] = -192;
        objectpoint.pos[2] = -16;
        racestartobject.points.push_back(objectpoint);

        objectpoint.pos[0] = -256;
        objectpoint.pos[1] = -192;
        objectpoint.pos[2] = 16;
        racestartobject.points.push_back(objectpoint);

        objectpoint.pos[0] = -256-16;
        objectpoint.pos[1] = -192-32;
        objectpoint.pos[2] = -16;
        racestartobject.points.push_back(objectpoint);

        objectpoint.pos[0] = -256-16;
        objectpoint.pos[1] = -192-32;
        objectpoint.pos[2] = 16;
        racestartobject.points.push_back(objectpoint);



        objectpoint.pos[0] = 256;
        objectpoint.pos[1] = -192;
        objectpoint.pos[2] = -16;
        racestartobject.points.push_back(objectpoint);

        objectpoint.pos[0] = 256;
        objectpoint.pos[1] = -192;
        objectpoint.pos[2] = 16;
        racestartobject.points.push_back(objectpoint);

        objectpoint.pos[0] = 256+16;
        objectpoint.pos[1] = -192-32;
        objectpoint.pos[2] = -16;
        racestartobject.points.push_back(objectpoint);

        objectpoint.pos[0] = 256+16;
        objectpoint.pos[1] = -192-32;
        objectpoint.pos[2] = 16;
        racestartobject.points.push_back(objectpoint);

        int black[3] = {0,0,0};
        int white[3] = {255,255,255};


        //+1 for back  +2 for outside +4 for side +8 for top


        face startface1(racestartobject.points, {0, 2, 10, 8}, black);
        racestartobject.faces.push_back(startface1);

        face startface2(racestartobject.points, {1, 3, 11, 9}, black);
        racestartobject.faces.push_back(startface2);

        face startface3(racestartobject.points, {2, 3, 11, 10}, black);
        racestartobject.faces.push_back(startface3);

        face startface4(racestartobject.points, {0, 1, 9, 8}, black);
        startface4.setinsideout(true);
        racestartobject.faces.push_back(startface4);


        face startface5(racestartobject.points, {4, 6, 14, 12}, black);
        racestartobject.faces.push_back(startface5);

        face startface6(racestartobject.points, {5, 7, 15, 13}, black);
        racestartobject.faces.push_back(startface6);

        face startface7(racestartobject.points, {6, 7, 15, 14}, black);
        racestartobject.faces.push_back(startface7);

        face startface8(racestartobject.points, {4, 5, 13, 12}, black);
        startface8.setinsideout(true);
        racestartobject.faces.push_back(startface8);


        face startface9(racestartobject.points, {8, 9, 13, 12}, white);
        startface9.setinsideout(true);
        racestartobject.faces.push_back(startface9);

        face startface10(racestartobject.points, {10, 11, 15, 14}, white);
        racestartobject.faces.push_back(startface10);

        face startface11(racestartobject.points, {8, 10, 14, 12}, white);
        racestartobject.faces.push_back(startface11);

        face startface12(racestartobject.points, {9, 11, 15, 13}, white);
        racestartobject.faces.push_back(startface12);



        displayobjects.push_back(&racestartobject);


        object tree1;
        treetemplate.push_back(&tree1);

        objectpoint.pos[0] = 0;
        objectpoint.pos[1] = 0;
        objectpoint.pos[2] = 16;
        treetemplate[0]->points.push_back(objectpoint);

        objectpoint.pos[0] = sqrt(3)*16/2;
        objectpoint.pos[1] = 0;
        objectpoint.pos[2] = -16/2;
        treetemplate[0]->points.push_back(objectpoint);

        objectpoint.pos[0] = -sqrt(3)*16/2;
        objectpoint.pos[1] = 0;
        objectpoint.pos[2] = -16/2;
        treetemplate[0]->points.push_back(objectpoint);



        objectpoint.pos[0] = 0;
        objectpoint.pos[1] = -36;
        objectpoint.pos[2] = 16;
        treetemplate[0]->points.push_back(objectpoint);

        objectpoint.pos[0] = sqrt(3)*16/2;
        objectpoint.pos[1] = -36;
        objectpoint.pos[2] = -16/2;
        treetemplate[0]->points.push_back(objectpoint);

        objectpoint.pos[0] = -sqrt(3)*16/2;
        objectpoint.pos[1] = -36;
        objectpoint.pos[2] = -16/2;
        treetemplate[0]->points.push_back(objectpoint);



        objectpoint.pos[0] = 0;
        objectpoint.pos[1] = -32;
        objectpoint.pos[2] = 48;
        treetemplate[0]->points.push_back(objectpoint);

        objectpoint.pos[0] = sqrt(3)*48/2;
        objectpoint.pos[1] = -32;
        objectpoint.pos[2] = -48/2;
        treetemplate[0]->points.push_back(objectpoint);

        objectpoint.pos[0] = -sqrt(3)*48/2;
        objectpoint.pos[1] = -32;
        objectpoint.pos[2] = -48/2;
        treetemplate[0]->points.push_back(objectpoint);

        objectpoint.pos[0] = 0;
        objectpoint.pos[1] = -80;
        objectpoint.pos[2] = 0;
        treetemplate[0]->points.push_back(objectpoint);



        objectpoint.pos[0] = 0;
        objectpoint.pos[1] = -64;
        objectpoint.pos[2] = 32;
        treetemplate[0]->points.push_back(objectpoint);

        objectpoint.pos[0] = sqrt(3)*32/2;
        objectpoint.pos[1] = -64;
        objectpoint.pos[2] = -32/2;
        treetemplate[0]->points.push_back(objectpoint);

        objectpoint.pos[0] = -sqrt(3)*32/2;
        objectpoint.pos[1] = -64;
        objectpoint.pos[2] = -32/2;
        treetemplate[0]->points.push_back(objectpoint);

        objectpoint.pos[0] = 0;
        objectpoint.pos[1] = -96;
        objectpoint.pos[2] = 0;
        treetemplate[0]->points.push_back(objectpoint);

        int brown[3] = {64,32,32};
        int forestgreen[3] = {0,128,0};

        face treeface1(treetemplate[0]->points, {0,1,4,3} ,brown);
        treetemplate[0]->faces.push_back(treeface1);

        face treeface2(treetemplate[0]->points, {1,2,5,4} ,brown);
        treetemplate[0]->faces.push_back(treeface2);

        face treeface3(treetemplate[0]->points, {2,0,3,5} ,brown);
        treetemplate[0]->faces.push_back(treeface3);


        face treeface4(treetemplate[0]->points, {6,7,9} ,forestgreen);
        treetemplate[0]->faces.push_back(treeface4);

        face treeface5(treetemplate[0]->points, {7,8,9} ,forestgreen);
        treetemplate[0]->faces.push_back(treeface5);

        face treeface6(treetemplate[0]->points, {8,6,9} ,forestgreen);
        treetemplate[0]->faces.push_back(treeface6);


        face treeface7(treetemplate[0]->points, {10,11,13} ,forestgreen);
        treetemplate[0]->faces.push_back(treeface7);

        face treeface8(treetemplate[0]->points, {11,12,13} ,forestgreen);
        treetemplate[0]->faces.push_back(treeface8);

        face treeface9(treetemplate[0]->points, {12,10,13} ,forestgreen);
        treetemplate[0]->faces.push_back(treeface9);


        point treeloc;
        treeloc.pos[0] = 0;
        treeloc.pos[1] = 0;
        treeloc.pos[2] = 0;
        treeobjlocations.push_back(treeloc);


        for (int i = 0; i < 4;i++){
            treetemplate.push_back(new object);
            for (int j = 0; j < 14; j++){
                objectpoint.pos[0] = treetemplate[0]->points[j].pos[0]*(i+2);
                objectpoint.pos[1] = treetemplate[0]->points[j].pos[1]*(i+2);
                objectpoint.pos[2] = treetemplate[0]->points[j].pos[2]*(i+2);
                treetemplate[i+1]->points.push_back(objectpoint);
                //std::cout << objectpoint.pos[0] << " " << objectpoint.pos[1] << " " << objectpoint.pos[2] << "\n";
            }
            //std::cout << "\n";

            for (int j = 0;j < 9; j++){
                face treefacej(treetemplate[i+1]->points, treetemplate[0]->faces[j].pointnums, treetemplate[0]->faces[j].colour); // almost a copy of original
                treetemplate[i+1]->faces.push_back(treefacej);
            }
        }


        timeval currenttime;
        gettimeofday(&currenttime,NULL);

        lastframetime = currenttime.tv_sec*1000000 + currenttime.tv_usec;




        if (startscreen()){ // the title screen
            quit = true;
        }


        gettrack(0);

        bool started = false;//true;
        bool finished = false;



        while (!quit) { //main loop


            while ( SDL_PollEvent( &e ) != 0 ){ //basic event thing so can quit
                if( e.type == SDL_QUIT ){
                    quit = true;
                }
                else if( e.type == SDL_KEYDOWN ){
                    if (e.key.keysym.sym == SDLK_ESCAPE){
                    quit = true;
                    }
                }
            }
            // get key input area
            const Uint8* currentKeyStates = SDL_GetKeyboardState( NULL );
            
            if( currentKeyStates[ SDL_SCANCODE_W ] ){
                //std::cout << "W";
                mainplayer.accelerate(0.1);}
            if( currentKeyStates[ SDL_SCANCODE_S ] ){
                //std::cout << "S";
                mainplayer.accelerate(-0.1);}
            /*if( currentKeyStates[ SDL_SCANCODE_A ] ){// strafe in car hard
                std::cout << "A";
                mainplayer.sideaccelerate(-0.25);}
            if( currentKeyStates[ SDL_SCANCODE_D ] ){
                std::cout << "D";
                mainplayer.sideaccelerate(0.25);}*/

            if( currentKeyStates[ SDL_SCANCODE_LEFT ] ){
                //std::cout << "LEFT";
                mainplayer.turnrotate(camrotspeed,0);}
            if( currentKeyStates[ SDL_SCANCODE_RIGHT ] ){
                //std::cout << "RIGHT";
                mainplayer.turnrotate(-camrotspeed,0);}
            /*if( currentKeyStates[ SDL_SCANCODE_DOWN ] ){
                std::cout << "DOWN";
                mainplayer.rotate(0,-camrotspeed);}
            if( currentKeyStates[ SDL_SCANCODE_UP ] ){
                std::cout << "UP";
                mainplayer.rotate(0,camrotspeed);}*/




            // all the main physics
            if (started){
                mainplayer.update();
            }
            else{
                mainplayer.needtomove[0] = 0;
                mainplayer.needtomove[1] = 0;
                mainplayer.needtomove[2] = 0;
                mainplayer.momentum[0] = 0;
                mainplayer.momentum[1] = 0;
                mainplayer.momentum[2] = 0;
                mainplayer.update();
            }


            SDL_SetRenderDrawColor( renderer, red*lightshade[0], green*lightshade[1], blue*lightshade[2], 0xFF );
            SDL_FillRect( screenSurface, NULL, SDL_MapRGB (screenSurface->format,red,green,blue )); // set background


            SDL_RenderClear( renderer ); // start rendering objects
            mainbuffer.newframe();


            renderground();


            mainplayer.selfrender();

            for (int i = 0; i < displayobjects.size(); i++){// all one time objects
                displayobjects[i]->objectrender();
            }
            for (int i = 0; i < treeobjlocations.size(); i++){
                treetemplate[i%5]->center[0] = treeobjlocations[i].pos[0];
                treetemplate[i%5]->center[1] = treeobjlocations[i].pos[1];
                treetemplate[i%5]->center[2] = treeobjlocations[i].pos[2];
                treetemplate[i%5]->objectrender();
            }

            mainbuffer.showlisted();

            for (int i = 0; i < maintrack.size(); i++){// mini map after everything else
                //if (visable){
                SDL_SetRenderDrawColor( renderer, 0x00, 0x00, 0x00, 0xFF );
                maintrack[i]->minimaprender();
            }
            

            timeval currenttime;
            gettimeofday(&currenttime,NULL);

            long long nowtime = currenttime.tv_sec*1000000 + currenttime.tv_usec;
            long long timedif = nowtime - mainplayer.timestartrace;
            long long timesincestartlap = nowtime - mainplayer.timelapstart;
            if (!started){
                if (timedif > 3000000){
                    started = true;
                    mainplayer.startrace();
                }
                else{
                    
                    std::string countdown = std::to_string(3-(timedif/1000000)%60);
                    SDL_SetRenderDrawColor(renderer ,0x00 ,0x00 ,0x00 ,0xff);
                    mainwordwriter.writechars(-32 , 0 , 64, countdown,4);

                    timedif = 0;
                    timesincestartlap = 0;

                }
                
            }
            std::string mins = std::to_string((timedif/1000000)/60);
            std::string secs = std::to_string((timedif/1000000)%60);
            std::string milsecs = std::to_string((timedif/1000)%1000);

            secs = std::string((2-secs.length()),'0') +secs;
            milsecs = std::string((3-milsecs.length()),'0') +milsecs;

            std::string totalracetimestring = "time " + mins + ":" + secs + "." + milsecs;

            SDL_SetRenderDrawColor(renderer ,0x00 ,0x00 ,0x00 ,0xff);
            mainwordwriter.writechars(-dispwidth/2+256 , dispheight/2-48 , 32, totalracetimestring,2);


            //std::cout << totalracetimestring << " HUD \n";



            if (timesincestartlap < 2000000 && mainplayer.laptimes.size() != 0){// show prev lap time for a short time just after finish lap except at start
                long long prevlaptime = mainplayer.laptimes[mainplayer.laptimes.size()-1];// last lap time

                mins = std::to_string((prevlaptime/1000000)/60);
                secs = std::to_string((prevlaptime/1000000)%60);
                milsecs = std::to_string((prevlaptime/1000)%1000);

                secs = std::string((2-secs.length()),'0') +secs;
                milsecs = std::string((3-milsecs.length()),'0') +milsecs;

                totalracetimestring = "time " + mins + ":" + secs + "." + milsecs;

                SDL_SetRenderDrawColor(renderer ,0x00 ,0x00 ,0x00 ,0xff);
                mainwordwriter.writechars(-dispwidth/2+256 , dispheight/2-128 , 16, totalracetimestring,2);


                //std::cout << totalracetimestring << " HUD \n";

                
            }

            if (mainplayer.laptimes.size() == mainplayer.numlaps && !finished){// finished race only on first detection
                std::string finishedstring = "finished";
                finished = true;
                SDL_SetRenderDrawColor(renderer ,0x00 ,0x00 ,0x00 ,0xff);
                mainwordwriter.writechars(0 , 0 , 32, finishedstring,2);

                if (finishedrace()){
                    quit = true;
                }
                else{
                    finished = false;
                    started = false;
                    mainplayer.startrace();// reset timer for countdown
                }

            }

            //mainplayer.startrace();// used to work out time per sec

            double scaler = 32/2048.0;
            

            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            SDL_RenderDrawPoint(renderer, mainplayer.selfpos[0]*scaler+dispwidth*3/4 -1, mainplayer.selfpos[2]*scaler+dispheight*3/4 -1);
            SDL_RenderDrawPoint(renderer, mainplayer.selfpos[0]*scaler+dispwidth*3/4 -1, mainplayer.selfpos[2]*scaler+dispheight*3/4);
            SDL_RenderDrawPoint(renderer, mainplayer.selfpos[0]*scaler+dispwidth*3/4 -1, mainplayer.selfpos[2]*scaler+dispheight*3/4 +1);
            SDL_RenderDrawPoint(renderer, mainplayer.selfpos[0]*scaler+dispwidth*3/4, mainplayer.selfpos[2]*scaler+dispheight*3/4 -1);
            SDL_RenderDrawPoint(renderer, mainplayer.selfpos[0]*scaler+dispwidth*3/4, mainplayer.selfpos[2]*scaler+dispheight*3/4);
            SDL_RenderDrawPoint(renderer, mainplayer.selfpos[0]*scaler+dispwidth*3/4, mainplayer.selfpos[2]*scaler+dispheight*3/4 +1);
            SDL_RenderDrawPoint(renderer, mainplayer.selfpos[0]*scaler+dispwidth*3/4 +1, mainplayer.selfpos[2]*scaler+dispheight*3/4 -1);
            SDL_RenderDrawPoint(renderer, mainplayer.selfpos[0]*scaler+dispwidth*3/4 +1, mainplayer.selfpos[2]*scaler+dispheight*3/4);
            SDL_RenderDrawPoint(renderer, mainplayer.selfpos[0]*scaler+dispwidth*3/4 +1, mainplayer.selfpos[2]*scaler+dispheight*3/4 +1);

            SDL_RenderDrawPoint(renderer, dispwidth / 2, dispheight/2);

            SDL_RenderPresent( renderer ); //update renderer ??
            

            gettimeofday(&currenttime,NULL);
            long long thisframetime = currenttime.tv_sec*1000000 + currenttime.tv_usec;
            std::cout << ( 16666 - (thisframetime-lastframetime)) /1000.0 << "\n";//16666 as 16.666 ms *1000
            if (( 16666 - (thisframetime-lastframetime)) /1000.0 > 0){
                SDL_Delay(( 16666 - (thisframetime-lastframetime)) /1000.0);
            }
            gettimeofday(&currenttime,NULL);
            std::cout << thisframetime-lastframetime << "\n";
            lastframetime = currenttime.tv_sec*1000000 + currenttime.tv_usec;
            std::cout << lastframetime << "\n";


        }
    }

    shutdown();

	return 0;
}