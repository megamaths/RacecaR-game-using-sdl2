#include <iostream>
#include <SDL2/SDL.h>
#include <cmath>
#include <vector>
#define _USE_MATH_DEFINES

const int dispwidth = 1080;
const int dispheight = 720;
const int dispdist = 1024;

const double camspeed = 16;
const double camrotspeed = 0.025;


//The window we'll be rendering to
SDL_Window* window = NULL;

//The surface contained by the window
SDL_Surface* screenSurface = NULL;

//The window renderer
SDL_Renderer* renderer = NULL;

struct point{
    double pos[3];
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
                    intersections.push_back(polygon[j].pos[0]+m*(i-polygon[j].pos[1]));
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
                    SDL_RenderDrawLine(renderer,currentpoint+dispwidth/2,i+dispheight/2,nextpoint+dispwidth/2,i+dispheight/2);
                }

                currentpoint = nextpoint;
                parity = !parity;

            }
        }
    }

}




bool isinpoly(std::vector<point> polygon, point querypoint){ /* this works like  fill poly but only on one line 
                                                            it asumes that the polygon is projected onto the zx plane
                                                            in future will change so point project onto polygon first*/


    double i = querypoint.pos[2];

    std::vector<int> intersections;

        
    for (int j = 0; j < polygon.size();j++){
        int otherpoint = (j+1)%polygon.size();
        if ((polygon[j].pos[2] < i && polygon[otherpoint].pos[2] > i) 
            || (polygon[j].pos[2] > i && polygon[otherpoint].pos[2] < i)){// if there is an intersection

            if (abs(polygon[j].pos[2]-polygon[otherpoint].pos[2]) < 1){// almost straight up/ down
                intersections.push_back(polygon[j].pos[0]);
            }
            else{
                double m = (polygon[j].pos[0]-polygon[otherpoint].pos[0])/(polygon[j].pos[2]-polygon[otherpoint].pos[2]);
                intersections.push_back(polygon[j].pos[0]+m*(i-polygon[j].pos[2]));
            }
            
        }
    }

    
    bool parity = false;
    int intersectionssize = intersections.size();
    long currentpoint = -1000000000;
    if (intersectionssize > 1){
        for (int j = 0; j < intersections.size();j++){
            long nextpoint = 1000000000;
            for (int k = 0; k < intersections.size();k++){
                if (intersections[k] < nextpoint && intersections[k] > currentpoint){
                    nextpoint = intersections[k];
                }
                
            }
            if (querypoint.pos[0] >= currentpoint && nextpoint >= querypoint.pos[0]){
                if (parity && nextpoint != 1000000000){// && j+1 != intersections.size()){
                    return true;
                }
                else{
                    return false;
                }
            }

            currentpoint = nextpoint;
            parity = !parity;

        }
    }


    return false;



}


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
            printf("CAMERA anglex %.3f angley %.3f (pos %.2f,%.2f,%.2f)\n", xangle, yangle, selfpos[0], selfpos[1], selfpos[2]);
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
            printf("CAMERA anglex %.3f angley %.3f (pos %.2f,%.2f,%.2f)\n", xangle, yangle, selfpos[0], selfpos[1], selfpos[2]);

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


class road{
    public:

        std::vector<point> points; // points will start by being linear interpolation but will swap for beizer curves
        double length;
        double spacing;
        double width;
        std::vector<point> leftpoints;
        std::vector<point> rightpoints;

        point getposofroad(double ratio){ // this will give position of the road at a given fraction through it 0 giving start and 1 end
            std::vector<point> laststagepoints;
            for (int i = 0;i < points.size()-1;i++){
                std::vector<point> newpoints;
                if (i == 0){
                    for (int j = 0; j < points.size()-1;j++){
                        point newpoint;
                        newpoint.pos[0] = (1-ratio)*points[j].pos[0]+ratio*points[j+1].pos[0];
                        newpoint.pos[2] = (1-ratio)*points[j].pos[2]+ratio*points[j+1].pos[2];
                        newpoints.push_back(newpoint);
                    }
                }
                else{
                    for (int j = 0; j < points.size()-1;j++){
                        point newpoint;
                        newpoint.pos[0] = (1-ratio)*laststagepoints[j].pos[0]+ratio*laststagepoints[j+1].pos[0];
                        newpoint.pos[2] = (1-ratio)*laststagepoints[j].pos[2]+ratio*laststagepoints[j+1].pos[2];
                        newpoints.push_back(newpoint);
                    }
                }

                laststagepoints.clear();
                for (int j = 0;j < newpoints.size();j++){
                    laststagepoints.push_back(newpoints[j]);
                }
            }

            return laststagepoints[0];
        
        }

        void getoutsidepoints(){
            leftpoints.clear();
            rightpoints.clear();


            int segnum = 0;
            for (double dist = 0;dist < length+spacing;dist += spacing){
                segnum = segnum + 1;



                double ratio = dist/length;
                point nextpoint = getposofroad(ratio);


                point slightlyfurtherpoint = getposofroad(ratio + 0.001);
                point normal;
                normal.pos[0] = -(slightlyfurtherpoint.pos[2]-nextpoint.pos[2]);
                normal.pos[2] = (slightlyfurtherpoint.pos[0]-nextpoint.pos[0]);

                double normlen = sqrt(normal.pos[0]*normal.pos[0]+normal.pos[2]*normal.pos[2]);

                normal.pos[0] = width*normal.pos[0]/normlen;
                normal.pos[2] = width*normal.pos[2]/normlen;

                point nextleftpoint;
                nextleftpoint.pos[0] = nextpoint.pos[0]+normal.pos[0];
                nextleftpoint.pos[1] = 0;
                nextleftpoint.pos[2] = nextpoint.pos[2]+normal.pos[2];
                point nextrightpoint;
                nextrightpoint.pos[0] = nextpoint.pos[0]-normal.pos[0];
                nextleftpoint.pos[1] = 0;
                nextrightpoint.pos[2] = nextpoint.pos[2]-normal.pos[2];


                leftpoints.push_back(nextleftpoint);
                rightpoints.push_back(nextrightpoint);

            }

        }

        void renderroad(){

            getoutsidepoints();



            point lastpoint;
            lastpoint.pos[0] = points[0].pos[0];
            lastpoint.pos[1] = 0;
            lastpoint.pos[2] = points[0].pos[2];
            

            point startleftpoint = leftpoints[0];
            
            point startrightpoint = rightpoints[0];
            


            double scaler = 128/length;

            SDL_RenderDrawLine(renderer,(lastpoint.pos[0])*scaler+dispwidth/2,(lastpoint.pos[2])*scaler+dispheight/2,
                                (startleftpoint.pos[0])*scaler+dispwidth/2,(startleftpoint.pos[2])*scaler+dispheight/2);
            SDL_RenderDrawLine(renderer,(lastpoint.pos[0])*scaler+dispwidth/2,(lastpoint.pos[2])*scaler+dispheight/2,
                                (startrightpoint.pos[0])*scaler+dispwidth/2,(startrightpoint.pos[2])*scaler+dispheight/2);


            int segnum = 0;
            for (double dist = spacing;dist < length+spacing;dist += spacing){
                if (segnum%2 == 0){
                    SDL_SetRenderDrawColor( renderer, 0x70, 0x70, 0x70, 0xFF);
                }
                else{
                    SDL_SetRenderDrawColor( renderer, 0x90, 0x90, 0x90, 0xFF);
                }
                segnum = segnum + 1;



                double ratio = dist/length;
                point nextpoint = getposofroad(ratio);

                SDL_RenderDrawLine(renderer,(lastpoint.pos[0])*scaler+dispwidth/2,(lastpoint.pos[2])*scaler+dispheight/2,
                                        (nextpoint.pos[0])*scaler+dispwidth/2,(nextpoint.pos[2])*scaler+dispheight/2);
                
                

                point newleftpoint = leftpoints[segnum];
                
                point newrightpoint = rightpoints[segnum];

                point lastleftpoint = leftpoints[segnum-1];

                point lastrightpoint = rightpoints[segnum-1];

                SDL_RenderDrawLine(renderer,(nextpoint.pos[0])*scaler+dispwidth/2,(nextpoint.pos[2])*scaler+dispheight/2,
                                (newleftpoint.pos[0])*scaler+dispwidth/2,(newleftpoint.pos[2])*scaler+dispheight/2);
                SDL_RenderDrawLine(renderer,(nextpoint.pos[0])*scaler+dispwidth/2,(nextpoint.pos[2])*scaler+dispheight/2,
                                (newrightpoint.pos[0])*scaler+dispwidth/2,(newrightpoint.pos[2])*scaler+dispheight/2);

                
                


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

                if (true){
                    fillpoly(newpolygon);
                }
                

                for (int i = 0;i < newpolygon.size();i++){
                    int otherpoint = (i+1)%newpolygon.size();
                    //std::cout << newpolygon[i].pos[0] << " " << newpolygon[i].pos[1] << "\n";

                    SDL_RenderDrawLine(renderer , newpolygon[i].pos[0]+dispwidth/2, newpolygon[i].pos[1]+dispheight/2
                                        , newpolygon[otherpoint].pos[0]+dispwidth/2 , newpolygon[otherpoint].pos[1]+dispheight/2);

                    
                }
                //std::cout << "\n";


                lastpoint.pos[0] = nextpoint.pos[0];
                lastpoint.pos[1] = 0;
                lastpoint.pos[2] = nextpoint.pos[2];
                

            }
            //std::cout << "\n";
        }

        bool isontrack(double givenpos[3]){// a rough aproximate answer to is a point on the road ignores height

            getoutsidepoints();

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
                    return true;
                }
                //std::cout << segnum << "\n";

                //std::cout << newleftpoint.pos[0] << " " << newleftpoint.pos[2] << "\n";
                //std::cout << newrightpoint.pos[0] << " " << newrightpoint.pos[2] << "\n";
                //std::cout << lastleftpoint.pos[0] << " " << lastleftpoint.pos[2] << "\n";
                //std::cout << lastrightpoint.pos[0] << " " << lastrightpoint.pos[2] << "\n";

            }

            return false;


        }



};

std::vector<road> maintrack;



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

                double lightpos[3];
                lightpos[0] = 128;
                lightpos[1] = -1024;
                lightpos[2] = 0;

                double brightness = 0.5-pointdotnorm(lightpos,center)/2;
                std::cout << brightness << " brightness\n";

                SDL_SetRenderDrawColor(renderer, colour[0]*brightness, colour[1]*brightness, colour[2]*brightness, 255);
        
                fillpoly(currentpolygon);
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
        double center[3];


        


        void objectrender(){
            std::cout << "start render\n";
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

            for (int i = 0;i < points.size();i++){
                double* locq;

                double startquat[4];
                startquat[0] = 0;
                startquat[1] = (double) points[i].pos[0];
                startquat[2] = (double) points[i].pos[1];
                startquat[3] = (double) points[i].pos[2];


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
                points[i].pos[0] = endx;
                points[i].pos[1] = endy;
                points[i].pos[2] = endz;
                //std::cout << (endx*endx+endy*endy+endz*endz) << "\n";
            }

            for (auto &f: faces) {
                f.invalidate();
            }
        }
};




class player{
    public:
        double selfpos[3];
        double needtomove[3];
        double playeranglex;
        double playerangley;

        double carspeed;
        double momentum[3];

        double pointing[3];

        object playercar;
        

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
            playeranglex = 0;
            playerangley = 0;
            carspeed = 0;
            momentum[0] = 0;
            momentum[1] = 0;
            momentum[2] = 0;
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
            carpoint.pos[1] = -32;
            carpoint.pos[2] = -32;

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

            needtomove[0] = needtomove[0] + x*cos(maincamera.xangle)-z*sin(maincamera.xangle);
            needtomove[2] = needtomove[2] + x*sin(maincamera.xangle)+z*cos(maincamera.xangle);
        }

        void premove(double x,double y,double z){
            needtomove[0] = needtomove[0] + x;
            needtomove[1] = needtomove[1] + y;
            needtomove[2] = needtomove[2] + z;
            
        }

        void move(){
            selfpos[0] = selfpos[0] + needtomove[0];
            selfpos[1] = selfpos[1] + needtomove[1];
            selfpos[2] = selfpos[2] + needtomove[2];

            double changevect[3];
            for (int dimension = 0; dimension < 3;dimension++){
                changevect[dimension] = selfpos[dimension] - maincamera.selfpos[dimension] - pointing[dimension]*256;
                
                momentum[dimension] = needtomove[dimension];
                std::cout << needtomove[dimension] << " new momentum\n";
                needtomove[dimension] = 0;
                
                playercar.center[dimension] = selfpos[dimension];
                //std::cout << "car pos" << playercar.center[dimension] << "\n";


                
            }
            maincamera.move(changevect[0],changevect[1],changevect[2]);


            

        }

        void rotate(double x, double y){

            maincamera.rotate(x , y);

            double changex = maincamera.xangle - playeranglex;
            double changey = maincamera.yangle - playerangley;

            if (changex != 0){
                double rotvect[3];
                rotvect[0] = 0;
                rotvect[1] = 1;
                rotvect[2] = 0;
                playercar.selfrotate(rotvect,-changex);
            }
            /*if (changey != 0){
                double rotvect[3];
                rotvect[0] = cos(playeranglex);
                rotvect[1] = 0;
                rotvect[2] = sin(playeranglex);
                playercar.selfrotate(rotvect,changey);

            }*/

            playeranglex = maincamera.xangle;
            playerangley = maincamera.yangle;


            for (int i = 0;i < 3;i++){
                pointing[i] = maincamera.pointing[i];
            }
            pointing[0] = pointing[0];
            pointing[2] = pointing[2];


            //std::cout << "pointing" << pointing[0] << " " << pointing[1] << " " << pointing[2] << "\n";



        }

        void accelerate(double acceleration){

            double roadtypemodifier = 1;
            for (int i = 0 ; i < maintrack.size(); i++){
                if (maintrack[i].isontrack(selfpos)){
                    roadtypemodifier = 2;
                }
            }


            acceleration = acceleration * roadtypemodifier;

            carspeed = carspeed + acceleration;


            momentum[0] = momentum[0] -acceleration*sin(maincamera.xangle);//+x*cos(maincamera.xangle); // x is drift
            momentum[2] = momentum[2] +acceleration*cos(maincamera.xangle);//+x*sin(maincamera.xangle);
            std::cout << momentum[0] << " " << momentum[1] << " " << momentum[2] << " momentum\n";

            double speed = sqrt(momentum[0]*momentum[0]/*+momentum[1]*momentum[1]*/+momentum[2]*momentum[2]);
            if (speed > camspeed*roadtypemodifier){
                momentum[0] = momentum[0] *camspeed/speed;
                momentum[1] = momentum[1] *camspeed/speed;
                momentum[2] = momentum[2] *camspeed/speed;
            }

            if (carspeed > camspeed*roadtypemodifier){
                carspeed = camspeed*roadtypemodifier;
            }

        }

        void speedcheck(){ // this works out friction and stuff on the car and slows it
            double frictionco = 0.001;
            for (int i = 0 ; i < maintrack.size(); i++){
                if (maintrack[i].isontrack(selfpos)){
                    frictionco = 0.002;
                }
            }

            double speed = sqrt(momentum[0]*momentum[0]/*+momentum[1]*momentum[1]*/+momentum[2]*momentum[2]);
            if (speed == 0){

            }
            else if (speed > 0){
                double newspeed = speed - (speed*speed+1)*frictionco;

                momentum[0] = momentum[0]*newspeed/speed;
                momentum[1] = momentum[1]*newspeed/speed;
                momentum[2] = momentum[2]*newspeed/speed;
            }
            else{
                double newspeed = speed + (speed*speed+1)*frictionco;

                momentum[0] = momentum[0]*newspeed/speed;
                momentum[1] = momentum[1]*newspeed/speed;
                momentum[2] = momentum[2]*newspeed/speed;
            }
            

            std::cout << carspeed << " carspeed\n";
            if (carspeed > 0){
                carspeed = carspeed - (carspeed*carspeed+1)*frictionco;
            }
            else{
                carspeed = carspeed + (carspeed*carspeed+1)*frictionco;
            }
            std::cout << carspeed << " carspeed\n";

        }



        void update(){



            speedcheck();

            premove(momentum[0],momentum[1],momentum[2]);


            //relmove(0,0,carspeed);
            move();

            rotate(0,0);

            
            
        }

        void selfrender(){
            playercar.objectrender();

        }



};

player mainplayer;


void renderground(){

    int horizon = dispdist*tan(maincamera.yangle)+dispheight/2; // height not matter as far enough away not matter
    bool iscarontrack = false;
    for (int i = 0; i < maintrack.size(); i++){
        if (maintrack[i].isontrack(mainplayer.selfpos)){
            iscarontrack = true;
        }
    }
    if (iscarontrack){
        SDL_SetRenderDrawColor( renderer, 0x00, 0xff, 0x00, 0xFF );
    }
    else{
        SDL_SetRenderDrawColor( renderer, 0xff, 0x00, 0xff, 0xFF );
    }
    for (int i = horizon;i < dispheight;i++){
        SDL_RenderDrawLine(renderer , 0,i,dispwidth,i);
    }

    for (int i = 0; i < maintrack.size(); i++){
        //if (visable){
        SDL_SetRenderDrawColor( renderer, 0x00, 0x00, 0x00, 0xFF );
        maintrack[i].renderroad();
        
                

    }
}



bool startup(){ // basic set up


    window = SDL_CreateWindow( "SDL Tutorial", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, dispwidth, dispheight, SDL_WINDOW_SHOWN );
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
        SDL_SetRenderDrawColor( renderer, 0xFF, 0xFF, 0xFF, 0xFF );

        bool quit = false;
        SDL_Event e;

        int blue = 0xff;
        int green = 0xff;
        int red = 0x00;
        SDL_UpdateWindowSurface( window );


        road roadsegment;


        point startpoint;
        point midpoint;
        point endpoint;
        
        roadsegment.length = 2048;
        roadsegment.spacing = 64;
        roadsegment.width = 256;
        
        startpoint.pos[0] = 2048;
        startpoint.pos[2] = 0;
        
        midpoint.pos[0] = 2048;
        midpoint.pos[2] = 2048;
        
        endpoint.pos[0] = 0;
        endpoint.pos[2] = 2048;

        roadsegment.points.clear();
        roadsegment.points.push_back(startpoint);
        roadsegment.points.push_back(midpoint);
        roadsegment.points.push_back(endpoint);

        maintrack.push_back(roadsegment);
        
        roadsegment.length = 2048;
        roadsegment.spacing = 64;
        roadsegment.width = 256;
        
        startpoint.pos[0] = 0;
        startpoint.pos[2] = 2048;
        
        midpoint.pos[0] = -2048;
        midpoint.pos[2] = 2048;
        
        endpoint.pos[0] = -2048;
        endpoint.pos[2] = 0;

        roadsegment.points.clear();
        roadsegment.points.push_back(startpoint);
        roadsegment.points.push_back(midpoint);
        roadsegment.points.push_back(endpoint);

        maintrack.push_back(roadsegment);
        
        roadsegment.length = 2048;
        roadsegment.spacing = 64;
        roadsegment.width = 256;
        
        startpoint.pos[0] = -2048;
        startpoint.pos[2] = 0;
        
        midpoint.pos[0] = -2048;
        midpoint.pos[2] = -2048;
        
        endpoint.pos[0] = 0;
        endpoint.pos[2] = -2048;
        roadsegment.points.clear();
        roadsegment.points.push_back(startpoint);
        roadsegment.points.push_back(midpoint);
        roadsegment.points.push_back(endpoint);

        maintrack.push_back(roadsegment);
        
        roadsegment.length = 2048;
        roadsegment.spacing = 64;
        roadsegment.width = 256;
        
        startpoint.pos[0] = 0;
        startpoint.pos[2] = -2048;
        
        midpoint.pos[0] = 2048;
        midpoint.pos[2] = -2048;
        
        endpoint.pos[0] = 2048;
        endpoint.pos[2] = 0;

        roadsegment.points.clear();
        roadsegment.points.push_back(startpoint);
        roadsegment.points.push_back(midpoint);
        roadsegment.points.push_back(endpoint);

        maintrack.push_back(roadsegment);



        mainplayer.makecar();


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
                std::cout << "W";
                mainplayer.accelerate(1);}
            if( currentKeyStates[ SDL_SCANCODE_S ] ){
                std::cout << "S";
                mainplayer.accelerate(-1);}
            if( currentKeyStates[ SDL_SCANCODE_A ] ){
                std::cout << "A";
                mainplayer.relmove(-camspeed,0,0);}
            if( currentKeyStates[ SDL_SCANCODE_D ] ){
                std::cout << "D";
                mainplayer.relmove(camspeed,0,0);}
            if( currentKeyStates[ SDL_SCANCODE_Q ] ){
                std::cout << "Q";
                mainplayer.relmove(0,-camspeed,0);}
            if( currentKeyStates[ SDL_SCANCODE_E ] ){
                std::cout << "E";
                mainplayer.relmove(0,camspeed,0);}

            if( currentKeyStates[ SDL_SCANCODE_LEFT ] ){
                std::cout << "LEFT";
                mainplayer.rotate(camrotspeed,0);}
            if( currentKeyStates[ SDL_SCANCODE_RIGHT ] ){
                std::cout << "RIGHT";
                mainplayer.rotate(-camrotspeed,0);}
            if( currentKeyStates[ SDL_SCANCODE_DOWN ] ){
                std::cout << "DOWN";
                mainplayer.rotate(0,-camrotspeed);}
            if( currentKeyStates[ SDL_SCANCODE_UP ] ){
                std::cout << "UP";
                mainplayer.rotate(0,camrotspeed);}




            // all the main physics
            mainplayer.update();


            SDL_SetRenderDrawColor( renderer, red, green, blue, 0xFF );
            SDL_FillRect( screenSurface, NULL, SDL_MapRGB (screenSurface->format,red,green,blue )); // set background


            SDL_RenderClear( renderer ); // start rendering objects


            renderground();


            mainplayer.selfrender();
            double length = 2048;
            double scaler = 128/length;

            /*SDL_SetRenderDrawColor(renderer , 0, 0, 0, 255); // this shows on the map where it thinks is in the track but is quite slow so not used exept debug
            for (double  i = -3036; i < 3036; i = i + 64){
                for (double j = -3036; j < 3036; j = j + 64){
                    //std::cout << j << "\n";
                    double thispoint[3] = {i , 0 , j};
                    
                    bool isontrack = false;
                    for (int k = 0; k < maintrack.size();k++){
                        if (maintrack[k].isontrack(thispoint)){
                            isontrack = true;
                            //std::cout << i << " " << j << " hey look i found the track\n";
                        }
                    }
                    if (isontrack){
                        SDL_RenderDrawLine(renderer, i*scaler+dispwidth/2, j*scaler+dispheight/2, i*scaler+dispwidth/2 , j*scaler+dispheight/2);
                        //std::cout << i*scaler+dispwidth/2 << " " << j*scaler+dispheight/2 << "\n";
                    }
                }
            }*/


            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            SDL_RenderDrawPoint(renderer, mainplayer.selfpos[0]*scaler+dispwidth / 2, mainplayer.selfpos[2]*scaler+dispheight/2);
            SDL_RenderDrawPoint(renderer, dispwidth / 2, dispheight/2);
            SDL_RenderPresent( renderer ); //update renderer ??
            SDL_Delay(1000/60);




            }
    }

    shutdown();

	return 0;
}