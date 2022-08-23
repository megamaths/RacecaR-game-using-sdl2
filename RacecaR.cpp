#include <iostream>
#include <SDL2/SDL.h>
#include <cmath>
#include <vector>
#include <sys/time.h>
#define _USE_MATH_DEFINES

const int dispwidth = 1080;
const int dispheight = 720;
const int dispdist = 1024;

const double camspeed = 16;
const double camrotspeed = 0.025;

double lightpos[3] = {128, -1024 , 0};


//The window we'll be rendering to
SDL_Window* window = NULL;

//The surface contained by the window
SDL_Surface* screenSurface = NULL;

//The window renderer
SDL_Renderer* renderer = NULL;

struct point{
    double pos[3];
};

struct hyperpoint{
    double pos[4];
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
    for (int i = 0;i < polygon.size();i++){
        int j = (i+1) %polygon.size();

        double side = ((polygon[j].pos[0] - polygon[i].pos[0]) * (querypoint.pos[2] - polygon[i].pos[2]) - 
                        (polygon[j].pos[2] - polygon[i].pos[2]) * (querypoint.pos[0] - polygon[i].pos[0]));


        //std::cout << polygon[i].pos[0] << " " << polygon[i].pos[2] << " point1\n";
        //std::cout << polygon[j].pos[0] << " " << polygon[j].pos[2] << " point2\n";
        //std::cout << querypoint.pos[0] << " " << querypoint.pos[2] << " pointq\n";
        //std::cout << side << "\n";
        if (i == 0){
            firstside = (side > 0);
        }
        if ((side > 0) != firstside){
            if (side != 0){
                return false;
            }
        }
    }
    
    return true;
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
            three.addline(-0.5,0.9 , 0.4,1);
            three.addline(-0.2,0 , 0.4,0);
            three.addline(-0.5,-0.9 , 0.4,-1);
            three.addline(0.4,1 , 0.5,0.5);
            three.addline(0.5,0.5 , 0.4,0);
            three.addline(0.4,-1 , 0.5,-0.5);
            three.addline(0.5,-0.5 , 0.4,0);

            letter four;//4
            four.addline(0.5,-0.2 , -0.5,-0.2);
            four.addline(-0.5,-0.2 , 0.2,1);
            four.addline(0.2,1 , 0.2,-1);

            letter five;//5
            five.addline(0.5,1 , -0.5,1);
            five.addline(-0.5,1 , -0.5,0);
            five.addline(-0.5,0 , 0.4,0.1);
            five.addline(0.4,0.1 , 0.5,-0.2);
            five.addline(0.5,-0.2 , 0.4,-1);
            five.addline(0.4,-1 , -0.5,-0.9);

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
                    SDL_SetRenderDrawColor(renderer , 255,0,0,255);
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
            for (int i = 0; i < newpoints.size(); i++){
                points.push_back(newpoints[i]);
            }
            roadid = id;

            getoutsidepoints();

        }


        hyperpoint getposofroad(double ratio){ // this will give position of the road at a given fraction through it 0 giving start and 1 end
            std::vector<hyperpoint> laststagepoints;
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

            return laststagepoints[0];
        
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
                std::cout << "start track pos" << nextleftpoint.pos[0] << " " << nextleftpoint.pos[1] << " " << nextleftpoint.pos[2] << "\n";
                std::cout << "start track pos" << nextrightpoint.pos[0] << " " << nextrightpoint.pos[1] << " " << nextrightpoint.pos[2] << "\n";
            }
        }

        void renderroad(){

            //getoutsidepoints();



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
                hyperpoint nextpoint = getposofroad(ratio);

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
                

                /*for (int i = 0;i < newpolygon.size();i++){// this draws an outline of the seg but is not cliped so can draw forever
                    int otherpoint = (i+1)%newpolygon.size();
                    //std::cout << newpolygon[i].pos[0] << " " << newpolygon[i].pos[1] << "\n";

                    SDL_RenderDrawLine(renderer , newpolygon[i].pos[0]+dispwidth/2, newpolygon[i].pos[1]+dispheight/2
                                        , newpolygon[otherpoint].pos[0]+dispwidth/2 , newpolygon[otherpoint].pos[1]+dispheight/2);

                    
                }*/
                //std::cout << "\n";


                lastpoint.pos[0] = nextpoint.pos[0];
                lastpoint.pos[1] = nextpoint.pos[1];
                lastpoint.pos[2] = nextpoint.pos[2];
                

            }
            //std::cout << "\n";
        }

        int isincheckpoint(double givenpos[3]){// similar to isontrack but only for first segment returns identification if true else -1

            //getoutsidepoints();

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
            if (segnum == -1){ // the point is not over a section of this track
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

            return height;
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
        double pointing[3] = {0,0,1}; // a normalised vector directing starting forwards of {0,0,1}
        double objectup[3] = {0,1,0}; // like pointing but points up at start for roll
        double center[3] = {0,0,0};


        


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




class player{
    public:
        double selfpos[3];
        double needtomove[3];
        double playeranglex;
        double playerangley;
        double playerroll;

        double momentum[3];
        double revspeed;
        double maxspeed;

        double pointing[3];
        double carup[3];

        object playercar;

        std::vector<int> checkpointspassed;
        int totalnumcheckpoints;

        long long timeatcheckpoint;
        std::vector<long long> laptimes;


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

            playeranglex = 0;
            playerangley = 0;
            playerroll = 0;

            momentum[0] = 0;
            momentum[1] = 0;
            momentum[2] = 0;

            revspeed = -camspeed/2;
            maxspeed = camspeed;


            totalnumcheckpoints = 0;

            timeval currenttime;
            gettimeofday(&currenttime,NULL);
            timeatcheckpoint = currenttime.tv_sec*1000000 + currenttime.tv_usec;


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

            /*needtomove[0] = needtomove[0] + x*cos(maincamera.xangle)-z*sin(maincamera.xangle);
            needtomove[2] = needtomove[2] + x*sin(maincamera.xangle)+z*cos(maincamera.xangle);*/

            needtomove[0] = needtomove[0] + x*pointing[2]+z*pointing[0];
            needtomove[2] = needtomove[2] + x*pointing[0]+z*pointing[2];
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
                changevect[dimension] = selfpos[dimension] - maincamera.selfpos[dimension] - maincamera.pointing[dimension]*256;
                
                momentum[dimension] = needtomove[dimension];
                
                needtomove[dimension] = 0;
                
                playercar.center[dimension] = selfpos[dimension];
                //std::cout << "car pos" << playercar.center[dimension] << "\n";

                
            }
            maincamera.move(changevect[0],changevect[1],changevect[2]);


            

        }

        void rotate(double x, double y){

            maincamera.rotate(x , y);

            double changex = maincamera.xangle - playeranglex;
            

            playeranglex = maincamera.xangle;
            

            pointing[0] = maincamera.pointing[0];
            pointing[2] = maincamera.pointing[2];

            double pointinglen = sqrt(pointing[0]*pointing[0]+pointing[2]*pointing[2]);

            pointing[0] = pointing[0]/pointinglen;
            pointing[2] = pointing[2]/pointinglen;
            


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

            momentum[0] = momentum[0] - acceleration*sin(maincamera.xangle);//+x*cos(maincamera.xangle); // x is drift
            momentum[2] = momentum[2] + acceleration*cos(maincamera.xangle);//+x*sin(maincamera.xangle);
            //std::cout << momentum[0] << " " << momentum[1] << " " << momentum[2] << " momentum\n";

        }

        void sideaccelerate(double sideacceleration){

            double roadtypemodifier = 1;
            for (int i = 0 ; i < maintrack.size(); i++){
                if (maintrack[i].isontrack(selfpos)){
                    roadtypemodifier = 2;
                }
            }

            sideacceleration = sideacceleration * roadtypemodifier;

            momentum[0] = momentum[0] + sideacceleration*cos(maincamera.xangle);
            momentum[2] = momentum[2] + sideacceleration*sin(maincamera.xangle);
            //std::cout << momentum[0] << " " << momentum[1] << " " << momentum[2] << " momentum\n";

        }

        void speedcheck(){ // this works out friction and stuff on the car and slows it
            double frictionco = 0.001;
            for (int i = 0 ; i < maintrack.size(); i++){
                if (maintrack[i].isontrack(selfpos)){
                    frictionco = 0.002;
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
            std::cout << speed << " speed\n";
            if (speed == 0){}
            else if (speed > 0){ // speed is always pos or 0 ignoring the direction
                double newspeed = speed - (speed*speed+1)*frictionco;
                if (newspeed > maxspeed*1000*frictionco){
                    newspeed = maxspeed*1000*frictionco;
                }

                momentum[0] = momentum[0]*newspeed/speed;
                momentum[1] = momentum[1]*newspeed/speed;
                momentum[2] = momentum[2]*newspeed/speed;
            }
            else{
                double newspeed = speed + (speed*speed+1)*frictionco;
                if (newspeed < revspeed*1000*frictionco){
                    newspeed = revspeed*1000*frictionco;
                }

                momentum[0] = momentum[0]*newspeed/speed;
                momentum[1] = momentum[1]*newspeed/speed;
                momentum[2] = momentum[2]*newspeed/speed;
            }
        }


        void correctcarobject(){

            double flatpointlength = sqrt(playercar.pointing[0]*playercar.pointing[0]+
                                        playercar.pointing[2]*playercar.pointing[2]);// this is the length of the flatened to xz plane pointing so can scale up
            double currentpointrot[3] = {-playercar.pointing[2]/flatpointlength,0/*playercar.pointing[1] for reasons of flatness*/
                                        ,playercar.pointing[0]/flatpointlength}; // this is the playercar.pointing rotated pi/2
            double rotpointdotwanted = 0;
            for (int i = 0; i < 3; i++){// is this rotated pointing forward or backward equivilent to is currentpointing left or right
                rotpointdotwanted = rotpointdotwanted + currentpointrot[i]*pointing[i];
            }
            double currentpointdotwanted = 0;// is this so i can see if it is totaly backwards for my checks
            currentpointdotwanted = currentpointdotwanted + playercar.pointing[0]*pointing[0];
            currentpointdotwanted = currentpointdotwanted + playercar.pointing[2]*pointing[2];
            

            double changex=0;
            if (abs(rotpointdotwanted) < 0.1 && currentpointdotwanted > 0){/*if very low then it is close enough not bother and if not oposite direction*/}
            else if (rotpointdotwanted > 0) {
                changex = 0.025;
            } else if (rotpointdotwanted < 0) {
                changex = -0.025;
            }




            //std::cout << currentx << " " << currenty << " current angles\n";
            //std::cout << newx << " " << newy << " new angles\n";
            //std::cout << changex << " " << changey << " " << changez << " change angles\n";

            std::cout << pointing[0] << " " << pointing[1] << " " << pointing[2] << " wanted pointing\n";
            std::cout << playercar.pointing[0] << " " << playercar.pointing[1] << " " << playercar.pointing[2] << " start pointing\n";

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

                std::cout << changex << " yaw\n";
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
                
                std::cout << -changey << " pitch\n";

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

            std::cout << "roll cross " << upcrosspointing[0] << " " << upcrosspointing[1] << " " << upcrosspointing[2] << "\n";
            std::cout << "roll " << nowdotcross << "\n";

            if (changez != 0){// roll
                double rotvect[3];
                double xzpointlen = sqrt(playercar.pointing[0]*playercar.pointing[0]+playercar.pointing[2]*playercar.pointing[2]);
                // useing angles is inacurate as angle is perfect but the actual direction is a bit off
                rotvect[0] = playercar.pointing[0]/xzpointlen;
                rotvect[1] = 0;
                rotvect[2] = playercar.pointing[2]/xzpointlen;
                playercar.selfrotate(rotvect,changez);

                std::cout << changez << " roll\n";
            }
            
            
            std::cout << playercar.pointing[0] << " " << playercar.pointing[1] << " " << playercar.pointing[2] << " end pointing\n";

            if (fabs(playercar.pointing[0] - pointing[0]) > 0.0005) {
                std::cout << "oops\n";
            }

            double enddot = 0;
            for (int i = 0; i < 3;i++){
                enddot = enddot + pointing[i]*playercar.pointing[i];
            }

            std::cout << startdot << " " << enddot << " dots\n";
            std::cout << enddot-startdot << " dots diff\n";

            double newx = atan2(playercar.pointing[0],playercar.pointing[2]);
            double newy = atan2(playercar.pointing[1],playercar.pointing[2]);

            std::cout << newx << " " << newy << " final angles\n";

        }


        void slopeaccelerate(){

            double slope = 0;
            double sideslope = 0;

            if (currenttrackid != -1){
                if (maintrack[currenttrackid].isontrack(selfpos)){
                    double height = maintrack[currenttrackid].trackheight(selfpos);
                    selfpos[1] = height;

                    double differentpos[3] = {selfpos[0]+pointing[0],selfpos[1],selfpos[2]+pointing[2]};
                    double differentheight = maintrack[currenttrackid].trackheight(differentpos);

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
                    double parallelheight = maintrack[currenttrackid].trackheight(parallelpos);

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
                if (maintrack[i].isontrack(selfpos)){
                    if (abs(maintrack[i].trackheight(selfpos)-selfpos[1]) < minchange){
                        minchange = abs(maintrack[i].trackheight(selfpos)-selfpos[1]);
                        currenttrackid = maintrack[i].roadid;
                    }
                }
            }
        }

        void checkpointcheck(){
            if (currenttrackid != -1){
                int checkpointid = maintrack[currenttrackid].isincheckpoint(selfpos);
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

                    if (checkpointid == 0 && checkpointspassed.size() == totalnumcheckpoints){
                        checkpointspassed.clear();

                        timeval currenttime;
                        gettimeofday(&currenttime,NULL);

                        long long newtime = currenttime.tv_sec*1000000 + currenttime.tv_usec;

                        std::cout << "checkpoint difference " << newtime-timeatcheckpoint << "\n";

                        laptimes.push_back(newtime-timeatcheckpoint);

                        timeatcheckpoint = newtime;

                        long long currentmintime = laptimes[0];
                        long long totaltime = laptimes[0];
                        for (int i = 1; i < laptimes.size(); i++){
                            if (laptimes[i] < currentmintime){
                                currentmintime = laptimes[i];
                            }
                            totaltime = totaltime + laptimes[i];

                        }

                        std::cout << "checkpoint besttime " << currentmintime << "\n";
                        std::cout << "checkpoint totaltime " << totaltime << "\n";

                    }
                }
            }
        }



        void update(){

            getcurrenttrackid();

            slopeaccelerate();

            checkpointcheck();


            speedcheck();

            premove(momentum[0],momentum[1],momentum[2]); // keep momentum


            //relmove(0,0,carspeed);
            move();

            rotate(0,0);

            correctcarobject();

            
            
        }

        void selfrender(){
            playercar.objectrender();

        }



};

player mainplayer;


void renderground(){

    int horizon = dispdist*tan(maincamera.yangle)+dispheight/2; // height not matter as far enough away not matter
    bool iscarontrack = false;
    bool carincheckpoint = false;
    for (int i = 0; i < maintrack.size(); i++){
        if (maintrack[i].isontrack(mainplayer.selfpos)){
            iscarontrack = true;
        }
        if (maintrack[i].isincheckpoint(mainplayer.selfpos) != -1){
            carincheckpoint = true;
        }
    }
    if (carincheckpoint){
        SDL_SetRenderDrawColor( renderer, 0x00, 0x00, 0xff, 0xFF );
    }
    else if (iscarontrack){
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


void gettrack(int num){

    maintrack.clear();
    hyperpoint startpoint;
    hyperpoint midpoint;
    hyperpoint midpoint2;
    hyperpoint endpoint;
    std::vector<hyperpoint> roadpoints;

    if (num == -1){// start screen
        mainplayer.selfpos[0] = 0;
        mainplayer.selfpos[2] = -1536;
        mainplayer.rotate(0,0);
        mainplayer.totalnumcheckpoints = 1;

        double roadlength = 4096;
        double roadspacing = 64;
        double roadwidth = 256;
        double roadid = 0;

        startpoint.pos[0] = 0;
        startpoint.pos[1] = 0;
        startpoint.pos[2] = -2048;
        startpoint.pos[3] = 0;

        endpoint.pos[0] = 0;
        endpoint.pos[1] = 0;
        endpoint.pos[2] = 2048;
        endpoint.pos[3] = 0;

        roadpoints.clear();
        roadpoints.push_back(startpoint);
        roadpoints.push_back(endpoint);

        road roadsegment0(roadlength, roadspacing, roadwidth, roadpoints, roadid);
        maintrack.push_back(roadsegment0);
    }


    if (num == 0){

        mainplayer.selfpos[0] = 0;
        mainplayer.selfpos[2] = -2048;
        mainplayer.rotate(M_PI,0);
        mainplayer.totalnumcheckpoints = 9;

        //curve 1
        
        double roadlength = 2048;
        double roadspacing = 64;
        double roadwidth = 256;
        double roadid = 0;
        
        startpoint.pos[0] = 0;
        startpoint.pos[1] = 0;
        startpoint.pos[2] = -2048;
        startpoint.pos[3] = 0;
        
        midpoint.pos[0] = 0;
        midpoint.pos[1] = 0;
        midpoint.pos[2] = -4096;
        midpoint.pos[3] = 0;
        
        endpoint.pos[0] = 2048;
        endpoint.pos[1] = 0;
        endpoint.pos[2] = -4096;
        endpoint.pos[3] = 0;

        roadpoints.clear();
        roadpoints.push_back(startpoint);
        roadpoints.push_back(midpoint);
        roadpoints.push_back(endpoint);

        road roadsegment0(roadlength, roadspacing, roadwidth, roadpoints, roadid);
        maintrack.push_back(roadsegment0);


        //curve 2
        
        roadlength = 2048;
        roadspacing = 64;
        roadwidth = 256;
        roadid = 1;
        
        startpoint.pos[0] = 2048;
        startpoint.pos[1] = 0;
        startpoint.pos[2] = -4096;
        startpoint.pos[3] = 0;
        
        midpoint.pos[0] = 4096;
        midpoint.pos[1] = 0;
        midpoint.pos[2] = -4096;
        midpoint.pos[3] = 0;
        
        endpoint.pos[0] = 4096;
        endpoint.pos[1] = 0;
        endpoint.pos[2] = -2048;
        endpoint.pos[3] = 0;

        roadpoints.clear();
        roadpoints.push_back(startpoint);
        roadpoints.push_back(midpoint);
        roadpoints.push_back(endpoint);

        road roadsegment1(roadlength, roadspacing, roadwidth, roadpoints, roadid);
        maintrack.push_back(roadsegment1);


        //curve 3
        
        roadlength = 2048;
        roadspacing = 64;
        roadwidth = 256;
        roadid = 2;
        
        startpoint.pos[0] = 4096;
        startpoint.pos[1] = 0;
        startpoint.pos[2] = -2048;
        startpoint.pos[3] = 0;
        
        midpoint.pos[0] = 4096;
        midpoint.pos[1] = 0;
        midpoint.pos[2] = 0;
        midpoint.pos[3] = 0;
        
        endpoint.pos[0] = 2048;
        endpoint.pos[1] = 0;
        endpoint.pos[2] = 0;
        endpoint.pos[3] = 0;

        roadpoints.clear();
        roadpoints.push_back(startpoint);
        roadpoints.push_back(midpoint);
        roadpoints.push_back(endpoint);

        road roadsegment2(roadlength, roadspacing, roadwidth, roadpoints, roadid);
        maintrack.push_back(roadsegment2);


        //stright 1
        
        roadlength = 2048;
        roadspacing = 64;
        roadwidth = 256;
        roadid = 3;
        
        startpoint.pos[0] = 2048;
        startpoint.pos[1] = 0;
        startpoint.pos[2] = 0;
        startpoint.pos[3] = 0;
        
        midpoint.pos[0] = 1024;
        midpoint.pos[1] = 0;
        midpoint.pos[2] = 0;
        midpoint.pos[3] = 0;

        midpoint2.pos[0] = 1024;
        midpoint2.pos[1] = -256;
        midpoint2.pos[2] = 0;
        midpoint2.pos[3] = 0;
        
        endpoint.pos[0] = 0;
        endpoint.pos[1] = -256;
        endpoint.pos[2] = 0;
        endpoint.pos[3] = 0;

        roadpoints.clear();
        roadpoints.push_back(startpoint);
        roadpoints.push_back(midpoint);
        roadpoints.push_back(midpoint2);
        roadpoints.push_back(endpoint);

        road roadsegment3(roadlength, roadspacing, roadwidth, roadpoints, roadid);
        maintrack.push_back(roadsegment3);


        //stright 2
        
        roadlength = 2048;
        roadspacing = 64;
        roadwidth = 256;
        roadid = 4;
        
        startpoint.pos[0] = 0;
        startpoint.pos[1] = -256;
        startpoint.pos[2] = 0;
        startpoint.pos[3] = 0;
        
        midpoint.pos[0] = -1024;
        midpoint.pos[1] = -256;
        midpoint.pos[2] = 0;
        midpoint.pos[3] = 0;

        midpoint2.pos[0] = -1024;
        midpoint2.pos[1] = 0;
        midpoint2.pos[2] = 0;
        midpoint2.pos[3] = 0;
        
        endpoint.pos[0] = -2048;
        endpoint.pos[1] = 0;
        endpoint.pos[2] = 0;
        endpoint.pos[3] = 0;

        roadpoints.clear();
        roadpoints.push_back(startpoint);
        roadpoints.push_back(midpoint);
        roadpoints.push_back(midpoint2);
        roadpoints.push_back(endpoint);

        road roadsegment4(roadlength, roadspacing, roadwidth, roadpoints, roadid);
        maintrack.push_back(roadsegment4);


        //curve 4
        
        roadlength = 2048;
        roadspacing = 64;
        roadwidth = 256;
        roadid = 5;
        
        startpoint.pos[0] = -2048;
        startpoint.pos[1] = 0;
        startpoint.pos[2] = 0;
        startpoint.pos[3] = 0;
        
        midpoint.pos[0] = -4096;
        midpoint.pos[1] = 0;
        midpoint.pos[2] = 0;
        midpoint.pos[3] = 0;
        
        endpoint.pos[0] = -4096;
        endpoint.pos[1] = 0;
        endpoint.pos[2] = 2048;
        endpoint.pos[3] = 0;

        roadpoints.clear();
        roadpoints.push_back(startpoint);
        roadpoints.push_back(midpoint);
        roadpoints.push_back(endpoint);

        road roadsegment5(roadlength, roadspacing, roadwidth, roadpoints, roadid);
        maintrack.push_back(roadsegment5);


        //curve 5
        
        roadlength = 2048;
        roadspacing = 64;
        roadwidth = 256;
        roadid = 6;
        
        startpoint.pos[0] = -4096;
        startpoint.pos[1] = 0;
        startpoint.pos[2] = 2048;
        startpoint.pos[3] = 0;
        
        midpoint.pos[0] = -4096;
        midpoint.pos[1] = 0;
        midpoint.pos[2] = 4096;
        midpoint.pos[3] = 0;
        
        endpoint.pos[0] = -2048;
        endpoint.pos[1] = 0;
        endpoint.pos[2] = 4096;
        endpoint.pos[3] = 0;

        roadpoints.clear();
        roadpoints.push_back(startpoint);
        roadpoints.push_back(midpoint);
        roadpoints.push_back(endpoint);

        road roadsegment6(roadlength, roadspacing, roadwidth, roadpoints, roadid);
        maintrack.push_back(roadsegment6);


        //curve 6
        
        roadlength = 2048;
        roadspacing = 64;
        roadwidth = 256;
        roadid = 7;
        
        startpoint.pos[0] = -2048;
        startpoint.pos[1] = 0;
        startpoint.pos[2] = 4096;
        startpoint.pos[3] = 0;
        
        midpoint.pos[0] = 0;
        midpoint.pos[1] = 0;
        midpoint.pos[2] = 4096;
        midpoint.pos[3] = 0;
        
        endpoint.pos[0] = 0;
        endpoint.pos[1] = 0;
        endpoint.pos[2] = 2048;
        endpoint.pos[3] = 0;

        roadpoints.clear();
        roadpoints.push_back(startpoint);
        roadpoints.push_back(midpoint);
        roadpoints.push_back(endpoint);

        road roadsegment7(roadlength, roadspacing, roadwidth, roadpoints, roadid);
        maintrack.push_back(roadsegment7);


        //stright 3
        
        roadlength = 2048;
        roadspacing = 64;
        roadwidth = 256;
        roadid = 8;
        
        startpoint.pos[0] = 0;
        startpoint.pos[1] = 0;
        startpoint.pos[2] = 2048;
        startpoint.pos[3] = 0;
        
        
        endpoint.pos[0] = 0;
        endpoint.pos[1] = 0;
        endpoint.pos[2] = -2048;
        endpoint.pos[3] = 0;

        roadpoints.clear();
        roadpoints.push_back(startpoint);
        roadpoints.push_back(endpoint);

        road roadsegment8(roadlength, roadspacing, roadwidth, roadpoints, roadid);
        maintrack.push_back(roadsegment8);
    }


        

    if (num == 1){
        mainplayer.selfpos[0] = 3072;
        mainplayer.totalnumcheckpoints = 6;

        //curve 1
        
        double roadlength = 2048;
        double roadspacing = 64;
        double roadwidth = 256;
        double roadid = 0;
        
        startpoint.pos[0] = 3072;
        startpoint.pos[1] = 0;
        startpoint.pos[2] = 0;
        startpoint.pos[3] = 0;
        
        midpoint.pos[0] = 3072;
        midpoint.pos[1] = 0;
        midpoint.pos[2] = 2048;
        midpoint.pos[3] = 0;
        
        endpoint.pos[0] = 1024;
        endpoint.pos[1] = 0;
        endpoint.pos[2] = 2048;
        endpoint.pos[3] = 0;

        roadpoints.clear();
        roadpoints.push_back(startpoint);
        roadpoints.push_back(midpoint);
        roadpoints.push_back(endpoint);

        road roadsegment0(roadlength, roadspacing, roadwidth, roadpoints, roadid);
        maintrack.push_back(roadsegment0);


        //stright 1

        roadlength = 2048;
        roadspacing = 64;
        roadwidth = 256;
        roadid = 1;
        
        startpoint.pos[0] = 1024;
        startpoint.pos[1] = 0;
        startpoint.pos[2] = 2048;
        startpoint.pos[3] = 0;
        
        midpoint.pos[0] = 0;
        midpoint.pos[1] = 0;
        midpoint.pos[2] = 2048;
        midpoint.pos[3] = 0;

        midpoint2.pos[0] = 0;
        midpoint2.pos[1] = -256;
        midpoint2.pos[2] = 2048;
        midpoint2.pos[3] = 0;
        
        endpoint.pos[0] = -1024;
        endpoint.pos[1] = -256;
        endpoint.pos[2] = 2048;
        endpoint.pos[3] = 0;

        roadpoints.clear();
        roadpoints.push_back(startpoint);
        roadpoints.push_back(midpoint);
        roadpoints.push_back(midpoint2);
        roadpoints.push_back(endpoint);

        road roadsegment1(roadlength, roadspacing, roadwidth, roadpoints, roadid);
        maintrack.push_back(roadsegment1);


        // curve 2
        
        roadlength = 2048;
        roadspacing = 64;
        roadwidth = 256;
        roadid = 2;
        
        startpoint.pos[0] = -1024;
        startpoint.pos[1] = -256;
        startpoint.pos[2] = 2048;
        startpoint.pos[3] = 0;
        
        midpoint.pos[0] = -3072;
        midpoint.pos[1] = -256;
        midpoint.pos[2] = 2048;
        midpoint.pos[3] = 0;
        
        endpoint.pos[0] = -3072;
        endpoint.pos[1] = -256;
        endpoint.pos[2] = 0;
        endpoint.pos[3] = 0;

        roadpoints.clear();
        roadpoints.push_back(startpoint);
        roadpoints.push_back(midpoint);
        roadpoints.push_back(endpoint);

        road roadsegment2(roadlength, roadspacing, roadwidth, roadpoints, roadid);
        maintrack.push_back(roadsegment2);


        // curve 3
        
        roadlength = 2048;
        roadspacing = 64;
        roadwidth = 256;
        roadid = 3;
        
        startpoint.pos[0] = -3072;
        startpoint.pos[1] = -256;
        startpoint.pos[2] = 0;
        startpoint.pos[3] = 0;
        
        midpoint.pos[0] = -3072;
        midpoint.pos[1] = -256;
        midpoint.pos[2] = -2048;
        midpoint.pos[3] = 0;
        
        endpoint.pos[0] = -1024;
        endpoint.pos[1] = -256;
        endpoint.pos[2] = -2048;
        endpoint.pos[3] = 0;

        roadpoints.clear();
        roadpoints.push_back(startpoint);
        roadpoints.push_back(midpoint);
        roadpoints.push_back(endpoint);

        road roadsegment3(roadlength, roadspacing, roadwidth, roadpoints, roadid);
        maintrack.push_back(roadsegment3);


        //stright 2

        roadlength = 2048;
        roadspacing = 64;
        roadwidth = 256;
        roadid = 4;
        
        startpoint.pos[0] = -1024;
        startpoint.pos[1] = -256;
        startpoint.pos[2] = -2048;
        startpoint.pos[3] = 0;
        
        midpoint.pos[0] = 0;
        midpoint.pos[1] = -256;
        midpoint.pos[2] = -2048;
        midpoint.pos[3] = 0;

        midpoint2.pos[0] = 0;
        midpoint2.pos[1] = 0;
        midpoint2.pos[2] = -2048;
        midpoint2.pos[3] = 0;
        
        endpoint.pos[0] = 1024;
        endpoint.pos[1] = 0;
        endpoint.pos[2] = -2048;
        endpoint.pos[3] = 0;

        roadpoints.clear();
        roadpoints.push_back(startpoint);
        roadpoints.push_back(midpoint);
        roadpoints.push_back(midpoint2);
        roadpoints.push_back(endpoint);

        road roadsegment4(roadlength, roadspacing, roadwidth, roadpoints, roadid);
        maintrack.push_back(roadsegment4);


        // curve 4
        
        roadlength = 2048;
        roadspacing = 64;
        roadwidth = 256;
        roadid = 5;
        
        startpoint.pos[0] = 1024;
        startpoint.pos[1] = 0;
        startpoint.pos[2] = -2048;
        startpoint.pos[3] = 0;
        
        midpoint.pos[0] = 3072;
        midpoint.pos[1] = 0;
        midpoint.pos[2] = -2048;
        midpoint.pos[3] = 0;
        
        endpoint.pos[0] = 3072;
        endpoint.pos[1] = 0;
        endpoint.pos[2] = 0;
        endpoint.pos[3] = 0;

        roadpoints.clear();
        roadpoints.push_back(startpoint);
        roadpoints.push_back(midpoint);
        roadpoints.push_back(endpoint);

        road roadsegment5(roadlength, roadspacing, roadwidth, roadpoints, roadid);
        maintrack.push_back(roadsegment5);
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

        mainwordwriter.makeletters();
        mainplayer.makecar();

        bool started = false;
        gettrack(-1);

        double mainmenucarspeed = 8;
        maincamera.rotate(-0.7 , -0.2);

        while (!started){// start menu stuff
            while ( SDL_PollEvent( &e ) != 0 ){ //basic event thing so can quit
                if( e.type == SDL_QUIT ){
                    started = true;
                    quit = true;
                }
                else if( e.type == SDL_KEYDOWN ){
                    if (e.key.keysym.sym == SDLK_ESCAPE){
                    started = true;
                    quit = true;
                    }
                }
            }

            // get key input area
            const Uint8* currentKeyStates = SDL_GetKeyboardState( NULL );

            if( currentKeyStates[ SDL_SCANCODE_SPACE ] ){
                std::cout << "START\n";
                started = true;
            }

            

            // all the main physics
            if (mainplayer.selfpos[2] > 0){
                mainplayer.selfpos[2] = -1536;
            }
            mainplayer.relmove(0 , 0 , mainmenucarspeed);
            mainplayer.move();

            SDL_SetRenderDrawColor( renderer, red, green, blue, 0xFF );
            SDL_FillRect( screenSurface, NULL, SDL_MapRGB (screenSurface->format,red,green,blue )); // set background


            SDL_RenderClear( renderer ); // start rendering objects


            renderground();


            mainplayer.selfrender();
            std::cout << "carpos " << mainplayer.selfpos[0] << " " << mainplayer.selfpos[1] << " " << mainplayer.selfpos[2] << "\n";
            std::cout << "carpos " << mainplayer.pointing[0] << " " << mainplayer.pointing[1] << " " << mainplayer.pointing[2] << "\n";

            std::string hellostring = "racecar";
            SDL_SetRenderDrawColor(renderer ,0x00 ,0x00 ,0x00 ,0xff);
            mainwordwriter.writechars(-dispwidth/2+32,128,64,hellostring,4);
            std::string numstring = "space to continue";
            mainwordwriter.writechars(-dispwidth/2+256 , -128 , 32, numstring,2);

            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            SDL_RenderPresent( renderer ); //update renderer ??
            SDL_Delay(1000/60);

        }



        gettrack(0);

        

        mainplayer.rotate(0,-0.3);


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
                mainplayer.sideaccelerate(-0.5);}
            if( currentKeyStates[ SDL_SCANCODE_D ] ){
                std::cout << "D";
                mainplayer.sideaccelerate(0.5);}
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

            std::string hellostring = "hello abcdefghijklmnopqrstuvwxyz";
            SDL_SetRenderDrawColor(renderer ,0x00 ,0x00 ,0x00 ,0xff);
            mainwordwriter.writechars(-dispwidth/2+32,0,32,hellostring,2);
            std::string numstring = "0123456789:.";
            mainwordwriter.writechars(-dispwidth/2+256 , -128 , 32, numstring,2);

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