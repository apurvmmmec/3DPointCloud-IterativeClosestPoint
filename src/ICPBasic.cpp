//
//  myicp.cpp
//  IGLFramework
//
//  Created by Apurv Nigam on 10/02/2017.
//
//

#include <stdio.h>
#include <iostream>
#include <igl/read_triangle_mesh.h>
#include <igl/point_mesh_squared_distance.h>
#include <igl/combine.h>
#include <Eigen/Geometry>
#include <thread>
#include <acq/myicp.h>
#include <ANN/ANN.h>
#include<random>
#include<chrono>
#include <acq/nearestNeighbour.h>

using namespace std;
using namespace Eigen;

/*
 Code to do basic Point to Point ICP
 This code is as a part of solution to Q1 to Q3 of the coursework
 
 We have to align 2 meshes which we call P and Q in this code.
 Steps
 Our main objective is to minimise the error between P and Q
 We first of all find the corresponding points between the mesh P and Q using the logic of nearest neighbourhood.
 I used ANN library to find, for every point in P , the corresponding point in Q which is nearest to P.
 We then calculate the average of all points in P and Q and then subtract the avg from all points.
 Then we calculate matrix C 
 Then do SVD of C to find U and V
 Then optimal translation and rotation are then calculated.
 We apply the transformation on all points in P. This step makes P more aligned to Q than previous.
 We now keep on repeating the above steps till the point when the error stops changing. We then say that the solution is converged.
 
 
 */


vector<MatrixXd> icpPointToPoint(vector<MatrixXd> vecV, updateView uv, colorUnMatched pV){
  
    
    //Code to rotate the mesh M2 by some angle alpha along any axis
    /*
     [in] alpha in degree
     2nd parameter is Vector3d(1,0,0) which is Unit vector in X direction for rotating along X axis
     Possible values are Vector3d(0,1,0) for rotation along Y axis and Vector3d(0,0,1) for Z axis
     vecV[1] =vecV[1]*AngleAxisd(alpha/180.*3.14, Vector3d(1,0,0)).toRotationMatrix();
     */
    
    //Please uncomment this line to add roataion to Mesh M2
    //vecV[0] =vecV[0]*AngleAxisd(80/180.*3.14, Vector3d(0,1,0)).toRotationMatrix();
    
    
    //Matrices to hold the meshes and their transforms and intermediate calculations.
    MatrixXd p,avgP, avgQ, pNorm, qNorm,C,R,t;
    
    //Find number of points in P and Q
    int nPoints;
    int numP =vecV[0].rows();
    int numQ =vecV[1].rows();
    double errNew=0;
    double err=1;
    
    //Map data structure to store the nearest neigbour correspondance between P and Q
    map<int,int> nn;
    
    if(numP>numQ){
        nPoints = numQ;
    }else{
        nPoints = numP;
    }
    MatrixXd q(3,numP);
    

    
    /*
     
     Please uncomment this code for addition of noise
     Code for adding Noise to 2nd mesh
     */
//    Vector3d min = vecV[1].colwise().minCoeff();
//    Vector3d max = vecV[1].colwise().maxCoeff();
//    double bbDLength = pow((max-min).squaredNorm(),0.5);
//    cout<<"Length of Diagonal of Bounding Box is "<<bbDLength<<endl;
//    double percentNoise = 3;
//    unsigned seed = chrono::system_clock::now().time_since_epoch().count();
//    default_random_engine generator (seed);
//    float mean = 0.0;
//    float sDev = bbDLength*percentNoise/100;
//    normal_distribution<double> distribution(mean,sDev);
//    for(int i=0;i<vecV[1].rows();i++){
//        for(int j=0;j<vecV[1].cols();j++){
//            vecV[1].row(i)(j)= vecV[1].row(i)(j)+distribution(generator);
//        }
//    }
    //End of adding noise
    
    //Iteration counter
    int iter=0;
    
    //The threshold value at which we consider convergance of solution
    double thresh= 1e-6;
    
    // We perform this loop till the solution converges. In each iteration of this loop, we try to minimise the distance between P and Q or the error between P an Q.
    while(abs(err-errNew)>thresh){
        iter=iter+1;
        cout<<"Iteration "<<iter;
        cout<<" Error delta="<<(err-errNew)<<endl;

        err= errNew;
        p  = vecV[0].transpose();
        
        //Find the correspondance between P and Q. Code for this is nearestNeighbour.cpp
        nn = findClosestPoints(vecV[0],vecV[1]);
        for (int i=0;i<numP;i++)
        {
            //Form the matrix Q using the correspondance info
            q.col(i)=(vecV[1].transpose()).col(nn.at(i));
        }
        
        //Calculate Average of AP and Q
        avgP = p.rowwise().mean();
        avgQ = q.rowwise().mean();
       
        //Subtract average from each point
        pNorm = p-avgP.replicate(1,p.cols());
        qNorm = q-avgQ.replicate(1,q.cols());
        
        //Calculate C and do its Singular Value Decomposition
        C = pNorm*qNorm.transpose();
        JacobiSVD<MatrixXd> svd(C,ComputeThinU | ComputeThinV);
        
        //Calculate rotation
        R= svd.matrixV()*svd.matrixU().transpose();
        
        //Calculate Translation
        t= avgQ -R*avgP;
        
        // Transform all point of mesh P by R and t
        vecV[0]=(R*(vecV[0].transpose())+t.replicate(1,vecV[0].transpose().cols())).transpose();
        
        //Calculate the new error
        errNew = (vecV[0].topRows(nPoints)-(q.transpose()).topRows(nPoints)).squaredNorm();
        //errNew = (p-q).squaredNorm();;

        cout<<"Error = "<<errNew<<endl;
        
        
        (*uv)(vecV);
    }
    //The solution is converged
    cout<<"Converged "<<", delta is ="<<abs(err-errNew)<<endl;
    
    
    
    /*Code to color unmatched Pixels in Q
     
     At this stage, q contains points that were ,matching to p when solution converged.
     We remove these matching points from 2nd mesh to find points which did not find match in 1st mesh
     in the aligned state and color them
     */
    
    vector<int> temp;
    for(int i=0;i<numP;i++){
        //Retrive all matching vertices of Q and sav in a vector
        temp.push_back(nn.at(i));
    }
    
    //notMatched is the vector that contains indices of vertices of Q that are not matched in P
    vector<int> notMatched;
    for(int i=0;i<numQ;i++){
        if(std::find(temp.begin(), temp.end(), i) == temp.end()) {
            //temp not contains i
            notMatched.push_back(i);
        }
    }
    /*
     Call back function called from here which updates the color of meshes in viewer
    */
    
    //Please uncomment the below line to call this callback function
    //(*pV)(notMatched);
    
    
        
    
    return vecV;
}

