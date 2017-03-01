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
#include <acq/icpNormal.h>
#include <acq/nearestNeighbour.h>

#include <ANN/ANN.h>

using namespace std;
using namespace Eigen;



// Function to do Point to plane ICP
// When we are moving the mesh P to align it with Q, this function uses normal at each point of Q in minimising the error between P and Q

vector<MatrixXd> icpNormal(vector<MatrixXd> vecV, MatrixXd norm, updateView uv){
    
    
    MatrixXd p;
    
    //Rotation and translation matrices
    MatrixXd R(3,3);
    MatrixXd t(3,1);
    
    
    int nPoints;
    int numP =vecV[0].rows();
    int numQ =vecV[1].rows();
    double errNew=0;
    double err=1;
    map<int,int> nn;
    if(numP>numQ){
        nPoints = numQ;
    }else{
        nPoints = numP;
    }
    MatrixXd q(numP,3);
    MatrixXd qN(numP,3);
    cout<<norm;
    
    // Normals at each vertex of Q
    MatrixXd normQ = norm.normalized();
    
    // For minimising the error between P and Q, we use the normal at Q.
    //To get the optimum R and T , we solve the eqn
    //Ax=b
    //Where each row of A is  [cross(P,N) N] where each cross(P,N) and N is a 1x3 vector making each row of A 1x6
    // So A is numPx 6 dimension
    
    MatrixXd A(numP,6);
    
    //each row of b is dot(-(p-q), N) which is a scalar. So b is numPx1
    MatrixXd b(numP,1);
    
    //X is the vector of [ alpha beta gamma tx ty tx].transpose
    // alpha , beta and gamma are rotations in radians along X , Y ans Z axis
    MatrixXd x(6,1);
    
    int iter=0;
    double thresh= 0.00000001;
    while(abs(err-errNew)>thresh){
        iter=iter+1;
        cout<<"Iteration "<<iter;
        cout<<" Error delta="<<(err-errNew)<<endl;
        
        err= errNew;
        p  = vecV[0];
        Vector3d tempP,tempQ,tempN,tempCross;
        nn = findClosestPoints(vecV[0],vecV[1]);
        for (int i=0;i<numP;i++)
        {
            // We form matrix q and matrix of normal of q i.e qN according to corresponce between P and Q
            q.row(i)=vecV[1].row(nn.at(i));
            qN.row(i)= normQ.row(nn.at(i));
            
            
            //We calculate A and b according to logic explained above
            A.row(i) << ( ( (Vector3d)p.row(i) ).cross( (Vector3d)qN.row(i) ) ).transpose(), qN.row(i);
            b.row(i) << ( -p.row(i) + q.row(i) ).dot( qN.row(i) );
        }
        
        // Since above SLE is non- linear, we assume that theta angle between line joing pi and qi is small approx 0.
        // So we solve by minimising least squares
        // x= pseudoinv(A) * b
        //pseudoinverse(A) = inverse(A.tran*A) * A.trans
        x= ( ( A.transpose() * A ).inverse() ) *( A.transpose() ) * b;
        
        //We get rotation and translation params from x
        double alpha    = x(0);
        double beta     = x(1);
        double gamma    = x(2);
        double tx       = x(3);
        double ty       = x(4);
        double tz       = x(5);
        
        //We contruct the rotation matrix using alpha , beta and gamma.
        // Note that thet are rotn in radians along X, Y and Z axis
        R =AngleAxisd(alpha, Vector3d(1,0,0)).toRotationMatrix()*
        AngleAxisd(beta, Vector3d(0,1,0)).toRotationMatrix()*
        AngleAxisd(gamma, Vector3d(0,0,1)).toRotationMatrix();
        
        //Construct translation matrix
        t<< tx,
            ty,
            tz;
        
        // Apply R and t to mesh P
        vecV[0]=(R*(vecV[0].transpose())+t.replicate(1,vecV[0].transpose().cols())).transpose();
        
        // calculate new error to see whethere the solution is converging or not. This is used at start of while loop to check convergence
        errNew=0;
        for (int i=0;i<nPoints;i++){
            errNew=errNew+ pow((vecV[0].row(i)-q.row(i)).dot(qN.row(i)),2);
        }
        cout<<"Error = "<<errNew<<endl;
        (*uv)(vecV);
    }
    //Solution is converged
    cout<<"Converged "<<", delta is ="<<abs(err-errNew)<<endl;
    
    return vecV;
}
