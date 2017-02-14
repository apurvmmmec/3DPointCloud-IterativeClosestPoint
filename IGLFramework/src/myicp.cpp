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
#include <cstdlib>						// C standard library
#include <cstdio>						// C I/O (for sscanf)
#include <cstring>						// string manipulation
#include <fstream>						// file I/O
#include <ANN/ANN.h>					// ANN declarations
#include<random>
#include<chrono>
using namespace std;
using namespace Eigen;




vector<MatrixXd> myFunc(vector<MatrixXd> vecV, updateView uv){
  
    
    
    //vecV[1] =vecV[1]*AngleAxisd(60/180.*3.14, Vector3d(0,1,0)).toRotationMatrix();
    
    MatrixXd p,qq, avgP, avgQ, pNorm, qNorm,C,R,t;
    int nPoints;
    int nPoints0 =vecV[0].rows();
    int nPoints1 =vecV[1].rows();
    double errNew=0;
    double err=1;
    map<int,int> nn;
    if(nPoints0>nPoints1){
        nPoints = nPoints1;
    }else{
        nPoints = nPoints0;
    }
    MatrixXd q(3,nPoints);
    

    unsigned seed = chrono::system_clock::now().time_since_epoch().count();
    default_random_engine generator (seed);
    float mean = 0.0;
    float sDev = 0.001;
    normal_distribution<double> distribution(mean,sDev);
    for(int i=0;i<vecV[1].rows();i++){
        for(int j=0;j<vecV[1].cols();j++){
            vecV[1].row(i)(j)= vecV[1].row(i)(j)+distribution(generator);
        }
    }
    int iter=0;
    double thresh= 1e-6;
    while(abs(err-errNew)>thresh){
        cout<<"Error Greater than "<<thresh<<", delta="<<(err-errNew)<<endl;
        err= (vecV[0].topRows(nPoints)-vecV[1].topRows(nPoints)).squaredNorm();;
        cout<<err<<endl;
        iter=iter+1;
        cout<<iter<<endl;
        
        p  = vecV[0].topRows(nPoints).transpose();
        qq = vecV[1].topRows(nPoints).transpose();
        
        nn = findClosestPoints(p,qq,nPoints );
        for (int i=0;i<nPoints;i++)
        {
            q.col(i)=qq.col(nn.at(i));
        }
        
        avgP = p.rowwise().mean();
        avgQ = q.rowwise().mean();
        
        pNorm = p-avgP.replicate(1,p.cols());
        qNorm = q-avgQ.replicate(1,q.cols());
        
        C = pNorm*qNorm.transpose();
        JacobiSVD<MatrixXd> svd(C,ComputeThinU | ComputeThinV);
        R= svd.matrixV()*svd.matrixU().transpose();
        t= avgQ -R*avgP;
        
        vecV[0]=(R*(vecV[0].transpose())+t.replicate(1,vecV[0].transpose().cols())).transpose();
        errNew = (vecV[0].topRows(nPoints)-vecV[1].topRows(nPoints)).squaredNorm();;
        cout<<errNew<<endl;
        
        
        (*uv)(vecV);
    }
    cout<<"Converged "<<", delta is ="<<(err-errNew)<<endl;
    
    return vecV;
}

map<int,int> findClosestPoints(MatrixXd vp, MatrixXd vq, int vmaxPts){
    
    int					nPts;					// actual number of data points
    ANNpointArray		dataPts;				// data points
    ANNpoint			queryPt;				// query point
    ANNidxArray			nnIdx;					// near neighbor indices
    ANNdistArray		dists;					// near neighbor distances
    ANNkd_tree*			kdTree;					// search structure
    int maxPts= vmaxPts;
    MatrixXd p = vp;
    MatrixXd q = vq;
    int				k				= 1;			// number of nearest neighbors
    int				dim				= 3;			// dimension
    double			eps				= 0;			// error bound
    
    queryPt = annAllocPt(dim);					// allocate query point
    dataPts = annAllocPts(maxPts, dim);			// allocate data points
    nnIdx = new ANNidx[k];						// allocate near neigh indices
    dists = new ANNdist[k];						// allocate near neighbor dists
    map<int,int> nearNb;
    
    nPts = 0;									// read data points
    
    for (int r = 0 ; r< maxPts;r++){
        dataPts[r][0] = q.col(r)(0);
        dataPts[r][1] = q.col(r)(1);
        dataPts[r][2] = q.col(r)(2);
    }
    
    
    kdTree = new ANNkd_tree(dataPts,maxPts,dim);
    
    for (int i=0;i<maxPts;i++){
        queryPt[0] = p.col(i)(0);queryPt[1] = p.col(i)(1);queryPt[2] = p.col(i)(2);
        
        kdTree->annkSearch(						// search
                           queryPt,						// query point
                           k,								// number of near neighbors
                           nnIdx,							// nearest neighbors (returned)
                           dists,							// distance (returned)
                           eps);							// error bound
        
        //cout << "\tNN:\tIndex\tDistance\n";
        //        cout << "\t" << i;
        //        for (int j = 0; j < k; j++) {			// print summary
        //            dists[j] = sqrt(dists[j]);			// unsquare distance
        //            cout << "\t" << nnIdx[j] ;
        //        }
        //        cout<<endl;
        
        nearNb[i] = nnIdx[0];
        
    }
    delete [] nnIdx;							// clean things up
    delete [] dists;
    delete kdTree;
    delete dataPts[0];
    delete[] dataPts;
    delete queryPt;
    annClose();									// done with ANN
    
    
    
    return nearNb;
}


//ANNpointArray findClosestPoints(ANNpointArray vp, ANNpoint vq, int vmaxPts){
//
//    ANNpointArray		dataPts;				// data points
//    ANNpointArray       dataPts1;
//    ANNpoint			queryPt;				// query point
//    ANNidxArray			nnIdx;					// near neighbor indices
//    ANNdistArray		dists;					// near neighbor distances
//    ANNkd_tree*			kdTree;					// search structure
//    int             maxPts          = vmaxPts;
//    int				k				= 1;			// number of nearest neighbors
//    int				dim				= 3;			// dimension
//    double			eps				= 0;			// error bound
//    nnIdx = new ANNidx[k];						// allocate near neigh indices
//    dists = new ANNdist[k];						// allocate near neighbor dists
//    dataPts=vp;
//    queryPt=vq;
//
//    //    cout<<"in func dataPts"<<endl;
//    //    for(int i=0;i<maxPts;i++){
//    //        cout<<dataPts[i][0]<<" ";
//    //    }
//    //    cout<<endl;
//    //    cout<<"in func queryPt"<<endl;
//    //    cout<<queryPt[0]<<" ";
//    //    cout<<endl;
//
//    kdTree = new ANNkd_tree(dataPts,maxPts,dim);
//    kdTree->annkSearch(queryPt,k,nnIdx,dists,eps);
//
//    cout << "\tNN:\tIndex\tDistance\n";
//    cout << "\t" << 0;
//    for (int j = 0; j < k; j++) {			// print summary
//        dists[j] = sqrt(dists[j]);			// unsquare distance
//        cout << "\t" << nnIdx[j] <<"\t"<<dists[j];
//    }
//    cout<<endl;
//    cout<<"Query"<<endl;
//    cout<<queryPt[0]<<" "<<queryPt[1]<<" "<<queryPt[2];
//    cout<<endl;
//    cout<<dataPts[nnIdx[0]][0]<<" "<<dataPts[nnIdx[0]][1]<<" "<<dataPts[nnIdx[0]][2];
//    cout<<endl;
//
//    dataPts1 = annAllocPts(maxPts-1,3);
//    int c=0;
//    for (int r = 0 ; r< maxPts;r++){
//        if(r!= nnIdx[0]){
//            dataPts1[c][0] = dataPts[r][0];
//            dataPts1[c][1] = dataPts[r][1];
//            dataPts1[c][2] = dataPts[r][2];
//            c++;
//        }
//    }
//
//    delete [] nnIdx;							// clean things up
//    delete [] dists;
//    delete kdTree;
//    annClose();									// done with ANN
//
//    return dataPts1;
//}

//    Eigen::MatrixXd a(2,3);
//    a<<1,2,3,4,5,6;
//    cout<<a<<endl;
//    avgP= a.rowwise().mean();
//    cout<<avgP<<endl;

//        Eigen::MatrixXd b(3,1);
//        b<<3,4,5;
//        cout<<b<<endl;
//        Eigen::MatrixXd c =a-b.replicate(1,a.cols());
//        cout<<c<<endl;
//        cout<<c.transpose()<<endl;
//avgP(0,0) = p.row(0).mean();
//avgP(1,0) = p.row(1).mean();
//avgP(2,0) = p.row(2).mean();


//    MatrixXd a(4,3);
//    a <<    1, 2, 3,
//    -4, 5, 6,
//    -1,-1, 0,
//    1,-1, 8;
//    MatrixXd b(4,3);
//    b <<    1, 1, 1,
//    -1, 1, 1,
//    -1,-1, 10,
//    1,-1, 10;
//
//    cout<<"a"<<endl<<a.col(2)(3)<<endl;
//    cout<<"b"<<endl<<b<<endl;
//    int maxPts=4;

//    map<int,int> nn = findClosestPoints(a,b,maxPts );
//    MatrixXd qq(maxPts,3);
//    for (int i=0;i<maxPts;i++)
//    {
//        int corrsp= nn.at(i);
//        cout<<corrsp<<endl;
//        qq.row(i)=b.row(corrsp);
//    }
//    cout<<qq;




//    MatrixXd c= b-a;
//    MatrixXd d = c.rowwise().squaredNorm();
//    MatrixXd e = d.colwise().sum();
//    cout<<d<<endl;
//    cout<<e<<endl;
//
//

//        MatrixXd a(4,3);
//        a <<    1, 2, 3,
//        -4, 5, 6,
//        -1,-1, 0,
//        1,-1, 8;
//
//        MatrixXd b(4,3);
//        b <<    1, 1, 1,
//        -1, 1, 1,
//        -1,-1, 10,
//        1,-1, 10;

// cout<<"a"<<endl<<a<<endl;

// construct a trivial random generator engine from a time-based seed:
// cout<<"a"<<endl<<a<<endl;


//    Vector3d m = vecV[1].colwise().minCoeff();
//    Vector3d M = vecV[1].colwise().maxCoeff();
